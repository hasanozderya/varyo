#include "config.h"
#if !VARIO_USE_WIFI
#include "ble_task.h"
#include "ble_protocol.h"
#include "xctrack_protocol.h"
#include "ble_commands.h"
#include "shared_state.h"
#include "debug_log.h"
#include "boot_sync.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLESecurity.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <atomic>
#include <string.h>

namespace {
    struct Command { uint32_t generation, modeEpoch; char line[BleProtocol::MAX_COMMAND]; };
    QueueHandle_t commands = nullptr;
    BleProtocol::LineReceiver receiver;
    std::atomic<bool> connected{false};
    std::atomic<bool> replyEnabled{false}, telemetryEnabled{false};
    std::atomic<bool> nmeaEnabled{false};
    std::atomic<uint16_t> txSubscription{0};
    std::atomic<uint32_t> txEpoch{0};
    uint32_t receiverEpoch = 0; // owned by the serialized BLE receive callback
    std::atomic<bool> receiveError{false};
    std::atomic<bool> restartAdvertising{false};
    std::atomic<uint32_t> generation{0};
    std::atomic<int> indicationStatus{-1};
    std::atomic<uint32_t> indicationCode{0};
    BLECharacteristic* tx = nullptr;
    BLECharacteristic* tele = nullptr;
    BLECharacteristic* gpsCharacteristic = nullptr;
    BLECharacteristic* healthCharacteristic = nullptr;
    std::atomic<bool> gpsEnabled{false}, healthEnabled{false};
#if defined(CONFIG_BLUEDROID_ENABLED)
    BLE2902* txCccd = nullptr;
    BLE2902* teleCccd = nullptr;
    BLE2902* gpsCccd = nullptr;
    BLE2902* healthCccd = nullptr;
#endif

    void setTxSubscription(uint16_t value) {
        const auto next = XcTrackProtocol::txMode(value);
        if (XcTrackProtocol::txMode(txSubscription.exchange(value)) != next) ++txEpoch;
        replyEnabled = next == XcTrackProtocol::TxMode::Json;
        nmeaEnabled = next == XcTrackProtocol::TxMode::Nmea;
    }

    bool notifyNmea(BLEServer* server, const char* data, size_t length) {
        // Send without the wrapper's possible notification->indication fallback.
        // No blocking ATT confirmation wait; retry on controller backpressure.
        const uint16_t connId = server->getConnId();
#if defined(CONFIG_NIMBLE_ENABLED)
        os_mbuf* packet = ble_hs_mbuf_from_flat(data, length);
        return packet && ble_gatts_notify_custom(connId, tx->getHandle(), packet) == 0;
#else
        return esp_ble_gatts_send_indicate(server->getGattsIf(), connId,
            tx->getHandle(), (uint16_t)length,
            (uint8_t*)data, false) == ESP_OK;
#endif
    }

    class ServerCallbacks : public BLEServerCallbacks {
        void onConnect(BLEServer*) override {
            receiver.reset();
            xQueueReset(commands);
            receiveError = false;
            setTxSubscription(0);
            telemetryEnabled = false;
            ++generation;
            connected = true;
            gpsEnabled = healthEnabled = false;
            SharedState::writeLink({true, false, false, 0, 0});
        }
        void onDisconnect(BLEServer*) override {
            connected = false;
            gpsEnabled = healthEnabled = false;
            SharedState::writeLink({});
            setTxSubscription(0);
            telemetryEnabled = false;
            receiver.reset();
            ++generation;
            restartAdvertising = true;
        }
    };
    class ReplyCallbacks : public BLECharacteristicCallbacks {
        void onStatus(BLECharacteristic*, Status status, uint32_t code) override {
#if defined(CONFIG_NIMBLE_ENABLED)
            // Notify completion must never acknowledge/fail a JSON indication.
            // Direct indication submission errors are handled by its caller.
            if (status != SUCCESS_INDICATE && status != ERROR_INDICATE_TIMEOUT &&
                status != ERROR_INDICATE_FAILURE && status != ERROR_INDICATE_DISABLED) return;
#endif
            indicationCode = code;
            indicationStatus = (int)status;
        }
#if defined(CONFIG_NIMBLE_ENABLED)
        void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t value) override {
            setTxSubscription(value);
        }
#endif
    };
    class TelemetryCallbacks : public BLECharacteristicCallbacks {
#if defined(CONFIG_NIMBLE_ENABLED)
        void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t value) override {
            telemetryEnabled = (value & 1) != 0;
        }
#endif
    };
    class OptionalCallbacks : public BLECharacteristicCallbacks {
    public:
        explicit OptionalCallbacks(std::atomic<bool>& enabled) : enabled_(enabled) {}
    private:
        std::atomic<bool>& enabled_;
#if defined(CONFIG_NIMBLE_ENABLED)
        void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t value) override {
            enabled_ = (value & 1) != 0;
        }
#endif
    };
    class SecurityCallbacks : public BLESecurityCallbacks {
        bool onSecurityRequest() override { return true; }
        void onPassKeyNotify(uint32_t key) override {
            SharedState::writeLink({true, false, true, key, millis()});
        }
#if defined(CONFIG_NIMBLE_ENABLED)
        void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
            const bool ok = desc && desc->sec_state.encrypted && desc->sec_state.authenticated;
            SharedState::writeLink({true, ok, false, 0, 0});
        }
#else
        void onAuthenticationComplete(esp_ble_auth_cmpl_t desc) override {
            SharedState::writeLink({true, desc.success, false, 0, 0});
        }
#endif
    };
    class RxCallbacks : public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic* characteristic) override {
            if (!connected || !replyEnabled) return;
            const uint32_t epoch = txEpoch;
            if (receiverEpoch != epoch) { receiver.reset(); receiverEpoch = epoch; }
            const String value = characteristic->getValue();
            for (size_t i = 0; i < value.length(); ++i) {
                const auto result = receiver.feed((uint8_t)value[i]);
                if (result == BleProtocol::LineReceiver::Invalid) receiveError = true;
                else if (result == BleProtocol::LineReceiver::Complete) {
                    // Store only complete lines. Never execute commands, write
                    // flash, or block for transmission in the BLE host callback.
                    Command command{};
                    command.generation = generation;
                    command.modeEpoch = epoch;
                    memcpy(command.line, receiver.line(), strlen(receiver.line()) + 1);
                    if (!replyEnabled || xQueueSend(commands, &command, 0) != pdTRUE)
                        receiveError = true;
                }
            }
        }
    };
}

void bleTaskFunc(void*) {
    commands = xQueueCreate(4, sizeof(Command));
    configASSERT(commands != nullptr);
    BootSync::waitForSensorsReady(8000);
    BLEDevice::init("Vario-BLE");
    BLEDevice::setMTU(185);
    static SecurityCallbacks securityCallbacks;
    BLEDevice::setSecurityCallbacks(&securityCallbacks);
    BLESecurity::setAuthenticationMode(true, true, true);
    BLESecurity::setCapability(ESP_IO_CAP_OUT);
    BLESecurity::setPassKey(false);
    BLESecurity::regenPassKeyOnConnect(true);
    // Let Android initiate bonding; do not block the BLE host on a dialog.
    BLESecurity::setForceAuthentication(false);
    BLEServer* server = BLEDevice::createServer();
    static ServerCallbacks serverCallbacks;
    static RxCallbacks rxCallbacks;
    static ReplyCallbacks replyCallbacks;
    static TelemetryCallbacks telemetryCallbacks;
    server->setCallbacks(&serverCallbacks);
    BLEService* service = server->createService(BleProtocol::SERVICE_UUID);
    tx = service->createCharacteristic(BleProtocol::TX_UUID,
        BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_NOTIFY);
    tx->setCallbacks(&replyCallbacks);
    BLECharacteristic* rx = service->createCharacteristic(BleProtocol::RX_UUID, BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_ENC | BLECharacteristic::PROPERTY_WRITE_AUTHEN);
#if defined(CONFIG_BLUEDROID_ENABLED)
    rx->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
#endif
    rx->setCallbacks(&rxCallbacks);
    tele = service->createCharacteristic(BleProtocol::TELEMETRY_UUID,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    tele->setCallbacks(&telemetryCallbacks);
    static OptionalCallbacks gpsCallbacks(gpsEnabled), healthCallbacks(healthEnabled);
    gpsCharacteristic = service->createCharacteristic(BleProtocol::GPS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    healthCharacteristic = service->createCharacteristic(BleProtocol::HEALTH_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    gpsCharacteristic->setCallbacks(&gpsCallbacks);
    healthCharacteristic->setCallbacks(&healthCallbacks);
    uint8_t initialHealth[20];
    BleProtocol::encodeHealth(SharedState::readFresh(millis()), millis(), initialHealth);
    healthCharacteristic->setValue(initialHealth, sizeof(initialHealth));
#if defined(CONFIG_BLUEDROID_ENABLED)
    txCccd = new BLE2902();
    teleCccd = new BLE2902();
    tx->addDescriptor(txCccd);
    tele->addDescriptor(teleCccd);
    gpsCccd = new BLE2902(); healthCccd = new BLE2902();
    gpsCharacteristic->addDescriptor(gpsCccd);
    healthCharacteristic->addDescriptor(healthCccd);
#endif
    service->start();
    BLEAdvertising* advertising = server->getAdvertising();
    advertising->addServiceUUID(BleProtocol::SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();
    DebugLog::log("BLE hazir: Vario-BLE; XCTrack NMEA notify / uygulama JSON indicate\n");

    uint32_t session = generation;
    uint32_t lastTelemetryMs = 0, lastSerialMs = 0;
    uint16_t sequence = 0;
    String response;
    size_t sent = 0;
    size_t inFlightBytes = 0;
    uint32_t indicationStartedMs = 0, lastExtraMs = 0;
    uint32_t lastTxEpoch = txEpoch;
    XcTrackProtocol::Stream nmea;
    for (;;) {
        const bool online = connected;
        const uint32_t currentSession = generation;
        if (session != currentSession) {
            // Never deliver an old client's response/partial request to a new one.
            session = currentSession;
            response = "";
            sent = 0;
            inFlightBytes = 0;
            indicationStatus = -1;
            sequence = 0;
            nmea.reset();
        }
        if (!online && restartAdvertising.exchange(false)) advertising->start();
#if defined(CONFIG_BLUEDROID_ENABLED)
        setTxSubscription(online ?
            (txCccd->getNotifications() ? 1 : 0) | (txCccd->getIndications() ? 2 : 0) : 0);
        telemetryEnabled = online && teleCccd->getNotifications();
        gpsEnabled = online && gpsCccd->getNotifications();
        healthEnabled = online && healthCccd->getNotifications();
#endif
        const uint32_t currentTxEpoch = txEpoch;
        if (lastTxEpoch != currentTxEpoch) {
            lastTxEpoch = currentTxEpoch;
            nmea.reset(); response = ""; sent = inFlightBytes = 0;
            indicationStatus = -1;
        }
        if (online && replyEnabled && response.isEmpty()) {
            if (receiveError.exchange(false)) {
                response = "{\"id\":null,\"ok\":false,\"error\":\"rx_overflow_or_invalid_line\"}\n";
            } else {
                Command command;
                if (xQueueReceive(commands, &command, 0) == pdTRUE && command.generation == session &&
                    command.modeEpoch == currentTxEpoch) {
                    response = executeBleCommand(command.line);
                    response += '\n';
                }
            }
            sent = 0;
        }
        if (online && replyEnabled && !response.isEmpty() && generation == session && txEpoch == currentTxEpoch) {
            if (inFlightBytes == 0) {
                const uint16_t mtu = max((uint16_t)23, server->getPeerMTU(server->getConnId()));
                const size_t chunk = min((size_t)182, (size_t)(mtu-3));
                inFlightBytes = min(chunk, response.length()-sent);
                indicationStatus = -1;
                indicationCode = 0;
                indicationStartedMs = millis();
#if defined(CONFIG_NIMBLE_ENABLED)
                // This Arduino NimBLE wrapper reports the real ACK through
                // onStatus, but its blocking indicate() semaphore is never
                // released by BLEServer's NOTIFY_TX handler. Send directly and
                // consume that same callback here, without the false timeout.
                const uint16_t connId = server->getConnId();
                os_mbuf* packet = ble_hs_mbuf_from_flat(
                    response.c_str() + sent, inFlightBytes);
                int rc = 0;
                if (!packet) {
                    rc = BLE_HS_ENOMEM;
                } else {
                    // NimBLE consumes packet even when the call fails.
                    // inFlightBytes serializes sends without the wrapper's
                    // private indication-wait bookkeeping.
                    rc = ble_gatts_indicate_custom(connId, tx->getHandle(), packet);
                }
                if (rc != 0) {
                    indicationCode = rc;
                    indicationStatus = (int)BLECharacteristicCallbacks::ERROR_GATT;
                }
#else
                tx->setValue((const uint8_t*)response.c_str() + sent, inFlightBytes);
                tx->indicate();
#endif
            }
            const int result = indicationStatus;
            if (result == (int)BLECharacteristicCallbacks::SUCCESS_INDICATE) {
                sent += inFlightBytes;
                inFlightBytes = 0;
                if (sent == response.length()) response = "";
            } else if (result != -1 || (uint32_t)(millis() - indicationStartedMs) >= 5000) {
                DebugLog::logf("BLE reply failed: status=%d code=%lu offset=%u\n",
                               result, (unsigned long)indicationCode.load(), (unsigned)sent);
                // Never append another JSON reply to an incomplete response.
                response = "";
                inFlightBytes = 0;
                if (connected && generation == session) server->disconnect(server->getConnId());
            }
        }
        const uint32_t nowMs = millis();
        if (connected && nmeaEnabled && generation == session && txEpoch == currentTxEpoch) {
            nmea.prepare(nowMs, SharedState::readFresh(nowMs), SharedState::readGps(nowMs),
                XcTrackProtocol::GpsStore::read());
            const size_t chunk = nmea.chunkSize(server->getPeerMTU(server->getConnId()));
            if (chunk && notifyNmea(server, nmea.data(), chunk)) nmea.advance(chunk);
        }
        if ((uint32_t)(nowMs - lastTelemetryMs) >= 200) {
            lastTelemetryMs = nowMs;
            uint8_t packet[BleProtocol::TELEMETRY_BYTES];
            BleProtocol::encodeTelemetry(SharedState::readFresh(nowMs), sequence++, nowMs, packet);
            tele->setValue(packet, sizeof(packet));
            if (connected && telemetryEnabled) tele->notify();
        }
        if ((uint32_t)(nowMs-lastExtraMs) >= 500) {
            lastExtraMs = nowMs;
            uint8_t packet[20];
            BleProtocol::encodeGps(SharedState::readGps(nowMs), packet);
            gpsCharacteristic->setValue(packet, sizeof(packet));
            if (connected && gpsEnabled) gpsCharacteristic->notify();
            BleProtocol::encodeHealth(SharedState::readFresh(nowMs), nowMs, packet);
            healthCharacteristic->setValue(packet, sizeof(packet));
            if (connected && healthEnabled) healthCharacteristic->notify();
        }
        if ((uint32_t)(nowMs - lastSerialMs) >= 200 && Serial) {
            lastSerialMs = nowMs;
            const String chunk = DebugLog::pullNewForSerial();
            if (chunk.length() && (size_t)Serial.availableForWrite() >= chunk.length()) Serial.print(chunk);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif
