#include "xctrack_protocol.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {
    int hexDigit(char c) {
        if (c >= '0' && c <= '9') return c-'0';
        if (c >= 'A' && c <= 'F') return c-'A'+10;
        if (c >= 'a' && c <= 'f') return c-'a'+10;
        return -1;
    }
    portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;
    XcTrackProtocol::GpsFrames gpsFrames;
}

XcTrackProtocol::TxMode XcTrackProtocol::txMode(uint16_t cccd) {
    // Our Android client explicitly requests indications. XCTrack requests
    // Nordic UART notifications. Never interleave text with JSON replies.
    return (cccd & 2) ? TxMode::Json : (cccd & 1) ? TxMode::Nmea : TxMode::Off;
}

size_t XcTrackProtocol::sentence(const char* body, char* out, size_t capacity) {
    if (!body || !out || !capacity) return 0;
    out[0] = '\0';
    const size_t n = strlen(body);
    if (n > LINE_BYTES-7 || n+7 > capacity) return 0;
    uint8_t checksum = 0;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t c = (uint8_t)body[i];
        if (c < 32 || c > 126 || c == '$' || c == '*') return 0;
        checksum ^= c;
    }
    const int written = snprintf(out, capacity, "$%s*%02X\r\n", body, checksum);
    return written > 0 && (size_t)written < capacity ? (size_t)written : 0;
}

size_t XcTrackProtocol::encodeVario(const VarioState& s, uint32_t nowMs,
                                    char* out, size_t capacity) {
    const bool baro = s.sampleSequence && (uint32_t)(nowMs-s.updatedMs) < 500 &&
        s.baroOk && (uint32_t)(nowMs-s.baroSampleMs) < 500 &&
        isfinite(s.pressurePa) && s.pressurePa >= 20000 && s.pressurePa <= 120000;
    const bool vario = baro && s.outputReady && isfinite(s.climbRateMps) && fabsf(s.climbRateMps) < 90;
    char body[64];
    // Pressure: Pa. Altitude deliberately absent: XCTrack owns its QNH.
    // Temperature and device battery are unavailable, not plausible fake values.
    snprintf(body, sizeof(body), "LK8EX1,%ld,99999,%ld,99,999,",
        baro ? lroundf(s.pressurePa) : 999999L,
        vario ? lroundf(s.climbRateMps*100.0f) : 9999L);
    return sentence(body, out, capacity);
}

XcTrackProtocol::GpsLineReceiver::Result XcTrackProtocol::GpsLineReceiver::feed(uint8_t c) {
    if (c == '$') {
        used_ = 0; collecting_ = true;
    }
    if (!collecting_) return Pending;
    if (c == '\n') {
        collecting_ = false;
        if (used_ && line_[used_-1] == '\r') --used_;
        if (used_ < 10 || used_ > LINE_BYTES-3 || line_[used_-3] != '*') return Pending;
        const int hi = hexDigit(line_[used_-2]), lo = hexDigit(line_[used_-1]);
        if (hi < 0 || lo < 0) return Pending;
        uint8_t sum = 0;
        for (size_t i = 1; i < used_-3; ++i) {
            if (line_[i] < 32 || line_[i] > 126 || line_[i] == '*') return Pending;
            sum ^= (uint8_t)line_[i];
        }
        if (sum != ((hi<<4)|lo)) return Pending;
        const Result kind = !strncmp(line_, "$GPRMC,", 7) || !strncmp(line_, "$GNRMC,", 7) ? Rmc :
            !strncmp(line_, "$GPGGA,", 7) || !strncmp(line_, "$GNGGA,", 7) ? Gga : Pending;
        if (kind != Pending) {
            line_[used_++] = '\r'; line_[used_++] = '\n'; line_[used_] = '\0';
        }
        return kind;
    }
    if ((c < 32 && c != '\r') || c > 126 || used_ >= LINE_BYTES-2) {
        collecting_ = false; return Pending;
    }
    line_[used_++] = (char)c;
    return Pending;
}

void XcTrackProtocol::GpsStore::feed(uint8_t byte, uint32_t nowMs) {
    static GpsLineReceiver receiver;
    const auto kind = receiver.feed(byte);
    if (kind == GpsLineReceiver::Pending) return;
    const size_t n = strlen(receiver.line())+1;
    portENTER_CRITICAL(&gpsMux);
    if (kind == GpsLineReceiver::Rmc) {
        memcpy(gpsFrames.rmc, receiver.line(), n); gpsFrames.rmcMs = nowMs;
    } else {
        memcpy(gpsFrames.gga, receiver.line(), n); gpsFrames.ggaMs = nowMs;
    }
    portEXIT_CRITICAL(&gpsMux);
}

XcTrackProtocol::GpsFrames XcTrackProtocol::GpsStore::read() {
    portENTER_CRITICAL(&gpsMux);
    const GpsFrames result = gpsFrames;
    portEXIT_CRITICAL(&gpsMux);
    return result;
}

void XcTrackProtocol::Stream::reset() { *this = Stream{}; }
void XcTrackProtocol::Stream::append(const char* line) {
    const size_t n = strlen(line);
    if (length_+n >= sizeof(buffer_)) return;
    memcpy(buffer_+length_, line, n+1); length_ += n;
}
void XcTrackProtocol::Stream::prepare(uint32_t nowMs, const VarioState& s,
                                      const GpsState& gps, const GpsFrames& frames) {
    if (sent_ < length_ && (uint32_t)(nowMs-batchMs_) < 250) return;
    length_ = sent_ = 0;
    if (started_ && (uint32_t)(nowMs-varioMs_) < 200) return;
    started_ = true; batchMs_ = varioMs_ = nowMs;
    // Terminate a possibly interrupted old line after congestion/re-subscribe.
    append("\r\n");
    char line[LINE_BYTES];
    if (encodeVario(s, nowMs, line, sizeof(line))) append(line);
    if (!gpsStarted_ || (uint32_t)(nowMs-gpsMs_) >= 1000) {
        gpsStarted_ = true; gpsMs_ = nowMs;
        const bool fix = gps.fix && (uint32_t)(nowMs-gps.fixMs) < 3000 &&
            isfinite(gps.latitude) && fabs(gps.latitude) <= 90 &&
            isfinite(gps.longitude) && fabs(gps.longitude) <= 180 &&
            isfinite(gps.hdop) && gps.hdop > 0 && gps.hdop <= 5 && gps.satellites >= 4;
        if (fix && frames.rmc[0] && (uint32_t)(nowMs-frames.rmcMs) < 3000) append(frames.rmc);
        else if (sentence("GPRMC,,V,,,,,,,,,,N", line, sizeof(line))) append(line);
        if (fix && frames.gga[0] && (uint32_t)(nowMs-frames.ggaMs) < 3000) append(frames.gga);
        else if (sentence("GPGGA,,,,,,0,00,99.9,,M,,M,,", line, sizeof(line))) append(line);
    }
}
size_t XcTrackProtocol::Stream::chunkSize(uint16_t mtu) const {
    const size_t payload = min((size_t)182, (size_t)(max((uint16_t)23, mtu)-3));
    return min(payload, length_-sent_);
}
void XcTrackProtocol::Stream::advance(size_t bytes) {
    sent_ += min(bytes, length_-sent_);
}
