#include "debug_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdarg.h>

namespace {
    SemaphoreHandle_t s_mutex = nullptr;
    constexpr size_t  BUF_CAP = 4096; // usable capacity (1 byte reserved for '\0')
    char   s_buf[BUF_CAP + 1];
    size_t s_len = 0;

    // Monotonic counters for pullNewForSerial(): s_totalAppended never
    // decreases, so "bytes evicted so far" = s_totalAppended - s_len at
    // any point in time. s_serialCursor tracks how far the (single)
    // Serial-mirroring consumer has read.
    size_t s_totalAppended = 0;
    size_t s_serialCursor  = 0;

    // Caller must already hold s_mutex.
    void appendLocked(const char* text) {
        size_t addLen = strlen(text);
        if (addLen > BUF_CAP) {
            // Pathological single write larger than the whole buffer —
            // keep only its tail.
            text += (addLen - BUF_CAP);
            addLen = BUF_CAP;
        }
        if (s_len + addLen > BUF_CAP) {
            size_t drop = (s_len + addLen) - BUF_CAP;
            memmove(s_buf, s_buf + drop, s_len - drop);
            s_len -= drop;
        }
        memcpy(s_buf + s_len, text, addLen);
        s_len += addLen;
        s_buf[s_len] = '\0';
        s_totalAppended += addLen;
    }
}

void DebugLog::init() {
    s_mutex = xSemaphoreCreateMutex();
    s_buf[0] = '\0';
}

void DebugLog::log(const char* text) {
    // NOT: Serial'e artik burada DOKUNULMUYOR. Bu fonksiyon vario_task
    // (Core 0, gercek-zamanli, 100Hz) icinden cagriliyor; USB-CDC'nin
    // baglanti kesilirken host FIFO'sunu bosaltmayi birakip
    // Serial.print()/write()'i suresiz bloklamasi ("seri port baglanip
    // kesilince vario donuyor" hatasi) buradan tamamen kaldirildi.
    // Serial mirror'lama artik SADECE ble_task icinde, dusuk oncelikli
    // ve non-blocking sekilde, pullNewForSerial() ile yapiliyor.
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
        appendLocked(text);
        xSemaphoreGive(s_mutex);
    }
}

void DebugLog::logf(const char* fmt, ...) {
    char tmp[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    log(tmp);
}

String DebugLog::snapshot() {
    String out;
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        out = s_buf;
        xSemaphoreGive(s_mutex);
    }
    return out;
}

String DebugLog::pullNewForSerial() {
    String out;
    if (!s_mutex) return out;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        size_t evictedTotal = s_totalAppended - s_len; // artik buffer'da olmayan toplam bayt

        // Cursor, daha once mirror'lanamadan buffer'dan silinmis
        // veriyi isaret ediyorsa (ble_task yeterince sik cagrilmadiysa)
        // o kayip satirlari atlayip mevcut buffer basina hizala.
        if (s_serialCursor < evictedTotal) {
            s_serialCursor = evictedTotal;
        }

        size_t offsetInBuf = s_serialCursor - evictedTotal;
        if (offsetInBuf < s_len) {
            out = s_buf + offsetInBuf; // henuz gonderilmemis kuyruk
        }
        s_serialCursor = s_totalAppended;
        xSemaphoreGive(s_mutex);
    }
    return out;
}
