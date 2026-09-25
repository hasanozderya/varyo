#pragma once

// Core 1: WiFi access-point + HTTP server. Serves live telemetry, the
// rolling debug log, and a form to adjust Tunables at runtime. Not
// real-time-sensitive — runs alongside the UI/GPS task on Core 1, never
// on Core 0 with the vario/audio loop.
//
// WiFi AP is only up for Timing::WIFI_ON_DURATION_MS after sensor init
// finishes (see config.h) — this task auto-shuts the radio off after
// that window and suspends itself. Cycle power to open the settings
// window again.
void webTaskFunc(void* pvParameters);
