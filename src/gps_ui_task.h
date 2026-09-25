#pragma once

// Core 1: lower-priority task. Drains the GPS UART into TinyGPS++ on
// every tick (fast enough at UI_TASK_HZ to never lose NMEA bytes), and
// redraws the OLED at the slower DISPLAY_REFRESH_HZ rate.
void uiTaskFunc(void* pvParameters);
