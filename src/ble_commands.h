#pragma once
#include <Arduino.h>

// Called only by the control task; callbacks must never write NVS or run commands.
String executeBleCommand(const char* jsonLine);
