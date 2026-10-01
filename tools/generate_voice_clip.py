"""Convert checked-in 16 kHz mono WAV prompts to flash-resident C++ arrays."""

from pathlib import Path
import struct
import wave


ROOT = Path(__file__).resolve().parent.parent
PROMPTS = {
    "FLIGHT_STARTED": "flight_started_tr.wav",
    "FLIGHT_ENDED": "flight_ended_tr.wav",
    "GPS_ACQUIRED": "gps_acquired_tr.wav",
    "GPS_LOST": "gps_lost_tr.wav",
    "ALTITUDE_LIMIT_APPROACHING": "altitude_limit_approaching_tr.wav",
}
HEADER = ROOT / "src/generated/voice_prompts.h"
IMPLEMENTATION = ROOT / "src/generated/voice_prompts.cpp"

decoded = {}
for symbol, filename in PROMPTS.items():
    source = ROOT / "assets/audio" / filename
    with wave.open(str(source), "rb") as wav:
        if wav.getnchannels() != 1 or wav.getsampwidth() != 2 or wav.getframerate() != 16000:
            raise SystemExit(f"{filename} must be 16 kHz, 16-bit, mono PCM")
        frames = wav.readframes(wav.getnframes())
    decoded[symbol] = struct.unpack(f"<{len(frames) // 2}h", frames)

header = [
    "#pragma once",
    "#include <stddef.h>",
    "#include <stdint.h>",
    "",
    "namespace VoicePrompts {",
]
for symbol, samples in decoded.items():
    header.append(f"    extern const int16_t {symbol}[];")
    header.append(f"    inline constexpr size_t {symbol}_COUNT = {len(samples)};")
header += ["}", ""]
HEADER.write_text("\n".join(header), encoding="utf-8")

lines = ['#include "voice_prompts.h"', "#include <Arduino.h>", ""]
for symbol, samples in decoded.items():
    lines.append(f"const int16_t VoicePrompts::{symbol}[] PROGMEM = {{")
    for index in range(0, len(samples), 16):
        lines.append("    " + ", ".join(str(value) for value in samples[index:index + 16]) + ",")
    lines += ["};", ""]
IMPLEMENTATION.write_text("\n".join(lines), encoding="utf-8")

print("generated " + ", ".join(f"{name}={len(samples)}" for name, samples in decoded.items()))
