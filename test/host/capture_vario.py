"""Capture/analyse the existing @VARIO diagnostic stream without resetting ESP32.

This is a downsampled diagnostic record, NOT a full-rate filter replay input.
Use production C++ stability checks for deterministic algorithm comparisons.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

FIELDS = """t_us dt_s baro_count pressure_pa temp_c baro_alt_m baro_vario_mps
kalman_alt_m kalman_vario_mps output_mps earth_z_mps2 pitch_deg roll_deg bias_mps2
raw_d1 raw_d2 filtered_d2 baro_variance imu_misses baro_misses long_ticks
imu_sample_ok ax_g ay_g az_g accel_bias_g""".split()


def parse_record(line):
    if not line.startswith("@VARIO,"):
        return None
    values = line.strip().split(",")[1:]
    if len(values) != len(FIELDS):
        return None
    try:
        values = list(map(float, values))
    except ValueError:
        return None
    if not all(map(math.isfinite, values)):
        return None
    return dict(zip(FIELDS, values))


def summarize(rows):
    if len(rows) < 2:
        raise ValueError("At least two valid @VARIO records are needed; check SERIAL_CSV.")
    elapsed = [0.0]
    for prev, row in zip(rows, rows[1:]):
        delta = (int(row["t_us"]) - int(prev["t_us"])) & 0xffffffff
        if delta == 0 or delta > 10_000_000:
            raise ValueError("Nonmonotonic timestamps, reboot, or >10 s gap; split the capture.")
        elapsed.append(elapsed[-1] + delta / 1e6)
    result = {
        "records": len(rows), "duration_s": elapsed[-1],
        "diagnostic_hz": (len(rows)-1)/elapsed[-1],
        "barometer_hz": (rows[-1]["baro_count"]-rows[0]["baro_count"])/elapsed[-1],
        "warning": "Stationarity/fusion state must be confirmed separately. Diagnostic stream is downsampled.",
    }
    for field in ("output_mps", "kalman_vario_mps", "baro_vario_mps", "earth_z_mps2",
                  "pressure_pa", "temp_c", "bias_mps2", "pitch_deg", "roll_deg", "baro_variance"):
        values = [r[field] for r in rows]
        result[field] = {"mean": statistics.fmean(values), "stddev": statistics.pstdev(values),
                         "min": min(values), "max": max(values)}
    # Separate slow pressure drift from random scatter on UNIQUE baro samples.
    unique = [(elapsed[i], r["pressure_pa"]) for i,r in enumerate(rows)
              if i == 0 or r["baro_count"] != rows[i-1]["baro_count"]]
    if len(unique) > 1:
        tm, pm = map(statistics.fmean, zip(*unique))
        denom = sum((t-tm)**2 for t,p in unique)
        slope = sum((t-tm)*(p-pm) for t,p in unique)/denom
        result["pressure_drift_pa_s"] = slope
        result["pressure_detrended_stddev_pa"] = statistics.pstdev(p-pm-slope*(t-tm) for t,p in unique)
    for key in ("imu_misses", "baro_misses", "long_ticks"):
        result[key+"_delta"] = rows[-1][key] - rows[0][key]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Capture serial port, e.g. COM3")
    source.add_argument("--input", type=Path, help="Analyse an existing raw serial .log")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--output", type=Path, help="New .log path; never overwrites an existing file")
    args = parser.parse_args()
    if args.input:
        with args.input.open(encoding="utf-8", errors="replace") as file:
            rows = [r for line in file if (r := parse_record(line)) is not None]
    else:
        if not args.output or not 1 <= args.seconds <= 300:
            parser.error("Capture requires --output and 1 <= --seconds <= 300")
        import serial  # Installed in PlatformIO's Python environment.
        args.output.parent.mkdir(parents=True, exist_ok=True)
        rows = []
        port = serial.Serial(port=None, baudrate=115200, timeout=.25)
        port.dtr = False
        port.rts = False
        port.port = args.port
        with args.output.open("x", encoding="utf-8") as log:
            port.open()
            try:
                until = time.monotonic()+args.seconds
                pending = bytearray()
                while time.monotonic() < until:
                    pending.extend(port.read(max(1, min(port.in_waiting,4096))))
                    while b"\n" in pending:
                        line, _, pending = pending.partition(b"\n")
                        text = line.decode("utf-8", errors="replace")
                        log.write(text+"\n")
                        record = parse_record(text)
                        if record is not None:
                            rows.append(record)
                    if len(pending) > 8192:
                        log.write(pending.decode("utf-8", errors="replace")+"\n")
                        pending.clear()
            finally:
                port.close()
        if rows:
            with args.output.with_suffix(".csv").open("x", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file,fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(rows)
    print(json.dumps(summarize(rows),ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
