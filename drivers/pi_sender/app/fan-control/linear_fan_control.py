#!/usr/bin/env python3
"""Linear Raspberry Pi PWM fan controller with a fail-safe maximum speed."""

from __future__ import annotations

import signal
import sys
import time
from pathlib import Path


LOW_C = 25.0
HIGH_C = 60.0
PWM_MIN = 0
PWM_MAX = 255
POLL_SECONDS = 1.0
REPORT_SECONDS = 30.0


def find_cpu_temperature() -> Path:
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            if (zone / "type").read_text().strip() == "cpu-thermal":
                return zone / "temp"
        except OSError:
            continue
    raise RuntimeError("cpu-thermal temperature sensor not found")


def find_pwm_fan() -> tuple[Path, Path, Path | None]:
    for hwmon in sorted(Path("/sys/class/hwmon").glob("hwmon*")):
        try:
            if (hwmon / "name").read_text().strip() != "pwmfan":
                continue
        except OSError:
            continue
        pwm = hwmon / "pwm1"
        enable = hwmon / "pwm1_enable"
        rpm = hwmon / "fan1_input"
        if pwm.exists() and enable.exists():
            return pwm, enable, rpm if rpm.exists() else None
    raise RuntimeError("pwmfan hwmon interface not found")


def pwm_for_temperature(temp_c: float) -> int:
    if temp_c <= LOW_C:
        return PWM_MIN
    if temp_c >= HIGH_C:
        return PWM_MAX
    return round((temp_c - LOW_C) * (PWM_MAX - PWM_MIN) / (HIGH_C - LOW_C))


def write_value(path: Path, value: int) -> None:
    path.write_text(f"{value}\n")


def main() -> int:
    stop = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    temperature_path = find_cpu_temperature()
    pwm_path, enable_path, rpm_path = find_pwm_fan()
    write_value(enable_path, 1)  # manual PWM mode
    last_pwm: int | None = None
    next_report = 0.0
    try:
        while not stop:
            try:
                temp_c = int(temperature_path.read_text().strip()) / 1000.0
                pwm = pwm_for_temperature(temp_c)
                if pwm != last_pwm:
                    write_value(pwm_path, pwm)
                    last_pwm = pwm
                now = time.monotonic()
                if now >= next_report:
                    rpm = None
                    if rpm_path is not None:
                        try:
                            rpm = int(rpm_path.read_text().strip())
                        except (OSError, ValueError):
                            pass
                    print(
                        f"temperature_c={temp_c:.2f} pwm={pwm} rpm={rpm}",
                        flush=True,
                    )
                    next_report = now + REPORT_SECONDS
            except (OSError, ValueError) as error:
                write_value(pwm_path, PWM_MAX)
                print(f"fan control error; forcing PWM 255: {error}",
                      file=sys.stderr, flush=True)
                return 1
            time.sleep(POLL_SECONDS)
    finally:
        # A stopped or failed controller must leave the fan at a safe speed.
        try:
            write_value(enable_path, 1)
            write_value(pwm_path, PWM_MAX)
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
