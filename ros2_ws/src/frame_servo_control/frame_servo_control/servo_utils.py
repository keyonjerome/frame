#!/usr/bin/env python3
import os
import time
from pathlib import Path
from typing import Tuple


STARTUP_HOLD_S = 3.0
SERVO_HZ = 50.0
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000

SERVO_A_PWMCHIP = 3
SERVO_A_CH = 0

SERVO_B_PWMCHIP = 2
SERVO_B_CH = 0


def describe_pwm(pwm, label):
    chip = pwm.chip
    ch = pwm.ch

    chip_path = f"/sys/class/pwm/pwmchip{chip}"
    pwm_path = f"{chip_path}/pwm{ch}"
    dev_path = os.path.realpath(f"{chip_path}/device")

    print(f"\n[{label}]")
    print(f"  pwmchip        : pwmchip{chip}")
    print(f"  channel        : pwm{ch}")
    print(f"  pwm sysfs path : {pwm_path}")
    print(f"  device backend : {dev_path}")

    try:
        with open(f"{pwm_path}/period", encoding='utf-8') as handle:
            print(f"  period (ns)    : {handle.read().strip()}")
        with open(f"{pwm_path}/duty_cycle", encoding='utf-8') as handle:
            print(f"  duty (ns)      : {handle.read().strip()}")
        with open(f"{pwm_path}/enable", encoding='utf-8') as handle:
            print(f"  enabled        : {handle.read().strip()}")
    except FileNotFoundError:
        print("  (pwm not yet exported)")


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def mirror_angle(angle_deg: float, min_angle: float, max_angle: float) -> float:
    angle_deg = clamp(angle_deg, min_angle, max_angle)
    return (min_angle + max_angle) - angle_deg


def angle_to_pulse_us(
    angle_deg: float,
    min_angle: float,
    max_angle: float,
) -> int:
    angle_deg = clamp(angle_deg, min_angle, max_angle)
    span = max_angle - min_angle
    if span <= 0.0:
        raise ValueError("max_angle must be greater than min_angle")
    t = (angle_deg - min_angle) / span
    pulse = MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)
    return int(round(pulse))


def servo_pulses_for_angle(
    angle_deg: float,
    min_angle: float,
    max_angle: float,
) -> Tuple[int, int]:
    a_pulse = angle_to_pulse_us(angle_deg, min_angle, max_angle)
    b_pulse = angle_to_pulse_us(
        mirror_angle(angle_deg, min_angle, max_angle),
        min_angle,
        max_angle,
    )
    return a_pulse, b_pulse


class SysfsPWM:
    def __init__(self, chip: int, ch: int):
        self.chip = chip
        self.ch = ch
        self.chip_path = Path(f"/sys/class/pwm/pwmchip{chip}")
        self.pwm_path = self.chip_path / f"pwm{ch}"
        if not self.chip_path.exists():
            raise RuntimeError(f"pwmchip{chip} not found")

    def _write(self, path: Path, value: str) -> None:
        path.write_text(str(value), encoding='utf-8')

    def export(self) -> None:
        if not self.pwm_path.exists():
            try:
                self._write(self.chip_path / "export", str(self.ch))
            except Exception:
                pass

            for _ in range(100):
                if self.pwm_path.exists():
                    break
                time.sleep(0.01)

        if not self.pwm_path.exists():
            raise RuntimeError(f"failed to export pwm{self.ch} on pwmchip{self.chip}")

    def enable(self, enabled: bool) -> None:
        try:
            self._write(self.pwm_path / "enable", "1" if enabled else "0")
        except Exception:
            pass

    def set_period_ns(self, period_ns: int) -> None:
        self._write(self.pwm_path / "period", str(period_ns))

    def set_duty_ns(self, duty_ns: int) -> None:
        self._write(self.pwm_path / "duty_cycle", str(duty_ns))

    def setup(self, hz: float, initial_pulse_us: int) -> None:
        period_ns = int(round(1e9 / hz))
        duty_ns = int(initial_pulse_us) * 1000
        duty_ns = max(0, min(duty_ns, period_ns))

        self.export()
        self.enable(False)
        self.set_period_ns(period_ns)
        self.set_duty_ns(duty_ns)
        self.enable(True)

    def set_pulse_us(self, pulse_us: int) -> None:
        self.set_duty_ns(int(pulse_us) * 1000)

    def relax(self) -> None:
        self.enable(False)


class DualServo:
    def __init__(self, pwm_a: SysfsPWM, pwm_b: SysfsPWM):
        self.a = pwm_a
        self.b = pwm_b

    def set_pulses(self, a_pulse_us: int, b_pulse_us: int) -> None:
        self.a.set_pulse_us(a_pulse_us)
        self.b.set_pulse_us(b_pulse_us)
