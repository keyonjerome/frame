#!/usr/bin/env python3
import time
import os
from pathlib import Path

# ===== User parameters =====
# Physical behavior range (your "geometric" joint limits)
MIN_ANGLE = 5
MAX_ANGLE = 25

# Timing (Arduino-ish)
STEP_DELAY_S = 0.015
STARTUP_HOLD_S = 3.0

# Servo PWM settings (start conservative; tune if needed)
SERVO_HZ = 50.0
MIN_PULSE_US = 1000   # typical: 1000
MAX_PULSE_US = 2000   # typical: 2000

# ---- Mapping guess based on your system ----
# pin 33 is confirmed as 32c0000.pwm -> pwmchip2/pwm0
SERVO_B_PWMCHIP = 2   # 32c0000.pwm
SERVO_B_CH      = 0

# pin 32 is guessed as 32e0000.pwm -> pwmchip3/pwm0
SERVO_A_PWMCHIP = 3   # 32e0000.pwm (guess)
SERVO_A_CH      = 0


def clamp(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, x))


def mirrorAngle(angleDeg: int) -> int:
    """Keep your semantics: constrain then mirror within joint limits."""
    angleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE)
    return (MIN_ANGLE + MAX_ANGLE) - angleDeg


def angle_to_pulse_us(angleDeg: int) -> int:
    """
    Map angle in [MIN_ANGLE, MAX_ANGLE] to pulse width in [MIN_PULSE_US, MAX_PULSE_US].
    This keeps your limited motion range but still gives full PWM resolution.
    """
    angleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE)
    span = (MAX_ANGLE - MIN_ANGLE)
    if span <= 0:
        raise ValueError("MAX_ANGLE must be > MIN_ANGLE")

    t = (angleDeg - MIN_ANGLE) / span  # 0..1
    pulse = MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)
    return int(round(pulse))


class SysfsPWM:
    """
    Minimal sysfs PWM driver: /sys/class/pwm/pwmchipN/pwmM/{period,duty_cycle,enable}
    """
    def __init__(self, pwmchip: int, channel: int):
        self.chip = pwmchip
        self.ch = channel
        self.chip_path = Path(f"/sys/class/pwm/pwmchip{pwmchip}")
        self.pwm_path = self.chip_path / f"pwm{channel}"
        if not self.chip_path.exists():
            raise RuntimeError(f"pwmchip{pwmchip} does not exist")

    def _write(self, p: Path, v: str) -> None:
        p.write_text(str(v))

    def export(self) -> None:
        if not self.pwm_path.exists():
            self._write(self.chip_path / "export", str(self.ch))
            # wait for sysfs node creation
            for _ in range(100):
                if self.pwm_path.exists():
                    return
                time.sleep(0.01)
            raise RuntimeError(f"Failed to export pwm{self.ch} on pwmchip{self.chip}")

    def enable(self, en: bool) -> None:
        self._write(self.pwm_path / "enable", "1" if en else "0")

    def set_period_ns(self, ns: int) -> None:
        self._write(self.pwm_path / "period", str(ns))

    def set_duty_ns(self, ns: int) -> None:
        self._write(self.pwm_path / "duty_cycle", str(ns))

    def set_pulse_us(self, pulse_us: int, hz: float) -> None:
        period_ns = int(round(1e9 / hz))
        duty_ns = int(pulse_us) * 1000
        if duty_ns > period_ns:
            duty_ns = period_ns
        if duty_ns < 0:
            duty_ns = 0

        # Many PWM drivers require disable before changing period
        self.enable(False)
        self.set_period_ns(period_ns)
        self.set_duty_ns(duty_ns)
        self.enable(True)

    def stop(self) -> None:
        # "Relax": stop PWM output
        try:
            self.enable(False)
        except Exception:
            pass


class DualServo:
    def __init__(self, a: SysfsPWM, b: SysfsPWM):
        self.a = a
        self.b = b

    def set_geometric_angle(self, geomDeg: int) -> None:
        geomDeg = clamp(geomDeg, MIN_ANGLE, MAX_ANGLE)
        a_pulse = angle_to_pulse_us(geomDeg)
        b_pulse = angle_to_pulse_us(mirrorAngle(geomDeg))

        self.a.set_pulse_us(a_pulse, SERVO_HZ)
        self.b.set_pulse_us(b_pulse, SERVO_HZ)

    def relax(self) -> None:
        self.a.stop()
        self.b.stop()


def main():
    # Sanity print mapping (so you notice if it’s wrong)
    print("Using sysfs PWM mapping:")
    print(f"  Servo A (pin 32, guessed) -> pwmchip{SERVO_A_PWMCHIP}/pwm{SERVO_A_CH} "
          f"({os.path.realpath(f'/sys/class/pwm/pwmchip{SERVO_A_PWMCHIP}/device')})")
    print(f"  Servo B (pin 33, known)  -> pwmchip{SERVO_B_PWMCHIP}/pwm{SERVO_B_CH} "
          f"({os.path.realpath(f'/sys/class/pwm/pwmchip{SERVO_B_PWMCHIP}/device')})")

    pwmA = SysfsPWM(SERVO_A_PWMCHIP, SERVO_A_CH)
    pwmB = SysfsPWM(SERVO_B_PWMCHIP, SERVO_B_CH)

    pwmA.export()
    pwmB.export()

    servos = DualServo(pwmA, pwmB)

    # 1) Set both to same geometric min
    servos.set_geometric_angle(MIN_ANGLE)

    # 2) Wait 3 seconds
    time.sleep(STARTUP_HOLD_S)

    try:
        while True:
            # 3) Sweep MIN -> MAX
            for geom in range(MIN_ANGLE, MAX_ANGLE + 1):
                servos.set_geometric_angle(geom)
                time.sleep(STEP_DELAY_S)

            # Sweep MAX -> MIN
            for geom in range(MAX_ANGLE, MIN_ANGLE - 1, -1):
                servos.set_geometric_angle(geom)
                time.sleep(STEP_DELAY_S)

    except KeyboardInterrupt:
        pass
    finally:
        servos.relax()
        print("\nStopped PWM (relaxed).")


if __name__ == "__main__":
    main()
