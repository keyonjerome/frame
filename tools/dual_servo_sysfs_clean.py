#!/usr/bin/env python3
import time
from pathlib import Path
import os

def describe_pwm(pwm, label):
    chip = pwm.chip
    ch   = pwm.ch

    chip_path = f"/sys/class/pwm/pwmchip{chip}"
    pwm_path  = f"{chip_path}/pwm{ch}"
    dev_path  = os.path.realpath(f"{chip_path}/device")

    print(f"\n[{label}]")
    print(f"  pwmchip        : pwmchip{chip}")
    print(f"  channel        : pwm{ch}")
    print(f"  pwm sysfs path : {pwm_path}")
    print(f"  device backend : {dev_path}")

    # Optional: show current state if already configured
    try:
        with open(f"{pwm_path}/period") as f:
            print(f"  period (ns)    : {f.read().strip()}")
        with open(f"{pwm_path}/duty_cycle") as f:
            print(f"  duty (ns)      : {f.read().strip()}")
        with open(f"{pwm_path}/enable") as f:
            print(f"  enabled        : {f.read().strip()}")
    except FileNotFoundError:
        print("  (pwm not yet exported)")

# ===== Joint limits (geometric) =====
MIN_ANGLE = 20
MAX_ANGLE = 40

# ===== Timing =====
STEP_DELAY_S = 0.015
STARTUP_HOLD_S = 3.0

# ===== Servo PWM tuning =====
SERVO_HZ = 50.0                 # standard servo
MIN_PULSE_US = 1000             # tune if needed (e.g., 900..1100)
MAX_PULSE_US = 2000             # tune if needed (e.g., 1900..2100)

# ===== PWM mappings (based on your debugfs tests) =====
# Pin 32 -> pwmchip3 (32e0000.pwm), channel 0
SERVO_A_PWMCHIP = 3
SERVO_A_CH = 0

# Pin 33 -> pwmchip2 (32c0000.pwm), channel 0
SERVO_B_PWMCHIP = 2
SERVO_B_CH = 0


def clamp(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, x))


def mirrorAngle(angleDeg: int) -> int:
    # keep your semantics: constrain then mirror in the joint limits
    angleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE)
    return (MIN_ANGLE + MAX_ANGLE) - angleDeg


def angle_to_pulse_us(angleDeg: int) -> int:
    """
    Map angle in [MIN_ANGLE, MAX_ANGLE] to pulse width in [MIN_PULSE_US, MAX_PULSE_US].
    """
    angleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE)
    span = MAX_ANGLE - MIN_ANGLE
    if span <= 0:
        raise ValueError("MAX_ANGLE must be > MIN_ANGLE")
    t = (angleDeg - MIN_ANGLE) / span  # 0..1
    pulse = MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)
    return int(round(pulse))


class SysfsPWM:
    """
    Robust sysfs PWM driver.
    Uses:
      /sys/class/pwm/pwmchipN/export
      /sys/class/pwm/pwmchipN/pwmM/{period,duty_cycle,enable}
    """
    def __init__(self, chip: int, ch: int):
        self.chip = chip
        self.ch = ch
        self.chip_path = Path(f"/sys/class/pwm/pwmchip{chip}")
        self.pwm_path = self.chip_path / f"pwm{ch}"
        if not self.chip_path.exists():
            raise RuntimeError(f"pwmchip{chip} not found")

    def _write(self, path: Path, value: str) -> None:
        path.write_text(str(value))

    def export(self) -> None:
        if not self.pwm_path.exists():
            # export channel
            try:
                self._write(self.chip_path / "export", str(self.ch))
            except Exception:
                # already exported by someone else, or permission issue
                pass

            # wait for node creation
            for _ in range(100):
                if self.pwm_path.exists():
                    break
                time.sleep(0.01)

        if not self.pwm_path.exists():
            raise RuntimeError(f"Failed to export pwm{self.ch} on pwmchip{self.chip}")

    def enable(self, en: bool) -> None:
        try:
            self._write(self.pwm_path / "enable", "1" if en else "0")
        except Exception:
            # some drivers are picky; we'll treat enable failures as non-fatal if state is already correct
            pass

    def set_period_ns(self, ns: int) -> None:
        self._write(self.pwm_path / "period", str(ns))

    def set_duty_ns(self, ns: int) -> None:
        self._write(self.pwm_path / "duty_cycle", str(ns))

    def setup(self, hz: float, initial_pulse_us: int) -> None:
        """
        Safe ordering:
          disable (best effort) -> set period -> set duty -> enable
        Then we only update duty during runtime.
        """
        period_ns = int(round(1e9 / hz))
        duty_ns = int(initial_pulse_us) * 1000
        duty_ns = max(0, min(duty_ns, period_ns))

        self.export()
        self.enable(False)
        self.set_period_ns(period_ns)
        self.set_duty_ns(duty_ns)
        self.enable(True)

    def set_pulse_us(self, pulse_us: int) -> None:
        # Only touch duty_cycle for smooth updates
        self.set_duty_ns(int(pulse_us) * 1000)

    def relax(self) -> None:
        # Turn PWM off (servo may go limp)
        self.enable(False)


class DualServo:
    def __init__(self, pwmA: SysfsPWM, pwmB: SysfsPWM):
        self.a = pwmA
        self.b = pwmB

    def set_geometric(self, geomDeg: int) -> None:
        geomDeg = clamp(geomDeg, MIN_ANGLE, MAX_ANGLE)
        a_pulse = angle_to_pulse_us(geomDeg)
        b_pulse = angle_to_pulse_us(mirrorAngle(geomDeg))
        self.a.set_pulse_us(a_pulse)
        self.b.set_pulse_us(b_pulse)


def main():
    pwmA = SysfsPWM(SERVO_A_PWMCHIP, SERVO_A_CH)
    pwmB = SysfsPWM(SERVO_B_PWMCHIP, SERVO_B_CH)

    # Initialize both at geometric MIN_ANGLE
    init_geom = MIN_ANGLE
    pwmA.setup(SERVO_HZ, angle_to_pulse_us(init_geom))
    pwmB.setup(SERVO_HZ, angle_to_pulse_us(mirrorAngle(init_geom)))

    print("\n=== PWM DEVICES IN USE ===")
    describe_pwm(pwmA, "Servo A (pin 32)")
    describe_pwm(pwmB, "Servo B (pin 33)")
    print("==========================\n")
    
    servos = DualServo(pwmA, pwmB)

    time.sleep(STARTUP_HOLD_S)

    try:
        while True:
            for geom in range(MIN_ANGLE, MAX_ANGLE + 1):
                servos.set_geometric(geom)
                time.sleep(STEP_DELAY_S)

            for geom in range(MAX_ANGLE, MIN_ANGLE - 1, -1):
                servos.set_geometric(geom)
                time.sleep(STEP_DELAY_S)

    except KeyboardInterrupt:
        pass
    finally:
        pwmA.relax()
        pwmB.relax()
        print("\nPWM disabled (relaxed).")


if __name__ == "__main__":
    main()
