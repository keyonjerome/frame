#!/usr/bin/env python3
import os
import time
from pathlib import Path

class SysfsPWM:
    """
    Control Linux sysfs PWM: /sys/class/pwm/pwmchipN/pwmM/...

    Notes:
    - Requires root (sudo) unless you adjust permissions.
    - Uses nanoseconds for period/duty.
    """

    def __init__(self, pwmchip: int, channel: int):
        self.pwmchip = pwmchip
        self.channel = channel
        self.chip_path = Path(f"/sys/class/pwm/pwmchip{pwmchip}")
        self.pwm_path = self.chip_path / f"pwm{channel}"

    @staticmethod
    def _write(path: Path, value: str) -> None:
        path.write_text(str(value))

    def export(self) -> None:
        if not self.pwm_path.exists():
            self._write(self.chip_path / "export", str(self.channel))
            # sysfs can take a moment to populate
            for _ in range(50):
                if self.pwm_path.exists():
                    break
                time.sleep(0.01)
        if not self.pwm_path.exists():
            raise RuntimeError(f"Failed to export pwm{self.channel} on pwmchip{self.pwmchip}")

    def unexport(self) -> None:
        if self.pwm_path.exists():
            self._write(self.chip_path / "unexport", str(self.channel))

    def enable(self, en: bool) -> None:
        self._write(self.pwm_path / "enable", "1" if en else "0")

    def set_period_ns(self, period_ns: int) -> None:
        self._write(self.pwm_path / "period", str(period_ns))

    def set_duty_ns(self, duty_ns: int) -> None:
        self._write(self.pwm_path / "duty_cycle", str(duty_ns))

    def set_polarity(self, polarity: str = "normal") -> None:
        # polarity: "normal" or "inversed" depending on kernel support
        pol_path = self.pwm_path / "polarity"
        if pol_path.exists():
            self._write(pol_path, polarity)

    def set_freq_hz(self, hz: float) -> None:
        period_ns = int(1e9 / hz)
        self.set_period_ns(period_ns)

    def set_pulse_us(self, pulse_us: float, freq_hz: float = 50.0) -> None:
        period_ns = int(1e9 / freq_hz)
        duty_ns = int(pulse_us * 1000.0)
        # Ensure duty <= period
        duty_ns = max(0, min(duty_ns, period_ns))
        self.set_period_ns(period_ns)
        self.set_duty_ns(duty_ns)

def find_pwmchip_by_platform(substr: str) -> int:
    """
    Find pwmchipN whose /sys/class/pwm/pwmchipN/device symlink contains substr.
    Example substr: "32c0000.pwm"
    """
    base = Path("/sys/class/pwm")
    for chip in sorted(base.glob("pwmchip*")):
        dev = chip / "device"
        try:
            target = os.path.realpath(dev)
        except OSError:
            continue
        if substr in target:
            return int(chip.name.replace("pwmchip", ""))
    raise RuntimeError(f"Could not find pwmchip for platform containing '{substr}'")

def duty_flip_test(pwm: SysfsPWM, freq_hz: float = 1000.0) -> None:
    """
    Easy-to-measure test: flip duty 10% <-> 90% at 1 kHz.
    On a multimeter you should see ~0.3V then ~3.0V average on a 3.3V pin.
    """
    pwm.export()
    pwm.enable(False)
    pwm.set_freq_hz(freq_hz)
    pwm.enable(True)

    period_ns = int(1e9 / freq_hz)
    d10 = int(period_ns * 0.10)
    d90 = int(period_ns * 0.90)

    while True:
        pwm.set_duty_ns(d10)
        time.sleep(1)
        pwm.set_duty_ns(d90)
        time.sleep(1)

def main():
    # Pin 33 (per your mapping) is coming from 32c0000.pwm -> pwmchip2 on your system.
    # We'll locate it robustly anyway:
    chip = find_pwmchip_by_platform("32c0000.pwm")
    pwm = SysfsPWM(pwmchip=chip, channel=0)

    print(f"Using pwmchip{chip}/pwm0 for 32c0000.pwm")

    # 1) First do the duty flip test (recommended for verifying the physical pin)
    # Probe header pin 33 to GND with a multimeter.
    duty_flip_test(pwm)

if __name__ == "__main__":
    main()
