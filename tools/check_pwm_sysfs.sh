#!/usr/bin/env bash
set -euo pipefail

PWM_CHIPS=(2 3)
PWM_CHANNEL=0
PWM_GROUP="${PWM_GROUP:-hostgpio}"
PERIOD_NS="${SERVO_PERIOD_NS:-20000000}"
DUTY_NS="${SERVO_DUTY_NS:-1500000}"
FIX=0

usage() {
  cat <<'EOF'
Usage: tools/check_pwm_sysfs.sh [--fix]

Checks the Jetson sysfs PWM channels used by frame_servo_control:
  - /sys/class/pwm/pwmchip2/pwm0
  - /sys/class/pwm/pwmchip3/pwm0

Default mode is read-only and returns non-zero if PWM is missing, unexported,
uninitialized, or not writable by the current user.

Use --fix on the Jetson host to export pwm0, set hostgpio group permissions,
and initialize a disabled 50 Hz servo-safe state:
  period=20000000 ns, duty_cycle=1500000 ns, enable=0

Environment overrides:
  PWM_GROUP        group granted write access, default hostgpio
  SERVO_PERIOD_NS  period to initialize, default 20000000
  SERVO_DUTY_NS    duty_cycle to initialize, default 1500000
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fix)
      FIX=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

need_sudo() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

write_sysfs() {
  local value="$1"
  local path="$2"
  if [[ "$(id -u)" -eq 0 ]]; then
    printf '%s' "$value" > "$path"
  else
    printf '%s' "$value" | sudo tee "$path" >/dev/null
  fi
}

status=0

echo "PWM sysfs check"
echo "user=$(id -un) uid=$(id -u) groups=$(id -Gn)"
echo

for chip in "${PWM_CHIPS[@]}"; do
  chip_path="/sys/class/pwm/pwmchip${chip}"
  pwm_path="${chip_path}/pwm${PWM_CHANNEL}"

  echo "== pwmchip${chip}/pwm${PWM_CHANNEL} =="

  if [[ ! -d "$chip_path" ]]; then
    echo "ERROR: missing $chip_path"
    echo "       Check Jetson pinmux/device-tree PWM exposure."
    status=1
    echo
    continue
  fi

  if [[ ! -d "$pwm_path" && "$FIX" -eq 1 ]]; then
    echo "exporting pwm${PWM_CHANNEL}"
    write_sysfs "$PWM_CHANNEL" "${chip_path}/export" || true
    for _ in $(seq 1 100); do
      [[ -d "$pwm_path" ]] && break
      sleep 0.01
    done
  fi

  if [[ ! -d "$pwm_path" ]]; then
    echo "ERROR: missing $pwm_path"
    echo "       Run: sudo $0 --fix"
    status=1
    echo
    continue
  fi

  if [[ "$FIX" -eq 1 ]]; then
    if getent group "$PWM_GROUP" >/dev/null 2>&1; then
      need_sudo chgrp -R "$PWM_GROUP" "$pwm_path"
      need_sudo chmod g+rw \
        "$pwm_path/period" \
        "$pwm_path/duty_cycle" \
        "$pwm_path/enable"
    else
      echo "WARN: group $PWM_GROUP does not exist; skipping group permission setup."
    fi

    write_sysfs 0 "$pwm_path/enable" || true
    write_sysfs "$PERIOD_NS" "$pwm_path/period"
    write_sysfs "$DUTY_NS" "$pwm_path/duty_cycle"
  fi

  ls -l "$pwm_path/period" "$pwm_path/duty_cycle" "$pwm_path/enable"
  period="$(cat "$pwm_path/period")"
  duty="$(cat "$pwm_path/duty_cycle")"
  enable="$(cat "$pwm_path/enable")"
  echo "period=$period duty_cycle=$duty enable=$enable"

  if [[ "$period" == "0" ]]; then
    echo "ERROR: period is 0; PWM is exported but not initialized."
    echo "       Run: sudo $0 --fix"
    status=1
  fi

  if [[ ! -w "$pwm_path/period" || ! -w "$pwm_path/duty_cycle" || ! -w "$pwm_path/enable" ]]; then
    echo "ERROR: current user cannot write period/duty_cycle/enable."
    if [[ "$FIX" -eq 0 ]]; then
      echo "       Run on the Jetson host: sudo $0 --fix"
    fi
    status=1
  else
    echo "write_permissions=ok"
  fi

  echo
done

if [[ "$status" -eq 0 ]]; then
  echo "PWM sysfs is available."
else
  echo "PWM sysfs is not ready."
fi

exit "$status"
