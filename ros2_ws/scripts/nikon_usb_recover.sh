#!/usr/bin/env bash
set -euo pipefail

VID="04b0"
PID="0445"
LSUSB_BIN="$(command -v lsusb || true)"
FUSER_BIN="$(command -v fuser || true)"

log(){ echo "[nikon_usb_recover] $*"; }

# Kill common auto-grabbers (harmless if not running)
kill_grabbers() {
  log "Killing common camera/MTP auto-grabbers (gvfs/gphoto/mtp-probe)..."
  sudo pkill -f gvfs-gphoto2-volume-monitor 2>/dev/null || true
  sudo pkill -f gvfsd-mtp 2>/dev/null || true
  sudo pkill -f gvfsd 2>/dev/null || true
  sudo pkill -f mtp-probe 2>/dev/null || true
  sudo pkill -f gphoto2 2>/dev/null || true
}

# Find current /dev/bus/usb/BBB/DDD for the Nikon (retry to handle re-enumeration)
find_devnode() {
  local attempt=0 line bus dev
  if [[ -z "${LSUSB_BIN}" ]]; then
    log "lsusb not found; install usbutils to enable USB port recovery."
    return 1
  fi
  while [[ ${attempt} -lt 3 ]]; do
    line="$("${LSUSB_BIN}" -d ${VID}:${PID} | head -n1 || true)"
    if [[ -n "${line}" ]]; then
      bus="$(awk '{print $2}' <<<"${line}")"
      dev="$(awk '{print $4}' <<<"${line}" | tr -d ':')"
      if [[ -n "${bus}" && -n "${dev}" ]]; then
        # Strip leading zeros to avoid octal interpretation in printf
        bus="$(printf '%03d' "$((10#${bus}))")"
        dev="$(printf '%03d' "$((10#${dev}))")"
        printf "/dev/bus/usb/%s/%s\n" "${bus}" "${dev}"
        return 0
      fi
    fi
    sleep 0.3
    attempt=$((attempt + 1))
  done
  log "Camera ${VID}:${PID} not found in lsusb after retries."
  return 1
}

# Unbind/rebind the USB device (resets kernel side without reboot)
usb_rebind() {
  local devnode="$1"
  if [[ ! -e "${devnode}" ]]; then
    log "Device node ${devnode} missing; skipping rebind."
    return 0
  fi
  local udev_path kpath
  udev_path="$(udevadm info -q path -n "${devnode}" 2>/dev/null || true)"
  if [[ -z "${udev_path}" ]]; then
    log "udevadm could not resolve ${devnode}; skipping rebind."
    return 0
  fi
  kpath="$(udevadm info -a -p "${udev_path}" | awk -F'\"' '/KERNELS==\"[0-9]+-/{print $2; exit}')"
  if [[ -z "${kpath}" ]]; then
    log "Could not determine sysfs KERNELS path (e.g., 1-2). Skipping rebind."
    return 0
  fi

  log "Rebinding USB device at ${kpath} ..."
  echo "${kpath}" | sudo tee /sys/bus/usb/drivers/usb/unbind >/dev/null || true
  sleep 0.5
  echo "${kpath}" | sudo tee /sys/bus/usb/drivers/usb/bind >/dev/null || true
}

# Show who is holding the device node (debug)
show_holders() {
  local devnode="$1"
  if [[ ! -e "${devnode}" ]]; then
    log "Device node ${devnode} does not exist; skipping holder check."
    return 0
  fi
  if [[ -z "${FUSER_BIN}" ]]; then
    log "fuser not installed; skipping holder check (install psmisc to enable)."
    return 0
  fi
  log "Checking holders of ${devnode} ..."
  sudo "${FUSER_BIN}" -v "${devnode}" || true
}

main() {
  if [[ $EUID -ne 0 ]]; then
    # re-exec with sudo so unbind works when needed
    exec sudo -E "$0" "$@"
  fi

  kill_grabbers

  local devnode
  devnode="$(find_devnode)" || { log "Skipping USB rebind (lsusb or camera not available)."; exit 0; }

  show_holders "${devnode}"
  usb_rebind "${devnode}"
  show_holders "${devnode}"

  log "Done. If mtplvcap still times out, power-cycle the camera (battery out 10s) and retry."
}

main "$@"
