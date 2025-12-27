#!/usr/bin/env bash
set -euo pipefail

VID="04b0"
PID="0445"

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

# Find current /dev/bus/usb/BBB/DDD for the Nikon
find_devnode() {
  local line
  line="$(lsusb -d ${VID}:${PID} | head -n1 || true)"
  if [[ -z "${line}" ]]; then
    log "Camera ${VID}:${PID} not found in lsusb."
    return 1
  fi
  # "Bus 001 Device 021: ID 04b0:0445 ..."
  local bus dev
  bus="$(awk '{print $2}' <<<"${line}")"
  dev="$(awk '{print $4}' <<<"${line}" | tr -d ':')"
  printf "/dev/bus/usb/%03d/%03d\n" "${bus}" "${dev}"
}

# Unbind/rebind the USB device (resets kernel side without reboot)
usb_rebind() {
  local devnode="$1"
  local kpath
  kpath="$(udevadm info -a -p "$(udevadm info -q path -n "${devnode}")" | \
           awk -F'"' '/KERNELS=="1-/{print $2; exit}')"
  if [[ -z "${kpath}" ]]; then
    log "Could not determine sysfs KERNELS path (e.g., 1-2). Skipping rebind."
    return 0
  fi

  log "Rebinding USB device at ${kpath} ..."
  echo "${kpath}" | sudo tee /sys/bus/usb/drivers/usb/unbind >/dev/null
  sleep 0.5
  echo "${kpath}" | sudo tee /sys/bus/usb/drivers/usb/bind >/dev/null
}

# Show who is holding the device node (debug)
show_holders() {
  local devnode="$1"
  log "Checking holders of ${devnode} ..."
  sudo fuser -v "${devnode}" || true
}

main() {
  if [[ $EUID -ne 0 ]]; then
    # re-exec with sudo so unbind works when needed
    exec sudo -E "$0" "$@"
  fi

  kill_grabbers

  local devnode
  devnode="$(find_devnode)" || exit 1

  show_holders "${devnode}"
  usb_rebind "${devnode}"
  show_holders "${devnode}"

  log "Done. If mtplvcap still times out, power-cycle the camera (battery out 10s) and retry."
}

main "$@"
