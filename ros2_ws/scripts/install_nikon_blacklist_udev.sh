#!/usr/bin/env bash
set -euo pipefail

VID="04b0"
PID="0445"
RULE_FILE="/etc/udev/rules.d/90-nikon-d3500-no-gphoto.rules"

if [[ $EUID -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

cat > "${RULE_FILE}" <<EOF
# Prevent gphoto2/gvfs/mtp helpers from auto-claiming Nikon D3500 (VID:${VID} PID:${PID})
ATTR{idVendor}=="${VID}", ATTR{idProduct}=="${PID}", ENV{ID_GPHOTO2}="0", ENV{ID_MTP_DEVICE}="0"
EOF

udevadm control --reload-rules
udevadm trigger

echo "Installed udev rule: ${RULE_FILE}"
echo "Unplug/replug the camera once."
