#!/usr/bin/env bash
set -euo pipefail

CAM_IP="${CAM_IP:-10.98.32.1}"
HOST_IP="${HOST_IP:-10.98.32.2}"
SUBNET_CIDR="${SUBNET_CIDR:-24}"
IFACE="${1:-}"

log() { echo "[zcam-eth] $*"; }
fail() { echo "[zcam-eth] ERROR: $*" >&2; exit 1; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Missing command: $1"
}

need_cmd ip
need_cmd ping
need_cmd curl

if [[ $EUID -ne 0 ]]; then
  fail "Run with sudo: sudo $0 [interface]"
fi

# Auto-detect a wired interface with carrier if none provided.
if [[ -z "$IFACE" ]]; then
  for dev in /sys/class/net/*; do
    name="$(basename "$dev")"

    [[ "$name" == "lo" ]] && continue
    [[ "$name" == docker* ]] && continue
    [[ "$name" == br-* ]] && continue
    [[ "$name" == veth* ]] && continue
    [[ "$name" == wl* ]] && continue

    if [[ -f "$dev/carrier" ]] && [[ "$(cat "$dev/carrier" 2>/dev/null || echo 0)" == "1" ]]; then
      IFACE="$name"
      break
    fi
  done
fi

[[ -n "$IFACE" ]] || fail "Could not auto-detect wired interface. Try: sudo $0 enp0s31f6"
[[ -d "/sys/class/net/$IFACE" ]] || fail "Interface does not exist: $IFACE"

log "Using interface: $IFACE"
log "Camera IP: $CAM_IP"
log "Host IP: $HOST_IP/$SUBNET_CIDR"

ip link set "$IFACE" up

if [[ -f "/sys/class/net/$IFACE/carrier" ]]; then
  carrier="$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo 0)"
  [[ "$carrier" == "1" ]] || fail "No Ethernet carrier on $IFACE. Check cable/camera Ethernet port."
fi

# Add host IP if missing.
if ! ip -4 addr show dev "$IFACE" | grep -q "inet ${HOST_IP}/${SUBNET_CIDR}"; then
  log "Adding ${HOST_IP}/${SUBNET_CIDR} to $IFACE"
  ip addr add "${HOST_IP}/${SUBNET_CIDR}" dev "$IFACE" 2>/dev/null || true
else
  log "Host IP already present"
fi

# Force only Z CAM subnet to use Ethernet.
log "Installing route for 10.98.32.0/24 via $IFACE"
ip route replace 10.98.32.0/24 dev "$IFACE" src "$HOST_IP"

log "Current route to camera:"
ip route get "$CAM_IP" || true

log "Pinging camera..."
if ping -I "$IFACE" -c 3 -W 1 "$CAM_IP"; then
  log "Ping OK"
else
  fail "Ping failed. Camera may not actually be at $CAM_IP, or Ethernet control/static IP is not active."
fi

log "Testing /info..."
curl --fail --silent --show-error --max-time 3 "http://${CAM_IP}/info"
echo

log "Trying to release any stale control session, if supported..."
curl --silent --show-error --max-time 2 "http://${CAM_IP}/ctrl/session?action=quit" || true
echo

log "Done. Z CAM Ethernet is reachable."
