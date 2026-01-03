#!/usr/bin/env bash
set -euo pipefail

MTPLVCAP_BIN="${MTPLVCAP_BIN:-mtplvcap}"
MTPLVCAP_HOST="${MTPLVCAP_HOST:-127.0.0.1}"
MTPLVCAP_PORT="${MTPLVCAP_PORT:-5600}"
MTPLVCAP_FPS="${MTPLVCAP_FPS:-0}"
# VID/PID to disambiguate multiple USB cameras
MTPLVCAP_VENDOR_ID="${MTPLVCAP_VENDOR_ID:-0x04b0}"
MTPLVCAP_PRODUCT_ID="${MTPLVCAP_PRODUCT_ID:-0x0445}"
MTPLVCAP_MAX_RES="${MTPLVCAP_MAX_RES:-false}"
MTPLVCAP_EXTRA_ARGS="${MTPLVCAP_EXTRA_ARGS:-}"

log(){ echo "[mtplvcap_start] $*"; }

if ! command -v "${MTPLVCAP_BIN}" >/dev/null 2>&1; then
  log "mtplvcap not found in PATH. Set MTPLVCAP_BIN or install mtplvcap."
  exit 1
fi

args=()

# Go flags are single-dash; these exist in upstream mtplvcap
args+=("-host" "${MTPLVCAP_HOST}")
args+=("-port" "${MTPLVCAP_PORT}")
args+=("-vendor-id" "${MTPLVCAP_VENDOR_ID}")
args+=("-product-id" "${MTPLVCAP_PRODUCT_ID}")

# Optional tuning
# if [[ "${MTPLVCAP_MAX_RES}" == "true" ]]; then
#   args+=("-max-resolution")
# fi
if [[ "${MTPLVCAP_FPS}" != "0" ]]; then
  args+=("-fps" "${MTPLVCAP_FPS}")
fi

read -r -a extra_args <<<"${MTPLVCAP_EXTRA_ARGS}"

log "Starting mtplvcap on ${MTPLVCAP_HOST}:${MTPLVCAP_PORT} (VID:PID ${MTPLVCAP_VENDOR_ID}:${MTPLVCAP_PRODUCT_ID})..."
log "Final mtplvcap command: ${MTPLVCAP_BIN}" "${args[@]}" "${extra_args[@]}"
# exec "${MTPLVCAP_BIN}" "${args[@]}" "${extra_args[@]}"

mtplvcap -host 127.0.0.1 -port 5600 -vendor-id 0x04b0 -product-id 0x0445