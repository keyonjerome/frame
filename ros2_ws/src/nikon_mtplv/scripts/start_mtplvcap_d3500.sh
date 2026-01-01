#!/usr/bin/env bash
set -euo pipefail

MTPLVCAP_BIN="${MTPLVCAP_BIN:-mtplvcap}"
MTPLVCAP_PORT="${MTPLVCAP_PORT:-5600}"
MTPLVCAP_RESOLUTION="${MTPLVCAP_RESOLUTION:-1920x1080}"
MTPLVCAP_FPS="${MTPLVCAP_FPS:-30}"
MTPLVCAP_EXTRA_ARGS="${MTPLVCAP_EXTRA_ARGS:-}"

log(){ echo "[mtplvcap_start] $*"; }

if ! command -v "${MTPLVCAP_BIN}" >/dev/null 2>&1; then
  log "mtplvcap not found in PATH. Set MTPLVCAP_BIN or install mtplvcap."
  exit 1
fi

help_output="$(${MTPLVCAP_BIN} --help 2>&1 || true)"
args=()

if grep -qE -- '(^|\s)(-p|--port)(\s|$)' <<<"${help_output}"; then
  args+=("--port" "${MTPLVCAP_PORT}")
else
  log "No port flag detected in mtplvcap help; set MTPLVCAP_EXTRA_ARGS if needed."
fi

if grep -qE -- '(^|\s)(--resolution|-r)(\s|$)' <<<"${help_output}"; then
  args+=("--resolution" "${MTPLVCAP_RESOLUTION}")
elif grep -qE -- '(^|\s)(--size|-s)(\s|$)' <<<"${help_output}"; then
  args+=("--size" "${MTPLVCAP_RESOLUTION}")
else
  log "No resolution flag detected; assuming mtplvcap defaults to max resolution."
fi

if grep -qE -- '(^|\s)(--fps|-f)(\s|$)' <<<"${help_output}"; then
  args+=("--fps" "${MTPLVCAP_FPS}")
fi

read -r -a extra_args <<<"${MTPLVCAP_EXTRA_ARGS}"

log "Starting mtplvcap on port ${MTPLVCAP_PORT} at ${MTPLVCAP_RESOLUTION}..."
exec "${MTPLVCAP_BIN}" "${args[@]}" "${extra_args[@]}"
