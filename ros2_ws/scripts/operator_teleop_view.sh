#!/usr/bin/env bash
set -euo pipefail

# Start joy + teleop + camera viewers from the operator laptop.
# Run from host; it will exec into the operator container automatically.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-create3_operator}"
CONTAINER_WS="/workspaces/frame"
SELF_IN_CONTAINER="${SELF_IN_CONTAINER:-0}"

# Defaults inside the container
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
TELEOP_CONFIG="${TELEOP_CONFIG:-${CONTAINER_WS}/src/frame_bringup/config/teleop_twist_joy_xbox.yaml}"
MAIN_IMAGE_TOPIC="${MAIN_IMAGE_TOPIC:-/image_rect}"         # Realsense IR remap
NIKON_IMAGE_TOPIC="${NIKON_IMAGE_TOPIC:-/nikon/image_raw}"  # mtplvcap stream
MAIN_WINDOW_TITLE="${MAIN_WINDOW_TITLE:-D421_IR}"
NIKON_WINDOW_TITLE="${NIKON_WINDOW_TITLE:-Nikon_MTPLVCAP}"
TMUX_SESSION="${TMUX_SESSION:-frame_operator}"

require() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

inside_container() {
  [[ "${SELF_IN_CONTAINER}" == "1" ]]
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

exec_in_container() {
  local subcmd="$1"; shift
  if ! container_running; then
    echo "Container ${CONTAINER_NAME} is not running. Start it with scripts/operator_container.sh start."
    exit 1
  fi

  docker exec -it \
    -e SELF_IN_CONTAINER=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e JOY_DEV="${JOY_DEV}" \
    -e TELEOP_CONFIG="${TELEOP_CONFIG}" \
    -e MAIN_IMAGE_TOPIC="${MAIN_IMAGE_TOPIC}" \
    -e NIKON_IMAGE_TOPIC="${NIKON_IMAGE_TOPIC}" \
    -e MAIN_WINDOW_TITLE="${MAIN_WINDOW_TITLE}" \
    -e NIKON_WINDOW_TITLE="${NIKON_WINDOW_TITLE}" \
    -e TMUX_SESSION="${TMUX_SESSION}" \
    "${CONTAINER_NAME}" \
    bash -lc "source /opt/ros/humble/setup.bash && ${CONTAINER_WS}/scripts/operator_teleop_view.sh ${subcmd} $*"
}

tile_viewers() {
  command -v wmctrl >/dev/null 2>&1 || {
    echo "wmctrl not installed; skipping auto-tiling of viewer windows."
    return
  }
  command -v xdpyinfo >/dev/null 2>&1 || {
    echo "xdpyinfo missing; skipping auto-tiling of viewer windows."
    return
  }

  local dims width height half_width
  dims=$(xdpyinfo | awk '/dimensions:/ {print $2}') || return
  width=${dims%x*}
  height=${dims#*x}
  half_width=$((width / 2))

  wmctrl -r "${MAIN_WINDOW_TITLE}" -e "0,0,0,${half_width},${height}" || true
  wmctrl -r "${NIKON_WINDOW_TITLE}" -e "0,${half_width},0,${half_width},${height}" || true
}

start_inside() {
  require tmux

  if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
    echo "tmux session ${TMUX_SESSION} already exists. Attach with \"$0 attach\" or stop with \"$0 stop\"."
    return
  fi

  echo "Starting teleop stack in tmux session: ${TMUX_SESSION}"
  echo "Joy button Y (index 3) toggles recording on the Jetson via joy_record_toggle."

  tmux new-session -d -s "${TMUX_SESSION}" -n joy \
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; ros2 run joy joy_node --ros-args -p dev:=${JOY_DEV} -p deadzone:=0.1 -p autorepeat_rate:=20.0"

  tmux new-window -t "${TMUX_SESSION}" -n teleop \
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; ros2 run teleop_twist_joy teleop_node --ros-args --params-file '${TELEOP_CONFIG}'"

  tmux new-window -t "${TMUX_SESSION}" -n view-d421 \
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; ros2 run image_view image_view --ros-args -p image:=${MAIN_IMAGE_TOPIC} -p window_name:=${MAIN_WINDOW_TITLE}"

  tmux new-window -t "${TMUX_SESSION}" -n view-nikon \
    "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; ros2 run image_view image_view --ros-args -p image:=${NIKON_IMAGE_TOPIC} -p window_name:=${NIKON_WINDOW_TITLE}"

  tmux select-window -t "${TMUX_SESSION}:0"

  # Give the viewer windows a moment to appear before tiling.
  sleep 2
  tile_viewers

  echo
  echo "Attach to the tmux session for logs: $0 attach"
}

attach_inside() {
  tmux attach -t "${TMUX_SESSION}"
}

stop_inside() {
  if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
    tmux kill-session -t "${TMUX_SESSION}"
    echo "Stopped tmux session ${TMUX_SESSION}."
  else
    echo "No tmux session named ${TMUX_SESSION}."
  fi
}

usage() {
  cat <<EOF
Usage: $0 {start|attach|stop}
  start  - run joy, teleop_twist_joy, and two image viewers (D421 + Nikon)
  attach - attach to the tmux session to see logs
  stop   - kill the tmux session

Environment:
  CONTAINER_NAME     (default: ${CONTAINER_NAME})
  ROS_DOMAIN_ID      (default: ${ROS_DOMAIN_ID})
  JOY_DEV            (default: ${JOY_DEV})
  TELEOP_CONFIG      (default: ${TELEOP_CONFIG})
  MAIN_IMAGE_TOPIC   (default: ${MAIN_IMAGE_TOPIC})
  NIKON_IMAGE_TOPIC  (default: ${NIKON_IMAGE_TOPIC})
  MAIN_WINDOW_TITLE  (default: ${MAIN_WINDOW_TITLE})
  NIKON_WINDOW_TITLE (default: ${NIKON_WINDOW_TITLE})
  TMUX_SESSION       (default: ${TMUX_SESSION})
EOF
}

# Dispatch: if we're on the host, exec into the container first.
if ! inside_container; then
  case "${1:-}" in
    start|attach|stop) exec_in_container "internal-${1}" ;;
    *) usage; exit 1 ;;
  esac
  exit 0
fi

# Inside container dispatch
case "${1:-}" in
  internal-start)  start_inside ;;
  internal-attach) attach_inside ;;
  internal-stop)   stop_inside ;;
  *) usage; exit 1 ;;
esac
