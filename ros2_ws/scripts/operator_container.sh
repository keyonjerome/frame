#!/usr/bin/env bash
set -euo pipefail

# Start/attach a persistent ROS 2 Humble teleop container for Create 3 operation.
# - Builds docker/Dockerfile.operator if the image is missing.
# - Runs with host networking and joystick/X11 access so GUI viewers work.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKERFILE="${WS_DIR}/docker/Dockerfile.operator"

IMAGE_NAME="${IMAGE_NAME:-create3-operator:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-create3_operator}"
CONTAINER_WS="/workspaces/frame"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

require() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

image_present() {
  docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1
}

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

build_image() {
  echo "Building ${IMAGE_NAME} from ${DOCKERFILE}..."
  docker build -t "${IMAGE_NAME}" -f "${DOCKERFILE}" "${WS_DIR}"
}

start_container() {
  require docker
  command -v xhost >/dev/null 2>&1 && xhost +local:docker >/dev/null || true

  image_present || build_image

  if container_running; then
    echo "Container already running: ${CONTAINER_NAME}"
    exit 0
  fi

  if container_exists; then
    echo "Starting existing container ${CONTAINER_NAME} (use \"$0 stop\" to recreate)."
    docker start "${CONTAINER_NAME}" >/dev/null
    exit 0
  fi

  XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

  echo "Launching ${CONTAINER_NAME} with ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, joystick=${JOY_DEV}"
  docker run -d \
    --name "${CONTAINER_NAME}" \
    --restart unless-stopped \
    --network host \
    --ipc host \
    --privileged \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTH}:${XAUTH}:ro" \
    -v "${WS_DIR}:${CONTAINER_WS}:rw" \
    -v /dev/input:/dev/input:rw \
    -v /run/udev:/run/udev:ro \
    "${IMAGE_NAME}" \
    bash -lc "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export JOY_DEV=${JOY_DEV}; tail -f /dev/null" \
    >/dev/null

  echo "Container started. Attach with: $0 shell"
}

shell_container() {
  if ! container_running; then
    echo "Container not running. Start it first with \"$0 start\"."
    exit 1
  fi
  docker exec -it \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e JOY_DEV="${JOY_DEV}" \
    "${CONTAINER_NAME}" bash
}

stop_container() {
  if container_running; then
    echo "Stopping ${CONTAINER_NAME}..."
    docker stop "${CONTAINER_NAME}" >/dev/null
  fi
  if container_exists; then
    echo "Removing ${CONTAINER_NAME}..."
    docker rm "${CONTAINER_NAME}" >/dev/null
  fi
  echo "Stopped."
}

status_container() {
  if container_running; then
    echo "RUNNING: ${CONTAINER_NAME}"
    docker ps --filter "name=${CONTAINER_NAME}"
  elif container_exists; then
    echo "STOPPED: ${CONTAINER_NAME}"
    docker ps -a --filter "name=${CONTAINER_NAME}"
  else
    echo "NOT PRESENT: ${CONTAINER_NAME}"
  fi
}

usage() {
  cat <<EOF
Usage: $0 {start|shell|status|stop}
  start  - build image if needed and start the persistent operator container
  shell  - attach an interactive shell to the running container
  status - show container status
  stop   - stop and remove the container

Environment:
  IMAGE_NAME      (default: ${IMAGE_NAME})
  CONTAINER_NAME  (default: ${CONTAINER_NAME})
  ROS_DOMAIN_ID   (default: ${ROS_DOMAIN_ID})
  JOY_DEV         (default: ${JOY_DEV})
EOF
}

case "${1:-}" in
  start)  start_container ;;
  shell)  shell_container ;;
  status) status_container ;;
  stop)   stop_container ;;
  *)      usage; exit 1 ;;
esac
