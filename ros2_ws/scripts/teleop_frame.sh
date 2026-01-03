#!/usr/bin/env bash
set -euo pipefail

# ---- Config you might tweak ----
CONTAINER_NAME="frame_teleop"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
IMAGE="ros:humble"

# Optional: keep a teleop config on the host and mount it in
HOST_TELEOP_YAML="${HOME}/.ros/teleop.yaml"
CONTAINER_TELEOP_YAML="/root/.ros/teleop.yaml"

# ---- Helpers ----
container_running() {
  docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

cmd_start() {
  if container_running; then
    echo "Container already running: ${CONTAINER_NAME}"
    exit 0
  fi

  if container_exists; then
    echo "Removing existing (stopped) container: ${CONTAINER_NAME}"
    docker rm "${CONTAINER_NAME}" >/dev/null
  fi

  # Ensure host teleop config dir exists (optional)
  mkdir -p "${HOME}/.ros"

  # Build docker args
  DOCKER_ARGS=(
    -d
    --name "${CONTAINER_NAME}"
    --network host
    --privileged
    -v /dev/input:/dev/input
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  )

  # Mount teleop config if present
  if [[ -f "${HOST_TELEOP_YAML}" ]]; then
    DOCKER_ARGS+=( -v "${HOST_TELEOP_YAML}:${CONTAINER_TELEOP_YAML}:ro" )
    echo "Mounting teleop config: ${HOST_TELEOP_YAML} -> ${CONTAINER_TELEOP_YAML}"
  else
    echo "No ${HOST_TELEOP_YAML} found (optional). You can create it later."
  fi

  echo "Starting container: ${CONTAINER_NAME} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
  docker run "${DOCKER_ARGS[@]}" "${IMAGE}" bash -lc "
    set -e
    apt-get update -qq
    apt-get install -y -qq ros-humble-joy ros-humble-teleop-twist-joy joystick
    mkdir -p /root/.ros
    echo 'Ready. Exec into this container to run joy/teleop.'
    tail -f /dev/null
  " >/dev/null

  echo "Started: ${CONTAINER_NAME}"
  echo
  echo "Now run these in TWO SEPARATE host terminals:"
  echo "  ${0} joy"
  echo "  ${0} teleop"
}

cmd_shell() {
  if ! container_running; then
    echo "Container not running. Start it first: ${0} start"
    exit 1
  fi
  docker exec -it "${CONTAINER_NAME}" bash
}

cmd_joy() {
  if ! container_running; then
    echo "Container not running. Start it first: ${0} start"
    exit 1
  fi
  docker exec -it "${CONTAINER_NAME}" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    echo 'Joystick devices:'
    ls -l /dev/input/js* 2>/dev/null || true
    echo 'Starting joy_node...'
    ros2 run joy joy_node
  "
}

cmd_teleop() {
  if ! container_running; then
    echo "Container not running. Start it first: ${0} start"
    exit 1
  fi

  # If you mounted a config, use it; otherwise run with defaults
  docker exec -it "${CONTAINER_NAME}" bash -lc "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    if [[ -f '${CONTAINER_TELEOP_YAML}' ]]; then
      echo 'Starting teleop_twist_joy with config: ${CONTAINER_TELEOP_YAML}'
      ros2 run teleop_twist_joy teleop_node --ros-args --params-file '${CONTAINER_TELEOP_YAML}'
    else
      echo 'No teleop.yaml mounted; starting teleop_twist_joy with defaults'
      ros2 run teleop_twist_joy teleop_node
    fi
  "
}

cmd_stop() {
  if container_running; then
    echo "Stopping: ${CONTAINER_NAME}"
    docker stop "${CONTAINER_NAME}" >/dev/null
  fi
  if container_exists; then
    echo "Removing: ${CONTAINER_NAME}"
    docker rm "${CONTAINER_NAME}" >/dev/null
  fi
  echo "Stopped."
}

cmd_status() {
  if container_running; then
    echo "RUNNING: ${CONTAINER_NAME}"
    docker ps --filter "name=${CONTAINER_NAME}"
  elif container_exists; then
    echo "EXISTS (stopped): ${CONTAINER_NAME}"
    docker ps -a --filter "name=${CONTAINER_NAME}"
  else
    echo "NOT PRESENT: ${CONTAINER_NAME}"
  fi
}

usage() {
  cat <<EOF
Usage:
  $0 start    # sta
