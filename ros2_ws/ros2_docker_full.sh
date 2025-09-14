#!/usr/bin/env bash
set -euo pipefail

NAME="frame_ros2"
IMAGE="osrf/ros:humble-desktop"

# Allow Docker to talk to your X server (safe for local:docker)
xhost +local:docker >/dev/null 2>&1 || true

# Create the container once if it doesn't exist yet
if ! docker inspect "$NAME" >/dev/null 2>&1; then
  echo "Creating persistent ROS 2 container: $NAME"
  docker create \
    --name "$NAME" \
    --hostname "$NAME" \
    --net=host \
    --ipc=host \
    -e DISPLAY \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$HOME/code/frame:/root/frame" \
    -v "$HOME/.ssh:/root/.ssh:ro" \
    "$IMAGE" \
    bash -lc 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "cd /root/frame" >> ~/.bashrc && exec bash'
fi

# Start (or restart) and attach to the same container
echo "Starting and attaching to $NAME …"
docker start "$NAME" >/dev/null
docker exec -it "$NAME" bash
