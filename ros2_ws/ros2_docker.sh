#!/usr/bin/env bash
set -e

# Make sure DISPLAY and SSH agent are ready
if [ -z "$SSH_AUTH_SOCK" ]; then
    echo "No SSH agent detected. Starting one..."
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_ed25519 2>/dev/null || true
    ssh-add ~/.ssh/id_rsa 2>/dev/null || true
fi

# Allow Docker to talk to your X server
xhost +local:docker >/dev/null

# Run the container
docker run -it --rm \
    --net=host \
    --ipc=host \
    --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/code/frame:/root/frame" \
    --volume="$SSH_AUTH_SOCK:/ssh-agent" \
    --env SSH_AUTH_SOCK=/ssh-agent \
    osrf/ros:humble-desktop \
    bash -c "source /opt/ros/humble/setup.bash && cd /root/frame && exec bash"
