#!/usr/bin/env bash
set -euo pipefail

# Handy wrapper to launch the full bringup (D421 + Nikon + teleop + recording).
# Override RECORD_DIR or VIDEO_DIR to change outputs; pass extra launch args via CLI.

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RECORD_DIR="${RECORD_DIR:-${WS_DIR}/../rosbags}"
VIDEO_DIR="${VIDEO_DIR:-${WS_DIR}/../videos}"

echo "Launching frame_bringup with record_output_dir=${RECORD_DIR}, video_output_dir=${VIDEO_DIR}"
ros2 launch frame_bringup frame_all.launch.py \
  record_output_dir:="${RECORD_DIR}" \
  video_output_dir:="${VIDEO_DIR}" \
  "$@"
