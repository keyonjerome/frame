#!/usr/bin/env bash
set -e

# ---- ROS 2 env (adjust if your paths differ) ----
source /opt/ros/humble/setup.bash
# If you build your own workspace:
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

# ---- Networking knobs for multi-device ROS 2 ----
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
# Optionally pick a specific RMW (uncomment one):
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---- Launch everything ----
exec ros2 launch dancer_detector follow_person.launch.py
