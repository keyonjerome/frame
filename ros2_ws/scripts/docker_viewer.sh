# allow X11 from containers (once per boot/session)
xhost +local:docker

# run a Humble desktop container and open an image window
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e ROS_DOMAIN_ID=42             # <--- set to match the robot (replace 42)
  osrf/ros:humble-desktop \
  bash -lc "ros2 topic list && \
            ros2 run image_view image_view --ros-args -r image:=/dancer_detector/overlay"