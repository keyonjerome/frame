# Launch Instructions

## Hardware
- NVIDIA Jetson ORIN Nano Super running Jetpack 6.2
- Intel Realsense D421 camera connected over USB 3.2 (3.1-enabled cable to the USB A port)
- 512 GB SSD for NVIDIA Jetson Orin Nano

## Software

### Orin Nano
Required tutorials:
- https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html#
    - Jetson Setup: https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html#jetson-platforms
    - Developer Environment Setup: https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html
        - If you follow their advice here and create the workspace on the `/mnt` directory, make sure to give yourself permission to it with `chmod` to make development easier.
    - Realsense Setup (note: we build the Realsense SDK from source in our *modified* Dockerfile, separately): https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/sensors/realsense_setup.html

## Running The Program
1. From the ISAAC ROS developer environment setup, you should have `isaac_ros_common` cloned to your `$ISAAC_ROS_WS`. Navigate into the folder, `isaac_ros_common/docker` and paste the `Dockerfile.realsense` from this repo into it, replacing the current one. This will change the `realsense-ros` version that's being used, and `apt-get install` required packages for object detection, in addition to the things done in the base Dockerfile.
2. From here, we follow the instructions to get YOLOV8 ready: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_object_detection/isaac_ros_yolov8/index.html
    - Note: The Dockerfile will install the `isaac-ros` packages for you.
    - Note: Change yolov8s to yolov8n in all download and launch scripts.
3. Clone this repository into our `$ISAAC_ROS_WS`, if you haven't already, `cd` to it, and `colcon build`.
3. Run the program with several shells in your Docker container, each having run `source ./install/setup.bash` at your `frame` repository root to get its packages :    
    1. Start the Realsense Camera:  `ros2 launch ir_to_rgb_remap d421_launch.py`
    2. Start the ISAAC ROS YOLOV8 node with yolov8-nano: `ros2 launch ir_to_rgb_remap isaac_ros_yolo_launch.py  model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8n.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8n.plan`
    3. Run the visualizer: `ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py`
    4. View the output: `ros2 run rqt_image_view rqt_image_view /yolov8_processed_image`
