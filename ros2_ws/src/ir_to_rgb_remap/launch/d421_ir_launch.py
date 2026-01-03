# SPDX-FileCopyrightText: NVIDIA
# SPDX-License-Identifier: Apache-2.0
#
# Launch RealSense D421 with IR + Depth, remap IR -> /image_rect and camera_info -> /camera_info_rect
# so the Isaac ROS yolov8 fragment can consume it unchanged.

import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    camera_name = 'camera'  # keep default so topic prefixes stay /camera/...
    # RealSense params for D421 (no RGB)
    rs_params = {
        'enable_infra1': True,
        'enable_infra2': False,
        'enable_depth': True,
        'enable_color': False,
        # modest resolution to keep GPU mem low
        'infra1_width': 640,
        'infra1_height': 480,
        'infra1_fps': 30,
        'depth_width': 640,
        'depth_height': 480,
        'depth_fps': 30,
        # don’t align depth to color (there is no color); we’ll use depth/image_rect_raw
        'align_depth': False,
        # keep things light
        'pointcloud.enable': False,
        # 0 == none, 1 == copy, 2 == linear_interpolation
        'unite_imu_method': 0,
    }

    # Container that will host RealSense + depth metric conversion
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        namespace='',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            # RealSense camera node
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name=camera_name,
                namespace='',
                parameters=[rs_params],
                # Remap IR→ generic inputs expected by Isaac encoders:
                #   /image_rect (image) and /camera_info_rect (camera info)
                # remappings=[
                #     (f'/{camera_name}/infra1/image_rect_raw', 'image_rect'),
                #     (f'/{camera_name}/infra1/camera_info',     'camera_info_rect'),
                # ],
            ),
            # Convert depth (uint16 mm) → float32 meters on topic /depth
            # Depth metric conversion is optional; disable to avoid missing plugin errors
            # ComposableNode(
            #     package='isaac_ros_depth_image_proc',
            #     plugin='nvidia::isaac_ros::depth_image_proc::ConvertMetricNode',
            #     name='convert_metric',
            #     remappings=[
            #         # Use the unaligned rectified depth image (not aligned_to_color)
            #         ( 'image_raw', f'/{camera_name}/depth/image_rect_raw'),
            #         ( 'image',     'depth'),
            #     ],
            # ),
            # 4) The IR→RGB remap node (standalone executable)
            #    - Subscribes to /<camera>/infra1/image_rect_raw + camera_info
            #    - Publishes /image_rect (rgb8) + /camera_info_rect
            ComposableNode(
                package='ir_to_rgb_remap',
                plugin='ir_to_rgb_remap::IrToRgbRemapNode',
                name='ir_to_rgb_remap',
                parameters=[{
                    'in_image':        f'/{camera_name}/infra1/image_rect_raw',
                    'in_camera_info':  f'/{camera_name}/infra1/camera_info',
                    'out_image':       '/image_rect',
                    'out_camera_info': '/camera_info_rect',
                }]
            )
        ],
    )

    # # 3) YOLOv8 decoder (component)
    # #    Adjust input tensor/topic names to your graph; default is "/tensor_sub" in NVIDIA examples.
    # yolo_node =        ComposableNode(
    #             package='isaac_ros_yolov8',
    #             plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
    #             name='yolo_v8_decoder',
    #             # If your decoder expects different topic names, remap here:
    #             # remappings=[ ('tensor', '/tensor_sub'), ('detections', '/yolov8/detections') ],
    #             parameters=[{
    #                 # set any decoder params you need, e.g. score_threshold, nms_iou_threshold, etc.
    #                 # 'score_threshold': 0.25
    #             }],
    #         )




    # Load the rest of the YOLOv8 graph into Isaac’s examples container as usual
    # Equivalent to: launch_fragments:=yolov8
    # yolov8_nodes = LoadComposableNodes(
    #     target_container='/isaac_ros_examples/container',  # this container is created by the examples runner
    #     composable_node_descriptions=[]
    # )
    # NOTE: You typically start the examples container via the standard examples launcher.
    # If you prefer a single-file launch that also instantiates yolov8 components here,
    # you can add those ComposableNode descriptions above instead of relying on the examples runner.

    return LaunchDescription([
        realsense_container,
        # If you normally launch yolov8 via the examples meta-launch, start that in another terminal:
        #   ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
        #     launch_fragments:=yolov8 \
        #     model_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/yolov8n.onnx \
        #     engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/yolov8n.plan
        #
        # Or, if your workflow already creates '/isaac_ros_examples/container', keep yolov8_nodes here:
        # yolov8_nodes,
    ])
