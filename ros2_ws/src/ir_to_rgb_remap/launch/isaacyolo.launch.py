# save as: launch/yolov8_minimal_core_free.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    model = os.environ.get('ISAAC_ROS_WS', '/workspaces/isaac_ros-dev') + '/isaac_ros_assets/models/yolov8/yolov8n.onnx'
    engine = os.environ.get('ISAAC_ROS_WS', '/workspaces/isaac_ros-dev') + '/isaac_ros_assets/models/yolov8/yolov8n.plan'

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='isaac_yolov8_container',
        namespace='',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            # Encoder (image -> tensor → Triton)
            ComposableNode(
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8EncoderNode',
                name='yolov8_encoder',
                parameters=[{
                    'model_file_path': model,
                    'engine_file_path': engine,
                    'network_image_width': 640,
                    'network_image_height': 640,
                    'max_batch_size': 1,
                }],
                remappings=[
                    ('image', '/image_rect'),
                    ('camera_info', '/camera_info_rect'),
                    # These are the usual defaults; leave them unless you’ve changed them elsewhere:
                    ('tensor', '/tensor_sub'),
                ],
            ),
            # Decoder (TensorRT outputs -> Detection2DArray)
            ComposableNode(
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                name='yolov8_decoder',
                remappings=[
                    ('tensor', '/tensor_sub'),
                    ('detections', '/yolov8/detections'),
                ],
                parameters=[{
                    # optional thresholds if exposed by your build:
                    # 'score_threshold': 0.25,
                    # 'nms_iou_threshold': 0.5,
                }],
            ),
        ],
    )
    return LaunchDescription([container])
