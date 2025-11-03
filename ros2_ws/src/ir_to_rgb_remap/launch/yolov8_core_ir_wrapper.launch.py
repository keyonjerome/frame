from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # User-settable model paths
    model = DeclareLaunchArgument(
        'model_file_path',
        default_value=os.path.join(
            os.environ.get('ISAAC_ROS_WS', '/workspaces/isaac_ros-dev'),
            'isaac_ros_assets/models/yolov8/yolov8n.onnx'
        )
    )
    engine = DeclareLaunchArgument(
        'engine_file_path',
        default_value=os.path.join(
            os.environ.get('ISAAC_ROS_WS', '/workspaces/isaac_ros-dev'),
            'isaac_ros_assets/models/yolov8/yolov8n.plan'
        )
    )

    # What the examples used to provide; pick something reasonable
    # (YOLO doesn’t strictly need this; it reads camera_info from the topic)
    camera_resolution_width  = DeclareLaunchArgument('camera_resolution_width',  default_value='640')
    camera_resolution_height = DeclareLaunchArgument('camera_resolution_height', default_value='480')
    camera_frame             = DeclareLaunchArgument('camera_frame',             default_value='camera_color_optical_frame')
    focal_fx                 = DeclareLaunchArgument('f_x',                      default_value='380.0')
    focal_fy                 = DeclareLaunchArgument('f_y',                      default_value='380.0')

    # Include the package core launch and pass topics + model paths (+ any required interface bits)
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isaac_ros_yolov8'),
                         'launch', 'isaac_ros_yolov8_core.launch.py')),
        launch_arguments={
            'input_image_topic':        '/image_rect',
            'input_camera_info_topic':  '/camera_info_rect',
            'model_file_path':          LaunchConfiguration('model_file_path'),
            'engine_file_path':         LaunchConfiguration('engine_file_path'),
            'network_image_width':      '640',
            'network_image_height':     '640',
            'max_batch_size':           '1',

            # only if your core launch exposes these (some versions ignore them):
            'camera_resolution_width':  LaunchConfiguration('camera_resolution_width'),
            'camera_resolution_height': LaunchConfiguration('camera_resolution_height'),
            'camera_frame':             LaunchConfiguration('camera_frame'),
            'f_x':                      LaunchConfiguration('f_x'),
            'f_y':                      LaunchConfiguration('f_y'),
        }.items()
    )

    return LaunchDescription([
        model, engine,
        camera_resolution_width, camera_resolution_height, camera_frame, focal_fx, focal_fy,
        core_launch
    ])
