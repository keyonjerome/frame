import os
from pathlib import Path


def _default_video_dir() -> str:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return str(parent / 'videos')
    return os.path.join(os.path.expanduser('~'), 'videos')


def _default_rosbag_dir() -> str:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return str(parent / 'rosbags')
    return os.path.join(os.path.expanduser('~'), 'rosbags')

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ir_share = get_package_share_directory('ir_to_rgb_remap')
    bringup_share = get_package_share_directory('frame_bringup')
    default_teleop_params = PathJoinSubstitution(
        [bringup_share, 'config', 'teleop_twist_joy_xbox.yaml']
    )
    default_velocity_smoother_params = PathJoinSubstitution(
        [bringup_share, 'config', 'velocity_smoother.yaml']
    )

    use_rqt_arg = DeclareLaunchArgument(
        'use_rqt',
        default_value='true',
        description='Launch rqt_image_view to preview a camera stream.',
    )
    rqt_image_topic_arg = DeclareLaunchArgument(
        'rqt_image_topic',
        default_value='/image_rect',
        description='Image topic to show in rqt_image_view.',
    )
    record_button_arg = DeclareLaunchArgument(
        'record_button',
        default_value='3',
        description='Joy button index to start/stop recording.',
    )
    record_output_dir_arg = DeclareLaunchArgument(
        'record_output_dir',
        default_value=_default_rosbag_dir(),
        description='Directory to store recorded rosbag files.',
    )
    record_topics_arg = DeclareLaunchArgument(
        'record_topics',
        default_value='/image_rect,/camera_info_rect,/camera/depth/image_rect_raw',
        description='Comma-separated list of topics to record.',
    )
    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix',
        default_value='dual_cam',
        description='Filename prefix for recordings.',
    )
    video_output_dir_arg = DeclareLaunchArgument(
        'video_output_dir',
        default_value=_default_video_dir(),
        description='Directory to store MP4 exports for the web UI.',
    )
    storage_id_arg = DeclareLaunchArgument(
        'storage_id',
        default_value='',
        description='Optional rosbag2 storage plugin (e.g., mcap).',
    )
    usb_record_topic_arg = DeclareLaunchArgument(
        'usb_record_topic',
        default_value='/usb_cam_stream/record',
        description='Topic to toggle USB recording.',
    )
    stream_url_arg = DeclareLaunchArgument(
        'stream_url',
        default_value='http://127.0.0.1:5600/mjpeg',
        description='URL to the already-running mtplvcap MJPEG stream.',
    )
    teleop_params_arg = DeclareLaunchArgument(
        'teleop_params',
        default_value=default_teleop_params,
        description='teleop_twist_joy params file to load.',
    )
    velocity_smoother_params_arg = DeclareLaunchArgument(
        'velocity_smoother_params',
        default_value=default_velocity_smoother_params,
        description='Velocity smoother params file to load.',
    )
    cmd_vel_in_arg = DeclareLaunchArgument(
        'cmd_vel_in',
        default_value='cmd_vel_raw',
        description='Input cmd_vel topic for the velocity smoother.',
    )
    cmd_vel_out_arg = DeclareLaunchArgument(
        'cmd_vel_out',
        default_value='cmd_vel',
        description='Output cmd_vel topic for the robot base.',
    )

    d421_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ir_share, 'launch', 'd421_ir_launch.py')
        )
    )

    record_node = Node(
        package='frame_bringup',
        executable='joy_record_toggle',
        name='joy_record_toggle',
        output='screen',
        parameters=[{
            'record_button': LaunchConfiguration('record_button'),
            'record_topics': LaunchConfiguration('record_topics'),
            'output_dir': LaunchConfiguration('record_output_dir'),
            'video_output_dir': LaunchConfiguration('video_output_dir'),
            'bag_prefix': LaunchConfiguration('bag_prefix'),
            'storage_id': LaunchConfiguration('storage_id'),
            'usb_record_topic': LaunchConfiguration('usb_record_topic'),
        }],
    )

    stream_node = Node(
        package='nikon_mtplv',
        executable='mtplvcap_stream',
        name='mtplvcap_stream',
        output='screen',
        parameters=[{'stream_url': LaunchConfiguration('stream_url')}],
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[LaunchConfiguration('teleop_params')],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_in'))],
    )
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('velocity_smoother_params')],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_in')),
            ('cmd_vel_smoothed', LaunchConfiguration('cmd_vel_out')),
        ],
    )
    velocity_smoother_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='velocity_smoother_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['velocity_smoother'],
        }],
    )

    return LaunchDescription(
        [
            use_rqt_arg,
            rqt_image_topic_arg,
            record_button_arg,
            record_output_dir_arg,
            record_topics_arg,
            bag_prefix_arg,
            video_output_dir_arg,
            storage_id_arg,
            usb_record_topic_arg,
            stream_url_arg,
            teleop_params_arg,
            velocity_smoother_params_arg,
            cmd_vel_in_arg,
            cmd_vel_out_arg,
            d421_launch,
            record_node,
            stream_node,
            teleop_node,
            velocity_smoother_node,
            velocity_smoother_manager_node,
        ]
    )
