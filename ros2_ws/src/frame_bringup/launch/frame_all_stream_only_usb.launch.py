import os
from pathlib import Path


def _default_video_dir() -> str:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return str(parent / 'videos')
    return os.path.join(os.path.expanduser('~'), 'videos')


def _default_gst_video_dir() -> str:
    override = os.environ.get('FRAME_GST_OUTPUT_DIR', '').strip()
    if override:
        return override

    video_dir = _default_video_dir()
    container_ws = os.environ.get('ISAAC_ROS_WS', '/workspaces/isaac_ros-dev')
    host_ws = os.environ.get(
        'FRAME_HOST_ISAAC_ROS_WS',
        '/mnt/nova_ssd/workspaces/isaac_ros-dev',
    )
    container_ws = container_ws.rstrip('/')
    host_ws = host_ws.rstrip('/')
    if video_dir == container_ws or video_dir.startswith(container_ws + '/'):
        return host_ws + video_dir[len(container_ws):]
    return video_dir


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
    servo_share = get_package_share_directory('frame_servo_control')
    default_teleop_params = PathJoinSubstitution(
        [bringup_share, 'config', 'teleop_twist_joy_xbox.yaml']
    )
    default_velocity_smoother_params = PathJoinSubstitution(
        [bringup_share, 'config', 'velocity_smoother.yaml']
    )
    default_servo_params = PathJoinSubstitution(
        [servo_share, 'config', 'dual_servo_control.yaml']
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
    gst_output_dir_arg = DeclareLaunchArgument(
        'gst_output_dir',
        default_value=_default_gst_video_dir(),
        description='Host-visible directory passed to gst_recording_daemon START.',
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
    gst_socket_path_arg = DeclareLaunchArgument(
        'gst_socket_path',
        default_value=os.environ.get(
            'GST_RECORDING_DAEMON_SOCKET',
            '/tmp/filmer_recorder_test.sock',
        ),
        description='Unix socket path for gst_recording_daemon.',
    )
    gst_command_timeout_sec_arg = DeclareLaunchArgument(
        'gst_command_timeout_sec',
        default_value='15.0',
        description='Timeout for gst_recording_daemon commands.',
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
    servo_params_arg = DeclareLaunchArgument(
        'servo_params',
        default_value=default_servo_params,
        description='Dual servo controller params file to load.',
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

    gst_communicator_node = Node(
        package='gst_communicator',
        executable='gst_communicator',
        name='gst_communicator',
        output='screen',
        parameters=[{
            'record_button': LaunchConfiguration('record_button'),
            'socket_path': LaunchConfiguration('gst_socket_path'),
            'output_dir': LaunchConfiguration('gst_output_dir'),
            'command_timeout_sec': LaunchConfiguration('gst_command_timeout_sec'),
        }],
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[LaunchConfiguration('teleop_params')],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_in'))],
    )
    servo_joy_node = Node(
        package='frame_servo_control',
        executable='dual_servo_velocity_control',
        name='dual_servo_control',
        output='screen',
        parameters=[LaunchConfiguration('servo_params')],
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
            gst_output_dir_arg,
            storage_id_arg,
            usb_record_topic_arg,
            gst_socket_path_arg,
            gst_command_timeout_sec_arg,
            teleop_params_arg,
            velocity_smoother_params_arg,
            servo_params_arg,
            cmd_vel_in_arg,
            cmd_vel_out_arg,
            d421_launch,
            record_node,
            gst_communicator_node,
            teleop_node,
            servo_joy_node,
            velocity_smoother_node,
            velocity_smoother_manager_node,
        ]
    )
