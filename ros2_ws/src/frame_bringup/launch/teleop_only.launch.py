import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('frame_bringup')
    default_params = PathJoinSubstitution(
        [bringup_share, 'config', 'teleop_twist_joy_xbox.yaml']
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file.',
    )
    params_file_arg = DeclareLaunchArgument(
        'teleop_params',
        default_value=default_params,
        description='teleop_twist_joy params file to load.',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[LaunchConfiguration('teleop_params')],
    )

    return LaunchDescription(
        [
            joy_dev_arg,
            params_file_arg,
            joy_node,
            teleop_node,
        ]
    )
