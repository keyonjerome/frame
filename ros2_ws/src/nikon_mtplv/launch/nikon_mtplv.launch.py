from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_scripts_dir = PathJoinSubstitution(
        [FindPackageShare('nikon_mtplv'), 'scripts']
    )

    scripts_dir_arg = DeclareLaunchArgument(
        'scripts_dir',
        default_value=default_scripts_dir,
        description='Directory containing nikon_usb_recover.sh and start_mtplvcap_d3500.sh',
    )
    mtplvcap_port_arg = DeclareLaunchArgument(
        'mtplvcap_port',
        default_value='5600',
        description='TCP port for mtplvcap to bind',
    )
    stream_url_arg = DeclareLaunchArgument(
        'stream_url',
        default_value=PythonExpression(
            ["'http://127.0.0.1:' + str(", LaunchConfiguration('mtplvcap_port'), ") + '/mjpeg'"]
        ),
        description='URL to the mtplvcap MJPEG stream (http://127.0.0.1:<mtplvcap_port>/mjpeg)',
    )

    # Recovery is disabled for now; we still use an ExecuteProcess stub to satisfy event handlers.
    recover_action = ExecuteProcess(cmd=['/bin/true'], output='screen')
    # To re-enable, replace the stub above with:
    # ExecuteProcess(
    #     cmd=[PathJoinSubstitution([LaunchConfiguration('scripts_dir'), 'nikon_usb_recover.sh'])],
    #     output='screen',
    # )

    mtplvcap_action = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [LaunchConfiguration('scripts_dir'), 'start_mtplvcap_d3500.sh']
            )
        ],
        env={'MTPLVCAP_PORT': LaunchConfiguration('mtplvcap_port')},
        output='screen',
    )

    stream_node = Node(
        package='nikon_mtplv',
        executable='mtplvcap_stream',
        output='screen',
        parameters=[{'stream_url': LaunchConfiguration('stream_url')}],
    )

    start_mtplvcap_on_recover_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=recover_action,
            on_exit=[TimerAction(period=5.0, actions=[mtplvcap_action])],
        )
    )

    start_node_on_mtplvcap_start = RegisterEventHandler(
        OnProcessStart(
            target_action=mtplvcap_action,
            on_start=[TimerAction(period=10.0, actions=[stream_node])],
        )
    )

    return LaunchDescription(
        [
            scripts_dir_arg,
            mtplvcap_port_arg,
            stream_url_arg,
            recover_action,
            start_mtplvcap_on_recover_exit,
            start_node_on_mtplvcap_start,
        ]
    )
