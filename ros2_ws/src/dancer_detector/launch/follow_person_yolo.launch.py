from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Tweak to your setup
    img_w = 640
    hfov_deg = 69.4
    target_dist = 1.75

    yolo = Node(
        package='dancer_detector',
        executable='yolo_ir_realsense',
        name='yolo_ir_realsense',
        output='screen',
        parameters=[{
            'weights_path':  '~//models/yolov3/yolov3.weights',
            'cfg_path':      '~//models/yolov3/yolov3.cfg',
            'names_path':    '~//models/yolov3/coco.names',
            'show_window':   True,
        }],
    )

    # Gate converts + allows start/stop via /dancer_detector/{start,stop}
    gate = Node(
        package='dancer_detector',
        executable='centroid_gate_to_centerdepth',
        name='centroid_gate_to_centerdepth',
        output='screen',
        parameters=[{
            'in_topic':  'person_centroid',
            'out_topic': '/person_center_depth',
            'initial_active': False,       # start disabled; UART will enable
        }],
    )

    follower = Node(
        package='dancer_detector',
        executable='person_follower',
        name='person_follower_pid',
        output='screen',
        parameters=[{
            'img_width': img_w,
            'hfov_deg': hfov_deg,
            'target_dist_m': target_dist,
            'max_lin_vel': 0.30,
            'max_ang_vel': 1.5,
            'deadband_px': 4.0,
            'deadband_m': 0.03,
            'lp_alpha_err': 0.25,
            'rate_hz': 20.0,
            'kp_yaw': 1.2, 'ki_yaw': 0.0, 'kd_yaw': 0.12,
            'kp_fwd': 0.8, 'ki_fwd': 0.0, 'kd_fwd': 0.05,
        }],
    )

    # UART M/P-toggle → calls /dancer_detector/{start,stop}
    uart_toggle = Node(
        package='dancer_detector',
        executable='uart_seg_toggle',
        name='uart_seg_toggle',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA0',
            'baud': 115200,
            'debounce_window_s': 5.0,  # ignore triggers for 5s after a toggle
            'initial_active': False,
            'start_service': '/dancer_detector/start',
            'stop_service':  '/dancer_detector/stop',
        }],
    )

    return LaunchDescription([yolo, gate, follower, uart_toggle])
