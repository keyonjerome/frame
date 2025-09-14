from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    # Common camera/follower params (tweak as needed)
    img_w = 640
    img_h = 480
    fps   = 30
    hfov_deg = 69.4
    target_dist = 1.75

    yolo = Node(
        package='dancer_detector',
        executable='yolo_ir_realsense',
        name='yolo_ir_realsense',
        output='screen',
        parameters=[{
            # override these paths if needed:
            'weights_path':  '~//models/yolov3/yolov3.weights',
            'cfg_path':      '~//models/yolov3/yolov3.cfg',
            'names_path':    '~//models/yolov3/coco.names',
            'show_window':   True,
        }],
    )

    bridge = Node(
        package='dancer_detector',
        executable='centroid_to_centerdepth',
        name='centroid_to_centerdepth',
        output='screen',
        parameters=[{
            'in_topic':  'person_centroid',
            'out_topic': '/person_center_depth',
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
            'max_lin_vel': 5.5,
            'max_ang_vel': 5.5,
            'deadband_px': 4.0,
            'deadband_m': 0.03,
            'lp_alpha_err': 0.25,
            'rate_hz': 20.0,
            'kp_yaw': 1.2, 'ki_yaw': 0.0, 'kd_yaw': 0.12,
            'kp_fwd': 0.8, 'ki_fwd': 0.0, 'kd_fwd': 0.05,
        }],
    )

    return LaunchDescription([
        yolo,
        bridge,
        follower,
    ])
