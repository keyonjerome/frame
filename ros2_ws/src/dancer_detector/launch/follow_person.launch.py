# follow_person.launch.py
# Launches dancer_seg + person_follower, then auto-calls /dancer_detector/start
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # --- Params you probably want to tweak ---
    img_w = 640
    img_h = 480
    fps   = 30
    hfov_deg = 69.4
    target_dist = 1.75

    seg_node = Node(
        package='dancer_detector',
        executable='dancer_seg',
        name='dancer_seg',
        output='screen',
        parameters=[{
            'backend': 'realsense',          # or 'uvc'
            'width': img_w,
            'height': img_h,
            'fps': fps,
            'detection_mode': 'near',        # 'near' or 'far_red'
            'depth_min_m': 0.3,
            'depth_max_m': 6.0,
            'blue_max_norm': 30,
            'min_blob_area_frac': 0.003,
            'morph_kernel_w': 7,
            'morph_kernel_h': 7,
            'alpha_mask': 0.35,
            'proc_hz': 15.0,
            # from your weighted version:
            'gauss_sigma_px': 40.0,
        }],
    )

    follower_node = Node(
        package='dancer_detector',
        executable='person_follower',
        name='person_follower_pid',
        output='screen',
        parameters=[{
            'img_width': img_w,
            'hfov_deg': hfov_deg,
            'target_dist_m': target_dist,
            'max_lin_vel': 10,       # tune caps here
            'max_ang_vel': 10,
            'deadband_px': 2.0,
            'deadband_m': 0.03,
            'lp_alpha_err': 0.25,
            'rate_hz': 20.0,
            'kp_yaw': 1.2, 'ki_yaw': 0.0, 'kd_yaw': 0.12,
            'kp_fwd': 0.8, 'ki_fwd': 0.0, 'kd_fwd': 0.05,
        }],
    )

    # Call the start service shortly after dancer_seg is up
    start_seg = TimerAction(
        period=2.5,  # small delay to ensure node is ready
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/dancer_detector/start', 'std_srvs/srv/Trigger', '{}'
            ],
            shell=False
        )]
    )

    return LaunchDescription([
        seg_node,
        follower_node,
        start_seg,
    ])
