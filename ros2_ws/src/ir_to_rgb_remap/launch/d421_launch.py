# launch/d421_ir_all_components.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    camera_name = 'camera'
    rs_params = {
        'enable_infra1': True,
        'enable_infra2': False,
        'enable_depth': True,
        'enable_color': False,
        'infra1_width': 640, 'infra1_height': 480, 'infra1_fps': 30,
        'depth_width': 640, 'depth_height': 480, 'depth_fps': 30,
        'align_depth': False,
        'pointcloud.enable': False,
        # 0 == none, 1 == copy, 2 == linear_interpolation
        'unite_imu_method': 0,
    }

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        namespace='',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            # RealSense
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name=camera_name,
                parameters=[rs_params],
            ),
            # Convert depth mm(uint16)->meters(float32) -> /depth
            ComposableNode(
                package='isaac_ros_depth_image_proc',
                plugin='nvidia::isaac_ros::depth_image_proc::ConvertMetricNode',
                name='convert_metric',
                remappings=[
                    ('image_raw', f'/{camera_name}/depth/image_rect_raw'),
                    ('image',     'depth'),
                ],
            ),
            # IR mono8 -> rgb8 on /image_rect, relay CameraInfo -> /camera_info_rect
            ComposableNode(
                package='ir_to_rgb_remap',
                plugin='ir_to_rgb_remap::IrToRgbRemapNode',
                name='ir_to_rgb_remap',
                parameters=[{
                    'in_image':       f'/{camera_name}/infra1/image_rect_raw',
                    'in_camera_info': f'/{camera_name}/infra1/camera_info',
                    'out_image':      '/image_rect',
                    'out_camera_info':'/camera_info_rect',
                }],
            ),
            # You can add your YOLOv8 encoder/decoder components here, or launch them separately
        ],
    )

    return LaunchDescription([container])
