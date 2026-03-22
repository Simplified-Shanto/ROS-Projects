from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lane_lab')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz.rviz')

    return LaunchDescription([
        Node(
            package='lane_lab',
            executable='mask_video_publisher',
            name='mask_video_publisher_node',
            output='screen',
            parameters=[{
                'video_path': '/home/shanto/ros2_ws_lane_lab/src/lane_lab/lane_lab/data/lane_segment.mp4',
                'mask_topic': '/lane/mask',
                'fps': 10.0,
                'loop': True,
            }],
        ),
        Node(
            package='path_planning',
            executable='centerline',
            name='centerline_finder_node',
            output='screen',
            parameters=[{
                'mask_topic': '/lane/mask',
                'centerline_topic': '/lane/centerline',
                'overlay_topic': '/lane/centerline_image',
                'publish_overlay': True,
            }],
        ),

        Node(
            package='control',
            executable='pure_pursuit',
            name='path_following_node',
            output='screen',
            parameters=[{
                'centerline_topic': '/lane/centerline',
                'mask_topic': '/lane/mask',
                'angle_topic': '/steering_cmd',
                'speed_topic': '/speed_cmd',
                'steer_method': 'center_offset',
                'steer_lookahead': 0.42,
                'steer_max_angle': 25.0,
                'steer_output_scale': 1.0,
                'additional_weight': 1.0,
                'default_speed': 200.0,
                'use_mask': True,
                'overlay_topic': '/lane/centerline_image',
                'debug_image_topic': '/lane/pure_pursuit_debug',
                'publish_debug_image': True,
                'log_angle': False,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
),

    ])
