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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
),

    ])
