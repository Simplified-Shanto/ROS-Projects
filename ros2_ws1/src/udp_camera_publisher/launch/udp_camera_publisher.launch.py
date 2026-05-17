from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument('topic', default_value='image_raw'),
        DeclareLaunchArgument('frame_id', default_value='camera'),
        DeclareLaunchArgument('retry_interval', default_value='2.0'),
        DeclareLaunchArgument('reliability', default_value='reliable'),
        DeclareLaunchArgument('qos_depth', default_value='1'),
        DeclareLaunchArgument('resize_width', default_value='0'),
        DeclareLaunchArgument('resize_height', default_value='0'),
        DeclareLaunchArgument('stats_interval', default_value='5.0'),
        Node(
            package='udp_camera_publisher',
            executable='udp_camera_publisher',
            name='udp_camera_publisher',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'topic': LaunchConfiguration('topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'retry_interval': LaunchConfiguration('retry_interval'),
                'reliability': LaunchConfiguration('reliability'),
                'qos_depth': LaunchConfiguration('qos_depth'),
                'resize_width': LaunchConfiguration('resize_width'),
                'resize_height': LaunchConfiguration('resize_height'),
                'stats_interval': LaunchConfiguration('stats_interval'),
            }],
        ),
    ])
