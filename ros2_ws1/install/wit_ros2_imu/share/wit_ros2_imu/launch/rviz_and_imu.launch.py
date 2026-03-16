import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wit_ros2_imu')
    rviz_config = os.path.join(package_share, 'rviz', 'imu.rviz')
    urdf_path = os.path.join(package_share, 'urdf', 'im10a_dummy.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baud': 9600},
        ],
        remappings=[('imu', '/imu/data')],
        output='screen',
    )

    imu_tf_node = Node(
        package='wit_ros2_imu',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        parameters=[
            {'parent_frame': 'map'},
            {'child_frame': 'imu_link'},
            {'imu_topic': '/imu/data'},
        ],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        imu_node,
        imu_tf_node,
        robot_state_publisher_node,
        rviz_node,
    ])
