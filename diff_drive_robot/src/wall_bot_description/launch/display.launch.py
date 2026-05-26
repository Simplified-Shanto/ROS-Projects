import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('wall_bot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'wall_bot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'wall_bot_rviz_config.rviz')

    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2', 
        arguments=['-d', rviz_config_file]
        
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
