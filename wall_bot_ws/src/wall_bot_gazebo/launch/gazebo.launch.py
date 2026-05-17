import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory('wall_bot_description')
    gazebo_pkg = get_package_share_directory('wall_bot_gazebo')

    xacro_file = os.path.join(description_pkg, 'urdf', 'wall_bot.urdf.xacro')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'lazyWorld.world')

    materials_path = os.path.join(gazebo_pkg, 'worlds')
    existing_path = os.environ.get('GAZEBO_RESOURCE_PATH','')
    if existing_path: 
        os.environ['GAZEBO_RESOURCE_PATH'] = existing_path + ':' + materials_path
    else: 
        os.environ['GAZEBO_RESOURCE_PATH'] = materials_path

    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
    '-topic', 'robot_description',
    '-entity', 'wall_bot',
    '-x', '0.0',
    '-y', '0.0',
    '-z', '0.0',
    '-R', '0.0',
    '-P', '0.0',
    '-Y', '0.0'
		],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot
    ])
