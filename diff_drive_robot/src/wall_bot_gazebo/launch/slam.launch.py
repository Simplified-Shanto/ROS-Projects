import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('wall_bot_gazebo')

    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
    rviz_config = os.path.join(gazebo_pkg, 'rviz', 'slam.rviz')

    default_world = os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_world.world')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo Sim world file to load.',
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Robot spawn x position in meters.',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='-2.0',
        description='Robot spawn y position in meters.',
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.20',
        description='Robot spawn z position in meters.',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
        }],
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gazebo,
        rviz,
        slam,
    ])
