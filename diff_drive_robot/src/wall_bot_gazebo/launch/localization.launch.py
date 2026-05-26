import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('wall_bot_gazebo')

    amcl_params = os.path.join(gazebo_pkg, 'config', 'amcl_params.yaml')
    map_file = LaunchConfiguration('map')

    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
    rviz_config = os.path.join(gazebo_pkg, 'rviz', 'localization.rviz')

    default_world = os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_world.world')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/shanto/Github/diff_drive_robot/maps/wall_bot_map.yaml',
        description='Absolute path to the saved map yaml file.',
    )

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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_params,
            {'yaml_filename': map_file},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gazebo,
        rviz,
        map_server,
        amcl,
        lifecycle_manager
    ])
