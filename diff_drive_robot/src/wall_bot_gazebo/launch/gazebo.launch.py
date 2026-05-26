import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory('wall_bot_description')
    gazebo_pkg = get_package_share_directory('wall_bot_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(description_pkg, 'urdf', 'wall_bot.urdf.xacro')
    bridge_config = os.path.join(gazebo_pkg, 'config', 'ros_gz_bridge.yaml')

    # Use a Gazebo Sim-native world here.
    default_world = os.path.join(gazebo_pkg, 'worlds', 'turtlebot3_world.world')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    models_path = os.path.join(gazebo_pkg, 'models')
    worlds_path = os.path.join(gazebo_pkg, 'worlds')

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

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            os.pathsep,
            models_path,
            os.pathsep,
            worlds_path,
        ],
    )

    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'laser_link',
            'wall_bot/base_footprint/lidar_sensor',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-name', 'wall_bot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        set_gz_resource_path,
        robot_state_publisher,
        lidar_frame_bridge,
        gz_sim,
        bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])
