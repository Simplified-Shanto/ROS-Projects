import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #########################
    # Robot State Publisher
    #########################
    description_pkg = get_package_share_directory('wall_bot_description')
    xacro_file = os.path.join(description_pkg, 'urdf', 'wall_bot.urdf.xacro')
 
    robot_description = Command(['xacro', ' ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    #########################
    # Gazebo Related Nodes 
    #########################

    gazebo_pkg = get_package_share_directory('wall_bot_gazebo')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'lazyWorld.world')

    materials_path = os.path.join(gazebo_pkg, 'worlds')
    existing_path = os.environ.get('GAZEBO_RESOURCE_PATH','')
    if existing_path: 
        os.environ['GAZEBO_RESOURCE_PATH'] = existing_path + ':' + materials_path
    else: 
        os.environ['GAZEBO_RESOURCE_PATH'] = materials_path

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
    '-y', '2.0',
    '-z', '0.0',
    '-R', '0.0',
    '-P', '0.0',
    '-Y', '0.0'
		],
        output='screen'
    )



    #########################
    # Wall Follower Node 
    #########################


    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.30',
        description='Maximum forward speed for the wall follower node',
    )
    turn_linear_speed_arg = DeclareLaunchArgument(
        'turn_linear_speed',
        default_value='0.12',
        description='Forward speed used while turning',
    )

    linear_speed = LaunchConfiguration('linear_speed')
    turn_linear_speed = LaunchConfiguration('turn_linear_speed')

    wall_follower_node = Node(
        package='wall_follower',
        executable='wall_follower_node',
        output='screen',
        parameters=[{
            'linear_speed': linear_speed,
            'turn_linear_speed': turn_linear_speed,
        }]
    )


    #########################
    # Line counter Node 
    #########################

    lap_count_arg = DeclareLaunchArgument(
        'lap_count',
        default_value='3',
        description='Number of laps to be completed by the robot',
    )
    lap_count = LaunchConfiguration('lap_count')

    line_counter_node = Node(
        package='wall_follower', 
        executable='line_counter_node', 
        output='screen', 
        parameters=[{
            'lap_count': lap_count
        }]
    )

    #########################
    # Rviz                  #
    #########################

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2', 
    )


    return LaunchDescription([
        linear_speed_arg,
        turn_linear_speed_arg,
        lap_count_arg, 
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot,
        RegisterEventHandler( # wall_follower_node starts after 5 second of exiting spawn_robot_node 
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    LogInfo(msg='wall_bot spawned. Starting wall follower in 5...'),
                    TimerAction(period=1.0, actions=[LogInfo(msg='4...')]),
                    TimerAction(period=2.0, actions=[LogInfo(msg='3...')]),
                    TimerAction(period=3.0, actions=[LogInfo(msg='2...')]),
                    TimerAction(period=4.0, actions=[LogInfo(msg='1...')]),
                    TimerAction(
                        period=5.0,
                        actions=[wall_follower_node]
                    )
                ],
            )
        ),
        line_counter_node, 
        rviz,
    ])
