from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    centerline_topic = LaunchConfiguration("centerline_topic")
    mask_topic = LaunchConfiguration("mask_topic")
    overlay_topic = LaunchConfiguration("overlay_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.expanduser("~/autodrive/src/autodrive_configs/params/pure_pursuit.yaml"),
                description="ROS2 parameter YAML file for pure pursuit node",
            ),
            DeclareLaunchArgument(
                "centerline_topic",
                default_value="/lane/centerline",
                description="Input centerline Path topic",
            ),
            DeclareLaunchArgument(
                "mask_topic",
                default_value="/lane/mask",
                description="Input lane mask topic",
            ),
            DeclareLaunchArgument(
                "overlay_topic",
                default_value="/lane/final_centerline_image",
                description="Overlay image topic to draw debug on",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/lane/final_drive_image",
                description="Final debug image topic with steering/speed text",
            ),
            Node(
                package="control",
                executable="pure_pursuit",
                name="pure_pursuit",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "centerline_topic": centerline_topic,
                        "mask_topic": mask_topic,
                        "overlay_topic": overlay_topic,
                        "debug_image_topic": debug_image_topic,
                    }
                ],
            )
        ]
    )
