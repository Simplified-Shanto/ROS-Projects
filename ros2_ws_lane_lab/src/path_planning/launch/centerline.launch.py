from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    mask_topic = LaunchConfiguration("mask_topic")
    centerline_topic = LaunchConfiguration("centerline_topic")
    overlay_topic = LaunchConfiguration("overlay_topic")
    background_topic = LaunchConfiguration("background_topic")
    publish_overlay = LaunchConfiguration("publish_overlay")
    show_overlay = LaunchConfiguration("show_overlay")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.expanduser("~/autodrive/src/autodrive_configs/params/centerline.yaml"),
                description="ROS2 parameter YAML file for centerline node",
            ),
            DeclareLaunchArgument(
                "mask_topic",
                default_value="/lane/mask",
                description="Input lane mask Image topic",
            ),
            DeclareLaunchArgument(
                "centerline_topic",
                default_value="/lane/centerline",
                description="Output centerline Path topic",
            ),
            DeclareLaunchArgument(
                "overlay_topic",
                default_value="/lane/centerline_image",
                description="Overlay image topic",
            ),
            DeclareLaunchArgument(
                "background_topic",
                default_value="",
                description="Optional background image topic for overlay",
            ),
            DeclareLaunchArgument(
                "publish_overlay",
                default_value="true",
                description="Publish overlay image",
            ),
            DeclareLaunchArgument(
                "show_overlay",
                default_value="true",
                description="Show overlay in a viewer window",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Override output frame_id (optional)",
            ),
            Node(
                package="path_planning",
                executable="centerline",
                name="path_planning_centerline",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "mask_topic": mask_topic,
                        "centerline_topic": centerline_topic,
                        "overlay_topic": overlay_topic,
                        "background_topic": background_topic,
                        "publish_overlay": publish_overlay,
                        "frame_id": frame_id,
                    }
                ],
            ),
            Node(
                package="image_tools",
                executable="showimage",
                name="path_planning_centerline_view",
                condition=IfCondition(show_overlay),
                output="log",
                arguments=["--ros-args", "--log-level", "warn"],
                remappings=[("image", overlay_topic)],
            ),
        ]
    )
