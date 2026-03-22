from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_centerline = LaunchConfiguration("show_centerline")
    show_debug = LaunchConfiguration("show_debug")

    centerline_node = Node(
        package="path_planning",
        executable="centerline",
        name="center_line",
        output="screen",
        parameters=[
            {
                "mask_topic": "/lane/mask",
                "centerline_topic": "/lane/centerline",
                "overlay_topic": "/lane/centerline_image",
                "publish_overlay": show_centerline,
                "centerline_threshold": 0.5,
                "centerline_step": 4,
                "centerline_min_width": 6,
                "centerline_min_pixels": 20,
                "centerline_smooth": 7,
                "centerline_color": "0,255,255",
                "centerline_thickness": 2,
            }
        ],
    )

    pure_pursuit_node = Node(
        package="control",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[
            {
                "centerline_topic": "/lane/centerline",
                "mask_topic": "/lane/mask",
                "angle_topic": "/brain/cmd/steer",
                "steer_method": "center_offset",
                "steer_lookahead": 0.42,
                "steer_wheelbase": 0.25,
                "lane_width_cm": 35.0,
                "steer_wheelbase_cm": 26.7,
                "centerline_threshold": 0.5,
                "centerline_min_width": 6,
                "centerline_min_pixels": 20,
                "steer_max_angle": 25.0,
                "additional_weight": 1.5,
                "steer_output_scale": 10.0,
                "speed_topic": "/brain/cmd/speed",
                "default_speed": 200.0,
                "steer_bias_deg": 0.0,
                "steer_deadband": 0.0,
                "steer_smooth": 0.0,
                "angle_in_degrees": True,
                "use_mask": True,
                "image_width": 320,
                "image_height": 240,
                "log_angle": False,
                "log_period": 0.5,
                "overlay_topic": "/lane/overlay_base",
                "debug_image_topic": "/lane_segment_image",
                "publish_debug_image": True,
                "draw_centerline": True,
                "draw_lookahead": True,
                "draw_angle_text": True,
                "draw_vehicle": True,
                "debug_line_thickness": 2,
                "debug_point_radius": 4,
                "debug_text_scale": 0.6,
                "debug_text_thickness": 2,
                "debug_centerline_color": "0,255,255",
                "debug_lookahead_color": "0,255,255",
                "debug_vehicle_color": "255,255,255",
                "debug_text_color": "0,255,255",
                "debug_line_color": "0,255,255",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_centerline",
                default_value="false",
                description="Show centerline overlay window",
            ),
            DeclareLaunchArgument(
                "show_debug",
                default_value="true",
                description="Show debug overlay window",
            ),
            centerline_node,
            TimerAction(period=1.0, actions=[pure_pursuit_node]),
            Node(
                package="image_view",
                executable="image_view",
                name="centerline_view",
                condition=IfCondition(show_centerline),
                remappings=[("image", "/lane/centerline_image")],
            ),
            Node(
                package="image_view",
                executable="image_view",
                name="debug_view",
                condition=IfCondition(show_debug),
                remappings=[("image", "/lane_segment_image")],
            ),
        ]
    )
