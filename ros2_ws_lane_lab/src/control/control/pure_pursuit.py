#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, String

from .steering_utils import compute_steering_angle

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

Point = Tuple[int, int]


def _image_to_numpy(msg: Image) -> Optional[np.ndarray]:
    if msg.height == 0 or msg.width == 0:
        return None
    encoding = msg.encoding.lower()
    if encoding in ("mono8", "8uc1"):
        dtype = np.uint8
        channels = 1
    elif encoding in ("mono16", "16uc1"):
        dtype = np.uint16
        channels = 1
    elif encoding in ("32fc1",):
        dtype = np.float32
        channels = 1
    else:
        return None
    row_bytes = msg.width * channels * np.dtype(dtype).itemsize
    if msg.step < row_bytes:
        return None
    data = np.frombuffer(msg.data, dtype=dtype)
    row_elems = msg.step // np.dtype(dtype).itemsize
    if data.size < row_elems * msg.height:
        return None
    data = data[: row_elems * msg.height]
    img = data.reshape((msg.height, row_elems))
    img = img[:, : msg.width * channels]
    if channels == 1:
        return img.reshape((msg.height, msg.width)).copy()
    return img.reshape((msg.height, msg.width, channels)).copy()


def _image_to_bgr(msg: Image) -> Optional[np.ndarray]:
    if msg.height == 0 or msg.width == 0:
        return None
    encoding = msg.encoding.lower()
    if encoding not in ("bgr8", "rgb8", "mono8"):
        return None
    channels = 3 if encoding in ("bgr8", "rgb8") else 1
    step = msg.step
    row_bytes = msg.width * channels
    if step < row_bytes:
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < step * msg.height:
        return None
    data = data[: step * msg.height]
    img = data.reshape((msg.height, step))
    img = img[:, :row_bytes]
    img = img.reshape((msg.height, msg.width, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if cv2 is not None else img[:, :, ::-1].copy()
    if encoding == "mono8":
        if cv2 is not None:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return np.repeat(img, 3, axis=2)
    return img.copy()


def _numpy_to_image_msg(img: np.ndarray, encoding: str, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(img.shape[0])
    msg.width = int(img.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(img.strides[0])
    msg.data = img.tobytes()
    return msg


def _parse_rgb(value, default=(0, 255, 255)) -> Tuple[int, int, int]:
    if value is None:
        return default
    text = str(value).strip()
    if not text:
        return default
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != 3:
        return default
    try:
        r, g, b = (int(float(p)) for p in parts)
    except Exception:
        return default
    r = int(np.clip(r, 0, 255))
    g = int(np.clip(g, 0, 255))
    b = int(np.clip(b, 0, 255))
    return (r, g, b)


def _path_to_points(path: Path) -> Sequence[Point]:
    points = []
    for pose in path.poses:
        x = int(round(pose.pose.position.x))
        y = int(round(pose.pose.position.y))
        points.append((x, y))
    return points


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        self.declare_parameter("centerline_topic", "/lane/centerline")
        self.declare_parameter("mask_topic", "/lane/mask")
        self.declare_parameter("angle_topic", "/steering_angle")
        # Video_test compatible parameter names (preferred)
        self.declare_parameter("steer_method", "center_offset")
        self.declare_parameter("steer_lookahead", 0.3)
        self.declare_parameter("adaptive_lookahead_enable", True)
        self.declare_parameter("adaptive_lookahead_min", 0.14)
        self.declare_parameter("adaptive_lookahead_span_scale", 0.85)
        self.declare_parameter("steer_wheelbase", 0.25)
        self.declare_parameter("steer_wheelbase_cm", 26.7)
        self.declare_parameter("steer_max_angle", 25.0)
        self.declare_parameter("steer_bias_deg", 0.0)
        self.declare_parameter("steer_deadband", 0.0)
        self.declare_parameter("steer_smooth", 0.0)
        self.declare_parameter("steer_kalman_enable", True)
        self.declare_parameter("steer_kalman_process_noise", 0.20)
        self.declare_parameter("steer_kalman_measurement_noise", 6.0)
        self.declare_parameter("steer_kalman_initial_covariance", 10.0)
        # Output scaling after steering is computed (deg or rad based on angle_in_degrees)
        self.declare_parameter("steer_output_scale", 10.0)
        self.declare_parameter("additional_weight", 1.0)
        self.declare_parameter("steer_cmd_limit", 250.0)
        self.declare_parameter("speed_topic", "/brain/cmd/speed")
        self.declare_parameter("parking_mode_topic", "/parking/mode_enabled")
        self.declare_parameter("disable_when_parking_mode", True)
        self.declare_parameter("behavior_mode_topic", "/autodrive/behavior/mode")
        self.declare_parameter("disable_when_behavior_parking_out", True)
        self.declare_parameter("disable_when_behavior_intersection", True)
        self.declare_parameter("intersection_status_topic", "/intersection/status")
        self.declare_parameter("disable_when_intersection_active", True)
        self.declare_parameter("default_speed", 200.0)
        self.declare_parameter("speed_scaling_max", 1.0)
        self.declare_parameter("speed_scaling_min", 1.0)
        self.declare_parameter("enable_turn_speed_control", True)
        self.declare_parameter("turn_speed_min", 120.0)
        self.declare_parameter("turn_angle_low_deg", 6.0)
        self.declare_parameter("turn_angle_high_deg", 20.0)
        self.declare_parameter("turn_speed_exponent", 1.0)
        self.declare_parameter("enable_edge_curve_compensation", True)
        self.declare_parameter("edge_comp_margin_px", 18)
        self.declare_parameter("edge_comp_margin_left_px", -1)
        self.declare_parameter("edge_comp_margin_right_px", -1)
        self.declare_parameter("edge_comp_boost_deg", 5.0)
        self.declare_parameter("edge_comp_min_angle_deg", 3.0)
        self.declare_parameter("edge_comp_exponent", 1.0)
        self.declare_parameter("lane_width_cm", 35.0)
        self.declare_parameter("centerline_threshold", 0.5)
        self.declare_parameter("centerline_min_width", 6)
        self.declare_parameter("centerline_min_pixels", 20)
        # Legacy aliases (fallback)
        self.declare_parameter("method", "center_offset")
        self.declare_parameter("lookahead", 0.3)
        self.declare_parameter("wheelbase", 0.25)
        self.declare_parameter("wheelbase_cm", 26.7)
        self.declare_parameter("mask_threshold", 0.5)
        self.declare_parameter("mask_min_width", 6)
        self.declare_parameter("mask_min_pixels", 20)
        self.declare_parameter("max_angle_deg", 25.0)
        self.declare_parameter("angle_in_degrees", True)
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("use_mask", True)
        self.declare_parameter("log_angle", False)
        self.declare_parameter("log_period", 0.5)
        self.declare_parameter("overlay_topic", "/lane_segment_image")
        self.declare_parameter("debug_image_topic", "/lane/debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("draw_centerline", True)
        self.declare_parameter("draw_lookahead", True)
        self.declare_parameter("draw_angle_text", True)
        self.declare_parameter("draw_vehicle", True)
        self.declare_parameter("debug_line_thickness", 2)
        self.declare_parameter("debug_point_radius", 4)
        self.declare_parameter("debug_text_scale", 0.6)
        self.declare_parameter("debug_text_thickness", 2)
        self.declare_parameter("debug_centerline_color", "0,255,255")
        self.declare_parameter("debug_lookahead_color", "0,255,255")
        self.declare_parameter("debug_vehicle_color", "255,255,255")
        self.declare_parameter("debug_text_color", "0,255,255")
        self.declare_parameter("debug_line_color", "0,255,255")
        self.declare_parameter("shutdown_stop_repeats", 6)
        self.declare_parameter("shutdown_stop_interval_sec", 0.05)

        self.centerline_topic = str(self.get_parameter("centerline_topic").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.angle_topic = str(self.get_parameter("angle_topic").value)
        self.method = str(self.get_parameter("steer_method").value)
        self.lookahead = float(self.get_parameter("steer_lookahead").value)
        self.adaptive_lookahead_enable = bool(self.get_parameter("adaptive_lookahead_enable").value)
        self.adaptive_lookahead_min = float(self.get_parameter("adaptive_lookahead_min").value)
        self.adaptive_lookahead_span_scale = float(self.get_parameter("adaptive_lookahead_span_scale").value)
        self.wheelbase = float(self.get_parameter("steer_wheelbase").value)
        self.wheelbase_cm = float(self.get_parameter("steer_wheelbase_cm").value)
        self.max_angle_deg = float(self.get_parameter("steer_max_angle").value)
        self.steer_kalman_enable = bool(self.get_parameter("steer_kalman_enable").value)
        self.steer_kalman_process_noise = float(self.get_parameter("steer_kalman_process_noise").value)
        self.steer_kalman_measurement_noise = float(self.get_parameter("steer_kalman_measurement_noise").value)
        self.steer_kalman_initial_covariance = float(
            self.get_parameter("steer_kalman_initial_covariance").value
        )
        self.lane_width_cm = float(self.get_parameter("lane_width_cm").value)
        self.mask_threshold = float(self.get_parameter("centerline_threshold").value)
        self.mask_min_width = int(self.get_parameter("centerline_min_width").value)
        self.mask_min_pixels = int(self.get_parameter("centerline_min_pixels").value)
        # Fallback to legacy names if they differ from defaults.
        if self.method == "center_offset":
            legacy = str(self.get_parameter("method").value)
            if legacy and legacy != "center_offset":
                self.method = legacy
        if self.lookahead == 0.3:
            legacy = float(self.get_parameter("lookahead").value)
            if legacy != 0.3:
                self.lookahead = legacy
        if self.wheelbase == 0.25:
            legacy = float(self.get_parameter("wheelbase").value)
            if legacy != 0.25:
                self.wheelbase = legacy
        if self.wheelbase_cm == 26.7:
            legacy = float(self.get_parameter("wheelbase_cm").value)
            if legacy != 26.7:
                self.wheelbase_cm = legacy
        if self.max_angle_deg == 25.0:
            legacy = float(self.get_parameter("max_angle_deg").value)
            if legacy != 25.0:
                self.max_angle_deg = legacy
        if self.mask_threshold == 0.5:
            legacy = float(self.get_parameter("mask_threshold").value)
            if legacy != 0.5:
                self.mask_threshold = legacy
        if self.mask_min_width == 6:
            legacy = int(self.get_parameter("mask_min_width").value)
            if legacy != 6:
                self.mask_min_width = legacy
        if self.mask_min_pixels == 20:
            legacy = int(self.get_parameter("mask_min_pixels").value)
            if legacy != 20:
                self.mask_min_pixels = legacy
        self.steer_bias_deg = float(self.get_parameter("steer_bias_deg").value)
        self.steer_deadband = float(self.get_parameter("steer_deadband").value)
        self.steer_smooth = float(self.get_parameter("steer_smooth").value)
        self.steer_output_scale = float(self.get_parameter("steer_output_scale").value)
        self.additional_weight = float(self.get_parameter("additional_weight").value)
        self.steer_cmd_limit = float(self.get_parameter("steer_cmd_limit").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.parking_mode_topic = str(self.get_parameter("parking_mode_topic").value)
        self.disable_when_parking_mode = bool(self.get_parameter("disable_when_parking_mode").value)
        self.behavior_mode_topic = str(self.get_parameter("behavior_mode_topic").value)
        self.disable_when_behavior_parking_out = bool(self.get_parameter("disable_when_behavior_parking_out").value)
        self.disable_when_behavior_intersection = bool(
            self.get_parameter("disable_when_behavior_intersection").value
        )
        self.intersection_status_topic = str(self.get_parameter("intersection_status_topic").value)
        self.disable_when_intersection_active = bool(self.get_parameter("disable_when_intersection_active").value)
        self.default_speed = float(self.get_parameter("default_speed").value)
        self.speed_scaling_max = float(self.get_parameter("speed_scaling_max").value)
        self.speed_scaling_min = float(self.get_parameter("speed_scaling_min").value)
        self.enable_turn_speed_control = bool(self.get_parameter("enable_turn_speed_control").value)
        self.turn_speed_min = float(self.get_parameter("turn_speed_min").value)
        self.turn_angle_low_deg = float(self.get_parameter("turn_angle_low_deg").value)
        self.turn_angle_high_deg = float(self.get_parameter("turn_angle_high_deg").value)
        self.turn_speed_exponent = float(self.get_parameter("turn_speed_exponent").value)
        self.enable_edge_curve_compensation = bool(self.get_parameter("enable_edge_curve_compensation").value)
        self.edge_comp_margin_px = int(self.get_parameter("edge_comp_margin_px").value)
        self.edge_comp_margin_left_px = int(self.get_parameter("edge_comp_margin_left_px").value)
        self.edge_comp_margin_right_px = int(self.get_parameter("edge_comp_margin_right_px").value)
        self.edge_comp_boost_deg = float(self.get_parameter("edge_comp_boost_deg").value)
        self.edge_comp_min_angle_deg = float(self.get_parameter("edge_comp_min_angle_deg").value)
        self.edge_comp_exponent = float(self.get_parameter("edge_comp_exponent").value)
        self.angle_in_degrees = bool(self.get_parameter("angle_in_degrees").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.use_mask = bool(self.get_parameter("use_mask").value)
        self.log_angle = bool(self.get_parameter("log_angle").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.draw_centerline = bool(self.get_parameter("draw_centerline").value)
        self.draw_lookahead = bool(self.get_parameter("draw_lookahead").value)
        self.draw_angle_text = bool(self.get_parameter("draw_angle_text").value)
        self.draw_vehicle = bool(self.get_parameter("draw_vehicle").value)
        self.debug_line_thickness = int(self.get_parameter("debug_line_thickness").value)
        self.debug_point_radius = int(self.get_parameter("debug_point_radius").value)
        self.debug_text_scale = float(self.get_parameter("debug_text_scale").value)
        self.debug_text_thickness = int(self.get_parameter("debug_text_thickness").value)
        c_center = _parse_rgb(self.get_parameter("debug_centerline_color").value)
        c_look = _parse_rgb(self.get_parameter("debug_lookahead_color").value)
        c_vehicle = _parse_rgb(self.get_parameter("debug_vehicle_color").value, default=(255, 255, 255))
        c_text = _parse_rgb(self.get_parameter("debug_text_color").value)
        c_line = _parse_rgb(self.get_parameter("debug_line_color").value)
        # OpenCV uses BGR
        self.debug_centerline_color = (c_center[2], c_center[1], c_center[0])
        self.debug_lookahead_color = (c_look[2], c_look[1], c_look[0])
        self.debug_vehicle_color = (c_vehicle[2], c_vehicle[1], c_vehicle[0])
        self.debug_text_color = (c_text[2], c_text[1], c_text[0])
        self.debug_line_color = (c_line[2], c_line[1], c_line[0])
        self.shutdown_stop_repeats = int(self.get_parameter("shutdown_stop_repeats").value)
        self.shutdown_stop_interval_sec = float(self.get_parameter("shutdown_stop_interval_sec").value)

        self._last_mask: Optional[np.ndarray] = None
        self._last_shape: Optional[Sequence[int]] = None
        self._steer_ema: Optional[float] = None
        self._steer_kf_x: Optional[float] = None
        self._steer_kf_p: float = max(1e-6, float(self.steer_kalman_initial_covariance))
        self._last_log_ns: int = 0
        self._last_overlay: Optional[np.ndarray] = None
        self._last_overlay_header = None
        self._stop_published = False
        self._parking_mode_enabled = False
        self._behavior_mode = "DRIVING"
        self._intersection_state = "IDLE"
        self._parking_suppress_logged = False
        self._suppress_reason = ""

        qos_mask = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_overlay = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mask_sub = self.create_subscription(Image, self.mask_topic, self._on_mask, qos_mask)
        self.center_sub = self.create_subscription(Path, self.centerline_topic, self._on_centerline, 10)
        self.parking_mode_sub = self.create_subscription(Bool, self.parking_mode_topic, self._on_parking_mode, 10)
        self.behavior_mode_sub = self.create_subscription(String, self.behavior_mode_topic, self._on_behavior_mode, 10)
        self.intersection_status_sub = self.create_subscription(
            String, self.intersection_status_topic, self._on_intersection_status, 10
        )
        self.angle_pub = self.create_publisher(Float32, self.angle_topic, 10)
        self.speed_pub = self.create_publisher(Float32, self.speed_topic, 10)
        self.overlay_sub = self.create_subscription(Image, self.overlay_topic, self._on_overlay, qos_mask)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, qos_overlay)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self._register_shutdown_handler()

        self.get_logger().info(f"Subscribed to centerline -> {self.centerline_topic}")
        self.get_logger().info(f"Publishing steering angle -> {self.angle_topic}")
        self.get_logger().info(f"Publishing speed -> {self.speed_topic}")
        self.get_logger().info(
            f"Parking command takeover gating: enabled={self.disable_when_parking_mode} "
            f"topic={self.parking_mode_topic}"
        )
        self.get_logger().info(
            f"Behavior mode gating (parking_out): enabled={self.disable_when_behavior_parking_out} "
            f"topic={self.behavior_mode_topic}"
        )
        self.get_logger().info(
            f"Behavior mode gating (intersection): enabled={self.disable_when_behavior_intersection} "
            f"topic={self.behavior_mode_topic}"
        )
        self.get_logger().info(
            f"Intersection status gating: enabled={self.disable_when_intersection_active} "
            f"topic={self.intersection_status_topic}"
        )
        self.get_logger().info(
            "Steering Kalman: "
            f"enabled={self.steer_kalman_enable}, "
            f"Q={self.steer_kalman_process_noise:.3f}, "
            f"R={self.steer_kalman_measurement_noise:.3f}, "
            f"P0={self.steer_kalman_initial_covariance:.3f}"
        )
        if self.use_mask:
            self.get_logger().info(f"Using mask -> {self.mask_topic}")
        if self.publish_debug_image:
            self.get_logger().info(f"Publishing debug image -> {self.debug_image_topic}")

    def _apply_steer_kalman(self, measured_angle_deg: float) -> float:
        if not self.steer_kalman_enable:
            return float(measured_angle_deg)
        z = float(measured_angle_deg)
        if not np.isfinite(z):
            return 0.0

        q = max(1e-9, float(self.steer_kalman_process_noise))
        r = max(1e-9, float(self.steer_kalman_measurement_noise))

        if self._steer_kf_x is None:
            self._steer_kf_x = z
            self._steer_kf_p = max(1e-6, float(self.steer_kalman_initial_covariance))
            return float(self._steer_kf_x)

        # Predict
        p_pred = self._steer_kf_p + q
        x_pred = self._steer_kf_x
        # Update
        k = p_pred / (p_pred + r)
        self._steer_kf_x = x_pred + k * (z - x_pred)
        self._steer_kf_p = (1.0 - k) * p_pred
        return float(self._steer_kf_x)

    def _on_parameters_changed(self, params):
        old_scale_max = float(self.speed_scaling_max)
        old_scale_min = float(self.speed_scaling_min)
        scaling_updated = False
        for p in params:
            name = p.name
            value = p.value
            if name == "speed_scaling_max":
                try:
                    self.speed_scaling_max = max(0.0, float(value))
                    scaling_updated = True
                except Exception:
                    return SetParametersResult(successful=False, reason="speed_scaling_max must be numeric")
            elif name == "speed_scaling_min":
                try:
                    self.speed_scaling_min = max(0.0, float(value))
                    scaling_updated = True
                except Exception:
                    return SetParametersResult(successful=False, reason="speed_scaling_min must be numeric")
            elif name == "default_speed":
                try:
                    self.default_speed = float(value)
                except Exception:
                    return SetParametersResult(successful=False, reason="default_speed must be numeric")
            elif name == "turn_speed_min":
                try:
                    self.turn_speed_min = float(value)
                except Exception:
                    return SetParametersResult(successful=False, reason="turn_speed_min must be numeric")
            elif name == "steer_lookahead":
                try:
                    self.lookahead = max(0.01, float(value))
                    self.get_logger().info(f"Runtime lookahead update: steer_lookahead={self.lookahead:.3f}")
                except Exception:
                    return SetParametersResult(successful=False, reason="steer_lookahead must be numeric")
            elif name == "steer_kalman_enable":
                try:
                    self.steer_kalman_enable = bool(value)
                    if not self.steer_kalman_enable:
                        self._steer_kf_x = None
                except Exception:
                    return SetParametersResult(successful=False, reason="steer_kalman_enable must be bool")
            elif name == "steer_kalman_process_noise":
                try:
                    self.steer_kalman_process_noise = max(1e-9, float(value))
                except Exception:
                    return SetParametersResult(successful=False, reason="steer_kalman_process_noise must be numeric")
            elif name == "steer_kalman_measurement_noise":
                try:
                    self.steer_kalman_measurement_noise = max(1e-9, float(value))
                except Exception:
                    return SetParametersResult(successful=False, reason="steer_kalman_measurement_noise must be numeric")
            elif name == "steer_kalman_initial_covariance":
                try:
                    self.steer_kalman_initial_covariance = max(1e-6, float(value))
                    self._steer_kf_p = float(self.steer_kalman_initial_covariance)
                    self._steer_kf_x = None
                except Exception:
                    return SetParametersResult(successful=False, reason="steer_kalman_initial_covariance must be numeric")
        if scaling_updated and (
            abs(self.speed_scaling_max - old_scale_max) > 1e-9
            or abs(self.speed_scaling_min - old_scale_min) > 1e-9
        ):
            speed_max_eff = float(self.default_speed) * float(self.speed_scaling_max)
            speed_min_eff = float(self.turn_speed_min) * float(self.speed_scaling_min)
            self.get_logger().info(
                "Runtime speed scaling update: "
                f"scale_max={self.speed_scaling_max:.3f}, scale_min={self.speed_scaling_min:.3f}, "
                f"effective_max={speed_max_eff:.1f}, effective_min={speed_min_eff:.1f}"
            )
        return SetParametersResult(successful=True)

    def _compute_speed_cmd(self, angle_deg: float) -> float:
        scale_max = float(self.speed_scaling_max)
        if not np.isfinite(scale_max):
            scale_max = 1.0
        scale_max = max(0.0, scale_max)

        scale_min = float(self.speed_scaling_min)
        if not np.isfinite(scale_min):
            scale_min = 1.0
        scale_min = max(0.0, scale_min)

        # Scale max/min limits first, then compute turn-based speed from scaled limits.
        speed_max = float(self.default_speed) * scale_max
        speed_min = float(self.turn_speed_min) * scale_min
        if not self.enable_turn_speed_control:
            return speed_max
        if not np.isfinite(speed_max):
            speed_max = 0.0
        if not np.isfinite(speed_min):
            speed_min = 0.0
        if speed_min > speed_max:
            speed_min, speed_max = speed_max, speed_min

        a = abs(float(angle_deg))
        low = max(0.0, float(self.turn_angle_low_deg))
        high = max(low + 1e-3, float(self.turn_angle_high_deg))
        exp = max(0.1, float(self.turn_speed_exponent))
        if a <= low:
            return speed_max
        if a >= high:
            return speed_min
        t = (a - low) / (high - low)
        t = float(np.clip(t, 0.0, 1.0)) ** exp
        return float(speed_max - (speed_max - speed_min) * t)

    def _apply_edge_curve_compensation(
        self,
        angle_deg: float,
        lookahead_point: Optional[Point],
        frame_shape: Sequence[int],
    ) -> float:
        if not self.enable_edge_curve_compensation:
            return float(angle_deg)
        if lookahead_point is None:
            return float(angle_deg)
        if abs(float(angle_deg)) < float(self.edge_comp_min_angle_deg):
            return float(angle_deg)

        h, w = int(frame_shape[0]), int(frame_shape[1])
        if h <= 0 or w <= 0:
            return float(angle_deg)
        lx, _ = lookahead_point
        base_margin = max(1, int(self.edge_comp_margin_px))
        left_margin = base_margin if int(self.edge_comp_margin_left_px) <= 0 else int(self.edge_comp_margin_left_px)
        right_margin = base_margin if int(self.edge_comp_margin_right_px) <= 0 else int(self.edge_comp_margin_right_px)
        left_margin = max(1, left_margin)
        right_margin = max(1, right_margin)
        boost_max = max(0.0, float(self.edge_comp_boost_deg))
        exponent = max(0.1, float(self.edge_comp_exponent))

        dist_left = float(lx)
        dist_right = float((w - 1) - lx)
        proximity = 0.0
        if angle_deg < 0.0 and dist_left < left_margin:
            proximity = (left_margin - dist_left) / float(left_margin)
        elif angle_deg > 0.0 and dist_right < right_margin:
            proximity = (right_margin - dist_right) / float(right_margin)
        if proximity <= 0.0:
            return float(angle_deg)

        boost = boost_max * (float(np.clip(proximity, 0.0, 1.0)) ** exponent)
        return float(angle_deg + np.sign(angle_deg) * boost)

    def _register_shutdown_handler(self):
        try:
            rclpy.get_default_context().on_shutdown(self._publish_stop)
        except Exception as exc:
            self.get_logger().warn(f"Failed to register shutdown handler: {exc}")

    def _publish_stop(self):
        if self._stop_published:
            return
        self._stop_published = True
        try:
            repeats = max(1, int(self.shutdown_stop_repeats))
            interval = max(0.0, float(self.shutdown_stop_interval_sec))
            angle_msg = Float32()
            angle_msg.data = 0.0
            speed_msg = Float32()
            speed_msg.data = 0.0
            for i in range(repeats):
                self.angle_pub.publish(angle_msg)
                self.speed_pub.publish(speed_msg)
                if i < repeats - 1 and interval > 0.0:
                    time.sleep(interval)
            self.get_logger().info(
                f"Published shutdown stop burst (steer=0, speed=0, repeats={repeats}, interval={interval:.3f}s)"
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish stop: {exc}")

    def publish_stop(self):
        self._publish_stop()

    def _on_mask(self, msg: Image):
        mask = _image_to_numpy(msg)
        if mask is None:
            return
        if mask.ndim == 3:
            mask = mask[:, :, 0]
        if mask.dtype != np.float32 and mask.dtype != np.float64:
            mask = mask.astype(np.float32) / 255.0
        else:
            mask = mask.astype(np.float32)
        self._last_mask = mask
        self._last_shape = mask.shape[:2]

    def _on_overlay(self, msg: Image):
        if not self.publish_debug_image or cv2 is None:
            return
        img = _image_to_bgr(msg)
        if img is None:
            return
        self._last_overlay = img
        self._last_overlay_header = msg.header

    def _on_parking_mode(self, msg: Bool):
        self._parking_mode_enabled = bool(msg.data)
        if not self._parking_mode_enabled:
            self._parking_suppress_logged = False
            self._suppress_reason = ""

    def _on_behavior_mode(self, msg: String):
        self._behavior_mode = str(msg.data).strip().upper()
        if self._behavior_mode not in ("PARKING_IN", "PARKING_OUT", "INTERSECTION"):
            self._parking_suppress_logged = False
            self._suppress_reason = ""

    def _on_intersection_status(self, msg: String):
        self._intersection_state = str(msg.data).strip().upper()
        if self._intersection_state not in ("RUNNING", "SETTLE"):
            self._parking_suppress_logged = False
            self._suppress_reason = ""

    def _resolve_frame_shape(self) -> Optional[Sequence[int]]:
        if self._last_shape is not None:
            return self._last_shape
        if self.image_height > 0 and self.image_width > 0:
            return (self.image_height, self.image_width)
        return None

    def _prepare_base_image(self, frame_shape: Sequence[int]) -> Optional[np.ndarray]:
        h, w = int(frame_shape[0]), int(frame_shape[1])
        if self._last_overlay is not None:
            img = self._last_overlay
            if img.shape[0] != h or img.shape[1] != w:
                if cv2 is None:
                    return None
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            return img.copy()
        if self._last_mask is not None:
            m = self._last_mask
            if m.shape[0] != h or m.shape[1] != w:
                if cv2 is None:
                    return None
                m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
            m_u8 = (np.clip(m, 0.0, 1.0) * 255.0).astype(np.uint8)
            vis = np.zeros((h, w, 3), dtype=np.uint8)
            vis[:, :, 1] = m_u8
            return vis
        return np.zeros((h, w, 3), dtype=np.uint8)

    def _draw_debug(
        self,
        base: np.ndarray,
        points: Sequence[Point],
        lookahead: Optional[Point],
        angle_out: float,
        angle_cmd: float,
        speed_cmd: float,
    ) -> np.ndarray:
        if cv2 is None:
            return base
        out = base.copy()
        if self.draw_centerline and len(points) >= 2:
            pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], False, self.debug_centerline_color, self.debug_line_thickness)
        h, w = out.shape[:2]
        if self.draw_vehicle:
            cv2.circle(out, (int(w * 0.5), h - 1), self.debug_point_radius, self.debug_vehicle_color, -1)
        if self.draw_lookahead and lookahead is not None:
            lx, ly = lookahead
            cv2.circle(out, (int(lx), int(ly)), self.debug_point_radius + 1, self.debug_lookahead_color, -1)
            if self.draw_vehicle:
                cv2.line(out, (int(w * 0.5), h - 1), (int(lx), int(ly)), self.debug_line_color, 1)
        if self.draw_angle_text:
            font = cv2.FONT_HERSHEY_TRIPLEX
            text3 = f"Steer cmd: {angle_cmd:.1f}"
            cv2.putText(
                out,
                text3,
                (10, 35),
                font,
                self.debug_text_scale,
                self.debug_text_color,
                self.debug_text_thickness,
                cv2.LINE_AA,
            )
            text4 = f"Speed cmd: {speed_cmd:.1f}"
            cv2.putText(
                out,
                text4,
                (10, 65),
                font,
                self.debug_text_scale,
                self.debug_text_color,
                self.debug_text_thickness,
                cv2.LINE_AA,
            )
        return out

    def _on_centerline(self, msg: Path):
        suppress = False
        suppress_reason = ""
        if self.disable_when_parking_mode and self._parking_mode_enabled:
            suppress = True
            suppress_reason = "parking_mode"
        elif self.disable_when_behavior_parking_out and self._behavior_mode in ("PARKING_IN", "PARKING_OUT"):
            suppress = True
            suppress_reason = f"behavior_mode={self._behavior_mode}"
        elif self.disable_when_behavior_intersection and self._behavior_mode == "INTERSECTION":
            suppress = True
            suppress_reason = "behavior_mode=INTERSECTION"
        elif self.disable_when_intersection_active and self._intersection_state in ("RUNNING", "SETTLE"):
            suppress = True
            suppress_reason = f"intersection_state={self._intersection_state}"
        if suppress:
            if (not self._parking_suppress_logged) or (suppress_reason != self._suppress_reason):
                self.get_logger().info(
                    f"pure_pursuit command publishing suppressed ({suppress_reason}); "
                    "centerline tracking stays active for smooth takeover."
                )
                self._parking_suppress_logged = True
                self._suppress_reason = suppress_reason

        points = _path_to_points(msg)
        if len(points) < 2:
            return
        frame_shape = self._resolve_frame_shape()
        if frame_shape is None:
            self.get_logger().warn("Missing frame shape. Provide mask or image_height/image_width.")
            return

        mask = self._last_mask if self.use_mask else None
        effective_lookahead = float(self.lookahead)
        if self.adaptive_lookahead_enable:
            h = max(1, int(frame_shape[0]))
            ys = [p[1] for p in points]
            span_px = max(0.0, float(max(ys) - min(ys)))
            span_ratio = float(np.clip(span_px / float(h), 0.0, 1.0))
            candidate = span_ratio * float(self.adaptive_lookahead_span_scale)
            candidate = max(float(self.adaptive_lookahead_min), candidate)
            effective_lookahead = float(np.clip(min(float(self.lookahead), candidate), 0.05, 0.95))
        angle_deg, lookahead_point = compute_steering_angle(
            points,
            frame_shape,
            lookahead=effective_lookahead,
            method=self.method,
            wheelbase=self.wheelbase,
            mask=mask,
            lane_width_cm=self.lane_width_cm,
            mask_threshold=self.mask_threshold,
            mask_min_width=self.mask_min_width,
            mask_min_pixels=self.mask_min_pixels,
            wheelbase_cm=self.wheelbase_cm,
            max_angle=self.max_angle_deg,
        )
        if angle_deg is None:
            return

        angle_deg = float(angle_deg - self.steer_bias_deg)
        if abs(angle_deg) <= self.steer_deadband:
            angle_deg = 0.0
        angle_deg = self._apply_edge_curve_compensation(angle_deg, lookahead_point, frame_shape)
        if np.isfinite(self.max_angle_deg) and self.max_angle_deg > 0:
            angle_deg = float(np.clip(angle_deg, -self.max_angle_deg, self.max_angle_deg))
        angle_deg = self._apply_steer_kalman(angle_deg)
        if 0.0 < self.steer_smooth < 1.0:
            if self._steer_ema is None:
                self._steer_ema = angle_deg
            else:
                self._steer_ema = self.steer_smooth * angle_deg + (1.0 - self.steer_smooth) * self._steer_ema
            angle_deg = float(self._steer_ema)
        angle_out_raw = angle_deg if self.angle_in_degrees else float(np.deg2rad(angle_deg))
        angle_out_pub = float(angle_out_raw * self.steer_output_scale * self.additional_weight)
        if np.isfinite(self.steer_cmd_limit) and self.steer_cmd_limit > 0.0:
            angle_out_pub = float(np.clip(angle_out_pub, -self.steer_cmd_limit, self.steer_cmd_limit))

        speed_cmd = self._compute_speed_cmd(angle_deg)
        if not suppress:
            msg_out = Float32()
            msg_out.data = float(angle_out_pub)
            self.angle_pub.publish(msg_out)

            speed_msg = Float32()
            speed_msg.data = float(speed_cmd)
            self.speed_pub.publish(speed_msg)
            if self.log_angle:
                now_ns = int(self.get_clock().now().nanoseconds)
                if self._last_log_ns == 0 or (now_ns - self._last_log_ns) >= int(self.log_period * 1e9):
                    self.get_logger().info(
                        f"steering_angle_raw={angle_out_raw:.2f} steering_angle_cmd={angle_out_pub:.2f} speed_cmd={speed_cmd:.2f}"
                    )
                    self._last_log_ns = now_ns

        if self.publish_debug_image and cv2 is not None:
            base = self._prepare_base_image(frame_shape)
            if base is None:
                return
            debug = self._draw_debug(
                base,
                points,
                lookahead_point,
                angle_out_raw,
                angle_out_pub,
                float(speed_cmd),
            )
            header = msg.header if self._last_overlay_header is None else self._last_overlay_header
            debug_msg = _numpy_to_image_msg(debug, "bgr8", header)
            self.debug_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        finally:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
