#!/usr/bin/env python3
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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


def _mask_to_binary(mask: np.ndarray, threshold: float) -> np.ndarray:
    if mask.dtype == np.bool_ or mask.dtype == bool:
        return mask
    return mask > float(threshold)


def _image_to_bgr(msg: Image) -> Optional[np.ndarray]:
    if msg.height == 0 or msg.width == 0:
        return None
    encoding = msg.encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        channels = 3
    elif encoding in ("mono8", "8uc1"):
        channels = 1
    else:
        return None
    row_bytes = msg.width * channels
    if msg.step < row_bytes:
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < msg.step * msg.height:
        return None
    data = data[: msg.step * msg.height]
    img = data.reshape((msg.height, msg.step))
    img = img[:, :row_bytes]
    if channels == 1:
        mono = img.reshape((msg.height, msg.width))
        return np.stack([mono, mono, mono], axis=-1).copy()
    img = img.reshape((msg.height, msg.width, 3))
    if encoding == "rgb8":
        img = img[:, :, ::-1]
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


def _parse_color(value: str) -> Tuple[int, int, int]:
    parts = [p.strip() for p in str(value).split(",") if p.strip()]
    if len(parts) != 3:
        return (0, 255, 255)
    try:
        r, g, b = (int(float(p)) for p in parts)
    except Exception:
        return (0, 255, 255)
    return (int(np.clip(r, 0, 255)), int(np.clip(g, 0, 255)), int(np.clip(b, 0, 255)))


def _keep_longest_run(points: Sequence[Point], max_gap: int) -> List[Point]:
    if len(points) <= 2:
        return list(points)
    runs = []
    start = 0
    for i in range(1, len(points)):
        if (points[i][1] - points[i - 1][1]) > max_gap:
            runs.append((start, i))
            start = i
    runs.append((start, len(points)))
    best = max(runs, key=lambda r: r[1] - r[0])
    return list(points[best[0]:best[1]])


def _smooth_centerline(points: Sequence[Point], window: int) -> List[Point]:
    window = max(1, int(window))
    if window <= 1 or len(points) < window:
        return list(points)
    xs = np.array([p[0] for p in points], dtype=np.float32)
    ys = [p[1] for p in points]
    kernel = np.ones(window, dtype=np.float32) / window
    pad = window // 2
    xs_pad = np.pad(xs, (pad, pad), mode="edge")
    xs_s = np.convolve(xs_pad, kernel, mode="valid")
    return [(int(round(x)), y) for x, y in zip(xs_s, ys)]


def _ransac_filter_points(
    points: Sequence[Point],
    iterations: int,
    residual_threshold: float,
    min_inlier_ratio: float,
    min_inliers: int,
    seed: int,
) -> List[Point]:
    if len(points) < 6:
        return list(points)

    pts = sorted(points, key=lambda p: p[1])
    ys = np.array([p[1] for p in pts], dtype=np.float32)
    xs = np.array([p[0] for p in pts], dtype=np.float32)
    n = len(pts)
    sample_size = 3  # quadratic model x = f(y)
    if n < sample_size:
        return pts

    rng = np.random.default_rng(int(seed))
    best_inliers = None
    best_count = 0
    best_err = float("inf")
    iters = max(1, int(iterations))
    thresh = max(0.5, float(residual_threshold))

    for _ in range(iters):
        sample_idx = rng.choice(n, size=sample_size, replace=False)
        try:
            coeffs = np.polyfit(ys[sample_idx], xs[sample_idx], deg=2)
        except Exception:
            continue
        pred = np.polyval(coeffs, ys)
        residual = np.abs(pred - xs)
        inliers = residual <= thresh
        count = int(np.count_nonzero(inliers))
        if count == 0:
            continue
        err = float(np.mean(residual[inliers]))
        if count > best_count or (count == best_count and err < best_err):
            best_count = count
            best_err = err
            best_inliers = inliers

    if best_inliers is None:
        return pts

    min_needed = max(int(min_inliers), int(np.ceil(n * max(0.0, min(1.0, min_inlier_ratio)))))
    if best_count < max(sample_size, min_needed):
        return pts

    inlier_pts = [p for p, keep in zip(pts, best_inliers) if keep]
    if len(inlier_pts) < sample_size:
        return pts
    return inlier_pts


def _natural_cubic_spline_1d(y: np.ndarray, x: np.ndarray, y_query: np.ndarray) -> np.ndarray:
    n = y.size
    if n < 3:
        return np.interp(y_query, y, x)

    h = np.diff(y)
    if np.any(h <= 0):
        return np.interp(y_query, y, x)

    alpha = np.zeros(n, dtype=np.float64)
    for i in range(1, n - 1):
        alpha[i] = (3.0 / h[i]) * (x[i + 1] - x[i]) - (3.0 / h[i - 1]) * (x[i] - x[i - 1])

    l = np.ones(n, dtype=np.float64)
    mu = np.zeros(n, dtype=np.float64)
    z = np.zeros(n, dtype=np.float64)

    for i in range(1, n - 1):
        l[i] = 2.0 * (y[i + 1] - y[i - 1]) - h[i - 1] * mu[i - 1]
        if abs(l[i]) < 1e-6:
            return np.interp(y_query, y, x)
        mu[i] = h[i] / l[i]
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i]

    b = np.zeros(n - 1, dtype=np.float64)
    c = np.zeros(n, dtype=np.float64)
    d = np.zeros(n - 1, dtype=np.float64)

    for j in range(n - 2, -1, -1):
        c[j] = z[j] - mu[j] * c[j + 1]
        b[j] = (x[j + 1] - x[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j])

    out = np.empty_like(y_query, dtype=np.float64)
    seg = np.searchsorted(y, y_query, side="right") - 1
    seg = np.clip(seg, 0, n - 2)
    dy = y_query - y[seg]
    out[:] = x[seg] + b[seg] * dy + c[seg] * dy * dy + d[seg] * dy * dy * dy
    return out


def _spline_centerline(points: Sequence[Point], samples: int) -> List[Point]:
    if len(points) < 4:
        return list(points)

    pts = sorted(points, key=lambda p: p[1])
    ys = np.array([p[1] for p in pts], dtype=np.float64)
    xs = np.array([p[0] for p in pts], dtype=np.float64)

    y_unique, unique_idx = np.unique(ys, return_index=True)
    x_unique = xs[unique_idx]
    if y_unique.size < 4:
        return pts

    sample_count = int(samples)
    if sample_count <= 0:
        y_query = y_unique
    else:
        sample_count = max(4, sample_count)
        y_query = np.linspace(float(y_unique[0]), float(y_unique[-1]), sample_count)

    x_query = _natural_cubic_spline_1d(y_unique, x_unique, y_query)
    return [(int(round(xv)), int(round(yv))) for xv, yv in zip(x_query, y_query)]


def _extract_centerline_points(
    mask: np.ndarray,
    step: int,
    min_width: int,
    min_pixels: int,
    threshold: float,
    smooth: int,
    max_width_ratio: float,
) -> List[Point]:
    if mask is None or mask.ndim != 2:
        return []
    mask_bin = _mask_to_binary(mask, threshold)
    h, w = mask_bin.shape
    max_width_px = int(round(float(np.clip(max_width_ratio, 0.05, 1.0)) * float(w)))
    points: List[Point] = []
    step = max(1, int(step))
    for y in range(0, h, step):
        row = mask_bin[y]
        xs = np.flatnonzero(row)
        if xs.size < min_pixels:
            continue
        if xs.size == 1:
            run_x0 = run_x1 = xs[0]
        else:
            gaps = np.where(np.diff(xs) > 1)[0]
            if gaps.size == 0:
                run_x0, run_x1 = xs[0], xs[-1]
            else:
                starts = np.r_[0, gaps + 1]
                ends = np.r_[gaps, xs.size - 1]
                lengths = ends - starts + 1
                best = int(np.argmax(lengths))
                run_x0 = xs[starts[best]]
                run_x1 = xs[ends[best]]
        run_width = int(run_x1 - run_x0 + 1)
        if run_width < min_width:
            continue
        # Reject over-wide rows (e.g. large false-positive blobs near vehicle).
        if run_width > max_width_px:
            continue
        x_center = int(round((run_x0 + run_x1) * 0.5))
        if 0 <= x_center < w:
            points.append((x_center, y))

    points = _keep_longest_run(points, max_gap=max(1, int(step * 1.5)))
    points = _smooth_centerline(points, window=smooth)
    return points


class CenterlineNode(Node):
    def __init__(self):
        super().__init__("path_planning_centerline")

        self.declare_parameter("mask_topic", "/lane/mask")
        self.declare_parameter("centerline_topic", "/lane/centerline")
        self.declare_parameter("overlay_topic", "/lane/centerline_image")
        self.declare_parameter("background_topic", "")
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("centerline_threshold", 0.5)
        self.declare_parameter("centerline_step", 4)
        self.declare_parameter("centerline_min_width", 6)
        self.declare_parameter("centerline_min_pixels", 20)
        self.declare_parameter("centerline_smooth", 7)
        self.declare_parameter("centerline_max_width_ratio", 0.70)
        self.declare_parameter("centerline_use_ransac", True)
        self.declare_parameter("centerline_ransac_iterations", 50)
        self.declare_parameter("centerline_ransac_residual_threshold", 8.0)
        self.declare_parameter("centerline_ransac_min_inlier_ratio", 0.5)
        self.declare_parameter("centerline_ransac_min_inliers", 8)
        self.declare_parameter("centerline_ransac_seed", 7)
        self.declare_parameter("centerline_use_spline", True)
        self.declare_parameter("centerline_spline_samples", 0)
        self.declare_parameter("centerline_color", "0,255,255")
        self.declare_parameter("centerline_thickness", 2)
        self.declare_parameter("frame_id", "")

        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.centerline_topic = str(self.get_parameter("centerline_topic").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.background_topic = str(self.get_parameter("background_topic").value)
        self.publish_overlay = bool(self.get_parameter("publish_overlay").value)
        self.threshold = float(self.get_parameter("centerline_threshold").value)
        self.step = int(self.get_parameter("centerline_step").value)
        self.min_width = int(self.get_parameter("centerline_min_width").value)
        self.min_pixels = int(self.get_parameter("centerline_min_pixels").value)
        self.smooth = int(self.get_parameter("centerline_smooth").value)
        self.max_width_ratio = float(self.get_parameter("centerline_max_width_ratio").value)
        self.use_ransac = bool(self.get_parameter("centerline_use_ransac").value)
        self.ransac_iterations = int(self.get_parameter("centerline_ransac_iterations").value)
        self.ransac_residual_threshold = float(self.get_parameter("centerline_ransac_residual_threshold").value)
        self.ransac_min_inlier_ratio = float(self.get_parameter("centerline_ransac_min_inlier_ratio").value)
        self.ransac_min_inliers = int(self.get_parameter("centerline_ransac_min_inliers").value)
        self.ransac_seed = int(self.get_parameter("centerline_ransac_seed").value)
        self.use_spline = bool(self.get_parameter("centerline_use_spline").value)
        self.spline_samples = int(self.get_parameter("centerline_spline_samples").value)
        self.centerline_color = _parse_color(self.get_parameter("centerline_color").value)
        self.centerline_thickness = int(self.get_parameter("centerline_thickness").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, self.mask_topic, self._on_mask, qos)
        self.bg_sub = None
        self._background = None
        if self.background_topic:
            self.bg_sub = self.create_subscription(Image, self.background_topic, self._on_background, qos)
        self.pub = self.create_publisher(Path, self.centerline_topic, 10)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, qos)

        self.get_logger().info(f"Subscribed to {self.mask_topic}")
        self.get_logger().info(f"Publishing centerline -> {self.centerline_topic}")
        if self.publish_overlay:
            self.get_logger().info(f"Publishing overlay -> {self.overlay_topic}")
        self.get_logger().info(
            f"RANSAC={'on' if self.use_ransac else 'off'} "
            f"(iters={self.ransac_iterations}, residual={self.ransac_residual_threshold})"
        )
        self.get_logger().info(
            f"Spline={'on' if self.use_spline else 'off'} "
            f"(samples={'input' if self.spline_samples <= 0 else self.spline_samples})"
        )
        if self.background_topic:
            self.get_logger().info(f"Using background image -> {self.background_topic}")

    def _on_background(self, msg: Image):
        bg = _image_to_bgr(msg)
        if bg is None:
            return
        self._background = bg

    def _on_mask(self, msg: Image):
        mask = _image_to_numpy(msg)
        if mask is None:
            self.get_logger().warn("Unsupported mask encoding or empty mask")
            return
        if mask.ndim == 3:
            mask = mask[:, :, 0]

        if mask.dtype != np.float32 and mask.dtype != np.float64:
            mask_f = mask.astype(np.float32) / 255.0
        else:
            mask_f = mask.astype(np.float32)

        points = _extract_centerline_points(
            mask_f,
            step=self.step,
            min_width=self.min_width,
            min_pixels=self.min_pixels,
            threshold=self.threshold,
            smooth=self.smooth,
            max_width_ratio=self.max_width_ratio,
        )
        if self.use_ransac and points:
            points = _ransac_filter_points(
                points,
                iterations=self.ransac_iterations,
                residual_threshold=self.ransac_residual_threshold,
                min_inlier_ratio=self.ransac_min_inlier_ratio,
                min_inliers=self.ransac_min_inliers,
                seed=self.ransac_seed,
            )
        if self.use_spline and points:
            points = _spline_centerline(points, samples=self.spline_samples)
        if points:
            h, w = mask_f.shape[:2]
            points = [(int(np.clip(x, 0, w - 1)), int(np.clip(y, 0, h - 1))) for x, y in points]
        if not points:
            return

        header = msg.header
        if self.frame_id:
            header = msg.header
            header.frame_id = self.frame_id

        path = Path()
        path.header = header
        for x, y in points:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub.publish(path)

        if self.publish_overlay and cv2 is not None:
            h, w = mask_f.shape[:2]
            vis = None
            if self._background is not None:
                if self._background.shape[0] != h or self._background.shape[1] != w:
                    vis = cv2.resize(self._background, (w, h), interpolation=cv2.INTER_LINEAR)
                else:
                    vis = self._background.copy()
            if vis is None:
                vis = np.zeros((h, w, 3), dtype=np.uint8)
                m = np.clip(mask_f * 255.0, 0, 255).astype(np.uint8)
                vis[:, :, 1] = m
            if len(points) >= 2:
                pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                color_bgr = (self.centerline_color[2], self.centerline_color[1], self.centerline_color[0])
                cv2.polylines(vis, [pts], False, color_bgr, int(self.centerline_thickness))
            elif len(points) == 1:
                color_bgr = (self.centerline_color[2], self.centerline_color[1], self.centerline_color[0])
                cv2.circle(vis, (points[0][0], points[0][1]), 2, color_bgr, -1)
            out = _numpy_to_image_msg(vis, "bgr8", header)
            self.overlay_pub.publish(out)


def main():
    rclpy.init()
    node = CenterlineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
