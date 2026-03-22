from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[int, int]


def _mask_to_binary(mask: np.ndarray, threshold: float) -> np.ndarray:
    if mask.dtype == np.bool_ or mask.dtype == bool:
        return mask
    return mask > float(threshold)


def _centerline_points(
    mask: np.ndarray,
    step: int = 4,
    min_width: int = 6,
    min_pixels: int = 20,
    threshold: float = 0.5,
) -> List[Point]:
    if mask is None or mask.ndim != 2:
        return []
    mask_bin = _mask_to_binary(mask, threshold)
    h, w = mask_bin.shape
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
        if (run_x1 - run_x0 + 1) < min_width:
            continue
        x_center = int(round((run_x0 + run_x1) * 0.5))
        if 0 <= x_center < w:
            points.append((x_center, y))
    return points


def _keep_longest_run(points: Sequence[Point], max_gap: int = 6) -> List[Point]:
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


def _smooth_centerline(points: Sequence[Point], window: int = 7) -> List[Point]:
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


def extract_centerline(
    mask: np.ndarray,
    step: int = 4,
    min_width: int = 6,
    min_pixels: int = 20,
    threshold: float = 0.5,
    smooth: int = 7,
) -> List[Point]:
    points = _centerline_points(mask, step=step, min_width=min_width, min_pixels=min_pixels, threshold=threshold)
    points = _keep_longest_run(points, max_gap=max(1, int(step * 1.5)))
    points = _smooth_centerline(points, window=smooth)
    return points


def lane_width_at_row(
    mask: np.ndarray,
    y: float,
    threshold: float = 0.5,
    min_width: int = 6,
    min_pixels: int = 20,
) -> Optional[Tuple[int, int]]:
    if mask is None or mask.ndim != 2:
        return None
    mask_bin = _mask_to_binary(mask, threshold)
    h, w = mask_bin.shape
    y = int(round(y))
    if y < 0 or y >= h:
        return None
    row = mask_bin[y]
    xs = np.flatnonzero(row)
    if xs.size < min_pixels:
        return None
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
    width = int(run_x1 - run_x0 + 1)
    if width < min_width:
        return None
    center_x = int(round((run_x0 + run_x1) * 0.5))
    if center_x < 0 or center_x >= w:
        return None
    return width, center_x


def centerline_x_at_row(points: Sequence[Point], y_target: float) -> Optional[float]:
    if points is None or len(points) == 0:
        return None
    pts = sorted(points, key=lambda p: p[1])
    y_target = float(y_target)
    if y_target <= pts[0][1]:
        return float(pts[0][0])
    if y_target >= pts[-1][1]:
        return float(pts[-1][0])
    for i in range(len(pts) - 1):
        y0 = float(pts[i][1])
        y1 = float(pts[i + 1][1])
        if y0 <= y_target <= y1:
            if abs(y1 - y0) < 1e-6:
                return float(pts[i][0])
            t = (y_target - y0) / (y1 - y0)
            return float(pts[i][0] + t * (pts[i + 1][0] - pts[i][0]))
    return float(pts[-1][0])


def pick_lookahead_point(
    points: Sequence[Point],
    frame_shape: Sequence[int],
    lookahead: float = 0.3,
) -> Tuple[Optional[Tuple[float, float]], Optional[float]]:
    if points is None or len(points) < 2:
        return None, None
    h = int(frame_shape[0])
    w = int(frame_shape[1])
    lookahead = float(lookahead)
    if not np.isfinite(lookahead):
        lookahead = 0.3
    lookahead = max(0.05, min(0.95, lookahead))
    pts = np.array(points, dtype=np.float32)
    order = np.argsort(pts[:, 1])
    pts = pts[order][::-1]  # bottom -> top
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    if seg.size == 0:
        return None, None
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    Ld = lookahead * float(h)
    if cum[-1] <= 1e-6:
        target = pts[0]
    elif Ld >= cum[-1]:
        target = pts[-1]
    else:
        idx = int(np.searchsorted(cum, Ld))
        idx = max(1, min(idx, len(pts) - 1))
        t = (Ld - cum[idx - 1]) / max(1e-6, (cum[idx] - cum[idx - 1]))
        target = pts[idx - 1] + t * (pts[idx] - pts[idx - 1])
    target[0] = np.clip(target[0], 0, w - 1)
    target[1] = np.clip(target[1], 0, h - 1)
    return (float(target[0]), float(target[1])), float(Ld)


def compute_steering_angle(
    points: Sequence[Point],
    frame_shape: Sequence[int],
    lookahead: float = 0.3,
    method: str = "center_offset",
    wheelbase: float = 0.25,
    mask: Optional[np.ndarray] = None,
    lane_width_cm: float = 0.0,
    mask_threshold: float = 0.5,
    mask_min_width: int = 6,
    mask_min_pixels: int = 20,
    wheelbase_cm: float = 26.7,
    max_angle: float = 25.0,
) -> Tuple[Optional[float], Optional[Point]]:
    if points is None or len(points) < 2:
        return None, None
    h = int(frame_shape[0])
    w = int(frame_shape[1])
    method = (method or "center_offset").lower()
    if method == "center_offset":
        lookahead = float(lookahead)
        if not np.isfinite(lookahead):
            lookahead = 0.3
        lookahead = max(0.05, min(0.95, lookahead))
        y_look = int(round((h - 1) - lookahead * h))
        x_look = centerline_x_at_row(points, y_look)
        if x_look is None:
            return None, None
        x_vehicle = (w - 1) * 0.5
        dx = x_look - x_vehicle
        max_angle = float(max_angle) if np.isfinite(max_angle) else 25.0
        max_angle = max(1.0, max_angle)
        angle_deg = float(np.clip((dx / (w * 0.5)) * max_angle, -max_angle, max_angle))
        return angle_deg, (int(round(x_look)), int(y_look))

    if method == "lookahead":
        lookahead = float(lookahead)
        if not np.isfinite(lookahead):
            lookahead = 0.3
        lookahead = max(0.05, min(0.95, lookahead))
        y_look = int(round((h - 1) - lookahead * h))
        ys = np.array([p[1] for p in points], dtype=np.float32)
        xs = np.array([p[0] for p in points], dtype=np.float32)
        roi = ys >= y_look
        if np.sum(roi) >= 2:
            ys_fit = ys[roi]
            xs_fit = xs[roi]
        else:
            ys_fit = ys
            xs_fit = xs
        try:
            a, b = np.polyfit(ys_fit, xs_fit, 1)
        except Exception:
            return None, None
        x_look = float(np.clip(a * y_look + b, 0, w - 1))
        x_vehicle = (w - 1) * 0.5
        dy = (h - 1) - y_look
        if dy <= 0:
            return None, None
        dx = x_look - x_vehicle
        angle_rad = np.arctan2(dx, dy)
        angle_deg = float(np.degrees(angle_rad))
        return angle_deg, (int(round(x_look)), int(y_look))

    # Pure pursuit
    target, _ = pick_lookahead_point(points, frame_shape, lookahead=lookahead)
    if target is None:
        return None, None
    x_t, y_t = target
    x_vehicle = (w - 1) * 0.5
    y_vehicle = h - 1
    dx = x_t - x_vehicle
    dy = y_vehicle - y_t
    if dy <= 1.0:
        return None, None
    alpha = np.arctan2(dx, dy)
    Ld = max(1.0, float(np.hypot(dx, dy)))
    wheelbase_px = float(wheelbase)
    if not np.isfinite(wheelbase_px):
        wheelbase_px = 0.25
    wheelbase_px = max(0.05, min(1.0, wheelbase_px)) * float(h)
    if lane_width_cm and lane_width_cm > 0 and mask is not None:
        lw = lane_width_at_row(
            mask,
            y_t,
            threshold=mask_threshold,
            min_width=mask_min_width,
            min_pixels=mask_min_pixels,
        )
        if lw is not None and wheelbase_cm > 0:
            width_px, _ = lw
            px_per_cm = width_px / float(lane_width_cm)
            if px_per_cm > 1e-3:
                dx = dx / px_per_cm
                dy = dy / px_per_cm
                Ld = max(1.0, float(np.hypot(dx, dy)))
                wheelbase = float(wheelbase_cm)
            else:
                wheelbase = wheelbase_px
        else:
            wheelbase = wheelbase_px
    else:
        wheelbase = wheelbase_px
    delta = np.arctan2(2.0 * wheelbase * np.sin(alpha), Ld)
    angle_deg = float(np.degrees(delta))
    return angle_deg, (int(round(x_t)), int(round(y_t)))
