import os
import sys
import time
import threading
from pathlib import Path


def configure_local_gstreamer():
    base = Path.home() / 'Desktop' / 'gst_local' / 'extract'
    plugin_dirs = [
        base / 'plugins_bad' / 'usr' / 'lib' / 'x86_64-linux-gnu' / 'gstreamer-1.0',
        base / 'libav' / 'usr' / 'lib' / 'x86_64-linux-gnu' / 'gstreamer-1.0',
    ]
    helper_lib_dir = base / 'bad' / 'usr' / 'lib' / 'x86_64-linux-gnu'

    existing_plugins = [str(path) for path in plugin_dirs if path.is_dir()]
    if existing_plugins:
        current = os.environ.get('GST_PLUGIN_PATH', '')
        parts = existing_plugins + ([current] if current else [])
        os.environ['GST_PLUGIN_PATH'] = ':'.join(parts)
        os.environ.setdefault('GST_REGISTRY', '/tmp/gst-local-registry.bin')

    if helper_lib_dir.is_dir():
        current = os.environ.get('LD_LIBRARY_PATH', '')
        parts = [str(helper_lib_dir)] + ([current] if current else [])
        os.environ['LD_LIBRARY_PATH'] = ':'.join(parts)


configure_local_gstreamer()

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


def has_gstreamer_support():
    info = cv2.getBuildInformation()
    for line in info.splitlines():
        if 'GStreamer' in line:
            return line.strip().endswith('YES') or 'YES (' in line
    return False


if not has_gstreamer_support():
    if os.environ.get('PYTHONNOUSERSITE') != '1':
        new_env = os.environ.copy()
        new_env['PYTHONNOUSERSITE'] = '1'
        os.execvpe(sys.executable, [sys.executable, *sys.argv], new_env)

    raise RuntimeError(
        f'OpenCV was loaded without GStreamer support from {cv2.__file__}. '
        'Install or use a GStreamer-enabled OpenCV build.'
    )


def build_pipeline(port):
    return (
        f'udpsrc port={port} ! '
        'h264parse ! '
        'avdec_h264 ! '
        'videoconvert ! '
        'appsink max-buffers=1 drop=true sync=false'
    )


def parse_reliability(value):
    if value == 'best_effort':
        return QoSReliabilityPolicy.BEST_EFFORT
    if value == 'reliable':
        return QoSReliabilityPolicy.RELIABLE
    raise ValueError(
        f"Unsupported reliability '{value}'. Use 'reliable' or 'best_effort'."
    )


class UdpCameraPublisher(Node):
    def __init__(self):
        super().__init__('udp_camera_publisher')

        self.declare_parameter('port', 5000)
        self.declare_parameter('topic', 'image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('retry_interval', 2.0)
        self.declare_parameter('reliability', 'reliable')
        self.declare_parameter('qos_depth', 1)
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('resize_height', 0)
        self.declare_parameter('stats_interval', 5.0)

        self._port = int(self.get_parameter('port').value)
        self._topic = str(self.get_parameter('topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._retry_interval = float(self.get_parameter('retry_interval').value)
        self._reliability_name = str(self.get_parameter('reliability').value).lower()
        self._reliability = parse_reliability(self._reliability_name)
        self._qos_depth = max(1, int(self.get_parameter('qos_depth').value))
        self._resize_width = max(0, int(self.get_parameter('resize_width').value))
        self._resize_height = max(0, int(self.get_parameter('resize_height').value))
        self._stats_interval = max(0.0, float(self.get_parameter('stats_interval').value))

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self._qos_depth,
            reliability=self._reliability,
        )
        self._publisher = self.create_publisher(Image, self._topic, qos)

        self._pipeline = build_pipeline(self._port)
        self._capture = None
        self._running = True
        self._last_read_warning_time = 0.0
        self._captured_frames = 0
        self._published_frames = 0
        self._stats_lock = threading.Lock()

        self._worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker.start()
        self._stats_timer = None
        if self._stats_interval > 0.0:
            self._stats_timer = self.create_timer(self._stats_interval, self._log_stats)

        self.get_logger().info(
            f'Publishing UDP stream from port {self._port} to topic {self._topic} '
            f'with {self._reliability_name} QoS (depth={self._qos_depth})'
        )
        if self._resize_width > 0 and self._resize_height > 0:
            self.get_logger().info(
                f'Resizing published frames to {self._resize_width}x{self._resize_height}'
            )

    def _open_capture(self):
        if self._capture is not None:
            self._capture.release()

        self._capture = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
        if not self._capture.isOpened():
            self.get_logger().warning(
                f'Failed to open UDP stream on port {self._port}. '
                f'Retrying in {self._retry_interval:.1f}s.'
            )
            self._capture.release()
            self._capture = None
            time.sleep(self._retry_interval)
            return False

        self.get_logger().info('UDP camera stream opened successfully.')
        return True

    def _prepare_frame(self, frame):
        if self._resize_width > 0 and self._resize_height > 0:
            return cv2.resize(frame, (self._resize_width, self._resize_height))
        return frame

    def _publish_frame(self, frame):
        frame = self._prepare_frame(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        try:
            self._publisher.publish(msg)
        except Exception:
            if not self._running or not rclpy.ok():
                return
            raise
        with self._stats_lock:
            self._published_frames += 1

    def _log_stats(self):
        with self._stats_lock:
            captured = self._captured_frames
            published = self._published_frames
            self._captured_frames = 0
            self._published_frames = 0

        captured_fps = captured / self._stats_interval
        published_fps = published / self._stats_interval
        self.get_logger().info(
            f'Capture FPS: {captured_fps:.1f}, published FPS: {published_fps:.1f}'
        )

    def _capture_loop(self):
        while self._running and rclpy.ok():
            if self._capture is None and not self._open_capture():
                continue

            ret, frame = self._capture.read()
            if not ret:
                now = time.monotonic()
                if now - self._last_read_warning_time > 2.0:
                    self.get_logger().warning(
                        'Failed to read a frame from the UDP stream. Reopening capture.'
                    )
                    self._last_read_warning_time = now

                self._capture.release()
                self._capture = None
                time.sleep(0.2)
                continue

            with self._stats_lock:
                self._captured_frames += 1
            self._publish_frame(frame)

    def destroy_node(self):
        self._running = False
        if self._stats_timer is not None:
            self._stats_timer.cancel()
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpCameraPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
