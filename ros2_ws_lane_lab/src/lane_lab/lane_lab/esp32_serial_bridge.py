import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import serial
    from serial import SerialException
except Exception:  # pragma: no cover
    serial = None
    SerialException = Exception


class Esp32SerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('steering_topic', '/steering_cmd')
        self.declare_parameter('speed_topic', '/speed_cmd')
        self.declare_parameter('send_hz', 15)
        self.declare_parameter('command_timeout_sec', 0.3)
        self.declare_parameter('servo_mid', 92)
        self.declare_parameter('servo_deg_per_unit', 1.0)
        self.declare_parameter('servo_min', 52)
        self.declare_parameter('servo_max', 132)
        self.declare_parameter('speed_min', 0)
        self.declare_parameter('speed_max', 255)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('dry_run', False)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.steering_topic = str(self.get_parameter('steering_topic').value)
        self.speed_topic = str(self.get_parameter('speed_topic').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.servo_mid = int(self.get_parameter('servo_mid').value)
        self.servo_deg_per_unit = float(self.get_parameter('servo_deg_per_unit').value)
        self.servo_min = int(self.get_parameter('servo_min').value)
        self.servo_max = int(self.get_parameter('servo_max').value)
        self.speed_min = int(self.get_parameter('speed_min').value)
        self.speed_max = int(self.get_parameter('speed_max').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.latest_speed = None
        self.latest_steer = None
        self.last_cmd_time = 0.0
        self.serial_conn = None
        self.last_status_log_time = 0.0
        self.stop_sent = False

        self.create_subscription(Float32, self.steering_topic, self.steering_callback, 10)
        self.create_subscription(Float32, self.speed_topic, self.speed_callback, 10)
        self.timer = self.create_timer(1.0 / self.send_hz, self.timer_callback)

        if self.dry_run:
            self.get_logger().info('ESP32 serial bridge running in dry-run mode.')
        else:
            self._connect_serial()
            self.get_logger().info('ESP32 serial bridge connected to serial device')

    def steering_callback(self, msg):
        self.latest_steer = msg.data
        self.last_cmd_time = time.time()
        self.stop_sent = False

    def speed_callback(self, msg):
        self.latest_speed = msg.data
        self.last_cmd_time = time.time()
        self.stop_sent = False

    def _connect_serial(self):
        if self.dry_run:
            return True
        if serial is None:
            self.get_logger().error('pyserial is not available, cannot open serial port.')
            return False
        if self.serial_conn is not None and self.serial_conn.is_open:
            return True
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Connected to ESP32 serial on {self.port} @ {self.baud}.')
            self.stop_sent = False
            return True
        except SerialException as exc:
            now = time.time()
            if now - self.last_status_log_time > 2.0:
                self.get_logger().warn(f'Failed to open serial port {self.port}: {exc}')
                self.last_status_log_time = now
            self.serial_conn = None
            return False

    def _close_serial(self):
        if self.serial_conn is None:
            return
        try:
            if self.serial_conn.is_open:
                self.serial_conn.close()
        except Exception:
            pass
        self.serial_conn = None

    def _build_drive_packet(self):
        if self.latest_speed is None or self.latest_steer is None:
            return None

        steer = self.latest_steer
        if self.invert_steering:
            steer = -steer

        servo_angle = self.servo_mid + steer * self.servo_deg_per_unit
        servo_angle = max(self.servo_min, min(self.servo_max, int(round(servo_angle))))
        speed_pwm = max(self.speed_min, min(self.speed_max, int(round(self.latest_speed))))
        return f'DRV,{speed_pwm},{servo_angle}\n'

    def _send_packet(self, packet):
        if self.dry_run:
            self.get_logger().info(packet.strip())
            return
        if not self._connect_serial():
            return
        try:
            self.serial_conn.write(packet.encode('utf-8'))
        except SerialException as exc:
            self.get_logger().warn(f'Serial write failed: {exc}')
            self._close_serial()

    def _send_stop(self):
        if self.stop_sent:
            return
        self._send_packet('STOP\n')
        self.stop_sent = True

    def timer_callback(self):
        if self.latest_speed is None or self.latest_steer is None:
            self._send_stop()
            return

        now = time.time()
        if self.last_cmd_time and (now - self.last_cmd_time) > self.command_timeout_sec:
            self._send_stop()
            return

        packet = self._build_drive_packet()
        if packet is None:
            self._send_stop()
            return

        self._send_packet(packet)
        self.stop_sent = False

    def destroy_node(self):
        try:
            self._send_packet('STOP\n')
        except Exception:
            pass
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
