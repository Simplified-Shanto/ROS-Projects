import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32 


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # `lazyWorld` has roughly a 1.0 m lane width, so keep the robot near
        # the 0.5 m centerline and use much lower corner entry speeds.
        self.declare_parameter('target_distance', 0.50)
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('turn_linear_speed', 0.12)
        self.declare_parameter('kp_distance', 1.20)
        self.declare_parameter('kp_angle', 0.80)
        self.declare_parameter('max_angular_speed', 1.40)
        self.declare_parameter('front_turn_distance', 0.70)
        self.declare_parameter('front_stop_distance', 0.32)
        self.declare_parameter('wall_detect_distance', 1.50)

        self.target_distance = float(self.get_parameter('target_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_linear_speed = float(self.get_parameter('turn_linear_speed').value)
        self.kp_distance = float(self.get_parameter('kp_distance').value)
        self.kp_angle = float(self.get_parameter('kp_angle').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.front_turn_distance = float(self.get_parameter('front_turn_distance').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.wall_detect_distance = float(self.get_parameter('wall_detect_distance').value)

        self.lap_completed = 0
        self.stop_commanded_time = time.time() # The time a stop command was sent from the lap completion logic 

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.line_count_subscriber = self.create_subscription(
            Int32, 
            '/line_count', 
            self.line_count_callback, 
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('Wall follower node started.')


    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def has_near_wall(self, distance):
        return not math.isinf(distance) and distance <= self.wall_detect_distance


    def sanitize_distance(self, distance):
        if math.isinf(distance):
            return self.wall_detect_distance
        return min(distance, self.wall_detect_distance)

    def get_range_at_angle(self, scan_msg, angle_degrees):
        """
        Returns LiDAR range at a desired angle in degrees.
        ROS convention:
        0 degrees = front
        +90 degrees = left
        -90 degrees = right
        """
        angle_radians = math.radians(angle_degrees)

        if angle_radians < scan_msg.angle_min or angle_radians > scan_msg.angle_max:
            return float('inf')

        index = int((angle_radians - scan_msg.angle_min) / scan_msg.angle_increment)

        if index < 0 or index >= len(scan_msg.ranges):
            return float('inf')

        value = scan_msg.ranges[index]

        if math.isnan(value) or math.isinf(value):
            return float('inf')

        return value

    def get_front_distance(self, scan_msg):
        """
        Uses a small front sector instead of only one ray.
        This makes obstacle detection more stable.
        """
        front_angles = [-15, -10, -5, 0, 5, 10, 15]
        distances = []

        for angle in front_angles:
            distance = self.get_range_at_angle(scan_msg, angle)
            if not math.isinf(distance):
                distances.append(distance)

        if len(distances) == 0:
            return float('inf')

        return min(distances)

    def compute_corner_turn(self, front_left_distance, front_right_distance):
        opening_error = (
            self.sanitize_distance(front_left_distance)
            - self.sanitize_distance(front_right_distance)
        )

        angular_z = self.clamp(
            self.kp_angle * opening_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        if abs(angular_z) < 0.8:
            return -0.8

        return angular_z

    def scan_callback(self, scan_msg):
        now = time.time()
        cmd = Twist()

        if self.lap_completed==0 or (self.lap_completed==1 and  now - self.stop_commanded_time < 4): 

            front_distance = self.get_front_distance(scan_msg)
            left_distance = self.get_range_at_angle(scan_msg, 90)
            right_distance = self.get_range_at_angle(scan_msg, -90)
            front_left_distance = self.get_range_at_angle(scan_msg, 45)
            front_right_distance = self.get_range_at_angle(scan_msg, -45)

            # Case 1: very close to the front wall, rotate decisively toward
            # the more open corner so the robot can stay near the tunnel center.
            if front_distance < self.front_stop_distance:
                cmd.linear.x = 0.0
                cmd.angular.z = self.compute_corner_turn(
                    front_left_distance,
                    front_right_distance
                )
                self.cmd_publisher.publish(cmd)
                return

            # Case 2: front wall is approaching, slow down and start the turn
            # before reaching the corner so the chassis cuts through the middle.
            if front_distance < self.front_turn_distance:
                cmd.linear.x = float(self.turn_linear_speed)
                cmd.angular.z = float(self.compute_corner_turn(
                    front_left_distance,
                    front_right_distance
                ))
                self.cmd_publisher.publish(cmd)
                return

            # Case 3: tunnel following. Use both side walls when available to stay
            # centered, and gracefully fall back to one wall when the opposite side
            # opens up near a corner.
            left_visible = self.has_near_wall(left_distance)
            right_visible = self.has_near_wall(right_distance)

            if left_visible and right_visible:
                distance_error = left_distance - right_distance
            elif right_visible:
                distance_error = self.target_distance - right_distance
            elif left_visible:
                distance_error = left_distance - self.target_distance
            else:
                distance_error = 0.0

            angle_error = (
                self.sanitize_distance(front_left_distance)
                - self.sanitize_distance(front_right_distance)
            )

            angular_z = (
                self.kp_distance * distance_error
                + self.kp_angle * angle_error
            )

            angular_z = self.clamp(
                angular_z,
                -self.max_angular_speed,
                self.max_angular_speed
            )

            steering_ratio = abs(angular_z) / self.max_angular_speed

            cmd.linear.x = float(self.clamp(
                self.linear_speed * (1.0 - 0.50 * steering_ratio),  #0.7
                self.turn_linear_speed,
                self.linear_speed
            ))
            cmd.angular.z = float(angular_z)
            self.cmd_publisher.publish(cmd)
        elif self.lap_completed == 1: 
            self.cmd_publisher.publish(cmd)
            self.get_logger().info(f'Robot is stopped!')
            self.lap_completed = 2 # Indicates a state where lap completion is done, stop command sending is done 


    def line_count_callback(self, msg):

        if self.lap_completed:
            return
        if msg.data >= 12:
            self.lap_completed = 1
            self.stop_commanded_time = time.time()
            self.get_logger().info(f'Commanding robot to stop at line count {msg.data}.')
  



def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
