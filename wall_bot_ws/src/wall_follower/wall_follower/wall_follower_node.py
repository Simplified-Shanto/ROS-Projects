import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Int32, String

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point 


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # `lazyWorld` has roughly a 1.0 m lane width, so keep the robot near
        # the 0.5 m centerline and use much lower corner entry speeds.
        self.declare_parameter('target_distance', 0.50)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('turn_linear_speed', 0.12)
        self.declare_parameter('kp_distance', 1.20)
        self.declare_parameter('kp_angle', 0.50)
        self.declare_parameter('max_angular_speed', 1.40)
        self.declare_parameter('front_turn_distance', 1.5)
        self.declare_parameter('front_stop_distance', 0.32)
        self.declare_parameter('wall_detect_distance', 1.50) #“beyond this distance, I don’t care exactly how far it is; it just means no nearby wall”
        self.declare_parameter('gap_difference', 0.30)


        self.target_distance = float(self.get_parameter('target_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_linear_speed = float(self.get_parameter('turn_linear_speed').value)
        self.kp_distance = float(self.get_parameter('kp_distance').value)
        self.kp_angle = float(self.get_parameter('kp_angle').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.front_turn_distance = float(self.get_parameter('front_turn_distance').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.wall_detect_distance = float(self.get_parameter('wall_detect_distance').value)
        self.gap_difference = float(self.get_parameter('gap_difference').value)


        self.active_gap_difference = 0 
        self.lap_completed = 0
        self.stop_commanded_time = time.time() # The time a stop command was sent from the lap completion logic 
        self.manual_stop_requested = False
        self.keyboard_thread = None
        self.keyboard_fd = None
        self.terminal_settings = None
        self.tower_passed = 0 
        self.state = 'FOLLOW_CENTER' # other states - STOP, AVOID_LEFT, AVOID_RIGHT



        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1
        )

        self.tower_color_subscriber = self.create_subscription(
            String,
            '/closest_tower_color', 
            self.tower_color_callback, 
            1 
        )

        self.line_count_subscriber = self.create_subscription(
            Int32, 
            '/line_count', 
            self.line_count_callback, 
            1
        )

        self.cmd_publisher = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            1
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,  # Message type 
            '/tower_debug_markers', # published topic 
              1) #Queue size 

        self.setup_keyboard_listener()
        self.get_logger().info('Wall follower node started.')
        self.get_logger().info("Press Ctrl+S (or 's') to command a manual stop.")


##################################
#   Helper functions 
##################################

    def make_sphere_marker(self, frame_id, stamp, marker_id, x, y, z, r, g, b): 
        marker = Marker()
        marker.header.frame_id = frame_id 
        marker.header.stamp = stamp
        marker.ns = 'tower_debug'
        marker.id = marker_id 
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD 

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0 

        marker.scale.x = 0.025 
        marker.scale.y = 0.025
        marker.scale.z = 0.025

        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0 

        marker.lifetime.sec = 0 
        marker.lifetime.nanosec = 200000000 #0.2s 

        return marker 
    
    def publish_tower_debug_markers(self, scan_msg, left_angle, left_range,  mid_angle, mid_range, right_angle, right_range): 
        frame_id = scan_msg.header.frame_id 
        stamp = scan_msg.header.stamp 

        z = 0.05 
        #here angles are in radians 
        lx = left_range * math.cos(left_angle)
        ly = left_range * math.sin(left_angle)

        mx = mid_range * math.cos(mid_angle)
        my = mid_range * math.sin(mid_angle)

        rx = right_range * math.cos(right_angle)
        ry = right_range * math.sin(right_angle)

        markerArray = MarkerArray()
        markerArray.markers.append(self.make_sphere_marker(frame_id, stamp,0,  lx , ly, z, 1.0, 0.0, 0.0)) # left edge = red 
        markerArray.markers.append(self.make_sphere_marker(frame_id, stamp,1,  mx, my, z, 1.0, 1.0, 0.0))  # middle = yellow 
        markerArray.markers.append(self.make_sphere_marker(frame_id, stamp,2, rx, ry, z, 0.0, 1.0, 0.0))  # left edge = green 

        self.marker_publisher.publish(markerArray)
   
    def find_valley(self, scan_msg, start_angle, end_angle):
        delta_threshold = 0.15 
        state = 'WAIT_FALL'

        left_edge = None 
        valley_bottom = None 
        tower_list = [] # This will contain (angle_rad, distance) for the discovered towers 

        prev_range = self.get_range_at_angle(scan_msg, start_angle)
        prev_angle_rad = math.radians(start_angle)

        if math.isinf(prev_range): 
            return []
        
        for angle_deg in range(start_angle + 1, end_angle + 1): 
            current_range = self.get_range_at_angle(scan_msg, angle_deg)
            if math.isinf(current_range): 
                continue

            current_delta = current_range - prev_range
            current_angle_rad = math.radians(angle_deg)

            if state == 'WAIT_FALL': 
                # Strong negative jump => object begins 
                if current_delta < -delta_threshold: 
                    left_edge = (current_angle_rad, current_range)
                    valley_bottom = (current_angle_rad, current_range)
                    state = 'TRACK_VALLEY'
            
            elif state == 'TRACK_VALLEY': 
                # Keep the closest point as the valley bottom 
                if valley_bottom is None or current_range < valley_bottom[1]: 
                    valley_bottom = (current_angle_rad, current_range)

                # Strong positive jump -> object ends 
                if current_delta > delta_threshold: 
                    right_edge = (prev_angle_rad, prev_range)

                
                    if left_edge is not None and valley_bottom is not None: 
                         angle_span_rad = abs(right_edge[0] - left_edge[0])
                         estimated_width = valley_bottom[1] * angle_span_rad # s = r * theta (radian) 
                         if True or 0.025 <= estimated_width <= 0.09: 
                            self.publish_tower_debug_markers(
                                scan_msg, 
                                left_edge[0], left_edge[1], 
                                valley_bottom[0], valley_bottom[1], 
                                right_edge[0], right_edge[1], 
                            )


                            tower_list.append((valley_bottom[0], valley_bottom[1])) # Only storing the angle and range of the valley_bottom as a tower characteristics 

                    # Reset for next valley 
                    state = 'WAIT_FALL'
                    left_edge = None
                    valley_bottom = None 
            prev_range = current_range
            prev_angle_rad = current_angle_rad
        return tower_list
        
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

    def publish_drive_command(self, linear_x, angular_z):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_publisher.publish(cmd)

    def setup_keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warn('Keyboard stop handler disabled: stdin is not a TTY.')
            return

        self.keyboard_fd = sys.stdin.fileno()
        self.terminal_settings = termios.tcgetattr(self.keyboard_fd)
        raw_settings = termios.tcgetattr(self.keyboard_fd)
        raw_settings[0] &= ~(termios.IXON | termios.IXOFF)
        raw_settings[3] &= ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(self.keyboard_fd, termios.TCSANOW, raw_settings)

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_listener_loop,
            daemon=True
        )
        self.keyboard_thread.start()

    def restore_terminal(self):
        if self.keyboard_fd is None or self.terminal_settings is None:
            return
        termios.tcsetattr(self.keyboard_fd, termios.TCSANOW, self.terminal_settings)
        self.keyboard_fd = None
        self.terminal_settings = None

    def keyboard_listener_loop(self):
        while rclpy.ok() and not self.manual_stop_requested:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            pressed_key = os.read(self.keyboard_fd, 1)
            if pressed_key in (b'\x13', b's', b'S'):
                self.manual_stop_requested = True
                self.get_logger().warn('Manual stop requested from keyboard.')
                break



    def tower_color_callback(self, tower_color): 
        if tower_color.data == 'green': 
            self.active_gap_difference = self.gap_difference
        elif tower_color.data=='red': 
            self.active_gap_difference  = -self.gap_difference
        else: 
            self.active_gap_difference = 0 




    def scan_callback(self, scan_msg):
        now = time.time()
        tower_list = self.find_valley(scan_msg, -80, 80)
        # sorted_tower_list =  sorted(tower_list, key=lambda x: x[1])

        # if self.state!='STOP': 
        #     if not sorted_tower_list or sorted_tower_list[0][1] > 1:  # If there's no tower, or there's a tower beyond 1 meter from the vehicle 
        #         self.state = 'FOLLOW_CENTER'
        #     else: 
        #         gap_difference =  self.desired_gap_difference

        front_distance = self.get_front_distance(scan_msg)
        left_distance = self.get_range_at_angle(scan_msg, 90)
        right_distance = self.get_range_at_angle(scan_msg, -90)
        front_left_distance = self.get_range_at_angle(scan_msg, 45)
        front_right_distance = self.get_range_at_angle(scan_msg, -45)

        if self.state=='FOLLOW_CENTER':
            # Case 2: front wall is approaching, slow down and start the turn
            # before reaching the corner so the chassis cuts through the middle.
            # if front_distance < self.front_turn_distance:
            #     linear_x = float(self.linear_speed)
            #     angular_z = float(self.compute_corner_turn(
            #         front_left_distance,
            #         front_right_distance
            #     ))
            #     self.publish_drive_command(linear_x=linear_x, angular_z=angular_z)
            #     return

            # Case 3: tunnel following. Use both side walls when available to stay
            # centered, and gracefully fall back to one wall when the opposite side
            # opens up near a corner.


            left_visible = self.has_near_wall(left_distance)
            right_visible = self.has_near_wall(right_distance)
            

            if right_visible:
                angle_with_right_ray = math.atan((front_right_distance*math.cos(math.pi/4) - right_distance)/(front_right_distance*math.sin(math.pi/4)))
                distance_from_right_wall = right_distance*math.cos(angle_with_right_ray)
            
            else: 
                distance_from_right_wall = self.wall_detect_distance

            if left_visible:
                angle_with_left_ray = math.atan((front_left_distance*math.cos(math.pi/4) - left_distance)/(front_left_distance*math.sin(math.pi/4)))
                distance_from_left_wall = left_distance*math.cos(angle_with_left_ray)
            else: 
                distance_from_left_wall = self.wall_detect_distance
                
            distance_error = distance_from_left_wall -  distance_from_right_wall - self.active_gap_difference
            self.get_logger().info(f"active_gap= {self.active_gap_difference}")


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

            linear_x = float(self.linear_speed)
            angular_z = float(angular_z)

            self.publish_drive_command(linear_x=linear_x, angular_z=angular_z)

        if self.manual_stop_requested:
            self.publish_drive_command(linear_x=0.0, angular_z=0.0)
            self.state = 'STOPPED'
            return
        

    def line_count_callback(self, msg):
        if self.lap_completed:
            return
        if msg.data >= 12:
            self.lap_completed = 1
            self.state = 'WAITING_TO_STOP'
            self.stop_commanded_time = time.time()
            self.get_logger().info(f'Commanding robot to stop at line count {msg.data}.')

                    #     elif self.lap_completed == 1: 
            # self.publish_drive_command(linear_x=0, angular_z=0)
            # self.get_logger().info(f'Robot is stopped!')
            # self.lap_completed = 2 # Indicates a state where lap completion is done, stop command sending is done 
            # self.state = 'STOPPED'
  



def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C can invalidate the ROS context before cleanup runs, so only
        # publish a final stop command if the node is still allowed to talk.
        if rclpy.ok():
            try:
                node.publish_drive_command(0.0, 0.0)
                time.sleep(0.2)
            except Exception:
                pass
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
