#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion

class PurePursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit_controller")
        
        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 1.5)
        self.max_lookahead = rospy.get_param("~max_lookahead_distance", 3.0)
        self.min_lookahead = rospy.get_param("~min_lookahead_distance", 0.5)
        self.velocity = rospy.get_param("~velocity", 5)
        self.wheel_base = rospy.get_param("~wheel_base", 0.3)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 1.0)
        
        self.path = []  # Global plan waypoints
        self.current_pose = None  # Robot's current pose
        
        # Subscribers
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # Publisher
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.spin()
        
    def path_callback(self, msg):
        """Extracts waypoints from the global plan."""
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    
    def odom_callback(self, msg):
        """Updates the robot's current position and orientation."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_pose = (pos.x, pos.y, yaw)
        self.pure_pursuit_control()
        
    def find_lookahead_point(self):
        """Finds the nearest waypoint at the lookahead distance."""
        if not self.path or self.current_pose is None:
            return None
        
        x, y, _ = self.current_pose
        for point in self.path:
            dist = np.hypot(point[0] - x, point[1] - y)
            if self.min_lookahead <= dist <= self.max_lookahead:
                return point
        return None
    
    def pure_pursuit_control(self):
        """Computes and publishes velocity commands based on Pure Pursuit."""
        if self.current_pose is None or not self.path:
            return
        
        lookahead = self.find_lookahead_point()
        if lookahead is None:
            rospy.loginfo("No valid lookahead point found!")
            return
        
        x, y, yaw = self.current_pose
        goal_x, goal_y = lookahead
        
        # Transform goal point to robot frame
        dx = goal_x - x
        dy = goal_y - y
        target_angle = np.arctan2(dy, dx) - yaw
        
        # Compute curvature and steering angle
        L = np.hypot(dx, dy)
        curvature = 2 * dy / (L ** 2)  # Pure Pursuit curvature formula
        steering_angle = np.arctan(curvature * self.wheel_base)
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = self.velocity
        cmd.angular.z = steering_angle  # Use angular velocity to approximate steering
        self.cmd_pub.publish(cmd)
        
        rospy.loginfo(f"Target: ({goal_x:.2f}, {goal_y:.2f}) | Steering: {steering_angle:.2f} rad")

if __name__ == "__main__":
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
