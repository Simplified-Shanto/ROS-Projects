#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def cmd_vel_to_ackermann(msg):
    # Convert Twist message to AckermannDriveStamped message
    ackermann_msg = AckermannDriveStamped()
    ackermann_msg.drive.speed = msg.linear.x  # Linear velocity maps to speed
    ackermann_msg.drive.steering_angle = msg.angular.z  # Angular velocity maps to steering angle

    # Publish the converted Ackermann message
    pub.publish(ackermann_msg)

def listener():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_to_ackermann_converter', anonymous=True)

    # Publisher to the /low_level/ackermann_cmd_mux/input/teleop topic
    global pub
    pub = rospy.Publisher('/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)

    # Subscriber to the /cmd_vel topic
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_to_ackermann)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()
