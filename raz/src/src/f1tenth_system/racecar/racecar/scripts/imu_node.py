#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu

# Global variables to store received sensor data
rpy_data = Vector3()
angular_velocity_data = Vector3()
acceleration_data = Vector3()
quaternion_data = Quaternion()

# Callback for Roll, Pitch, Yaw data
def rpy_callback(data):
    global rpy_data
    rpy_data = data

# Callback for Angular Velocity data
def angular_velocity_callback(data):
    global angular_velocity_data
    angular_velocity_data = data

# Callback for Acceleration data
def acceleration_callback(data):
    global acceleration_data
    acceleration_data = data

# Callback for Quaternion data
def quaternion_callback(data):
    global quaternion_data
    quaternion_data = data

def imu_publisher():
    # Initialize the ROS node
    rospy.init_node('imu_data_aggregator', anonymous=True)

    # Subscribe to the topics where Arduino publishes data
    rospy.Subscriber('rpy', Vector3, rpy_callback)
    rospy.Subscriber('angular_velocity', Vector3, angular_velocity_callback)
    rospy.Subscriber('acceleration', Vector3, acceleration_callback)
    rospy.Subscriber('quaternion', Quaternion, quaternion_callback)

    # Publisher for the IMU data
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

    # Loop to publish the combined IMU message
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create a new IMU message
        imu_msg = Imu()

        # Fill in the IMU message with data from the Arduino
        imu_msg.orientation = quaternion_data  # Quaternion for orientation
        imu_msg.angular_velocity = angular_velocity_data  # Angular velocity (in rad/s)
        imu_msg.linear_acceleration = acceleration_data  # Linear acceleration (in m/s^2)

        # Publish the IMU message
        imu_pub.publish(imu_msg)

        # Sleep for the remainder of the cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
