#!/usr/bin/env python3
"""
ROS Noetic (Python) - subscribe to a camera topic, draw text on the frame,
and publish a new image topic.

This is intentionally simple for beginners:
- One subscriber: /usb_cam/image_raw
- One publisher:  /vision/output_image
- Uses cv_bridge to convert between ROS Image messages and OpenCV images
"""

import rospy
import cv2
import numpy as np 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TextOverlayRelay:
    def __init__(self):
        # CvBridge converts:
        #   ROS sensor_msgs/Image <-> OpenCV cv::Mat (numpy array in Python)
        self.bridge = CvBridge()

        # Read input topic from a ROS parameter so you can change it at runtime/launch
        # Example: _image_topic:=/usb_cam/image_raw
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")

        # Publish to an ABSOLUTE topic name to avoid confusion about namespaces
        self.output_topic = rospy.get_param("~output_topic", "/vision/output_image")

        # Create publisher (queue_size=1 reduces latency and avoids building backlog)
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)

        # Subscribe to the camera image topic
        # queue_size=1 => always process the latest frame (good for real-time)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.callback, queue_size=1)

        rospy.loginfo(f"[text_overlay_relay] Subscribed to: {self.image_topic}")
        rospy.loginfo(f"[text_overlay_relay] Publishing to:  {self.output_topic}")

                # HSV range (default: green-ish). Tune in runtime.
        self.h_low  = rospy.get_param("~h_low", 35)
        self.s_low  = rospy.get_param("~s_low", 80)
        self.v_low  = rospy.get_param("~v_low", 80)
        
        self.h_high = rospy.get_param("~h_high", 85)
        self.s_high = rospy.get_param("~s_high", 255)
        self.v_high = rospy.get_param("~v_high", 255)

    def callback(self, msg: Image):
        """
        Called every time a new image arrives on self.image_topic.
        """
        try:
            # Convert ROS Image -> OpenCV BGR image
            # Most USB cams in ROS publish bgr8; this forces consistent format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn(f"cv_bridge conversion failed: {e}")
            return
        
         # ----- Color mask (HSV) -----
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_low, self.s_low, self.v_low], dtype=np.uint8)
        upper = np.array([self.h_high, self.s_high, self.v_high], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)


        # Put text on the image (OpenCV uses BGR color)
        cv2.putText(
            masked_frame,
            "output",                 # text
            (30, 50),                 # (x, y) position
            cv2.FONT_HERSHEY_SIMPLEX, # font
            2.0,                      # font scale
            (0, 255, 0),              # color (green)
            2,                        # thickness
            cv2.LINE_AA               # line type (smooth)
        )

        try:
            # Convert OpenCV image -> ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(masked_frame, encoding="bgr8")

            # Preserve original timing + frame_id (good ROS practice)
            out_msg.header = msg.header

            # Publish processed image
            self.pub.publish(out_msg)
        except Exception as e:
            rospy.logwarn(f"publish failed: {e}")


if __name__ == "__main__":
    # Initialize ROS node.
    # anonymous=True prevents name collisions if you accidentally run twice.
    rospy.init_node("image_text_overlay", anonymous=True)

    node = TextOverlayRelay()

    # Keep the node alive so it continues to receive callbacks.
    rospy.spin()
