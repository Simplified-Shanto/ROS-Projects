#!/usr/bin/env python3
"""
Ultralytics YOLO -> ROS overlay publisher (simple)

Subscribes:
  ~image_topic (default: /usb_cam/image_raw)    [sensor_msgs/Image]

Publishes:
  /yolo/overlay                                 [sensor_msgs/Image] (BGR annotated image)

Assumptions:
- cv_bridge works
- ultralytics is installed and can run on your Jetson (GPU via device="0")
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class UltralyticsYoloOverlayNode:
    def __init__(self):
        self.bridge = CvBridge()

        # ROS params
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.model_path  = rospy.get_param("~model", "weights/best.pt")

        # Inference params
        self.imgsz = int(rospy.get_param("~imgsz", 416))
        self.conf  = float(rospy.get_param("~conf", 0.35))
        self.iou   = float(rospy.get_param("~iou", 0.45))

        # device: "0" for CUDA GPU 0 on Jetson, or "cpu"
        self.device = str(rospy.get_param("~device", "0"))

        # Optional: skip frames to reduce load
        self.frame_skip = int(rospy.get_param("~frame_skip", 0))
        self._count = 0

        # Publisher
        self.pub_overlay = rospy.Publisher("/yolo/overlay", Image, queue_size=1)

        rospy.loginfo("[ultralytics_yolo_overlay] Loading model: %s", self.model_path)
        self.model = YOLO(self.model_path)

        # Subscriber
        self.sub = rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)

        rospy.loginfo("[ultralytics_yolo_overlay] Subscribed to: %s", self.image_topic)
        rospy.loginfo("[ultralytics_yolo_overlay] Publishing to: /yolo/overlay")
        rospy.loginfo("[ultralytics_yolo_overlay] device=%s imgsz=%d conf=%.2f iou=%.2f",
                      self.device, self.imgsz, self.conf, self.iou)

    def cb(self, msg: Image):
        # Optional frame skipping
        if self.frame_skip > 0:
            self._count += 1
            if (self._count % (self.frame_skip + 1)) != 0:
                return

        # ROS -> OpenCV (BGR)
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("[ultralytics_yolo_overlay] cv_bridge imgmsg_to_cv2 failed: %s", e)
            return

        # Inference (single frame)
        try:
            r = self.model.predict(
                source=frame_bgr,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                verbose=False
            )[0]
        except Exception as e:
            rospy.logwarn("[ultralytics_yolo_overlay] model.predict failed: %s", e)
            return

        # Draw boxes on the frame (returns BGR numpy array)
        try:
            annotated_bgr = r.plot()
        except Exception as e:
            rospy.logwarn("[ultralytics_yolo_overlay] r.plot() failed: %s", e)
            return

        # OpenCV -> ROS
        try:
            out = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
            out.header = msg.header
            self.pub_overlay.publish(out)
        except Exception as e:
            rospy.logwarn("[ultralytics_yolo_overlay] publish failed: %s", e)


if __name__ == "__main__":
    rospy.init_node("ultralytics_yolo_overlay", anonymous=True)
    UltralyticsYoloOverlayNode()
    rospy.spin()
