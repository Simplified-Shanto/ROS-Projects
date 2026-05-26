import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from std_msgs.msg import Bool, Int32 
import time 


class LineCounter(Node):
    def __init__(self):

        super().__init__('line_counter_node')
        self.declare_parameter('lap_count', 3)

        self.bridge = CvBridge()
        self.lap_count = int(self.get_parameter('lap_count').value)
        self.current_count = 0 
        self.done_sent = False 
        self.last_detected_color = 'none'
        self.line_count = 0
        self.last_count_time = 0.0 
        self.cooldown_seconds = 1.5 
      
        self.image_subscriber = self.create_subscription(
            Image, #Message type 
            '/line_camera/image_raw', #Message topic 
            self.image_callback, # Callback function uponing receiving a message from the subscribed topic 
            10
        )

        self.line_count_publisher = self.create_publisher(
            Int32, #Message type 
            '/line_count', # Topic 
              10 # Queue size. 
        ) 
        
        self.debug_frame_publisher = self.create_publisher(
            Image,  #Message type 
            '/debug_frame', # Topic 
            10 # Queue size. 
        )

       

        self.get_logger().info('Lap counter node started.')

    def detect_line_color(self, roi):
        # Split the ROI into B, G, R channel arrays. We cast to int16 so
        # channel comparisons/subtractions stay safe and do not wrap around
        # like uint8 values can.
        b = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        r = roi[:, :, 2].astype(np.int16)

        # Pixels are treated as "white floor" only when all three channels
        # are bright. This produces a boolean mask with one True/False value
        # per pixel.
        white_mask = (b > 190) & (g > 190) & (r > 190)

        # Invert that mask so we keep only the non-white, color-candidate
        # pixels. These are the pixels worth checking for blue/orange.
        color_mask = ~white_mask

        # If almost the entire ROI is still white, there is probably no
        # meaningful colored stripe in view yet.
        if np.count_nonzero(color_mask) < 0.05 * color_mask.size:
            return 'none'

        # Blue/orange masks are also boolean images. A pixel becomes True only
        # if it passes the rough color rules for that class.
        blue_mask = color_mask & (b > 120) & (b > g + 25) & (b > r + 25)
        orange_mask = color_mask & (r > 150) & (g > 80) & (r > b + 35) & (b < 140)

        # Count how many pixels in the ROI look blue or orange.
        blue_pixels = np.count_nonzero(blue_mask)
        orange_pixels = np.count_nonzero(orange_mask)

        # Require the detected color to occupy at least 3% of the whole ROI.
        # This avoids classifying tiny noisy patches as a real line region.
        min_pixels = int(0.03 * roi.shape[0] * roi.shape[1])

        if blue_pixels < min_pixels and orange_pixels < min_pixels:
            return 'none'
        if blue_pixels > orange_pixels:
            return 'blue'
        return 'orange'

    def image_callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        height, width, _ = frame.shape
        roi = frame[int(height * 0.75):height, :]
        detected_color = self.detect_line_color(roi)

        if detected_color != self.last_detected_color:
            self.get_logger().info(f'Detected line color: {detected_color}')
            self.last_detected_color = detected_color
            now = time.time()
            if now - self.last_count_time > self.cooldown_seconds: 
                self.line_count+=1
                self.last_count_time = now
                msg = Int32()
                msg.data = self.line_count
                self.line_count_publisher.publish(msg)
                    

            

        debug_roi = roi.copy()
        cv2.putText(
            debug_roi,
            f'Line: {detected_color}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_roi, 
            f'Count: {self.line_count}', 
            (150, 30), 
            cv2.FONT_HERSHEY_SIMPLEX,   
            0.8, 
            (0, 255, 0), 
            2, 
            cv2.LINE_AA, 
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_roi, encoding='bgr8')
        debug_msg.header = image_msg.header 
        self.debug_frame_publisher.publish(debug_msg)

        # cv2.imshow("Cropped Frame", debug_roi)
        # cv2.waitKey(1)

  

def main(args=None):
    rclpy.init(args=args)
    node = LineCounter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
