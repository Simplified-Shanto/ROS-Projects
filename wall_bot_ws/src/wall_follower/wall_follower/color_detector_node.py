import os

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.bridge = CvBridge()
        self.last_detected_color = 'none'
        self.enable_debug_view = False #bool(os.environ.get('DISPLAY'))
        self.trackbars_initialized = False

        self.default_thresholds = {
            'red1': {'h_min': 0, 'h_max': 20, 's_min': 30, 's_max': 255, 'v_min': 0, 'v_max': 180},
            'red2': {'h_min': 165, 'h_max': 179, 's_min': 30, 's_max': 255, 'v_min': 0, 'v_max': 180},
            'green': {'h_min': 32, 'h_max': 95, 's_min': 112, 's_max': 255, 'v_min': 0, 'v_max': 255},
        }

        self.image_subscriber = self.create_subscription(
            Image,
            '/line_camera/image_raw',
            self.image_callback,
            1
        )
        self.tower_color_publisher = self.create_publisher(
            String,
            '/closest_tower_color',
            1
        )
        if self.enable_debug_view:
            self.initialize_trackbars()

        self.get_logger().info('Color detector node started.')

    @staticmethod
    def _noop(_value):
        pass

    @staticmethod
    def _ordered_pair(low, high):
        return (low, high) if low <= high else (high, low)

    def initialize_trackbars(self):
        if self.trackbars_initialized:
            return

        cv2.namedWindow('HSV Thresholds', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('HSV Thresholds', 920, 720)

        trackbar_specs = [
            ('R1 H min', 179, self.default_thresholds['red1']['h_min']),
            ('R1 S min', 255, self.default_thresholds['red1']['s_min']),
            ('R1 V min', 255, self.default_thresholds['red1']['v_min']),

            ('R1 H max', 179, self.default_thresholds['red1']['h_max']),
            ('R1 S max', 255, self.default_thresholds['red1']['s_max']),
            ('R1 V max', 255, self.default_thresholds['red1']['v_max']),

            ('R2 H min', 179, self.default_thresholds['red2']['h_min']),
            ('R2 H max', 179, self.default_thresholds['red2']['h_max']),
          
            ('G H min', 179, self.default_thresholds['green']['h_min']),
            ('G S min', 255, self.default_thresholds['green']['s_min']),
            ('G V min', 255, self.default_thresholds['green']['v_min']),

            ('G H max', 179, self.default_thresholds['green']['h_max']),
            ('G S max', 255, self.default_thresholds['green']['s_max']),
            ('G V max', 255, self.default_thresholds['green']['v_max']),
        ]

        for name, max_value, initial_value in trackbar_specs:
            cv2.createTrackbar(name, 'HSV Thresholds', initial_value, max_value, self._noop)

        self.trackbars_initialized = True
        self.draw_threshold_panel(self.default_thresholds)

    def draw_threshold_panel(self, thresholds):
        panel = np.full((240, 880, 3), 245, dtype=np.uint8)

        cv2.putText(panel, 'HSV Threshold Tuning', (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (30, 30, 30), 2)
        cv2.putText(panel, 'Red range 1', (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2)
        cv2.putText(panel, 'Red range 2', (315, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2)
        cv2.putText(panel, 'Green range', (610, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 0), 2)

        groups = [
            ('red1', 20, (0, 0, 220)),
            ('red2', 315, (0, 0, 220)),
            ('green', 610, (0, 140, 0)),
        ]
        field_labels = [
            ('h_min', 'H min'),
            ('h_max', 'H max'),
            ('s_min', 'S min'),
            ('s_max', 'S max'),
            ('v_min', 'V min'),
            ('v_max', 'V max'),
        ]

        for group_name, x_origin, color in groups:
            y = 105
            for field_name, display_name in field_labels:
                value = thresholds[group_name][field_name]
                cv2.putText(
                    panel,
                    f'{display_name}: {value:>3}',
                    (x_origin, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    color,
                    2,
                )
                y += 24

        cv2.putText(
            panel,
            'Adjust sliders below. Red uses two hue ranges because HSV wraps around at 179.',
            (20, 225),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (70, 70, 70),
            1,
        )

        cv2.imshow('HSV Thresholds', panel)

    def get_thresholds(self):
        if not self.enable_debug_view or not self.trackbars_initialized:
            return self.default_thresholds

        red1_h_min = cv2.getTrackbarPos('R1 H min', 'HSV Thresholds')
        red1_h_max = cv2.getTrackbarPos('R1 H max', 'HSV Thresholds')
        red1_s_min = cv2.getTrackbarPos('R1 S min', 'HSV Thresholds')
        red1_s_max = cv2.getTrackbarPos('R1 S max', 'HSV Thresholds')
        red1_v_min = cv2.getTrackbarPos('R1 V min', 'HSV Thresholds')
        red1_v_max = cv2.getTrackbarPos('R1 V max', 'HSV Thresholds')

        red2_h_min = cv2.getTrackbarPos('R2 H min', 'HSV Thresholds')
        red2_h_max = cv2.getTrackbarPos('R2 H max', 'HSV Thresholds')
        red2_s_min = cv2.getTrackbarPos('R1 S min', 'HSV Thresholds')
        red2_s_max = cv2.getTrackbarPos('R1 S max', 'HSV Thresholds')
        red2_v_min = cv2.getTrackbarPos('R1 V min', 'HSV Thresholds')
        red2_v_max = cv2.getTrackbarPos('R1 V max', 'HSV Thresholds')

        green_h_min = cv2.getTrackbarPos('G H min', 'HSV Thresholds')
        green_h_max = cv2.getTrackbarPos('G H max', 'HSV Thresholds')
        green_s_min = cv2.getTrackbarPos('G S min', 'HSV Thresholds')
        green_s_max = cv2.getTrackbarPos('G S max', 'HSV Thresholds')
        green_v_min = cv2.getTrackbarPos('G V min', 'HSV Thresholds')
        green_v_max = cv2.getTrackbarPos('G V max', 'HSV Thresholds')

        red1_h_min, red1_h_max = self._ordered_pair(red1_h_min, red1_h_max)
        red1_s_min, red1_s_max = self._ordered_pair(red1_s_min, red1_s_max)
        red1_v_min, red1_v_max = self._ordered_pair(red1_v_min, red1_v_max)

        red2_h_min, red2_h_max = self._ordered_pair(red2_h_min, red2_h_max)
        red2_s_min, red2_s_max = self._ordered_pair(red2_s_min, red2_s_max)
        red2_v_min, red2_v_max = self._ordered_pair(red2_v_min, red2_v_max)

        green_h_min, green_h_max = self._ordered_pair(green_h_min, green_h_max)
        green_s_min, green_s_max = self._ordered_pair(green_s_min, green_s_max)
        green_v_min, green_v_max = self._ordered_pair(green_v_min, green_v_max)

        return {
            'red1': {'h_min': red1_h_min, 'h_max': red1_h_max, 's_min': red1_s_min, 's_max': red1_s_max, 'v_min': red1_v_min, 'v_max': red1_v_max},
            'red2': {'h_min': red2_h_min, 'h_max': red2_h_max, 's_min': red2_s_min, 's_max': red2_s_max, 'v_min': red2_v_min, 'v_max': red2_v_max},
            'green': {'h_min': green_h_min, 'h_max': green_h_max, 's_min': green_s_min, 's_max': green_s_max, 'v_min': green_v_min, 'v_max': green_v_max},
        }

    def image_callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresholds = self.get_thresholds()

        # red_lower1 = np.array([thresholds['red1']['h_min'], thresholds['red1']['s_min'], thresholds['red1']['v_min']])
        # red_upper1 = np.array([thresholds['red1']['h_max'], thresholds['red1']['s_max'], thresholds['red1']['v_max']])

        # red_lower2 = np.array([thresholds['red2']['h_min'], thresholds['red2']['s_min'], thresholds['red2']['v_min']])
        # red_upper2 = np.array([thresholds['red2']['h_max'], thresholds['red2']['s_max'], thresholds['red2']['v_max']])

        # green_lower = np.array([thresholds['green']['h_min'], thresholds['green']['s_min'], thresholds['green']['v_min']])
        # green_upper = np.array([thresholds['green']['h_max'], thresholds['green']['s_max'], thresholds['green']['v_max']])


        red_lower1 = np.array([0, 30, 0])
        red_upper1 = np.array([20, 255, 255])

        red_lower2 = np.array([165, 30, 0])
        red_upper2 = np.array([179, 255, 255])

        green_lower = np.array([32, 112, 0])
        green_upper = np.array([95, 255, 255])



        red_mask1 = cv2.inRange(img_hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(img_hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
      
        green_mask = cv2.inRange(img_hsv, green_lower, green_upper)

        red_pixels = int(np.count_nonzero(red_mask))
        green_pixels = int(np.count_nonzero(green_mask))
        min_pixels = 200#int(0.005 * red_mask.size)
        
        # self.get_logger().info(f"green_pixel = {green_pixels}")
        # self.get_logger().info(f"red_pixel={red_pixels}")
        detected_color = 'none'
        if red_pixels > green_pixels and red_pixels > min_pixels:
            detected_color = 'red'
        elif green_pixels > min_pixels:
            detected_color = 'green'

        if detected_color != self.last_detected_color:
            self.last_detected_color = detected_color
            self.get_logger().info(f'Detected tower color: {detected_color}')

        # Publish every processed frame so downstream control keeps a fresh
        # timestamp even when the same color stays visible for a while.
        msg = String()
        msg.data = detected_color
        self.tower_color_publisher.publish(msg)

        if self.enable_debug_view:
            self.draw_threshold_panel(thresholds)
            red_masked_frame = cv2.bitwise_and(frame, frame, mask=red_mask)
            green_masked_frame = cv2.bitwise_and(frame, frame, mask=green_mask)
            mask_preview = cv2.cvtColor(np.maximum(red_mask, green_mask), cv2.COLOR_GRAY2BGR)
            #cv2.imshow('Original Frame', frame)
            cv2.imshow('Red Masked Frame', red_masked_frame)
            cv2.imshow('Green Masked Frame', green_masked_frame)
           # cv2.imshow('Color Mask', mask_preview)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
