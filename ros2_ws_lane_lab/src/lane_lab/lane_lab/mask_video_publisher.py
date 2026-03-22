import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image


def empty(_):
    pass


class MaskVideoPublisher(Node):
    def __init__(self):
        super().__init__('mask_video_publisher')

        self.declare_parameter('video_path', '')
        self.declare_parameter('mask_topic', '/lane/mask')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('loop', True)

        self.video_path = self.get_parameter('video_path').value
        self.mask_topic = self.get_parameter('mask_topic').value
        self.fps = float(self.get_parameter('fps').value)
        self.loop = bool(self.get_parameter('loop').value)

        if not self.video_path:
            raise RuntimeError('Parameter "video_path" must be provided.')

        if self.fps <= 0.0:
            self.get_logger().warn('FPS must be > 0. Falling back to 10.0')
            self.fps = 10.0

        self.publisher_ = self.create_publisher(Image, self.mask_topic, 10)
        self.cap = cv2.VideoCapture(self.video_path)
        self.frame_count = 0

        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open video: {self.video_path}')

        self.timer = self.create_timer(1.0 / self.fps, self.publish_next_frame)

        # cv2.namedWindow('Trackbars')
        # cv2.resizeWindow('Trackbars', 900, 320)
        # cv2.namedWindow('Result img')

        # cv2.createTrackbar('HUE MIN', 'Trackbars', 0, 179, empty)
        # cv2.createTrackbar('HUE MAX', 'Trackbars', 179, 179, empty)
        # cv2.createTrackbar('SAT MIN', 'Trackbars', 0, 255, empty)
        # cv2.createTrackbar('SAT MAX', 'Trackbars', 255, 255, empty)
        # cv2.createTrackbar('VAL MIN', 'Trackbars', 0, 255, empty)
        # cv2.createTrackbar('VAL MAX', 'Trackbars', 255, 255, empty)

    def publish_next_frame(self):
        # h_min = cv2.getTrackbarPos('HUE MIN', 'Trackbars')
        # h_max = cv2.getTrackbarPos('HUE MAX', 'Trackbars')
        # s_min = cv2.getTrackbarPos('SAT MIN', 'Trackbars')
        # s_max = cv2.getTrackbarPos('SAT MAX', 'Trackbars')
        # v_min = cv2.getTrackbarPos('VAL MIN', 'Trackbars')
        # v_max = cv2.getTrackbarPos('VAL MAX', 'Trackbars')

        # lower = np.array([h_min, s_min, v_min])
        # upper = np.array([h_max, s_max, v_max])
        lower = np.array([10, 83, 50])
        upper = np.array([42, 255, 212])

        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error('Failed to read video after looping.')
                    return
            else:
                self.get_logger().info('End of video reached. Stopping publisher.')
                self.timer.cancel()
                return

        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, lower, upper)
        result_img = cv2.bitwise_and(frame, frame, mask=mask)

        overlay = result_img.copy()
        cv2.rectangle(overlay, (10, 10), (250, 105), (0, 0, 0), -1)
        result_img = cv2.addWeighted(overlay, 0.45, result_img, 0.55, 0.0)

        # cv2.putText(
        #     result_img,
        #     f'H: {h_min}-{h_max}',
        #     (20, 35),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.7,
        #     (0, 255, 255),
        #     2,
        #     cv2.LINE_AA,
        # )
        # cv2.putText(
        #     result_img,
        #     f'S: {s_min}-{s_max}',
        #     (20, 65),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.7,
        #     (0, 255, 255),
        #     2,
        #     cv2.LINE_AA,
        # )
        # cv2.putText(
        #     result_img,
        #     f'V: {v_min}-{v_max}',
        #     (20, 95),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.7,
        #     (0, 255, 255),
        #     2,
        #     cv2.LINE_AA,
        # )

        # cv2.imshow('Result img', mask)
        # cv2.waitKey(1)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lane_mask'
        msg.height = mask.shape[0]
        msg.width = mask.shape[1]
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = mask.shape[1]
        msg.data = mask.tobytes()

        self.publisher_.publish(msg)
        self.frame_count += 1

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MaskVideoPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
