#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfOdometryRelay(Node):
    def __init__(self):
        super().__init__('tf_odometry_relay')
        self.subscription = self.create_subscription(
            TFMessage,
            '/ackermann_steering_controller/tf_odometry',
            self.relay_callback,
            10,
        )
        self.publisher = self.create_publisher(TFMessage, '/tf', 10)

    def relay_callback(self, msg: TFMessage):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfOdometryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
