import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data')

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        imu_topic = self.get_parameter('imu_topic').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, imu_topic, self.handle_imu, 10)

    def handle_imu(self, msg: Imu):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
