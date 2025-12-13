#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('publish_tf', True)

        self._odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 50)

        self.get_logger().info(f'Subscribing odom: {self._odom_topic}, publish_tf={self._publish_tf}')

    def _on_odom(self, msg: Odometry) -> None:
        if not self._publish_tf:
            return

        if not msg.child_frame_id or not msg.header.frame_id:
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
