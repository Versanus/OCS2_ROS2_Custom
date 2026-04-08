#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTfBridge(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('parent_frame_id', '')
        self.declare_parameter('child_frame_id', '')

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self._warned_missing_parent = False
        self._warned_missing_child = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(f'Odom TF bridge started: {self.odom_topic} -> TF')

    def odom_callback(self, msg: Odometry):
        parent_frame = self.parent_frame_id or msg.header.frame_id
        child_frame = self.child_frame_id or msg.child_frame_id

        if not parent_frame:
            if not self._warned_missing_parent:
                self.get_logger().warn(
                    'Received odometry without header.frame_id and no parent_frame_id override. Skipping TF.'
                )
                self._warned_missing_parent = True
            return

        if not child_frame:
            if not self._warned_missing_child:
                self.get_logger().warn(
                    'Received odometry without child_frame_id and no child_frame_id override. Skipping TF.'
                )
                self._warned_missing_child = True
            return

        transform = TransformStamped()
        transform.header.frame_id = parent_frame
        # RViz needs the odom->base TF timestamp to line up with the live joint-state TF.
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec < 1000000:
            transform.header.stamp = self.get_clock().now().to_msg()
        else:
            transform.header.stamp = msg.header.stamp
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
