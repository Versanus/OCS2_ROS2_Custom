#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node


class OdomToPathBridge(Node):
    def __init__(self):
        super().__init__('odom_to_path_bridge')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('path_topic', 'odom_path')
        self.declare_parameter('path_frame_id', '')
        self.declare_parameter('max_length', 100)
        self.declare_parameter('min_position_delta', 0.02)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.path_frame_id = self.get_parameter('path_frame_id').get_parameter_value().string_value
        self.max_length = max(1, self.get_parameter('max_length').get_parameter_value().integer_value)
        self.min_position_delta = self.get_parameter('min_position_delta').get_parameter_value().double_value

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.path_msg = Path()
        self._last_xyz = None

        self.get_logger().info(f'Odom Path bridge started: {self.odom_topic} -> {self.path_topic}')

    def odom_callback(self, msg: Odometry):
        frame_id = self.path_frame_id or msg.header.frame_id or 'odom'
        pose = msg.pose.pose.position
        xyz = (pose.x, pose.y, pose.z)

        if self._last_xyz is not None:
            dx = xyz[0] - self._last_xyz[0]
            dy = xyz[1] - self._last_xyz[1]
            dz = xyz[2] - self._last_xyz[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < self.min_position_delta:
                self.path_msg.header.stamp = msg.header.stamp
                self.path_msg.header.frame_id = frame_id
                self.path_pub.publish(self.path_msg)
                return

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = msg.pose.pose

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.header.frame_id = frame_id
        self.path_msg.poses.append(pose_stamped)

        if len(self.path_msg.poses) > self.max_length:
            self.path_msg.poses = self.path_msg.poses[-self.max_length:]

        self._last_xyz = xyz
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPathBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
