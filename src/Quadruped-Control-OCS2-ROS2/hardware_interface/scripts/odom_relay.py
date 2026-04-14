#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')

        self.declare_parameter('input_topic', 'odom')
        self.declare_parameter('output_topic', 'rviz_hardware_odom')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self.odom_callback,
            10,
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)

        self.get_logger().info(
            f'Odom relay started: {self.input_topic} -> {self.output_topic}'
        )

    def odom_callback(self, msg: Odometry):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
