#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from legged_msgs.msg import SimulatorStateData


class StateToOdom(Node):
    def __init__(self):
        super().__init__('state_to_odom')

        self.declare_parameter('input_topic', 'simulator_state_data')
        self.declare_parameter('output_topic', 'odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self._warned_missing_pose = False
        self._warned_missing_quat = False

        self.subscription = self.create_subscription(
            SimulatorStateData,
            self.input_topic,
            self.state_callback,
            10,
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)

        self.get_logger().info(
            f"State odom bridge started: {self.input_topic} -> {self.output_topic}"
        )

    def state_callback(self, msg: SimulatorStateData):
        if len(msg.base_pose_values) < 3:
            if not self._warned_missing_pose:
                self.get_logger().warn(
                    'Received simulator_state_data without a full base_pose_values vector. Skipping.'
                )
                self._warned_missing_pose = True
            return

        if len(msg.base_quat_values) < 4:
            if not self._warned_missing_quat:
                self.get_logger().warn(
                    'Received simulator_state_data without a full base_quat_values vector. Skipping.'
                )
                self._warned_missing_quat = True
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = msg.base_pose_values[0]
        odom.pose.pose.position.y = msg.base_pose_values[1]
        odom.pose.pose.position.z = msg.base_pose_values[2]

        odom.pose.pose.orientation.w = msg.base_quat_values[0]
        odom.pose.pose.orientation.x = msg.base_quat_values[1]
        odom.pose.pose.orientation.y = msg.base_quat_values[2]
        odom.pose.pose.orientation.z = msg.base_quat_values[3]

        if len(msg.base_linvel_values) >= 3:
            odom.twist.twist.linear.x = msg.base_linvel_values[0]
            odom.twist.twist.linear.y = msg.base_linvel_values[1]
            odom.twist.twist.linear.z = msg.base_linvel_values[2]

        if len(msg.base_angvel_values) >= 3:
            odom.twist.twist.angular.x = msg.base_angvel_values[0]
            odom.twist.twist.angular.y = msg.base_angvel_values[1]
            odom.twist.twist.angular.z = msg.base_angvel_values[2]

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = StateToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
