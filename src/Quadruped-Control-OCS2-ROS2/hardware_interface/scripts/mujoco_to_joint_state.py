#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from legged_msgs.msg import SimulatorStateData


class MujocoToJointState(Node):
    def __init__(self):
        super().__init__('mujoco_to_joint_state')

        self.declare_parameter('input_topic', 'simulator_state_data')
        self.declare_parameter('output_topic', 'joint_states')
        self.declare_parameter(
            'joint_names',
            [
                'LF_HAA', 'LF_HFE', 'LF_KFE',
                'LH_HAA', 'LH_HFE', 'LH_KFE',
                'RF_HAA', 'RF_HFE', 'RF_KFE',
                'RH_HAA', 'RH_HFE', 'RH_KFE',
            ],
        )
        self.declare_parameter('publish_velocity', True)
        self.declare_parameter('publish_effort', False)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.joint_names = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)
        self.publish_velocity = self.get_parameter('publish_velocity').get_parameter_value().bool_value
        self.publish_effort = self.get_parameter('publish_effort').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            SimulatorStateData,
            self.input_topic,
            self.state_callback,
            10,
        )
        self.publisher = self.create_publisher(JointState, self.output_topic, 10)

        self.get_logger().info(
            f"MuJoCo JointState bridge started: {self.input_topic} -> {self.output_topic}"
        )

    def state_callback(self, msg: SimulatorStateData):
        if len(msg.joint_position_values) < len(self.joint_names):
            self.get_logger().warn(
                f"Expected at least {len(self.joint_names)} joint positions, got {len(msg.joint_position_values)}. Skipping."
            )
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_names)
        joint_state.position = list(msg.joint_position_values[:len(self.joint_names)])

        if self.publish_velocity and len(msg.joint_velocity_values) >= len(self.joint_names):
            joint_state.velocity = list(msg.joint_velocity_values[:len(self.joint_names)])

        if self.publish_effort and len(msg.joint_torque_values) >= len(self.joint_names):
            joint_state.effort = list(msg.joint_torque_values[:len(self.joint_names)])

        self.publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoToJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
