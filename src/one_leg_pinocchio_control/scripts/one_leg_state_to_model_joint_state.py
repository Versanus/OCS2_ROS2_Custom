#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OneLegStateToModelJointState(Node):
    def __init__(self):
        super().__init__('one_leg_state_to_model_joint_state')

        self.declare_parameter('input_topic', 'htdw_joint_state')
        self.declare_parameter('output_topic', 'one_leg_model_joint_states')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('hardware_joint_names', ['FL_HipX', 'FL_HipY', 'FL_Knee'])
        self.declare_parameter('model_joint_names', ['fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint'])
        self.declare_parameter('model_position_offsets', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'all_model_joint_names',
            [
                'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint',
                'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint',
                'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint',
                'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint',
            ],
        )

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.hardware_joint_names = list(self.get_parameter('hardware_joint_names').value)
        self.model_joint_names = list(self.get_parameter('model_joint_names').value)
        self.model_position_offsets = self._fit_list(
            list(self.get_parameter('model_position_offsets').value), 0.0, len(self.hardware_joint_names)
        )
        self.all_model_joint_names = list(self.get_parameter('all_model_joint_names').value)

        if len(self.hardware_joint_names) != len(self.model_joint_names):
            raise ValueError('hardware_joint_names and model_joint_names must have the same length.')

        self.name_map = dict(zip(self.hardware_joint_names, self.model_joint_names))
        self.offset_map = dict(zip(self.hardware_joint_names, self.model_position_offsets))
        self.latest_position = {name: 0.0 for name in self.all_model_joint_names}
        self.latest_velocity = {name: 0.0 for name in self.all_model_joint_names}
        self.latest_effort = {name: 0.0 for name in self.all_model_joint_names}
        for hardware_name, model_name in self.name_map.items():
            self.latest_position[model_name] = self.offset_map.get(hardware_name, 0.0)

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.subscription = self.create_subscription(JointState, self.input_topic, self._state_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_model_state)

        self.get_logger().info(f'RViz state mapper: {self.input_topic} -> {self.output_topic}')

    def _fit_list(self, values, fallback, length):
        if len(values) >= length:
            return [float(v) for v in values[:length]]
        return [float(fallback)] * length

    def _state_callback(self, msg):
        for index, hardware_name in enumerate(msg.name):
            model_name = self.name_map.get(hardware_name)
            if model_name is None:
                continue

            if index < len(msg.position):
                position = float(msg.position[index])
                if math.isfinite(position):
                    self.latest_position[model_name] = position + self.offset_map.get(hardware_name, 0.0)
            if index < len(msg.velocity):
                velocity = float(msg.velocity[index])
                if math.isfinite(velocity):
                    self.latest_velocity[model_name] = velocity
            if index < len(msg.effort):
                effort = float(msg.effort[index])
                if math.isfinite(effort):
                    self.latest_effort[model_name] = effort

        self._publish_model_state()

    def _publish_model_state(self):
        mapped = JointState()
        mapped.header.stamp = self.get_clock().now().to_msg()
        mapped.name = list(self.all_model_joint_names)
        mapped.position = [self.latest_position[name] for name in self.all_model_joint_names]
        mapped.velocity = [self.latest_velocity[name] for name in self.all_model_joint_names]
        mapped.effort = [self.latest_effort[name] for name in self.all_model_joint_names]
        self.publisher.publish(mapped)


def main(args=None):
    rclpy.init(args=args)
    node = OneLegStateToModelJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
