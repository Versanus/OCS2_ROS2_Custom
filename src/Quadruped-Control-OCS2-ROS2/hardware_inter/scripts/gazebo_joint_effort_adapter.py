#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from legged_msgs.msg import JointControlData
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GazeboJointEffortAdapter(Node):
    def __init__(self):
        super().__init__('gazebo_joint_effort_adapter')

        default_joint_names = [
            'LF_HAA', 'LF_HFE', 'LF_KFE',
            'LH_HAA', 'LH_HFE', 'LH_KFE',
            'RF_HAA', 'RF_HFE', 'RF_KFE',
            'RH_HAA', 'RH_HFE', 'RH_KFE',
        ]

        self.declare_parameter('command_topic', 'joint_control_data')
        self.declare_parameter('input_joint_state_topic', 'joint_states_raw')
        self.declare_parameter('output_joint_state_topic', 'joint_states')
        self.declare_parameter('force_topic_prefix', '/model/quad_mini_tuned/joint')
        self.declare_parameter('base_kp', 10.0)
        self.declare_parameter('base_kd', 0.30)
        self.declare_parameter('joint_names', default_joint_names)

        self.command_topic = self.get_parameter('command_topic').value
        self.input_joint_state_topic = self.get_parameter('input_joint_state_topic').value
        self.output_joint_state_topic = self.get_parameter('output_joint_state_topic').value
        self.force_topic_prefix = str(self.get_parameter('force_topic_prefix').value).rstrip('/')
        self.base_kp = float(self.get_parameter('base_kp').value)
        self.base_kd = float(self.get_parameter('base_kd').value)
        self.joint_names = list(self.get_parameter('joint_names').value)

        if len(self.joint_names) != 12:
            raise ValueError('joint_names must contain exactly 12 joints.')

        self.latest_positions = {}
        self.latest_velocities = {}
        self.latest_efforts = {joint_name: 0.0 for joint_name in self.joint_names}
        self.latest_joint_state_header = None
        self.latest_command = None
        self.warned_missing_state = False
        self.received_command_count = 0
        self.applied_command_count = 0

        self.force_publishers = {
            joint_name: self.create_publisher(
                Float64,
                f'{self.force_topic_prefix}/{joint_name}/cmd_force',
                10,
            )
            for joint_name in self.joint_names
        }
        self.joint_state_publisher = self.create_publisher(JointState, self.output_joint_state_topic, 20)

        self.create_subscription(JointState, self.input_joint_state_topic, self.joint_state_callback, 20)
        self.create_subscription(JointControlData, self.command_topic, self.command_callback, 20)
        self.apply_timer = self.create_timer(0.001, self.apply_latest_command)

        self.get_logger().info(
            f'Gazebo joint effort adapter started: {self.command_topic} -> {self.force_topic_prefix} '
            f'with joint states {self.input_joint_state_topic} -> {self.output_joint_state_topic} '
            f'(base_kp={self.base_kp:.3f}, base_kd={self.base_kd:.3f}, apply_rate_hz=1000)'
        )

    def joint_state_callback(self, msg: JointState):
        updated_positions = dict(self.latest_positions)
        updated_velocities = dict(self.latest_velocities)

        for index, joint_name in enumerate(msg.name):
            if joint_name not in self.force_publishers:
                continue

            if index < len(msg.position):
                updated_positions[joint_name] = float(msg.position[index])
            if index < len(msg.velocity):
                updated_velocities[joint_name] = float(msg.velocity[index])

        self.latest_positions = updated_positions
        self.latest_velocities = updated_velocities
        self.latest_joint_state_header = msg.header

        if self.latest_command is None:
            self.publish_joint_state_feedback()

    def command_callback(self, msg: JointControlData):
        expected_size = len(self.joint_names)
        if (
            len(msg.joint_position) != expected_size
            or len(msg.joint_velocity) != expected_size
            or len(msg.joint_torque) != expected_size
        ):
            self.get_logger().warn(
                'Ignoring joint_control_data with invalid sizes: '
                f'position={len(msg.joint_position)} velocity={len(msg.joint_velocity)} '
                f'torque={len(msg.joint_torque)} expected={expected_size}'
            )
            return

        self.latest_command = {
            'joint_torque': [float(value) for value in msg.joint_torque],
            'joint_position': [float(value) for value in msg.joint_position],
            'joint_velocity': [float(value) for value in msg.joint_velocity],
            'kp_ratio': self._sanitize_gain_ratio(msg.kp),
            'kd_ratio': self._sanitize_gain_ratio(msg.kd),
        }

        self.received_command_count += 1
        if self.received_command_count == 1:
            self.get_logger().info(
                f'Received first joint_control_data command. '
                f'Adapter will now reapply the latest PD+feedforward effort at 1 kHz.'
            )

    def apply_latest_command(self):
        if self.latest_command is None:
            return

        missing_joints = [joint_name for joint_name in self.joint_names if joint_name not in self.latest_positions]
        if missing_joints:
            if not self.warned_missing_state:
                self.get_logger().warn(
                    f'Waiting for joint state feedback before applying Gazebo efforts. '
                    f'Missing joints: {", ".join(missing_joints)}'
                )
                self.warned_missing_state = True
            return

        effective_kp = self.base_kp * self.latest_command['kp_ratio']
        effective_kd = self.base_kd * self.latest_command['kd_ratio']

        for index, joint_name in enumerate(self.joint_names):
            joint_position = self.latest_positions[joint_name]
            joint_velocity = self.latest_velocities.get(joint_name, 0.0)

            effort = (
                self.latest_command['joint_torque'][index]
                + effective_kp * (self.latest_command['joint_position'][index] - joint_position)
                + effective_kd * (self.latest_command['joint_velocity'][index] - joint_velocity)
            )
            self.latest_efforts[joint_name] = effort

            effort_msg = Float64()
            effort_msg.data = effort
            self.force_publishers[joint_name].publish(effort_msg)

        self.publish_joint_state_feedback()

        if self.warned_missing_state:
            self.warned_missing_state = False

        self.applied_command_count += 1
        if self.applied_command_count == 1:
            self.get_logger().info(
                f'Applying Gazebo efforts from the cached joint_control_data stream with '
                f'kp_ratio={self.latest_command["kp_ratio"]:.3f} '
                f'kd_ratio={self.latest_command["kd_ratio"]:.3f}.'
            )

    def publish_joint_state_feedback(self):
        if self.latest_joint_state_header is None:
            return

        msg = JointState()
        msg.header = self.latest_joint_state_header
        msg.name = list(self.joint_names)
        msg.position = [self.latest_positions.get(joint_name, 0.0) for joint_name in self.joint_names]
        msg.velocity = [self.latest_velocities.get(joint_name, 0.0) for joint_name in self.joint_names]
        msg.effort = [self.latest_efforts.get(joint_name, 0.0) for joint_name in self.joint_names]
        self.joint_state_publisher.publish(msg)

    @staticmethod
    def _sanitize_gain_ratio(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(0.0, float(value))


def main(args=None):
    rclpy.init(args=args)
    node = GazeboJointEffortAdapter()
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
