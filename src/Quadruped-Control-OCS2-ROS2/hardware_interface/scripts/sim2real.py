#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from legged_msgs.msg import SimulatorStateData

#ros2 run hardware_interface sim2real.py

class SimToRealBridge(Node):
    def __init__(self):
        super().__init__('sim2real_bridge')

        self.declare_parameter('input_topic', 'simulator_state_data')
        self.declare_parameter('output_topic', 'joint_cmd')
        self.declare_parameter('default_velocity', 5.0)
        self.declare_parameter('default_effort', 12.0)
        self.declare_parameter('use_input_velocity', True)
        self.declare_parameter('use_input_effort', True)
        self.declare_parameter(
            'source_joint_names',
            [
                'LF_HAA', 'LF_HFE', 'LF_KFE',
                'LH_HAA', 'LH_HFE', 'LH_KFE',
                'RF_HAA', 'RF_HFE', 'RF_KFE',
                'RH_HAA', 'RH_HFE', 'RH_KFE',
            ],
        )
        self.declare_parameter(
            'output_joint_names',
            [
                'FL_HipX', 'FL_HipY', 'FL_Knee',
                'FR_HipX', 'FR_HipY', 'FR_Knee',
                'HL_HipX', 'HL_HipY', 'HL_Knee',
                'HR_HipX', 'HR_HipY', 'HR_Knee',
            ],
        )
        self.declare_parameter(
            'invert_output_joints',
            ['FR_Knee', 'HR_Knee'],
        )

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.default_velocity = self.get_parameter('default_velocity').get_parameter_value().double_value
        self.default_effort = self.get_parameter('default_effort').get_parameter_value().double_value
        self.use_input_velocity = self.get_parameter('use_input_velocity').get_parameter_value().bool_value
        self.use_input_effort = self.get_parameter('use_input_effort').get_parameter_value().bool_value
        self.source_joint_names = list(
            self.get_parameter('source_joint_names').get_parameter_value().string_array_value
        )
        self.output_joint_names = list(
            self.get_parameter('output_joint_names').get_parameter_value().string_array_value
        )
        self.invert_output_joints = set(
            self.get_parameter('invert_output_joints').get_parameter_value().string_array_value
        )

        self.source_to_target_map = {
            'LF_HAA': 'FL_HipX', 'LF_HFE': 'FL_HipY', 'LF_KFE': 'FL_Knee',
            'RF_HAA': 'FR_HipX', 'RF_HFE': 'FR_HipY', 'RF_KFE': 'FR_Knee',
            'LH_HAA': 'HL_HipX', 'LH_HFE': 'HL_HipY', 'LH_KFE': 'HL_Knee',
            'RH_HAA': 'HR_HipX', 'RH_HFE': 'HR_HipY', 'RH_KFE': 'HR_Knee',
        }
        self.target_to_source_map = {target: source for source, target in self.source_to_target_map.items()}

        self.subscription = self.create_subscription(
            SimulatorStateData,
            self.input_topic,
            self.state_callback,
            10,
        )
        self.publisher = self.create_publisher(JointTrajectory, self.output_topic, 10)

        self.get_logger().info(
            f"Sim2Real bridge started: {self.input_topic} -> {self.output_topic}"
        )

    def state_callback(self, msg: SimulatorStateData):
        position_by_name = self._vector_to_map(msg.joint_position_values)
        velocity_by_name = self._vector_to_map(msg.joint_velocity_values)
        effort_by_name = self._vector_to_map(msg.joint_torque_values)

        if not position_by_name:
            self.get_logger().warn('Received SimulatorStateData without joint positions. Skipping.')
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.output_joint_names)

        for target_name in self.output_joint_names:
            source_name = self.target_to_source_map.get(target_name)
            if source_name is None:
                continue

            position = position_by_name.get(source_name, 0.0)
            velocity = velocity_by_name.get(source_name, self.default_velocity)
            effort = effort_by_name.get(source_name, self.default_effort)

            if target_name in self.invert_output_joints:
                position *= -1.0
                velocity *= -1.0
                effort *= -1.0

            point = JointTrajectoryPoint()
            point.positions = [float(position)]
            point.velocities = [0.0]
            point.effort = [0.0]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0
            trajectory.points.append(point)

        self.publisher.publish(trajectory)

    def _vector_to_map(self, values):
        return {
            joint_name: float(values[index])
            for index, joint_name in enumerate(self.source_joint_names)
            if index < len(values)
        }


def main(args=None):
    rclpy.init(args=args)
    node = SimToRealBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
