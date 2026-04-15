#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from legged_msgs.msg import JointControlData
from stm2ros.msg import JointControlState


class SimToRealBridge(Node):
    def __init__(self):
        super().__init__('sim2real_bridge')

        self.declare_parameter('input_topic', 'joint_control_data')
        self.declare_parameter('output_topic', 'joint_cmd')
        self.declare_parameter('default_position', 0.0)
        self.declare_parameter('default_effort', 12.0)
        self.declare_parameter('use_input_position', True)
        self.declare_parameter('use_input_effort', True)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('motors_enabled', True)
        self.declare_parameter('feedforward_torque_enabled', True)
        self.declare_parameter('enable_tuning_window', False)
        self.declare_parameter('gain_scale_min', 0.0)
        self.declare_parameter('gain_scale_max', 10.0)
        self.declare_parameter('gain_scale_resolution', 0.01)
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
            [],
        )

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.default_position = self.get_parameter('default_position').get_parameter_value().double_value
        self.default_effort = self.get_parameter('default_effort').get_parameter_value().double_value
        self.use_input_position = self.get_parameter('use_input_position').get_parameter_value().bool_value
        self.use_input_effort = self.get_parameter('use_input_effort').get_parameter_value().bool_value
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.motors_enabled = self.get_parameter('motors_enabled').get_parameter_value().bool_value
        self.feedforward_torque_enabled = self.get_parameter(
            'feedforward_torque_enabled'
        ).get_parameter_value().bool_value
        self.enable_tuning_window = self.get_parameter('enable_tuning_window').get_parameter_value().bool_value
        self.gain_scale_min = self.get_parameter('gain_scale_min').get_parameter_value().double_value
        self.gain_scale_max = self.get_parameter('gain_scale_max').get_parameter_value().double_value
        self.gain_scale_resolution = self.get_parameter('gain_scale_resolution').get_parameter_value().double_value
        self.source_joint_names = list(
            self.get_parameter('source_joint_names').get_parameter_value().string_array_value
        )
        self.output_joint_names = list(
            self.get_parameter('output_joint_names').get_parameter_value().string_array_value
        )
        self.invert_output_joints = set(
            self.get_parameter('invert_output_joints').get_parameter_value().string_array_value
        )
        self.gain_scale_max = max(self.gain_scale_min, self.gain_scale_max)
        self.gain_scale_resolution = max(0.001, self.gain_scale_resolution)

        self.source_to_target_map = {
            'LF_HAA': 'FL_HipX', 'LF_HFE': 'FL_HipY', 'LF_KFE': 'FL_Knee',
            'RF_HAA': 'FR_HipX', 'RF_HFE': 'FR_HipY', 'RF_KFE': 'FR_Knee',
            'LH_HAA': 'HL_HipX', 'LH_HFE': 'HL_HipY', 'LH_KFE': 'HL_Knee',
            'RH_HAA': 'HR_HipX', 'RH_HFE': 'HR_HipY', 'RH_KFE': 'HR_Knee',
        }
        self.target_to_source_map = {target: source for source, target in self.source_to_target_map.items()}
        self._updating_controls = False
        self._shutdown_started = False
        self.message_count = 0
        self.last_actuator_mode = None
        self.root = None
        self.status_var = None
        self.motor_toggle_button = None
        self.torque_toggle_button = None
        self.gain_scale_vars = {}
        self.gain_entry_vars = {}

        self.add_on_set_parameters_callback(self._handle_parameter_update)

        self.subscription = self.create_subscription(
            JointControlData,
            self.input_topic,
            self.control_callback,
            10,
        )
        self.publisher = self.create_publisher(JointControlState, self.output_topic, 10)

        if len(self.output_joint_names) != 12:
            raise ValueError('output_joint_names must contain 12 joint names for JointControlState.')

        if self.enable_tuning_window:
            self._setup_tuning_window()

        self.get_logger().info(
            f"Sim2Real bridge started: {self.input_topic} -> {self.output_topic}"
        )

    def control_callback(self, msg: JointControlData):
        position_by_name = self._vector_to_map(msg.joint_position)
        velocity_by_name = self._vector_to_map(msg.joint_velocity)
        torque_by_name = self._vector_to_map(msg.joint_torque)
        joint_position = []
        joint_velocity = []
        joint_torque = []

        for target_name in self.output_joint_names:
            source_name = self.target_to_source_map.get(target_name)
            if source_name is None:
                joint_position.append(float(self.default_position))
                joint_velocity.append(0.0)
                joint_torque.append(float(self.default_effort))
                continue

            if self.use_input_position:
                position = position_by_name.get(source_name, self.default_position)
            else:
                position = self.default_position

            velocity = velocity_by_name.get(source_name, 0.0)

            if self.use_input_effort:
                torque = torque_by_name.get(source_name, self.default_effort)
            else:
                torque = self.default_effort

            if target_name in self.invert_output_joints:
                position *= -1.0
                velocity *= -1.0
                torque *= -1.0

            joint_position.append(float(position))
            joint_velocity.append(float(1.2*abs(velocity)))
            #joint_velocity.append(3.0)
            #joint_torque.append(abs(float(torque)))
            joint_torque.append(12.0)

        self.message_count += 1
        self.last_actuator_mode = int(msg.actuator_mode)

        joint_cmd = JointControlState()
        joint_cmd.joint_names = list(self.output_joint_names)
        torque_enabled = self.motors_enabled and self.feedforward_torque_enabled
        joint_cmd.joint_torque = joint_torque if torque_enabled else [0.0] * len(joint_torque)
        joint_cmd.joint_position = joint_position
        joint_cmd.joint_velocity = joint_velocity
        joint_cmd.kp = float(self.kp if self.motors_enabled else 0.0)
        joint_cmd.kd = float(self.kd if self.motors_enabled else 0.0)

        self.publisher.publish(joint_cmd)
        self._update_status_text()

    def _vector_to_map(self, values):
        return {
            joint_name: float(values[index])
            for index, joint_name in enumerate(self.source_joint_names)
            if index < len(values)
        }

    def _handle_parameter_update(self, params):
        new_kp = self.kp
        new_kd = self.kd
        new_motors_enabled = self.motors_enabled
        new_feedforward_torque_enabled = self.feedforward_torque_enabled

        for param in params:
            if param.name == 'kp':
                new_kp = float(param.value)
            elif param.name == 'kd':
                new_kd = float(param.value)
            elif param.name == 'motors_enabled':
                new_motors_enabled = bool(param.value)
            elif param.name == 'feedforward_torque_enabled':
                new_feedforward_torque_enabled = bool(param.value)

        self.kp = new_kp
        self.kd = new_kd
        self.motors_enabled = new_motors_enabled
        self.feedforward_torque_enabled = new_feedforward_torque_enabled
        self._sync_gain_controls()
        self._sync_motor_button()
        self._sync_torque_button()
        self._update_status_text()
        return SetParametersResult(successful=True)

    def _setup_tuning_window(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.root = None
            self.get_logger().warn(
                f"Could not open tuning window ({exc}). Continuing without GUI tuning."
            )
            return

        self.root.title('Sim2Real Gain Tuning')
        self.root.geometry('440x280')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)

        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill='both', expand=True)
        frame.columnconfigure(1, weight=1)

        ttk.Label(
            frame,
            text='Live gain ratios. 1.0 means no change.',
            anchor='center',
        ).grid(row=0, column=0, columnspan=4, sticky='ew', pady=(0, 10))

        self._build_gain_row(frame, row=1, gain_name='kp', label='KP Ratio')
        self._build_gain_row(frame, row=2, gain_name='kd', label='KD Ratio')

        self.motor_toggle_button = tk.Button(
            frame,
            command=self._toggle_motors,
            font=('Arial', 11, 'bold'),
            width=18,
            relief=tk.RAISED,
        )
        self.motor_toggle_button.grid(row=3, column=0, columnspan=4, sticky='ew', pady=(10, 0))

        self.torque_toggle_button = tk.Button(
            frame,
            command=self._toggle_feedforward_torque,
            font=('Arial', 11, 'bold'),
            width=18,
            relief=tk.RAISED,
        )
        self.torque_toggle_button.grid(row=4, column=0, columnspan=4, sticky='ew', pady=(8, 0))

        self.status_var = tk.StringVar(
            value=f'Waiting for {self.input_topic}...'
        )
        ttk.Label(
            frame,
            textvariable=self.status_var,
            anchor='center',
        ).grid(row=5, column=0, columnspan=4, sticky='ew', pady=(12, 0))

        ttk.Label(
            frame,
            text=f'Publishing to {self.output_topic}',
            anchor='center',
        ).grid(row=6, column=0, columnspan=4, sticky='ew', pady=(6, 0))

        self._sync_gain_controls()
        self._sync_motor_button()
        self._sync_torque_button()

    def _build_gain_row(self, parent, row, gain_name, label):
        current_value = self._get_gain_value(gain_name)
        scale_var = tk.DoubleVar(value=current_value)
        entry_var = tk.StringVar(value=self._format_gain(current_value))
        self.gain_scale_vars[gain_name] = scale_var
        self.gain_entry_vars[gain_name] = entry_var

        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=(0, 10), pady=6)

        scale = tk.Scale(
            parent,
            from_=self.gain_scale_min,
            to=self.gain_scale_max,
            resolution=self.gain_scale_resolution,
            orient=tk.HORIZONTAL,
            showvalue=False,
            length=220,
            variable=scale_var,
            command=lambda value, name=gain_name: self._on_scale_change(name, value),
        )
        scale.grid(row=row, column=1, sticky='ew', pady=6)

        entry = ttk.Entry(parent, width=8, textvariable=entry_var)
        entry.grid(row=row, column=2, padx=8, pady=6)
        entry.bind('<Return>', lambda _event, name=gain_name: self._on_entry_commit(name))
        entry.bind('<FocusOut>', lambda _event, name=gain_name: self._on_entry_commit(name))

        ttk.Button(
            parent,
            text='Reset',
            command=lambda name=gain_name: self._set_gain_from_ui(name, 1.0),
        ).grid(row=row, column=3, pady=6)

    def _on_scale_change(self, gain_name, value):
        if self._updating_controls:
            return
        snapped_value = self._snap_gain(float(value))
        self._set_gain_from_ui(gain_name, snapped_value)

    def _on_entry_commit(self, gain_name):
        if self._updating_controls:
            return

        raw_value = self.gain_entry_vars[gain_name].get().strip()
        try:
            parsed_value = float(raw_value)
        except ValueError:
            self._sync_gain_controls()
            return

        self._set_gain_from_ui(gain_name, parsed_value)

    def _set_gain_from_ui(self, gain_name, value):
        clamped_value = self._clamp_gain(value)
        current_value = self._get_gain_value(gain_name)

        if abs(clamped_value - current_value) < 1e-9:
            self._sync_gain_controls()
            return

        self.set_parameters([Parameter(gain_name, value=float(clamped_value))])

    def _get_gain_value(self, gain_name):
        return self.kp if gain_name == 'kp' else self.kd

    def _clamp_gain(self, value):
        return max(self.gain_scale_min, min(self.gain_scale_max, float(value)))

    def _snap_gain(self, value):
        steps = round((value - self.gain_scale_min) / self.gain_scale_resolution)
        snapped = self.gain_scale_min + steps * self.gain_scale_resolution
        return self._clamp_gain(snapped)

    def _format_gain(self, value):
        return f'{float(value):.2f}'

    def _sync_gain_controls(self):
        if self.root is None:
            return
        self.root.after_idle(self._apply_gain_controls)

    def _apply_gain_controls(self):
        if self.root is None:
            return

        self._updating_controls = True
        try:
            for gain_name in ('kp', 'kd'):
                value = self._get_gain_value(gain_name)
                self.gain_scale_vars[gain_name].set(value)
                self.gain_entry_vars[gain_name].set(self._format_gain(value))
        finally:
            self._updating_controls = False

    def _toggle_motors(self):
        self.set_parameters([Parameter('motors_enabled', value=not self.motors_enabled)])

    def _sync_motor_button(self):
        if self.root is None or self.motor_toggle_button is None:
            return
        self.root.after_idle(self._apply_motor_button)

    def _apply_motor_button(self):
        if self.motor_toggle_button is None:
            return

        if self.motors_enabled:
            self.motor_toggle_button.config(
                text='Motor Output: ON',
                bg='#2E8B57',
                fg='white',
                activebackground='#256f46',
                activeforeground='white',
            )
        else:
            self.motor_toggle_button.config(
                text='Motor Output: OFF',
                bg='#B22222',
                fg='white',
                activebackground='#8f1b1b',
                activeforeground='white',
            )

    def _toggle_feedforward_torque(self):
        self.set_parameters([
            Parameter(
                'feedforward_torque_enabled',
                value=not self.feedforward_torque_enabled,
            )
        ])

    def _sync_torque_button(self):
        if self.root is None or self.torque_toggle_button is None:
            return
        self.root.after_idle(self._apply_torque_button)

    def _apply_torque_button(self):
        if self.torque_toggle_button is None:
            return

        if self.feedforward_torque_enabled:
            self.torque_toggle_button.config(
                text='Feedforward Torque: ON',
                bg='#4169E1',
                fg='white',
                activebackground='#3354b4',
                activeforeground='white',
            )
        else:
            self.torque_toggle_button.config(
                text='Feedforward Torque: OFF',
                bg='#696969',
                fg='white',
                activebackground='#555555',
                activeforeground='white',
            )

    def _update_status_text(self):
        if self.status_var is None:
            return

        status_text = (
            f'output={"ON" if self.motors_enabled else "OFF"} | '
            f'torque_ff={"ON" if self.feedforward_torque_enabled else "OFF"} | '
            f'Msgs: {self.message_count} | '
            f'mode: {self.last_actuator_mode if self.last_actuator_mode is not None else "-"} | '
            f'kp={self._format_gain(self.kp)} kd={self._format_gain(self.kd)}'
        )

        if self.root is None:
            return
        self.root.after_idle(lambda: self.status_var.set(status_text))

    def run(self):
        if self.root is None:
            rclpy.spin(self)
            return

        self.root.after(1, self._pump_ros)
        self.root.mainloop()

    def _pump_ros(self):
        if self.root is None or self._shutdown_started or not rclpy.ok():
            return

        rclpy.spin_once(self, timeout_sec=0.0)
        if self.root is not None:
            self.root.after(1, self._pump_ros)

    def shutdown(self):
        if self._shutdown_started:
            return

        self._shutdown_started = True

        if self.root is not None:
            root = self.root
            self.root = None
            try:
                root.quit()
                root.destroy()
            except tk.TclError:
                pass

        if rclpy.ok():
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimToRealBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
