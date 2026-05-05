#!/usr/bin/env python3

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stm2ros.msg import JointControlState


class OneLegControlGui(Node):
    def __init__(self):
        super().__init__('one_leg_control_gui')

        self.declare_parameter('controller_node', '/one_leg_inverse_dynamics_node')
        self.declare_parameter('state_topic', 'htdw_joint_state')
        self.declare_parameter('command_topic', 'joint_cmd')
        self.declare_parameter('active_joint_names', ['FL_HipX', 'FL_HipY', 'FL_Knee'])
        self.declare_parameter('position_min', [-0.8, -1.2, -2.3])
        self.declare_parameter('position_max', [0.8, 1.2, 0.1])
        self.declare_parameter('default_desired_positions', [0.0, 0.0, 0.0])
        self.declare_parameter('default_desired_velocities', [0.0, 0.0, 0.0])
        self.declare_parameter('default_desired_accelerations', [0.0, 0.0, 0.0])
        self.declare_parameter('default_id_kp', [1.5, 1.5, 1.0])
        self.declare_parameter('default_id_kd', [0.08, 0.08, 0.05])
        self.declare_parameter('default_max_torque', 6.0)
        self.declare_parameter('default_command_kp', 0.0)
        self.declare_parameter('default_command_kd', 0.0)
        self.declare_parameter('default_feedforward_torque_enabled', False)
        self.declare_parameter('default_joint_feedforward_enabled', [0.0, 0.0, 0.0])

        self.controller_node = self.get_parameter('controller_node').value
        self.state_topic = self.get_parameter('state_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.active_joint_names = list(self.get_parameter('active_joint_names').value)
        self.position_min = self._fit_list(list(self.get_parameter('position_min').value), -1.0)
        self.position_max = self._fit_list(list(self.get_parameter('position_max').value), 1.0)
        self.default_desired_positions = self._fit_list(
            list(self.get_parameter('default_desired_positions').value), 0.0
        )
        self.default_desired_velocities = self._fit_list(
            list(self.get_parameter('default_desired_velocities').value), 0.0
        )
        self.default_desired_accelerations = self._fit_list(
            list(self.get_parameter('default_desired_accelerations').value), 0.0
        )
        self.default_id_kp = self._fit_list(list(self.get_parameter('default_id_kp').value), 0.0)
        self.default_id_kd = self._fit_list(list(self.get_parameter('default_id_kd').value), 0.0)
        self.default_max_torque = float(self.get_parameter('default_max_torque').value)
        self.default_command_kp = float(self.get_parameter('default_command_kp').value)
        self.default_command_kd = float(self.get_parameter('default_command_kd').value)
        self.default_feedforward_torque_enabled = bool(
            self.get_parameter('default_feedforward_torque_enabled').value
        )
        self.default_joint_feedforward_enabled = self._fit_list(
            list(self.get_parameter('default_joint_feedforward_enabled').value), 0.0
        )

        self.set_parameters_client = self.create_client(SetParameters, f'{self.controller_node}/set_parameters')
        self.get_parameters_client = self.create_client(GetParameters, f'{self.controller_node}/get_parameters')

        self.latest_state = None
        self.latest_command = None
        self.state_count = 0
        self.command_count = 0
        self.controller_params_loaded = False
        self.get_params_inflight = False

        self.create_subscription(JointState, self.state_topic, self._state_callback, 10)
        self.create_subscription(JointControlState, self.command_topic, self._command_callback, 10)

        self.root = tk.Tk()
        self.root.title('One Leg Torque Control')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self.status_var = tk.StringVar(value='Controller: checking...')
        self.state_var = tk.StringVar(value='State: no message yet')
        self.command_var = tk.StringVar(value='Command: no message yet')
        self.enabled_var = tk.StringVar(value='Disabled')
        self.soft_stop_var = tk.StringVar(value='Soft stop: unknown')
        self.last_action_var = tk.StringVar(value='Start disabled. Verify topics before enabling.')

        self.desired_vars = []
        self.id_kp_vars = []
        self.id_kp_labels = []
        self.id_kd_vars = []
        self.id_kd_labels = []
        self.torque_vars = []
        self.torque_labels = []
        self.command_kp_var = tk.DoubleVar(value=self.default_command_kp)
        self.command_kd_var = tk.DoubleVar(value=self.default_command_kd)
        self.feedforward_vars = [
            tk.BooleanVar(value=self.default_feedforward_torque_enabled and value >= 0.5)
            for value in self.default_joint_feedforward_enabled
        ]
        self.feedback_rows = {}

        self._build_layout()
        self._refresh_controller_params()
        self.root.after(50, self._spin_ros)
        self.root.after(250, self._refresh_status_text)

    def _fit_list(self, values, fallback):
        if len(values) >= len(self.active_joint_names):
            return [float(v) for v in values[: len(self.active_joint_names)]]
        return [float(fallback)] * len(self.active_joint_names)

    def _build_layout(self):
        self.root.columnconfigure(0, weight=1)

        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky='nsew')
        main.columnconfigure(0, weight=1)

        title = ttk.Label(main, text='One Leg Torque Control', font=('TkDefaultFont', 14, 'bold'))
        title.grid(row=0, column=0, sticky='w')

        ttk.Label(main, textvariable=self.status_var).grid(row=1, column=0, sticky='w', pady=(8, 0))
        ttk.Label(main, textvariable=self.state_var).grid(row=2, column=0, sticky='w')
        ttk.Label(main, textvariable=self.command_var).grid(row=3, column=0, sticky='w')
        ttk.Label(main, textvariable=self.last_action_var).grid(row=4, column=0, sticky='w', pady=(0, 10))

        stop_button = tk.Button(
            main,
            text='MOTOR STOP: ZERO KP / KD / T_FF',
            command=self._motor_stop_zero_all,
            bg='#b00020',
            fg='white',
            activebackground='#7f0017',
            activeforeground='white',
            font=('TkDefaultFont', 13, 'bold'),
            relief='raised',
            height=2,
        )
        stop_button.grid(row=5, column=0, sticky='ew', pady=(4, 8))

        button_row = ttk.Frame(main)
        button_row.grid(row=6, column=0, sticky='ew', pady=(4, 8))
        for column in range(4):
            button_row.columnconfigure(column, weight=1)

        ttk.Button(button_row, text='Enable', command=self._enable).grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(button_row, text='Disable', command=self._disable).grid(row=0, column=1, sticky='ew', padx=4)
        ttk.Button(button_row, text='Soft Stop', command=self._soft_stop).grid(row=0, column=2, sticky='ew', padx=4)
        ttk.Button(button_row, text='Clear Fault', command=self._clear_fault).grid(row=0, column=3, sticky='ew', padx=(4, 0))

        ttk.Label(main, textvariable=self.enabled_var, font=('TkDefaultFont', 11, 'bold')).grid(
            row=7, column=0, sticky='w'
        )
        ttk.Label(main, textvariable=self.soft_stop_var).grid(row=8, column=0, sticky='w', pady=(0, 10))

        sliders = ttk.LabelFrame(main, text='Desired Position')
        sliders.grid(row=9, column=0, sticky='ew')
        sliders.columnconfigure(1, weight=1)

        for index, joint_name in enumerate(self.active_joint_names):
            value = tk.DoubleVar(value=self.default_desired_positions[index])
            self.desired_vars.append(value)
            ttk.Label(sliders, text=joint_name).grid(row=index, column=0, sticky='w', padx=8, pady=4)
            ttk.Scale(
                sliders,
                from_=self.position_min[index],
                to=self.position_max[index],
                variable=value,
                command=lambda _unused=None: self._update_slider_labels(),
            ).grid(row=index, column=1, sticky='ew', padx=8, pady=4)
            label = ttk.Label(sliders, text='0.000 rad', width=12)
            label.grid(row=index, column=2, sticky='e', padx=8, pady=4)
            value.label = label

        id_gains = ttk.LabelFrame(main, text='Inverse Dynamics des_acc Gains')
        id_gains.grid(row=10, column=0, sticky='ew', pady=(10, 0))
        id_gains.columnconfigure(1, weight=1)
        id_gains.columnconfigure(3, weight=1)

        ttk.Label(id_gains, text='joint').grid(row=0, column=0, sticky='w', padx=8, pady=(4, 2))
        ttk.Label(id_gains, text='id_kp').grid(row=0, column=1, sticky='w', padx=8, pady=(4, 2))
        ttk.Label(id_gains, text='id_kd').grid(row=0, column=3, sticky='w', padx=8, pady=(4, 2))

        for row, joint_name in enumerate(self.active_joint_names, start=1):
            kp_var = tk.DoubleVar(value=self.default_id_kp[row - 1])
            kd_var = tk.DoubleVar(value=self.default_id_kd[row - 1])
            self.id_kp_vars.append(kp_var)
            self.id_kd_vars.append(kd_var)

            ttk.Label(id_gains, text=joint_name).grid(row=row, column=0, sticky='w', padx=8, pady=4)
            ttk.Scale(
                id_gains,
                from_=0.0,
                to=100.0,
                variable=kp_var,
                command=lambda _unused=None: self._update_slider_labels(),
            ).grid(row=row, column=1, sticky='ew', padx=8, pady=4)
            kp_label = ttk.Label(id_gains, text='0.000', width=10)
            kp_label.grid(row=row, column=2, sticky='e', padx=8, pady=4)
            self.id_kp_labels.append(kp_label)

            ttk.Scale(
                id_gains,
                from_=0.0,
                to=50.0,
                variable=kd_var,
                command=lambda _unused=None: self._update_slider_labels(),
            ).grid(row=row, column=3, sticky='ew', padx=8, pady=4)
            kd_label = ttk.Label(id_gains, text='0.000', width=10)
            kd_label.grid(row=row, column=4, sticky='e', padx=8, pady=4)
            self.id_kd_labels.append(kd_label)

        output_row = ttk.LabelFrame(main, text='STM Output: t_ff + PD')
        output_row.grid(row=11, column=0, sticky='ew', pady=(10, 0))
        output_row.columnconfigure(1, weight=1)

        ttk.Label(output_row, text='t_ff enable').grid(row=0, column=0, sticky='w', padx=8, pady=4)
        tff_enable_row = ttk.Frame(output_row)
        tff_enable_row.grid(row=0, column=1, columnspan=2, sticky='ew', padx=8, pady=4)
        for column, joint_name in enumerate(self.active_joint_names):
            tff_enable_row.columnconfigure(column, weight=1)
            ttk.Checkbutton(
                tff_enable_row,
                text=joint_name,
                variable=self.feedforward_vars[column],
                command=self._apply_sliders,
            ).grid(row=0, column=column, sticky='w')

        for index, joint_name in enumerate(self.active_joint_names):
            torque_var = tk.DoubleVar(value=self.default_max_torque)
            self.torque_vars.append(torque_var)
            ttk.Label(output_row, text=f'{joint_name} t_ff limit').grid(
                row=1 + index, column=0, sticky='w', padx=8, pady=4
            )
            ttk.Scale(
                output_row,
                from_=0.0,
                to=12.0,
                variable=torque_var,
                command=lambda _unused=None: self._update_slider_labels(),
            ).grid(row=1 + index, column=1, sticky='ew', padx=8, pady=4)
            torque_label = ttk.Label(output_row, text='0.000 Nm', width=12)
            torque_label.grid(row=1 + index, column=2, sticky='e', padx=8, pady=4)
            self.torque_labels.append(torque_label)

        pd_row = 1 + len(self.active_joint_names)
        ttk.Label(output_row, text='kp').grid(row=pd_row, column=0, sticky='w', padx=8, pady=4)
        ttk.Scale(
            output_row,
            from_=0.0,
            to=50.0,
            variable=self.command_kp_var,
            command=lambda _unused=None: self._update_slider_labels(),
        ).grid(row=pd_row, column=1, sticky='ew', padx=8, pady=4)
        self.command_kp_label = ttk.Label(output_row, text='0.000', width=12)
        self.command_kp_label.grid(row=pd_row, column=2, sticky='e', padx=8, pady=4)

        ttk.Label(output_row, text='kd').grid(row=pd_row + 1, column=0, sticky='w', padx=8, pady=4)
        ttk.Scale(
            output_row,
            from_=0.0,
            to=5.0,
            variable=self.command_kd_var,
            command=lambda _unused=None: self._update_slider_labels(),
        ).grid(row=pd_row + 1, column=1, sticky='ew', padx=8, pady=4)
        self.command_kd_label = ttk.Label(output_row, text='0.000', width=12)
        self.command_kd_label.grid(row=pd_row + 1, column=2, sticky='e', padx=8, pady=4)

        ttk.Label(
            output_row,
            text='kp/kd are global in stm2ros/JointControlState.',
        ).grid(row=pd_row + 2, column=0, columnspan=3, sticky='w', padx=8, pady=(2, 6))

        apply_row = ttk.Frame(main)
        apply_row.grid(row=12, column=0, sticky='ew', pady=(10, 0))
        apply_row.columnconfigure(0, weight=1)
        apply_row.columnconfigure(1, weight=1)
        apply_row.columnconfigure(2, weight=1)
        ttk.Button(apply_row, text='Apply Sliders', command=self._apply_sliders).grid(
            row=0, column=0, sticky='ew', padx=(0, 4)
        )
        ttk.Button(apply_row, text='Gravity Comp', command=self._gravity_comp_mode).grid(
            row=0, column=1, sticky='ew', padx=4
        )
        ttk.Button(apply_row, text='Reset Safe Defaults', command=self._reset_defaults).grid(
            row=0, column=2, sticky='ew', padx=(4, 0)
        )
        for column, joint_name in enumerate(self.active_joint_names):
            ttk.Button(
                apply_row,
                text=f'Grav {joint_name}',
                command=lambda index=column: self._gravity_comp_joint_mode(index),
            ).grid(row=1, column=column, sticky='ew', padx=4, pady=(6, 0))

        feedback = ttk.LabelFrame(main, text='Feedback')
        feedback.grid(row=13, column=0, sticky='ew', pady=(10, 0))
        for column in range(5):
            feedback.columnconfigure(column, weight=1)

        headers = ['joint', 'position', 'velocity', 'effort', 'cmd t_ff']
        for column, header in enumerate(headers):
            ttk.Label(feedback, text=header).grid(row=0, column=column, sticky='ew', padx=6, pady=(4, 2))

        for row, joint_name in enumerate(self.active_joint_names, start=1):
            row_vars = {
                'position': tk.StringVar(value='--'),
                'velocity': tk.StringVar(value='--'),
                'effort': tk.StringVar(value='--'),
                'command': tk.StringVar(value='--'),
            }
            self.feedback_rows[joint_name] = row_vars
            ttk.Label(feedback, text=joint_name).grid(row=row, column=0, sticky='w', padx=6, pady=2)
            ttk.Label(feedback, textvariable=row_vars['position']).grid(row=row, column=1, sticky='e', padx=6, pady=2)
            ttk.Label(feedback, textvariable=row_vars['velocity']).grid(row=row, column=2, sticky='e', padx=6, pady=2)
            ttk.Label(feedback, textvariable=row_vars['effort']).grid(row=row, column=3, sticky='e', padx=6, pady=2)
            ttk.Label(feedback, textvariable=row_vars['command']).grid(row=row, column=4, sticky='e', padx=6, pady=2)

        hint = ttk.Label(
            main,
            text='Only the first three joint_cmd slots are active. Check zero torque before enabling.',
        )
        hint.grid(row=14, column=0, sticky='w', pady=(12, 0))

        self._update_slider_labels()

    def _state_callback(self, msg):
        self.latest_state = msg
        self.state_count += 1

    def _command_callback(self, msg):
        self.latest_command = msg
        self.command_count += 1

    def _set_params(self, values):
        if not self.set_parameters_client.service_is_ready():
            self.last_action_var.set('Controller parameter service is not ready.')
            return

        request = SetParameters.Request()
        request.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        future = self.set_parameters_client.call_async(request)
        future.add_done_callback(self._set_params_done)

    def _make_parameter(self, name, value):
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()

        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        elif isinstance(value, list):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter.value.double_array_value = [float(v) for v in value]
        else:
            raise TypeError(f'Unsupported parameter type for {name}')

        return parameter

    def _set_params_done(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.last_action_var.set(f'Parameter update failed: {error}')
            return

        failed = [result.reason for result in response.results if not result.successful]
        if failed:
            self.last_action_var.set(f'Parameter rejected: {failed[0]}')
        else:
            self.last_action_var.set('Parameter update accepted.')
            self._refresh_controller_params()

    def _refresh_controller_params(self):
        if self.get_params_inflight or not self.get_parameters_client.service_is_ready():
            return

        request = GetParameters.Request()
        request.names = [
            'enabled',
            'soft_stop',
            'desired_positions',
            'id_kp',
            'id_kd',
            'max_torque',
            'command_kp',
            'command_kd',
            'feedforward_torque_enabled',
            'joint_feedforward_enabled',
        ]
        self.get_params_inflight = True
        future = self.get_parameters_client.call_async(request)
        future.add_done_callback(self._get_params_done)

    def _get_params_done(self, future):
        try:
            values = future.result().values
        except Exception:
            self.get_params_inflight = False
            return

        if len(values) < 10:
            self.get_params_inflight = False
            return

        self.enabled_var.set('Enabled' if values[0].bool_value else 'Disabled')
        self.soft_stop_var.set(f'Soft stop: {"on" if values[1].bool_value else "off"}')

        desired = list(values[2].double_array_value)
        if len(desired) == len(self.desired_vars):
            for index, value in enumerate(desired):
                self.desired_vars[index].set(float(value))

        id_kp = list(values[3].double_array_value)
        if len(id_kp) == len(self.id_kp_vars):
            for index, value in enumerate(id_kp):
                self.id_kp_vars[index].set(float(value))

        id_kd = list(values[4].double_array_value)
        if len(id_kd) == len(self.id_kd_vars):
            for index, value in enumerate(id_kd):
                self.id_kd_vars[index].set(float(value))

        torques = list(values[5].double_array_value)
        if len(torques) == len(self.torque_vars):
            for index, value in enumerate(torques):
                self.torque_vars[index].set(float(abs(value)))

        self.command_kp_var.set(float(values[6].double_value))
        self.command_kd_var.set(float(values[7].double_value))
        global_feedforward_enabled = bool(values[8].bool_value)
        joint_feedforward = list(values[9].double_array_value)
        if len(joint_feedforward) == len(self.feedforward_vars):
            for index, value in enumerate(joint_feedforward):
                self.feedforward_vars[index].set(global_feedforward_enabled and float(value) >= 0.5)

        self._update_slider_labels()
        self.controller_params_loaded = True
        self.get_params_inflight = False

    def _enable(self):
        self._set_params({'soft_stop': False, 'enabled': True})
        self.last_action_var.set('Enable requested.')

    def _disable(self):
        self._set_params({'enabled': False})
        self.last_action_var.set('Disable requested.')

    def _soft_stop(self):
        self._set_params({'soft_stop': True, 'enabled': False})
        self.last_action_var.set('Soft stop requested.')

    def _motor_stop_zero_all(self):
        for var in self.torque_vars:
            var.set(0.0)
        for var in self.id_kp_vars:
            var.set(0.0)
        for var in self.id_kd_vars:
            var.set(0.0)
        self.command_kp_var.set(0.0)
        self.command_kd_var.set(0.0)
        for var in self.feedforward_vars:
            var.set(False)
        self._update_slider_labels()
        self._set_params({
            'enabled': False,
            'soft_stop': True,
            'feedforward_torque_enabled': False,
            'joint_feedforward_enabled': [0.0] * len(self.active_joint_names),
            'id_kp': [0.0] * len(self.active_joint_names),
            'id_kd': [0.0] * len(self.active_joint_names),
            'command_kp': 0.0,
            'command_kd': 0.0,
            'max_torque': [0.0] * len(self.active_joint_names),
        })
        self.last_action_var.set('MOTOR STOP requested: kp=0, kd=0, t_ff=0.')

    def _clear_fault(self):
        self._set_params({'enabled': False, 'soft_stop': False, 'clear_faults': True})
        self.last_action_var.set('Fault clear requested. Controller remains disabled.')

    def _apply_sliders(self):
        desired = [var.get() for var in self.desired_vars]
        id_kp = [max(0.0, var.get()) for var in self.id_kp_vars]
        id_kd = [max(0.0, var.get()) for var in self.id_kd_vars]
        torques = [max(0.0, var.get()) for var in self.torque_vars]
        joint_feedforward_enabled = [1.0 if var.get() else 0.0 for var in self.feedforward_vars]
        self._set_params({
            'desired_positions': desired,
            'id_kp': id_kp,
            'id_kd': id_kd,
            'max_torque': torques,
            'command_kp': max(0.0, self.command_kp_var.get()),
            'command_kd': max(0.0, self.command_kd_var.get()),
            'feedforward_torque_enabled': any(value >= 0.5 for value in joint_feedforward_enabled),
            'joint_feedforward_enabled': joint_feedforward_enabled,
        })
        self.last_action_var.set('Applying desired positions, ID gains, per-joint t_ff, kp, and kd.')

    def _gravity_comp_mode(self):
        joint_feedforward_enabled = [1.0 if var.get() else 0.0 for var in self.feedforward_vars]
        self._apply_gravity_comp(joint_feedforward_enabled, 'Gravity comp mode applied to selected t_ff joints.')

    def _gravity_comp_joint_mode(self, joint_index):
        joint_feedforward_enabled = [0.0] * len(self.active_joint_names)
        joint_feedforward_enabled[joint_index] = 1.0
        for index, var in enumerate(self.feedforward_vars):
            var.set(index == joint_index)
        self._apply_gravity_comp(
            joint_feedforward_enabled,
            f'Gravity comp mode applied to {self.active_joint_names[joint_index]} only.',
        )

    def _apply_gravity_comp(self, joint_feedforward_enabled, status):
        for var in self.id_kp_vars:
            var.set(0.0)
        for var in self.id_kd_vars:
            var.set(0.0)
        self.command_kp_var.set(0.0)
        self.command_kd_var.set(0.0)
        self._update_slider_labels()
        self._set_params({
            'desired_velocities': [0.0] * len(self.active_joint_names),
            'desired_accelerations': [0.0] * len(self.active_joint_names),
            'id_kp': [0.0] * len(self.active_joint_names),
            'id_kd': [0.0] * len(self.active_joint_names),
            'max_torque': [max(0.0, var.get()) for var in self.torque_vars],
            'command_kp': 0.0,
            'command_kd': 0.0,
            'feedforward_torque_enabled': any(value >= 0.5 for value in joint_feedforward_enabled),
            'joint_feedforward_enabled': joint_feedforward_enabled,
        })
        self.last_action_var.set(status)

    def _reset_defaults(self):
        for index, value in enumerate(self.default_desired_positions):
            self.desired_vars[index].set(value)
        for index, value in enumerate(self.default_id_kp):
            self.id_kp_vars[index].set(value)
        for index, value in enumerate(self.default_id_kd):
            self.id_kd_vars[index].set(value)
        for var in self.torque_vars:
            var.set(self.default_max_torque)
        self.command_kp_var.set(self.default_command_kp)
        self.command_kd_var.set(self.default_command_kd)
        for index, var in enumerate(self.feedforward_vars):
            var.set(self.default_feedforward_torque_enabled and self.default_joint_feedforward_enabled[index] >= 0.5)
        self._update_slider_labels()
        self._set_params({
            'desired_positions': list(self.default_desired_positions),
            'desired_velocities': list(self.default_desired_velocities),
            'desired_accelerations': list(self.default_desired_accelerations),
            'id_kp': list(self.default_id_kp),
            'id_kd': list(self.default_id_kd),
            'max_torque': [self.default_max_torque] * len(self.active_joint_names),
            'command_kp': self.default_command_kp,
            'command_kd': self.default_command_kd,
            'feedforward_torque_enabled': self.default_feedforward_torque_enabled,
            'joint_feedforward_enabled': list(self.default_joint_feedforward_enabled),
        })
        self.last_action_var.set('Safe defaults restored, including inverse-dynamics gains.')

    def _update_slider_labels(self):
        for value in self.desired_vars:
            value.label.configure(text=f'{value.get():.3f} rad')
        for index, value in enumerate(self.id_kp_vars):
            self.id_kp_labels[index].configure(text=f'{value.get():.3f}')
        for index, value in enumerate(self.id_kd_vars):
            self.id_kd_labels[index].configure(text=f'{value.get():.3f}')
        for index, value in enumerate(self.torque_vars):
            self.torque_labels[index].configure(text=f'{value.get():.3f} Nm')
        self.command_kp_label.configure(text=f'{self.command_kp_var.get():.3f}')
        self.command_kd_label.configure(text=f'{self.command_kd_var.get():.3f}')

    def _refresh_status_text(self):
        ready = self.set_parameters_client.service_is_ready()
        self.status_var.set(f'Controller: {"ready" if ready else "not ready"} ({self.controller_node})')
        if ready and not self.controller_params_loaded:
            self._refresh_controller_params()

        if self.latest_state is None:
            self.state_var.set(f'State: waiting on /{self.state_topic.lstrip("/")}')
        else:
            names = ', '.join(self.latest_state.name[:3]) if self.latest_state.name else 'no names'
            self.state_var.set(f'State: {self.state_count} msgs, names: {names}')

        if self.latest_command is None:
            self.command_var.set(f'Command: waiting on /{self.command_topic.lstrip("/")}')
        else:
            max_torque = max((abs(v) for v in self.latest_command.joint_torque), default=0.0)
            self.command_var.set(
                f'Command: {self.command_count} msgs, max t_ff {max_torque:.3f} Nm, '
                f'kp {self.latest_command.kp:.3f}, kd {self.latest_command.kd:.3f}'
            )

        self._refresh_feedback_table()

        self.root.after(250, self._refresh_status_text)

    def _refresh_feedback_table(self):
        state_by_name = {}
        if self.latest_state is not None:
            for index, name in enumerate(self.latest_state.name):
                state_by_name[name] = {
                    'position': self._value_at(self.latest_state.position, index),
                    'velocity': self._value_at(self.latest_state.velocity, index),
                    'effort': self._value_at(self.latest_state.effort, index),
                }

        command_by_name = {}
        if self.latest_command is not None:
            for index, name in enumerate(self.latest_command.joint_names):
                command_by_name[name] = self._value_at(self.latest_command.joint_torque, index)

        for joint_name in self.active_joint_names:
            row = self.feedback_rows[joint_name]
            state = state_by_name.get(joint_name)
            if state is None:
                row['position'].set('--')
                row['velocity'].set('--')
                row['effort'].set('--')
            else:
                row['position'].set(self._format_value(state['position'], 'rad'))
                row['velocity'].set(self._format_value(state['velocity'], 'rad/s'))
                row['effort'].set(self._format_value(state['effort'], 'Nm'))

            row['command'].set(self._format_value(command_by_name.get(joint_name), 'Nm'))

    def _value_at(self, values, index):
        if index < len(values):
            return float(values[index])
        return None

    def _format_value(self, value, unit):
        if value is None or not math.isfinite(value):
            return '--'
        return f'{value:.4f} {unit}'

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(20, self._spin_ros)

    def _on_close(self):
        self._motor_stop_zero_all()
        self.root.after(300, self.root.destroy)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = OneLegControlGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
