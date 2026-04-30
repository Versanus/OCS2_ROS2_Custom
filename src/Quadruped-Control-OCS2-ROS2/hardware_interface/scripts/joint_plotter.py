#!/usr/bin/env python3
import math
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from legged_msgs.msg import SimulatorStateData


WINDOW_SECONDS = 10.0
RMS_WINDOW_SECONDS = 0.5
REFRESH_MS = 10
MAX_DRAW_POINTS = 200

JOINT_NAMES = [
    'FL_HipX', 'FL_HipY', 'FL_Knee',
    'FR_HipX', 'FR_HipY', 'FR_Knee',
    'HL_HipX', 'HL_HipY', 'HL_Knee',
    'HR_HipX', 'HR_HipY', 'HR_Knee',
]

SIM_SOURCE_JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',
    'LH_HAA', 'LH_HFE', 'LH_KFE',
    'RF_HAA', 'RF_HFE', 'RF_KFE',
    'RH_HAA', 'RH_HFE', 'RH_KFE',
]

SIM_TO_DISPLAY_NAME = {
    'LF_HAA': 'FL_HipX', 'LF_HFE': 'FL_HipY', 'LF_KFE': 'FL_Knee',
    'RF_HAA': 'FR_HipX', 'RF_HFE': 'FR_HipY', 'RF_KFE': 'FR_Knee',
    'LH_HAA': 'HL_HipX', 'LH_HFE': 'HL_HipY', 'LH_KFE': 'HL_Knee',
    'RH_HAA': 'HR_HipX', 'RH_HFE': 'HR_HipY', 'RH_KFE': 'HR_Knee',
}

SOURCE_LABELS = {
    'sim': 'MuJoCo',
    'hardware': 'Hardware',
}

SOURCE_COLORS = {
    'sim': '#1f77b4',
    'hardware': '#d62728',
}

POSITION_LIMIT_SCALE = 1.5
POSITION_LIMITS_DEG = {
    'HipX': (-25.0, 25.0),
    'HipY': (-45.0, 45.0),
    'LeftKnee': (55.0, 95.0),
    'RightKnee': (-95.0, -55.0),
}


def canonical_joint_name(name: str) -> str:
    return ''.join(char for char in name.upper() if char.isalnum())


def scale_limits(limits, scale):
    low, high = limits
    center = (low + high) / 2.0
    half_span = ((high - low) / 2.0) * scale
    return (center - half_span, center + half_span)


CANONICAL_NAME_MAP = {
    canonical_joint_name('FL_HipX'): 'FL_HipX',
    canonical_joint_name('FL_HipY'): 'FL_HipY',
    canonical_joint_name('FL_Knee'): 'FL_Knee',
    canonical_joint_name('FR_HipX'): 'FR_HipX',
    canonical_joint_name('FR_HipY'): 'FR_HipY',
    canonical_joint_name('FR_Knee'): 'FR_Knee',
    canonical_joint_name('HL_HipX'): 'HL_HipX',
    canonical_joint_name('HL_HipY'): 'HL_HipY',
    canonical_joint_name('HL_Knee'): 'HL_Knee',
    canonical_joint_name('HR_HipX'): 'HR_HipX',
    canonical_joint_name('HR_HipY'): 'HR_HipY',
    canonical_joint_name('HR_Knee'): 'HR_Knee',
    canonical_joint_name('LF_HAA'): 'FL_HipX',
    canonical_joint_name('LF_HFE'): 'FL_HipY',
    canonical_joint_name('LF_KFE'): 'FL_Knee',
    canonical_joint_name('RF_HAA'): 'FR_HipX',
    canonical_joint_name('RF_HFE'): 'FR_HipY',
    canonical_joint_name('RF_KFE'): 'FR_Knee',
    canonical_joint_name('LH_HAA'): 'HL_HipX',
    canonical_joint_name('LH_HFE'): 'HL_HipY',
    canonical_joint_name('LH_KFE'): 'HL_Knee',
    canonical_joint_name('RH_HAA'): 'HR_HipX',
    canonical_joint_name('RH_HFE'): 'HR_HipY',
    canonical_joint_name('RH_KFE'): 'HR_Knee',
    canonical_joint_name('LF_HipX'): 'FL_HipX',
    canonical_joint_name('LF_HipY'): 'FL_HipY',
    canonical_joint_name('LF_Knee'): 'FL_Knee',
    canonical_joint_name('RF_HipX'): 'FR_HipX',
    canonical_joint_name('RF_HipY'): 'FR_HipY',
    canonical_joint_name('RF_Knee'): 'FR_Knee',
    canonical_joint_name('LH_HipX'): 'HL_HipX',
    canonical_joint_name('LH_HipY'): 'HL_HipY',
    canonical_joint_name('LH_Knee'): 'HL_Knee',
    canonical_joint_name('RH_HipX'): 'HR_HipX',
    canonical_joint_name('RH_HipY'): 'HR_HipY',
    canonical_joint_name('RH_Knee'): 'HR_Knee',
}


class JointPlotterNode(Node):
    def __init__(self):
        super().__init__('joint_plotter')

        self._lock = threading.Lock()
        self._warned_short_sim_positions = False
        self._warned_short_sim_torques = False
        self._plot_data = self._build_empty_plot_data()
        self._last_seen = {'sim': None, 'hardware': None}

        self.simulator_subscription = self.create_subscription(
            SimulatorStateData,
            'simulator_state_data',
            self._simulator_callback,
            10,
        )
        self.hardware_subscription = self.create_subscription(
            JointState,
            'htdw_joint_state',
            self._hardware_callback,
            10,
        )

    def _build_empty_plot_data(self):
        return {
            'position': {
                'sim': {joint: deque() for joint in JOINT_NAMES},
                'hardware': {joint: deque() for joint in JOINT_NAMES},
            },
            'torque': {
                'sim': {joint: deque() for joint in JOINT_NAMES},
                'hardware': {joint: deque() for joint in JOINT_NAMES},
            },
        }

    def _simulator_callback(self, msg: SimulatorStateData):
        now = time.monotonic()

        with self._lock:
            self._last_seen['sim'] = now

            if len(msg.joint_position_values) < len(SIM_SOURCE_JOINT_NAMES) and not self._warned_short_sim_positions:
                self.get_logger().warn(
                    f'Expected {len(SIM_SOURCE_JOINT_NAMES)} simulator joint positions, '
                    f'got {len(msg.joint_position_values)}.'
                )
                self._warned_short_sim_positions = True

            if len(msg.joint_torque_values) < len(SIM_SOURCE_JOINT_NAMES) and not self._warned_short_sim_torques:
                self.get_logger().warn(
                    f'Expected {len(SIM_SOURCE_JOINT_NAMES)} simulator joint torques, '
                    f'got {len(msg.joint_torque_values)}.'
                )
                self._warned_short_sim_torques = True

            self._append_sim_values('position', msg.joint_position_values, now)
            self._append_sim_values('torque', msg.joint_torque_values, now)

    def _hardware_callback(self, msg: JointState):
        now = time.monotonic()

        with self._lock:
            self._last_seen['hardware'] = now

            for index, raw_name in enumerate(msg.name):
                joint_name = CANONICAL_NAME_MAP.get(canonical_joint_name(raw_name))
                if joint_name is None:
                    continue

                if index < len(msg.position):
                    self._append_value('position', 'hardware', joint_name, now, float(msg.position[index]))

                if index < len(msg.effort):
                    self._append_value('torque', 'hardware', joint_name, now, float(msg.effort[index]))

    def _append_sim_values(self, signal_name, values, now):
        for index, source_name in enumerate(SIM_SOURCE_JOINT_NAMES):
            if index >= len(values):
                break
            joint_name = SIM_TO_DISPLAY_NAME[source_name]
            self._append_value(signal_name, 'sim', joint_name, now, float(values[index]))

    def _append_value(self, signal_name, source_name, joint_name, timestamp, value):
        series = self._plot_data[signal_name][source_name][joint_name]
        series.append((timestamp, value))
        self._trim_series(series, timestamp)

    def _trim_series(self, series, now):
        while series and now - series[0][0] > WINDOW_SECONDS:
            series.popleft()

    def get_snapshot(self):
        now = time.monotonic()
        snapshot = {
            'position': {'sim': {}, 'hardware': {}},
            'torque': {'sim': {}, 'hardware': {}},
        }

        with self._lock:
            for signal_name in ('position', 'torque'):
                for source_name in ('sim', 'hardware'):
                    for joint_name in JOINT_NAMES:
                        series = self._plot_data[signal_name][source_name][joint_name]
                        self._trim_series(series, now)
                        snapshot[signal_name][source_name][joint_name] = list(series)
            last_seen = dict(self._last_seen)

        return now, snapshot, last_seen


class PlotCanvas(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, background='white', highlightthickness=0)

    def draw_plot(
        self,
        title,
        anchor_time,
        sim_points,
        hardware_points,
        show_sim,
        show_hardware,
        y_min,
        y_max,
        sim_label='S',
        hardware_label='H',
    ):
        self.delete('all')

        width = max(self.winfo_width(), 160)
        height = max(self.winfo_height(), 120)

        left = 48
        right = 10
        top = 24
        bottom = 22
        plot_width = max(width - left - right, 10)
        plot_height = max(height - top - bottom, 10)
        x_min = anchor_time - WINDOW_SECONDS
        x_max = anchor_time

        self.create_rectangle(0, 0, width, height, fill='white', outline='')
        self.create_rectangle(left, top, left + plot_width, top + plot_height, outline='#cfcfcf')
        self.create_text(left, 10, text=title, anchor='w', fill='#202020', font=('TkDefaultFont', 9, 'bold'))
        self.create_text(
            left,
            height - 8,
            text='10 s',
            anchor='w',
            fill='#666666',
            font=('TkDefaultFont', 8),
        )
        self.create_text(
            left + plot_width,
            height - 8,
            text='now',
            anchor='e',
            fill='#666666',
            font=('TkDefaultFont', 8),
        )

        visible_series = []
        if show_sim and sim_points:
            visible_series.append(('sim', sim_points))
        if show_hardware and hardware_points:
            visible_series.append(('hardware', hardware_points))

        if y_min <= 0.0 <= y_max:
            zero_y = top + ((y_max - 0.0) / (y_max - y_min)) * plot_height
            self.create_line(left, zero_y, left + plot_width, zero_y, fill='#e6e6e6')

        self.create_text(6, top, text=f'{y_max:.1f}', anchor='w', fill='#666666', font=('TkDefaultFont', 8))
        self.create_text(
            6,
            top + plot_height,
            text=f'{y_min:.1f}',
            anchor='sw',
            fill='#666666',
            font=('TkDefaultFont', 8),
        )

        if visible_series:
            for source_name, series in visible_series:
                coords = self._series_to_coords(
                    series,
                    x_min,
                    x_max,
                    y_min,
                    y_max,
                    left,
                    top,
                    plot_width,
                    plot_height,
                )

                if len(coords) >= 4:
                    self.create_line(*coords, fill=SOURCE_COLORS[source_name], width=2, smooth=False)
                elif len(coords) == 2:
                    x, y = coords
                    self.create_oval(x - 2, y - 2, x + 2, y + 2, fill=SOURCE_COLORS[source_name], outline='')

            self._draw_latest_values(
                width,
                sim_points[-1][1] if sim_points else None,
                hardware_points[-1][1] if hardware_points else None,
                show_sim,
                show_hardware,
                sim_label,
                hardware_label,
            )
        else:
            self.create_text(
                left + (plot_width / 2),
                top + (plot_height / 2),
                text='No data',
                fill='#777777',
                font=('TkDefaultFont', 9),
            )

    def _series_to_coords(self, series, x_min, x_max, y_min, y_max, left, top, plot_width, plot_height):
        if not series:
            return []

        coords = []
        for timestamp, value in series:
            x_ratio = 0.0 if x_max <= x_min else (timestamp - x_min) / (x_max - x_min)
            x = left + min(max(x_ratio, 0.0), 1.0) * plot_width
            y_ratio = 0.5 if math.isclose(y_max, y_min) else (y_max - value) / (y_max - y_min)
            y = top + min(max(y_ratio, 0.0), 1.0) * plot_height
            coords.extend((x, y))

        return coords

    def _draw_latest_values(
        self,
        width,
        sim_value,
        hardware_value,
        show_sim,
        show_hardware,
        sim_label,
        hardware_label,
    ):
        legend_x = width - 10
        text_bits = []
        if show_sim and sim_value is not None:
            text_bits.append(f'{sim_label} {sim_value:.2f}')
        if show_hardware and hardware_value is not None:
            text_bits.append(f'{hardware_label} {hardware_value:.2f}')

        if text_bits:
            self.create_text(
                legend_x,
                10,
                text=' | '.join(text_bits),
                anchor='e',
                fill='#444444',
                font=('TkDefaultFont', 8),
            )


class SignalWindow:
    def __init__(self, window, signal_name, title):
        self.window = window
        self.signal_name = signal_name
        self.title = title
        self.single_joint_mode = False
        self.show_sim = True
        self.show_hardware = True
        self.show_rms = False
        self.selected_joint = tk.StringVar(value=JOINT_NAMES[0])
        self.status_var = tk.StringVar(value='Waiting for data')
        self.mode_button = None
        self.rms_button = None
        self.sim_button = None
        self.hardware_button = None
        self.plot_container = None
        self.plot_widgets = {}

        self._build_ui()
        self._rebuild_plots()

    def _build_ui(self):
        self.window.title(self.title)
        self.window.geometry('1280x900')

        controls = ttk.Frame(self.window, padding=8)
        controls.pack(fill='x')

        ttk.Label(controls, text='Joint').pack(side='left')

        joint_selector = ttk.Combobox(
            controls,
            textvariable=self.selected_joint,
            values=JOINT_NAMES,
            state='readonly',
            width=12,
        )
        joint_selector.pack(side='left', padx=(6, 12))
        joint_selector.bind('<<ComboboxSelected>>', self._on_joint_selected)

        self.mode_button = tk.Button(controls, command=self._toggle_mode, width=16)
        self.mode_button.pack(side='left', padx=4)

        self.rms_button = tk.Button(controls, command=self._toggle_rms, width=14)
        self.rms_button.pack(side='left', padx=4)

        self.sim_button = tk.Button(controls, command=self._toggle_sim, width=14)
        self.sim_button.pack(side='left', padx=4)

        self.hardware_button = tk.Button(controls, command=self._toggle_hardware, width=16)
        self.hardware_button.pack(side='left', padx=4)

        ttk.Label(controls, textvariable=self.status_var).pack(side='right')

        self.plot_container = ttk.Frame(self.window, padding=(8, 0, 8, 8))
        self.plot_container.pack(fill='both', expand=True)

        self._sync_buttons()

    def _on_joint_selected(self, _event):
        if self.single_joint_mode:
            self._rebuild_plots()

    def _toggle_mode(self):
        self.single_joint_mode = not self.single_joint_mode
        self._sync_buttons()
        self._rebuild_plots()

    def _toggle_rms(self):
        self.show_rms = not self.show_rms
        self._sync_buttons()

    def _toggle_sim(self):
        self.show_sim = not self.show_sim
        self._sync_buttons()

    def _toggle_hardware(self):
        self.show_hardware = not self.show_hardware
        self._sync_buttons()

    def _sync_buttons(self):
        mode_text = 'Show 12 Graphs' if self.single_joint_mode else 'Show Single Joint'
        self.mode_button.config(text=mode_text)
        self._style_toggle_button(self.rms_button, self.show_rms, 'RMS')
        self._style_toggle_button(self.sim_button, self.show_sim, 'MuJoCo')
        self._style_toggle_button(self.hardware_button, self.show_hardware, 'Hardware')

    def _style_toggle_button(self, button, enabled, label):
        if enabled:
            button.config(text=f'{label}: ON', bg='#cfe9cf', activebackground='#b7dfb7')
        else:
            button.config(text=f'{label}: OFF', bg='#f0d0d0', activebackground='#e7baba')

    def _rebuild_plots(self):
        for child in self.plot_container.winfo_children():
            child.destroy()

        self.plot_widgets = {}

        if self.single_joint_mode:
            canvas = PlotCanvas(self.plot_container)
            canvas.pack(fill='both', expand=True)
            self.plot_widgets[self.selected_joint.get()] = canvas
            return

        for row in range(4):
            self.plot_container.grid_rowconfigure(row, weight=1)
        for col in range(3):
            self.plot_container.grid_columnconfigure(col, weight=1)

        for index, joint_name in enumerate(JOINT_NAMES):
            row = index // 3
            col = index % 3
            canvas = PlotCanvas(self.plot_container)
            canvas.grid(row=row, column=col, sticky='nsew', padx=4, pady=4)
            self.plot_widgets[joint_name] = canvas

    def refresh(self, now, signal_data, last_seen):
        self.status_var.set(self._build_status_text(now, last_seen))
        anchor_time = self._resolve_anchor_time(now, signal_data)

        for joint_name, canvas in self.plot_widgets.items():
            sim_points, hardware_points, sim_label, hardware_label = self._prepare_display_series(
                signal_data['sim'].get(joint_name, []),
                signal_data['hardware'].get(joint_name, []),
            )
            y_min, y_max = self._axis_limits(joint_name)
            canvas.draw_plot(
                self._plot_title(joint_name),
                anchor_time,
                sim_points,
                hardware_points,
                self.show_sim,
                self.show_hardware,
                y_min,
                y_max,
                sim_label,
                hardware_label,
            )

    def _build_status_text(self, now, last_seen):
        parts = []
        for source_name in ('sim', 'hardware'):
            last_time = last_seen.get(source_name)
            if last_time is None or now - last_time > 1.5:
                parts.append(f'{SOURCE_LABELS[source_name]}: waiting')
            else:
                parts.append(f'{SOURCE_LABELS[source_name]}: live')
        return ' | '.join(parts)

    def _resolve_anchor_time(self, fallback_time, signal_data):
        latest_timestamp = None

        for source_name in ('sim', 'hardware'):
            for joint_name in JOINT_NAMES:
                points = signal_data[source_name].get(joint_name, [])
                if not points:
                    continue
                point_time = points[-1][0]
                if latest_timestamp is None or point_time > latest_timestamp:
                    latest_timestamp = point_time

        if latest_timestamp is None:
            return fallback_time

        return latest_timestamp

    def _prepare_display_series(self, sim_points, hardware_points):
        sim_points = self._normalize_points(sim_points)
        hardware_points = self._normalize_points(hardware_points)

        if not self.show_rms:
            return sim_points, hardware_points, 'S', 'H'

        comparison_points = self._compute_comparison_rms_series(sim_points, hardware_points)
        if self.show_sim and not self.show_hardware:
            return comparison_points, [], 'RMS', 'H'
        if self.show_hardware and not self.show_sim:
            return [], comparison_points, 'S', 'RMS'
        return comparison_points, [], 'RMS', 'H'

    def _normalize_points(self, points):
        if not points:
            return []

        if self.signal_name == 'position':
            return [(timestamp, math.degrees(value)) for timestamp, value in points]

        return points

    def _compute_comparison_rms_series(self, sim_points, hardware_points):
        if not sim_points or not hardware_points:
            return []

        events = [(timestamp, 'sim', value) for timestamp, value in sim_points]
        events.extend((timestamp, 'hardware', value) for timestamp, value in hardware_points)
        events.sort(key=lambda item: item[0])

        comparison_points = []
        last_sim = None
        last_hardware = None

        for timestamp, source_name, value in events:
            if source_name == 'sim':
                last_sim = value
            else:
                last_hardware = value

            if last_sim is None or last_hardware is None:
                continue

            comparison_points.append((timestamp, last_sim - last_hardware))

        return self._compute_rms_series(comparison_points)

    def _compute_rms_series(self, points):
        rms_points = []
        window = deque()
        sum_squares = 0.0

        for timestamp, value in points:
            square = value * value
            window.append((timestamp, square))
            sum_squares += square

            while window and timestamp - window[0][0] > RMS_WINDOW_SECONDS:
                _, old_square = window.popleft()
                sum_squares -= old_square

            rms_value = math.sqrt(sum_squares / len(window)) if window else 0.0
            rms_points.append((timestamp, rms_value))

        return rms_points

    def _axis_limits(self, joint_name):
        if self.signal_name == 'position':
            y_min, y_max = self._position_limit_for_joint(joint_name)
            if self.show_rms:
                return (0.0, max(abs(y_min), abs(y_max)))
            return (y_min, y_max)

        torque_limit = self._torque_limit_for_joint(joint_name)
        return (0.0, torque_limit) if self.show_rms else (-torque_limit, torque_limit)

    def _position_limit_for_joint(self, joint_name):
        if joint_name.endswith('HipX'):
            return scale_limits(POSITION_LIMITS_DEG['HipX'], POSITION_LIMIT_SCALE)
        if joint_name.endswith('HipY'):
            return scale_limits(POSITION_LIMITS_DEG['HipY'], POSITION_LIMIT_SCALE)
        if joint_name.startswith(('FL_', 'HL_')):
            return scale_limits(POSITION_LIMITS_DEG['LeftKnee'], POSITION_LIMIT_SCALE)
        return scale_limits(POSITION_LIMITS_DEG['RightKnee'], POSITION_LIMIT_SCALE)

    def _torque_limit_for_joint(self, joint_name):
        if joint_name.endswith('HipX'):
            return 6.0
        return 12.0

    def _plot_title(self, joint_name):
        mode_label = 'RMS Error' if self.show_rms else 'Raw'
        unit = 'deg' if self.signal_name == 'position' else 'Nm'
        return f'{joint_name} [{unit}] {mode_label}'


class JointPlotterApp:
    def __init__(self, node: JointPlotterNode):
        self.node = node
        self.closed = False
        self.root = tk.Tk()
        self.position_window = tk.Toplevel(self.root)

        self.torque_view = SignalWindow(self.root, 'torque', 'Joint Torque Plotter')
        self.position_view = SignalWindow(self.position_window, 'position', 'Joint Position Plotter')

        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)
        self.position_window.protocol('WM_DELETE_WINDOW', self.shutdown)

    def run(self):
        self._schedule_refresh()
        self.root.mainloop()

    def _schedule_refresh(self):
        if self.closed:
            return

        now, snapshot, last_seen = self.node.get_snapshot()
        self.torque_view.refresh(now, snapshot['torque'], last_seen)
        self.position_view.refresh(now, snapshot['position'], last_seen)
        self.root.after(REFRESH_MS, self._schedule_refresh)

    def shutdown(self):
        if self.closed:
            return

        self.closed = True
        self.position_window.destroy()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = JointPlotterNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = JointPlotterApp(node)

    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
