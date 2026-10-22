#!/usr/bin/env python3
import csv
import json
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import rclpy
from geometry_msgs.msg import WrenchStamped
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QProgressBar, QPushButton, QScrollArea, QTabWidget, QVBoxLayout, QWidget)

RESULTS = Path.home() / 'Desktop' / 'tms_hummingbird' / 'results'
AXES = ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')
MAX_THRUST = 1961.04 * 9.80665e-3
SERVO_SETTLING_SEC = 0.5
SERVO_TARGET_TOL_DEG = 0.5
SERVO_MOVE_TIMEOUT_SEC = 12.0
SERVO_COMMAND_LPF_SEC = 2.0
SERVO_MAX_RATE_DEG_S = 60.0
RAW_COLUMNS = ['timestamp', 'sensor_timestamp_us', 'phase', 'condition_index', 'alpha_deg',
    'beta_deg', 'motor_target_u', 'motor_output_u', 'alpha_target_deg', 'beta_target_deg'] + [
    f'servo_{i}_target_deg' for i in range(6)] + [
    f'servo_{i}_commanded_deg' for i in range(6)] + [
    f'raw_{x}' for x in AXES] + [f'corrected_{x}' for x in AXES] + [f'rotated_{x}' for x in AXES]
SUMMARY_COLUMNS = ['condition_index', 'alpha_deg', 'beta_deg', 'motor_u', 'motor_percent',
    'estimated_rotor_thrust_N', 'estimated_total_thrust_N', 'sample_count', 'valid'] + [
    f'{x}_{stat}' for x in AXES for stat in ('mean', 'variance', 'std')]
DIST_COLUMNS = ['condition_index', 'alpha_deg', 'beta_deg', 'motor_u', 'sample_count', 'valid'] + [
    f'delta_{x}_mean' for x in AXES]


def values(low, high, step):
    if step <= 0 or high < low:
        raise ValueError('Min / Max / Step 설정을 확인하세요.')
    count = int(math.floor((high - low) / step + 1e-8))
    return [round(low + i * step, 6) for i in range(count + 1)]


class RosBridge(Node):
    def __init__(self, events):
        super().__init__('tms_gui')
        self.events = events
        self.motor_pub = self.create_publisher(Float32, '/tms/motor_target', 10)
        self.limit_pub = self.create_publisher(Float32, '/tms/u_max', 10)
        self.servo_pub = self.create_publisher(Float32MultiArray, '/tms/servo_target_deg', 10)
        self.mode_pub = self.create_publisher(Bool, '/tms/offboard_request', 10)
        self.arm_pub = self.create_publisher(Bool, '/tms/arm_request', 10)
        self.stop_pub = self.create_publisher(Bool, '/tms/emergency_stop', 10)
        self.create_subscription(WrenchStamped, '/RFT_FORCE1', self.on_sensor, qos_profile_sensor_data)
        self.create_subscription(Bool, '/tms/servo_ready',
            lambda m: self.events.put(('servo', time.monotonic(), m.data)), 10)
        self.create_subscription(Float32MultiArray, '/tms/servo_commanded_deg',
            lambda m: self.events.put(('servo_commanded', time.monotonic(), list(m.data))), qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
            lambda m: self.events.put(('px4', time.monotonic(), (m.nav_state, m.arming_state, m.failsafe))), qos_profile_sensor_data)
        self.create_subscription(String, '/tms/bldc_state',
            lambda m: self.events.put(('bldc', time.monotonic(), m.data)), 10)
        self.create_subscription(Float32, '/tms/motor_output',
            lambda m: self.events.put(('output', time.monotonic(), float(m.data))), 10)

    def on_sensor(self, msg):
        w = msg.wrench
        raw = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)
        stamp = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
        self.events.put(('sensor', time.monotonic(), (stamp, raw)))

    def motor(self, u):
        self.motor_pub.publish(Float32(data=float(u)))

    def motor_limit(self, u_max):
        self.limit_pub.publish(Float32(data=float(u_max)))

    def servo(self, targets):
        self.servo_pub.publish(Float32MultiArray(data=[float(v) for v in targets]))

    def request(self, pub, enabled=True):
        pub.publish(Bool(data=enabled))


class Gui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('TMS HummingBird — Thrust & Wall Effect')
        self.resize(1500, 980)
        self.events = queue.Queue()
        self.ros = RosBridge(self.events)
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros,), daemon=True)
        self.ros_thread.start()
        self.last = {}
        self.px4 = None
        self.bldc = ''
        self.servo_ready = False
        self.servo_target = [0.0] * 6
        self.servo_commanded = [0.0] * 6
        self.servo_stable_start = None
        self.motor_output = 0.0
        self.manual_motor = 0.0
        self.manual_servo_target = [0.0] * 6
        self.raw = np.zeros(6)
        self.corrected = np.zeros(6)
        self.rotated = np.zeros(6)
        self.zero = np.zeros(6)
        self.calibrated = False
        self.calibrating = False
        self.cal_samples = []
        self.phase = 'IDLE'
        self.conditions = []
        self.index = 0
        self.samples = []
        self.ramp_from = 0.0
        self.phase_start = 0.0
        self.run_dir = None
        self.raw_file = None
        self.summary_file = None
        self.dist_file = None
        self.baseline = {}
        self.baseline_path = None
        self.graph_time = deque(maxlen=600)
        self.graph_values = [deque(maxlen=600) for _ in range(4)]
        self.t0 = time.monotonic()
        self.build_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def spinbox(self, low, high, value, step, decimals=2):
        box = QDoubleSpinBox()
        box.setRange(low, high)
        box.setDecimals(decimals)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def range_group(self, title, low, high, step, bounds, enabled=False):
        group = QGroupBox(title)
        form = QFormLayout(group)
        toggle = QCheckBox('Enable') if enabled else None
        if toggle:
            form.addRow(toggle)
        widgets = [self.spinbox(bounds[0], bounds[1], value, step, 3) for value in (low, high, step)]
        for label, widget in zip(('Min', 'Max', 'Step'), widgets):
            form.addRow(label, widget)
        return group, toggle, widgets

    def build_ui(self):
        tabs = QTabWidget()
        self.setCentralWidget(tabs)
        scroll = QScrollArea()
        outer = QWidget()
        root = QVBoxLayout(outer)
        scroll.setWidget(outer)
        scroll.setWidgetResizable(True)
        tabs.addTab(scroll, 'Experiment / Live F/T')
        check_page = QWidget()
        check_root = QVBoxLayout(check_page)
        tabs.addTab(check_page, 'Manual CHK')
        top = QHBoxLayout()
        root.addLayout(top)
        conn = QGroupBox('Connections / mode')
        cf = QFormLayout(conn)
        self.px4_label = QLabel('PX4 DDS: disconnected')
        self.servo_label = QLabel('Dynamixel: disconnected')
        self.sensor_label = QLabel('F/T: disconnected')
        self.bldc_label = QLabel('BLDC: disconnected')
        for label in (self.px4_label, self.servo_label, self.sensor_label, self.bldc_label):
            cf.addRow(label)
        self.wall = QCheckBox('Wall')
        self.wall.toggled.connect(self.validate_baseline)
        self.distance = self.spinbox(0.001, 10.0, 0.2, 0.01, 3)
        self.distance.valueChanged.connect(self.validate_baseline)
        cf.addRow('Mode', self.wall)
        cf.addRow('d (m)', self.distance)
        self.baseline_button = QPushButton('Select d=inf summary CSV')
        self.baseline_button.clicked.connect(self.select_baseline)
        self.baseline_label = QLabel('먼저 d=inf 기준 실험을 수행하거나 기준 CSV를 선택하세요.')
        cf.addRow(self.baseline_button)
        cf.addRow(self.baseline_label)
        top.addWidget(conn)
        motor_group, _, self.motor_range = self.range_group('Motor u [normalized]', 0.0, 0.2, 0.05, (0.0, 1.0))
        self.u_max = self.spinbox(0.0, 1.0, 0.35, 0.01, 3)
        motor_group.layout().insertRow(0, 'u max (output limit)', self.u_max)
        self.u_max.valueChanged.connect(self.validate_baseline)
        for widget in self.motor_range:
            widget.valueChanged.connect(self.validate_baseline)
        self.motor_label = QLabel('command u: 0.0% | estimated: 0.00 N/rotor, 0.00 N total')
        motor_group.layout().addRow(self.motor_label)
        top.addWidget(motor_group)
        servo_box = QGroupBox('Servo target [deg]')
        sv = QVBoxLayout(servo_box)
        alpha_group, self.alpha_enable, self.alpha_range = self.range_group('Alpha ±30°', 0, 0, 5, (-30, 30), True)
        beta_group, self.beta_enable, self.beta_range = self.range_group('Beta ±180°', 0, 0, 10, (-180, 180), True)
        for widget in self.alpha_range + self.beta_range + [self.alpha_enable, self.beta_enable]:
            (widget.valueChanged if isinstance(widget, QDoubleSpinBox) else widget.toggled).connect(self.validate_baseline)
        sv.addWidget(alpha_group)
        sv.addWidget(beta_group)
        self.angle_label = QLabel('Alpha 0° | Beta 0°')
        sv.addWidget(self.angle_label)
        top.addWidget(servo_box)
        cal = QGroupBox('Sensor calibration')
        cl = QFormLayout(cal)
        self.yaw = self.spinbox(-180, 180, 45, 0.1, 2)
        self.yaw.valueChanged.connect(self.validate_baseline)
        self.cal_time = self.spinbox(0.5, 60, 5, 0.5, 1)
        self.zero_button = QPushButton('Zero Calibration')
        self.zero_button.clicked.connect(self.start_calibration)
        self.reset_button = QPushButton('Reset Zero')
        self.reset_button.clicked.connect(self.reset_zero)
        self.cal_label = QLabel('Not calibrated')
        self.force_label = QLabel('Raw: —\nCorrected+Yaw: —')
        cl.addRow('Sensor Yaw Rotation (deg)', self.yaw)
        cl.addRow('Zero time (s)', self.cal_time)
        cl.addRow(self.zero_button, self.reset_button)
        cl.addRow(self.cal_label)
        cl.addRow(self.force_label)
        top.addWidget(cal)
        control = QGroupBox('Experiment')
        layout = QGridLayout(control)
        self.ramp = self.spinbox(0.1, 60, 2, 0.5, 1)
        self.settle = self.spinbox(0.1, 60, 2, 0.5, 1)
        self.measure = self.spinbox(0.1, 600, 5, 1, 1)
        for i, (name, widget) in enumerate((('Ramp (s)', self.ramp), ('Settling (s)', self.settle), ('Measurement (s)', self.measure))):
            layout.addWidget(QLabel(name), 0, i * 2)
            layout.addWidget(widget, 0, i * 2 + 1)
        self.offboard_button = QPushButton('Enter Offboard')
        self.arm_button = QPushButton('Arm')
        self.start_button = QPushButton('START')
        self.stop_button = QPushButton('STOP')
        self.emergency_button = QPushButton('Emergency Stop')
        self.offboard_button.clicked.connect(self.request_offboard)
        self.arm_button.clicked.connect(self.request_arm)
        self.start_button.clicked.connect(self.start_run)
        self.stop_button.clicked.connect(lambda: self.stop_run('stopped'))
        self.emergency_button.clicked.connect(lambda: self.stop_run('emergency'))
        for i, button in enumerate((self.offboard_button, self.arm_button, self.start_button, self.stop_button, self.emergency_button)):
            layout.addWidget(button, 1, i)
        self.phase_label = QLabel('IDLE')
        self.progress = QProgressBar()
        layout.addWidget(self.phase_label, 2, 0, 1, 4)
        layout.addWidget(self.progress, 2, 4, 1, 2)
        root.addWidget(control)
        graph_grid = QGridLayout()
        root.addLayout(graph_grid)
        self.plots = []
        self.curves = []
        for i, title in enumerate(('Measured Force [N]', 'Measured Torque [Nm]', 'Wall-induced Force [N]', 'Wall-induced Torque [Nm]')):
            plot = pg.PlotWidget(title=title)
            plot.addLegend()
            plot.setMinimumHeight(250)
            plot.showGrid(x=True, y=True)
            labels = AXES[0:3] if i % 2 == 0 else AXES[3:6]
            colors = ('r', 'g', 'b')
            curves = [plot.plot(pen=pg.mkPen(color, width=2), name=label) for color, label in zip(colors, labels)]
            self.curves.append(curves)
            self.plots.append(plot)
            graph_grid.addWidget(plot, i // 2, i % 2)
        check_status = QGroupBox('Connection check')
        check_status_layout = QFormLayout(check_status)
        self.check_label = QLabel('Waiting for ROS topics')
        check_status_layout.addRow(self.check_label)
        check_root.addWidget(check_status)
        servo_check = QGroupBox('Dynamixel CHK — target angles [deg]')
        servo_check_layout = QFormLayout(servo_check)
        self.servo_chk = QCheckBox('Dynamixel CHK enable')
        self.servo_chk.toggled.connect(self.servo_check_changed)
        self.check_alpha = self.spinbox(-30, 30, 0, 1, 2)
        self.check_beta = self.spinbox(-180, 180, 0, 1, 2)
        self.check_servo_apply = QPushButton('Apply Alpha / Beta')
        self.check_servo_zero = QPushButton('Servo 0°')
        self.check_servo_apply.clicked.connect(self.apply_manual_servo)
        self.check_servo_zero.clicked.connect(self.zero_manual_servo)
        servo_check_layout.addRow(self.servo_chk)
        servo_check_layout.addRow('Alpha', self.check_alpha)
        servo_check_layout.addRow('Beta', self.check_beta)
        servo_check_layout.addRow(self.check_servo_apply, self.check_servo_zero)
        check_root.addWidget(servo_check)
        offboard_check = QGroupBox('PX4 Offboard / BLDC CHK')
        offboard_check_layout = QFormLayout(offboard_check)
        self.offboard_chk = QCheckBox('Offboard CHK enable')
        self.offboard_chk.toggled.connect(self.offboard_check_changed)
        self.check_mode_button = QPushButton('Request Offboard')
        self.check_arm_button = QPushButton('Arm')
        self.check_disarm_button = QPushButton('Disarm / exit Offboard')
        self.check_u = self.spinbox(0, 1, 0, 0.01, 3)
        self.check_motor_label = QLabel('u max 0.350 | estimated 0.00 N/rotor, 0.00 N total')
        self.check_motor_apply = QPushButton('Apply motor u')
        self.check_motor_zero = QPushButton('Motor 0')
        self.check_emergency = QPushButton('Emergency Stop')
        self.check_mode_button.clicked.connect(self.check_request_offboard)
        self.check_arm_button.clicked.connect(self.check_request_arm)
        self.check_disarm_button.clicked.connect(self.check_disarm)
        self.check_motor_apply.clicked.connect(self.apply_manual_motor)
        self.check_motor_zero.clicked.connect(self.zero_manual_motor)
        self.check_emergency.clicked.connect(lambda: self.stop_run('emergency'))
        offboard_check_layout.addRow(self.offboard_chk)
        offboard_check_layout.addRow(self.check_mode_button, self.check_arm_button)
        offboard_check_layout.addRow('Normalized motor u', self.check_u)
        offboard_check_layout.addRow(self.check_motor_label)
        offboard_check_layout.addRow(self.check_motor_apply, self.check_motor_zero)
        offboard_check_layout.addRow(self.check_disarm_button, self.check_emergency)
        check_root.addWidget(offboard_check)
        check_root.addStretch()
        self.validate_baseline()

    def servo_check_changed(self, enabled):
        if not enabled:
            self.zero_manual_servo()
        self.validate_baseline()

    def offboard_check_changed(self, enabled):
        if not enabled:
            self.check_disarm()
        self.validate_baseline()

    def apply_manual_servo(self):
        if self.phase != 'IDLE' or not self.servo_chk.isChecked() or not (self.connected('servo', 0.2) and self.servo_ready):
            QMessageBox.warning(self, 'Dynamixel CHK', 'Dynamixel 연결과 CHK Enable을 확인하세요.')
            return
        self.manual_servo_target = self.target_angles(self.check_alpha.value(), self.check_beta.value())
        self.servo_target = self.manual_servo_target.copy()
        self.ros.servo(self.servo_target)

    def zero_manual_servo(self):
        self.manual_servo_target = [0.0] * 6
        self.servo_target = self.manual_servo_target.copy()
        self.ros.servo(self.servo_target)

    def check_request_offboard(self):
        if self.phase != 'IDLE' or not self.offboard_chk.isChecked():
            QMessageBox.warning(self, 'Offboard CHK', 'Offboard CHK Enable을 먼저 켜세요.')
            return
        self.request_offboard()

    def check_request_arm(self):
        if self.phase != 'IDLE' or not self.offboard_chk.isChecked():
            QMessageBox.warning(self, 'Offboard CHK', 'Offboard CHK Enable을 먼저 켜세요.')
            return
        self.request_arm()

    def apply_manual_motor(self):
        if (self.phase != 'IDLE' or not self.offboard_chk.isChecked() or not self.ready_for_control() or
                self.px4[0] != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                self.px4[1] != VehicleStatus.ARMING_STATE_ARMED or
                self.check_u.value() > self.u_max.value() + 1e-6):
            QMessageBox.warning(self, 'BLDC CHK', 'Offboard, Arm, 연결, Zero Calibration, u max를 확인하세요.')
            return
        self.manual_motor = self.check_u.value()
        self.ros.motor(self.manual_motor)

    def zero_manual_motor(self):
        self.manual_motor = 0.0
        self.ros.motor(0)

    def check_disarm(self):
        self.zero_manual_motor()
        self.ros.request(self.ros.arm_pub, False)
        self.ros.request(self.ros.mode_pub, False)

    def grid_conditions(self):
        motors = values(*[x.value() for x in self.motor_range])
        if any(u > self.u_max.value() + 1e-6 for u in motors):
            raise ValueError('Motor Max는 u max 이하로 설정하세요.')
        alphas = values(*[x.value() for x in self.alpha_range]) if self.alpha_enable.isChecked() else [0.0]
        betas = values(*[x.value() for x in self.beta_range]) if self.beta_enable.isChecked() else [0.0]
        if any(abs(a) > 30 for a in alphas) or any(abs(b) > 180 for b in betas):
            raise ValueError('Servo 기계적 제한 초과')
        if len(alphas) * len(betas) * len(motors) > 10000:
            raise ValueError('실험 조건이 10,000개를 초과합니다.')
        return [(a, b, u) for a in alphas for b in betas for u in motors]

    def validate_baseline(self):
        self.distance.setEnabled(self.wall.isChecked())
        try:
            conditions = self.grid_conditions()
            valid = True
            reason = 'd=inf: baseline 생성 모드'
            if self.wall.isChecked():
                valid = False
                reason = '먼저 d=inf 기준 실험을 수행하거나 기준 CSV를 선택하세요.'
                if self.baseline_path:
                    meta = json.loads((self.baseline_path.parent / 'metadata.json').read_text())
                    with self.baseline_path.open(newline='') as file:
                        rows = list(csv.DictReader(file))
                    actual = [(float(r['alpha_deg']), float(r['beta_deg']), float(r['motor_u'])) for r in rows if r['valid'] == 'True']
                    if (meta['mode'] != 'd_inf' or meta['state'] != 'complete' or
                            meta['completed_conditions'] != meta['total_conditions'] or actual != conditions):
                        reason = 'Baseline 조건 또는 유효 측정 행이 현재 설정과 다릅니다.'
                    elif abs(meta['sensor_yaw_deg'] - self.yaw.value()) > 1e-6 or meta['zero_calibrated'] != self.calibrated:
                        reason = 'Baseline Yaw 또는 Zero Calibration 상태가 다릅니다.'
                    elif (meta.get('u_max') != self.u_max.value() or
                          meta['motor_range'] != [x.value() for x in self.motor_range] or
                          meta['alpha_range'] != [x.value() for x in self.alpha_range] or
                          meta['beta_range'] != [x.value() for x in self.beta_range] or
                          meta['alpha_enabled'] != self.alpha_enable.isChecked() or
                          meta['beta_enabled'] != self.beta_enable.isChecked() or
                          meta['ramp_s'] != self.ramp.value() or
                          meta['settling_s'] != self.settle.value() or
                          meta['measurement_s'] != self.measure.value() or
                          meta.get('servo_command_lpf_sec') != SERVO_COMMAND_LPF_SEC or
                          meta.get('servo_max_rate_deg_s') != SERVO_MAX_RATE_DEG_S or
                          meta.get('servo_command_tolerance_deg') != SERVO_TARGET_TOL_DEG or
                          meta.get('servo_command_dwell_sec') != SERVO_SETTLING_SEC or
                          meta.get('servo_move_timeout_sec') != SERVO_MOVE_TIMEOUT_SEC):
                        reason = 'Baseline 모터/서보 범위 또는 측정 시간이 다릅니다.'
                    else:
                        self.baseline = {(a, b, u): np.array([float(r[f'{x}_mean']) for x in AXES])
                            for (a, b, u), r in zip(actual, rows)}
                        valid = True
                        reason = f'Baseline loaded: {self.baseline_path}'
            self.baseline_label.setText(reason)
            self.start_button.setEnabled(valid and self.phase == 'IDLE' and
                not self.servo_chk.isChecked() and not self.offboard_chk.isChecked())
        except (ValueError, KeyError, OSError, json.JSONDecodeError) as exc:
            self.baseline_label.setText(f'Baseline/config invalid: {exc}')
            self.start_button.setEnabled(False)

    def select_baseline(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Select d=inf summary CSV', str(RESULTS), 'CSV (*.csv)')
        if path:
            self.baseline_path = Path(path)
            self.baseline = {}
            self.validate_baseline()

    def connected(self, key, timeout):
        return time.monotonic() - self.last.get(key, -1e10) < timeout

    def ready_for_control(self):
        return (self.connected('px4', 1) and self.connected('servo', 0.2) and self.servo_ready and
            self.connected('sensor', 0.5) and self.connected('bldc', 1) and
            self.connected('output', 1) and self.bldc.startswith('READY') and
            self.px4 is not None and not self.px4[2] and self.calibrated)

    def request_offboard(self):
        if not self.ready_for_control():
            QMessageBox.warning(self, 'Not ready', 'PX4 / Dynamixel / F/T / BLDC 연결 및 Zero Calibration을 확인하세요.')
            return
        self.ros.request(self.ros.mode_pub)

    def request_arm(self):
        if not self.ready_for_control() or self.px4[0] != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            QMessageBox.warning(self, 'Not ready', 'Offboard 상태와 연결·보정을 먼저 확인하세요.')
            return
        self.ros.request(self.ros.arm_pub)

    def start_calibration(self):
        if self.phase != 'IDLE' or not self.connected('sensor', 0.5) or (self.px4 and self.px4[1] == VehicleStatus.ARMING_STATE_ARMED):
            return
        self.cal_samples = []
        self.cal_start = time.monotonic()
        self.calibrating = True
        self.cal_label.setText('Collecting zero samples...')

    def reset_zero(self):
        if self.phase != 'IDLE' or (self.px4 and self.px4[1] == VehicleStatus.ARMING_STATE_ARMED):
            return
        self.zero[:] = 0
        self.calibrated = False
        self.calibrating = False
        self.cal_label.setText('Not calibrated')
        self.validate_baseline()

    def target_angles(self, a, b):
        return [-b, b, a, a, -a, -a]

    def start_run(self):
        self.validate_baseline()
        if not self.start_button.isEnabled() or self.servo_chk.isChecked() or self.offboard_chk.isChecked() or not self.ready_for_control() or self.px4[0] != VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.px4[1] != VehicleStatus.ARMING_STATE_ARMED:
            QMessageBox.warning(self, 'Not ready', '기준 데이터, 연결, Zero Calibration, Offboard 및 수동 Arm 상태를 확인하세요.')
            return
        try:
            self.conditions = self.grid_conditions()
            distance_text = f'{self.distance.value():.3f}'.rstrip('0')
            distance_text += '0' * max(0, 2 - len(distance_text.split('.')[1]))
            tag = 'd_inf' if not self.wall.isChecked() else f'd_{distance_text}m'
            folder = RESULTS / tag
            folder.mkdir(parents=True, exist_ok=True)
            self.run_dir = folder / f'{datetime.now():%Y%m%d_%H%M%S_%f}_{tag}'
            self.run_dir.mkdir(exist_ok=False)
            self.raw_file = (self.run_dir / f'raw_{tag}.csv').open('w', newline='')
            self.summary_file = (self.run_dir / f'summary_{tag}.csv').open('w', newline='')
            self.dist_file = (self.run_dir / f'disturbance_{tag}.csv').open('w', newline='') if self.wall.isChecked() else None
            self.raw_writer = csv.DictWriter(self.raw_file, RAW_COLUMNS)
            self.summary_writer = csv.DictWriter(self.summary_file, SUMMARY_COLUMNS)
            self.raw_writer.writeheader()
            self.summary_writer.writeheader()
            if self.dist_file:
                self.dist_writer = csv.DictWriter(self.dist_file, DIST_COLUMNS)
                self.dist_writer.writeheader()
            self.index = 0
            self.samples = []
            self.progress.setValue(0)
            self.phase = 'MOTOR_DOWN'
            self.phase_start = time.monotonic()
            self.servo_stable_start = None
            self.ros.motor(0)
            self.save_metadata('running')
            self.set_running(True)
            self.start_button.setEnabled(False)
        except (ValueError, OSError) as exc:
            self.stop_run('error')
            QMessageBox.warning(self, 'Start failed', str(exc))

    def set_running(self, running):
        widgets = [self.wall, self.distance, self.baseline_button, self.yaw, self.cal_time,
            self.zero_button, self.reset_button, self.ramp, self.settle, self.measure, self.u_max,
            self.alpha_enable, self.beta_enable, self.servo_chk, self.offboard_chk,
            self.check_alpha, self.check_beta, self.check_u] + self.motor_range + self.alpha_range + self.beta_range
        for widget in widgets:
            widget.setEnabled(not running)
        if not running:
            self.distance.setEnabled(self.wall.isChecked())

    def save_metadata(self, state):
        if not self.run_dir:
            return
        data = dict(mode='wall' if self.wall.isChecked() else 'd_inf', distance_m=self.distance.value() if self.wall.isChecked() else None,
            sensor_yaw_deg=self.yaw.value(), zero_calibrated=self.calibrated, zero_offset=self.zero.tolist(),
            motor_range=[x.value() for x in self.motor_range], u_max=self.u_max.value(),
            alpha_enabled=self.alpha_enable.isChecked(),
            alpha_range=[x.value() for x in self.alpha_range], beta_enabled=self.beta_enable.isChecked(),
            beta_range=[x.value() for x in self.beta_range], ramp_s=self.ramp.value(), settling_s=self.settle.value(),
            measurement_s=self.measure.value(), servo_command_lpf_sec=SERVO_COMMAND_LPF_SEC,
            servo_max_rate_deg_s=SERVO_MAX_RATE_DEG_S,
            servo_command_tolerance_deg=SERVO_TARGET_TOL_DEG, servo_command_dwell_sec=SERVO_SETTLING_SEC,
            servo_move_timeout_sec=SERVO_MOVE_TIMEOUT_SEC,
            servo_mapping='servo_0=-beta, servo_1=+beta, servo_2/3=+alpha, servo_4/5=-alpha',
            baseline_csv=str(self.baseline_path) if self.wall.isChecked() else None,
            completed_conditions=self.index, total_conditions=len(self.conditions), state=state,
            thrust_model='F_rotor_N=1961.04*9.80665e-3*u^2',
            yaw_formula='Rz(yaw)*(raw-zero) for force and torque; positive yaw rotates sensor x toward output y')
        temp = self.run_dir / 'metadata.tmp'
        temp.write_text(json.dumps(data, indent=2))
        os.replace(temp, self.run_dir / 'metadata.json')

    def stop_run(self, state):
        self.manual_motor = 0.0
        self.manual_servo_target = [0.0] * 6
        for toggle in (self.servo_chk, self.offboard_chk):
            toggle.blockSignals(True)
            toggle.setChecked(False)
            toggle.blockSignals(False)
        self.ros.motor(0)
        self.ros.servo([0] * 6)
        self.ros.request(self.ros.arm_pub, False)
        self.ros.request(self.ros.mode_pub, False)
        if state == 'emergency':
            self.ros.request(self.ros.stop_pub)
        if self.phase != 'IDLE':
            self.save_metadata(state)
        for name in ('raw_file', 'summary_file', 'dist_file'):
            file = getattr(self, name, None)
            if file:
                file.flush()
                os.fsync(file.fileno())
                file.close()
                setattr(self, name, None)
        self.phase = 'IDLE'
        self.servo_stable_start = None
        self.phase_label.setText(state.upper())
        if state == 'complete':
            self.progress.setValue(100)
        self.run_dir = None
        self.set_running(False)
        self.validate_baseline()

    def process_sensor(self, stamp, sample, received=None):
        self.raw = np.array(sample, dtype=float)
        if not np.isfinite(self.raw).all():
            return
        if self.calibrating and (received is None or received >= self.cal_start):
            self.cal_samples.append(self.raw.copy())
        self.corrected = self.raw - self.zero
        c = math.cos(math.radians(self.yaw.value()))
        s = math.sin(math.radians(self.yaw.value()))
        self.rotated = self.corrected.copy()
        for offset in (0, 3):
            x, y = self.corrected[offset:offset + 2]
            self.rotated[offset] = c * x - s * y
            self.rotated[offset + 1] = s * x + c * y
        if self.phase != 'IDLE' and self.raw_file:
            a, b, u = self.conditions[self.index]
            row = dict(timestamp=datetime.now().isoformat(timespec='microseconds'), sensor_timestamp_us=stamp,
                phase=self.phase, condition_index=self.index, alpha_deg=a, beta_deg=b, motor_target_u=u,
                motor_output_u=self.motor_output, alpha_target_deg=a, beta_target_deg=b)
            row.update({f'servo_{i}_target_deg': v for i, v in enumerate(self.servo_target)})
            row.update({f'servo_{i}_commanded_deg': v for i, v in enumerate(self.servo_commanded)})
            for label, value in zip(AXES, self.raw): row[f'raw_{label}'] = value
            for label, value in zip(AXES, self.corrected): row[f'corrected_{label}'] = value
            for label, value in zip(AXES, self.rotated): row[f'rotated_{label}'] = value
            self.raw_writer.writerow(row)
            self.raw_file.flush()
            if (self.phase == 'MEASUREMENT' and
                    (received is None or self.phase_start <= received <= self.phase_start + self.measure.value())):
                self.samples.append(self.rotated.copy())
        condition = self.conditions[self.index] if self.phase != 'IDLE' else None
        baseline = self.baseline.get(condition) if condition and self.wall.isChecked() else None
        delta = self.rotated - baseline if baseline is not None else np.full(6, np.nan)
        self.graph_time.append(time.monotonic() - self.t0)
        for store, vector in zip(self.graph_values, (self.rotated[:3], self.rotated[3:], delta[:3], delta[3:])):
            store.append(vector.copy())

    def finish_condition(self):
        a, b, u = self.conditions[self.index]
        array = np.array(self.samples)
        valid = len(array) >= 2 and np.isfinite(array).all()
        mean = np.mean(array, axis=0) if valid else np.full(6, np.nan)
        var = np.var(array, axis=0, ddof=1) if valid else np.full(6, np.nan)
        std = np.sqrt(var)
        row = dict(condition_index=self.index, alpha_deg=a, beta_deg=b, motor_u=u,
            motor_percent=100 * u, estimated_rotor_thrust_N=MAX_THRUST * u * u,
            estimated_total_thrust_N=4 * MAX_THRUST * u * u, sample_count=len(array), valid=valid)
        for i, axis in enumerate(AXES):
            row[f'{axis}_mean'] = mean[i]
            row[f'{axis}_variance'] = var[i]
            row[f'{axis}_std'] = std[i]
        self.summary_writer.writerow(row)
        self.summary_file.flush()
        os.fsync(self.summary_file.fileno())
        if self.dist_file:
            base = self.baseline[(a, b, u)]
            delta = mean - base
            row = dict(condition_index=self.index, alpha_deg=a, beta_deg=b, motor_u=u,
                sample_count=len(array), valid=valid)
            row.update({f'delta_{axis}_mean': delta[i] for i, axis in enumerate(AXES)})
            self.dist_writer.writerow(row)
            self.dist_file.flush()
            os.fsync(self.dist_file.fileno())
        self.index += 1
        self.save_metadata('running')
        if self.index >= len(self.conditions):
            self.stop_run('complete')
            return
        prev = (a, b)
        next_angles = self.conditions[self.index][:2]
        self.samples = []
        self.phase_start = time.monotonic()
        if next_angles != prev:
            self.phase = 'MOTOR_DOWN'
            self.servo_stable_start = None
            self.ros.motor(0)
        else:
            self.ramp_from = self.motor_output
            self.phase = 'RAMP'

    def experiment_tick(self, now):
        if self.phase == 'IDLE':
            return
        if not self.ready_for_control() or self.px4[0] != VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.px4[1] != VehicleStatus.ARMING_STATE_ARMED:
            self.stop_run('connection_loss')
            return
        a, b, u = self.conditions[self.index]
        elapsed = now - self.phase_start
        if self.phase == 'MOTOR_DOWN':
            self.ros.motor(0)
            if self.motor_output <= 0.001 and elapsed >= 0.3:
                self.servo_target = self.target_angles(a, b)
                self.ros.servo(self.servo_target)
                self.phase = 'SERVO'
                self.phase_start = now
                self.servo_stable_start = None
        elif self.phase == 'SERVO':
            self.ros.motor(0)
            self.ros.servo(self.servo_target)
            if (self.connected('servo_commanded', 0.2) and
                    self.last['servo_commanded'] >= self.phase_start and
                    len(self.servo_commanded) == 6 and
                    max(abs(x - y) for x, y in zip(self.servo_commanded, self.servo_target)) <= SERVO_TARGET_TOL_DEG):
                if self.servo_stable_start is None:
                    self.servo_stable_start = now
                elif now - self.servo_stable_start >= SERVO_SETTLING_SEC:
                    self.phase = 'RAMP'
                    self.phase_start = now
                    self.ramp_from = 0.0
            else:
                self.servo_stable_start = None
            if elapsed >= SERVO_MOVE_TIMEOUT_SEC and self.phase == 'SERVO':
                self.stop_run('servo_timeout')
                return
        elif self.phase == 'RAMP':
            fraction = min(1.0, elapsed / self.ramp.value())
            self.ros.motor(self.ramp_from + (u - self.ramp_from) * fraction)
            if fraction >= 1.0:
                self.phase = 'SETTLING'
                self.phase_start = now
        elif self.phase == 'SETTLING':
            self.ros.motor(u)
            if elapsed >= self.settle.value():
                self.samples = []
                self.phase = 'MEASUREMENT'
                self.phase_start = now
        elif self.phase == 'MEASUREMENT':
            self.ros.motor(u)
            if elapsed >= self.measure.value():
                self.finish_condition()
                if self.phase == 'IDLE':
                    return
        command_error = (max(abs(x - y) for x, y in zip(self.servo_commanded, self.servo_target))
            if len(self.servo_commanded) == 6 else math.inf)
        self.angle_label.setText(f'Alpha target {a:.2f}° | Beta target {b:.2f}° | servo command error {command_error:.2f}°')
        self.phase_label.setText(f'{self.phase} | condition {self.index + 1}/{len(self.conditions)} | u={u:.3f}')
        self.progress.setValue(int(100 * self.index / len(self.conditions)))

    def tick(self):
        now = time.monotonic()
        for _ in range(1000):
            try: kind, received, data = self.events.get_nowait()
            except queue.Empty: break
            self.last[kind] = received
            if kind == 'px4': self.px4 = data
            elif kind == 'servo': self.servo_ready = data
            elif kind == 'servo_commanded': self.servo_commanded = data
            elif kind == 'bldc': self.bldc = data
            elif kind == 'output': self.motor_output = data
            elif kind == 'sensor': self.process_sensor(*data, received=received)
        if self.calibrating and now - self.cal_start >= self.cal_time.value():
            self.calibrating = False
            if len(self.cal_samples) >= 2 and self.connected('sensor', 0.5):
                self.zero = np.mean(self.cal_samples, axis=0)
                self.calibrated = True
                self.cal_label.setText(f'Calibrated: {len(self.cal_samples)} samples')
            else:
                self.cal_label.setText('Calibration failed: insufficient samples')
            self.validate_baseline()
        self.px4_label.setText('PX4 DDS: connected' if self.connected('px4', 1) else 'PX4 DDS: disconnected')
        self.servo_label.setText('Dynamixel: ready' if self.connected('servo', 0.2) and self.servo_ready else 'Dynamixel: not ready')
        self.sensor_label.setText('F/T: connected' if self.connected('sensor', 0.5) else 'F/T: disconnected')
        self.bldc_label.setText(f'BLDC: {self.bldc}' if self.connected('bldc', 1) else 'BLDC: disconnected')
        self.force_label.setText('Raw: ' + ', '.join(f'{x:.3f}' for x in self.raw) + '\nCorrected+Yaw: ' + ', '.join(f'{x:.3f}' for x in self.rotated))
        self.motor_label.setText(f'command u: {100*self.motor_output:.1f}% normalized | estimated: {MAX_THRUST*self.motor_output**2:.2f} N/rotor, {4*MAX_THRUST*self.motor_output**2:.2f} N total')
        self.ros.motor_limit(self.u_max.value())
        self.experiment_tick(now)
        if self.phase == 'IDLE':
            if self.manual_motor and (self.manual_motor > self.u_max.value() + 1e-6 or
                    not self.offboard_chk.isChecked() or not self.ready_for_control() or
                    self.px4[0] != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                    self.px4[1] != VehicleStatus.ARMING_STATE_ARMED):
                self.check_disarm()
            self.ros.motor(self.manual_motor if self.offboard_chk.isChecked() else 0)
            if self.servo_chk.isChecked() and self.connected('servo', 0.2) and self.servo_ready:
                self.servo_target = self.manual_servo_target.copy()
            else:
                self.manual_servo_target = [0.0] * 6
                self.servo_target = [0.0] * 6
            self.ros.servo(self.servo_target)
        self.check_motor_label.setText(
            f'u max {self.u_max.value():.3f} | applied {100*self.manual_motor:.1f}% normalized | '
            f'estimated {MAX_THRUST*self.manual_motor**2:.2f} N/rotor, '
            f'{4*MAX_THRUST*self.manual_motor**2:.2f} N total')
        self.check_label.setText(
            f'F/T: {"connected" if self.connected("sensor", 0.5) else "disconnected"} | '
            f'Dynamixel: {"ready" if self.connected("servo", 0.2) and self.servo_ready else "not ready"} | '
            f'PX4: {"connected" if self.connected("px4", 1) else "disconnected"} | '
            f'BLDC: {self.bldc if self.connected("bldc", 1) else "disconnected"} | '
            f'manual u: {self.manual_motor:.3f}, servo target: {self.servo_target}, '
            f'filtered command: {[round(x, 2) for x in self.servo_commanded]}')
        if self.graph_time:
            t = list(self.graph_time)
            for i, (plot, curves, values) in enumerate(zip(self.plots, self.curves, self.graph_values)):
                matrix = np.array(values)
                for j, curve in enumerate(curves): curve.setData(t, matrix[:, j])
                if i >= 2:
                    plot.setTitle(('Wall-induced Force [N]' if i == 2 else 'Wall-induced Torque [Nm]') +
                        ('' if self.wall.isChecked() and self.baseline else ' — Baseline not loaded'))

    def closeEvent(self, event):
        self.stop_run('closed')
        self.timer.stop()
        rclpy.shutdown()
        self.ros_thread.join(timeout=2)
        self.ros.destroy_node()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    window = Gui()
    window.show()
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
