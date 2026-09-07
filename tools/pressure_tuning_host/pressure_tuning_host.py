from __future__ import annotations

import json
import base64
import hashlib
import hmac
import math
import os
import socket
import struct
import sys
import threading
import time
import urllib.parse
import urllib.request
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports
from PyQt5.QtCore import QPointF, QSettings, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
)

from protocol import (
    CMD_DEBUG_CONTROL, CMD_DEFAULTS, CMD_GET_ALL, CMD_GET_STATUS,
    CMD_GET_HEAT_PID, CMD_HELLO, CMD_PREPARE, CMD_PREPARE_HEAT, CMD_SAVE,
    CMD_SET_FIELD, CMD_SET_HEAT_PID, CMD_SET_PROFILE, CMD_SET_TEMPERATURE,
    CMD_START, CMD_STOP,
    CMD_TELEMETRY_CONTROL, PROFILE_KEYS, RSP_ACK, RSP_HEAT_PID, RSP_HOME_EVENT,
    RSP_INFO, RSP_PROFILE, RSP_TELEMETRY, STATUS_TEXT, FrameParser, build_frame,
    build_screen_work_frame, decode_heat_pid, decode_home_event, decode_profile,
    decode_telemetry, encode_heat_pid, encode_profile, encode_profile_field,
)


DINGTALK_WEBHOOK = "https://oapi.dingtalk.com/robot/send?access_token=b0ef02462cd85301a8252512f9f4160eb798032a5281318c33da02a29692b64d"
DINGTALK_SECRET = "SEC099b9cec364bec6b0008c14193e20b4bb6eea8c2b793642f4e1dd2cc95fe820d"
STATS_DIR = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "HTP100_Pressure_Tuning_Host"
STATS_FILE = STATS_DIR / "aging_stats.json"
AGING_LOG_FILE = STATS_DIR / "aging_cycles.txt"
EFFECTIVE_AREA_MM2 = 84.8
PASCAL_PER_MMHG = 133.322
NEWTON_PER_MMHG = PASCAL_PER_MMHG * EFFECTIVE_AREA_MM2 * 1.0e-6


APP_STATES = ["启动", "回零", "待机", "预热", "就绪", "治疗", "停止", "故障", "关机"]
STAGES = {0: "快速前进", 1: "慢速前进", 2: "PID保压", 3: "回退", 0xFF: "未运行"}
RANGES = ["≤200", "201–300", "301–400", "401–500", ">500"]
COLUMNS = [
    ("灵敏度\nmV/V", "sensitivity_mv_v", 0.05, 5.0, 4),
    ("偏移量\nmmHg", "offset_mmhg", -200.0, 200.0, 1),
    ("快速速度", "fast_speed", 0.0, 65535.0, 0),
    ("慢速速度", "approach_speed", 0.0, 65535.0, 0),
    ("回退速度", "retract_speed", 0.0, 65535.0, 0),
    ("回退时间\nms", "retract_ms", 0.0, 10000.0, 0),
    ("快慢切换\n目标%", "speed_switch_percent", 5.0, 100.0, 1),
    ("PID切换\n目标%", "hold_switch_percent", 5.0, 100.0, 1),
    ("Kp", "kp", 0.0, 1000.0, 1),
    ("Ki", "ki", 0.0, 100.0, 1),
]
DEFAULTS = [
    [0.380, 0, 20000, 6000, 15000, 30, 50, 500, 50, 0],
    [0.370, 0, 30000, 9000, 18000, 90, 92, 600, 150, 0],
    [0.370, 0, 40000, 9000, 20000, 65, 70, 800, 150, 0],
    [0.370, 0, 45000, 9000, 20000, 55, 60, 800, 100, 0],
    [0.370, 0, 45000, 9000, 20000, 50, 50, 1000, 100, 0],
]


class PressurePlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.samples = deque(maxlen=600)
        self.setMinimumHeight(220)

    def add_sample(self, pressure: float, target: float) -> None:
        if math.isfinite(pressure) and math.isfinite(target):
            self.samples.append((pressure, target))
            self.update()

    def clear(self) -> None:
        self.samples.clear()
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#10151d"))
        left, top, right, bottom = 48, 14, 12, 28
        width = max(1, self.width() - left - right)
        height = max(1, self.height() - top - bottom)
        values = list(self.samples)
        max_y = max(100.0, *(max(p, t) for p, t in values)) if values else 650.0
        max_y = min(700.0, math.ceil((max_y + 25.0) / 50.0) * 50.0)
        painter.setFont(QFont("Microsoft YaHei", 8))
        for i in range(6):
            y = top + height * i / 5
            value = max_y * (5 - i) / 5
            painter.setPen(QPen(QColor("#263241"), 1))
            painter.drawLine(left, int(y), left + width, int(y))
            painter.setPen(QColor("#8694a6"))
            painter.drawText(3, int(y + 4), f"{value:.0f}")
        if len(values) < 2:
            painter.setPen(QColor("#758397"))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待压力遥测数据")
            return
        def points(field: int):
            count = len(values)
            return [QPointF(left + width * i / max(1, count - 1),
                            top + height * (1.0 - max(0.0, min(max_y, row[field])) / max_y))
                    for i, row in enumerate(values)]
        painter.setPen(QPen(QColor("#f5c542"), 1.5))
        painter.drawPolyline(*points(1))
        painter.setPen(QPen(QColor("#18b9ff"), 2.0))
        painter.drawPolyline(*points(0))
        painter.setPen(QColor("#18b9ff"))
        painter.drawText(left, self.height() - 8, "压力")
        painter.setPen(QColor("#f5c542"))
        painter.drawText(left + 48, self.height() - 8, "目标")


class TemperaturePlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.samples = deque(maxlen=600)
        self.setMinimumHeight(220)

    def add_sample(self, temperature: float, target: float,
                   power_percent: float | None = None,
                   integral_output_percent: float | None = None) -> None:
        if math.isfinite(temperature) and math.isfinite(target):
            power = (power_percent if power_percent is not None and
                     math.isfinite(power_percent) else None)
            integral_output = (
                integral_output_percent
                if integral_output_percent is not None and
                math.isfinite(integral_output_percent) else None)
            self.samples.append((temperature, target, power, integral_output))
            self.update()

    def clear(self) -> None:
        self.samples.clear()
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#10151d"))
        left, top, right, bottom = 48, 14, 48, 28
        width = max(1, self.width() - left - right)
        height = max(1, self.height() - top - bottom)
        values = list(self.samples)
        if values:
            all_values = [value for row in values for value in row[:2]]
            min_y = math.floor((min(all_values) - 2.0) / 5.0) * 5.0
            max_y = math.ceil((max(all_values) + 2.0) / 5.0) * 5.0
            if max_y - min_y < 10.0:
                max_y = min_y + 10.0
        else:
            min_y, max_y = 20.0, 50.0
        painter.setFont(QFont("Microsoft YaHei", 8))
        integral_outputs = [abs(row[3]) for row in values if row[3] is not None]
        auxiliary_limit = max(100.0, math.ceil(max(integral_outputs, default=0.0) / 50.0) * 50.0)
        for i in range(6):
            y = top + height * i / 5
            value = max_y - (max_y - min_y) * i / 5
            auxiliary_value = auxiliary_limit - 2.0 * auxiliary_limit * i / 5
            painter.setPen(QPen(QColor("#263241"), 1))
            painter.drawLine(left, int(y), left + width, int(y))
            painter.setPen(QColor("#8694a6"))
            painter.drawText(3, int(y + 4), f"{value:.1f}")
            painter.drawText(self.width() - 45, int(y + 4), f"{auxiliary_value:.0f}")
        if len(values) < 2:
            painter.setPen(QColor("#758397"))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待温度遥测数据")
            return

        def points(field: int):
            count = len(values)
            span = max_y - min_y
            return [QPointF(
                left + width * i / max(1, count - 1),
                top + height * (1.0 - max(0.0, min(1.0, (row[field] - min_y) / span))))
                for i, row in enumerate(values)]

        def auxiliary_points(field: int):
            count = len(values)
            return [QPointF(
                left + width * i / max(1, count - 1),
                top + height * (1.0 - (
                    max(-auxiliary_limit, min(auxiliary_limit, row[field])) +
                    auxiliary_limit) / (2.0 * auxiliary_limit)))
                for i, row in enumerate(values) if row[field] is not None]

        painter.setPen(QPen(QColor("#f5c542"), 1.5))
        painter.drawPolyline(*points(1))
        painter.setPen(QPen(QColor("#ff5c7a"), 2.0))
        painter.drawPolyline(*points(0))
        power_points = auxiliary_points(2)
        if len(power_points) >= 2:
            painter.setPen(QPen(QColor("#18b9ff"), 1.8))
            painter.drawPolyline(*power_points)
        integral_points = auxiliary_points(3)
        if len(integral_points) >= 2:
            painter.setPen(QPen(QColor("#a77bff"), 1.8))
            painter.drawPolyline(*integral_points)
        painter.setPen(QColor("#ff5c7a"))
        painter.drawText(left, self.height() - 8, "温度")
        painter.setPen(QColor("#f5c542"))
        painter.drawText(left + 48, self.height() - 8, "目标")
        painter.setPen(QColor("#18b9ff"))
        painter.drawText(left + 96, self.height() - 8, "功率%")
        painter.setPen(QColor("#a77bff"))
        painter.drawText(left + 156, self.height() - 8, "积分输出%")


class MainWindow(QMainWindow):
    ding_result = pyqtSignal(bool, str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("HTP100 压力标定、温控/压力 PID 调参工具")
        self.resize(1550, 980)
        self.serial = None
        self.parser = FrameParser()
        self.sequence = 0
        self.command_queue = deque()
        self.pending = None
        self.pending_since = 0
        self.last_frame_received = 0.0
        self.ignored_bytes = 0
        self.profile_widgets = []
        self.suppress_table_writes = False
        self.pending_field_updates = {}
        self.last_telemetry = {}
        self.aging_state = "idle"
        self.aging_deadline = 0.0
        self.aging_run_started = 0.0
        self.aging_max_pressure = 0.0
        self.last_debug_keepalive = 0.0
        self.debug_mode_enabled = False
        self.stats = self._load_aging_stats()
        self.settings = QSettings("Shengluokai", "HTP100PressureTuningHost")
        self.ui_font_size = int(self.settings.value("font_size", 10))
        self.ui_scale_percent = int(self.settings.value("ui_scale", 100))
        self.aging_pressure_tolerance_value = float(
            self.settings.value("aging_pressure_tolerance_mmhg", 25.0))
        self._build_ui()
        self.field_write_timer = QTimer(self)
        self.field_write_timer.setSingleShot(True)
        self.field_write_timer.timeout.connect(self._flush_field_updates)
        self._load_defaults()
        self.io_timer = QTimer(self)
        self.io_timer.timeout.connect(self._poll_serial)
        self.io_timer.start(20)
        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self._refresh_ports)
        self.port_timer.start(1500)
        self.aging_timer = QTimer(self)
        self.aging_timer.timeout.connect(self._aging_tick)
        self.aging_timer.start(100)
        self.ding_result.connect(self._ding_result)
        self._refresh_ports()

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        self.setCentralWidget(root)

        connection = QHBoxLayout()
        connection.addWidget(QLabel("USART2 串口"))
        self.port_box = QComboBox()
        self.port_box.setMinimumWidth(180)
        connection.addWidget(self.port_box)
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self._toggle_connection)
        connection.addWidget(self.connect_button)
        self.connection_label = QLabel("未连接")
        connection.addWidget(self.connection_label)
        connection.addStretch()
        protocol_note = QLabel("调参：7A A7 ｜ 实际屏幕：5A A5 / 6A A6 ｜ 115200 8N1")
        protocol_note.setStyleSheet("color:#f0a830; font-weight:600")
        connection.addWidget(protocol_note)
        connection.addWidget(QLabel("字体"))
        self.font_size_spin = QSpinBox()
        self.font_size_spin.setRange(8, 24)
        self.font_size_spin.setValue(self.ui_font_size)
        self.font_size_spin.setSuffix(" pt")
        self.font_size_spin.valueChanged.connect(self._apply_ui_scale)
        connection.addWidget(self.font_size_spin)
        connection.addWidget(QLabel("界面"))
        self.ui_scale_spin = QSpinBox()
        self.ui_scale_spin.setRange(75, 200)
        self.ui_scale_spin.setSingleStep(5)
        self.ui_scale_spin.setValue(self.ui_scale_percent)
        self.ui_scale_spin.setSuffix(" %")
        self.ui_scale_spin.valueChanged.connect(self._apply_ui_scale)
        connection.addWidget(self.ui_scale_spin)
        layout.addLayout(connection)

        table_group = QGroupBox("五挡压力参数（修改即时写 RAM；保存会先同步整表再写外置 EEPROM）")
        self.table_group = table_group
        table_group.setFixedHeight(280)
        table_layout = QVBoxLayout(table_group)
        self.table = QTableWidget(5, len(COLUMNS) + 1)
        self.table.setHorizontalHeaderLabels(["目标压力档"] + [c[0] for c in COLUMNS])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setFixedHeight(200)
        self.table.setColumnWidth(0, 90)
        for row, label in enumerate(RANGES):
            item = QTableWidgetItem(label + " mmHg")
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, 0, item)
            row_widgets = {}
            for col, (_title, key, minimum, maximum, decimals) in enumerate(COLUMNS, 1):
                spin = QDoubleSpinBox()
                spin.setRange(minimum, maximum)
                spin.setDecimals(decimals)
                spin.setKeyboardTracking(False)
                spin.setAlignment(Qt.AlignRight)
                spin.setButtonSymbols(QDoubleSpinBox.NoButtons)
                self.table.setCellWidget(row, col, spin)
                row_widgets[key] = spin
                spin.valueChanged.connect(
                    lambda value, profile=row, field=key:
                    self._schedule_field_update(profile, field, value))
            self.profile_widgets.append(row_widgets)
        self.table.resizeColumnsToContents()
        table_layout.addWidget(self.table)
        buttons = QHBoxLayout()
        self.instant_write_check = QCheckBox("单元格修改后即时写入 RAM")
        self.instant_write_check.setChecked(True)
        buttons.addWidget(self.instant_write_check)
        for text, callback in (
            ("读取主控 RAM 五挡", self._read_all),
            ("同步整表并保存到外置 EEPROM", self._save_to_storage),
            ("恢复程序默认值", self._restore_defaults), ("导入 JSON", self._import_json),
            ("导出 JSON", self._export_json),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        buttons.addStretch()
        table_layout.addLayout(buttons)
        layout.addWidget(table_group)

        aging_group = QGroupBox("调试与自动老化（测试专用）")
        aging = QGridLayout(aging_group)
        self.aging_button = QPushButton("进入调试并开始老化")
        self.aging_button.setCheckable(True)
        self.aging_button.clicked.connect(self._toggle_aging)
        aging.addWidget(self.aging_button, 0, 0, 1, 2)
        self.ding_button = QPushButton("测试钉钉通知")
        self.ding_button.clicked.connect(self._test_dingtalk)
        aging.addWidget(self.ding_button, 0, 2)
        aging.addWidget(QLabel("老化目标压力"), 0, 3)
        self.aging_target = QDoubleSpinBox()
        self.aging_target.setRange(1, 625)
        self.aging_target.setValue(350)
        self.aging_target.setSuffix(" mmHg")
        self.aging_target.valueChanged.connect(self._update_force_labels)
        aging.addWidget(self.aging_target, 0, 4)
        self.aging_force_label = QLabel()
        self.aging_force_label.setStyleSheet("font-weight:600;color:#196a8a")
        aging.addWidget(self.aging_force_label, 0, 5)
        self.continue_after_failure = QCheckBox("失败后继续下一轮")
        aging.addWidget(self.continue_after_failure, 0, 6)
        aging.addWidget(QLabel("前进时间"), 1, 0)
        self.aging_forward_seconds = QDoubleSpinBox()
        self.aging_forward_seconds.setRange(1.0, 120.0)
        self.aging_forward_seconds.setDecimals(1)
        self.aging_forward_seconds.setValue(5.0)
        self.aging_forward_seconds.setSuffix(" 秒")
        aging.addWidget(self.aging_forward_seconds, 1, 1)
        aging.addWidget(QLabel("后退等待"), 1, 2)
        self.aging_reverse_seconds = QDoubleSpinBox()
        self.aging_reverse_seconds.setRange(1.0, 120.0)
        self.aging_reverse_seconds.setDecimals(1)
        self.aging_reverse_seconds.setValue(5.0)
        self.aging_reverse_seconds.setSuffix(" 秒")
        aging.addWidget(self.aging_reverse_seconds, 1, 3)
        aging.addWidget(QLabel("允许欠压"), 1, 4)
        self.aging_pressure_tolerance = QDoubleSpinBox()
        self.aging_pressure_tolerance.setRange(0.0, 200.0)
        self.aging_pressure_tolerance.setDecimals(1)
        self.aging_pressure_tolerance.setValue(
            self.aging_pressure_tolerance_value)
        self.aging_pressure_tolerance.setSuffix(" mmHg")
        self.aging_pressure_tolerance.setToolTip(
            "前进时间内最高压力低于‘目标压力－允许欠压’时，本轮判定失败")
        self.aging_pressure_tolerance.valueChanged.connect(
            lambda value: self.settings.setValue(
                "aging_pressure_tolerance_mmhg", value))
        aging.addWidget(self.aging_pressure_tolerance, 1, 5)
        self.aging_status = QLabel("未运行")
        self.aging_status.setStyleSheet("font-size:15px;font-weight:600;color:#536274")
        aging.addWidget(self.aging_status, 2, 0, 1, 3)
        self.stats_label = QLabel()
        aging.addWidget(self.stats_label, 2, 3, 1, 4)
        warning = QLabel("流程：0x1037 进入自动预热 → 0x1036 开始自动治疗 → 按前进时间挤压 → 0x1038 停止 → 按后退时间等待回零")
        warning.setStyleSheet("color:#a46610")
        aging.addWidget(warning, 3, 0, 1, 7)
        self._refresh_stats_label()
        layout.addWidget(aging_group)

        heat_pid_group = QGroupBox("温控 PID 调试（参数写入 RAM，重启恢复默认值）")
        heat_pid = QGridLayout(heat_pid_group)
        self.heat_target = QDoubleSpinBox()
        self.heat_target.setRange(20.0, 43.5)
        self.heat_target.setDecimals(1)
        self.heat_target.setValue(42.5)
        self.heat_target.setSuffix(" °C")
        self.heat_target.setKeyboardTracking(False)
        heat_pid.addWidget(QLabel("目标温度"), 0, 0)
        heat_pid.addWidget(self.heat_target, 0, 1)
        self.heat_pid_widgets = {}
        for column, (key, title, maximum, value) in enumerate((
            ("kp", "Kp", 100.0, 30.0),
            ("ki", "Ki", 50.0, 5.0),
            ("kd", "Kd", 20.0, 5.0),
        ), 1):
            spin = QDoubleSpinBox()
            spin.setRange(0.0, maximum)
            spin.setDecimals(3)
            spin.setValue(value)
            spin.setKeyboardTracking(False)
            self.heat_pid_widgets[key] = spin
            heat_pid.addWidget(QLabel(title), 0, column * 2)
            heat_pid.addWidget(spin, 0, column * 2 + 1)
        apply_pid_button = QPushButton("应用温控 PID")
        apply_pid_button.clicked.connect(self._write_heat_pid)
        heat_pid.addWidget(apply_pid_button, 0, 8)
        read_pid_button = QPushButton("读取温控 PID")
        read_pid_button.clicked.connect(self._read_heat_pid)
        heat_pid.addWidget(read_pid_button, 0, 9)
        start_heat_button = QPushButton("准备并开始温控")
        start_heat_button.setToolTip(
            "依次写入目标与 PID、准备加热并开始治疗；需要有效眼盾，并遵循眼盾消耗规则。")
        start_heat_button.clicked.connect(self._start_heat_tuning)
        heat_pid.addWidget(start_heat_button, 0, 10)
        stop_heat_button = QPushButton("停止并回零")
        stop_heat_button.setStyleSheet("background:#9b2f35;color:white")
        stop_heat_button.clicked.connect(self._stop)
        heat_pid.addWidget(stop_heat_button, 0, 11)
        layout.addWidget(heat_pid_group)

        lower = QHBoxLayout()
        control_group = QGroupBox("压力运行测试")
        controls = QGridLayout(control_group)
        controls.addWidget(QLabel("目标压力"), 0, 0)
        self.target = QDoubleSpinBox()
        self.target.setRange(1, 650)
        self.target.setValue(350)
        self.target.setSuffix(" mmHg")
        self.target.valueChanged.connect(self._update_force_labels)
        controls.addWidget(self.target, 0, 1)
        self.target_force_label = QLabel()
        self.target_force_label.setStyleSheet("font-weight:600;color:#196a8a")
        controls.addWidget(self.target_force_label, 0, 2)
        self.prepare_button = QPushButton("1. 准备压力")
        self.prepare_button.clicked.connect(self._prepare)
        self.start_button = QPushButton("2. 开始")
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton("停止并回零")
        self.stop_button.setStyleSheet("background:#9b2f35;color:white")
        self.stop_button.clicked.connect(self._stop)
        controls.addWidget(self.prepare_button, 1, 0)
        controls.addWidget(self.start_button, 1, 1)
        controls.addWidget(self.stop_button, 1, 2)
        self.telemetry_check = QCheckBox("实时遥测")
        self.telemetry_check.setChecked(True)
        self.telemetry_check.toggled.connect(self._set_telemetry)
        self.period = QSpinBox()
        self.period.setRange(50, 2000)
        self.period.setValue(50)
        self.period.setSuffix(" ms")
        self.period.valueChanged.connect(self._set_telemetry)
        controls.addWidget(self.telemetry_check, 2, 0)
        controls.addWidget(self.period, 2, 1)
        clear_button = QPushButton("清空曲线")
        clear_button.clicked.connect(self.plot_clear)
        controls.addWidget(clear_button, 2, 2)
        self.live_labels = {}
        fields = [("pressure", "压力"), ("temperature", "温度"),
                  ("motor_speed", "电机速度指令"), ("pressure_error", "压力误差"),
                  ("heat_power", "加热功率"), ("heat_integral_output", "温控积分输出"),
                  ("raw", "ADC原始值"), ("zero", "零点原始值"),
                  ("stage", "压力阶段"), ("state", "设备状态"), ("eye", "眼盾/回零"),
                  ("power", "供电/调试"), ("fault", "故障码")]
        live_widget = QWidget()
        live_grid = QGridLayout(live_widget)
        live_grid.setContentsMargins(0, 0, 0, 0)
        live_grid.setVerticalSpacing(3)
        for index, (key, title) in enumerate(fields):
            row = index // 2
            column = (index % 2) * 2
            live_grid.addWidget(QLabel(title), row, column)
            label = QLabel("—")
            label.setStyleSheet("font-size:14px;font-weight:600")
            live_grid.addWidget(label, row, column + 1)
            self.live_labels[key] = label
        controls.addWidget(live_widget, 3, 0, 1, 3)
        self.plot = PressurePlot()
        self.temperature_plot = TemperaturePlot()
        lower.addWidget(control_group, 1)
        lower.addWidget(self.plot, 2)
        lower.addWidget(self.temperature_plot, 2)
        layout.addLayout(lower)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(165)
        self.log.setStyleSheet("font-family:Consolas,Microsoft YaHei")
        layout.addWidget(self.log)
        self.statusBar().showMessage("表格修改即时写入 RAM；点击保存后写入外置 EEPROM，重启仍有效")
        self._update_force_labels()
        self._apply_ui_scale()

    def _update_force_labels(self, *_args) -> None:
        if hasattr(self, "target_force_label"):
            self.target_force_label.setText(
                f"≈ {self.target.value() * NEWTON_PER_MMHG:.3f} N")
        if hasattr(self, "aging_force_label"):
            self.aging_force_label.setText(
                f"≈ {self.aging_target.value() * NEWTON_PER_MMHG:.3f} N")

    def _apply_ui_scale(self, *_args) -> None:
        if not hasattr(self, "font_size_spin"):
            return
        font_size = self.font_size_spin.value()
        scale_percent = self.ui_scale_spin.value()
        scale = scale_percent / 100.0
        QApplication.instance().setFont(QFont("Microsoft YaHei", font_size))
        if hasattr(self, "table"):
            self.table.verticalHeader().setDefaultSectionSize(max(22, int(26 * scale)))
            self.table.setFixedHeight(max(175, int(200 * scale)))
            self.table_group.setFixedHeight(max(250, int(280 * scale)))
        if hasattr(self, "plot"):
            self.plot.setMinimumHeight(int(220 * scale))
        if hasattr(self, "temperature_plot"):
            self.temperature_plot.setMinimumHeight(int(220 * scale))
        if hasattr(self, "log"):
            self.log.setMaximumHeight(int(165 * scale))
        self.settings.setValue("font_size", font_size)
        self.settings.setValue("ui_scale", scale_percent)

    def _load_defaults(self) -> None:
        for row, values in enumerate(DEFAULTS):
            self._set_profile(row, dict(zip(PROFILE_KEYS, values)))

    def _get_profile(self, row: int) -> dict:
        return {key: widget.value() for key, widget in self.profile_widgets[row].items()}

    def _set_profile(self, row: int, values: dict) -> None:
        if not 0 <= row < 5:
            return
        previous = self.suppress_table_writes
        self.suppress_table_writes = True
        try:
            for key, widget in self.profile_widgets[row].items():
                if key in values:
                    widget.setValue(float(values[key]))
        finally:
            self.suppress_table_writes = previous

    def _schedule_field_update(self, profile: int, field: str, value: float) -> None:
        if self.suppress_table_writes or not hasattr(self, "instant_write_check") or \
                not self.instant_write_check.isChecked():
            return
        self.pending_field_updates[(profile, field)] = float(value)
        self.field_write_timer.start(300)

    def _flush_field_updates(self) -> None:
        updates = list(self.pending_field_updates.items())
        self.pending_field_updates.clear()
        if not updates:
            return
        if not self.serial or not self.serial.is_open:
            self._log("参数只改在表格中：串口未连接，尚未写入主控 RAM", "WARN")
            return
        for (profile, field), value in updates:
            payload = encode_profile_field(profile, field, value)
            self._queue_ack(
                CMD_SET_FIELD, payload,
                f"即时写入第 {profile + 1} 挡 {field}={value:g}")

    def _queue_entire_table_to_ram(self) -> bool:
        profiles = [self._get_profile(row) for row in range(5)]
        for row, values in enumerate(profiles):
            if values["hold_switch_percent"] < values["speed_switch_percent"]:
                QMessageBox.critical(
                    self, "参数错误",
                    f"第 {row + 1} 挡 PID 切换百分比不能小于快慢切换百分比。")
                return False

        # Send one complete profile per pressure range.  The old implementation
        # sent 12 field updates per row before CMD_SAVE, so one missed ACK could
        # discard the remaining queue before the EEPROM save command was ever
        # transmitted.  A complete profile is validated atomically by the MCU
        # and reduces the save sequence from 60 RAM writes to five.
        for row, values in enumerate(profiles):
            self._queue_ack(
                CMD_SET_PROFILE, encode_profile(row, values),
                f"整挡同步第 {row + 1} 挡（含电机 Kp/Ki）")
        return True

    def _log(self, message: str, level: str = "INFO") -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.append(f"[{stamp}] [{level}] {message}")

    def _load_aging_stats(self) -> dict:
        try:
            data = json.loads(STATS_FILE.read_text(encoding="utf-8"))
            return {key: int(data.get(key, 0)) for key in ("total", "success", "failed")}
        except Exception:
            return {"total": 0, "success": 0, "failed": 0}

    def _save_aging_stats(self) -> None:
        STATS_DIR.mkdir(parents=True, exist_ok=True)
        STATS_FILE.write_text(json.dumps(self.stats, ensure_ascii=False, indent=2), encoding="utf-8")
        self._refresh_stats_label()

    def _refresh_stats_label(self) -> None:
        if hasattr(self, "stats_label"):
            self.stats_label.setText(
                f"累计轮回：{self.stats['total']} ｜ 成功：{self.stats['success']} ｜ 失败：{self.stats['failed']}")

    def _record_cycle(self, success: bool, reason: str) -> None:
        self.stats["total"] += 1
        self.stats["success" if success else "failed"] += 1
        self._save_aging_stats()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = (f"{timestamp}\t{'SUCCESS' if success else 'FAILED'}\t"
                f"target={self.aging_target.value():.1f}\tmax={self.aging_max_pressure:.1f}\t"
                f"reason={reason}\ttotal={self.stats['total']}\t"
                f"success={self.stats['success']}\tfailed={self.stats['failed']}\n")
        STATS_DIR.mkdir(parents=True, exist_ok=True)
        with AGING_LOG_FILE.open("a", encoding="utf-8") as handle:
            handle.write(line)

    def _send_dingtalk(self, content: str) -> None:
        def worker() -> None:
            try:
                timestamp = str(int(time.time() * 1000))
                digest = hmac.new(
                    DINGTALK_SECRET.encode("utf-8"),
                    f"{timestamp}\n{DINGTALK_SECRET}".encode("utf-8"),
                    hashlib.sha256,
                ).digest()
                sign = urllib.parse.quote_plus(base64.b64encode(digest).decode("ascii"))
                url = f"{DINGTALK_WEBHOOK}&timestamp={timestamp}&sign={sign}"
                body = json.dumps({"msgtype": "text", "text": {"content": content}},
                                  ensure_ascii=False).encode("utf-8")
                request = urllib.request.Request(
                    url, data=body, headers={"Content-Type": "application/json;charset=utf-8"},
                    method="POST")
                with urllib.request.urlopen(request, timeout=10) as response:
                    result = json.loads(response.read().decode("utf-8"))
                if result.get("errcode") != 0:
                    raise RuntimeError(result.get("errmsg", str(result)))
                self.ding_result.emit(True, "钉钉通知发送成功")
            except Exception as error:
                self.ding_result.emit(False, f"钉钉通知发送失败：{error}")
        threading.Thread(target=worker, daemon=True).start()

    def _ding_result(self, success: bool, message: str) -> None:
        self._log(message, "INFO" if success else "ERROR")

    def _test_dingtalk(self) -> None:
        self._log("正在发送钉钉测试通知")
        self._send_dingtalk(
            f"HTP100上位机钉钉通知测试\n电脑：{socket.gethostname()}\n时间：{datetime.now():%Y-%m-%d %H:%M:%S}")

    def _refresh_ports(self) -> None:
        current = self.port_box.currentText()
        ports = [port.device for port in list_ports.comports()]
        if [self.port_box.itemText(i) for i in range(self.port_box.count())] != ports:
            self.port_box.clear()
            self.port_box.addItems(ports)
            if current in ports:
                self.port_box.setCurrentText(current)

    def _toggle_connection(self) -> None:
        if self.serial and self.serial.is_open:
            self._disconnect()
            return
        port = self.port_box.currentText()
        if not port:
            QMessageBox.warning(self, "无串口", "没有检测到可用串口。")
            return
        try:
            self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.3)
            self.serial.reset_input_buffer()
        except Exception as error:
            QMessageBox.critical(self, "连接失败", str(error))
            return
        self.parser = FrameParser()
        self.last_frame_received = 0.0
        self.connect_button.setText("断开")
        self.connection_label.setText(f"已连接 {port}")
        self.connection_label.setStyleSheet("color:#168a49;font-weight:600")
        self._log(f"连接 {port}，USART2 115200 8N1")
        self._send_direct(CMD_HELLO, b"", "协议握手")
        QTimer.singleShot(120, self._read_all)
        QTimer.singleShot(250, self._set_telemetry)

    def _disconnect(self) -> None:
        if self.serial:
            try:
                if self.serial.is_open:
                    if self.aging_state != "idle" or self.debug_mode_enabled:
                        self.serial.write(build_screen_work_frame(0x1038, 0.0))
                    self.serial.write(build_frame(CMD_DEBUG_CONTROL, self._next_sequence(), b"\x00"))
                    self.serial.flush()
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.pending = None
        self.command_queue.clear()
        self.connect_button.setText("连接")
        self.connection_label.setText("未连接")
        self.connection_label.setStyleSheet("")
        self._set_debug_ui(False)
        self._stop_aging_ui("串口断开，自动老化已停止")
        self._log("串口已断开")

    def _next_sequence(self) -> int:
        value = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        return value

    def _send_direct(self, command: int, payload: bytes, description: str) -> int | None:
        if not self.serial or not self.serial.is_open:
            self._log("尚未连接串口", "WARN")
            return None
        sequence = self._next_sequence()
        try:
            frame = build_frame(command, sequence, payload)
            written = self.serial.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f"仅发送 {written}/{len(frame)} 字节")
            self._log(f"TX {description} cmd=0x{command:02X} seq={sequence}")
            return sequence
        except Exception as error:
            self._log(f"发送失败：{error}", "ERROR")
            self._disconnect()
            return None

    def _send_screen_command(self, command: int, value: float, description: str) -> bool:
        if not self.serial or not self.serial.is_open:
            self._log("尚未连接串口", "WARN")
            return False
        try:
            self.serial.write(build_screen_work_frame(command, value))
            self._log(f"TX 屏幕协议 {description} cmd=0x{command:04X} value={value:.1f}")
            return True
        except Exception as error:
            self._log(f"屏幕命令发送失败：{error}", "ERROR")
            self._disconnect()
            return False

    def _set_debug_ui(self, enabled: bool) -> None:
        self.debug_mode_enabled = enabled

    def _send_debug_keepalive(self) -> None:
        if not self.serial or not self.serial.is_open:
            return
        try:
            self.serial.write(build_frame(CMD_DEBUG_CONTROL, self._next_sequence(), b"\x01"))
            self.last_debug_keepalive = time.monotonic()
        except Exception as error:
            self._log(f"调试模式保活失败：{error}", "ERROR")
            self._disconnect()

    def _toggle_aging(self, enabled: bool) -> None:
        if not enabled:
            if self.aging_state != "idle":
                self._send_screen_command(0x1038, 0.0, "自动治疗停止")
            self._stop_aging_ui("用户停止自动老化")
            return
        if not self.serial or not self.serial.is_open:
            self._stop_aging_ui("未连接串口")
            QMessageBox.warning(self, "未连接", "请先连接主板 USART2。")
            return
        answer = QMessageBox.warning(
            self, "启动自动老化",
            "上位机将循环控制加热和电机，并在调试模式下绕过充电联锁。\n"
            "请确认设备有人看护、机械运动区域安全。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            self._stop_aging_ui("未运行")
            return
        self.aging_button.setText("停止老化并退出调试")
        self.aging_button.setStyleSheet("background:#9b2f35;color:white;font-weight:600")
        if not self.debug_mode_enabled:
            self._queue_ack(CMD_DEBUG_CONTROL, b"\x01", "自动开启调试模式",
                            self._start_aging)
        else:
            self._start_aging()

    def _start_aging(self) -> None:
        self._set_debug_ui(True)
        self.last_debug_keepalive = 0.0
        state = self.last_telemetry.get("app_state")
        fault = self.last_telemetry.get("fault", 0)
        if state is None:
            self.aging_state = "wait_status"
            self.aging_deadline = time.monotonic() + 2.0
            self.aging_status.setText("正在读取主控状态")
            self._send_direct(CMD_GET_STATUS, b"", "自动老化启动前状态查询")
            return
        if fault or state == 7:
            self._aging_fail(f"启动前设备存在故障 0x{fault:04X}", count_cycle=False)
            return
        if state == 2:
            self._begin_aging_cycle()
        elif state == 1:
            self.aging_state = "wait_initial_home"
            self.aging_deadline = time.monotonic() + 20.0
            self.aging_status.setText("等待当前回零完成")
        else:
            self._send_screen_command(0x1038, 0.0, "启动前停止")
            self.aging_state = "wait_initial_home"
            self.aging_deadline = time.monotonic() + 20.0
            self.aging_status.setText("正在停止当前动作并等待回零")

    def _begin_aging_cycle(self) -> None:
        self.aging_max_pressure = 0.0
        self.aging_run_started = 0.0
        self.pending_cycle_failure = ""
        target = self.aging_target.value()
        if not self._send_screen_command(0x1037, target, "自动模式准备/预热"):
            self._aging_fail("自动准备命令发送失败")
            return
        self.aging_state = "wait_ready"
        self.aging_deadline = time.monotonic() + 3.0
        self.aging_prepare_sent = time.monotonic()
        self.aging_status.setText(f"第 {self.stats['total'] + 1} 轮：自动预热命令已发送")

    def _stop_aging_ui(self, message: str) -> None:
        self.aging_state = "idle"
        if hasattr(self, "aging_button"):
            self.aging_button.blockSignals(True)
            self.aging_button.setChecked(False)
            self.aging_button.blockSignals(False)
            self.aging_button.setText("进入调试并开始老化")
            self.aging_button.setStyleSheet("")
            self.aging_status.setText(message)
        if (self.debug_mode_enabled and self.serial and self.serial.is_open):
            self._queue_ack(CMD_DEBUG_CONTROL, b"\x00", "停止老化并退出调试模式",
                            lambda: self._set_debug_ui(False))

    def _complete_cycle(self, success: bool, reason: str, safe_to_continue: bool = True) -> None:
        self._record_cycle(success, reason)
        level = "INFO" if success else "ERROR"
        self._log(
            f"第 {self.stats['total']} 轮{'成功' if success else '失败'}：{reason}，"
            f"最高压力 {self.aging_max_pressure:.1f} mmHg", level)
        if not success:
            self._send_dingtalk(
                "HTP100自动老化故障\n"
                f"电脑：{socket.gethostname()}\n串口：{self.port_box.currentText()}\n"
                f"时间：{datetime.now():%Y-%m-%d %H:%M:%S}\n原因：{reason}\n"
                f"目标压力：{self.aging_target.value():.1f} mmHg\n"
                f"最高压力：{self.aging_max_pressure:.1f} mmHg\n"
                f"累计：{self.stats['total']}，成功：{self.stats['success']}，失败：{self.stats['failed']}")
        should_continue = success or (self.continue_after_failure.isChecked() and safe_to_continue)
        if self.aging_button.isChecked() and should_continue:
            self.aging_state = "cooldown"
            self.aging_deadline = time.monotonic() + 2.0
            self.aging_status.setText("本轮结束，2秒后开始下一轮")
        else:
            self._stop_aging_ui("自动老化因故障停止" if not success else "自动老化停止")

    def _aging_fail(self, reason: str, count_cycle: bool = True,
                    safe_to_continue: bool = False) -> None:
        if count_cycle:
            self._complete_cycle(False, reason, safe_to_continue)
        else:
            self._log(reason, "ERROR")
            self._send_dingtalk(
                f"HTP100自动老化未能启动\n电脑：{socket.gethostname()}\n"
                f"时间：{datetime.now():%Y-%m-%d %H:%M:%S}\n原因：{reason}")
            self._stop_aging_ui(reason)

    def _aging_tick(self) -> None:
        now = time.monotonic()
        if self.debug_mode_enabled and now - self.last_debug_keepalive >= 5.0:
            self._send_debug_keepalive()
        if self.aging_state == "idle":
            return
        state = self.last_telemetry.get("app_state")
        fault = self.last_telemetry.get("fault", 0)
        if fault and self.aging_state not in ("wait_home", "wait_initial_home"):
            self._send_screen_command(0x1038, 0.0, "故障停止")
            self._aging_fail(f"设备故障 0x{fault:04X}")
            return
        if self.aging_state == "wait_status":
            if state is not None:
                self._start_aging()
            elif now >= self.aging_deadline:
                self._aging_fail("2秒内未收到主控状态", count_cycle=False)
        elif self.aging_state == "wait_initial_home":
            if state == 2 and self.last_telemetry.get("home_valid"):
                self._begin_aging_cycle()
            elif now >= self.aging_deadline:
                self._aging_fail("启动前20秒内未完成回零", count_cycle=False)
        elif self.aging_state == "wait_ready":
            if state in (3, 4) and now - self.aging_prepare_sent >= 0.5:
                if self._send_screen_command(0x1036, 0.0, "自动模式开始"):
                    self.aging_state = "wait_running"
                    self.aging_deadline = now + 5.0
                    self.aging_status.setText("自动预热已进入，等待电机启动")
            elif state == 2 and now - self.aging_prepare_sent >= 3.0:
                self._aging_fail(self._prepare_failure_reason())
            elif now >= self.aging_deadline:
                self._aging_fail("自动预热命令后3秒未进入预热状态")
        elif self.aging_state == "wait_running":
            if state == 5:
                self.aging_state = "running"
                self.aging_run_started = now
                self.aging_status.setText(
                    f"电机向前挤压：0.0 / {self.aging_forward_seconds.value():.1f} 秒")
            elif now >= self.aging_deadline:
                self._aging_fail("开始命令后5秒内电机未进入运行状态")
        elif self.aging_state == "running":
            elapsed = now - self.aging_run_started
            forward_seconds = self.aging_forward_seconds.value()
            self.aging_status.setText(
                f"电机向前挤压：{min(forward_seconds, elapsed):.1f} / "
                f"{forward_seconds:.1f} 秒，最高 {self.aging_max_pressure:.1f} mmHg")
            if state != 5:
                self._aging_fail(f"前进计时内治疗意外中止，状态={state}")
                return
            if elapsed >= forward_seconds:
                tolerance = self.aging_pressure_tolerance.value()
                threshold = max(0.0, self.aging_target.value() - tolerance)
                if self.aging_max_pressure < threshold:
                    self.pending_cycle_failure = (
                        f"前进{forward_seconds:.1f}秒最高压力{self.aging_max_pressure:.1f}，"
                        f"未达到{threshold:.1f} mmHg（允许欠压{tolerance:.1f} mmHg）")
                self._send_screen_command(0x1038, 0.0, "自动模式停止")
                self.aging_state = "wait_home"
                self.aging_deadline = now + self.aging_reverse_seconds.value()
                self.aging_status.setText(
                    f"停止命令已发送，等待回零（最长 {self.aging_reverse_seconds.value():.1f} 秒）")
        elif self.aging_state == "wait_home":
            if now >= self.aging_deadline:
                self._aging_fail(
                    f"停止命令后{self.aging_reverse_seconds.value():.1f}秒未收到主控回零完成事件")
        elif self.aging_state == "cooldown" and now >= self.aging_deadline:
            self._begin_aging_cycle()

    def _prepare_failure_reason(self) -> str:
        data = self.last_telemetry
        if data.get("charging") and not data.get("debug_mode"):
            return "自动准备被充电联锁拒绝"
        if not data.get("home_valid", True):
            return "自动准备被拒绝：回零无效"
        if data.get("eye") not in (1, 2, 4, None):
            return f"自动准备被拒绝：眼盾状态={data.get('eye')}"
        if not data.get("zero_valid"):
            return "自动准备被拒绝：ADS1220零点无效"
        return "自动准备命令后3秒仍停留在待机状态"

    def _queue_ack(self, command: int, payload: bytes, description: str,
                   after=None) -> None:
        self.command_queue.append((command, payload, description, after))
        self._pump_queue()

    def _pump_queue(self) -> None:
        if self.pending is not None or not self.command_queue:
            return
        command, payload, description, after = self.command_queue.popleft()
        sequence = self._send_direct(command, payload, description)
        if sequence is not None:
            self.pending = (sequence, command, description, after)
            self.pending_since = time.monotonic()

    def _command_observed_in_telemetry(self, command: int) -> bool:
        state = self.last_telemetry.get("app_state")
        if command in (CMD_PREPARE, CMD_PREPARE_HEAT):
            return state in (3, 4)
        if command == CMD_START:
            return state == 5
        if command == CMD_STOP:
            return state in (1, 2)
        return False

    def _poll_serial(self) -> None:
        if not self.serial or not self.serial.is_open:
            return
        try:
            # Always drain data already buffered by Windows before judging a
            # timeout.  Otherwise a delayed GUI timer can report a false
            # timeout even though the ACK is already waiting in the driver.
            waiting = self.serial.in_waiting
            if waiting:
                frames, ignored = self.parser.feed(self.serial.read(waiting))
                self.ignored_bytes += ignored
                for frame in frames:
                    self._handle_frame(frame)
        except Exception as error:
            self._log(f"串口异常：{error}", "ERROR")
            self._disconnect()
            return

        if self.pending and time.monotonic() - self.pending_since > 3.0:
            _seq, command, description, after = self.pending
            if self._command_observed_in_telemetry(command):
                self._log(f"未收到ACK，但遥测确认命令已经执行：{description}", "WARN")
                if after:
                    after()
            elif self.last_frame_received < self.pending_since:
                self._log(
                    f"应答超时，执行结果未知：{description}；本次发送后没有收到任何主控数据，"
                    "请检查 PA2(MCU TX)→USB串口RX 接线，禁止重复点击",
                    "ERROR")
            else:
                self._log(
                    f"应答超时，执行结果未知：{description}；禁止按‘失败’重复执行",
                    "ERROR")
            self.pending = None
            self.command_queue.clear()

    def _handle_frame(self, frame) -> None:
        self.last_frame_received = time.monotonic()
        if frame.command == RSP_INFO and len(frame.payload) == 6:
            firmware, version, count = struct.unpack("<IBB", frame.payload)
            self._log(f"握手成功：固件 {firmware}，调参协议 v{version}，{count} 挡")
        elif frame.command == RSP_PROFILE:
            try:
                index, values = decode_profile(frame.payload)
                self._set_profile(index, values)
                self._log(f"收到第 {index + 1} 挡参数")
            except ValueError as error:
                self._log(str(error), "ERROR")
        elif frame.command == RSP_HEAT_PID:
            try:
                values = decode_heat_pid(frame.payload)
                for key, value in values.items():
                    self.heat_pid_widgets[key].setValue(value)
                self._log(
                    f"收到温控 PID：Kp={values['kp']:.3f}，"
                    f"Ki={values['ki']:.3f}，Kd={values['kd']:.3f}")
            except ValueError as error:
                self._log(str(error), "ERROR")
        elif frame.command == RSP_ACK and len(frame.payload) == 2:
            request, status = frame.payload
            text = STATUS_TEXT.get(status, f"未知状态 {status}")
            level = "INFO" if status == 0 else "ERROR"
            matched = (self.pending and frame.sequence == self.pending[0] and
                       request == self.pending[1])
            if request != CMD_DEBUG_CONTROL or matched or status != 0:
                self._log(f"RX 应答 cmd=0x{request:02X}：{text}", level)
            if matched:
                _seq, _cmd, _description, after = self.pending
                self.pending = None
                if status == 0 and after:
                    after()
                if status != 0:
                    self.command_queue.clear()
                    if request == CMD_DEBUG_CONTROL:
                        self._set_debug_ui(False)
                        self._stop_aging_ui("调试模式开启失败")
                self._pump_queue()
        elif frame.command == RSP_TELEMETRY:
            try:
                self._show_telemetry(decode_telemetry(frame.payload))
            except ValueError as error:
                self._log(str(error), "ERROR")
        elif frame.command == RSP_HOME_EVENT:
            try:
                event = decode_home_event(frame.payload)
                self._handle_home_event(event)
            except ValueError as error:
                self._log(str(error), "ERROR")

    def _handle_home_event(self, event: dict) -> None:
        success = bool(event["success"])
        self._log(
            f"RX 主控回零完成事件：{'成功' if success else '失败'}，"
            f"停止原因={event['stop_reason']}，故障=0x{event['fault']:04X}",
            "INFO" if success else "ERROR")
        if self.aging_state == "wait_initial_home":
            if success:
                self._begin_aging_cycle()
            else:
                self._aging_fail(f"启动前回零失败 0x{event['fault']:04X}", count_cycle=False)
        elif self.aging_state == "wait_home":
            if not success:
                self._aging_fail(f"主控报告回零失败 0x{event['fault']:04X}")
            elif getattr(self, "pending_cycle_failure", ""):
                self._complete_cycle(False, self.pending_cycle_failure, safe_to_continue=True)
            else:
                self._complete_cycle(True, "压力达标且主控回零成功")

    def _show_telemetry(self, data: dict) -> None:
        self.last_telemetry = data
        pressure = data["pressure_mmhg"]
        target = data["target_mmhg"]
        self.live_labels["pressure"].setText(f"{pressure:.1f} / {target:.1f} mmHg")
        target_temperature = data.get("target_temperature_c")
        measured_temperature = data.get("temperature_c")
        if data.get("temperature_valid") and measured_temperature is not None:
            self.live_labels["temperature"].setText(
                f"{measured_temperature:.2f} / {target_temperature:.1f} °C")
        elif target_temperature is not None:
            self.live_labels["temperature"].setText(
                f"— / {target_temperature:.1f} °C（传感器暂不可用）")
        else:
            self.live_labels["temperature"].setText("旧固件未提供")
        heat_power = data.get("heat_power_percent")
        heat_integral_output = data.get("heat_integral_output")
        heat_integral_output_percent = (
            heat_integral_output * (100.0 / 254.0)
            if heat_integral_output is not None else None)
        self.live_labels["heat_power"].setText(
            f"{heat_power:.1f} %" if heat_power is not None else "旧固件未提供")
        self.live_labels["heat_integral_output"].setText(
            f"{heat_integral_output:.2f} PWM ({heat_integral_output_percent:.1f} %)"
            if heat_integral_output is not None else "旧固件未提供")
        speed_command = data.get("motor_speed_command")
        pressure_error = data.get("pressure_error")
        self.live_labels["motor_speed"].setText(
            f"{speed_command:+d}" if speed_command is not None else "旧固件未提供")
        self.live_labels["pressure_error"].setText(
            f"{pressure_error:+.1f} mmHg"
            if pressure_error is not None else "旧固件未提供")
        self.live_labels["raw"].setText(str(data["raw"]))
        self.live_labels["zero"].setText(f"{data['zero_raw']}（{'有效' if data['zero_valid'] else '无效'}）")
        self.live_labels["stage"].setText(STAGES.get(data["stage"], str(data["stage"])))
        state = data["app_state"]
        self.live_labels["state"].setText(APP_STATES[state] if state < len(APP_STATES) else str(state))
        eye_names = {0: "未插入", 1: "新眼盾", 2: "本次使用中", 3: "已报废", 4: "维修眼盾"}
        eye = data.get("eye")
        home = data.get("home_valid")
        self.live_labels["eye"].setText(
            f"{eye_names.get(eye, '旧固件未提供')} / 回零{'有效' if home else '无效'}"
            if home is not None else "旧固件未提供")
        debug_mode = data.get("debug_mode")
        self.live_labels["power"].setText(
            f"{'充电' if data['charging'] else '电池'} / "
            f"{'调试模式' if debug_mode else '正常联锁'}")
        if debug_mode is not None and self.debug_mode_enabled != bool(debug_mode):
            was_aging = self.aging_state != "idle"
            self._set_debug_ui(bool(debug_mode))
            if was_aging and not debug_mode:
                self._aging_fail("主控调试模式已退出（通信保活超时）")
        fault = data["fault"]
        self.live_labels["fault"].setText(f"0x{fault:04X}" if fault else "无")
        self.live_labels["fault"].setStyleSheet(
            "font-size:16px;font-weight:600;color:#d23b43" if fault else
            "font-size:16px;font-weight:600;color:#168a49")
        self.plot.add_sample(pressure, target)
        if (data.get("temperature_valid") and measured_temperature is not None and
                target_temperature is not None):
            self.temperature_plot.add_sample(
                measured_temperature, target_temperature, heat_power,
                heat_integral_output_percent)
        if self.aging_state == "running":
            self.aging_max_pressure = max(self.aging_max_pressure, pressure)

    def _read_all(self) -> None:
        self._send_direct(CMD_GET_ALL, b"", "读取全部参数")

    def _read_heat_pid(self) -> None:
        self._send_direct(CMD_GET_HEAT_PID, b"", "读取温控 PID")

    def _heat_pid_payload(self) -> bytes:
        return encode_heat_pid(
            self.heat_pid_widgets["kp"].value(),
            self.heat_pid_widgets["ki"].value(),
            self.heat_pid_widgets["kd"].value())

    def _write_heat_pid(self) -> None:
        if not self.serial or not self.serial.is_open:
            self._log("串口未连接，无法写入温控 PID", "ERROR")
            return
        self._queue_ack(CMD_SET_HEAT_PID, self._heat_pid_payload(),
                        "应用温控 PID 到 RAM", self._read_heat_pid)

    def _start_heat_tuning(self) -> None:
        if not self.serial or not self.serial.is_open:
            self._log("串口未连接，无法开始温控调试", "ERROR")
            return
        target = self.heat_target.value()
        self._queue_ack(CMD_SET_TEMPERATURE, struct.pack("<f", target),
                        f"设置温控目标 {target:.1f} °C")
        self._queue_ack(CMD_SET_HEAT_PID, self._heat_pid_payload(),
                        "应用温控 PID 到 RAM")
        self._queue_ack(CMD_PREPARE_HEAT, b"", "准备加热模式")
        self._queue_ack(CMD_START, b"", "开始温控调试")

    def _save_to_storage(self) -> None:
        if not self.serial or not self.serial.is_open:
            self._log("串口未连接，无法保存到外置 EEPROM", "ERROR")
            return
        answer = QMessageBox.question(
            self, "保存压力参数",
            "先将当前表格的五挡参数全部同步到主控 RAM，\n"
            "再保存到外置 EEPROM？保存成功后 MCU 重启仍会使用这些参数。")
        if answer != QMessageBox.Yes:
            return
        # Replace pending debounce data with an authoritative full-table sync.
        # CMD_SAVE is queued last and therefore cannot run after a rejected
        # field; a failed ACK clears the remaining queue.
        self.field_write_timer.stop()
        self.pending_field_updates.clear()
        if self._queue_entire_table_to_ram():
            self._log("已提交保存任务：同步 5 挡完整参数后写入外置 EEPROM")
            self._queue_ack(CMD_SAVE, b"", "保存五挡参数到外置 EEPROM", self._read_all)

    def _restore_defaults(self) -> None:
        answer = QMessageBox.question(
            self, "恢复默认值",
            "恢复程序内置默认参数到 RAM？\n"
            "如需重启后继续使用默认值，还要点击“同步整表并保存到外置 EEPROM”。")
        if answer != QMessageBox.Yes:
            return
        self._queue_ack(CMD_DEFAULTS, b"", "恢复固件默认值", self._read_all)

    def _prepare(self) -> None:
        target = self.target.value()
        self._queue_ack(CMD_PREPARE, struct.pack("<f", target), f"准备压力 {target:.1f} mmHg")

    def _start(self) -> None:
        self._queue_ack(CMD_START, b"", "开始压力治疗")

    def _stop(self) -> None:
        self.command_queue.clear()
        self.pending = None
        self._queue_ack(CMD_STOP, b"", "停止并回零")

    def _set_telemetry(self, *_args) -> None:
        period = self.period.value() if self.telemetry_check.isChecked() else 0
        self._queue_ack(CMD_TELEMETRY_CONTROL, struct.pack("<H", period),
                        "开启遥测" if period else "关闭遥测")

    def plot_clear(self) -> None:
        self.plot.clear()
        self.temperature_plot.clear()

    def _export_json(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "导出参数", "pressure_profiles.json", "JSON (*.json)")
        if not path:
            return
        data = {
            "format": "HTP100-pressure-profile-v8",
            "profiles": [self._get_profile(row) for row in range(5)],
            "heat_pid": {
                key: widget.value() for key, widget in self.heat_pid_widgets.items()
            },
            "temperature_target_c": self.heat_target.value(),
        }
        try:
            Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            self._log(f"已导出 {path}")
        except Exception as error:
            QMessageBox.critical(self, "导出失败", str(error))

    def _import_json(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "导入参数", "", "JSON (*.json)")
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if data.get("format") != "HTP100-pressure-profile-v8":
                raise ValueError("旧版压力参数含义已变更，请使用 v8 配置或程序默认值")
            profiles = data["profiles"]
            if len(profiles) != 5:
                raise ValueError("配置必须恰好包含五挡参数")
            for row, values in enumerate(profiles):
                missing = [key for key in PROFILE_KEYS if key not in values]
                if missing:
                    raise ValueError(
                        f"第 {row + 1} 挡缺少参数：{', '.join(missing)}")
                self._set_profile(row, values)
            heat_pid = data.get("heat_pid")
            if isinstance(heat_pid, dict):
                for key, widget in self.heat_pid_widgets.items():
                    if key in heat_pid:
                        widget.setValue(float(heat_pid[key]))
            if "temperature_target_c" in data:
                self.heat_target.setValue(float(data["temperature_target_c"]))
            self._log(
                f"已导入 {path} 到表格与温控 PID 控件；压力参数点击保存后写入设备，"
                "温控 PID 点击应用后写入 RAM")
        except Exception as error:
            QMessageBox.critical(self, "导入失败", str(error))

    def closeEvent(self, event) -> None:
        if self.serial and self.serial.is_open and self.telemetry_check.isChecked():
            try:
                self.serial.write(build_frame(CMD_TELEMETRY_CONTROL, self._next_sequence(), b"\x00\x00"))
            except Exception:
                pass
        self._disconnect()
        event.accept()


def main() -> int:
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    # Windows can reuse a position from a disconnected high-DPI monitor even
    # though this application does not persist geometry. Constrain every
    # launch to the current primary screen and center it explicitly.
    app.processEvents()
    screen = app.primaryScreen()
    if screen is not None:
        available = screen.availableGeometry()
        width = min(window.width(), max(900, available.width() - 40))
        height = min(window.height(), max(650, available.height() - 40))
        window.resize(width, height)
        window.move(available.center() - window.rect().center())
    window.raise_()
    window.activateWindow()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
