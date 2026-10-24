"""Live Qt dashboard for CARLA RGB, radar, telemetry, and other sensors."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QFontDatabase, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from collector.live import CarlaCollector, LiveState, SessionRecorder
from monitor.qt.experiment_evaluator import ExperimentEvaluatorThread, METHODS
from monitor.qt.yolo_worker import YoloWorker
from monitor.qt.yolo_overlay_generator import (
    OverlayGenerationSummary,
    OverlayGeneratorThread,
)
from monitor.qt.weather_workflow import WeatherWorkflowDialog
from monitor.qt.traffic_lights import TrafficLightControlThread, TrafficLightResult


class RadarPlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.points: list[dict[str, float]] = []
        self.setMinimumSize(360, 250)

    def set_points(self, points: list[dict[str, float]]) -> None:
        self.points = points
        self.update()

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0a1826"))
        origin_x = self.width() / 2
        origin_y = self.height() - 23
        scale = min((self.height() - 38) / 100.0, (self.width() - 42) / 70.0)
        painter.setPen(QPen(QColor("#29445b"), 1))
        for distance in (25, 50, 75, 100):
            radius = distance * scale
            painter.drawArc(
                int(origin_x - radius),
                int(origin_y - radius),
                int(2 * radius),
                int(2 * radius),
                0,
                180 * 16,
            )
            painter.drawText(int(origin_x + 5), int(origin_y - radius - 4), f"{distance} m")
        painter.setPen(QPen(QColor("#4a718b"), 1))
        for angle in (-17.5, 0, 17.5):
            radians = math.radians(angle)
            painter.drawLine(
                int(origin_x),
                int(origin_y),
                int(origin_x + 100 * scale * math.sin(radians)),
                int(origin_y - 100 * scale * math.cos(radians)),
            )
        for point in self.points:
            distance = point["distance_m"]
            azimuth = point["azimuth_rad"]
            x = origin_x + distance * scale * math.sin(azimuth)
            y = origin_y - distance * scale * math.cos(azimuth)
            if not (0 <= x < self.width() and 0 <= y < self.height()):
                continue
            velocity = point["relative_velocity_mps"]
            color = QColor("#fb6376") if velocity > 0.5 else QColor("#49a9ff") if velocity < -0.5 else QColor("#f5d76e")
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(int(x - 3), int(y - 3), 6, 6)
        painter.setPen(QColor("#d8e9f4"))
        painter.setBrush(QColor("#d8e9f4"))
        painter.drawEllipse(int(origin_x - 5), int(origin_y - 5), 10, 10)


def value_label() -> QLabel:
    label = QLabel("—")
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    label.setWordWrap(True)
    return label


def field(grid: QGridLayout, row: int, title: str) -> QLabel:
    name = QLabel(title)
    name.setObjectName("fieldName")
    result = value_label()
    grid.addWidget(name, row, 0)
    grid.addWidget(result, row, 1)
    return result


class MonitorWindow(QMainWindow):
    camera_frame_ready = Signal()
    recording_finished = Signal(bool)

    def __init__(self, collector: CarlaCollector, recorder: SessionRecorder,
                 yolo_worker: YoloWorker | None = None) -> None:
        super().__init__()
        self.collector = collector
        self.recorder = recorder
        self.yolo_worker = yolo_worker
        self._yolo_enabled_by_record_button = False
        self._tracking_enabled_before_record = False
        self._radar_fusion_enabled_before_record = False
        self._overlay_generator: OverlayGeneratorThread | None = None
        self._experiment_evaluator: ExperimentEvaluatorThread | None = None
        self._recording_active = False
        self._recording_deadline: float | None = None
        self._recording_auto_stop = False
        self._workflow_metadata: dict[str, Any] | None = None
        self._weather_workflow_dialog: WeatherWorkflowDialog | None = None
        self._traffic_light_thread: TrafficLightControlThread | None = None
        self._traffic_light_settings: dict[str, Any] | None = None
        self._video_focus_splitter_sizes: list[int] | None = None
        self.last_camera_frame = -1
        self.last_detection_received_at: tuple[Any, ...] | None = None
        self.last_radar_frame = -1
        self.display_times: list[float] = []
        self._video_update_pending = threading.Event()
        self.setWindowTitle("CARLA RadarVision · Live Sensor Monitor")
        self.resize(1500, 900)
        self._build_ui()
        self._apply_style()
        self.recording_timer = QTimer(self)
        self.recording_timer.setInterval(250)
        self.recording_timer.timeout.connect(self._update_recording_clock)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(200)
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self._refresh_video)
        self.video_timer.start(200)
        self.camera_frame_ready.connect(self._refresh_video, Qt.ConnectionType.QueuedConnection)
        self.collector.on_camera_frame = self._queue_video_refresh
        self.collector.start()
        if self.yolo_worker:
            self.yolo_worker.start()

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(18, 16, 18, 16)
        outer.setSpacing(12)

        header = QHBoxLayout()
        title = QLabel("CARLA RADARVISION")
        title.setObjectName("title")
        header.addWidget(title)
        header.addStretch()
        self.status = QLabel("CARLA 연결 중")
        self.status.setObjectName("status")
        header.addWidget(self.status)
        self.record_button = QPushButton("일반 기록 시작")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self._toggle_recording)
        header.addWidget(self.record_button)
        self.yolo_record_button = QPushButton("YOLO 기록 시작")
        self.yolo_record_button.setCheckable(True)
        self.yolo_record_button.setEnabled(False)
        self.yolo_record_button.toggled.connect(self._toggle_yolo_recording)
        header.addWidget(self.yolo_record_button)
        self.tracking_record_button = QPushButton("YOLO+트래킹 기록 시작")
        self.tracking_record_button.setCheckable(True)
        self.tracking_record_button.setEnabled(False)
        self.tracking_record_button.toggled.connect(self._toggle_tracking_recording)
        header.addWidget(self.tracking_record_button)
        self.radar_fusion_record_button = QPushButton("YOLO+레이더 융합 기록 시작")
        self.radar_fusion_record_button.setCheckable(True)
        self.radar_fusion_record_button.setEnabled(False)
        self.radar_fusion_record_button.toggled.connect(self._toggle_radar_fusion_recording)
        header.addWidget(self.radar_fusion_record_button)
        self.weather_workflow_button = QPushButton("날씨 수집 워크플로")
        self.weather_workflow_button.clicked.connect(self._open_weather_workflow)
        header.addWidget(self.weather_workflow_button)
        outer.addLayout(header)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        self._main_splitter = splitter
        outer.addWidget(splitter, 1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        self.camera_info = QLabel("Frame —  ·  Sim time —  ·  Capture FPS —  ·  Display FPS —")

        detection_controls = QHBoxLayout()
        self.yolo_toggle = QPushButton("실시간 탐지 시작")
        self.yolo_toggle.setCheckable(True)
        self.yolo_toggle.setEnabled(self.yolo_worker is not None)
        if self.yolo_worker is None:
            self.yolo_toggle.setToolTip("--yolo-library와 --yolo-engine 옵션을 지정해 앱을 실행하세요.")
        self.yolo_toggle.toggled.connect(self._toggle_yolo)
        detection_controls.addWidget(self.yolo_toggle)
        self.tracking_toggle = QPushButton("트래킹 시작")
        self.tracking_toggle.setCheckable(True)
        self.tracking_toggle.setEnabled(False)
        self.tracking_toggle.toggled.connect(self._toggle_tracking)
        detection_controls.addWidget(self.tracking_toggle)
        detection_controls.addWidget(QLabel("추적 유지"))
        self.tracking_max_lost_frames = QSpinBox()
        self.tracking_max_lost_frames.setRange(0, 100)
        self.tracking_max_lost_frames.setValue(
            getattr(self.yolo_worker, "max_lost_frames", 10))
        self.tracking_max_lost_frames.setSuffix(" 프레임")
        self.tracking_max_lost_frames.setToolTip(
            "YOLO 검출이 끊긴 뒤 트랙을 예측 상태로 유지할 처리 프레임 수. "
            "변경하면 현재 추적 ID가 초기화됩니다.")
        self.tracking_max_lost_frames.setEnabled(self.yolo_worker is not None)
        self.tracking_max_lost_frames.valueChanged.connect(self._set_tracking_max_lost_frames)
        detection_controls.addWidget(self.tracking_max_lost_frames)
        self.radar_correction_toggle = QPushButton()
        self.radar_correction_toggle.setCheckable(True)
        self.radar_correction_toggle.setChecked(
            getattr(self.yolo_worker, "radar_track_correction_enabled", False))
        self.radar_correction_toggle.setText(
            "레이더 위치 보정 켜짐" if self.radar_correction_toggle.isChecked()
            else "레이더 위치 보정 꺼짐")
        self.radar_correction_toggle.setEnabled(self.yolo_worker is not None)
        self.radar_correction_toggle.setToolTip(
            "실험 기능: YOLO 미검출 중 레이더로 예측 트랙을 보정합니다. "
            "변경하면 레이더 추적 ID가 초기화됩니다.")
        self.radar_correction_toggle.toggled.connect(self._set_radar_track_correction_enabled)
        detection_controls.addWidget(self.radar_correction_toggle)
        self.video_focus_button = QPushButton("영상 크게 보기")
        self.video_focus_button.setCheckable(True)
        self.video_focus_button.setToolTip("분석·계기판 영역을 접어 비교 영상을 세로로 크게 봅니다.")
        self.video_focus_button.toggled.connect(self._toggle_video_focus)
        detection_controls.addWidget(self.video_focus_button)
        self.yolo_config_info = QLabel(
            "YOLO DLL·엔진 옵션 필요" if self.yolo_worker is None else "YOLO11n 준비 중"
        )
        detection_controls.addWidget(self.yolo_config_info, 1)
        left_layout.addLayout(detection_controls)

        detection_group = QGroupBox("비교 화면 A")
        detection_layout = QVBoxLayout(detection_group)
        detection_view_row = QHBoxLayout()
        detection_view_row.addWidget(QLabel("표시할 영상"))
        self.detection_view = QComboBox()
        self._add_video_view_options(self.detection_view, "yolo")
        detection_view_row.addWidget(self.detection_view, 1)
        detection_layout.addLayout(detection_view_row)
        self.detection_camera = QLabel("YOLO11n 미설정")
        self.detection_camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.detection_camera.setMinimumSize(320, 240)
        self.detection_camera.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.detection_camera.setObjectName("camera")
        detection_layout.addWidget(self.detection_camera, 1)
        self.detection_info = QLabel("탐지 대기 중")
        detection_layout.addWidget(self.detection_info)
        tracking_group = QGroupBox("비교 화면 B")
        tracking_layout = QVBoxLayout(tracking_group)
        tracking_view_row = QHBoxLayout()
        tracking_view_row.addWidget(QLabel("표시할 영상"))
        self.tracking_view = QComboBox()
        self._add_video_view_options(self.tracking_view, "tracking")
        tracking_view_row.addWidget(self.tracking_view, 1)
        tracking_layout.addLayout(tracking_view_row)
        self.tracking_camera = QLabel("트래킹 대기 중")
        self.tracking_camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.tracking_camera.setMinimumSize(320, 240)
        self.tracking_camera.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.tracking_camera.setObjectName("camera")
        tracking_layout.addWidget(self.tracking_camera, 1)
        self.tracking_info = QLabel("트래킹 꺼짐")
        tracking_layout.addWidget(self.tracking_info)
        compare_layout = QHBoxLayout()
        compare_layout.addWidget(detection_group, 1)
        compare_layout.addWidget(tracking_group, 1)
        left_layout.addLayout(compare_layout, 1)
        self.generate_boxes_button = QPushButton("저장 세션 전체 비교 영상 생성")
        self.generate_boxes_button.clicked.connect(self._generate_saved_yolo_images)
        left_layout.addWidget(self.generate_boxes_button)
        self.generator_info = QLabel("원본·YOLO·영상 트래킹·레이더 융합을 2×2 비교 영상으로 생성합니다")
        self.generator_info.setWordWrap(True)
        left_layout.addWidget(self.generator_info)
        evaluation_controls = QWidget()
        evaluation_range = QHBoxLayout(evaluation_controls)
        evaluation_range.setContentsMargins(0, 0, 0, 0)
        evaluation_range.addWidget(QLabel("평가 거리"))
        self.evaluation_range = QComboBox()
        for label, bounds in (
            ("전체 거리 (구간별)", (0.0, None, True)),
            ("0~3m", (0.0, 3.0, False)),
            ("3~5m", (3.0, 5.0, False)),
            ("5~10m", (5.0, 10.0, False)),
            ("10~15m", (10.0, 15.0, False)),
            ("3~15m (기존)", (3.0, 15.0, True)),
            ("15~20m", (15.0, 20.0, False)),
            ("20~30m", (20.0, 30.0, False)),
            ("30m 이상", (30.0, None, True)),
            ("사용자 지정", None),
        ):
            self.evaluation_range.addItem(label, bounds)
        evaluation_range.addWidget(self.evaluation_range)
        self.evaluation_min = QDoubleSpinBox()
        self.evaluation_max = QDoubleSpinBox()
        for distance_input in (self.evaluation_min, self.evaluation_max):
            distance_input.setRange(0.0, 1000.0)
            distance_input.setDecimals(1)
            distance_input.setSuffix(" m")
            distance_input.setEnabled(False)
            evaluation_range.addWidget(distance_input)
        self.evaluation_min.setValue(3.0)
        self.evaluation_max.setValue(15.0)
        self.evaluation_range.currentIndexChanged.connect(self._evaluation_range_changed)
        left_layout.addWidget(evaluation_controls)
        self.evaluate_button = QPushButton("저장 세션/날씨 워크플로 결과 생성")
        self.evaluate_button.clicked.connect(self._evaluate_saved_session)
        left_layout.addWidget(self.evaluate_button)
        self.evaluation_info = QLabel("실제 관측된 차량의 거리 오차를 구간별로 비교하고 예측 트랙은 별도로 집계합니다.")
        self.evaluation_info.setWordWrap(True)
        left_layout.addWidget(self.evaluation_info)
        left_layout.addWidget(self.camera_info)

        vehicle_group = QGroupBox("VEHICLE · 위치와 이동 상태")
        vehicle_grid = QGridLayout(vehicle_group)
        self.vehicle_fields = {
            "position": field(vehicle_grid, 0, "위치 (m)"),
            "speed": field(vehicle_grid, 1, "속도"),
            "yaw": field(vehicle_grid, 2, "방향 (yaw)"),
            "velocity": field(vehicle_grid, 3, "속도 벡터 (m/s)"),
            "frame": field(vehicle_grid, 4, "프레임 / 시뮬레이션 시간"),
        }
        left_layout.addWidget(vehicle_group)
        self._video_detail_widgets = (
            self.generate_boxes_button, self.generator_info, evaluation_controls,
            self.evaluate_button, self.evaluation_info, self.camera_info, vehicle_group,
        )
        splitter.addWidget(left)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(430)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 6, 0)
        right_layout.setSpacing(10)
        scroll.setWidget(right)
        splitter.addWidget(scroll)
        self._telemetry_scroll = scroll
        splitter.setSizes([940, 540])

        radar_group = QGroupBox("RADAR · 거리 / 각도 / 상대속도")
        radar_layout = QVBoxLayout(radar_group)
        self.radar_plot = RadarPlot()
        radar_layout.addWidget(self.radar_plot)
        self.radar_info = QLabel("Frame —  ·  Points —  ·  FPS —")
        radar_layout.addWidget(self.radar_info)
        self.radar_table = QTableWidget(0, 4)
        self.radar_table.setHorizontalHeaderLabels(["거리 m", "방위각 °", "고도각 °", "상대속도 m/s"])
        self.radar_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.radar_table.verticalHeader().setVisible(False)
        self.radar_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.radar_table.setMinimumHeight(190)
        radar_layout.addWidget(self.radar_table)
        right_layout.addWidget(radar_group)

        accuracy_group = QGroupBox("POSITION · 위치 추정 정확도")
        accuracy_layout = QVBoxLayout(accuracy_group)
        self.position_accuracy_info = QLabel("트래킹을 켜면 추정 위치와 CARLA 기준 위치를 비교합니다")
        self.position_accuracy_info.setWordWrap(True)
        self.position_accuracy_info.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        accuracy_layout.addWidget(self.position_accuracy_info)
        right_layout.addWidget(accuracy_group)

        imu_group = QGroupBox("IMU · 가속도 / 각속도 / 나침반")
        imu_grid = QGridLayout(imu_group)
        self.imu_fields = {
            "accel": field(imu_grid, 0, "가속도 (m/s²)"),
            "gyro": field(imu_grid, 1, "각속도 (rad/s)"),
            "compass": field(imu_grid, 2, "나침반"),
            "frame": field(imu_grid, 3, "프레임 / FPS"),
        }
        right_layout.addWidget(imu_group)

        gnss_group = QGroupBox("GNSS · 위도 / 경도 / 고도")
        gnss_grid = QGridLayout(gnss_group)
        self.gnss_fields = {
            "latitude": field(gnss_grid, 0, "위도"),
            "longitude": field(gnss_grid, 1, "경도"),
            "altitude": field(gnss_grid, 2, "고도 (m)"),
            "frame": field(gnss_grid, 3, "프레임 / FPS"),
        }
        right_layout.addWidget(gnss_group)

        event_group = QGroupBox("EVENTS · 충돌 / 차선 이탈")
        event_layout = QVBoxLayout(event_group)
        self.events = QLabel("감지된 이벤트 없음")
        self.events.setWordWrap(True)
        self.events.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        event_layout.addWidget(self.events)
        right_layout.addWidget(event_group)

        traffic_group = QGroupBox("TRAFFIC LIGHTS · 신호등")
        traffic_layout = QVBoxLayout(traffic_group)
        traffic_row = QHBoxLayout()
        self.use_red_lights = QCheckBox("빨간불 사용 (자동 신호 순환)")
        self.use_red_lights.toggled.connect(self._set_red_duration_enabled)
        traffic_row.addWidget(self.use_red_lights)
        self.red_light_seconds = QSpinBox()
        self.red_light_seconds.setRange(1, 300)
        self.red_light_seconds.setValue(15)
        self.red_light_seconds.setSuffix("초")
        self.red_light_seconds.setEnabled(False)
        self.red_light_seconds.setToolTip(
            "한 방향의 빨간 대기 목표 시간입니다. 초록 10초·노랑 3초는 유지하며 "
            "교차로 구조상 불가능하면 최소 대기 시간을 적용합니다.")
        traffic_row.addWidget(self.red_light_seconds)
        traffic_layout.addLayout(traffic_row)
        self.apply_traffic_lights_button = QPushButton("신호등 설정 적용")
        self.apply_traffic_lights_button.clicked.connect(self._apply_traffic_light_settings)
        traffic_layout.addWidget(self.apply_traffic_lights_button)
        self.traffic_light_info = QLabel("미적용 · 끄면 모든 신호등을 초록불로 고정합니다.")
        self.traffic_light_info.setWordWrap(True)
        traffic_layout.addWidget(self.traffic_light_info)
        right_layout.addWidget(traffic_group)

        record_group = QGroupBox("RECORDING · 수집 데이터 저장")
        record_layout = QVBoxLayout(record_group)
        duration_row = QHBoxLayout()
        duration_row.addWidget(QLabel("기록 시간"))
        self.record_minutes = QSpinBox()
        self.record_minutes.setRange(0, 120)
        self.record_minutes.setSuffix("분")
        self.record_minutes.setToolTip("0분 0초는 수동 기록입니다.")
        duration_row.addWidget(self.record_minutes)
        self.record_seconds = QSpinBox()
        self.record_seconds.setRange(0, 59)
        self.record_seconds.setSuffix("초")
        self.record_seconds.setToolTip("0분 0초는 수동 기록입니다.")
        duration_row.addWidget(self.record_seconds)
        record_layout.addLayout(duration_row)
        self.recording_timer_info = QLabel("0분 0초: 중지 버튼을 누를 때까지 수동 기록")
        record_layout.addWidget(self.recording_timer_info)
        self.record_minutes.valueChanged.connect(self._update_recording_duration_hint)
        self.record_seconds.valueChanged.connect(self._update_recording_duration_hint)
        self.record_info = QLabel("기록 대기 중")
        self.record_info.setWordWrap(True)
        record_layout.addWidget(self.record_info)
        right_layout.addWidget(record_group)
        right_layout.addStretch()

    @staticmethod
    def _add_video_view_options(combo: QComboBox, default: str) -> None:
        for label, value in (
            ("원본", "original"),
            ("YOLO 검출", "yolo"),
            ("트래킹", "tracking"),
            ("레이더 융합 트래킹", "radar_fusion"),
        ):
            combo.addItem(label, value)
        index = combo.findData(default)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #0b1320; color: #e9f0f7; font-size: 13px; }
            QGroupBox { border: 1px solid #2b4055; border-radius: 8px; margin-top: 13px;
                        padding-top: 10px; font-weight: bold; color: #88c9ff; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
            QLabel#title { color: #f4f8fc; font-size: 22px; font-weight: 800; letter-spacing: 2px; }
            QLabel#status { color: #65d6b2; font-weight: bold; margin-right: 15px; }
            QLabel#fieldName { color: #8fa5b9; min-width: 158px; }
            QLabel#camera { background: #07101c; border: 1px solid #2b4055; border-radius: 5px; }
            QPushButton { background: #173e57; border: 1px solid #396688; border-radius: 6px;
                          padding: 8px 14px; font-weight: bold; }
            QPushButton:hover { background: #205575; }
            QTableWidget { background: #0a1826; gridline-color: #29445b; border: 1px solid #29445b; }
            QHeaderView::section { background: #183047; color: #d7e9f4; border: 0; padding: 4px; }
            QScrollArea { border: 0; }
            """
        )

    def _set_checked(self, button: QPushButton, checked: bool) -> None:
        button.blockSignals(True)
        button.setChecked(checked)
        button.blockSignals(False)

    @staticmethod
    def _render_video_view(frame: Any, detections: Any, view: str) -> QImage:
        image = QImage(
            frame.bgra, frame.width, frame.height,
            frame.width * 4, QImage.Format.Format_ARGB32,
        )
        if detections is None or view == "original":
            return image
        image = image.copy()
        painter = QPainter(image)
        if view == "yolo":
            painter.setPen(QPen(QColor("#4ee38b"), 2))
            for box in detections.boxes:
                painter.drawRect(box.x, box.y, box.width, box.height)
                painter.drawText(box.x, max(14, box.y - 4), f"C{box.class_id} {box.confidence:.2f}")
        elif view in ("tracking", "radar_fusion"):
            objects = {
                int(item["track_id"]): item
                for item in (detections.positioning or {}).get("objects", [])
            }
            view_tracks = detections.vision_tracks if view == "tracking" else detections.tracks
            for track in view_tracks:
                estimate = objects.get(int(track.track_id), {})
                radar_used = estimate.get("distance_source") == "radar_near_cluster"
                color = (QColor("#51c9ff") if track.radar_corrected else
                         QColor("#51c9ff") if radar_used else QColor("#f5d76e")) \
                    if view == "radar_fusion" else (
                        QColor("#f5d76e") if track.predicted else QColor("#4ee38b")
                    )
                pen = QPen(color, 2)
                if track.predicted:
                    pen.setStyle(Qt.PenStyle.DashLine)
                painter.setPen(pen)
                painter.drawRect(track.x, track.y, track.width, track.height)
                if view == "radar_fusion":
                    distance = estimate.get("estimated_distance_m")
                    source = "레이더" if radar_used else "카메라"
                    label = (f"ID {track.track_id} · 거리 {distance:.1f}m ({source})"
                             if distance is not None else f"ID {track.track_id} · 거리 측정 대기")
                else:
                    label = f"ID {track.track_id}  C{track.class_id} {track.confidence:.2f}"
                if track.predicted:
                    label += f" · 누락 {track.missed_frames}"
                    if view == "radar_fusion" and track.radar_corrected:
                        label += " · 레이더 보정"
                painter.drawText(track.x, max(14, track.y - 4), label)
        painter.end()
        return image

    @staticmethod
    def _set_video_label(label: QLabel, pixmap: QPixmap) -> None:
        label.setPixmap(pixmap.scaled(
            label.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        ))

    def _recording_duration_seconds(self) -> int:
        return self.record_minutes.value() * 60 + self.record_seconds.value()

    def _recording_weather(self) -> dict[str, Any] | None:
        if self._workflow_metadata is not None:
            return self._workflow_metadata["weather_parameters"]
        state = getattr(self.collector, "state", None)
        return state.snapshot().get("weather") if state is not None else None

    def _set_red_duration_enabled(self, enabled: bool) -> None:
        self.red_light_seconds.setEnabled(enabled)

    def _apply_traffic_light_settings(self) -> None:
        if self._traffic_light_thread is not None:
            return
        use_red = self.use_red_lights.isChecked()
        red_seconds = float(self.red_light_seconds.value())
        thread = TrafficLightControlThread(
            self.collector.host, self.collector.port, use_red, red_seconds, self)
        self._traffic_light_thread = thread
        self.apply_traffic_lights_button.setEnabled(False)
        self.traffic_light_info.setText("CARLA 신호등 설정 적용 중…")
        thread.applied.connect(self._traffic_light_applied)
        thread.failed.connect(self._traffic_light_failed)
        thread.finished.connect(self._traffic_light_finished)
        thread.start()

    def _traffic_light_applied(self, result: TrafficLightResult,
                               use_red: bool, red_seconds: float) -> None:
        self._traffic_light_settings = {
            "red_enabled": use_red,
            "red_seconds": red_seconds if use_red else None,
            "expected_red_min_seconds": result.expected_red_min_seconds,
            "expected_red_max_seconds": result.expected_red_max_seconds,
            "green_seconds": 10.0 if use_red else None,
            "yellow_seconds": 3.0 if use_red else None,
            "traffic_light_count": result.light_count,
        }
        if use_red:
            minimum = result.expected_red_min_seconds
            maximum = result.expected_red_max_seconds
            if minimum is None or maximum is None:
                raise RuntimeError("Missing traffic-light timing result")
            actual = (f"{minimum:g}초" if abs(minimum - maximum) < 0.01
                      else f"{minimum:g}~{maximum:g}초")
            self.traffic_light_info.setText(
                f"신호등 {result.light_count}개: 초록 10초·노랑 3초, "
                f"빨간불 요청 {red_seconds:g}초 → 예상 대기 {actual}")
        else:
            self.traffic_light_info.setText(f"신호등 {result.light_count}개: 초록불 고정")

    def _traffic_light_failed(self, message: str) -> None:
        self.traffic_light_info.setText(f"신호등 설정 실패: {message}")

    def _traffic_light_finished(self) -> None:
        thread = self._traffic_light_thread
        self._traffic_light_thread = None
        self.apply_traffic_lights_button.setEnabled(True)
        if thread is not None:
            thread.deleteLater()

    def _open_weather_workflow(self) -> None:
        if self._weather_workflow_dialog is None:
            self._weather_workflow_dialog = WeatherWorkflowDialog(self)
        self._weather_workflow_dialog.show()
        self._weather_workflow_dialog.raise_()
        self._weather_workflow_dialog.activateWindow()

    def start_workflow_recording(self, mode: str, duration_seconds: int,
                                 metadata: dict[str, Any]) -> bool:
        if self.recorder.is_enabled() or not 1 <= duration_seconds <= 7259:
            return False
        button = {
            "standard": self.record_button,
            "yolo": self.yolo_record_button,
            "tracking": self.tracking_record_button,
            "radar_fusion": self.radar_fusion_record_button,
        }.get(mode)
        if button is None or not button.isEnabled():
            return False
        minutes, seconds = divmod(duration_seconds, 60)
        self.record_minutes.setValue(minutes)
        self.record_seconds.setValue(seconds)
        self._workflow_metadata = metadata
        try:
            button.click()
            return self.recorder.is_enabled() and self.recorder.mode == mode
        finally:
            self._workflow_metadata = None

    def stop_workflow_recording(self) -> None:
        button = {
            "standard": self.record_button,
            "yolo": self.yolo_record_button,
            "tracking": self.tracking_record_button,
            "radar_fusion": self.radar_fusion_record_button,
        }.get(self.recorder.mode)
        if button is not None and button.isChecked():
            button.setChecked(False)

    def _update_recording_duration_hint(self) -> None:
        if self._recording_active:
            return
        duration = self._recording_duration_seconds()
        if duration:
            minutes, seconds = divmod(duration, 60)
            self.recording_timer_info.setText(
                f"{minutes}분 {seconds}초 기록 후 자동 중지")
        else:
            self.recording_timer_info.setText("0분 0초: 중지 버튼을 누를 때까지 수동 기록")

    def _start_recording_clock(self) -> None:
        duration = self._recording_duration_seconds()
        self._recording_active = True
        self._recording_deadline = time.monotonic() + duration if duration else None
        self.record_minutes.setEnabled(False)
        self.record_seconds.setEnabled(False)
        self.tracking_max_lost_frames.setEnabled(False)
        self.radar_correction_toggle.setEnabled(False)
        if duration:
            self.recording_timer.start()
            self._update_recording_clock()
        else:
            self.recording_timer_info.setText("수동 기록 중 · 중지 버튼으로 종료")

    def _finish_recording_clock(self, reason: str | None = None) -> None:
        if not self._recording_active:
            return
        self.recording_timer.stop()
        self._recording_active = False
        self._recording_deadline = None
        self.record_minutes.setEnabled(True)
        self.record_seconds.setEnabled(True)
        self.tracking_max_lost_frames.setEnabled(self.yolo_worker is not None)
        self.radar_correction_toggle.setEnabled(self.yolo_worker is not None)
        self.recording_timer_info.setText(
            reason or ("지정한 시간이 지나 자동으로 기록을 마쳤습니다."
                       if self._recording_auto_stop else "기록을 중지했습니다.")
        )
        self.recording_finished.emit(self._recording_auto_stop)

    def _update_recording_clock(self) -> None:
        if not self._recording_active or self._recording_deadline is None:
            return
        if not self.recorder.is_enabled():
            self._finish_recording_clock("기록을 중지했습니다.")
            return
        remaining = math.ceil(self._recording_deadline - time.monotonic())
        if remaining > 0:
            minutes, seconds = divmod(remaining, 60)
            self.recording_timer_info.setText(f"자동 중지까지 {minutes:02d}:{seconds:02d} 남음")
            return
        button = {
            "standard": self.record_button,
            "yolo": self.yolo_record_button,
            "tracking": self.tracking_record_button,
            "radar_fusion": self.radar_fusion_record_button,
        }.get(self.recorder.mode)
        self._recording_auto_stop = True
        try:
            if button is not None and button.isChecked():
                button.setChecked(False)
            else:
                self.recorder.stop_session()
                self._finish_recording_clock()
        finally:
            self._recording_auto_stop = False

    def _toggle_recording(self, enabled: bool) -> None:
        if enabled:
            try:
                self.recorder.start_session(
                    "standard", planned_duration_seconds=self._recording_duration_seconds() or None,
                    weather=self._recording_weather(),
                    traffic_lights=self._traffic_light_settings,
                    **({"weather_workflow": self._workflow_metadata}
                       if self._workflow_metadata is not None else {}))
            except Exception as exc:
                self._set_checked(self.record_button, False)
                self.record_info.setText(f"기록 시작 실패: {exc}")
                return
            self.record_button.setText("일반 기록 중지")
            self.yolo_record_button.setEnabled(False)
            self.tracking_record_button.setEnabled(False)
            self.radar_fusion_record_button.setEnabled(False)
            self._start_recording_clock()
        else:
            self.recorder.stop_session()
            self._finish_recording_clock()
            self.record_button.setText("일반 기록 시작")
            self.yolo_record_button.setEnabled(self._yolo_ready())
            self.tracking_record_button.setEnabled(self._yolo_ready())
            self.radar_fusion_record_button.setEnabled(self._yolo_ready())

    def _yolo_ready(self) -> bool:
        return bool(
            self.yolo_worker
            and not self.yolo_worker.status.startswith("YOLO11n 오류")
            and self.yolo_worker.status.startswith((
                "YOLO11n 준비됨",
                "YOLO11n 탐지 중",
                "YOLO11n 탐지 꺼짐",
            ))
        )

    def _toggle_yolo_recording(self, enabled: bool) -> None:
        if enabled:
            if not self._start_ai_recording("yolo"):
                self._set_checked(self.yolo_record_button, False)
                return
            self.yolo_record_button.setText("YOLO 기록 중지")
        else:
            if self.recorder.mode == "yolo":
                self._stop_ai_recording()
            self.yolo_record_button.setText("YOLO 기록 시작")

    def _toggle_tracking_recording(self, enabled: bool) -> None:
        if enabled:
            if not self._start_ai_recording("tracking"):
                self._set_checked(self.tracking_record_button, False)
                return
            self.tracking_record_button.setText("YOLO+트래킹 기록 중지")
        else:
            if self.recorder.mode == "tracking":
                self._stop_ai_recording()
            self.tracking_record_button.setText("YOLO+트래킹 기록 시작")

    def _toggle_radar_fusion_recording(self, enabled: bool) -> None:
        if enabled:
            if not self._start_ai_recording("radar_fusion"):
                self._set_checked(self.radar_fusion_record_button, False)
                return
            self.radar_fusion_record_button.setText("레이더 융합 기록 중지")
        else:
            if self.recorder.mode == "radar_fusion":
                self._stop_ai_recording()
            self.radar_fusion_record_button.setText("YOLO+레이더 융합 기록 시작")

    def _start_ai_recording(self, mode: str) -> bool:
        if not self.yolo_worker or self.recorder.is_enabled() or not self._yolo_ready():
            return False
        self._yolo_enabled_by_record_button = not self.yolo_worker.enabled
        self._tracking_enabled_before_record = self.yolo_worker.tracking_enabled
        self._radar_fusion_enabled_before_record = self.yolo_worker.radar_fusion_enabled
        try:
            self.recorder.start_session(
                mode, self.yolo_worker.engine_path,
                planned_duration_seconds=self._recording_duration_seconds() or None,
                weather=self._recording_weather(),
                traffic_lights=self._traffic_light_settings,
                **({"tracking_max_lost_frames": self.tracking_max_lost_frames.value()}
                   if mode in ("tracking", "radar_fusion") else {}),
                **({"radar_track_correction_enabled": self.radar_correction_toggle.isChecked()}
                   if mode == "radar_fusion" else {}),
                **({"weather_workflow": self._workflow_metadata}
                   if self._workflow_metadata is not None else {}))
        except Exception as exc:
            self._yolo_enabled_by_record_button = False
            self._tracking_enabled_before_record = False
            self._radar_fusion_enabled_before_record = False
            self.record_info.setText(f"AI 기록 시작 실패: {exc}")
            return False
        self.yolo_worker.set_enabled(True)
        tracking_enabled = mode in ("tracking", "radar_fusion")
        if tracking_enabled:
            self.yolo_worker.set_radar_fusion_enabled(mode == "radar_fusion")
        self.yolo_worker.set_tracking_enabled(tracking_enabled)
        self._set_checked(self.yolo_toggle, True)
        self.yolo_toggle.setText("실시간 탐지 중지")
        self._set_checked(self.tracking_toggle, tracking_enabled)
        self.tracking_toggle.setText("트래킹 중지" if tracking_enabled else "트래킹 시작")
        self.record_button.setEnabled(False)
        self.yolo_record_button.setEnabled(mode == "yolo")
        self.tracking_record_button.setEnabled(mode == "tracking")
        self.radar_fusion_record_button.setEnabled(mode == "radar_fusion")
        self.yolo_toggle.setEnabled(False)
        self.tracking_toggle.setEnabled(False)
        self._start_recording_clock()
        return True

    def _stop_ai_recording(self) -> None:
        self.recorder.stop_session()
        self._finish_recording_clock()
        if self.yolo_worker:
            restore_tracking = self._tracking_enabled_before_record and not self._yolo_enabled_by_record_button
            self.yolo_worker.set_radar_fusion_enabled(self._radar_fusion_enabled_before_record)
            self.yolo_worker.set_tracking_enabled(restore_tracking)
            self._set_checked(self.tracking_toggle, restore_tracking)
            self.tracking_toggle.setText("트래킹 중지" if restore_tracking else "트래킹 시작")
        if self._yolo_enabled_by_record_button and self.yolo_worker:
            self.yolo_worker.set_enabled(False)
            self._set_checked(self.yolo_toggle, False)
            self.yolo_toggle.setText("실시간 탐지 시작")
            self.detection_camera.clear()
            self.tracking_camera.clear()
            self.detection_info.setText("탐지 꺼짐")
            self.tracking_info.setText("트래킹 꺼짐")
        self._yolo_enabled_by_record_button = False
        self._tracking_enabled_before_record = False
        self._radar_fusion_enabled_before_record = False
        self.record_button.setEnabled(True)
        self.yolo_record_button.setEnabled(self._yolo_ready())
        self.tracking_record_button.setEnabled(self._yolo_ready())
        self.radar_fusion_record_button.setEnabled(self._yolo_ready())
        self.yolo_toggle.setEnabled(self._yolo_ready())
        self.tracking_toggle.setEnabled(bool(self.yolo_worker and self.yolo_worker.enabled))

    def _toggle_yolo(self, enabled: bool) -> None:
        if not self.yolo_worker:
            return
        if not enabled and self.recorder.mode in ("yolo", "tracking", "radar_fusion"):
            self._set_checked(self.yolo_record_button, False)
            self._set_checked(self.tracking_record_button, False)
            self._set_checked(self.radar_fusion_record_button, False)
            self._stop_ai_recording()
        self.yolo_worker.set_enabled(enabled)
        self.yolo_toggle.setText("실시간 탐지 중지" if enabled else "실시간 탐지 시작")
        if not enabled:
            self.last_detection_received_at = None
            self.yolo_worker.set_tracking_enabled(False)
            self._set_checked(self.tracking_toggle, False)
            self.tracking_toggle.setText("트래킹 시작")
            self.tracking_toggle.setEnabled(False)
        else:
            self.tracking_toggle.setEnabled(True)

    def _toggle_tracking(self, enabled: bool) -> None:
        if not self.yolo_worker or not self.yolo_worker.enabled:
            self._set_checked(self.tracking_toggle, False)
            return
        self.yolo_worker.set_tracking_enabled(enabled)
        self.tracking_toggle.setText("트래킹 중지" if enabled else "트래킹 시작")

    def _set_tracking_max_lost_frames(self, frames: int) -> None:
        if self.yolo_worker is not None:
            self.yolo_worker.set_max_lost_frames(frames)

    def _set_radar_track_correction_enabled(self, enabled: bool) -> None:
        if self.yolo_worker is not None:
            self.yolo_worker.set_radar_track_correction_enabled(enabled)
        self.radar_correction_toggle.setText(
            "레이더 위치 보정 켜짐" if enabled else "레이더 위치 보정 꺼짐")

    def _toggle_video_focus(self, enabled: bool) -> None:
        if enabled:
            self._video_focus_splitter_sizes = self._main_splitter.sizes()
        for widget in self._video_detail_widgets:
            widget.setVisible(not enabled)
        self._telemetry_scroll.setVisible(not enabled)
        if not enabled and self._video_focus_splitter_sizes is not None:
            self._main_splitter.setSizes(self._video_focus_splitter_sizes)
            self._video_focus_splitter_sizes = None
        self.video_focus_button.setText("기본 화면으로" if enabled else "영상 크게 보기")
        self.last_detection_received_at = None

    def _generate_saved_yolo_images(self) -> None:
        if self._overlay_generator is not None and self._overlay_generator.isRunning():
            return
        recordings_dir = Path("data/recordings").resolve()
        selected = QFileDialog.getExistingDirectory(
            self,
            "비교 영상을 만들 기록 세션 선택",
            str(recordings_dir if recordings_dir.exists() else Path.cwd()),
        )
        if not selected:
            return
        session_dir = Path(selected)
        active_session = self.recorder.stats()["path"]
        if (
            self.recorder.is_enabled()
            and active_session is not None
            and session_dir.resolve() == active_session.resolve()
        ):
            self.generator_info.setText("기록 중인 세션입니다. 기록을 중지한 뒤 비교 영상을 생성하세요.")
            return
        self.generate_boxes_button.setEnabled(False)
        self.generator_info.setText(f"오버레이 이미지와 비교 영상 생성 중: {session_dir}")
        worker = OverlayGeneratorThread(session_dir, parent=self)
        self._overlay_generator = worker
        worker.completed.connect(self._overlay_generation_completed)
        worker.failed.connect(self._overlay_generation_failed)
        worker.finished.connect(self._overlay_generation_thread_finished)
        worker.start()

    def _overlay_generation_completed(self, summary: OverlayGenerationSummary) -> None:
        self.generator_info.setText(
            f"탐지 이미지 {summary.generated_images}장 · 트래킹 이미지 {summary.generated_tracking_images}장 · "
            f"레이더 융합 이미지 {summary.generated_radar_fusion_images}장 · "
            f"2×2 비교 영상 {summary.video_frames}프레임 · "
            f"원본 누락 {summary.missing_images}건 · "
            f"잘못된 기록 {summary.invalid_records}건\n영상: {summary.comparison_video}\n저장 위치: {summary.output_dir}"
        )

    def _overlay_generation_failed(self, error: str) -> None:
        self.generator_info.setText(f"생성 실패: {error}")

    def _overlay_generation_thread_finished(self) -> None:
        self.generate_boxes_button.setEnabled(True)
        self._overlay_generator = None

    def _evaluation_range_changed(self, _index: int) -> None:
        custom = self.evaluation_range.currentData() is None
        self.evaluation_min.setEnabled(custom)
        self.evaluation_max.setEnabled(custom)

    def _evaluate_saved_session(self) -> None:
        if self._experiment_evaluator is not None and self._experiment_evaluator.isRunning():
            return
        bounds = self.evaluation_range.currentData()
        min_distance, max_distance, include_max = (
            (self.evaluation_min.value(), self.evaluation_max.value(), True)
            if bounds is None else bounds
        )
        if max_distance is not None and max_distance <= min_distance:
            self.evaluation_info.setText("최대 거리는 최소 거리보다 커야 합니다.")
            return
        recordings_dir = Path("data/recordings").resolve()
        selected = QFileDialog.getExistingDirectory(
            self,
            "실험 결과를 만들 세션 또는 날씨 워크플로 상위 폴더 선택",
            str(recordings_dir if recordings_dir.exists() else Path.cwd()),
        )
        if not selected:
            return
        session_dir = Path(selected)
        active_session = self.recorder.stats()["path"]
        if (self.recorder.is_enabled() and active_session is not None
                and active_session.resolve().is_relative_to(session_dir.resolve())):
            self.evaluation_info.setText("기록 중인 세션입니다. 기록을 중지한 뒤 결과를 생성하세요.")
            return
        self.evaluate_button.setEnabled(False)
        self.evaluation_info.setText(f"거리별 실험 결과 계산 중: {session_dir}")
        worker = ExperimentEvaluatorThread(session_dir, min_distance, max_distance,
                                           include_max, parent=self)
        self._experiment_evaluator = worker
        worker.completed.connect(self._evaluation_completed)
        worker.failed.connect(self._evaluation_failed)
        worker.finished.connect(self._evaluation_thread_finished)
        worker.start()

    def _evaluation_completed(self, summary: dict[str, Any]) -> None:
        if summary.get("kind") == "weather_workflow":
            lines = [f"날씨 {summary['weather_count']}종 · 세션 {summary['session_count']}개 · "
                     f"평가 프레임 {summary['frames_scored']}개"]
            for preset, weather in summary["weather_results"].items():
                stats = weather["methods"]["radar_tracking_fusion"]
                mae = f"{stats['mae_m']:.2f}m" if stats["mae_m"] is not None else "측정 없음"
                lines.append(f"{preset}: 관측+예측 거리 {stats['distance_samples']}건 · MAE {mae}")
            if summary["skipped_sessions"]:
                lines.append(f"제외 세션 {len(summary['skipped_sessions'])}개 · 보고서에서 사유 확인")
            lines.append(f"통합 요약 보고서: {summary['report_path']}")
            lines.append(f"날씨별 상세 통합 보고서: {summary['detailed_report_path']}")
            lines.append(f"날씨별 보고서: {Path(summary['report_path']).parent}")
            self.evaluation_info.setText("\n".join(lines))
            return
        distance_filter = summary["distance_filter_m"]
        high = distance_filter["max_m"]
        range_label = (f"{distance_filter['min_inclusive']:g}m 이상"
                       + (f" {high:g}m {'이하' if distance_filter['include_max'] else '미만'}"
                          if high is not None else ""))
        lines = [f"GT 차체 간격 {range_label} · {summary['frames_scored']}프레임 · 투영 GT {summary['eligible_gt_opportunities']}건(가림 미판정)"]
        for method, label in METHODS.items():
            result = summary["methods"][method]
            mae = f"{result['mae_m']:.2f}m" if result["mae_m"] is not None else "측정 없음"
            lines.append(f"{label}: 최종 거리 MAE {mae}, 관측 {result['observed_distance_samples']}건 + 예측 {result['predicted_distance_samples']}건 = 전체 {result['distance_samples']}건")
        lines.append("거리 구간별 결과와 3~15m 전체 추적 기준 결과는 보고서에 포함됩니다.")
        lines.append(f"결과 보고서: {summary['report_path']}")
        lines.append(f"요약: {summary['summary_path']}")
        lines.append(f"상세 CSV: {summary['csv_path']}")
        self.evaluation_info.setText("\n".join(lines))

    def _evaluation_failed(self, error: str) -> None:
        self.evaluation_info.setText(f"실험 결과 생성 실패: {error}")

    def _evaluation_thread_finished(self) -> None:
        self.evaluate_button.setEnabled(True)
        self._experiment_evaluator = None

    @staticmethod
    def _xyz(values: dict[str, float]) -> str:
        return f"X {values['x']:.2f}   Y {values['y']:.2f}   Z {values['z']:.2f}"

    def _queue_video_refresh(self) -> None:
        if not self._video_update_pending.is_set():
            self._video_update_pending.set()
            self.camera_frame_ready.emit()

    def _refresh_video(self) -> None:
        self._video_update_pending.clear()
        snapshot = self.collector.state.snapshot()
        camera = snapshot["camera"]
        if camera is None:
            if self.last_camera_frame == -1:
                return
            self.camera_info.setText("Frame —  ·  Sim time —  ·  Capture FPS —  ·  Display FPS —")
            self.last_camera_frame = -1
            self.last_detection_received_at = None
            self.display_times.clear()
            return

        detections = snapshot["detections"]
        display_frame = detections.camera if detections is not None else camera
        tracking_enabled = bool(self.yolo_worker and self.yolo_worker.tracking_enabled)
        left_view = str(self.detection_view.currentData())
        right_view = str(self.tracking_view.currentData())
        display_key = (
            display_frame.received_at, detections is not None,
            tracking_enabled, left_view, right_view,
        )
        if display_key == self.last_detection_received_at:
            return

        if detections is None or left_view == right_view:
            pixmap = QPixmap.fromImage(self._render_video_view(display_frame, detections, left_view))
            left_pixmap = right_pixmap = pixmap
        else:
            left_pixmap = QPixmap.fromImage(
                self._render_video_view(display_frame, detections, left_view)
            )
            right_pixmap = QPixmap.fromImage(
                self._render_video_view(display_frame, detections, right_view)
            )
        if left_pixmap is right_pixmap and self.detection_camera.size() == self.tracking_camera.size():
            scaled = left_pixmap.scaled(
                self.detection_camera.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
            self.detection_camera.setPixmap(scaled)
            self.tracking_camera.setPixmap(scaled)
        else:
            self._set_video_label(self.detection_camera, left_pixmap)
            self._set_video_label(self.tracking_camera, right_pixmap)
        self.last_detection_received_at = display_key

        if display_frame.frame != self.last_camera_frame:
            self.last_camera_frame = display_frame.frame
            now = time.monotonic()
            self.display_times.append(now)
            self.display_times = [t for t in self.display_times if now - t <= 2.0]
        display_fps = (
            (len(self.display_times) - 1) / (self.display_times[-1] - self.display_times[0])
            if len(self.display_times) >= 2 and self.display_times[-1] > self.display_times[0]
            else 0.0
        )
        self.camera_info.setText(
            f"Frame {camera.frame}  ·  Sim time {camera.timestamp:.3f} s  ·  "
            f"Capture FPS {snapshot['rates']['camera']:.1f}  ·  Display FPS {display_fps:.1f}"
        )

    def _refresh(self) -> None:
        snapshot = self.collector.state.snapshot()
        self.status.setText(snapshot["status"])
        rates = snapshot["rates"]
        camera = snapshot["camera"]

        detections = snapshot["detections"]
        if self.yolo_worker and self.yolo_worker.status.startswith("YOLO11n 오류"):
            self.yolo_toggle.setEnabled(False)
            self.yolo_config_info.setText(self.yolo_worker.status)
            if self.recorder.mode in ("yolo", "tracking", "radar_fusion"):
                self.recorder.stop_session()
                self._finish_recording_clock("YOLO 오류로 기록을 중지했습니다.")
                self.yolo_worker.set_radar_fusion_enabled(self._radar_fusion_enabled_before_record)
                self._set_checked(self.yolo_record_button, False)
                self._set_checked(self.tracking_record_button, False)
                self._set_checked(self.radar_fusion_record_button, False)
                self.yolo_record_button.setText("YOLO 기록 시작")
                self.tracking_record_button.setText("YOLO+트래킹 기록 시작")
                self.radar_fusion_record_button.setText("YOLO+레이더 융합 기록 시작")
                self.record_button.setEnabled(True)
                self.yolo_record_button.setEnabled(False)
                self.tracking_record_button.setEnabled(False)
                self.radar_fusion_record_button.setEnabled(False)
                self._yolo_enabled_by_record_button = False
                self._tracking_enabled_before_record = False
                self._radar_fusion_enabled_before_record = False
            if self.yolo_toggle.isChecked():
                self.yolo_toggle.setChecked(False)
            self.yolo_toggle.setText("탐지 오류")
        elif self.yolo_worker:
            self.yolo_config_info.setText(self.yolo_worker.status)
            if self.recorder.mode in ("yolo", "tracking", "radar_fusion"):
                self.record_button.setEnabled(False)
                self.yolo_record_button.setEnabled(self.recorder.mode == "yolo")
                self.tracking_record_button.setEnabled(self.recorder.mode == "tracking")
                self.radar_fusion_record_button.setEnabled(self.recorder.mode == "radar_fusion")
                self.yolo_toggle.setEnabled(False)
                self.tracking_toggle.setEnabled(False)
            elif self.recorder.mode == "standard":
                self.record_button.setEnabled(True)
                self.yolo_record_button.setEnabled(False)
                self.tracking_record_button.setEnabled(False)
                self.radar_fusion_record_button.setEnabled(False)
            else:
                self.record_button.setEnabled(True)
                self.yolo_record_button.setEnabled(self._yolo_ready())
                self.tracking_record_button.setEnabled(self._yolo_ready())
                self.radar_fusion_record_button.setEnabled(self._yolo_ready())
                self.yolo_toggle.setEnabled(self._yolo_ready())
                self.tracking_toggle.setEnabled(bool(self.yolo_worker.enabled))
        tracking_enabled = bool(self.yolo_worker and self.yolo_worker.tracking_enabled)
        if detections is not None:
            detected_camera = detections.camera
            self.detection_info.setText(
                f"Frame {detected_camera.frame} · {self.detection_view.currentText()} · "
                f"검출 {len(detections.boxes)}개 · 처리 {detections.processing_ms:.1f} ms"
            )
            right_mode = self.tracking_view.currentData()
            if right_mode == "original":
                right_status = "원본 영상"
            elif right_mode == "yolo":
                right_status = f"검출 {len(detections.boxes)}개"
            elif not tracking_enabled:
                right_status = "트래킹 꺼짐 · 원본 영상 표시"
            elif right_mode == "radar_fusion":
                radar_count = sum(bool(item.get("radar_used")) for item in
                                  (detections.positioning or {}).get("objects", []))
                right_status = f"트랙 {len(detections.tracks)}개 · 레이더 거리 연결 {radar_count}개"
            else:
                view_tracks = detections.vision_tracks
                right_status = f"트랙 {len(view_tracks)}개 · 영상 기준"
            self.tracking_info.setText(
                f"Frame {detected_camera.frame} · {self.tracking_view.currentText()} · {right_status}"
            )
        elif camera is not None:
            if self.yolo_worker:
                state_text = "탐지 결과 대기 중" if self.yolo_worker.enabled else "실시간 탐지가 꺼져 있음"
            else:
                state_text = "YOLO11n 미설정"
            self.detection_info.setText(
                f"Frame {camera.frame} · {self.detection_view.currentText()} · {state_text}"
            )
            self.tracking_info.setText(
                f"Frame {camera.frame} · {self.tracking_view.currentText()} · "
                f"{('탐지 결과 대기 중' if tracking_enabled else 'YOLO 탐지 후 사용 가능')}"
            )
        else:
            self.detection_camera.clear()
            self.tracking_camera.clear()
            if self.yolo_worker:
                self.detection_camera.setText(self.yolo_worker.status)
                self.detection_info.setText(
                    "카메라 프레임 대기 중"
                )
            else:
                self.detection_camera.setText("YOLO11n 미설정")
                self.detection_info.setText("카메라 프레임 대기 중")
            self.tracking_camera.setText("카메라 프레임 대기 중")
            self.tracking_info.setText("카메라 프레임 대기 중")
            self.last_detection_received_at = None

        positioning = snapshot["positioning"]
        if positioning is not None:
            summary = positioning["live_summary"]
            camera_stats = summary["camera"]
            fusion_stats = summary["radar_fusion"]
            camera_rmse = camera_stats["rmse_m"]
            fusion_rmse = fusion_stats["rmse_m"]
            camera_value = f"{camera_rmse:.2f} m (n={camera_stats['samples']})" if camera_rmse is not None else "계산 대기"
            fusion_value = f"{fusion_rmse:.2f} m (n={fusion_stats['samples']})" if fusion_rmse is not None else "계산 대기"
            radar_delta = positioning.get("radar_delta_frames")
            radar_sync = f"레이더 Δframe {radar_delta:+d}" if radar_delta is not None else "레이더 동기화 대기"
            truth_delta = positioning.get("ground_truth_delta_frames")
            truth_sync = f"CARLA 기준 Δframe {truth_delta:+d}" if truth_delta is not None else "CARLA 기준 위치 대기"
            self.position_accuracy_info.setText(
                f"카메라 위치 RMSE {camera_value} · 레이더 융합 RMSE {fusion_value}\n"
                f"기준 위치 매칭 {summary['camera_matches']}개 · {radar_sync} · {truth_sync}"
            )
        else:
            self.position_accuracy_info.setText(
                "트래킹을 켜고 CARLA 차량이 카메라에 잡히면 위치 오차를 계산합니다"
            )

        radar = snapshot["radar"]
        if radar is not None:
            points = radar["points"]
            self.radar_info.setText(
                f"Frame {radar['frame']}  ·  Points {len(points)}  ·  FPS {rates['radar']:.1f}  ·  "
                f"Sim time {radar['simulation_timestamp']:.3f} s"
            )
            if radar["frame"] != self.last_radar_frame:
                self.radar_plot.set_points(points)
                ordered_points = sorted(points, key=lambda point: point["distance_m"])
                self.radar_table.setUpdatesEnabled(False)
                try:
                    self.radar_table.setRowCount(len(ordered_points))
                    for row, point in enumerate(ordered_points):
                        values = (
                            f"{point['distance_m']:.1f}",
                            f"{math.degrees(point['azimuth_rad']):+.1f}",
                            f"{math.degrees(point['altitude_rad']):+.1f}",
                            f"{point['relative_velocity_mps']:+.2f}",
                        )
                        for column, value in enumerate(values):
                            self.radar_table.setItem(row, column, QTableWidgetItem(value))
                finally:
                    self.radar_table.setUpdatesEnabled(True)
                self.last_radar_frame = radar["frame"]
        elif self.last_radar_frame != -1:
            self.radar_plot.set_points([])
            self.radar_table.setRowCount(0)
            self.radar_info.setText("Frame —  ·  Points —  ·  FPS —")
            self.last_radar_frame = -1

        vehicle = snapshot["vehicle"]
        if vehicle is not None:
            pose = vehicle["transform"]
            self.vehicle_fields["position"].setText(self._xyz(pose["location"]))
            self.vehicle_fields["speed"].setText(f"{vehicle['speed_kmh']:.1f} km/h")
            self.vehicle_fields["yaw"].setText(f"{pose['rotation']['yaw']:.1f}°")
            self.vehicle_fields["velocity"].setText(self._xyz(vehicle["velocity_mps"]))
            self.vehicle_fields["frame"].setText(
                f"{vehicle['frame']} / {vehicle['simulation_timestamp']:.3f} s  ·  {rates['vehicle']:.1f} FPS"
            )
        else:
            for label in self.vehicle_fields.values():
                label.setText("—")

        imu = snapshot["imu"]
        if imu is not None:
            self.imu_fields["accel"].setText(self._xyz(imu["accelerometer_mps2"]))
            self.imu_fields["gyro"].setText(self._xyz(imu["gyroscope_rps"]))
            self.imu_fields["compass"].setText(f"{imu['compass_deg']:.1f}°")
            self.imu_fields["frame"].setText(f"{imu['frame']}  ·  {rates['imu']:.1f} FPS")
        else:
            for label in self.imu_fields.values():
                label.setText("—")

        gnss = snapshot["gnss"]
        if gnss is not None:
            self.gnss_fields["latitude"].setText(f"{gnss['latitude']:.7f}°")
            self.gnss_fields["longitude"].setText(f"{gnss['longitude']:.7f}°")
            self.gnss_fields["altitude"].setText(f"{gnss['altitude_m']:.2f}")
            self.gnss_fields["frame"].setText(f"{gnss['frame']}  ·  {rates['gnss']:.1f} FPS")
        else:
            for label in self.gnss_fields.values():
                label.setText("—")

        events = snapshot["events"]
        self.events.setText(
            "\n".join(
                f"#{event['frame']}  "
                + (
                    f"충돌 · {event['other_actor']}"
                    if event["kind"] == "collision"
                    else f"차선 이탈 · {', '.join(event['markings'])}"
                )
                for event in events[:5]
            )
            if events
            else "감지된 이벤트 없음"
        )
        stats = self.recorder.stats()
        if stats["path"] is None:
            record_summary = "기록 대기 중"
        else:
            mode = {
                "yolo": "YOLO 기록",
                "tracking": "YOLO+트래킹 기준선 기록",
                "radar_fusion": "YOLO+레이더 융합 트래킹 기록",
                "standard": "일반 기록",
            }.get(stats["mode"], "기록")
            active = "진행 중" if stats["enabled"] else "완료"
            record_summary = (
                f"{mode} {active} · {stats['path']}\n"
                f"JPEG {stats['saved_frames']}개 저장 · 이미지 누락 {stats['dropped_frames']}개 · "
                f"측정값 누락 {stats['dropped_records']}개"
            )
            summary_path = Path(stats["path"]) / "accuracy_summary.json"
            if not stats["enabled"] and summary_path.is_file():
                record_summary += f"\n위치 정확도 요약: {summary_path}"
        self.record_info.setText(record_summary + (f"\n{stats['error']}" if stats["error"] else ""))

    def closeEvent(self, event: Any) -> None:
        if self._weather_workflow_dialog is not None:
            self._weather_workflow_dialog.shutdown()
        if self._traffic_light_thread is not None:
            self._traffic_light_thread.wait()
        self.recording_timer.stop()
        self.timer.stop()
        self.video_timer.stop()
        self.collector.on_camera_frame = None
        self.collector.stop()
        if self.yolo_worker:
            self.yolo_worker.stop()
        self.recorder.stop()
        if self._overlay_generator and self._overlay_generator.isRunning():
            self._overlay_generator.wait()
        if self._experiment_evaluator and self._experiment_evaluator.isRunning():
            self._experiment_evaluator.wait()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="CARLA Qt live sensor monitor")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role", default="hero", help="CARLA vehicle role_name")
    parser.add_argument("--output", type=Path, default=Path("data/recordings"))
    parser.add_argument("--yolo-library", type=Path, help="yolo11_detector DLL/.so path")
    parser.add_argument("--yolo-engine", type=Path, help="YOLO11n TensorRT engine path")
    parser.add_argument("--tensorrt-bin", type=Path, help="Windows TensorRT DLL directory")
    parser.add_argument("--opencv-bin", type=Path, help="Windows OpenCV DLL directory")
    args = parser.parse_args()
    if bool(args.yolo_library) != bool(args.yolo_engine):
        parser.error("--yolo-library and --yolo-engine must be specified together")

    app = QApplication(sys.argv[:1])
    font_family = "Segoe UI"
    if sys.platform == "win32":
        font_file = Path("C:/Windows/Fonts/malgun.ttf")
        if font_file.exists():
            font_id = QFontDatabase.addApplicationFont(str(font_file))
            families = QFontDatabase.applicationFontFamilies(font_id) if font_id >= 0 else []
            if families:
                font_family = families[0]
    app.setFont(QFont(font_family, 10))
    state = LiveState()
    recorder = SessionRecorder(args.output, args.host, args.port, args.role)
    collector = CarlaCollector(state, recorder, args.host, args.port, args.role)
    dll_directories = tuple(path for path in (args.tensorrt_bin, args.opencv_bin) if path)
    yolo_worker = (YoloWorker(state, recorder, args.yolo_library, args.yolo_engine, dll_directories)
                   if args.yolo_library else None)
    window = MonitorWindow(collector, recorder, yolo_worker)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
