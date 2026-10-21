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
    QComboBox,
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
        self.last_camera_frame = -1
        self.last_detection_received_at: tuple[Any, ...] | None = None
        self.last_radar_frame = -1
        self.display_times: list[float] = []
        self._video_update_pending = threading.Event()
        self.setWindowTitle("CARLA RadarVision · Live Sensor Monitor")
        self.resize(1500, 900)
        self._build_ui()
        self._apply_style()
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
        outer.addLayout(header)

        splitter = QSplitter(Qt.Orientation.Horizontal)
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
        self.evaluate_button = QPushButton("저장 세션 3~15m 실험 결과 생성")
        self.evaluate_button.clicked.connect(self._evaluate_saved_session)
        left_layout.addWidget(self.evaluate_button)
        self.evaluation_info = QLabel("CARLA GT 기준 3~15m 차량의 세 위치 추정 방법을 비교합니다.")
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

        record_group = QGroupBox("RECORDING · 수집 데이터 저장")
        record_layout = QVBoxLayout(record_group)
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
                color = (QColor("#51c9ff") if radar_used else QColor("#f5d76e")) \
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
                painter.drawText(track.x, max(14, track.y - 4), label)
        painter.end()
        return image

    @staticmethod
    def _set_video_label(label: QLabel, pixmap: QPixmap) -> None:
        label.setPixmap(pixmap.scaled(
            label.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        ))

    def _toggle_recording(self, enabled: bool) -> None:
        if enabled:
            try:
                self.recorder.start_session("standard")
            except Exception as exc:
                self._set_checked(self.record_button, False)
                self.record_info.setText(f"기록 시작 실패: {exc}")
                return
            self.record_button.setText("일반 기록 중지")
            self.yolo_record_button.setEnabled(False)
            self.tracking_record_button.setEnabled(False)
            self.radar_fusion_record_button.setEnabled(False)
        else:
            self.recorder.stop_session()
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
            self.recorder.start_session(mode, self.yolo_worker.engine_path)
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
        return True

    def _stop_ai_recording(self) -> None:
        self.recorder.stop_session()
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

    def _evaluate_saved_session(self) -> None:
        if self._experiment_evaluator is not None and self._experiment_evaluator.isRunning():
            return
        recordings_dir = Path("data/recordings").resolve()
        selected = QFileDialog.getExistingDirectory(
            self,
            "실험 결과를 만들 저장 세션 선택",
            str(recordings_dir if recordings_dir.exists() else Path.cwd()),
        )
        if not selected:
            return
        session_dir = Path(selected)
        active_session = self.recorder.stats()["path"]
        if (self.recorder.is_enabled() and active_session is not None
                and session_dir.resolve() == active_session.resolve()):
            self.evaluation_info.setText("기록 중인 세션입니다. 기록을 중지한 뒤 결과를 생성하세요.")
            return
        self.evaluate_button.setEnabled(False)
        self.evaluation_info.setText(f"3~15m 실험 결과 계산 중: {session_dir}")
        worker = ExperimentEvaluatorThread(session_dir, parent=self)
        self._experiment_evaluator = worker
        worker.completed.connect(self._evaluation_completed)
        worker.failed.connect(self._evaluation_failed)
        worker.finished.connect(self._evaluation_thread_finished)
        worker.start()

    def _evaluation_completed(self, summary: dict[str, Any]) -> None:
        lines = [f"GT 차체 간격 3~15m · {summary['frames_scored']}프레임 · 대상 {summary['eligible_gt_opportunities']}건"]
        for method, label in METHODS.items():
            result = summary["methods"][method]
            mae = f"{result['mae_m']:.2f}m" if result["mae_m"] is not None else "측정 없음"
            lines.append(f"{label}: 거리 MAE {mae}, {result['distance_samples']}/{result['gt_opportunities']}건")
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
