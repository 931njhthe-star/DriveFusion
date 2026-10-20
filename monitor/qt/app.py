"""Live Qt dashboard for CARLA RGB, radar, telemetry, and other sensors."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Any

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QFontDatabase, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication,
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
    def __init__(self, collector: CarlaCollector, recorder: SessionRecorder) -> None:
        super().__init__()
        self.collector = collector
        self.recorder = recorder
        self.last_camera_frame = -1
        self.last_radar_frame = -1
        self.display_times: list[float] = []
        self.setWindowTitle("CARLA RadarVision · Live Sensor Monitor")
        self.resize(1500, 900)
        self._build_ui()
        self._apply_style()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)
        self.collector.start()

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
        self.record_button = QPushButton("● 기록 중지")
        self.record_button.clicked.connect(self._toggle_recording)
        header.addWidget(self.record_button)
        outer.addLayout(header)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        outer.addWidget(splitter, 1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        camera_group = QGroupBox("RGB CAMERA · 실시간 영상")
        camera_layout = QVBoxLayout(camera_group)
        self.camera = QLabel("카메라 프레임 대기 중")
        self.camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera.setMinimumSize(640, 480)
        self.camera.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.camera.setObjectName("camera")
        camera_layout.addWidget(self.camera, 1)
        self.camera_info = QLabel("Frame —  ·  Sim time —  ·  Capture FPS —  ·  Display FPS —")
        camera_layout.addWidget(self.camera_info)
        left_layout.addWidget(camera_group, 1)

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
        self.record_info = QLabel(str(self.recorder.path))
        self.record_info.setWordWrap(True)
        record_layout.addWidget(self.record_info)
        right_layout.addWidget(record_group)
        right_layout.addStretch()

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

    def _toggle_recording(self) -> None:
        enabled = self.recorder.toggle()
        self.record_button.setText("● 기록 중지" if enabled else "○ 기록 시작")

    @staticmethod
    def _xyz(values: dict[str, float]) -> str:
        return f"X {values['x']:.2f}   Y {values['y']:.2f}   Z {values['z']:.2f}"

    def _refresh(self) -> None:
        snapshot = self.collector.state.snapshot()
        self.status.setText(snapshot["status"])
        rates = snapshot["rates"]

        camera = snapshot["camera"]
        if camera is not None:
            if camera.frame != self.last_camera_frame:
                image = QImage(
                    camera.bgra,
                    camera.width,
                    camera.height,
                    camera.width * 4,
                    QImage.Format.Format_ARGB32,
                ).copy()
                self.camera.setPixmap(
                    QPixmap.fromImage(image).scaled(
                        self.camera.size(),
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation,
                    )
                )
                self.last_camera_frame = camera.frame
                self.display_times.append(time.monotonic())
                self.display_times = [t for t in self.display_times if time.monotonic() - t <= 2.0]
            display_fps = (
                (len(self.display_times) - 1) / (self.display_times[-1] - self.display_times[0])
                if len(self.display_times) >= 2 and self.display_times[-1] > self.display_times[0]
                else 0.0
            )
            self.camera_info.setText(
                f"Frame {camera.frame}  ·  Sim time {camera.timestamp:.3f} s  ·  "
                f"Capture FPS {rates['camera']:.1f}  ·  Display FPS {display_fps:.1f}"
            )
        elif self.last_camera_frame != -1:
            self.camera.clear()
            self.camera.setText("카메라 프레임 대기 중")
            self.camera_info.setText("Frame —  ·  Sim time —  ·  Capture FPS —  ·  Display FPS —")
            self.last_camera_frame = -1
            self.display_times.clear()

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
        self.record_info.setText(
            f"{self.recorder.path}\n"
            f"JPEG {stats['saved_frames']}개 저장 · 이미지 누락 {stats['dropped_frames']}개 · "
            f"측정값 누락 {stats['dropped_records']}개"
            + (f"\n{stats['error']}" if stats["error"] else "")
        )

    def closeEvent(self, event: Any) -> None:
        self.timer.stop()
        self.collector.stop()
        self.recorder.stop()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="CARLA Qt live sensor monitor")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role", default="hero", help="CARLA vehicle role_name")
    parser.add_argument("--output", type=Path, default=Path("data/recordings"))
    args = parser.parse_args()

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
    window = MonitorWindow(collector, recorder)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
