"""Schedule separate CARLA recording sessions under selected weather presets."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any
from uuid import uuid4

import carla
from PySide6.QtCore import QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QVBoxLayout,
)

from collector.live import weather_parameters


WEATHER_PRESETS = (
    ("맑음 · 낮", "ClearNoon"),
    ("흐림 · 낮", "CloudyNoon"),
    ("젖은 도로 · 낮", "WetNoon"),
    ("약한 비 · 낮", "SoftRainNoon"),
    ("보통 비 · 낮", "MidRainyNoon"),
    ("강한 비 · 낮", "HardRainNoon"),
    ("맑음 · 저녁", "ClearSunset"),
    ("흐림 · 저녁", "CloudySunset"),
    ("강한 비 · 저녁", "HardRainSunset"),
    ("맑음 · 밤", "ClearNight"),
    ("흐림 · 밤", "CloudyNight"),
    ("강한 비 · 밤", "HardRainNight"),
)
RECORDING_MODES = (
    ("일반", "standard"),
    ("YOLO 검출", "yolo"),
    ("YOLO + 트래킹", "tracking"),
    ("YOLO + 레이더 융합", "radar_fusion"),
)
class ApplyWeatherThread(QThread):
    applied = Signal(dict)
    failed = Signal(str)

    def __init__(self, host: str, port: int, preset: str, parent: Any = None) -> None:
        super().__init__(parent)
        self.host = host
        self.port = port
        self.preset = preset

    def run(self) -> None:
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(3.0)
            world = client.get_world()
            weather = getattr(carla.WeatherParameters, self.preset)
            world.set_weather(weather)
            self.applied.emit(weather_parameters(weather))
        except Exception as exc:
            self.failed.emit(str(exc))


class WeatherWorkflowDialog(QDialog):
    def __init__(self, monitor: Any) -> None:
        super().__init__(monitor)
        self.monitor = monitor
        self.setWindowTitle("날씨별 데이터 수집 워크플로")
        self.resize(650, 430)
        self._running = False
        self._steps: list[tuple[str, int]] = []
        self._index = 0
        self._workflow_id = ""
        self._applied_weather: dict[str, float] = {}
        self._weather_applied_at = ""
        self._recording_step = False
        self._original_duration = (0, 0)
        self._weather_thread: ApplyWeatherThread | None = None
        self._warmup_timer = QTimer(self)
        self._warmup_timer.setSingleShot(True)
        self._warmup_timer.timeout.connect(self._start_recording)
        self._health_timer = QTimer(self)
        self._health_timer.setInterval(1000)
        self._health_timer.timeout.connect(self._check_connection)
        self.monitor.recording_finished.connect(self._recording_finished)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("위에서 아래 순서로 날씨를 적용하고, 각 날씨를 별도 세션에 기록합니다."))
        settings = QHBoxLayout()
        settings.addWidget(QLabel("기록 모드"))
        self.mode = QComboBox()
        for label, value in RECORDING_MODES:
            self.mode.addItem(label, value)
        self.mode.setCurrentIndex(self.mode.findData("radar_fusion"))
        settings.addWidget(self.mode)
        settings.addWidget(QLabel("날씨 적용 후 대기"))
        self.warmup = QSpinBox()
        self.warmup.setRange(0, 60)
        self.warmup.setValue(5)
        self.warmup.setSuffix("초")
        settings.addWidget(self.warmup)
        layout.addLayout(settings)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(("날씨", "분", "초"))
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)
        self.table.setColumnWidth(1, 90)
        self.table.setColumnWidth(2, 90)
        self.table.verticalHeader().setVisible(False)
        layout.addWidget(self.table)
        controls = QHBoxLayout()
        self.add_button = QPushButton("단계 추가")
        self.remove_button = QPushButton("선택 삭제")
        self.up_button = QPushButton("위로")
        self.down_button = QPushButton("아래로")
        for button in (self.add_button, self.remove_button, self.up_button, self.down_button):
            controls.addWidget(button)
        self.add_button.clicked.connect(lambda: self._add_step("ClearNoon", 600))
        self.remove_button.clicked.connect(self._remove_step)
        self.up_button.clicked.connect(lambda: self._move_step(-1))
        self.down_button.clicked.connect(lambda: self._move_step(1))
        layout.addLayout(controls)

        self.info = QLabel("CARLA와 hero 차량이 연결되어 있어야 시작할 수 있습니다.")
        self.info.setWordWrap(True)
        layout.addWidget(self.info)
        actions = QHBoxLayout()
        self.start_button = QPushButton("워크플로 시작")
        self.stop_button = QPushButton("워크플로 중지")
        self.stop_button.setEnabled(False)
        actions.addWidget(self.start_button)
        actions.addWidget(self.stop_button)
        layout.addLayout(actions)
        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(lambda: self._finish("워크플로를 중지했습니다.", stop_recording=True))
        for preset in ("ClearNoon", "CloudyNoon", "HardRainNoon"):
            self._add_step(preset, 600)

    def _add_step(self, preset: str, duration: int) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        weather = QComboBox()
        for label, value in WEATHER_PRESETS:
            weather.addItem(label, value)
        weather.setCurrentIndex(weather.findData(preset))
        minutes = QSpinBox()
        minutes.setRange(0, 120)
        minutes.setValue(duration // 60)
        seconds = QSpinBox()
        seconds.setRange(0, 59)
        seconds.setValue(duration % 60)
        for column, widget in enumerate((weather, minutes, seconds)):
            self.table.setCellWidget(row, column, widget)
        self.table.selectRow(row)

    def _rows(self) -> list[tuple[str, int]]:
        return [
            (self.table.cellWidget(row, 0).currentData(),
             self.table.cellWidget(row, 1).value() * 60
             + self.table.cellWidget(row, 2).value())
            for row in range(self.table.rowCount())
        ]

    def _remove_step(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)

    def _move_step(self, direction: int) -> None:
        row = self.table.currentRow()
        target = row + direction
        if row < 0 or not 0 <= target < self.table.rowCount():
            return
        rows = self._rows()
        rows[row], rows[target] = rows[target], rows[row]
        self.table.setRowCount(0)
        for preset, duration in rows:
            self._add_step(preset, duration)
        self.table.selectRow(target)

    def _set_editable(self, editable: bool) -> None:
        for widget in (self.table, self.mode, self.warmup, self.add_button,
                       self.remove_button, self.up_button, self.down_button,
                       self.start_button):
            widget.setEnabled(editable)
        self.stop_button.setEnabled(not editable)

    def _start(self) -> None:
        if self._running:
            return
        if self._weather_thread is not None and self._weather_thread.isRunning():
            self.info.setText("이전 날씨 변경이 끝난 뒤 다시 시작하세요.")
            return
        steps = self._rows()
        if not steps or any(duration <= 0 for _, duration in steps):
            self.info.setText("단계를 추가하고 각 기록 시간을 1초 이상으로 설정하세요.")
            return
        if self.monitor.recorder.is_enabled():
            self.info.setText("현재 기록을 중지한 뒤 워크플로를 시작하세요.")
            return
        if not self.monitor.collector.state.snapshot()["status"].startswith("연결됨"):
            self.info.setText("CARLA와 hero 차량 연결을 확인하세요.")
            return
        if self.mode.currentData() != "standard" and not self.monitor._yolo_ready():
            self.info.setText("선택한 기록 모드에는 YOLO가 준비되어 있어야 합니다.")
            return
        self._steps = steps
        self._index = 0
        self._workflow_id = datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid4().hex[:8]
        workflow_dir = Path(self.monitor.recorder.root) / f"workflow_{self._workflow_id}"
        try:
            workflow_dir.mkdir(parents=True, exist_ok=False)
            (workflow_dir / "workflow.json").write_text(json.dumps({
                "workflow_id": self._workflow_id,
                "created_at": datetime.now().astimezone().isoformat(),
                "recording_mode": self.mode.currentData(),
                "settle_seconds": self.warmup.value(),
                "steps": [{"step": index, "weather_preset": preset,
                           "planned_duration_seconds": duration}
                          for index, (preset, duration) in enumerate(steps, 1)],
            }, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            self.info.setText(f"워크플로 폴더 생성 실패: {exc}")
            return
        self._original_duration = (self.monitor.record_minutes.value(),
                                   self.monitor.record_seconds.value())
        self._recording_step = False
        self._running = True
        self._set_editable(False)
        self._health_timer.start()
        self._apply_weather()

    def _apply_weather(self) -> None:
        if not self._running:
            return
        preset, _duration = self._steps[self._index]
        self.info.setText(f"{self._index + 1}/{len(self._steps)} · {preset} 날씨 적용 중...")
        worker = ApplyWeatherThread(self.monitor.collector.host, self.monitor.collector.port,
                                    preset, self)
        self._weather_thread = worker
        workflow_id = self._workflow_id
        worker.applied.connect(
            lambda parameters: self._weather_applied(parameters)
            if self._running and workflow_id == self._workflow_id else None)
        worker.failed.connect(
            lambda error: self._weather_failed(error)
            if self._running and workflow_id == self._workflow_id else None)
        worker.finished.connect(lambda: self._clear_weather_thread(worker))
        worker.finished.connect(worker.deleteLater)
        worker.start()

    def _clear_weather_thread(self, worker: ApplyWeatherThread) -> None:
        if self._weather_thread is worker:
            self._weather_thread = None

    def _check_connection(self) -> None:
        if self._running and not self.monitor.collector.state.snapshot()["status"].startswith("연결됨"):
            self._finish("CARLA 연결이 끊겨 워크플로를 중지했습니다.", stop_recording=True)

    def _weather_applied(self, parameters: dict[str, float]) -> None:
        if not self._running:
            return
        if not self.monitor.collector.state.snapshot()["status"].startswith("연결됨"):
            self._finish("CARLA 연결이 끊겨 워크플로를 중지했습니다.")
            return
        self._applied_weather = parameters
        self._weather_applied_at = datetime.now().astimezone().isoformat()
        delay = self.warmup.value()
        self.info.setText(f"{self._index + 1}/{len(self._steps)} · 날씨 적용됨 · {delay}초 안정화 대기")
        self._warmup_timer.start(delay * 1000)

    def _weather_failed(self, error: str) -> None:
        if self._running:
            self._finish(f"날씨 변경 실패: {error}")

    def _start_recording(self) -> None:
        if not self._running:
            return
        if not self.monitor.collector.state.snapshot()["status"].startswith("연결됨"):
            self._finish("CARLA 연결이 끊겨 워크플로를 중지했습니다.")
            return
        preset, duration = self._steps[self._index]
        metadata = {
            "workflow_id": self._workflow_id,
            "step": self._index + 1,
            "total_steps": len(self._steps),
            "weather_preset": preset,
            "weather_parameters": self._applied_weather,
            "weather_applied_at": self._weather_applied_at,
            "settle_seconds": self.warmup.value(),
        }
        if not self.monitor.start_workflow_recording(self.mode.currentData(), duration, metadata):
            self._finish("기록을 시작하지 못했습니다. CARLA 연결과 기록 모드를 확인하세요.")
            return
        self._recording_step = True
        minutes, seconds = divmod(duration, 60)
        self.info.setText(
            f"{self._index + 1}/{len(self._steps)} · {preset} 기록 중 · {minutes}분 {seconds}초")

    def _recording_finished(self, automatic: bool) -> None:
        if not self._running or not self._recording_step:
            return
        self._recording_step = False
        if not automatic:
            self._finish("기록이 중지되어 워크플로를 종료했습니다.")
            return
        self._index += 1
        if self._index >= len(self._steps):
            self._finish("모든 날씨의 데이터 수집을 마쳤습니다.")
        else:
            QTimer.singleShot(0, self._apply_weather)

    def _finish(self, message: str, stop_recording: bool = False) -> None:
        if not self._running:
            return
        self._running = False
        self._warmup_timer.stop()
        self._health_timer.stop()
        if stop_recording and self._recording_step:
            self.monitor.stop_workflow_recording()
        self._recording_step = False
        self.monitor.record_minutes.setValue(self._original_duration[0])
        self.monitor.record_seconds.setValue(self._original_duration[1])
        self._set_editable(True)
        self.info.setText(message)

    def shutdown(self) -> None:
        self._finish("워크플로를 중지했습니다.", stop_recording=True)
        if self._weather_thread is not None and self._weather_thread.isRunning():
            self._weather_thread.wait()

    def closeEvent(self, event: Any) -> None:
        self._finish("워크플로를 중지했습니다.", stop_recording=True)
        super().closeEvent(event)
