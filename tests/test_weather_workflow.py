"""Weather workflow schedules separate timed recording sessions."""

from __future__ import annotations

import time
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch
from uuid import uuid4

from PySide6.QtWidgets import QApplication

from monitor.qt.app import MonitorWindow
from monitor.qt.weather_workflow import ApplyWeatherThread, WeatherWorkflowDialog
from tests.test_timed_recording import FakeCollector, FakeRecorder, FakeWorker


class WeatherWorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.temporary = Path(__file__).parent / f"_workflow_test_{uuid4().hex}"
        self.temporary.mkdir()
        self.recorder = FakeRecorder()
        self.recorder.root = self.temporary
        self.monitor = MonitorWindow(FakeCollector(), self.recorder, FakeWorker())
        self.dialog = WeatherWorkflowDialog(self.monitor)
        self.dialog.mode.setCurrentIndex(self.dialog.mode.findData("standard"))
        self.dialog.warmup.setValue(0)
        self.dialog.table.setRowCount(0)
        self.dialog._add_step("ClearNoon", 1)
        self.dialog._add_step("HardRainNoon", 1)
        self.dialog._apply_weather = lambda: self.dialog._weather_applied({"cloudiness": 10.0})

    def tearDown(self) -> None:
        self.dialog.shutdown()
        self.dialog.close()
        self.monitor.close()
        if self.temporary.resolve().parent == Path(__file__).parent.resolve():
            shutil.rmtree(self.temporary)

    def _process(self) -> None:
        for _ in range(3):
            self.app.processEvents()

    def test_weather_worker_sets_carla_world_preset(self) -> None:
        class World:
            weather = None

            def set_weather(self, weather: object) -> None:
                self.weather = weather

        world = World()

        class Client:
            def __init__(self, host: str, port: int) -> None:
                self.host = host
                self.port = port

            def set_timeout(self, timeout: float) -> None:
                pass

            def get_world(self) -> World:
                return world

        worker = ApplyWeatherThread("127.0.0.1", 2000, "HardRainNoon")
        applied: list[dict] = []
        worker.applied.connect(applied.append)
        with patch("monitor.qt.weather_workflow.carla.Client", Client):
            worker.run()
        self.assertIsNotNone(world.weather)
        self.assertEqual(applied[0]["precipitation"], world.weather.precipitation)

    def test_auto_advances_weather_and_keeps_separate_metadata(self) -> None:
        self.monitor.record_minutes.setValue(2)
        self.dialog.start_button.click()
        self._process()
        self.assertEqual(self.recorder.starts, [("standard", 1)])
        self.monitor._recording_deadline = time.monotonic() - 1
        self.monitor._update_recording_clock()
        self._process()
        self.assertEqual(self.recorder.starts, [("standard", 1), ("standard", 1)])
        first, second = self.recorder.weather_workflows
        self.assertEqual(first["weather_preset"], "ClearNoon")
        self.assertEqual(second["weather_preset"], "HardRainNoon")
        self.assertEqual(first["workflow_id"], second["workflow_id"])
        self.assertEqual(first["step"], 1)
        self.assertEqual(second["step"], 2)
        self.assertEqual(self.recorder.weathers[0], {"cloudiness": 10.0})
        self.monitor._recording_deadline = time.monotonic() - 1
        self.monitor._update_recording_clock()
        self.assertFalse(self.dialog._running)
        self.assertEqual(self.monitor.record_minutes.value(), 2)
        self.assertFalse(self.recorder.enabled)

    def test_manual_stop_aborts_remaining_steps(self) -> None:
        self.dialog.start_button.click()
        self._process()
        self.monitor.record_button.click()
        self._process()
        self.assertFalse(self.dialog._running)
        self.assertEqual(len(self.recorder.starts), 1)


if __name__ == "__main__":
    unittest.main()
