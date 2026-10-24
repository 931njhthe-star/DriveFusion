"""Recording duration controls use the same stop path as the manual buttons."""

from __future__ import annotations

import os
from pathlib import Path
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from monitor.qt.app import MonitorWindow


class FakeCollector:
    on_camera_frame = None
    host = "127.0.0.1"
    port = 2000

    class State:
        @staticmethod
        def snapshot() -> dict[str, str]:
            return {"status": "연결됨 · Town02"}

    state = State()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class FakeRecorder:
    def __init__(self) -> None:
        self.root = Path("unused")
        self.mode: str | None = None
        self.enabled = False
        self.starts: list[tuple[str, int | None]] = []
        self.weather_workflows: list[dict | None] = []
        self.weathers: list[dict | None] = []
        self.tracking_max_lost_frames: list[int] = []
        self.radar_track_correction_enabled: list[bool | None] = []
        self.stop_calls = 0

    def start_session(self, mode: str, _engine: Path | None = None,
                      planned_duration_seconds: int | None = None,
                      weather_workflow: dict | None = None,
                      weather: dict | None = None,
                      tracking_max_lost_frames: int = 10,
                       radar_track_correction_enabled: bool | None = None,
                       traffic_lights: dict | None = None) -> Path:
        self.mode = mode
        self.enabled = True
        self.starts.append((mode, planned_duration_seconds))
        self.weather_workflows.append(weather_workflow)
        self.weathers.append(weather)
        self.tracking_max_lost_frames.append(tracking_max_lost_frames)
        self.radar_track_correction_enabled.append(radar_track_correction_enabled)
        return Path("unused")

    def stop_session(self) -> None:
        self.mode = None
        self.enabled = False
        self.stop_calls += 1

    def is_enabled(self) -> bool:
        return self.enabled

    def stop(self) -> None:
        self.stop_session()


class FakeWorker:
    status = "YOLO11n 준비됨"
    engine_path = Path("engine")

    def __init__(self) -> None:
        self.enabled = False
        self.tracking_enabled = False
        self.radar_fusion_enabled = True
        self.max_lost_frames = 10
        self.radar_track_correction_enabled = False

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        self.status = "YOLO11n 탐지 중" if enabled else "YOLO11n 탐지 꺼짐"

    def set_tracking_enabled(self, enabled: bool) -> None:
        self.tracking_enabled = enabled

    def set_radar_fusion_enabled(self, enabled: bool) -> None:
        self.radar_fusion_enabled = enabled

    def set_max_lost_frames(self, frames: int) -> None:
        self.max_lost_frames = frames

    def set_radar_track_correction_enabled(self, enabled: bool) -> None:
        self.radar_track_correction_enabled = enabled


class TimedRecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.recorder = FakeRecorder()
        self.worker = FakeWorker()
        self.window = MonitorWindow(FakeCollector(), self.recorder, self.worker)

    def tearDown(self) -> None:
        self.window.close()

    def test_zero_duration_keeps_manual_start_and_stop(self) -> None:
        self.window.record_button.click()
        self.assertEqual(self.recorder.starts, [("standard", None)])
        self.assertTrue(self.window.record_button.isChecked())
        self.assertIsNone(self.window._recording_deadline)
        self.window.record_button.click()
        self.assertFalse(self.window.record_button.isChecked())
        self.assertTrue(self.window.record_minutes.isEnabled())

    def test_timed_standard_recording_stops_and_can_be_stopped_early(self) -> None:
        self.window.record_seconds.setValue(1)
        self.assertIn("자동 중지", self.window.recording_timer_info.text())
        self.window.record_button.click()
        self.assertEqual(self.recorder.starts, [("standard", 1)])
        self.assertFalse(self.window.record_seconds.isEnabled())
        self.window._recording_deadline = time.monotonic() - 1
        self.window._update_recording_clock()
        self.assertFalse(self.window.record_button.isChecked())
        self.assertFalse(self.recorder.enabled)
        self.assertIn("자동", self.window.recording_timer_info.text())
        self.assertTrue(self.window.record_seconds.isEnabled())

        self.window.record_button.click()
        self.window.record_button.click()
        self.assertFalse(self.recorder.enabled)
        self.assertIn("중지", self.window.recording_timer_info.text())

    def test_timed_ai_recording_uses_existing_stop_path(self) -> None:
        self.window.record_minutes.setValue(10)
        self.window.radar_fusion_record_button.setEnabled(True)
        self.window.radar_fusion_record_button.click()
        self.assertEqual(self.recorder.starts, [("radar_fusion", 600)])
        self.assertTrue(self.worker.enabled)
        self.window._recording_deadline = time.monotonic() - 1
        self.window._update_recording_clock()
        self.assertFalse(self.window.radar_fusion_record_button.isChecked())
        self.assertFalse(self.recorder.enabled)
        self.assertFalse(self.worker.enabled)

    def test_tracking_retention_applies_to_live_worker_and_recording(self) -> None:
        self.window.tracking_max_lost_frames.setValue(50)
        self.assertEqual(self.worker.max_lost_frames, 50)
        self.window.radar_fusion_record_button.setEnabled(True)
        self.window.radar_fusion_record_button.click()
        self.assertEqual(self.recorder.tracking_max_lost_frames[-1], 50)
        self.assertFalse(self.window.tracking_max_lost_frames.isEnabled())
        self.window.radar_fusion_record_button.click()
        self.assertTrue(self.window.tracking_max_lost_frames.isEnabled())

    def test_radar_correction_choice_is_saved_and_locked_during_recording(self) -> None:
        self.window.radar_correction_toggle.click()
        self.assertTrue(self.worker.radar_track_correction_enabled)
        self.window.radar_fusion_record_button.setEnabled(True)
        self.window.radar_fusion_record_button.click()
        self.assertEqual(self.recorder.radar_track_correction_enabled[-1], True)
        self.assertFalse(self.window.radar_correction_toggle.isEnabled())
        self.window.radar_fusion_record_button.click()
        self.assertTrue(self.window.radar_correction_toggle.isEnabled())

    def test_video_focus_expands_comparison_area_and_restores_controls(self) -> None:
        self.window.video_focus_button.click()
        self.assertTrue(self.window.video_focus_button.isChecked())
        self.assertTrue(self.window._telemetry_scroll.isHidden())
        self.assertTrue(all(widget.isHidden() for widget in self.window._video_detail_widgets))
        self.assertFalse(self.window.detection_camera.isHidden())
        self.assertFalse(self.window.tracking_camera.isHidden())

        self.window.video_focus_button.click()
        self.assertFalse(self.window.video_focus_button.isChecked())
        self.assertFalse(self.window._telemetry_scroll.isHidden())
        self.assertTrue(all(not widget.isHidden() for widget in self.window._video_detail_widgets))


if __name__ == "__main__":
    unittest.main()
