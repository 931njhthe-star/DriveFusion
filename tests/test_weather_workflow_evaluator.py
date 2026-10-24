"""Workflow grouping preserves weather and session boundaries in pooled results."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch
from uuid import uuid4

from collector.live import SessionRecorder
from monitor.qt.experiment_evaluator import METHODS
from monitor.qt.weather_workflow_evaluator import evaluate_weather_workflow


def _sample_rows(errors: list[float]) -> list[dict]:
    rows = []
    for frame, error in enumerate(errors, 1):
        for method in METHODS:
            rows.append({
                "frame": frame, "gt_actor_id": 7, "gt_distance_m": 5.0,
                "method": method, "error_m": error,
                "distance_error_signed_m": error, "predicted_error_m": None,
                "position_error_m": error, "radar_used": False,
                "track_id": 1, "observed": True, "detection_matched": True,
            })
    return rows


class WeatherWorkflowEvaluatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = Path(__file__).parent / f"_workflow_eval_{uuid4().hex}"
        self.temporary.mkdir()

    def tearDown(self) -> None:
        if self.temporary.resolve().parent == Path(__file__).parent.resolve():
            shutil.rmtree(self.temporary)

    def test_recorder_saves_weather_in_workflow_folders_and_manual_session(self) -> None:
        recorder = SessionRecorder(self.temporary, "127.0.0.1", 2000, "hero")
        context = {"workflow_id": "example", "weather_preset": "HardRainNoon",
                   "step": 1, "weather_parameters": {"precipitation": 100.0}}
        session = recorder.start_session("standard", weather_workflow=context,
                                         weather={"precipitation": 100.0},
                                         traffic_lights={"red_enabled": True,
                                                         "red_seconds": 20.0,
                                                         "traffic_light_count": 5})
        recorder.stop_session()
        self.assertEqual(session.parent.name, "weather_HardRainNoon")
        self.assertEqual(session.parent.parent.name, "workflow_example")
        metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["weather"]["precipitation"], 100.0)
        self.assertEqual(metadata["weather_workflow"]["step"], 1)
        self.assertEqual(metadata["traffic_lights"]["red_seconds"], 20.0)
        manual = recorder.start_session("standard", weather={"cloudiness": 12.0})
        recorder.stop_session()
        self.assertEqual(manual.parent, self.temporary)
        self.assertEqual(json.loads((manual / "session.json").read_text(
            encoding="utf-8"))["weather"]["cloudiness"], 12.0)
        tracking = recorder.start_session("tracking", tracking_max_lost_frames=50)
        recorder.stop_session()
        self.assertEqual(json.loads((tracking / "session.json").read_text(
            encoding="utf-8"))["tracking_max_lost_frames"], 50)
        fusion = recorder.start_session("radar_fusion", radar_track_correction_enabled=False)
        recorder.stop_session()
        self.assertFalse(json.loads((fusion / "session.json").read_text(
            encoding="utf-8"))["radar_track_correction_enabled"])

    def test_parent_folder_creates_weather_and_sample_weighted_combined_reports(self) -> None:
        root = self.temporary / "workflow_example"
        root.mkdir()
        (root / "workflow.json").write_text(json.dumps({"workflow_id": "example"}),
                                              encoding="utf-8")
        for preset in ("ClearNoon", "HardRainNoon"):
            session = root / f"weather_{preset}" / f"session_{preset}"
            session.mkdir(parents=True)
            (session / "session.json").write_text(json.dumps({
                "weather_workflow": {"workflow_id": "example", "weather_preset": preset},
            }), encoding="utf-8")
        for name, preset in (("session_ClearNoon_repeat", "ClearNoon"),
                             ("session_invalid", "HardRainNoon")):
            session = root / f"weather_{preset}" / name
            session.mkdir()
            (session / "session.json").write_text(json.dumps({
                "weather_workflow": {"workflow_id": "example", "weather_preset": preset},
            }), encoding="utf-8")

        def fake_evaluate(session_dir: Path, *, rows_out: list[dict], **_: object) -> dict:
            if session_dir.name == "session_invalid":
                raise ValueError("GT 기록 없음")
            errors = ([1.0] if session_dir.name == "session_ClearNoon" else
                      [5.0] if session_dir.name == "session_ClearNoon_repeat" else
                      [3.0, 3.0, 3.0])
            samples = _sample_rows(errors)
            has_mask = session_dir.name != "session_ClearNoon_repeat"
            for row in samples:
                row["track_id"] = 2 if session_dir.name == "session_ClearNoon_repeat" else 1
                row["visibility_mask_available"] = has_mask
                row["visible_gt"] = has_mask
                row["yolo_detected_visible_gt"] = (
                    session_dir.name == "session_ClearNoon" if has_mask else None)
            rows_out.extend(samples)
            return {"session": str(session_dir), "frames_scored": len(errors),
                    "visibility_mask_frames_aligned": len(errors) if has_mask else 0}

        with patch("monitor.qt.weather_workflow_evaluator.evaluate_session", fake_evaluate):
            summary = evaluate_weather_workflow(root)
        self.assertEqual(summary["session_count"], 3)
        self.assertEqual(summary["weather_results"]["ClearNoon"]["session_count"], 2)
        self.assertEqual(summary["common_samples"], 5)
        self.assertEqual(summary["eligible_gt_opportunities"], 5)
        self.assertEqual(summary["methods"]["camera_tracking"]["mae_m"], 3.0)
        self.assertEqual(summary["methods"]["camera_tracking"]["id_switches"], 0)
        self.assertEqual(summary["visibility_sessions"], 2)
        self.assertEqual(summary["visibility"]["visible_gt_opportunities"], 4)
        self.assertEqual(summary["visibility"]["yolo_recall"], 0.25)
        self.assertEqual(len(summary["skipped_sessions"]), 1)
        self.assertTrue(Path(summary["report_path"]).is_file())
        detailed = Path(summary["detailed_report_path"])
        self.assertTrue(detailed.is_file())
        self.assertTrue(Path(summary["report_path"]).read_text(encoding="utf-8").startswith(
            "# 날씨별·통합 거리 추정 결과"))
        self.assertIn("날씨별 가시성 마스크 기준 YOLO 검출률",
                      Path(summary["report_path"]).read_text(encoding="utf-8"))
        detailed_text = detailed.read_text(encoding="utf-8")
        self.assertIn("## 날씨별 상세 분석", detailed_text)
        self.assertIn("### ClearNoon 거리 추정 결과", detailed_text)
        self.assertIn("#### 거리 구간별 결과", detailed_text)
        self.assertIn("### HardRainNoon 거리 추정 결과", detailed_text)
        self.assertTrue(Path(summary["weather_results"]["ClearNoon"]["report_path"]).is_file())
        self.assertTrue(Path(summary["weather_results"]["HardRainNoon"]["report_path"]).is_file())


if __name__ == "__main__":
    unittest.main()
