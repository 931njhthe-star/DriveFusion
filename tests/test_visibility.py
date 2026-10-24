"""Instance-mask visibility is a separate denominator from projected GT."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import unittest
from uuid import uuid4

from PySide6.QtGui import QColor, QImage

from collector.live import SessionRecorder, VisibilityMaskFrame
from core.yolo11_detector import Box
from monitor.qt.experiment_evaluator import _summary_for_rows, evaluate_session
from monitor.qt.positioning import _project_actor_box
from monitor.qt.visibility import match_visible_detections, visible_vehicle_boxes


def _mask(width: int = 64, height: int = 48) -> QImage:
    image = QImage(width, height, QImage.Format.Format_ARGB32)
    image.fill(QColor(0, 0, 0, 255))
    for y in range(10, 25):
        for x in range(8, 28):
            image.setPixelColor(x, y, QColor(14, 7, 1, 255))  # actor 263
    for y in range(10, 13):
        for x in range(35, 40):
            image.setPixelColor(x, y, QColor(14, 8, 1, 255))  # actor 264
    return image


class VisibilityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.folder = Path(__file__).parent / f"_visibility_{uuid4().hex}"
        self.folder.mkdir()

    def tearDown(self) -> None:
        shutil.rmtree(self.folder)

    def test_mask_pixel_counts_and_visible_yolo_match(self) -> None:
        path = self.folder / "mask.png"
        self.assertTrue(_mask().save(str(path), "PNG"))
        projected = {
            263: {"x": 5, "y": 7, "width": 26, "height": 22},
            264: {"x": 32, "y": 8, "width": 12, "height": 10},
        }
        visible = visible_vehicle_boxes(path, projected, 64, 48)
        self.assertEqual(visible[263]["pixels"], 300)
        self.assertEqual(visible[263]["box"],
                         {"x": 8, "y": 10, "width": 20, "height": 15})
        self.assertTrue(visible[263]["eligible"])
        self.assertEqual(visible[264]["pixels"], 15)
        self.assertFalse(visible[264]["eligible"])
        boxes = [Box(x=8, y=10, width=20, height=15,
                     class_id=2, confidence=0.9)]
        self.assertEqual(match_visible_detections(visible, boxes), {263})

    def test_recorder_writes_lossless_mask_and_frame_index(self) -> None:
        image = _mask()
        recorder = SessionRecorder(self.folder, "127.0.0.1", 2000, "hero")
        session = recorder.start_session("standard")
        recorder.record_visibility_mask(VisibilityMaskFrame(
            frame=123, timestamp=4.1, received_at=5.2,
            width=64, height=48, bgra=bytes(image.bits()), sensor_id=9,
        ))
        recorder.stop_session()
        index = [json.loads(line) for line in
                 (session / "visibility_masks.jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertEqual([item["frame"] for item in index], [123])
        saved = session / index[0]["image"]
        self.assertTrue(saved.is_file())
        visible = visible_vehicle_boxes(saved, {
            263: {"x": 5, "y": 7, "width": 26, "height": 22},
        }, 64, 48)
        self.assertEqual(visible[263]["pixels"], 300)

    def test_visibility_recall_ignores_missing_masks(self) -> None:
        rows = []
        for frame, available, visible, detected in (
            (1, True, True, True), (2, True, True, False),
            (3, True, False, False), (4, False, None, None),
        ):
            for method in ("camera_tracking", "vision_radar_fusion",
                           "radar_tracking_fusion"):
                rows.append({
                    "frame": frame, "gt_actor_id": 263, "gt_distance_m": 12.0,
                    "method": method, "track_id": None, "observed": False,
                    "detection_matched": False, "error_m": None,
                    "predicted_error_m": None, "distance_error_signed_m": None,
                    "position_error_m": None, "radar_used": False,
                    "visibility_mask_available": available, "visible_gt": visible,
                    "yolo_detected_visible_gt": detected,
                })
        result = _summary_for_rows(rows)["visibility"]
        self.assertEqual(result["mask_aligned_projected_gt"], 3)
        self.assertEqual(result["visible_gt_opportunities"], 2)
        self.assertEqual(result["yolo_detected_visible_gt"], 1)
        self.assertEqual(result["yolo_recall"], 0.5)

    def test_legacy_rows_have_no_detection_rate(self) -> None:
        rows = [{
            "frame": 1, "gt_actor_id": 263, "gt_distance_m": 12.0,
            "method": method, "track_id": None, "observed": False,
            "detection_matched": False, "error_m": None,
            "predicted_error_m": None, "distance_error_signed_m": None,
            "position_error_m": None, "radar_used": False,
        } for method in ("camera_tracking", "vision_radar_fusion",
                         "radar_tracking_fusion")]
        result = _summary_for_rows(rows)["visibility"]
        self.assertEqual(result["status"], "not_recorded")
        self.assertIsNone(result["yolo_recall"])

    def test_session_evaluation_reads_mask_and_reports_detection_rate(self) -> None:
        session = self.folder / "session"
        session.mkdir()
        zero = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
        def pose(x: float, z: float = 0.0) -> dict:
            return {"location": {"x": x, "y": 0.0, "z": z}, "rotation": zero}
        def actor(actor_id: int, x: float) -> dict:
            return {"actor_id": actor_id, "type_id": "vehicle.test.car",
                    "transform": pose(x),
                    "bounding_box": {"location": {"x": 0.0, "y": 0.0, "z": 0.75},
                                     "extent": {"x": 1.0, "y": 0.8, "z": 0.75}}}
        target = actor(263, 12.0)
        camera = SimpleNamespace(width=640, height=480, fov=90.0,
                                 sensor_transform=pose(0.0, 2.4))
        box = _project_actor_box(target, camera)
        self.assertIsNotNone(box)
        left, top = int(box["x"]) + 1, int(box["y"]) + 1
        image = QImage(640, 480, QImage.Format.Format_ARGB32)
        image.fill(QColor(0, 0, 0, 255))
        for y in range(top, top + 30):
            for x in range(left, left + 30):
                image.setPixelColor(x, y, QColor(14, 7, 1, 255))
        mask_dir = session / "visibility_masks"
        mask_dir.mkdir()
        self.assertTrue(image.save(str(mask_dir / "1_00000001.png"), "PNG"))
        (session / "session.json").write_text(
            json.dumps({"recording_mode": "tracking"}), encoding="utf-8")
        (session / "visibility_masks.jsonl").write_text(json.dumps({
            "kind": "visibility_mask", "frame": 1, "image": "visibility_masks/1_00000001.png",
        }) + "\n", encoding="utf-8")
        (session / "measurements.jsonl").write_text(json.dumps({
            "kind": "camera", "sensor_id": 10, "frame": 1,
            "simulation_timestamp": 1.0, "width": 640, "height": 480,
            "fov": 90.0, "sensor_transform": camera.sensor_transform,
        }) + "\n", encoding="utf-8")
        (session / "ground_truth.jsonl").write_text(json.dumps({
            "kind": "ground_truth", "frame": 1, "ego_actor_id": 1,
            "ego_transform": pose(0), "ego_ground_z": 0.0,
            "vehicles": [actor(1, 0), target],
        }) + "\n", encoding="utf-8")
        (session / "detections.jsonl").write_text(json.dumps({
            "kind": "detection", "sensor_id": 10, "frame": 1,
            "receive_timestamp": 1.0,
            "boxes": [{"x": left, "y": top, "width": 30, "height": 30,
                       "class_id": 2, "confidence": 0.9}],
        }) + "\n", encoding="utf-8")
        summary = evaluate_session(session)
        visibility = summary["visibility"]
        self.assertEqual(summary["visibility_mask_frames_aligned"], 1)
        self.assertEqual(visibility["visible_gt_opportunities"], 1)
        self.assertEqual(visibility["yolo_detected_visible_gt"], 1)
        self.assertEqual(visibility["yolo_recall"], 1.0)
        self.assertIn("YOLO 검출률", Path(summary["report_path"]).read_text(
            encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
