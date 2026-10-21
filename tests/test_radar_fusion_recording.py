from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
from queue import Queue
import threading
import unittest

from PySide6.QtGui import QColor, QImage
from PySide6.QtWidgets import QApplication

from collector.live import SessionRecorder
from monitor.qt.yolo_overlay_generator import generate_comparison_video

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
_QT_APP = QApplication.instance() or QApplication([])


class RadarFusionRecordingTests(unittest.TestCase):
    def test_radar_fusion_session_writes_dedicated_jsonl(self) -> None:
        session = Path(__file__).parent
        measurements_path = session / "measurements.jsonl"
        fusion_path = session / "radar_fusion.jsonl"
        vision_path = session / "vision_tracks.jsonl"
        previous = {path: path.read_bytes() if path.exists() else None
                    for path in (measurements_path, fusion_path, vision_path)}
        try:
            records: Queue[dict] = Queue()
            records.put({"schema_version": 1, "kind": "radar_fusion", "frame": 9,
                         "objects": [{"track_id": 2, "radar_used": True}]})
            records.put({"schema_version": 1, "kind": "vision_tracking", "frame": 9,
                         "tracks": [{"track_id": 1}]})
            stop = threading.Event()
            stop.set()
            recorder = SessionRecorder(session, "127.0.0.1", 2000, "hero")
            recorder._run(session, records, Queue(), stop)

            written = [json.loads(line) for line in fusion_path.read_text(
                encoding="utf-8"
            ).splitlines()]
            self.assertEqual(written[0]["frame"], 9)
            vision_written = [json.loads(line) for line in vision_path.read_text(
                encoding="utf-8"
            ).splitlines()]
            self.assertEqual(vision_written[0]["tracks"][0]["track_id"], 1)
        finally:
            for path, data in previous.items():
                if data is None:
                    path.unlink(missing_ok=True)
                else:
                    path.write_bytes(data)

    @unittest.skipUnless(shutil.which("ffmpeg"), "ffmpeg is required for MP4 encoding")
    def test_comparison_video_encodes_all_views_from_legacy_fusion_data(self) -> None:
        session = Path(__file__).parent
        frame_paths = [session / f"comparison_test_frame_{index}.jpg" for index in (1, 2)]
        test_paths = [
            *frame_paths,
            session / "session.json",
            session / "measurements.jsonl",
            session / "detections.jsonl",
            session / "tracks.jsonl",
            session / "vision_tracks.jsonl",
            session / "positions.jsonl",
            session / "comparison_test.mp4",
        ]
        previous = {path: path.read_bytes() if path.exists() else None for path in test_paths}
        try:
            (session / "vision_tracks.jsonl").unlink(missing_ok=True)
            rows = []
            for frame_number in (1, 2):
                image_path = frame_paths[frame_number - 1]
                image = QImage(64, 48, QImage.Format.Format_RGB32)
                image.fill(QColor("#233344"))
                self.assertTrue(image.save(str(image_path), "JPEG", 90))
                rows.append({
                    "kind": "camera", "sensor_id": 7, "frame": frame_number,
                    "simulation_timestamp": (frame_number - 1) * 0.05,
                    "image": image_path.name,
                })
            (session / "measurements.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
            )
            (session / "session.json").write_text(
                json.dumps({"recording_mode": "radar_fusion"}), encoding="utf-8"
            )
            (session / "detections.jsonl").write_text(
                json.dumps({"sensor_id": 7, "frame": 1,
                            "boxes": [{"x": 5, "y": 5, "width": 20, "height": 20,
                                       "class_id": 2, "confidence": 0.9}]}) + "\n",
                encoding="utf-8",
            )
            (session / "tracks.jsonl").write_text(
                json.dumps({"sensor_id": 7, "frame": 1,
                            "tracks": [{"track_id": 3, "x": 5, "y": 5, "width": 20,
                                        "height": 20, "class_id": 2, "confidence": 0.9}]}) + "\n",
                encoding="utf-8",
            )
            (session / "positions.jsonl").write_text(
                json.dumps({"frame": 1, "objects": [{"track_id": 3, "radar_used": True,
                                                       "radar_range_m": 12.0,
                                                       "fused_xy_ego_m": {"forward_m": 11.8,
                                                                          "right_m": 0.5}}]}) + "\n",
                encoding="utf-8",
            )

            output, frame_count = generate_comparison_video(
                session, session / "comparison_test.mp4"
            )

            self.assertEqual(frame_count, 2)
            self.assertTrue(output.is_file())
            self.assertGreater(output.stat().st_size, 1000)
        finally:
            for path, data in previous.items():
                if data is None:
                    path.unlink(missing_ok=True)
                else:
                    path.write_bytes(data)


if __name__ == "__main__":
    unittest.main()
