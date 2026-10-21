from __future__ import annotations

import json
from pathlib import Path
from queue import Queue
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from collector.live import CameraFrame, CarlaCollector, DetectionFrame, LiveState, SessionRecorder
from core.yolo11_detector import Box
from monitor.qt.positioning import (
    PositionAccuracyAccumulator,
    PositioningEngine,
    RadarBoxMatch,
    _project_actor_box,
    _rotate_local_to_world,
    _rotate_world_to_local,
    match_radar_to_boxes,
    write_accuracy_summary,
)
from monitor.qt.tracker import MultiObjectTracker, TrackedObject


def _transform(x: float = 0.0, y: float = 0.0, z: float = 0.0,
               yaw: float = 0.0) -> dict:
    return {
        "location": {"x": x, "y": y, "z": z},
        "rotation": {"pitch": 0.0, "yaw": yaw, "roll": 0.0},
    }


class PositioningTests(unittest.TestCase):
    def test_camera_radar_imu_gnss_spawn_together_at_fifteen_fps(self) -> None:
        collector = CarlaCollector(LiveState(), SessionRecorder(
            Path("unused"), "127.0.0.1", 2000, "hero"
        ))
        client = MagicMock()
        world = MagicMock()
        blueprints: dict[str, MagicMock] = {}
        world.get_blueprint_library().find.side_effect = (
            lambda name: blueprints.setdefault(name, MagicMock())
        )
        world.get_actor.side_effect = lambda _actor_id: MagicMock()
        client.apply_batch_sync.return_value = [
            SimpleNamespace(actor_id=actor_id, error="") for actor_id in range(10, 14)
        ]
        collector._spawn_sensor = MagicMock()

        with patch("collector.live.carla.command.SpawnActor", side_effect=lambda *args: args):
            collector._attach_sensors(client, world, SimpleNamespace(id=7))

        self.assertEqual(len(client.apply_batch_sync.call_args.args[0]), 4)
        self.assertIs(client.apply_batch_sync.call_args.args[1], False)
        for name in ("sensor.camera.rgb", "sensor.other.radar",
                     "sensor.other.imu", "sensor.other.gnss"):
            attributes = dict(call.args for call in blueprints[name].set_attribute.call_args_list)
            self.assertEqual(attributes["sensor_tick"], "0.066666")

    def test_sensor_context_requires_exact_camera_frame(self) -> None:
        state = LiveState()
        state.update("radar", {"frame": 99})
        state.update("radar", {"frame": 101})
        state.update("ground_truth", {"frame": 100})

        radar, radar_delta, truth, truth_delta = state.position_context(100)

        self.assertIsNone(radar)
        self.assertIsNone(radar_delta)
        self.assertEqual(truth["frame"], 100)
        self.assertEqual(truth_delta, 0)

        state.update("radar", {"frame": 100})
        radar, radar_delta, _, _ = state.position_context(100)
        self.assertEqual(radar["frame"], 100)
        self.assertEqual(radar_delta, 0)

    def test_sensor_context_waits_for_same_frame_radar(self) -> None:
        state = LiveState()
        state.update("ground_truth", {"frame": 100})
        state.update("radar", {"frame": 99})
        entered = threading.Event()
        result = []

        def reader() -> None:
            entered.set()
            result.append(state.wait_for_position_context(100, timeout_s=1.0))

        worker = threading.Thread(target=reader)
        worker.start()
        self.assertTrue(entered.wait(1.0))
        state.update("radar", {"frame": 100})
        worker.join(1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(result[0][0]["frame"], 100)
        self.assertEqual(result[0][1], 0)

    def test_carla_rotation_round_trip(self) -> None:
        local = (10.0, 2.0, 1.0)
        world = _rotate_local_to_world(local, _transform(yaw=90)["rotation"])
        self.assertAlmostEqual(world[0], -2.0, places=6)
        self.assertAlmostEqual(world[1], 10.0, places=6)
        recovered = _rotate_world_to_local(world, _transform(yaw=90)["rotation"])
        for actual, expected in zip(recovered, local):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_projected_radar_return_matches_detection_box(self) -> None:
        camera = SimpleNamespace(
            width=640, height=480, fov=90.0,
            sensor_transform=_transform(z=2.0),
        )
        radar = {
            "frame": 100,
            "sensor_transform": _transform(x=2.2, z=0.7),
            "points": [{
                "distance_m": 8.0,
                "azimuth_rad": 0.0,
                "altitude_rad": 0.0,
                "relative_velocity_mps": 1.5,
            }],
        }
        truth = {"ego_transform": _transform()}
        box = SimpleNamespace(x=285.0, y=245.0, width=70.0, height=90.0)

        matches = match_radar_to_boxes(camera, [box], radar, truth)

        self.assertIsNotNone(matches[0])
        assert matches[0] is not None
        self.assertAlmostEqual(matches[0].x_ego_m, 10.2, places=5)
        self.assertAlmostEqual(matches[0].range_m, 10.2, places=5)
        self.assertEqual(matches[0].point_count, 1)

    def test_radar_match_rejects_different_frame(self) -> None:
        camera = SimpleNamespace(
            frame=100, width=640, height=480, fov=90.0,
            sensor_transform=_transform(z=2.0),
        )
        box = SimpleNamespace(x=285.0, y=245.0, width=70.0, height=90.0)
        radar = {
            "frame": 99,
            "sensor_transform": _transform(x=2.2, z=0.7),
            "points": [{"distance_m": 8.0, "azimuth_rad": 0.0,
                        "altitude_rad": 0.0, "relative_velocity_mps": 1.5}],
        }
        self.assertEqual(match_radar_to_boxes(camera, [box], radar,
                         {"ego_transform": _transform()}), (None,))

    def test_radar_match_uses_nearest_supported_cluster(self) -> None:
        camera = SimpleNamespace(frame=100, width=640, height=480, fov=90.0,
                                 sensor_transform=_transform(z=2.4))
        box = SimpleNamespace(x=285.0, y=230.0, width=70.0, height=106.0)
        radar = {"frame": 100, "sensor_transform": _transform(x=2.2, z=0.7),
                 "points": [{"distance_m": distance, "azimuth_rad": 0.0,
                             "altitude_rad": 0.0, "relative_velocity_mps": 0.0}
                            for distance in (4.0, 5.9, 6.0, 6.1)]}
        truth = {"ego_transform": _transform(), "ego_ground_z": 0.0}

        match = match_radar_to_boxes(camera, [box], radar, truth)[0]

        self.assertIsNotNone(match)
        self.assertEqual(match.point_count, 3)
        self.assertAlmostEqual(match.x_ego_m, 8.2)
        self.assertAlmostEqual(match.range_spread_m, 0.1)

    def test_distance_selects_radar_only_for_supported_cluster_within_one_meter(self) -> None:
        camera = SimpleNamespace(frame=100, timestamp=5.0, width=640, height=480,
                                 fov=90.0, sensor_transform=_transform(z=2.4))
        box = SimpleNamespace(x=285.0, y=230.0, width=70.0, height=106.0)
        ego = {"actor_id": 1, "bounding_box": {
            "location": {"x": 0.0, "y": 0.0, "z": 0.75},
            "extent": {"x": 0.5, "y": 0.5, "z": 0.75}},
            "transform": _transform()}
        truth = {"frame": 100, "ego_actor_id": 1, "ego_transform": _transform(),
                 "ego_ground_z": 0.0, "vehicles": [ego]}
        track = SimpleNamespace(track_id=1, class_id=2, x=285, y=230,
                                width=70, height=106, detection_index=0,
                                predicted=False, missed_frames=0)

        def result(distances: tuple[float, ...]) -> dict:
            radar = {"frame": 100, "sensor_transform": _transform(x=2.2, z=0.7),
                     "points": [{"distance_m": distance, "azimuth_rad": 0.0,
                                 "altitude_rad": 0.0, "relative_velocity_mps": 0.0}
                                for distance in distances]}
            return PositioningEngine().estimate(camera, [box], [track], radar, truth, 0, 0)["objects"][0]

        accepted = result((5.9, 6.0, 6.1))
        self.assertEqual(accepted["distance_source"], "radar_near_cluster")
        self.assertEqual(accepted["estimated_distance_m"], accepted["radar_distance_m"])
        self.assertLess(abs(accepted["radar_distance_m"] - accepted["camera_distance_m"]), 1.0)

        sparse = result((6.0,))
        self.assertEqual(sparse["distance_source"], "camera")
        self.assertEqual(sparse["distance_rejection_reason"], "insufficient_radar_points")

        disagreed = result((7.9, 8.0, 8.1))
        self.assertEqual(disagreed["distance_source"], "camera")
        self.assertEqual(disagreed["distance_rejection_reason"], "camera_radar_distance_disagreement")
        self.assertEqual(disagreed["estimated_distance_m"], disagreed["camera_distance_m"])

    def test_radar_match_rejects_far_point_inside_vehicle_image(self) -> None:
        camera = SimpleNamespace(
            frame=100, width=640, height=480, fov=90.0,
            sensor_transform=_transform(z=2.4),
        )
        box = SimpleNamespace(x=285.0, y=200.0, width=70.0, height=140.0)
        truth = {"ego_transform": _transform(), "ego_ground_z": 0.0}
        radar = {
            "frame": 100, "sensor_transform": _transform(x=2.2, z=0.7),
            "points": [{"distance_m": 38.0, "azimuth_rad": 0.0,
                        "altitude_rad": 0.0, "relative_velocity_mps": 0.0}],
        }

        self.assertEqual(match_radar_to_boxes(camera, [box], radar, truth), (None,))

    def test_single_radar_point_has_low_weight_and_large_jump_falls_back(self) -> None:
        engine = PositioningEngine()
        truth = {"ego_transform": _transform(), "ego_ground_z": 0.0,
                 "ego_actor_id": 1, "vehicles": []}
        track = SimpleNamespace(track_id=1, class_id=2, x=285, y=245,
                                width=70, height=91, detection_index=0,
                                predicted=False, missed_frames=0)

        def estimate(frame: int, timestamp: float, box_bottom: float,
                     radar_distance: float) -> dict:
            truth["frame"] = frame
            camera = SimpleNamespace(
                frame=frame, timestamp=timestamp, width=640, height=480,
                fov=90.0, sensor_transform=_transform(z=2.4),
            )
            box = SimpleNamespace(x=285.0, y=230.0, width=70.0,
                                  height=box_bottom - 230.0)
            radar = {
                "frame": frame, "sensor_transform": _transform(x=2.2, z=0.7),
                "points": [{"distance_m": radar_distance, "azimuth_rad": 0.0,
                            "altitude_rad": 0.0, "relative_velocity_mps": 0.0}],
            }
            return engine.estimate(camera, [box], [track], radar, truth, 0, 0)["objects"][0]

        first = estimate(100, 5.0, 336.0, 8.0)
        self.assertTrue(first["radar_used"])
        camera_x = first["camera_xy_ego_m"]["forward_m"]
        radar_x = first["radar_xy_ego_m"]["forward_m"]
        self.assertAlmostEqual(first["fused_xy_ego_m"]["forward_m"],
                               0.75 * camera_x + 0.25 * radar_x)

        second = estimate(101, 5.067, 295.0, 11.8)
        self.assertFalse(second["radar_used"])
        self.assertEqual(second["radar_rejection_reason"], "implausible_position_jump")
        self.assertEqual(second["fused_xy_ego_m"], second["camera_xy_ego_m"])

    def test_position_engine_compares_camera_and_fused_estimates_to_truth(self) -> None:
        camera = SimpleNamespace(
            frame=100, timestamp=5.0, width=640, height=480, fov=90.0,
            sensor_transform=_transform(z=2.4),
        )
        actor = {
            "actor_id": 42,
            "type_id": "vehicle.tesla.model3",
            "transform": _transform(x=10.0),
            "bounding_box": {
                "location": {"x": 0.0, "y": 0.0, "z": 0.75},
                "extent": {"x": 2.0, "y": 0.9, "z": 0.75},
            },
        }
        truth = {
            "frame": 100,
            "ego_actor_id": 1,
            "ego_transform": _transform(),
            "ego_ground_z": 0.0,
            "vehicles": [actor],
        }
        projected = _project_actor_box(actor, camera)
        self.assertIsNotNone(projected)
        assert projected is not None
        box = SimpleNamespace(
            x=projected["x"], y=projected["y"],
            width=projected["width"], height=projected["height"],
            class_id=2, confidence=0.95,
        )
        track = SimpleNamespace(
            track_id=3, class_id=2, x=int(box.x), y=int(box.y),
            width=int(box.width), height=int(box.height), detection_index=0,
            predicted=False, missed_frames=0, radar_range_m=9.0,
            radar_range_rate_mps=-0.5, radar_age_s=0.0,
        )
        engine = PositioningEngine()

        result = engine.estimate(
            camera, [box], [track],
            radar={"frame": 100, "sensor_transform": _transform(), "points": [{
                "distance_m": 9.0, "azimuth_rad": 0.0, "altitude_rad": 0.0,
                "relative_velocity_mps": 0.5,
            }]},
            truth=truth, radar_delta_frames=0, truth_delta_frames=0,
        )

        self.assertEqual(result["ground_truth_frame"], 100)
        # The same-frame radar return supplies the radar-assisted range.
        target = result["objects"][0]
        self.assertEqual(target["ground_truth_actor_id"], 42)
        self.assertTrue(target["radar_used"])
        self.assertIsNotNone(target["camera_error_m"])
        self.assertIsNotNone(target["fusion_error_m"])
        self.assertEqual(result["live_summary"]["camera"]["samples"], 1)

    def test_position_engine_falls_back_to_camera_without_radar_match(self) -> None:
        camera = SimpleNamespace(
            frame=100, timestamp=5.0, width=640, height=480, fov=90.0,
            sensor_transform=_transform(z=2.4),
        )
        actor = {
            "actor_id": 42,
            "type_id": "vehicle.tesla.model3",
            "transform": _transform(x=10.0),
            "bounding_box": {
                "location": {"x": 0.0, "y": 0.0, "z": 0.75},
                "extent": {"x": 2.0, "y": 0.9, "z": 0.75},
            },
        }
        truth = {
            "frame": 100, "ego_actor_id": 1,
            "ego_transform": _transform(), "ego_ground_z": 0.0,
            "vehicles": [actor],
        }
        projected = _project_actor_box(actor, camera)
        assert projected is not None
        box = SimpleNamespace(**projected, class_id=2, confidence=0.95)
        track = SimpleNamespace(
            track_id=1, class_id=2, x=int(box.x), y=int(box.y),
            width=int(box.width), height=int(box.height), detection_index=0,
            predicted=False, missed_frames=0, radar_range_m=None, radar_age_s=None,
        )

        result = PositioningEngine().estimate(
            camera, [box], [track], radar=None, truth=truth,
            radar_delta_frames=None, truth_delta_frames=0,
        )

        target = result["objects"][0]
        self.assertFalse(target["radar_used"])
        self.assertEqual(target["fusion_source"], "camera_only")
        self.assertEqual(target["fused_xy_ego_m"], target["camera_xy_ego_m"])
        self.assertEqual(target["fusion_error_m"], target["camera_error_m"])

    def test_accuracy_accumulator_accepts_unavailable_estimate(self) -> None:
        accumulator = PositionAccuracyAccumulator()
        result = accumulator.update([
            {"ground_truth_actor_id": 5, "camera_error_m": None,
             "fusion_error_m": None, "radar_used": False},
        ], visible_truth=1, radar_delta=0)
        self.assertEqual(result["camera"]["samples"], 0)
        self.assertEqual(result["camera_matches"], 0)


class RadarTrackingTests(unittest.TestCase):
    def test_radar_range_resolves_ambiguous_track_assignment(self) -> None:
        tracker = MultiObjectTracker()
        initial = [Box(100, 100, 40, 40, 2, 0.9), Box(100, 100, 40, 40, 2, 0.9)]
        tracker.update(initial, 1.0, [
            RadarBoxMatch(10.0, 0.0, 10.0, 0.0, 1),
            RadarBoxMatch(20.0, 0.0, 20.0, 0.0, 1),
        ])

        tracks = tracker.update(initial, 1.05, [
            RadarBoxMatch(20.0, 0.0, 20.0, 0.0, 1),
            RadarBoxMatch(10.0, 0.0, 10.0, 0.0, 1),
        ])

        by_id = {track.track_id: track for track in tracks}
        self.assertEqual(by_id[1].detection_index, 1)
        self.assertEqual(by_id[2].detection_index, 0)
        self.assertAlmostEqual(by_id[1].radar_range_m or 0.0, 10.0)


class RecorderQueueTests(unittest.TestCase):
    def test_tracking_frame_records_detection_position_and_tracks_together(self) -> None:
        recorder = SessionRecorder(Path("unused"), "127.0.0.1", 2000, "hero")
        recorder._enabled = True
        recorder._mode = "tracking"
        recorder._records = Queue()
        frame = CameraFrame(100, 5.0, 1.0, 4, 4, b"", sensor_id=11)
        track = TrackedObject(1, 0, 0, 2, 2, 2, 0.9, 0, False, 0, 1, 0.0, 0.0)
        positioning = {"schema_version": 1, "kind": "positioning", "frame": 100}

        recorder.record_detection(DetectionFrame(
            frame, (Box(0, 0, 2, 2, 2, 0.9),), 1.5, (track,), positioning
        ))

        rows = [recorder._records.get_nowait() for _ in range(3)]
        self.assertEqual([row["kind"] for row in rows], ["detection", "positioning", "tracking"])
        self.assertEqual([row["frame"] for row in rows], [100, 100, 100])

    def test_radar_fusion_frame_records_visual_and_fused_tracks_separately(self) -> None:
        recorder = SessionRecorder(Path("unused"), "127.0.0.1", 2000, "hero")
        recorder._enabled = True
        recorder._mode = "radar_fusion"
        recorder._records = Queue()
        frame = CameraFrame(100, 5.0, 1.0, 4, 4, b"", sensor_id=11)
        fused_track = TrackedObject(1, 0, 0, 2, 2, 2, 0.9, 0, False, 0, 1, 0.0, 0.0,
                                    radar_range_m=8.0)
        vision_track = TrackedObject(7, 1, 0, 2, 2, 2, 0.8, 0, False, 0, 1, 0.0, 0.0)
        positioning = {"schema_version": 1, "kind": "positioning", "frame": 100}

        recorder.record_detection(DetectionFrame(
            frame, (Box(0, 0, 2, 2, 2, 0.9),), 1.5,
            (fused_track,), positioning, vision_tracks=(vision_track,),
        ))

        rows = [recorder._records.get_nowait() for _ in range(5)]
        by_kind = {row["kind"]: row for row in rows}
        self.assertEqual(set(by_kind), {
            "detection", "positioning", "radar_fusion", "tracking", "vision_tracking",
        })
        self.assertEqual(by_kind["tracking"]["tracks"][0]["track_id"], 1)
        self.assertEqual(by_kind["tracking"]["tracks"][0]["radar_range_m"], 8.0)
        self.assertEqual(by_kind["vision_tracking"]["tracks"][0]["track_id"], 7)

    def test_standard_session_filters_yolo_and_ground_truth_rows(self) -> None:
        recorder = SessionRecorder(Path("unused"), "127.0.0.1", 2000, "hero")
        recorder._enabled = True
        recorder._mode = "standard"
        recorder._records = Queue()

        recorder.record_many((
            {"kind": "ground_truth", "frame": 100},
            {"kind": "positioning", "frame": 100},
            {"kind": "vehicle", "frame": 100},
        ))

        self.assertEqual(recorder._records.qsize(), 1)
        self.assertEqual(recorder._records.get_nowait()["kind"], "vehicle")


class AccuracySummaryTests(unittest.TestCase):
    def test_summary_reports_camera_fusion_and_fallback_errors(self) -> None:
        session = Path(__file__).parent
        positions_path = session / "positions.jsonl"
        summary_path = session / "accuracy_summary.json"
        records = [{
            "visible_ground_truth": 2,
            "radar_delta_frames": 0,
            "objects": [
                {"ground_truth_actor_id": 10, "camera_xy_ego_m": {"forward_m": 8.0, "right_m": 0.0},
                 "camera_error_m": 2.0, "fusion_error_m": 1.0, "radar_used": True,
                 "fusion_source": "camera+radar"},
                {"ground_truth_actor_id": 11, "camera_xy_ego_m": None,
                 "camera_error_m": None, "fusion_error_m": 3.0, "radar_used": True,
                 "fusion_source": "radar"},
            ],
        }]
        try:
            positions_path.write_text(
                "".join(json.dumps(item) + "\n" for item in records), encoding="utf-8"
            )
            summary = write_accuracy_summary(session)
            self.assertEqual(summary["frames"], 1)
            self.assertEqual(summary["camera_only_position_error"]["rmse_m"], 2.0)
            self.assertAlmostEqual(summary["radar_fusion_position_error"]["rmse_m"], (5.0 ** 0.5))
            self.assertEqual(summary["camera_plus_radar_position_error"]["samples"], 1)
            self.assertEqual(summary["radar_only_fallback_position_error"]["samples"], 1)
            self.assertTrue(summary_path.is_file())
        finally:
            positions_path.unlink(missing_ok=True)
            summary_path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
