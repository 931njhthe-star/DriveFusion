from __future__ import annotations

from types import SimpleNamespace
import unittest

from monitor.qt.experiment_evaluator import (
    _eligible_truth, _footprint, _footprint_distance, _point_footprint_distance, _stats,
)


def _transform(x: float = 0.0, z: float = 0.0, y: float = 0.0,
               yaw: float = 0.0) -> dict:
    return {
        "location": {"x": x, "y": y, "z": z},
        "rotation": {"pitch": 0.0, "yaw": yaw, "roll": 0.0},
    }


def _actor(actor_id: int, x: float) -> dict:
    return {
        "actor_id": actor_id,
        "type_id": f"vehicle.test.{actor_id}",
        "transform": _transform(x),
        "bounding_box": {
            "location": {"x": 0.0, "y": 0.0, "z": 0.75},
            "extent": {"x": 0.5, "y": 0.5, "z": 0.75},
        },
    }


class ExperimentEvaluatorTests(unittest.TestCase):
    def test_gt_distance_filter_includes_three_and_fifteen_meters(self) -> None:
        camera = SimpleNamespace(
            width=640, height=480, fov=90.0, sensor_transform=_transform(z=2.4)
        )
        truth = {
            "ego_actor_id": 1,
            "ego_transform": _transform(),
            "vehicles": [_actor(1, 0.0), _actor(2, 3.99), _actor(3, 4.0),
                         _actor(4, 16.0), _actor(5, 16.01)],
        }

        eligible, ego_models, target_models = _eligible_truth(camera, truth)

        self.assertEqual(set(eligible), {3, 4})
        self.assertEqual(eligible[3][2], 3.0)
        self.assertEqual(eligible[4][2], 15.0)
        self.assertEqual(ego_models, {"vehicle.test.1"})
        self.assertEqual(target_models, {"vehicle.test.3", "vehicle.test.4"})

    def test_clearance_uses_both_rotated_vehicle_footprints(self) -> None:
        ego = _actor(1, 0.0)
        target = _actor(2, 4.0)
        ego_polygon = _footprint(ego, ego["transform"])
        target_polygon = _footprint(target, ego["transform"])
        self.assertAlmostEqual(_footprint_distance(ego_polygon, target_polygon), 3.0)
        self.assertAlmostEqual(_point_footprint_distance((3.5, 0.0), ego_polygon), 3.0)

        target["transform"] = _transform(4.0, yaw=90.0)
        target_polygon = _footprint(target, ego["transform"])
        self.assertAlmostEqual(_footprint_distance(ego_polygon, target_polygon), 3.0)

        target["transform"] = _transform(0.0, y=0.8, yaw=45.0)
        target_polygon = _footprint(target, ego["transform"])
        self.assertEqual(_footprint_distance(ego_polygon, target_polygon), 0.0)

    def test_collinear_separated_footprints_are_not_treated_as_touching(self) -> None:
        ego = _actor(1, 0.0)
        target = _actor(2, 4.0)
        target["transform"] = _transform(4.0, y=1.0)
        ego_polygon = _footprint(ego, ego["transform"])
        target_polygon = _footprint(target, ego["transform"])
        self.assertAlmostEqual(_footprint_distance(ego_polygon, target_polygon), 3.0)

    def test_missing_positions_count_against_coverage(self) -> None:
        rows = [
            {"method": "camera_tracking", "gt_actor_id": 2, "track_id": 1,
             "observed": True, "error_m": 1.0, "radar_used": False},
            {"method": "camera_tracking", "gt_actor_id": 3, "track_id": None,
             "observed": False, "error_m": None, "radar_used": False},
        ]

        stats = _stats(rows, "camera_tracking")

        self.assertEqual(stats["gt_opportunities"], 2)
        self.assertEqual(stats["position_samples"], 1)
        self.assertEqual(stats["coverage"], 0.5)


if __name__ == "__main__":
    unittest.main()
