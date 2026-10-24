from __future__ import annotations

from types import SimpleNamespace
import unittest

from monitor.qt.experiment_evaluator import (
    _band_for_distance, _eligible_truth, _footprint, _footprint_distance,
    _point_footprint_distance, _selected_rows, _stats, _summary_for_rows,
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
    def test_gt_opportunities_include_distances_outside_legacy_range(self) -> None:
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

        self.assertEqual(set(eligible), {2, 3, 4, 5})
        self.assertAlmostEqual(eligible[2][2], 2.99)
        self.assertEqual(eligible[3][2], 3.0)
        self.assertEqual(eligible[4][2], 15.0)
        self.assertAlmostEqual(eligible[5][2], 15.01)
        self.assertEqual(ego_models, {"vehicle.test.1"})
        self.assertEqual(target_models, {f"vehicle.test.{i}" for i in (2, 3, 4, 5)})

    def test_distance_bands_are_disjoint_and_legacy_range_is_inclusive(self) -> None:
        self.assertEqual([_band_for_distance(d) for d in (0, 3, 5, 10, 15, 20, 30)],
                         ["0~3m", "3~5m", "5~10m", "10~15m", "15~20m", "20~30m", "30m 이상"])
        rows = [{"gt_distance_m": distance} for distance in (2.99, 3.0, 15.0, 15.01)]
        self.assertEqual(_selected_rows(rows, 3.0, 15.0), rows[1:3])
        self.assertEqual(_selected_rows(rows, 3.0, 15.0, include_max=False), rows[1:2])

    def test_final_distance_uses_observed_or_retained_track_once(self) -> None:
        rows = []
        for frame, error, observed, predicted_error, signed in (
            (1, 1.0, True, None, 1.0),
            (2, None, True, None, None),
            (3, None, False, 2.0, -2.0),
        ):
            for method in ("camera_tracking", "vision_radar_fusion", "radar_tracking_fusion"):
                rows.append({"frame": frame, "gt_actor_id": 7, "method": method,
                             "gt_distance_m": 4.0,
                             "error_m": error, "distance_error_signed_m": error,
                             "tracking_distance_error_signed_m": signed,
                             "predicted_error_m": predicted_error,
                             "position_error_m": None, "track_id": None,
                             "observed": observed, "detection_matched": observed,
                             "radar_used": False})
        summary = _summary_for_rows(rows)
        self.assertEqual(summary["eligible_gt_opportunities"], 3)
        self.assertEqual(summary["common_samples"], 2)
        self.assertEqual(summary["methods"]["camera_tracking"]["observed_matches"], 2)
        self.assertEqual(summary["methods"]["camera_tracking"]["coverage"], 0.5)
        self.assertEqual(summary["methods"]["camera_tracking"]["observed_distance_samples"], 1)
        self.assertEqual(summary["methods"]["camera_tracking"]["predicted_distance_samples"], 1)
        self.assertEqual(summary["methods"]["camera_tracking"]["distance_samples"], 2)
        self.assertEqual(summary["methods"]["camera_tracking"]["mae_m"], 1.5)
        self.assertAlmostEqual(summary["methods"]["camera_tracking"]["rmse_m"],
                               (2.5) ** 0.5)
        self.assertEqual(summary["methods"]["camera_tracking"]["distance_bias_m"], -0.5)
        self.assertEqual(summary["methods"]["camera_tracking"]["observed_mae_m"], 1.0)
        self.assertEqual(summary["common_sample_mae_m"]["camera_tracking"], 1.5)

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

    def test_observation_takes_priority_if_both_error_fields_are_present(self) -> None:
        row = {
            "method": "camera_tracking", "error_m": 1.0,
            "predicted_error_m": 9.0, "distance_error_signed_m": 1.0,
            "tracking_distance_error_signed_m": 1.0,
            "observed": True, "detection_matched": True,
            "position_error_m": None, "track_id": None,
            "gt_actor_id": 7, "radar_used": False,
        }
        stats = _stats([row], "camera_tracking")
        self.assertEqual(stats["distance_samples"], 1)
        self.assertEqual(stats["observed_distance_samples"], 1)
        self.assertEqual(stats["predicted_distance_samples"], 0)
        self.assertEqual(stats["mae_m"], 1.0)

    def test_prediction_quality_rejections_are_separate_from_unavailable_projection(self) -> None:
        rows = []
        for reason in ("prediction_position_jump", "prediction_projection_unavailable"):
            rows.append({
                "method": "camera_tracking", "error_m": None,
                "predicted_error_m": None, "distance_error_signed_m": None,
                "position_error_m": None, "track_id": 7,
                "gt_actor_id": 3, "observed": False,
                "detection_matched": False, "radar_used": False,
                "distance_rejection_reason": reason,
            })
        stats = _stats(rows, "camera_tracking")
        self.assertEqual(stats["predicted_track_matches"], 2)
        self.assertEqual(stats["prediction_distance_unavailable"], 2)
        self.assertEqual(stats["prediction_distance_rejected"], 1)
        self.assertEqual(stats["distance_samples"], 0)

    def test_collinear_separated_footprints_are_not_treated_as_touching(self) -> None:
        ego = _actor(1, 0.0)
        target = _actor(2, 4.0)
        target["transform"] = _transform(4.0, y=1.0)
        ego_polygon = _footprint(ego, ego["transform"])
        target_polygon = _footprint(target, ego["transform"])
        self.assertAlmostEqual(_footprint_distance(ego_polygon, target_polygon), 3.0)

    def test_projected_gt_is_not_distance_completion_denominator(self) -> None:
        rows = [
            {"method": "camera_tracking", "gt_actor_id": 2, "track_id": 1,
             "observed": True, "detection_matched": True,
             "error_m": 1.0, "position_error_m": 1.0,
             "radar_used": False},
            {"method": "camera_tracking", "gt_actor_id": 3, "track_id": None,
             "observed": False, "detection_matched": False,
             "error_m": None, "position_error_m": None,
             "radar_used": False},
        ]

        stats = _stats(rows, "camera_tracking")

        self.assertEqual(stats["gt_opportunities"], 2)
        self.assertEqual(stats["position_samples"], 1)
        self.assertEqual(stats["observed_matches"], 1)
        self.assertEqual(stats["coverage"], 1.0)
        self.assertEqual(stats["projected_match_rate"], 0.5)


if __name__ == "__main__":
    unittest.main()
