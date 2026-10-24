"""Radar-only observations correct retained tracks without becoming YOLO observations."""

from __future__ import annotations

import unittest

from core.yolo11_detector import Box
from monitor.qt.positioning import RadarBoxMatch
from monitor.qt.tracker import MultiObjectTracker


def _match(world_x: float, u: float) -> RadarBoxMatch:
    return RadarBoxMatch(
        x_ego_m=world_x, y_ego_m=0.0, range_m=world_x,
        range_rate_mps=0.0, point_count=3,
        world_x_m=world_x, world_y_m=0.0, u_px=u, v_px=110.0,
    )


def _points(world_x: float, u: float) -> tuple[dict[str, float], ...]:
    return tuple({
        "world_x_m": world_x + offset, "world_y_m": 0.0,
        "u": u + offset, "v": 110.0,
        "ground_u": u + offset, "ground_v": 110.0,
        "range_m": world_x + offset, "range_rate_mps": 0.0,
    } for offset in (-0.1, 0.0, 0.1))


class RadarTrackCorrectionTests(unittest.TestCase):
    @staticmethod
    def _prime(tracker: MultiObjectTracker, boxes: tuple[Box, ...],
               matches: tuple[RadarBoxMatch, ...]) -> None:
        for timestamp in (0.0, 0.03, 0.06):
            tracker.update(boxes, timestamp, matches)

    def test_radar_moves_track_during_yolo_gap_but_keeps_it_predicted(self) -> None:
        box = Box(100, 100, 40, 30, 2, 0.9)
        radar_tracker = MultiObjectTracker(max_lost_frames=3,
                                           radar_correction_enabled=True)
        vision_tracker = MultiObjectTracker(max_lost_frames=3)
        self._prime(radar_tracker, (box,), (_match(10.0, 110.0),))
        vision_tracker.update((box,), 0.0)

        corrected = radar_tracker.update((), 0.1, (), _points(10.4, 117.0))[0]
        visual = vision_tracker.update((), 0.1)[0]

        self.assertEqual(corrected.track_id, 1)
        self.assertTrue(corrected.predicted)
        self.assertTrue(corrected.radar_corrected)
        self.assertGreater(corrected.x, visual.x + 4)
        self.assertEqual(visual.x, 100)
        self.assertFalse(visual.radar_corrected)
        next_frame = radar_tracker.update((), 0.2)[0]
        self.assertFalse(next_frame.radar_corrected)

    def test_legacy_tracker_keeps_score_only_behavior(self) -> None:
        tracker = MultiObjectTracker()
        self._prime(tracker, (Box(100, 100, 40, 30, 2, 0.9),),
                    (_match(10.0, 110.0),))
        track = tracker.update((), 0.1, (), _points(10.4, 117.0))[0]
        self.assertTrue(track.predicted)
        self.assertFalse(track.radar_corrected)
        self.assertEqual(track.x, 100)

    def test_sparse_or_ambiguous_radar_does_not_correct(self) -> None:
        box = Box(100, 100, 40, 30, 2, 0.9)
        tracker = MultiObjectTracker(radar_correction_enabled=True)
        self._prime(tracker, (box,), (_match(10.0, 110.0),))
        self.assertFalse(tracker.update((), 0.1, (), _points(10.5, 130.0)[:1])[0]
                         .radar_corrected)

        tracker = MultiObjectTracker(radar_correction_enabled=True)
        self._prime(tracker,
            (box, Box(180, 100, 40, 30, 2, 0.9)),
            (_match(10.0, 110.0), _match(10.4, 190.0)),
        )
        predicted = tracker.update((), 0.1, (), _points(10.2, 150.0))
        self.assertTrue(all(not track.radar_corrected for track in predicted))

    def test_retention_limit_still_applies_to_radar_corrected_tracks(self) -> None:
        tracker = MultiObjectTracker(max_lost_frames=1,
                                     radar_correction_enabled=True)
        self._prime(tracker, (Box(100, 100, 40, 30, 2, 0.9),),
                    (_match(10.0, 110.0),))
        self.assertTrue(tracker.update((), 0.1, (), _points(10.1, 117.0)))
        self.assertEqual(tracker.update((), 0.2, (), _points(10.2, 130.0)), ())


if __name__ == "__main__":
    unittest.main()
