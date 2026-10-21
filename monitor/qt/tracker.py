"""Lightweight class-aware IoU and center-distance multi-object tracker."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from core.yolo11_detector import Box


@dataclass(frozen=True)
class TrackedObject:
    track_id: int
    x: int
    y: int
    width: int
    height: int
    class_id: int
    confidence: float
    detection_index: int | None
    predicted: bool
    missed_frames: int
    hits: int
    velocity_x_px_s: float
    velocity_y_px_s: float
    radar_range_m: float | None = None
    radar_range_rate_mps: float | None = None
    radar_age_s: float | None = None


@dataclass
class _Track:
    track_id: int
    x: float
    y: float
    width: float
    height: float
    class_id: int
    confidence: float
    last_update: float
    last_observed: float
    observed_center_x: float
    observed_center_y: float
    velocity_x_px_s: float = 0.0
    velocity_y_px_s: float = 0.0
    hits: int = 1
    missed_frames: int = 0
    radar_range_m: float | None = None
    radar_range_rate_mps: float | None = None
    radar_last_seen: float | None = None

    def predict(self, now: float) -> tuple[float, float, float, float]:
        dt = min(max(now - self.last_update, 0.0), 1.0)
        return (
            self.x + self.velocity_x_px_s * dt,
            self.y + self.velocity_y_px_s * dt,
            self.width,
            self.height,
        )


def _center(box: tuple[float, float, float, float]) -> tuple[float, float]:
    x, y, width, height = box
    return x + width / 2.0, y + height / 2.0


def _iou(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    lx, ly, lw, lh = left
    rx, ry, rw, rh = right
    intersection_width = max(0.0, min(lx + lw, rx + rw) - max(lx, rx))
    intersection_height = max(0.0, min(ly + lh, ry + rh) - max(ly, ry))
    intersection = intersection_width * intersection_height
    union = lw * lh + rw * rh - intersection
    return intersection / union if union > 0.0 else 0.0


class MultiObjectTracker:
    """Assign stable IDs using class, predicted overlap, and center distance."""

    def __init__(self, max_lost_frames: int = 10) -> None:
        self.max_lost_frames = max_lost_frames
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    def update(
        self,
        boxes: Sequence[Box],
        received_at: float,
        radar_matches: Sequence[object | None] = (),
    ) -> tuple[TrackedObject, ...]:
        now = float(received_at)
        tracks = list(self._tracks.values())
        predictions = {track.track_id: track.predict(now) for track in tracks}
        candidates: list[tuple[float, int, int]] = []

        for track in tracks:
            predicted_box = predictions[track.track_id]
            predicted_center = _center(predicted_box)
            distance_limit = max(40.0, math.hypot(predicted_box[2], predicted_box[3]) * 1.5)
            for detection_index, box in enumerate(boxes):
                if box.class_id != track.class_id:
                    continue
                measured = (float(box.x), float(box.y), float(box.width), float(box.height))
                measured_center = _center(measured)
                distance = math.hypot(
                    measured_center[0] - predicted_center[0],
                    measured_center[1] - predicted_center[1],
                )
                overlap = _iou(predicted_box, measured)
                if overlap < 0.05 and distance > distance_limit:
                    continue
                proximity = max(0.0, 1.0 - distance / distance_limit)
                score = overlap + proximity * 0.15
                radar = radar_matches[detection_index] if detection_index < len(radar_matches) else None
                if radar is not None and track.radar_range_m is not None:
                    elapsed = min(max(now - track.last_update, 0.0), 1.0)
                    predicted_range = track.radar_range_m
                    if track.radar_range_rate_mps is not None:
                        predicted_range += track.radar_range_rate_mps * elapsed
                    gate = max(6.0, predicted_range * 0.25)
                    range_error = abs(float(radar.range_m) - predicted_range)
                    score += 0.22 * (1.0 - min(range_error / gate, 1.0))
                candidates.append((score, track.track_id, detection_index))

        candidates.sort(key=lambda item: (-item[0], item[1], item[2]))
        matched_track_ids: set[int] = set()
        matched_detection_indices: set[int] = set()
        output: list[TrackedObject] = []

        for _score, track_id, detection_index in candidates:
            if track_id in matched_track_ids or detection_index in matched_detection_indices:
                continue
            track = self._tracks[track_id]
            box = boxes[detection_index]
            elapsed = min(max(now - track.last_update, 0.0), 1.0)
            center_x, center_y = _center(
                (float(box.x), float(box.y), float(box.width), float(box.height))
            )
            dt_observed = max(now - track.last_observed, 1e-3)
            measured_vx = (center_x - track.observed_center_x) / dt_observed
            measured_vy = (center_y - track.observed_center_y) / dt_observed
            track.velocity_x_px_s = 0.65 * track.velocity_x_px_s + 0.35 * measured_vx
            track.velocity_y_px_s = 0.65 * track.velocity_y_px_s + 0.35 * measured_vy
            track.x = float(box.x)
            track.y = float(box.y)
            track.width = float(box.width)
            track.height = float(box.height)
            track.confidence = float(box.confidence)
            radar = radar_matches[detection_index] if detection_index < len(radar_matches) else None
            if radar is not None:
                track.radar_range_m = float(radar.range_m)
                measured_rate = float(radar.range_rate_mps)
                track.radar_range_rate_mps = (
                    measured_rate if track.radar_range_rate_mps is None
                    else 0.65 * track.radar_range_rate_mps + 0.35 * measured_rate
                )
                track.radar_last_seen = now
            elif track.radar_range_m is not None and track.radar_range_rate_mps is not None:
                track.radar_range_m = max(
                    0.0, track.radar_range_m + track.radar_range_rate_mps * elapsed
                )
            track.last_update = now
            track.last_observed = now
            track.observed_center_x = center_x
            track.observed_center_y = center_y
            track.hits += 1
            track.missed_frames = 0
            matched_track_ids.add(track_id)
            matched_detection_indices.add(detection_index)
            output.append(self._snapshot(track, detection_index, predicted=False))

        for track in tracks:
            if track.track_id in matched_track_ids:
                continue
            elapsed = min(max(now - track.last_update, 0.0), 1.0)
            track.x, track.y, track.width, track.height = predictions[track.track_id]
            if track.radar_range_m is not None and track.radar_range_rate_mps is not None:
                track.radar_range_m = max(
                    0.0, track.radar_range_m + track.radar_range_rate_mps * elapsed
                )
            track.last_update = now
            track.missed_frames += 1
            if track.missed_frames <= self.max_lost_frames:
                output.append(self._snapshot(track, None, predicted=True))
            else:
                self._tracks.pop(track.track_id, None)

        for detection_index, box in enumerate(boxes):
            if detection_index in matched_detection_indices:
                continue
            center_x, center_y = _center(
                (float(box.x), float(box.y), float(box.width), float(box.height))
            )
            track = _Track(
                track_id=self._next_id,
                x=float(box.x),
                y=float(box.y),
                width=float(box.width),
                height=float(box.height),
                class_id=int(box.class_id),
                confidence=float(box.confidence),
                last_update=now,
                last_observed=now,
                observed_center_x=center_x,
                observed_center_y=center_y,
            )
            radar = radar_matches[detection_index] if detection_index < len(radar_matches) else None
            if radar is not None:
                track.radar_range_m = float(radar.range_m)
                track.radar_range_rate_mps = float(radar.range_rate_mps)
                track.radar_last_seen = now
            self._tracks[track.track_id] = track
            output.append(self._snapshot(track, detection_index, predicted=False))
            self._next_id += 1

        return tuple(sorted(output, key=lambda item: item.track_id))

    @staticmethod
    def _snapshot(track: _Track, detection_index: int | None, predicted: bool) -> TrackedObject:
        return TrackedObject(
            track_id=track.track_id,
            x=int(round(track.x)),
            y=int(round(track.y)),
            width=max(1, int(round(track.width))),
            height=max(1, int(round(track.height))),
            class_id=track.class_id,
            confidence=track.confidence,
            detection_index=detection_index,
            predicted=predicted,
            missed_frames=track.missed_frames,
            hits=track.hits,
            velocity_x_px_s=track.velocity_x_px_s,
            velocity_y_px_s=track.velocity_y_px_s,
            radar_range_m=track.radar_range_m,
            radar_range_rate_mps=track.radar_range_rate_mps,
            radar_age_s=(max(0.0, track.last_update - track.radar_last_seen)
                         if track.radar_last_seen is not None else None),
        )
