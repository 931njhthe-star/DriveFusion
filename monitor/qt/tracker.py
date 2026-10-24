"""Lightweight class-aware IoU and center-distance multi-object tracker."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from core.yolo11_detector import Box
from monitor.qt.radar_track_correction import RadarTrackCluster, radar_track_clusters


RADAR_TRACK_MAX_AGE_S = 1.0
RADAR_TRACK_GATE_M = 1.0
RADAR_TRACK_RATE_GATE_MPS = 4.0
RADAR_TRACK_AMBIGUITY_MARGIN_M = 0.75
RADAR_PIXEL_CORRECTION_WEIGHT_X = 0.7
RADAR_PIXEL_CORRECTION_WEIGHT_Y = 0.7
RADAR_PIXEL_INNOVATION_MAX_PX = 12.0


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
    radar_corrected: bool = False
    radar_correction_points: int = 0


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
    radar_range_reliable: bool = False
    radar_last_seen: float | None = None
    radar_anchor_last_seen: float | None = None
    radar_world_xy: tuple[float, float] | None = None
    radar_observed_world_xy: tuple[float, float] | None = None
    radar_world_velocity_xy: tuple[float, float] = (0.0, 0.0)
    radar_offset_px: tuple[float, float] | None = None
    radar_offset_hits: int = 0
    radar_offset_jitter_px: float = 0.0
    radar_corrected: bool = False
    radar_correction_points: int = 0

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

    def __init__(self, max_lost_frames: int = 10,
                 radar_correction_enabled: bool = False) -> None:
        self.max_lost_frames = max_lost_frames
        self.radar_correction_enabled = radar_correction_enabled
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
        radar_points: Sequence[dict[str, float]] = (),
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
                if (radar is not None and track.radar_range_m is not None
                        and track.radar_range_reliable
                        and self._valid_radar_range(radar)):
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
            track.radar_corrected = False
            track.radar_correction_points = 0
            radar = radar_matches[detection_index] if detection_index < len(radar_matches) else None
            if radar is not None:
                self._update_radar_state(track, radar, now)
                if self._valid_radar_anchor(radar):
                    self._update_radar_offset(track, center_x, track.y + track.height,
                                              radar)
            elif track.radar_range_m is not None and track.radar_range_rate_mps is not None:
                track.radar_range_m = max(
                    0.0, track.radar_range_m + track.radar_range_rate_mps * elapsed
                )
                self._advance_radar_world(track, elapsed)
            track.last_update = now
            track.last_observed = now
            track.observed_center_x = center_x
            track.observed_center_y = center_y
            track.hits += 1
            track.missed_frames = 0
            matched_track_ids.add(track_id)
            matched_detection_indices.add(detection_index)
            output.append(self._snapshot(track, detection_index, predicted=False))

        radar_updates = (self._radar_only_updates(
            tracks, matched_track_ids, boxes, radar_points, now
        ) if self.radar_correction_enabled else {})
        for track in tracks:
            if track.track_id in matched_track_ids:
                continue
            elapsed = min(max(now - track.last_update, 0.0), 1.0)
            track.x, track.y, track.width, track.height = predictions[track.track_id]
            self._advance_radar_world(track, elapsed)
            if track.radar_range_m is not None and track.radar_range_rate_mps is not None:
                track.radar_range_m = max(
                    0.0, track.radar_range_m + track.radar_range_rate_mps * elapsed
                )
            track.radar_corrected = False
            track.radar_correction_points = 0
            cluster = radar_updates.get(track.track_id)
            if cluster is not None and track.radar_offset_px is not None:
                center_x = track.x + track.width / 2.0
                bottom_y = track.y + track.height
                corrected_x = cluster.u_px + track.radar_offset_px[0]
                corrected_bottom = cluster.v_px + track.radar_offset_px[1]
                pixel_gap = math.hypot(corrected_x - center_x,
                                       corrected_bottom - bottom_y)
                pixel_gate = min(
                    RADAR_PIXEL_INNOVATION_MAX_PX,
                    max(6.0, math.hypot(track.width, track.height) * 0.25),
                )
                if pixel_gap <= pixel_gate:
                    track.x += RADAR_PIXEL_CORRECTION_WEIGHT_X * (corrected_x - center_x)
                    track.y += RADAR_PIXEL_CORRECTION_WEIGHT_Y * (corrected_bottom - bottom_y)
                    self._update_radar_state(track, cluster, now)
                    track.radar_corrected = True
                    track.radar_correction_points = cluster.point_count
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
                self._update_radar_state(track, radar, now)
                if self._valid_radar_anchor(radar):
                    self._update_radar_offset(track, center_x, track.y + track.height,
                                              radar)
            self._tracks[track.track_id] = track
            output.append(self._snapshot(track, detection_index, predicted=False))
            self._next_id += 1

        return tuple(sorted(output, key=lambda item: item.track_id))

    @staticmethod
    def _valid_radar_range(radar: object) -> bool:
        return (int(getattr(radar, "point_count", 0)) >= 3
                and float(getattr(radar, "range_spread_m", float("inf"))) <= 0.5)

    @staticmethod
    def _valid_radar_anchor(radar: object) -> bool:
        coordinates = (
            getattr(radar, "world_x_m", None), getattr(radar, "world_y_m", None),
            getattr(radar, "u_px", None), getattr(radar, "v_px", None),
        )
        return (MultiObjectTracker._valid_radar_range(radar)
                and all(value is not None and math.isfinite(float(value))
                        for value in coordinates))

    @staticmethod
    def _advance_radar_world(track: _Track, elapsed: float) -> None:
        if track.radar_world_xy is None:
            return
        track.radar_world_xy = (
            track.radar_world_xy[0] + track.radar_world_velocity_xy[0] * elapsed,
            track.radar_world_xy[1] + track.radar_world_velocity_xy[1] * elapsed,
        )

    @staticmethod
    def _update_radar_offset(
        track: _Track, center_x: float, bottom_y: float, radar: object,
    ) -> None:
        measured = (center_x - float(radar.u_px), bottom_y - float(radar.v_px))
        previous = track.radar_offset_px
        if previous is None or math.dist(measured, previous) > 12.0:
            track.radar_offset_px = measured
            track.radar_offset_hits = 1
            track.radar_offset_jitter_px = 0.0
            return
        difference = math.dist(measured, previous)
        track.radar_offset_px = (
            0.7 * previous[0] + 0.3 * measured[0],
            0.7 * previous[1] + 0.3 * measured[1],
        )
        track.radar_offset_hits = min(track.radar_offset_hits + 1, 10)
        track.radar_offset_jitter_px = (
            0.7 * track.radar_offset_jitter_px + 0.3 * difference
        )

    def _update_radar_state(self, track: _Track, radar: object, now: float) -> None:
        track.radar_range_m = float(radar.range_m)
        track.radar_range_reliable = self._valid_radar_range(radar)
        measured_rate = float(radar.range_rate_mps)
        track.radar_range_rate_mps = (
            measured_rate if track.radar_range_rate_mps is None
            else 0.65 * track.radar_range_rate_mps + 0.35 * measured_rate
        )
        if self._valid_radar_anchor(radar):
            measured_world = (float(radar.world_x_m), float(radar.world_y_m))
            if track.radar_observed_world_xy is not None and track.radar_anchor_last_seen is not None:
                dt = now - track.radar_anchor_last_seen
                if 0.0 < dt <= RADAR_TRACK_MAX_AGE_S:
                    measured_velocity = (
                        (measured_world[0] - track.radar_observed_world_xy[0]) / dt,
                        (measured_world[1] - track.radar_observed_world_xy[1]) / dt,
                    )
                    if math.hypot(*measured_velocity) <= 30.0:
                        track.radar_world_velocity_xy = (
                            0.65 * track.radar_world_velocity_xy[0]
                            + 0.35 * measured_velocity[0],
                            0.65 * track.radar_world_velocity_xy[1]
                            + 0.35 * measured_velocity[1],
                        )
            track.radar_world_xy = measured_world
            track.radar_observed_world_xy = measured_world
            track.radar_anchor_last_seen = now
        track.radar_last_seen = now

    @staticmethod
    def _radar_only_updates(
        tracks: Sequence[_Track], matched_ids: set[int], boxes: Sequence[Box],
        radar_points: Sequence[dict[str, float]], now: float,
    ) -> dict[int, RadarTrackCluster]:
        if not radar_points:
            return {}
        clusters = radar_track_clusters(radar_points, boxes)
        candidates: list[tuple[float, int, int]] = []
        for cluster_index, cluster in enumerate(clusters):
            nearby = []
            for track in tracks:
                if (track.track_id in matched_ids or track.radar_world_xy is None
                        or track.radar_offset_px is None or track.radar_offset_hits < 2
                        or track.radar_offset_jitter_px > 6.0
                        or track.radar_anchor_last_seen is None
                        or now - track.radar_anchor_last_seen > RADAR_TRACK_MAX_AGE_S):
                    continue
                dt = min(max(now - track.last_update, 0.0), 1.0)
                predicted_x = track.radar_world_xy[0] + track.radar_world_velocity_xy[0] * dt
                predicted_y = track.radar_world_xy[1] + track.radar_world_velocity_xy[1] * dt
                distance = math.hypot(cluster.world_x_m - predicted_x,
                                      cluster.world_y_m - predicted_y)
                if distance > RADAR_TRACK_GATE_M:
                    continue
                if (track.radar_range_rate_mps is not None
                        and abs(cluster.range_rate_mps - track.radar_range_rate_mps)
                        > RADAR_TRACK_RATE_GATE_MPS):
                    continue
                nearby.append((distance, track.track_id))
            nearby.sort()
            if not nearby:
                continue
            if (len(nearby) > 1
                    and nearby[1][0] - nearby[0][0] < RADAR_TRACK_AMBIGUITY_MARGIN_M):
                continue
            candidates.append((nearby[0][0], nearby[0][1], cluster_index))

        assigned: dict[int, RadarTrackCluster] = {}
        used_clusters: set[int] = set()
        for _distance, track_id, cluster_index in sorted(candidates):
            if track_id not in assigned and cluster_index not in used_clusters:
                assigned[track_id] = clusters[cluster_index]
                used_clusters.add(cluster_index)
        return assigned

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
            radar_corrected=track.radar_corrected,
            radar_correction_points=track.radar_correction_points,
        )
