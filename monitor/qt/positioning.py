"""Camera/radar position estimates and evaluation against CARLA-only ground truth."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Sequence


@dataclass(frozen=True)
class RadarBoxMatch:
    x_ego_m: float
    y_ego_m: float
    range_m: float
    range_rate_mps: float
    point_count: int
    range_spread_m: float = 0.0
    points_ego_xy: tuple[tuple[float, float], ...] = ()


RADAR_CAMERA_GATE_M = 3.0
SINGLE_POINT_RADAR_WEIGHT = 0.25
MULTI_POINT_RADAR_WEIGHT = 0.80
MAX_TARGET_SPEED_MPS = 20.0
POSITION_JUMP_ALLOWANCE_M = 3.0
RADAR_DISTANCE_AGREEMENT_M = 1.0
MIN_DISTANCE_RADAR_POINTS = 3
MAX_DISTANCE_CLUSTER_SPREAD_M = 0.5


def _vec(value: dict[str, float]) -> tuple[float, float, float]:
    return float(value["x"]), float(value["y"]), float(value["z"])


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _rotate_local_to_world(
    point: tuple[float, float, float], rotation: dict[str, float]
) -> tuple[float, float, float]:
    pitch = math.radians(float(rotation["pitch"]))
    yaw = math.radians(float(rotation["yaw"]))
    roll = math.radians(float(rotation["roll"]))
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)
    matrix = (
        (cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr),
        (cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr),
        (sp, -cp * sr, cp * cr),
    )
    return tuple(sum(matrix[row][col] * point[col] for col in range(3)) for row in range(3))


def _rotate_world_to_local(
    point: tuple[float, float, float], rotation: dict[str, float]
) -> tuple[float, float, float]:
    pitch = math.radians(float(rotation["pitch"]))
    yaw = math.radians(float(rotation["yaw"]))
    roll = math.radians(float(rotation["roll"]))
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)
    matrix = (
        (cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr),
        (cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr),
        (sp, -cp * sr, cp * cr),
    )
    return tuple(sum(matrix[row][col] * point[row] for row in range(3)) for col in range(3))


def _to_world(
    local: tuple[float, float, float], transform: dict[str, Any]
) -> tuple[float, float, float]:
    return _add(_vec(transform["location"]), _rotate_local_to_world(local, transform["rotation"]))


def _to_local(
    world: tuple[float, float, float], transform: dict[str, Any]
) -> tuple[float, float, float]:
    return _rotate_world_to_local(_sub(world, _vec(transform["location"])), transform["rotation"])


def _ego_footprint(truth: dict[str, Any] | None) -> tuple[tuple[float, float], ...] | None:
    if not truth or not truth.get("ego_transform"):
        return None
    actor = next((item for item in truth.get("vehicles", [])
                  if int(item["actor_id"]) == int(truth["ego_actor_id"])), None)
    if actor is None:
        return None
    bounds = actor["bounding_box"]
    center = _vec(bounds["location"])
    length, width = float(bounds["extent"]["x"]), float(bounds["extent"]["y"])
    points = []
    for x, y in ((-length, -width), (length, -width),
                 (length, width), (-length, width)):
        world = _to_world((center[0] + x, center[1] + y, center[2]), actor["transform"])
        local = _to_local(world, truth["ego_transform"])
        points.append((local[0], local[1]))
    return tuple(points)


def _point_to_ego_distance(point: tuple[float, float],
                           polygon: tuple[tuple[float, float], ...]) -> float:
    def cross(a: tuple[float, float], b: tuple[float, float]) -> float:
        return a[0] * b[1] - a[1] * b[0]

    turns = [cross((polygon[(i + 1) % 4][0] - polygon[i][0],
                    polygon[(i + 1) % 4][1] - polygon[i][1]),
                   (point[0] - polygon[i][0], point[1] - polygon[i][1]))
             for i in range(4)]
    if min(turns) >= -1e-9 or max(turns) <= 1e-9:
        return 0.0
    distances = []
    for i, start in enumerate(polygon):
        end = polygon[(i + 1) % 4]
        dx, dy = end[0] - start[0], end[1] - start[1]
        fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx
                                      + (point[1] - start[1]) * dy) / (dx * dx + dy * dy)))
        distances.append(math.dist(point, (start[0] + fraction * dx,
                                           start[1] + fraction * dy)))
    return min(distances)


def _intrinsics(camera: Any) -> tuple[float, float, float, float]:
    width, height = float(camera.width), float(camera.height)
    focal = width / (2.0 * math.tan(math.radians(float(camera.fov)) / 2.0))
    return focal, focal, width / 2.0, height / 2.0


def _world_to_pixel(
    world_point: tuple[float, float, float], camera: Any
) -> tuple[float, float, float] | None:
    sensor_transform = camera.sensor_transform
    if not sensor_transform:
        return None
    local = _to_local(world_point, sensor_transform)
    # CARLA camera-local axes are forward/right/up; pinhole axes are right/down/forward.
    depth, right, up = local
    if depth <= 0.05:
        return None
    fx, fy, cx, cy = _intrinsics(camera)
    return cx + fx * right / depth, cy - fy * up / depth, depth


def _box_iou(left: dict[str, float], right: dict[str, float]) -> float:
    lx1, ly1 = left["x"], left["y"]
    lx2, ly2 = lx1 + left["width"], ly1 + left["height"]
    rx1, ry1 = right["x"], right["y"]
    rx2, ry2 = rx1 + right["width"], ry1 + right["height"]
    iw = max(0.0, min(lx2, rx2) - max(lx1, rx1))
    ih = max(0.0, min(ly2, ry2) - max(ly1, ry1))
    intersection = iw * ih
    union = left["width"] * left["height"] + right["width"] * right["height"] - intersection
    return intersection / union if union > 0.0 else 0.0


def _project_actor_box(actor: dict[str, Any], camera: Any) -> dict[str, float] | None:
    bounds = actor.get("bounding_box")
    actor_transform = actor.get("transform")
    if not bounds or not actor_transform or not camera.sensor_transform:
        return None
    center = _vec(bounds["location"])
    extent = _vec(bounds["extent"])
    pixels = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                local = (
                    center[0] + sx * extent[0],
                    center[1] + sy * extent[1],
                    center[2] + sz * extent[2],
                )
                pixels.append(_world_to_pixel(_to_world(local, actor_transform), camera))
    visible = [point for point in pixels if point is not None]
    if len(visible) < 2:
        return None
    left = max(0.0, min(point[0] for point in visible))
    top = max(0.0, min(point[1] for point in visible))
    right = min(float(camera.width), max(point[0] for point in visible))
    bottom = min(float(camera.height), max(point[1] for point in visible))
    if right <= left or bottom <= top:
        return None
    return {"x": left, "y": top, "width": right - left, "height": bottom - top}


def _ego_ground_z(ground_truth: dict[str, Any] | None) -> float | None:
    if not ground_truth:
        return None
    value = ground_truth.get("ego_ground_z")
    return float(value) if value is not None else None


def _camera_ground_position(
    box: Any, camera: Any, ground_truth: dict[str, Any] | None
) -> tuple[float, float] | None:
    if not camera.sensor_transform or not ground_truth or not ground_truth.get("ego_transform"):
        return None
    ground_z = _ego_ground_z(ground_truth)
    if ground_z is None:
        return None
    fx, fy, cx, cy = _intrinsics(camera)
    u = float(box.x) + float(box.width) / 2.0
    v = float(box.y) + float(box.height)
    local_ray = (1.0, (u - cx) / fx, -(v - cy) / fy)
    direction = _rotate_local_to_world(local_ray, camera.sensor_transform["rotation"])
    if abs(direction[2]) < 1e-5:
        return None
    origin = _vec(camera.sensor_transform["location"])
    distance = (ground_z - origin[2]) / direction[2]
    if distance <= 0.0 or distance > 250.0:
        return None
    point_world = _add(origin, tuple(component * distance for component in direction))
    point_ego = _to_local(point_world, ground_truth["ego_transform"])
    return point_ego[0], point_ego[1]


def _radar_points_in_camera(
    camera: Any, radar: dict[str, Any] | None, ground_truth: dict[str, Any] | None
) -> list[dict[str, float]]:
    if not radar or not camera.sensor_transform or not radar.get("sensor_transform"):
        return []
    ego_transform = (ground_truth or {}).get("ego_transform")
    if not ego_transform:
        return []
    output = []
    radar_transform = radar["sensor_transform"]
    for point in radar.get("points", []):
        depth = float(point["distance_m"])
        azimuth = float(point["azimuth_rad"])
        altitude = float(point["altitude_rad"])
        local_radar = (
            depth * math.cos(altitude) * math.cos(azimuth),
            depth * math.cos(altitude) * math.sin(azimuth),
            depth * math.sin(altitude),
        )
        world_point = _to_world(local_radar, radar_transform)
        pixel = _world_to_pixel(world_point, camera)
        if pixel is None:
            continue
        ego_point = _to_local(world_point, ego_transform)
        output.append({
            "u": pixel[0], "v": pixel[1], "camera_depth_m": pixel[2],
            "x_ego_m": ego_point[0], "y_ego_m": ego_point[1],
            "range_m": math.hypot(ego_point[0], ego_point[1]),
            "range_rate_mps": -float(point["relative_velocity_mps"]),
        })
    return output


def match_radar_to_boxes(
    camera: Any, boxes: Sequence[Any], radar: dict[str, Any] | None,
    ground_truth: dict[str, Any] | None,
) -> tuple[RadarBoxMatch | None, ...]:
    """Associate radar returns to 2D detections by projected pixel containment."""
    camera_frame = getattr(camera, "frame", None)
    if radar is None or (camera_frame is not None
                         and int(radar["frame"]) != int(camera_frame)):
        return tuple(None for _ in boxes)
    radar_pixels = _radar_points_in_camera(camera, radar, ground_truth)
    assignments: list[list[dict[str, float]]] = [[] for _ in boxes]
    camera_positions = [_camera_ground_position(box, camera, ground_truth) for box in boxes]
    for point in radar_pixels:
        candidates = []
        for index, box in enumerate(boxes):
            camera_xy = camera_positions[index]
            if (camera_xy is not None
                    and abs(point["x_ego_m"] - camera_xy[0])
                    > max(RADAR_CAMERA_GATE_M, 0.15 * abs(camera_xy[0]))):
                continue
            margin_x = max(4.0, float(box.width) * 0.08)
            margin_y = max(4.0, float(box.height) * 0.08)
            if (box.x - margin_x <= point["u"] <= box.x + box.width + margin_x
                    and box.y - margin_y <= point["v"] <= box.y + box.height + margin_y):
                center_x = box.x + box.width / 2.0
                center_y = box.y + box.height / 2.0
                norm = math.hypot(
                    (point["u"] - center_x) / max(float(box.width), 1.0),
                    (point["v"] - center_y) / max(float(box.height), 1.0),
                )
                candidates.append((norm, index))
        if candidates:
            assignments[min(candidates)[1]].append(point)
    matches: list[RadarBoxMatch | None] = []
    for candidates in assignments:
        if not candidates:
            matches.append(None)
            continue
        candidates.sort(key=lambda point: point["range_m"])
        clusters: list[list[dict[str, float]]] = []
        for point in candidates:
            if (not clusters or point["range_m"] - clusters[-1][-1]["range_m"] > 0.5
                    or point["range_m"] - clusters[-1][0]["range_m"] > 1.0):
                clusters.append([point])
            else:
                clusters[-1].append(point)
        cluster = clusters[0]
        for group in clusters:
            if len(group) < MIN_DISTANCE_RADAR_POINTS:
                continue
            group_range = median(point["range_m"] for point in group)
            if max(abs(point["range_m"] - group_range) for point in group) <= MAX_DISTANCE_CLUSTER_SPREAD_M:
                cluster = group
                break
        center_range = median(point["range_m"] for point in cluster)
        matches.append(RadarBoxMatch(
            x_ego_m=median(point["x_ego_m"] for point in cluster),
            y_ego_m=median(point["y_ego_m"] for point in cluster),
            range_m=median(point["range_m"] for point in cluster),
            range_rate_mps=median(point["range_rate_mps"] for point in cluster),
            point_count=len(cluster),
            range_spread_m=max(abs(point["range_m"] - center_range) for point in cluster),
            points_ego_xy=tuple((point["x_ego_m"], point["y_ego_m"])
                                for point in cluster),
        ))
    return tuple(matches)


def _ground_truth_matches(
    camera: Any, boxes: Sequence[Any], truth: dict[str, Any] | None
) -> tuple[dict[int, dict[str, Any]], int]:
    if not truth:
        return {}, 0
    projected = []
    for actor in truth.get("vehicles", []):
        if int(actor.get("actor_id", -1)) == int(truth.get("ego_actor_id", -2)):
            continue
        projected_box = _project_actor_box(actor, camera)
        if projected_box:
            projected.append((actor, projected_box))
    candidates = []
    for box_index, box in enumerate(boxes):
        if int(getattr(box, "class_id", -1)) not in (2, 3, 5, 7):
            continue
        detection_box = {"x": float(box.x), "y": float(box.y),
                         "width": float(box.width), "height": float(box.height)}
        for actor, gt_box in projected:
            overlap = _box_iou(detection_box, gt_box)
            if overlap >= 0.05:
                candidates.append((overlap, box_index, actor))
    candidates.sort(key=lambda item: (-item[0], item[1], int(item[2]["actor_id"])))
    result: dict[int, dict[str, Any]] = {}
    used_actors: set[int] = set()
    for overlap, box_index, actor in candidates:
        actor_id = int(actor["actor_id"])
        if box_index in result or actor_id in used_actors:
            continue
        result[box_index] = {"actor": actor, "iou": overlap}
        used_actors.add(actor_id)
    return result, len(projected)


class PositionAccuracyAccumulator:
    def __init__(self) -> None:
        self.camera_error_sum = 0.0
        self.camera_error_square_sum = 0.0
        self.fusion_error_sum = 0.0
        self.fusion_error_square_sum = 0.0
        self.visible_truth = 0
        self.camera_matched = 0
        self.fusion_matched = 0
        self.fusion_source_counts = {"camera+radar": 0, "radar": 0, "camera_only": 0}
        self.aligned_radar_frames = 0
        self.misaligned_radar_frames = 0

    @staticmethod
    def _stats(errors: Iterable[float]) -> dict[str, float | int | None]:
        values = list(errors)
        if not values:
            return {"samples": 0, "mae_m": None, "rmse_m": None}
        return {
            "samples": len(values),
            "mae_m": sum(values) / len(values),
            "rmse_m": math.sqrt(sum(value * value for value in values) / len(values)),
        }

    def update(self, estimates: list[dict[str, Any]], visible_truth: int,
               radar_delta: int | None) -> dict[str, Any]:
        self.visible_truth += visible_truth
        if radar_delta is not None and radar_delta == 0:
            self.aligned_radar_frames += 1
        elif radar_delta is not None:
            self.misaligned_radar_frames += 1
        for estimate in estimates:
            if estimate.get("ground_truth_actor_id") is None:
                continue
            if estimate.get("camera_xy_ego_m") is not None or estimate.get("radar_used"):
                source = str(estimate.get("fusion_source", "camera_only"))
                if source in self.fusion_source_counts:
                    self.fusion_source_counts[source] += 1
            camera_error = estimate.get("camera_error_m")
            if camera_error is not None:
                self.camera_matched += 1
                value = float(camera_error)
                self.camera_error_sum += value
                self.camera_error_square_sum += value * value
            if estimate.get("radar_used"):
                fusion_error = estimate.get("fusion_error_m")
                if fusion_error is not None:
                    self.fusion_matched += 1
                    value = float(fusion_error)
                    self.fusion_error_sum += value
                    self.fusion_error_square_sum += value * value
        return self.snapshot()

    @staticmethod
    def _running_stats(count: int, total: float, square_total: float) -> dict[str, float | int | None]:
        if count == 0:
            return {"samples": 0, "mae_m": None, "rmse_m": None}
        return {
            "samples": count,
            "mae_m": total / count,
            "rmse_m": math.sqrt(square_total / count),
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "camera": self._running_stats(
                self.camera_matched, self.camera_error_sum, self.camera_error_square_sum
            ),
            "radar_fusion": self._running_stats(
                self.fusion_matched, self.fusion_error_sum, self.fusion_error_square_sum
            ),
            "visible_ground_truth": self.visible_truth,
            "camera_matches": self.camera_matched,
            "radar_fusion_matches": self.fusion_matched,
            "fusion_source_counts": dict(self.fusion_source_counts),
            "radar_exact_frame_pairs": self.aligned_radar_frames,
            "radar_offset_frame_pairs": self.misaligned_radar_frames,
        }


class PositioningEngine:
    """Estimate target ground positions; use CARLA truth only for evaluation."""

    def __init__(self) -> None:
        self._track_actor_ids: dict[int, int] = {}
        self._last_positions: dict[int, tuple[float, tuple[float, float]]] = {}
        self.accuracy = PositionAccuracyAccumulator()

    def reset(self) -> None:
        self._track_actor_ids.clear()
        self._last_positions.clear()
        self.accuracy = PositionAccuracyAccumulator()

    def estimate(
        self, camera: Any, boxes: Sequence[Any], tracks: Sequence[Any],
        radar: dict[str, Any] | None, truth: dict[str, Any] | None,
        radar_delta_frames: int | None, truth_delta_frames: int | None,
    ) -> dict[str, Any]:
        if radar is not None and int(radar["frame"]) != int(camera.frame):
            radar = None
            radar_delta_frames = None
        radar_by_box = match_radar_to_boxes(camera, boxes, radar, truth)
        truth_by_box, visible_truth = _ground_truth_matches(camera, boxes, truth)
        ego_transform = (truth or {}).get("ego_transform")
        ego_footprint = _ego_footprint(truth)
        timestamp = float(camera.timestamp)
        estimate_rows = []
        active_track_ids = set()
        for track in tracks:
            active_track_ids.add(track.track_id)
            box = (boxes[track.detection_index]
                   if track.detection_index is not None and track.detection_index < len(boxes)
                   else track)
            camera_xy = _camera_ground_position(box, camera, truth)
            radar_match = (
                radar_by_box[track.detection_index]
                if track.detection_index is not None and track.detection_index < len(radar_by_box)
                else None
            )
            if radar_match is None and track.detection_index is None:
                radar_match = match_radar_to_boxes(camera, (track,), radar, truth)[0]
            radar_xy = ((radar_match.x_ego_m, radar_match.y_ego_m)
                        if radar_match else None)
            radar_range = float(radar_match.range_m) if radar_match else None
            radar_rate = float(radar_match.range_rate_mps) if radar_match else None
            radar_point_count = int(radar_match.point_count) if radar_match else 0
            radar_age = 0.0 if radar_match else None
            radar_rejection_reason = None
            previous = self._last_positions.get(track.track_id)
            if radar_xy is not None and previous is not None and ego_transform is not None:
                elapsed = timestamp - previous[0]
                if 0.0 < elapsed <= 1.0:
                    radar_world = _to_world((radar_xy[0], radar_xy[1], 0.0), ego_transform)
                    jump = math.dist(radar_world[:2], previous[1])
                    if jump > POSITION_JUMP_ALLOWANCE_M + MAX_TARGET_SPEED_MPS * elapsed:
                        radar_xy = None
                        radar_rejection_reason = "implausible_position_jump"
            fused_xy = None
            if camera_xy is not None and radar_xy is not None:
                radar_weight = (SINGLE_POINT_RADAR_WEIGHT if radar_point_count == 1
                                else MULTI_POINT_RADAR_WEIGHT)
                fused_xy = ((1.0 - radar_weight) * camera_xy[0] + radar_weight * radar_xy[0],
                            0.65 * camera_xy[1] + 0.35 * radar_xy[1])
            elif radar_xy is not None:
                fused_xy = radar_xy
            elif camera_xy is not None:
                fused_xy = camera_xy
            camera_distance = (_point_to_ego_distance(camera_xy, ego_footprint)
                               if camera_xy is not None and ego_footprint is not None else None)
            radar_distance = (median(_point_to_ego_distance(point, ego_footprint)
                                     for point in radar_match.points_ego_xy)
                              if radar_xy is not None and ego_footprint is not None
                              and radar_match.points_ego_xy else None)
            distance_source = "camera" if camera_distance is not None else "unavailable"
            distance_rejection_reason = radar_rejection_reason
            estimated_distance = camera_distance
            if radar_distance is not None and camera_distance is not None:
                if radar_point_count < MIN_DISTANCE_RADAR_POINTS:
                    distance_rejection_reason = "insufficient_radar_points"
                elif radar_match.range_spread_m > MAX_DISTANCE_CLUSTER_SPREAD_M:
                    distance_rejection_reason = "radar_cluster_spread"
                elif abs(radar_distance - camera_distance) > RADAR_DISTANCE_AGREEMENT_M:
                    distance_rejection_reason = "camera_radar_distance_disagreement"
                else:
                    estimated_distance = radar_distance
                    distance_source = "radar_near_cluster"
                    distance_rejection_reason = None
            if fused_xy is not None and ego_transform is not None and not track.predicted:
                fused_world = _to_world((fused_xy[0], fused_xy[1], 0.0), ego_transform)
                self._last_positions[track.track_id] = (timestamp, fused_world[:2])

            matched_truth = None
            if track.detection_index is not None and track.detection_index in truth_by_box:
                matched_truth = truth_by_box[track.detection_index]
                actor_id = int(matched_truth["actor"]["actor_id"])
                for other_track_id, other_actor_id in list(self._track_actor_ids.items()):
                    if other_track_id != track.track_id and other_actor_id == actor_id:
                        self._track_actor_ids.pop(other_track_id, None)
                self._track_actor_ids[track.track_id] = actor_id
            elif (track.predicted and track.track_id in self._track_actor_ids
                  and truth is not None):
                actor_id = self._track_actor_ids[track.track_id]
                actor = next((item for item in (truth or {}).get("vehicles", [])
                              if int(item.get("actor_id", -1)) == actor_id), None)
                if actor is not None:
                    matched_truth = {"actor": actor, "iou": None}
            gt_xy = None
            gt_actor_id = None
            match_iou = None
            if matched_truth:
                actor = matched_truth["actor"]
                actor_transform = actor["transform"]
                bounds = actor["bounding_box"]
                center = _vec(bounds["location"])
                bottom_local = (center[0], center[1], center[2] - float(bounds["extent"]["z"]))
                ground_world = _to_world(bottom_local, actor_transform)
                gt_local = _to_local(ground_world, truth["ego_transform"])
                gt_xy = (gt_local[0], gt_local[1])
                gt_actor_id = int(actor["actor_id"])
                match_iou = matched_truth["iou"]
            camera_error = math.dist(camera_xy, gt_xy) if camera_xy is not None and gt_xy else None
            fusion_error = math.dist(fused_xy, gt_xy) if fused_xy is not None and gt_xy else None
            estimate_rows.append({
                "track_id": int(track.track_id),
                "class_id": int(track.class_id),
                "bbox": {"x": int(track.x), "y": int(track.y),
                         "width": int(track.width), "height": int(track.height)},
                "observed": not bool(track.predicted),
                "missed_frames": int(track.missed_frames),
                "camera_xy_ego_m": _xy(camera_xy),
                "radar_xy_ego_m": _xy(radar_xy),
                "radar_range_m": radar_range,
                "radar_range_rate_mps": radar_rate,
                "radar_points": radar_point_count,
                "radar_cluster_spread_m": (radar_match.range_spread_m if radar_match else None),
                "radar_age_s": radar_age,
                "radar_rejection_reason": radar_rejection_reason,
                "radar_used": radar_xy is not None,
                "camera_distance_m": camera_distance,
                "radar_distance_m": radar_distance,
                "estimated_distance_m": estimated_distance,
                "distance_source": distance_source,
                "distance_rejection_reason": distance_rejection_reason,
                "fused_xy_ego_m": _xy(fused_xy),
                "fusion_source": ("camera+radar" if camera_xy is not None and radar_xy is not None
                                  else "radar" if radar_xy is not None else "camera_only"),
                "ground_truth_actor_id": gt_actor_id,
                "ground_truth_xy_ego_m": _xy(gt_xy),
                "ground_truth_iou": match_iou,
                "camera_error_m": camera_error,
                "fusion_error_m": fusion_error,
            })
        for track_id in list(self._track_actor_ids):
            if track_id not in active_track_ids:
                self._track_actor_ids.pop(track_id, None)
        for track_id in list(self._last_positions):
            if track_id not in active_track_ids:
                self._last_positions.pop(track_id, None)
        summary = self.accuracy.update(estimate_rows, visible_truth, radar_delta_frames)
        return {
            "schema_version": 1,
            "kind": "positioning",
            "frame": int(camera.frame),
            "simulation_timestamp": float(camera.timestamp),
            "radar_frame": int(radar["frame"]) if radar else None,
            "radar_delta_frames": radar_delta_frames,
            "ground_truth_frame": int(truth["frame"]) if truth else None,
            "ground_truth_delta_frames": truth_delta_frames,
            "visible_ground_truth": visible_truth,
            "fusion_weights": {"longitudinal_radar_single_point": SINGLE_POINT_RADAR_WEIGHT,
                               "longitudinal_radar_multi_point": MULTI_POINT_RADAR_WEIGHT,
                               "lateral_camera": 0.65, "lateral_radar": 0.35},
            "objects": estimate_rows,
            "live_summary": summary,
        }


def _xy(value: tuple[float, float] | None) -> dict[str, float] | None:
    if value is None:
        return None
    return {"forward_m": float(value[0]), "right_m": float(value[1])}


def write_accuracy_summary(session_dir: Path) -> dict[str, Any]:
    """Summarize matched per-object errors in a tracking session."""
    session_dir = session_dir.resolve()
    input_path = session_dir / "positions.jsonl"
    if not input_path.is_file():
        raise FileNotFoundError(f"positions.jsonl not found in {session_dir}")
    camera_errors: list[float] = []
    fusion_errors: list[float] = []
    camera_radar_errors: list[float] = []
    radar_only_errors: list[float] = []
    fusion_source_counts = {"camera+radar": 0, "radar": 0, "camera_only": 0}
    total_visible_truth = camera_matches = fusion_matches = 0
    total_frames = invalid_lines = radar_exact = radar_offset = radar_missing = 0
    with input_path.open("r", encoding="utf-8") as source:
        for line in source:
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                total_frames += 1
                total_visible_truth += int(record.get("visible_ground_truth", 0))
                delta = record.get("radar_delta_frames")
                if delta is None:
                    radar_missing += 1
                elif int(delta) == 0:
                    radar_exact += 1
                else:
                    radar_offset += 1
                for target in record.get("objects", []):
                    camera_error = target.get("camera_error_m")
                    if camera_error is not None:
                        camera_errors.append(float(camera_error))
                        camera_matches += 1
                    fusion_error = target.get("fusion_error_m")
                    if target.get("radar_used") and fusion_error is not None:
                        fusion_errors.append(float(fusion_error))
                        fusion_matches += 1
                        source_kind = target.get("fusion_source")
                        if source_kind == "camera+radar":
                            camera_radar_errors.append(float(fusion_error))
                        elif source_kind == "radar":
                            radar_only_errors.append(float(fusion_error))
                    if (target.get("ground_truth_actor_id") is not None
                            and (target.get("camera_xy_ego_m") is not None or target.get("radar_used"))):
                        source_kind = str(target.get("fusion_source", "camera_only"))
                        if source_kind in fusion_source_counts:
                            fusion_source_counts[source_kind] += 1
            except (json.JSONDecodeError, TypeError, ValueError):
                invalid_lines += 1
    camera_stats = PositionAccuracyAccumulator._stats(camera_errors)
    fusion_stats = PositionAccuracyAccumulator._stats(fusion_errors)
    summary = {
        "schema_version": 1,
        "session": str(session_dir),
        "frames": total_frames,
        "invalid_position_records": invalid_lines,
        "visible_ground_truth_objects": total_visible_truth,
        "camera_estimates_matched": camera_matches,
        "radar_fusion_estimates_matched": fusion_matches,
        "camera_coverage": camera_matches / total_visible_truth if total_visible_truth else None,
        "radar_fusion_coverage": fusion_matches / total_visible_truth if total_visible_truth else None,
        "camera_only_position_error": camera_stats,
        "radar_fusion_position_error": fusion_stats,
        "camera_plus_radar_position_error": PositionAccuracyAccumulator._stats(camera_radar_errors),
        "radar_only_fallback_position_error": PositionAccuracyAccumulator._stats(radar_only_errors),
        "position_source_counts": fusion_source_counts,
        "radar_alignment": {"exact_frames": radar_exact, "offset_frames": radar_offset,
                            "missing_frames": radar_missing},
        "notes": [
            "Position errors compare matched YOLO tracks with projected CARLA vehicle bounding boxes.",
            "Radar-assisted errors include tracks with a current or recent associated radar range.",
            "Position estimates use a flat local road plane and fixed baseline fusion weights.",
            "Radar returns may represent a vehicle surface point while the CARLA reference is the bounding-box ground center.",
        ],
    }
    (session_dir / "accuracy_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary
