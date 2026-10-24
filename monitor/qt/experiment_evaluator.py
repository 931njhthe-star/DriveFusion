"""Replay a saved session and compare three target-position pipelines with CARLA truth."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator

from PySide6.QtCore import QThread, Signal

from core.yolo11_detector import Box
from monitor.qt.positioning import (
    PositioningEngine,
    _project_actor_box,
    _to_local,
    _to_world,
    _vec,
    match_radar_to_boxes,
    radar_tracking_points,
)
from monitor.qt.tracker import MultiObjectTracker
from monitor.qt.visibility import (
    DETECTION_IOU, MIN_VISIBLE_BOX_SIDE_PX, MIN_VISIBLE_PIXELS,
    match_visible_detections, visible_vehicle_boxes,
)


METHODS = {
    "camera_tracking": "트래킹 + 카메라",
    "vision_radar_fusion": "트래킹 + 카메라·레이더 융합",
    "radar_tracking_fusion": "레이더 융합 트래킹 + 카메라·레이더 융합",
}
LEGACY_MIN_DISTANCE_M = 3.0
LEGACY_MAX_DISTANCE_M = 15.0
DISTANCE_BANDS = (
    ("0~3m", 0.0, 3.0),
    ("3~5m", 3.0, 5.0),
    ("5~10m", 5.0, 10.0),
    ("10~15m", 10.0, 15.0),
    ("15~20m", 15.0, 20.0),
    ("20~30m", 20.0, 30.0),
    ("30m 이상", 30.0, None),
)


def _records(path: Path) -> Iterator[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path.name}:{line_number}: 잘못된 JSON 기록") from exc


def _key(record: dict[str, Any]) -> tuple[int, int]:
    return int(record.get("sensor_id", -1)), int(record["frame"])


def _truth_xy(actor: dict[str, Any], truth: dict[str, Any]) -> tuple[float, float]:
    bounds = actor["bounding_box"]
    center = _vec(bounds["location"])
    ground_local = (center[0], center[1], center[2] - float(bounds["extent"]["z"]))
    ground_world = _to_world(ground_local, actor["transform"])
    relative = _to_local(ground_world, truth["ego_transform"])
    return relative[0], relative[1]


def _footprint(actor: dict[str, Any], ego_transform: dict[str, Any]) -> tuple[tuple[float, float], ...]:
    """CARLA bounding-box ground footprint in ego coordinates, in edge order."""
    bounds = actor["bounding_box"]
    center = _vec(bounds["location"])
    half_length = float(bounds["extent"]["x"])
    half_width = float(bounds["extent"]["y"])
    corners = ((-half_length, -half_width), (half_length, -half_width),
               (half_length, half_width), (-half_length, half_width))
    result = []
    for x, y in corners:
        world = _to_world((center[0] + x, center[1] + y, center[2]), actor["transform"])
        local = _to_local(world, ego_transform)
        result.append((local[0], local[1]))
    return tuple(result)


def _cross(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _inside(point: tuple[float, float], polygon: tuple[tuple[float, float], ...]) -> bool:
    turns = [_cross(polygon[i], polygon[(i + 1) % len(polygon)], point)
             for i in range(len(polygon))]
    return min(turns) >= -1e-9 or max(turns) <= 1e-9


def _point_segment_distance(point: tuple[float, float], a: tuple[float, float],
                            b: tuple[float, float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.dist(point, a)
    fraction = max(0.0, min(1.0, ((point[0] - a[0]) * dx
                                  + (point[1] - a[1]) * dy) / length_squared))
    return math.dist(point, (a[0] + fraction * dx, a[1] + fraction * dy))


def _point_footprint_distance(point: tuple[float, float],
                              polygon: tuple[tuple[float, float], ...]) -> float:
    if _inside(point, polygon):
        return 0.0
    return min(_point_segment_distance(point, polygon[i], polygon[(i + 1) % len(polygon)])
               for i in range(len(polygon)))


def _footprint_distance(left: tuple[tuple[float, float], ...],
                        right: tuple[tuple[float, float], ...]) -> float:
    """Shortest planar clearance between two convex vehicle footprints."""
    separated = False
    for polygon in (left, right):
        for i, a in enumerate(polygon):
            b = polygon[(i + 1) % len(polygon)]
            axis = (a[1] - b[1], b[0] - a[0])
            left_projection = [point[0] * axis[0] + point[1] * axis[1] for point in left]
            right_projection = [point[0] * axis[0] + point[1] * axis[1] for point in right]
            if (max(left_projection) < min(right_projection) - 1e-9
                    or max(right_projection) < min(left_projection) - 1e-9):
                separated = True
                break
        if separated:
            break
    if not separated:
        return 0.0
    return min(
        min(_point_footprint_distance(point, other) for point in polygon)
        for polygon, other in ((left, right), (right, left))
    )


def _eligible_truth(
    camera: Any, truth: dict[str, Any]
) -> tuple[dict[int, tuple[float, float, float]], set[str], set[str]]:
    eligible: dict[int, tuple[float, float, float]] = {}
    ego_models: set[str] = set()
    target_models: set[str] = set()
    ego_actor = next((actor for actor in truth.get("vehicles", [])
                      if int(actor["actor_id"]) == int(truth["ego_actor_id"])), None)
    if ego_actor is None:
        return eligible, ego_models, target_models
    ego_footprint = _footprint(ego_actor, truth["ego_transform"])
    for actor in truth.get("vehicles", []):
        actor_id = int(actor["actor_id"])
        if actor_id == int(truth["ego_actor_id"]):
            ego_models.add(str(actor.get("type_id", "unknown")))
            continue
        gt_x, gt_y = _truth_xy(actor, truth)
        distance = _footprint_distance(ego_footprint, _footprint(actor, truth["ego_transform"]))
        if _project_actor_box(actor, camera):
            eligible[actor_id] = (gt_x, gt_y, distance)
            target_models.add(str(actor.get("type_id", "unknown")))
    return eligible, ego_models, target_models


def _stats(rows: list[dict[str, Any]], method: str) -> dict[str, Any]:
    selected = [row for row in rows if row["method"] == method]
    observed_matches = sum(bool(row.get("detection_matched")) for row in selected)
    observed_errors = [row["error_m"] for row in selected if row["error_m"] is not None]
    predicted_errors = [row["predicted_error_m"] for row in selected
                        if row.get("error_m") is None
                        and row.get("predicted_error_m") is not None]
    errors = [error for row in selected if (error := _tracking_error(row)) is not None]
    corrected_errors = [row["predicted_error_m"] for row in selected
                        if row.get("error_m") is None
                        and row.get("predicted_error_m") is not None
                        and row.get("radar_corrected")]
    uncorrected_errors = [row["predicted_error_m"] for row in selected
                          if row.get("error_m") is None
                          and row.get("predicted_error_m") is not None
                          and not row.get("radar_corrected")]
    observed_signed_errors = [row["distance_error_signed_m"] for row in selected
                              if row.get("distance_error_signed_m") is not None]
    signed_errors = [row.get("tracking_distance_error_signed_m",
                             row.get("distance_error_signed_m")) for row in selected]
    signed_errors = [error for error in signed_errors if error is not None]
    position_errors = [row["position_error_m"] for row in selected
                       if row.get("position_error_m") is not None]
    corrected_position_errors = [row["predicted_position_error_m"] for row in selected
                                 if row.get("predicted_position_error_m") is not None
                                 and row.get("radar_corrected")]
    corrected_camera_position_errors = [
        row["predicted_camera_position_error_m"] for row in selected
        if row.get("predicted_camera_position_error_m") is not None
        and row.get("radar_corrected")
    ]
    corrected_bbox_errors = [row["predicted_bbox_center_error_px"] for row in selected
                             if row.get("predicted_bbox_center_error_px") is not None
                             and row.get("radar_corrected")]
    radar_errors = [_tracking_error(row) for row in selected if row["radar_used"]]
    radar_errors = [error for error in radar_errors if error is not None]
    fallback_errors = [_tracking_error(row) for row in selected if not row["radar_used"]]
    fallback_errors = [error for error in fallback_errors if error is not None]
    prediction_rejections: dict[str, int] = {}
    predicted_track_matches = 0
    for row in selected:
        if row.get("track_id") is None or row.get("observed"):
            continue
        predicted_track_matches += 1
        reason = row.get("distance_rejection_reason")
        if isinstance(reason, str) and reason.startswith("prediction_"):
            prediction_rejections[reason] = prediction_rejections.get(reason, 0) + 1
    switches = 0
    previous: dict[tuple[str, int], int] = {}
    for row in selected:
        track_id = row["track_id"]
        actor_id = row["gt_actor_id"]
        if track_id is None or not row["observed"]:
            continue
        actor_key = (str(row.get("session", "")), actor_id)
        if actor_key in previous and previous[actor_key] != track_id:
            switches += 1
        previous[actor_key] = track_id
    return {
        "gt_opportunities": len(selected),
        "observed_matches": observed_matches,
        "distance_samples": len(errors),
        "observed_distance_samples": len(observed_errors),
        "predicted_distance_samples": len(predicted_errors),
        "predicted_track_matches": predicted_track_matches,
        "prediction_distance_unavailable": sum(prediction_rejections.values()),
        "prediction_distance_rejected": sum(
            count for reason, count in prediction_rejections.items()
            if reason != "prediction_projection_unavailable"
        ),
        "prediction_distance_rejection_reasons": prediction_rejections,
        "radar_corrected_predicted_frames": sum(
            bool(row.get("radar_corrected")) and not row.get("observed")
            for row in selected),
        "predicted_mae_m": (sum(predicted_errors) / len(predicted_errors)
                            if predicted_errors else None),
        "radar_corrected_predicted_samples": len(corrected_errors),
        "radar_corrected_predicted_mae_m": (
            sum(corrected_errors) / len(corrected_errors) if corrected_errors else None),
        "uncorrected_predicted_samples": len(uncorrected_errors),
        "uncorrected_predicted_mae_m": (
            sum(uncorrected_errors) / len(uncorrected_errors) if uncorrected_errors else None),
        "position_samples": len(position_errors),
        "radar_corrected_predicted_position_samples": len(corrected_position_errors),
        "radar_corrected_predicted_position_mae_m": (
            sum(corrected_position_errors) / len(corrected_position_errors)
            if corrected_position_errors else None),
        "radar_corrected_predicted_camera_position_mae_m": (
            sum(corrected_camera_position_errors) / len(corrected_camera_position_errors)
            if corrected_camera_position_errors else None),
        "radar_corrected_predicted_bbox_center_mae_px": (
            sum(corrected_bbox_errors) / len(corrected_bbox_errors)
            if corrected_bbox_errors else None),
        "coverage": len(observed_errors) / observed_matches if observed_matches else None,
        "projected_match_rate": observed_matches / len(selected) if selected else None,
        "mae_m": sum(errors) / len(errors) if errors else None,
        "rmse_m": math.sqrt(sum(error * error for error in errors) / len(errors)) if errors else None,
        "distance_bias_m": (sum(signed_errors) / len(signed_errors)
                            if signed_errors else None),
        "observed_mae_m": (sum(observed_errors) / len(observed_errors)
                           if observed_errors else None),
        "observed_rmse_m": (math.sqrt(sum(error * error for error in observed_errors)
                                      / len(observed_errors)) if observed_errors else None),
        "observed_distance_bias_m": (
            sum(observed_signed_errors) / len(observed_signed_errors)
            if observed_signed_errors else None),
        "position_mae_m": (sum(position_errors) / len(position_errors)
                           if position_errors else None),
        "position_rmse_m": (math.sqrt(sum(error * error for error in position_errors)
                                      / len(position_errors)) if position_errors else None),
        "id_switches": switches,
        "radar_used_samples": len(radar_errors),
        "camera_fallback_samples": len(fallback_errors) if method != "camera_tracking" else 0,
        "radar_used_mae_m": sum(radar_errors) / len(radar_errors) if radar_errors else None,
        "camera_fallback_mae_m": (sum(fallback_errors) / len(fallback_errors)
                                  if fallback_errors and method != "camera_tracking" else None),
    }


def _tracking_error(row: dict[str, Any]) -> float | None:
    """One distance error per GT vehicle/frame: current YOLO OR retained track."""
    observed = row.get("error_m")
    return observed if observed is not None else row.get("predicted_error_m")


def _mount(sensor: dict[str, Any] | None, truth: dict[str, Any]) -> dict[str, Any] | None:
    if not sensor or not sensor.get("sensor_transform"):
        return None
    location = _to_local(_vec(sensor["sensor_transform"]["location"]), truth["ego_transform"])
    sensor_rotation = sensor["sensor_transform"]["rotation"]
    ego_rotation = truth["ego_transform"]["rotation"]
    return {
        "sensor_id": sensor.get("sensor_id"),
        "location_ego_m": {"x": location[0], "y": location[1], "z": location[2]},
        "rotation_delta_deg_approx": {
            axis: (float(sensor_rotation[axis]) - float(ego_rotation[axis]) + 180) % 360 - 180
            for axis in ("pitch", "yaw", "roll")
        },
        "source_frame": sensor["frame"],
    }


def _summary_for_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    paired: dict[tuple[str, int, int], dict[str, float | None]] = {}
    for row in rows:
        paired.setdefault((str(row.get("session", "")), row["frame"], row["gt_actor_id"]), {})[row["method"]] = _tracking_error(row)
    common = [values for values in paired.values()
              if all(values.get(method) is not None for method in METHODS)]
    masked = [row for row in rows if row["method"] == "camera_tracking"
              and row.get("visibility_mask_available")]
    visible = [row for row in masked if row.get("visible_gt")]
    detected = sum(bool(row.get("yolo_detected_visible_gt")) for row in visible)
    tracking = sum(bool(row.get("detection_matched")) for row in visible)
    distance = sum(row.get("error_m") is not None for row in visible)
    return {
        "eligible_gt_opportunities": len(paired),
        "zero_distance_opportunities": len({(str(row.get("session", "")), row["frame"], row["gt_actor_id"])
                                            for row in rows if row["gt_distance_m"] == 0}),
        "common_samples": len(common),
        "common_sample_mae_m": {
            method: sum(values[method] for values in common) / len(common) if common else None
            for method in METHODS
        },
        "methods": {key: {"label": label, **_stats(rows, key)} for key, label in METHODS.items()},
        "visibility": {
            "status": "available" if masked else "not_recorded",
            "mask_aligned_projected_gt": len(masked),
            "visible_gt_opportunities": len(visible),
            "yolo_detected_visible_gt": detected,
            "yolo_recall": detected / len(visible) if visible else None,
            "tracking_matched_visible_gt": tracking,
            "distance_samples_visible_gt": distance,
            "min_visible_pixels": MIN_VISIBLE_PIXELS,
            "min_visible_box_side_px": MIN_VISIBLE_BOX_SIDE_PX,
            "yolo_box_iou_threshold": DETECTION_IOU,
        },
    }


def _band_for_distance(distance: float) -> str:
    return next(label for label, low, high in DISTANCE_BANDS
                if low <= distance and (high is None or distance < high))


def _selected_rows(rows: list[dict[str, Any]], low: float,
                   high: float | None, include_max: bool = True) -> list[dict[str, Any]]:
    return [row for row in rows if low <= row["gt_distance_m"]
            and (high is None or (row["gt_distance_m"] <= high if include_max
                                  else row["gt_distance_m"] < high))]


def _visibility_report_lines(summary: dict[str, Any]) -> list[str]:
    visibility = summary["visibility"]
    lines = ["", "## 가시성 마스크 기준 YOLO 검출률", ""]
    if visibility["status"] != "available":
        lines.extend([
            "이 세션/선택 범위에 평가 가능한 가시성 마스크가 없어 검출률을 계산하지 않았습니다.",
            "투영 GT 대비 관측 비율은 YOLO 검출률이 아닙니다.",
        ])
        return lines
    lines.extend([
        (f"- 마스크 정렬 프레임: {summary.get('visibility_mask_frames_aligned', 0)}"
         f"/{summary['frames_scored']}; 유효하지 않은 마스크: "
         f"{summary.get('visibility_mask_frames_invalid', 0)}"),
        (f"- 가시 GT: 같은 프레임 인스턴스 마스크에서 차량 픽셀 "
         f"{MIN_VISIBLE_PIXELS}개 이상, 가시 박스 각 변 "
         f"{MIN_VISIBLE_BOX_SIDE_PX}px 이상인 투영 GT."),
        (f"- YOLO 검출: 차량 클래스 박스와 가시 박스의 IoU "
         f"{DETECTION_IOU:g} 이상, 1:1 매칭. 검출률의 분모는 가시 GT 차량·프레임 수."),
        "- 이 값은 가시 GT에 대한 재현율이며, GT에 연결되지 않은 YOLO 박스의 정밀도는 포함하지 않습니다.",
        "- 미매칭에는 미검출과 박스 위치·크기 불일치가 함께 포함됩니다.",
        "- 마스크는 다른 물체에 의한 기하학적 가림을 반영하지만 RGB 영상의 야간 조도·비·안개 속 식별 가능성은 판정하지 않습니다.",
        "- 이전 세션과 마스크 누락 프레임은 검출률 분모에서 제외합니다.",
        "", "| 범위 | 마스크 정렬 투영 GT | 가시 GT V | YOLO 검출 D/V | 추적·GT 연결/V | 거리 산출/V |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    def row(label: str, item: dict[str, Any]) -> str:
        v = item["visible_gt_opportunities"]
        detected = item["yolo_detected_visible_gt"]
        rate = (f"{detected}/{v} ({detected / v * 100:.1f}%)"
                if v else "측정 불가")
        return (f"| {label} | {item['mask_aligned_projected_gt']} | {v} | {rate} | "
                f"{item['tracking_matched_visible_gt']}/{v} | "
                f"{item['distance_samples_visible_gt']}/{v} |")
    lines.append(row("전체", visibility))
    for band in summary.get("distance_bands", []):
        lines.append(row(band["label"], band["visibility"]))
    return lines


def evaluate_session(session_dir: Path, output_dir: Path | None = None,
                     min_distance_m: float = 0.0,
                     max_distance_m: float | None = None,
                     include_max: bool = True,
                     rows_out: list[dict[str, Any]] | None = None,
                     max_lost_frames_override: int | None = None,
                     radar_track_correction_override: bool | None = None) -> dict[str, Any]:
    """Replay all frames once, then report selected and fixed GT distance ranges."""
    if (not math.isfinite(min_distance_m) or min_distance_m < 0
            or (max_distance_m is not None and
                (not math.isfinite(max_distance_m) or max_distance_m <= min_distance_m))):
        raise ValueError("거리 범위는 0m 이상이며 최대 거리는 최소 거리보다 커야 합니다.")
    if ((max_lost_frames_override is not None
         or radar_track_correction_override is not None) and output_dir is None):
        raise ValueError("추적 설정을 바꿔 재분석할 때는 별도의 output_dir가 필요합니다")
    session_dir = Path(session_dir).resolve()
    required = ("session.json", "measurements.jsonl", "detections.jsonl", "ground_truth.jsonl")
    missing = [name for name in required if not (session_dir / name).is_file()]
    if missing:
        raise ValueError(f"평가에 필요한 기록이 없습니다: {', '.join(missing)}")
    metadata = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    if metadata.get("recording_mode") not in ("tracking", "radar_fusion"):
        raise ValueError("GT를 저장한 트래킹 또는 레이더 융합 세션을 선택하세요.")

    max_lost_frames = (metadata.get("tracking_max_lost_frames", 10)
                       if max_lost_frames_override is None else max_lost_frames_override)
    if (isinstance(max_lost_frames, bool) or not isinstance(max_lost_frames, int)
            or not 0 <= max_lost_frames <= 100):
        raise ValueError("Invalid tracking_max_lost_frames in session.json")
    radar_correction_enabled = (
        metadata.get("radar_track_correction_enabled", False)
        if radar_track_correction_override is None
        else radar_track_correction_override
    )
    if not isinstance(radar_correction_enabled, bool):
        raise ValueError("Invalid radar_track_correction_enabled in session.json")

    cameras: dict[tuple[int, int], dict[str, Any]] = {}
    radars: dict[int, dict[str, Any]] = {}
    for record in _records(session_dir / "measurements.jsonl"):
        if record.get("kind") == "camera":
            cameras[_key(record)] = record
        elif record.get("kind") == "radar":
            radars[int(record["frame"])] = record
    truths = {int(record["frame"]): record for record in _records(session_dir / "ground_truth.jsonl")}
    mask_index = session_dir / "visibility_masks.jsonl"
    mask_records = ({int(record["frame"]): record for record in _records(mask_index)}
                    if mask_index.is_file() else {})
    detections = list(_records(session_dir / "detections.jsonl"))
    detections.sort(key=lambda record: (int(record["frame"]), int(record.get("sensor_id", -1))))
    if not detections or not truths or not cameras:
        raise ValueError("YOLO 검출, 카메라 또는 CARLA GT 기록이 비어 있습니다.")

    vision_tracker = MultiObjectTracker(max_lost_frames=max_lost_frames)
    radar_tracker = MultiObjectTracker(
        max_lost_frames=max_lost_frames,
        radar_correction_enabled=radar_correction_enabled,
    )
    vision_positioner = PositioningEngine()
    radar_positioner = PositioningEngine()
    rows: list[dict[str, Any]] = []
    frames_scored = 0
    radar_deltas: dict[str, int] = {}
    ego_models: set[str] = set()
    camera_mount = radar_mount = None
    aligned_mask_frames = invalid_mask_frames = 0

    for detection in detections:
        frame = int(detection["frame"])
        camera_record = cameras.get(_key(detection))
        truth = truths.get(frame)
        if camera_record is None or truth is None:
            continue  # Strict frame alignment for GT and image.
        camera = SimpleNamespace(
            frame=frame,
            timestamp=float(camera_record["simulation_timestamp"]),
            width=int(camera_record["width"]),
            height=int(camera_record["height"]),
            fov=float(camera_record["fov"]),
            sensor_transform=camera_record.get("sensor_transform"),
        )
        radar = radars.get(frame)
        delta = 0 if radar is not None else None
        radar_deltas[str(delta) if delta is not None else "missing"] = (
            radar_deltas.get(str(delta) if delta is not None else "missing", 0) + 1
        )
        boxes = tuple(Box(**box) for box in detection.get("boxes", []))
        matches = match_radar_to_boxes(camera, boxes, radar, truth)
        radar_points = (radar_tracking_points(camera, radar, truth)
                        if radar_correction_enabled else ())
        timestamp = float(detection.get("receive_timestamp", camera.timestamp))
        vision_tracks = vision_tracker.update(boxes, timestamp)
        radar_tracks = radar_tracker.update(boxes, timestamp, matches, radar_points)
        vision_result = vision_positioner.estimate(
            camera, boxes, vision_tracks, radar, truth, delta, 0
        )
        radar_result = radar_positioner.estimate(
            camera, boxes, radar_tracks, radar, truth, delta, 0
        )
        frames_scored += 1
        if camera_mount is None:
            camera_mount = _mount(camera_record, truth)
        if radar_mount is None:
            radar_mount = _mount(radar, truth)

        eligible, frame_ego_models, _ = _eligible_truth(camera, truth)
        ego_models.update(frame_ego_models)
        actors_by_id = {int(actor["actor_id"]): actor
                        for actor in truth.get("vehicles", [])}
        actor_models = {actor_id: str(actor.get("type_id", "unknown"))
                        for actor_id, actor in actors_by_id.items()}
        visible: dict[int, dict[str, Any]] = {}
        visible_detection_ids: set[int] = set()
        mask_available = False
        mask_record = mask_records.get(frame)
        if mask_record is not None:
            actors = {int(actor["actor_id"]): actor for actor in truth.get("vehicles", [])}
            projected = {actor_id: box for actor_id in eligible
                         if (box := _project_actor_box(actors[actor_id], camera)) is not None}
            try:
                if any(actor_id > 0xffff for actor_id in projected):
                    raise ValueError("가시성 마스크의 16비트 ID 범위를 넘는 차량 ID")
                visible = visible_vehicle_boxes(
                    session_dir / mask_record["image"], projected,
                    camera.width, camera.height,
                )
                visible_detection_ids = match_visible_detections(visible, boxes)
                mask_available = True
                aligned_mask_frames += 1
            except (ValueError, KeyError, OSError):
                invalid_mask_frames += 1

        vision_by_actor = {obj["ground_truth_actor_id"]: obj for obj in vision_result["objects"]
                           if obj["ground_truth_actor_id"] in eligible}
        radar_by_actor = {obj["ground_truth_actor_id"]: obj for obj in radar_result["objects"]
                          if obj["ground_truth_actor_id"] in eligible}
        for actor_id, (gt_x, gt_y, gt_distance) in eligible.items():
            for method in METHODS:
                obj = (radar_by_actor if method == "radar_tracking_fusion" else vision_by_actor).get(actor_id)
                field = "camera_xy_ego_m" if method == "camera_tracking" else "fused_xy_ego_m"
                estimate = obj.get(field) if obj else None
                camera_distance = obj.get("camera_distance_m") if obj else None
                raw_camera_distance = obj.get("raw_camera_distance_m") if obj else None
                radar_distance = obj.get("radar_distance_m") if obj else None
                fused_distance = obj.get("estimated_distance_m") if obj else None
                estimated_distance = (camera_distance if method == "camera_tracking"
                                      else fused_distance)
                rejection_reason = (obj.get("distance_rejection_reason") if obj else None)
                if (method == "camera_tracking" and rejection_reason is not None
                        and not rejection_reason.startswith("prediction_")):
                    rejection_reason = None
                if method == "camera_tracking":
                    position_source = ("camera" if camera_distance is not None
                                       else "unavailable")
                else:
                    position_source = (obj.get("distance_source", "unavailable")
                                       if obj else "unavailable")
                detection_matched = bool(obj and obj["observed"]
                                         and obj["ground_truth_iou"] is not None)
                raw_signed_error = (estimated_distance - gt_distance
                                    if estimated_distance is not None else None)
                signed_error = raw_signed_error if detection_matched else None
                predicted_error = (abs(raw_signed_error) if obj and not obj["observed"]
                                   and raw_signed_error is not None else None)
                tracking_signed_error = (
                    raw_signed_error if (detection_matched or obj and not obj["observed"])
                    else None
                )
                raw_position_error = (math.hypot(float(estimate["forward_m"]) - gt_x,
                                                 float(estimate["right_m"]) - gt_y)
                                      if estimate is not None else None)
                position_error = raw_position_error if detection_matched else None
                predicted_position_error = (
                    raw_position_error if obj and not obj["observed"] else None
                )
                camera_xy = obj.get("camera_xy_ego_m") if obj else None
                predicted_camera_position_error = (
                    math.hypot(float(camera_xy["forward_m"]) - gt_x,
                               float(camera_xy["right_m"]) - gt_y)
                    if obj and not obj["observed"] and camera_xy is not None else None
                )
                projected_box = (_project_actor_box(actors_by_id[actor_id], camera)
                                 if obj and not obj["observed"] else None)
                track_box = obj.get("bbox") if obj else None
                predicted_bbox_center_error = (
                    math.hypot(
                        float(track_box["x"]) + float(track_box["width"]) / 2.0
                        - (projected_box["x"] + projected_box["width"] / 2.0),
                        float(track_box["y"]) + float(track_box["height"]) / 2.0
                        - (projected_box["y"] + projected_box["height"] / 2.0),
                    ) if projected_box is not None and track_box is not None else None
                )
                rows.append({
                    "frame": frame,
                    "simulation_timestamp": camera.timestamp,
                    "gt_actor_id": actor_id,
                    "gt_vehicle_model": actor_models[actor_id],
                    "gt_distance_m": gt_distance,
                    "distance_band": _band_for_distance(gt_distance),
                    "visibility_mask_available": mask_available,
                    "visible_pixels": (visible.get(actor_id, {}).get("pixels", 0)
                                       if mask_available else None),
                    "visible_gt": (bool(visible.get(actor_id, {}).get("eligible"))
                                   if mask_available else None),
                    "yolo_detected_visible_gt": (actor_id in visible_detection_ids
                                                 if mask_available else None),
                    "gt_forward_m": gt_x,
                    "gt_right_m": gt_y,
                    "camera_distance_m": camera_distance,
                    "raw_camera_distance_m": raw_camera_distance,
                    "radar_distance_m": radar_distance,
                    "fused_distance_m": fused_distance,
                    "estimated_distance_m": estimated_distance,
                    "distance_error_signed_m": signed_error,
                    "tracking_distance_error_signed_m": tracking_signed_error,
                    "camera_radar_gap_m": (abs(radar_distance - camera_distance)
                                           if radar_distance is not None
                                           and camera_distance is not None else None),
                    "radar_points": obj.get("radar_points") if obj else None,
                    "radar_cluster_spread_m": obj.get("radar_cluster_spread_m") if obj else None,
                    "distance_rejection_reason": rejection_reason,
                    "prediction_jump_m": (obj.get("prediction_jump_m") if obj else None),
                    "prediction_jump_limit_m": (
                        obj.get("prediction_jump_limit_m") if obj else None),
                    "prediction_pixel_sensitivity_m": (
                        obj.get("prediction_pixel_sensitivity_m") if obj else None),
                    "method": method,
                    "track_id": obj["track_id"] if obj else None,
                    "observed": obj["observed"] if obj else False,
                    "missed_frames": obj.get("missed_frames") if obj else None,
                    "radar_corrected": obj.get("radar_corrected", False) if obj else False,
                    "radar_correction_points": obj.get("radar_correction_points", 0) if obj else 0,
                    "detection_matched": detection_matched,
                    "estimate_forward_m": estimate["forward_m"] if estimate else None,
                    "estimate_right_m": estimate["right_m"] if estimate else None,
                    "error_m": abs(signed_error) if signed_error is not None else None,
                    "predicted_error_m": predicted_error,
                    "tracking_error_m": (abs(tracking_signed_error)
                                         if tracking_signed_error is not None else None),
                    "position_error_m": position_error,
                    "predicted_position_error_m": predicted_position_error,
                    "predicted_camera_position_error_m": predicted_camera_position_error,
                    "predicted_bbox_center_error_px": predicted_bbox_center_error,
                    "radar_used": (obj.get("distance_source") == "radar_near_cluster"
                                   if obj and method != "camera_tracking" else False),
                    "position_source": position_source,
                    "radar_frame_delta": delta,
                })

    if not frames_scored:
        raise ValueError("카메라·YOLO·GT가 같은 프레임인 기록을 찾지 못했습니다.")
    if rows_out is not None:
        rows_out.extend(rows)
    selected_rows = _selected_rows(rows, min_distance_m, max_distance_m, include_max)
    legacy_rows = _selected_rows(rows, LEGACY_MIN_DISTANCE_M, LEGACY_MAX_DISTANCE_M)
    selected_stats = _summary_for_rows(selected_rows)
    legacy_stats = _summary_for_rows(legacy_rows)
    bands = []
    for label, low, high in DISTANCE_BANDS:
        if high is not None and high <= min_distance_m:
            continue
        if (max_distance_m is not None and
                (low > max_distance_m or (low == max_distance_m and not include_max))):
            continue
        band_rows = [row for row in selected_rows if low <= row["gt_distance_m"]
                     and (high is None or row["gt_distance_m"] < high)]
        bands.append({"label": label, "min_inclusive": low, "max_exclusive": high,
                      **_summary_for_rows(band_rows)})
    summary: dict[str, Any] = {
        "schema_version": 5,
        "session": str(session_dir),
        "tracking_max_lost_frames": max_lost_frames,
        "radar_track_correction_enabled": radar_correction_enabled,
        "weather": metadata.get("weather"),
        "weather_workflow": metadata.get("weather_workflow"),
        "distance_filter_m": {"min_inclusive": min_distance_m, "max_m": max_distance_m,
                              "include_max": include_max,
                              "reference": "shortest planar gap between ego and target CARLA bounding-box footprints"},
        "frames_scored": frames_scored,
        "visibility_mask_frames_recorded": len(mask_records),
        "visibility_mask_frames_aligned": aligned_mask_frames,
        "visibility_mask_frames_invalid": invalid_mask_frames,
        **selected_stats,
        "distance_bands": bands,
        "legacy_3m_15m": legacy_stats,
        "radar_frame_deltas": radar_deltas,
        "ego_vehicle_models": sorted(ego_models),
        "target_vehicle_models": sorted({row["gt_vehicle_model"] for row in selected_rows}),
        "camera_mount": camera_mount,
        "radar_mount": radar_mount,
        "camera_intrinsics": {"width": camera.width, "height": camera.height, "fov_deg": camera.fov},
        "notes": [
            "GT 거리는 같은 프레임의 두 차량 CARLA bounding-box 평면 외곽 사이 최단 거리다.",
            "카메라와 레이더 거리 추정은 관측한 상대 차량 지점부터 내 차량 외곽까지의 거리다.",
            "레이더 가까운 묶음이 3점 이상이고 거리 중앙값에서 최대 편차 0.5m 이내이며 카메라 거리와 1m 이내로 일치할 때 레이더 거리를 선택한다.",
            "상대 차량의 GT 크기와 자세는 거리 추정에 사용하지 않고 평가에만 사용한다.",
            "관측 지점이 상대 차량에서 가장 가까운 지점이 아닐 수 있어 추정 간격에 양의 편향이 생길 수 있다.",
            "기본 거리 구간은 상한을 제외하고 사용자 지정 범위는 상한을 포함한다.",
            "기존 3~15m 전체 수치는 선택 범위와 무관하게 3m와 15m를 모두 포함한다.",
            "최종 거리 MAE/RMSE는 현재 YOLO 관측 트랙 또는 유지 중인 예측 트랙 중 GT와 연결되어 거리를 산출한 차량·프레임을 한 번씩 사용한다.",
            "관측과 예측 거리 표본 수 및 MAE는 최종 지표의 구성 항목으로 따로 표시한다.",
            "예측 트랙의 카메라 지면 투영은 마지막 관측 월드 위치에서 3m+20m/s×경과시간 이내이고 1픽셀 민감도가 max(3m, 추정 거리의 10%) 이내일 때만 거리로 인정한다.",
            "급변 거리를 거부해도 트랙은 유지하며, 거부된 예측 거리는 최종 MAE/RMSE 표본에 포함하지 않는다.",
            "관측 거리 산출률의 분모는 현재 YOLO 관측·GT 연결 수이며, 투영 GT 수는 YOLO 검출률의 분모로 사용할 수 없다.",
            "0m GT는 차량 bounding box의 평면 외곽이 닿거나 겹친 경우다.",
            "카메라·레이더·GT는 CARLA 프레임 번호가 같은 측정만 사용한다.",
            "레이더를 사용할 수 없으면 품질 검사를 통과한 카메라 거리로 대체하며, 유효한 거리도 없으면 미산출로 둔다.",
            "투영 GT 수는 화면 안에 상자가 투영된 차량 기준이며 실제 가림 여부나 픽셀 가시성을 판단하지 않는다.",
            "가시 GT는 같은 프레임 인스턴스 분할 마스크의 차량 픽셀 50개 이상, 가시 박스 각 변 8px 이상 기준이다.",
            "마스크 가시성은 기하학적 노출이며 야간 조도·안개에서 RGB로 식별 가능한지는 보장하지 않는다.",
            "YOLO 검출률은 가시 GT와 차량 검출 박스의 가시 박스 IoU 0.5 이상 1:1 연결 기준이다.",
            "ID switch는 관측된 GT 연관 트랙 ID가 이전 관측과 달라진 횟수다.",
        ],
    }
    destination = Path(output_dir).resolve() if output_dir else session_dir / "experiment_results"
    destination.mkdir(parents=True, exist_ok=True)
    if min_distance_m == 0 and max_distance_m is None:
        suffix = "all"
    elif max_distance_m is None:
        suffix = f"{min_distance_m:g}m_plus"
    else:
        suffix = (f"{min_distance_m:g}m_{max_distance_m:g}m"
                  + ("_exclusive" if not include_max else ""))
    csv_path = destination / f"position_errors_distance_profile_{suffix}.csv"
    json_path = destination / f"summary_distance_profile_{suffix}.json"
    report_path = destination / f"report_distance_profile_{suffix}.md"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(selected_rows[0]) if selected_rows else [
            "frame", "simulation_timestamp", "gt_actor_id", "gt_vehicle_model", "gt_distance_m", "distance_band",
            "gt_forward_m", "gt_right_m", "camera_distance_m", "raw_camera_distance_m",
            "radar_distance_m",
            "fused_distance_m", "estimated_distance_m", "distance_error_signed_m",
            "tracking_distance_error_signed_m",
            "camera_radar_gap_m", "radar_points", "radar_cluster_spread_m",
            "distance_rejection_reason", "prediction_jump_m",
            "prediction_jump_limit_m", "prediction_pixel_sensitivity_m",
            "method", "track_id", "observed", "detection_matched",
            "estimate_forward_m", "estimate_right_m", "error_m", "predicted_error_m",
            "tracking_error_m",
            "position_error_m", "predicted_position_error_m", "radar_corrected",
            "radar_used",
            "position_source", "radar_frame_delta",
        ])
        writer.writeheader()
        writer.writerows(selected_rows)
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    def fmt(value: float | None) -> str:
        return f"{value:.3f}" if value is not None else "-"

    def fmt_m(value: float | None) -> str:
        return f"{value:.3f}m" if value is not None else "측정 없음"

    report = [
        "# 차량 간 거리 추정 실험 결과",
        "",
        f"- 세션: `{session_dir}`",
        f"- 날씨: `{json.dumps(metadata.get('weather'), ensure_ascii=False)}`",
        (f"- 워크플로 날씨 프리셋: `{metadata['weather_workflow']['weather_preset']}`"
         if metadata.get("weather_workflow") else "- 워크플로 날씨 프리셋: 없음"),
        (f"- 선택 범위: CARLA GT 차체 외곽 간격 {min_distance_m:g}m 이상"
         + (f", {max_distance_m:g}m {'이하' if include_max else '미만'}"
            if max_distance_m is not None else ", 상한 없음")),
        f"- 평가 프레임: {frames_scored}",
        f"- 화면에 투영된 GT 차량·프레임 조합(가림 미판정): {summary['eligible_gt_opportunities']}",
        f"- 세 방법 모두 관측 또는 예측으로 거리를 낸 공통 표본: {summary['common_samples']}",
        f"- 내 차량 모델: {', '.join(summary['ego_vehicle_models']) or '기록 없음'}",
        "- 최종 거리 MAE/RMSE의 분모: 현재 YOLO 관측 거리 표본 OR 유지 트랙 예측 거리 표본(차량·프레임당 한 번).",
        "", "## 선택 범위의 방법별 결과", "",
        "| 방법 | 관측 매칭 | 거리 표본 관측+예측=전체 | 최종 거리 MAE (m) | 최종 RMSE (m) | 최종 편향 (m) | 공통 표본 MAE (m) | 관측 MAE (m) | 예측 MAE (m) | 관측 위치 MAE (m) | ID 변경 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    report.insert(2, f"- 추적 유지: {max_lost_frames} 처리 프레임")
    report.insert(3, f"- YOLO 공백 중 레이더 위치 보정: {'적용' if radar_correction_enabled else '미적용'}")
    for method, label in METHODS.items():
        result = summary["methods"][method]
        report.append(
            f"| {label} | {result['observed_matches']} | "
            f"{result['observed_distance_samples']}+{result['predicted_distance_samples']}={result['distance_samples']} | "
            f"{fmt(result['mae_m'])} | {fmt(result['rmse_m'])} | "
            f"{fmt(result['distance_bias_m'])} | "
            f"{fmt(summary['common_sample_mae_m'][method])} | "
            f"{fmt(result['observed_mae_m'])} | {fmt(result['predicted_mae_m'])} | "
            f"{fmt(result['position_mae_m'])} | {result['id_switches']} |"
        )
    report.extend([
        "", "## 예측 거리 품질 검사", "",
        "트랙 ID는 유지하되 마지막 관측 위치에서 물리적으로 불가능하게 이동하거나 박스 지면 투영이 불안정하면 해당 프레임의 거리는 산출하지 않습니다. GT 오차를 보고 거부하지 않습니다.",
        "", "| 방법 | GT 연결 예측 트랙 | 거리 산출 | 거리 미산출 | 품질 검사 거부 | 미산출 이유별 건수 |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ])
    for method, label in METHODS.items():
        result = summary["methods"][method]
        reasons = ", ".join(
            f"{reason}: {count}"
            for reason, count in sorted(result["prediction_distance_rejection_reasons"].items())
        ) or "없음"
        report.append(
            f"| {label} | {result['predicted_track_matches']} | "
            f"{result['predicted_distance_samples']} | "
            f"{result['prediction_distance_unavailable']} | "
            f"{result['prediction_distance_rejected']} | {reasons} |"
        )
    radar_prediction = summary["methods"]["radar_tracking_fusion"]
    report.extend([
        "", "## YOLO 미검출 중 레이더 보정", "",
        "레이더 보정 프레임은 현재 YOLO 관측과 구분해 집계하며, 거리 산출·GT 연결이 가능하면 최종 거리 MAE/RMSE에도 포함합니다.",
        f"- 레이더 보정 예측: {radar_prediction['radar_corrected_predicted_frames']}프레임, "
        f"거리 평가 {radar_prediction['radar_corrected_predicted_samples']}건, "
        f"MAE {fmt(radar_prediction['radar_corrected_predicted_mae_m'])}m",
        f"- 레이더 보정 예측의 평면 위치 평가: "
        f"{radar_prediction['radar_corrected_predicted_position_samples']}건, "
        f"MAE {fmt(radar_prediction['radar_corrected_predicted_position_mae_m'])}m",
        f"- 레이더 보정 예측 박스 중심과 투영 GT 중심 차이: "
        f"{fmt(radar_prediction['radar_corrected_predicted_bbox_center_mae_px'])}px "
        "(가림 미판정 기하학적 기준)",
        f"- 보정 없는 예측: 거리 평가 "
        f"{radar_prediction['uncorrected_predicted_samples']}건, "
        f"MAE {fmt(radar_prediction['uncorrected_predicted_mae_m'])}m",
    ])
    report.extend(_visibility_report_lines(summary))
    report.extend([
        "", "## 거리 구간별 결과", "",
        "구간은 하한을 포함하고 상한은 제외합니다. 선택 범위 밖의 표본은 집계하지 않습니다. `-`는 거리 표본이 없어 측정할 수 없다는 뜻입니다.", "",
        "| GT 거리 | 방법 | 관측 매칭/투영 GT | 관측 거리/관측 매칭 | 예측 거리 | 전체 거리 | 최종 MAE (m) | 최종 RMSE (m) | 편향 (m) | 공통 표본 MAE (m) | 관측 MAE (m) | 예측 MAE (m) | 레이더 선택/전체 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for band in bands:
        for method, label in METHODS.items():
            result = band["methods"][method]
            radar_share = (f"{result['radar_used_samples']}/{result['distance_samples']}"
                           if method != "camera_tracking" else "-")
            report.append(
                f"| {band['label']} | {label} | "
                f"{result['observed_matches']}/{result['gt_opportunities']} | "
                f"{result['observed_distance_samples']}/{result['observed_matches']} | "
                f"{result['predicted_distance_samples']} | {result['distance_samples']} | "
                f"{fmt(result['mae_m'])} | {fmt(result['rmse_m'])} | "
                f"{fmt(result['distance_bias_m'])} | "
                f"{fmt(band['common_sample_mae_m'][method])} | "
                f"{fmt(result['observed_mae_m'])} | {fmt(result['predicted_mae_m'])} | "
                f"{radar_share} |"
            )
    report.append(f"\n0m GT(차체 외곽 접촉·겹침): {summary['zero_distance_opportunities']}건")
    report.extend([
        "", "## 3~15m 전체 추적 거리 결과", "",
        f"- 투영 GT 차량·프레임 조합: {legacy_stats['eligible_gt_opportunities']}",
        f"- 세 방법 공통 추적 거리 표본: {legacy_stats['common_samples']}",
        "", "| 방법 | 관측 매칭 | 관측+예측=전체 거리 표본 | 최종 거리 MAE (m) | 최종 RMSE (m) | 최종 편향 (m) | 공통 표본 MAE (m) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for method, label in METHODS.items():
        result = legacy_stats["methods"][method]
        report.append(
            f"| {label} | {result['observed_matches']} | "
            f"{result['observed_distance_samples']}+{result['predicted_distance_samples']}={result['distance_samples']} | "
            f"{fmt(result['mae_m'])} | {fmt(result['rmse_m'])} | "
            f"{fmt(result['distance_bias_m'])} | "
            f"{fmt(legacy_stats['common_sample_mae_m'][method])} |"
        )
    report.extend(["", "## 레이더 사용과 카메라 대체", ""])
    for method in ("vision_radar_fusion", "radar_tracking_fusion"):
        result = summary["methods"][method]
        report.append(
            f"- {METHODS[method]}: 레이더 사용 {result['radar_used_samples']}건 "
            f"(MAE {fmt_m(result['radar_used_mae_m'])}), 카메라 대체 "
            f"{result['camera_fallback_samples']}건 "
            f"(MAE {fmt_m(result['camera_fallback_mae_m'])})"
        )
    report.extend(["", "## 측정 조건", ""] + [f"- {note}" for note in summary["notes"]])
    report.extend([
        "", "## 센서 장착 정보", "",
        f"- 카메라: `{json.dumps(camera_mount, ensure_ascii=False)}`",
        f"- 레이더: `{json.dumps(radar_mount, ensure_ascii=False)}`",
        "", f"세부 차량별 거리·위치 오차는 `{csv_path.name}`에서 확인할 수 있다.", "",
    ])
    report_path.write_text("\n".join(report), encoding="utf-8")
    summary["summary_path"] = str(json_path)
    summary["csv_path"] = str(csv_path)
    summary["report_path"] = str(report_path)
    return summary


class ExperimentEvaluatorThread(QThread):
    completed = Signal(object)
    failed = Signal(str)

    def __init__(self, session_dir: Path, min_distance_m: float = 0.0,
                 max_distance_m: float | None = None, include_max: bool = True,
                 parent: Any = None) -> None:
        super().__init__(parent)
        self.session_dir = session_dir
        self.min_distance_m = min_distance_m
        self.max_distance_m = max_distance_m
        self.include_max = include_max

    def run(self) -> None:
        try:
            from monitor.qt.weather_workflow_evaluator import (
                evaluate_weather_workflow, is_weather_workflow,
            )
            evaluator = evaluate_weather_workflow if is_weather_workflow(self.session_dir) else evaluate_session
            self.completed.emit(evaluator(
                self.session_dir, min_distance_m=self.min_distance_m,
                max_distance_m=self.max_distance_m, include_max=self.include_max))
        except Exception as exc:
            self.failed.emit(str(exc))
