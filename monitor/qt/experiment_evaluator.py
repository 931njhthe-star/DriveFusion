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
)
from monitor.qt.tracker import MultiObjectTracker


METHODS = {
    "camera_tracking": "트래킹 + 카메라",
    "vision_radar_fusion": "트래킹 + 카메라·레이더 융합",
    "radar_tracking_fusion": "레이더 융합 트래킹 + 카메라·레이더 융합",
}
MIN_DISTANCE_M = 3.0
MAX_DISTANCE_M = 15.0


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
        if MIN_DISTANCE_M <= distance <= MAX_DISTANCE_M and _project_actor_box(actor, camera):
            eligible[actor_id] = (gt_x, gt_y, distance)
            target_models.add(str(actor.get("type_id", "unknown")))
    return eligible, ego_models, target_models


def _stats(rows: list[dict[str, Any]], method: str) -> dict[str, Any]:
    selected = [row for row in rows if row["method"] == method]
    errors = [row["error_m"] for row in selected if row["error_m"] is not None]
    signed_errors = [row["distance_error_signed_m"] for row in selected
                     if row.get("distance_error_signed_m") is not None]
    position_errors = [row["position_error_m"] for row in selected
                       if row.get("position_error_m") is not None]
    radar_errors = [row["error_m"] for row in selected
                    if row["error_m"] is not None and row["radar_used"]]
    fallback_errors = [row["error_m"] for row in selected
                       if row["error_m"] is not None and not row["radar_used"]]
    switches = 0
    previous: dict[int, int] = {}
    for row in selected:
        track_id = row["track_id"]
        actor_id = row["gt_actor_id"]
        if track_id is None or not row["observed"]:
            continue
        if actor_id in previous and previous[actor_id] != track_id:
            switches += 1
        previous[actor_id] = track_id
    return {
        "gt_opportunities": len(selected),
        "distance_samples": len(errors),
        "position_samples": len(errors),
        "coverage": len(errors) / len(selected) if selected else None,
        "mae_m": sum(errors) / len(errors) if errors else None,
        "rmse_m": math.sqrt(sum(error * error for error in errors) / len(errors)) if errors else None,
        "distance_bias_m": (sum(signed_errors) / len(signed_errors)
                            if signed_errors else None),
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


def evaluate_session(session_dir: Path, output_dir: Path | None = None) -> dict[str, Any]:
    """Write per-target CSV and JSON summary for GT distances from 3 through 15 m."""
    session_dir = Path(session_dir).resolve()
    required = ("session.json", "measurements.jsonl", "detections.jsonl", "ground_truth.jsonl")
    missing = [name for name in required if not (session_dir / name).is_file()]
    if missing:
        raise ValueError(f"평가에 필요한 기록이 없습니다: {', '.join(missing)}")
    metadata = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    if metadata.get("recording_mode") not in ("tracking", "radar_fusion"):
        raise ValueError("GT를 저장한 트래킹 또는 레이더 융합 세션을 선택하세요.")

    cameras: dict[tuple[int, int], dict[str, Any]] = {}
    radars: dict[int, dict[str, Any]] = {}
    for record in _records(session_dir / "measurements.jsonl"):
        if record.get("kind") == "camera":
            cameras[_key(record)] = record
        elif record.get("kind") == "radar":
            radars[int(record["frame"])] = record
    truths = {int(record["frame"]): record for record in _records(session_dir / "ground_truth.jsonl")}
    detections = list(_records(session_dir / "detections.jsonl"))
    detections.sort(key=lambda record: (int(record["frame"]), int(record.get("sensor_id", -1))))
    if not detections or not truths or not cameras:
        raise ValueError("YOLO 검출, 카메라 또는 CARLA GT 기록이 비어 있습니다.")

    vision_tracker = MultiObjectTracker()
    radar_tracker = MultiObjectTracker()
    vision_positioner = PositioningEngine()
    radar_positioner = PositioningEngine()
    rows: list[dict[str, Any]] = []
    frames_scored = 0
    radar_deltas: dict[str, int] = {}
    vehicle_models: set[str] = set()
    ego_models: set[str] = set()
    camera_mount = radar_mount = None

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
        timestamp = float(detection.get("receive_timestamp", camera.timestamp))
        vision_tracks = vision_tracker.update(boxes, timestamp)
        radar_tracks = radar_tracker.update(boxes, timestamp, matches)
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

        eligible, frame_ego_models, frame_target_models = _eligible_truth(camera, truth)
        ego_models.update(frame_ego_models)
        vehicle_models.update(frame_target_models)
        actor_models = {int(actor["actor_id"]): str(actor.get("type_id", "unknown"))
                        for actor in truth.get("vehicles", [])}

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
                radar_distance = obj.get("radar_distance_m") if obj else None
                fused_distance = obj.get("estimated_distance_m") if obj else None
                estimated_distance = (camera_distance if method == "camera_tracking"
                                      else fused_distance)
                signed_error = (estimated_distance - gt_distance
                                if estimated_distance is not None else None)
                position_error = (math.hypot(float(estimate["forward_m"]) - gt_x,
                                             float(estimate["right_m"]) - gt_y)
                                  if estimate is not None else None)
                rows.append({
                    "frame": frame,
                    "simulation_timestamp": camera.timestamp,
                    "gt_actor_id": actor_id,
                    "gt_vehicle_model": actor_models[actor_id],
                    "gt_distance_m": gt_distance,
                    "gt_forward_m": gt_x,
                    "gt_right_m": gt_y,
                    "camera_distance_m": camera_distance,
                    "radar_distance_m": radar_distance,
                    "fused_distance_m": fused_distance,
                    "estimated_distance_m": estimated_distance,
                    "distance_error_signed_m": signed_error,
                    "camera_radar_gap_m": (abs(radar_distance - camera_distance)
                                           if radar_distance is not None
                                           and camera_distance is not None else None),
                    "radar_points": obj.get("radar_points") if obj else None,
                    "radar_cluster_spread_m": obj.get("radar_cluster_spread_m") if obj else None,
                    "distance_rejection_reason": (obj.get("distance_rejection_reason")
                                                  if obj and method != "camera_tracking" else None),
                    "method": method,
                    "track_id": obj["track_id"] if obj else None,
                    "observed": obj["observed"] if obj else False,
                    "estimate_forward_m": estimate["forward_m"] if estimate else None,
                    "estimate_right_m": estimate["right_m"] if estimate else None,
                    "error_m": abs(signed_error) if signed_error is not None else None,
                    "position_error_m": position_error,
                    "radar_used": (obj.get("distance_source") == "radar_near_cluster"
                                   if obj and method != "camera_tracking" else False),
                    "position_source": (obj.get("distance_source", "unavailable")
                                        if obj and method != "camera_tracking"
                                        else "camera" if obj else "unavailable"),
                    "radar_frame_delta": delta,
                })

    if not frames_scored:
        raise ValueError("카메라·YOLO·GT가 같은 프레임인 기록을 찾지 못했습니다.")
    summary: dict[str, Any] = {
        "schema_version": 2,
        "session": str(session_dir),
        "distance_filter_m": {"min_inclusive": MIN_DISTANCE_M, "max_inclusive": MAX_DISTANCE_M,
                              "reference": "shortest planar gap between ego and target CARLA bounding-box footprints"},
        "frames_scored": frames_scored,
        "eligible_gt_opportunities": len(rows) // len(METHODS),
        "radar_frame_deltas": radar_deltas,
        "ego_vehicle_models": sorted(ego_models),
        "target_vehicle_models": sorted(vehicle_models),
        "camera_mount": camera_mount,
        "radar_mount": radar_mount,
        "camera_intrinsics": {"width": camera.width, "height": camera.height, "fov_deg": camera.fov},
        "methods": {key: {"label": label, **_stats(rows, key)} for key, label in METHODS.items()},
        "notes": [
            "GT 거리는 같은 프레임의 두 차량 CARLA bounding-box 평면 외곽 사이 최단 거리다.",
            "카메라와 레이더 거리 추정은 관측한 상대 차량 지점부터 내 차량 외곽까지의 거리다.",
            "레이더 가까운 묶음이 3점 이상이고 거리 중앙값에서 최대 편차 0.5m 이내이며 카메라 거리와 1m 이내로 일치할 때 레이더 거리를 선택한다.",
            "상대 차량의 GT 크기와 자세는 거리 추정에 사용하지 않고 평가에만 사용한다.",
            "관측 지점이 상대 차량에서 가장 가까운 지점이 아닐 수 있어 추정 간격에 양의 편향이 생길 수 있다.",
            "거리 MAE/RMSE는 GT 차체 간격 3~15m에서 계산한다. 중심 위치 오차는 보조 지표다.",
            "카메라·레이더·GT는 CARLA 프레임 번호가 같은 측정만 사용한다.",
            "레이더를 사용할 수 없으면 카메라 거리로 대체한다.",
            "GT 기회 수는 카메라 화면에 투영 가능한 차량 기준이며 가림 여부는 판단하지 않는다.",
            "ID switch는 관측된 GT 연관 트랙 ID가 이전 관측과 달라진 횟수다.",
        ],
    }
    paired = {}
    for row in rows:
        paired.setdefault((row["frame"], row["gt_actor_id"]), {})[row["method"]] = row["error_m"]
    common = [values for values in paired.values()
              if all(values.get(method) is not None for method in METHODS)]
    summary["common_samples"] = len(common)
    summary["common_sample_mae_m"] = {
        method: sum(values[method] for values in common) / len(common) if common else None
        for method in METHODS
    }

    destination = Path(output_dir).resolve() if output_dir else session_dir / "experiment_results"
    destination.mkdir(parents=True, exist_ok=True)
    csv_path = destination / "position_errors_3m_15m.csv"
    json_path = destination / "summary_3m_15m.json"
    report_path = destination / "report_3m_15m.md"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]) if rows else [
            "frame", "simulation_timestamp", "gt_actor_id", "gt_vehicle_model", "gt_distance_m",
            "gt_forward_m", "gt_right_m", "camera_distance_m", "radar_distance_m",
            "fused_distance_m", "estimated_distance_m", "distance_error_signed_m",
            "camera_radar_gap_m", "radar_points", "radar_cluster_spread_m",
            "distance_rejection_reason",
            "method", "track_id", "observed", "estimate_forward_m", "estimate_right_m",
            "error_m", "position_error_m", "radar_used",
            "position_source", "radar_frame_delta",
        ])
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    def fmt(value: float | None) -> str:
        return f"{value:.3f}" if value is not None else "-"

    def fmt_m(value: float | None) -> str:
        return f"{value:.3f}m" if value is not None else "측정 없음"

    report = [
        "# 차량 간 거리 추정 실험 결과",
        "",
        f"- 세션: `{session_dir}`",
        f"- 거리 조건: CARLA GT 차체 외곽 간격 {MIN_DISTANCE_M:g}m 이상, {MAX_DISTANCE_M:g}m 이하",
        f"- 평가 프레임: {frames_scored}",
        f"- GT 차량·프레임 조합: {summary['eligible_gt_opportunities']}",
        f"- 세 방법 모두 거리를 낸 공통 표본: {summary['common_samples']}",
        f"- 내 차량 모델: {', '.join(summary['ego_vehicle_models']) or '기록 없음'}",
        "",
        "| 방법 | 거리 산출/GT 기회 | 거리 MAE (m) | 거리 RMSE (m) | 거리 편향 (m) | 공통 표본 거리 MAE (m) | 중심 위치 MAE (m) | ID 변경 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for method, label in METHODS.items():
        result = summary["methods"][method]
        report.append(
            f"| {label} | {result['distance_samples']}/{result['gt_opportunities']} | "
            f"{fmt(result['mae_m'])} | {fmt(result['rmse_m'])} | "
            f"{fmt(result['distance_bias_m'])} | "
            f"{fmt(summary['common_sample_mae_m'][method])} | "
            f"{fmt(result['position_mae_m'])} | {result['id_switches']} |"
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
        "", "세부 차량별 거리·위치 오차는 `position_errors_3m_15m.csv`에서 확인할 수 있다.", "",
    ])
    report_path.write_text("\n".join(report), encoding="utf-8")
    summary["summary_path"] = str(json_path)
    summary["csv_path"] = str(csv_path)
    summary["report_path"] = str(report_path)
    return summary


class ExperimentEvaluatorThread(QThread):
    completed = Signal(object)
    failed = Signal(str)

    def __init__(self, session_dir: Path, parent: Any = None) -> None:
        super().__init__(parent)
        self.session_dir = session_dir

    def run(self) -> None:
        try:
            self.completed.emit(evaluate_session(self.session_dir))
        except Exception as exc:
            self.failed.emit(str(exc))
