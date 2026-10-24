"""CARLA sensor collection and session recording for the Qt monitor."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time
from typing import Any, Callable

import carla
from PySide6.QtGui import QImage


WEATHER_FIELDS = (
    "cloudiness", "precipitation", "precipitation_deposits", "wetness",
    "wind_intensity", "fog_density", "fog_distance", "fog_falloff",
    "sun_azimuth_angle", "sun_altitude_angle", "scattering_intensity",
    "mie_scattering_scale", "rayleigh_scattering_scale", "dust_storm",
)


def weather_parameters(weather: carla.WeatherParameters) -> dict[str, float]:
    return {name: float(getattr(weather, name)) for name in WEATHER_FIELDS}


@dataclass(frozen=True)
class CameraFrame:
    frame: int
    timestamp: float
    received_at: float
    width: int
    height: int
    bgra: bytes
    sensor_id: int = 0
    fov: float = 90.0
    sensor_transform: dict[str, Any] | None = None


@dataclass(frozen=True)
class VisibilityMaskFrame:
    frame: int
    timestamp: float
    received_at: float
    width: int
    height: int
    bgra: bytes
    sensor_id: int


@dataclass(frozen=True)
class DetectionFrame:
    camera: CameraFrame
    boxes: tuple[Any, ...]
    processing_ms: float
    tracks: tuple[Any, ...] = ()
    positioning: dict[str, Any] | None = None
    vision_tracks: tuple[Any, ...] = ()


def vector(value: Any) -> dict[str, float]:
    return {"x": float(value.x), "y": float(value.y), "z": float(value.z)}


def transform(value: Any) -> dict[str, Any]:
    return {
        "location": vector(value.location),
        "rotation": {
            "pitch": float(value.rotation.pitch),
            "yaw": float(value.rotation.yaw),
            "roll": float(value.rotation.roll),
        },
    }


class LiveState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sensor_ready = threading.Condition(self._lock)
        self.status = "CARLA 연결 중"
        self.camera: CameraFrame | None = None
        self.detections: DetectionFrame | None = None
        self.radar: dict[str, Any] | None = None
        self.vehicle: dict[str, Any] | None = None
        self.imu: dict[str, Any] | None = None
        self.gnss: dict[str, Any] | None = None
        self.positioning: dict[str, Any] | None = None
        self.weather: dict[str, Any] | None = None
        self._radar_by_frame: dict[int, dict[str, Any]] = {}
        self._ground_truth_by_frame: dict[int, dict[str, Any]] = {}
        self.events: deque[dict[str, Any]] = deque(maxlen=12)
        self.arrivals: dict[str, deque[float]] = {
            name: deque(maxlen=120)
            for name in ("camera", "detections", "radar", "vehicle", "imu", "gnss",
                         "positioning", "ground_truth")
        }

    def set_status(self, text: str) -> None:
        with self._lock:
            self.status = text

    def clear(self, status: str) -> None:
        with self._lock:
            self.status = status
            self.camera = None
            self.detections = None
            self.radar = None
            self.vehicle = None
            self.imu = None
            self.gnss = None
            self.positioning = None
            self.weather = None
            self._radar_by_frame.clear()
            self._ground_truth_by_frame.clear()
            self.events.clear()
            for arrivals in self.arrivals.values():
                arrivals.clear()
            self._sensor_ready.notify_all()

    def update(self, name: str, value: Any) -> None:
        with self._lock:
            if name in ("collision", "lane_invasion"):
                self.events.appendleft(value)
            else:
                setattr(self, name, value)
                self.arrivals[name].append(time.monotonic())
                if name == "radar":
                    self._radar_by_frame[int(value["frame"])] = value
                    self._trim_history(self._radar_by_frame)
                    self._sensor_ready.notify_all()
                elif name == "ground_truth":
                    self._ground_truth_by_frame[int(value["frame"])] = value
                    self._trim_history(self._ground_truth_by_frame)
                    self._sensor_ready.notify_all()

    def set_weather(self, value: dict[str, Any]) -> None:
        with self._lock:
            self.weather = value

    @staticmethod
    def _trim_history(history: dict[int, dict[str, Any]], limit: int = 256) -> None:
        if len(history) > limit:
            for frame in sorted(history)[:-limit]:
                history.pop(frame, None)

    def position_context(
        self, camera_frame: int
    ) -> tuple[dict[str, Any] | None, int | None,
               dict[str, Any] | None, int | None]:
        with self._lock:
            radar = self._radar_by_frame.get(camera_frame)
            truth = self._ground_truth_by_frame.get(camera_frame)
            return radar, 0 if radar is not None else None, truth, 0 if truth is not None else None

    def wait_for_position_context(
        self, camera_frame: int, timeout_s: float = 0.05
    ) -> tuple[dict[str, Any] | None, int | None,
               dict[str, Any] | None, int | None]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._sensor_ready:
            while (camera_frame not in self._radar_by_frame
                   or camera_frame not in self._ground_truth_by_frame):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._sensor_ready.wait(remaining)
            radar = self._radar_by_frame.get(camera_frame)
            truth = self._ground_truth_by_frame.get(camera_frame)
            return radar, 0 if radar is not None else None, truth, 0 if truth is not None else None

    def get_camera(self) -> CameraFrame | None:
        with self._lock:
            return self.camera

    def clear_detections(self) -> None:
        with self._lock:
            self.detections = None
            self.positioning = None

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            rates = {}
            for name, arrivals in self.arrivals.items():
                recent = [item for item in arrivals if now - item <= 2.0]
                rates[name] = (
                    (len(recent) - 1) / (recent[-1] - recent[0])
                    if len(recent) >= 2 and recent[-1] > recent[0]
                    else 0.0
                )
            return {
                "status": self.status,
                "camera": self.camera,
                "detections": self.detections,
                "radar": self.radar,
                "vehicle": self.vehicle,
                "imu": self.imu,
                "gnss": self.gnss,
                "positioning": self.positioning,
                "weather": self.weather,
                "ground_truth": self._ground_truth_by_frame[max(self._ground_truth_by_frame)]
                if self._ground_truth_by_frame else None,
                "events": list(self.events),
                "rates": rates,
            }


class SessionRecorder:
    """Write JSONL measurements and JPEG frames without blocking sensor callbacks."""

    def __init__(self, root: Path, host: str, port: int, role: str) -> None:
        self._lock = threading.Lock()
        self.root = root
        self.host = host
        self.port = port
        self.role = role
        self.path: Path | None = None
        self._mode: str | None = None
        self._last_mode: str | None = None
        self._records: Queue[dict[str, Any]] | None = None
        self._images: Queue[tuple[CameraFrame, int]] | None = None
        self._visibility_masks: Queue[VisibilityMaskFrame] | None = None
        self._stop: threading.Event | None = None
        self._thread: threading.Thread | None = None
        self._enabled = False
        self._saved_frames = 0
        self._dropped_frames = 0
        self._dropped_records = 0
        self._error = ""

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def mode(self) -> str | None:
        with self._lock:
            return self._mode

    def start_session(self, mode: str, yolo_engine: Path | None = None,
                      planned_duration_seconds: int | None = None,
                      weather_workflow: dict[str, Any] | None = None,
                      weather: dict[str, Any] | None = None,
                      tracking_max_lost_frames: int = 10,
                       radar_track_correction_enabled: bool | None = None,
                       traffic_lights: dict[str, Any] | None = None) -> Path:
        if mode not in ("standard", "yolo", "tracking", "radar_fusion"):
            raise ValueError(f"Unknown recording mode: {mode}")
        if planned_duration_seconds is not None and planned_duration_seconds <= 0:
            raise ValueError("Planned recording duration must be positive")
        if (isinstance(tracking_max_lost_frames, bool)
                or not isinstance(tracking_max_lost_frames, int)
                or not 0 <= tracking_max_lost_frames <= 100):
            raise ValueError("Tracking retention must be between 0 and 100 frames")
        if (radar_track_correction_enabled is not None
                and not isinstance(radar_track_correction_enabled, bool)):
            raise ValueError("Radar track correction must be a boolean")
        if weather_workflow is not None:
            import re
            if (not re.fullmatch(r"[A-Za-z0-9_-]+", str(weather_workflow.get("workflow_id", "")))
                    or not re.fullmatch(r"[A-Za-z0-9_-]+", str(weather_workflow.get("weather_preset", "")))):
                raise ValueError("Invalid weather workflow path")
        with self._lock:
            if self._enabled:
                raise RuntimeError("A recording session is already active")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            parent = self.root
            if weather_workflow is not None:
                parent = (parent / f"workflow_{weather_workflow['workflow_id']}"
                          / f"weather_{weather_workflow['weather_preset']}")
                parent.mkdir(parents=True, exist_ok=True)
            path = parent / f"{timestamp}_{mode}"
            path.mkdir(parents=True, exist_ok=False)
            (path / "camera").mkdir()
            (path / "visibility_masks").mkdir()
            if mode in ("yolo", "tracking", "radar_fusion"):
                (path / "detections.jsonl").touch()
            if mode in ("tracking", "radar_fusion"):
                (path / "tracks.jsonl").touch()
                (path / "positions.jsonl").touch()
                (path / "ground_truth.jsonl").touch()
            if mode == "radar_fusion":
                (path / "vision_tracks.jsonl").touch()
                (path / "radar_fusion.jsonl").touch()
            metadata = {
                "created_at": datetime.now().astimezone().isoformat(),
                "carla_host": self.host,
                "carla_port": self.port,
                "vehicle_role": self.role,
                "recording_mode": mode,
                "weather": weather,
                "traffic_lights": traffic_lights,
                "camera_format": "JPEG",
                "measurements_format": "JSONL",
                "visibility_mask": {
                    "sensor": "sensor.camera.instance_segmentation",
                    "format": "PNG (raw semantic and instance channels)",
                    "index": "visibility_masks.jsonl",
                    "frame_alignment": "same CARLA frame as RGB camera",
                    "instance_id": "CARLA 0.9.16 actor ID, G low byte, B high byte",
                },
            }
            if planned_duration_seconds is not None:
                metadata["planned_duration_seconds"] = planned_duration_seconds
            if weather_workflow is not None:
                metadata["weather_workflow"] = weather_workflow
            if mode in ("yolo", "tracking", "radar_fusion"):
                metadata["yolo_engine"] = str(yolo_engine) if yolo_engine else None
            if mode in ("tracking", "radar_fusion"):
                metadata["tracking_max_lost_frames"] = tracking_max_lost_frames
                metadata["radar_track_correction_enabled"] = (
                    mode == "radar_fusion"
                    and radar_track_correction_enabled is True)
                metadata["position_estimation"] = {
                    "camera_model": "pinhole projection to local flat ground plane",
                    "radar_association": "project radar returns into camera image and match to detection boxes",
                    "coordinate_frame": "ego vehicle; X forward, Y right, Z up; positions in meters",
                    "camera_sensor_tick_seconds": 0.066666,
                    "radar_sensor_tick_seconds": 0.066666,
                    "max_sensor_frame_delta": 0,
                    "max_radar_range_age_seconds": 0.0,
                    "fusion_weights": {
                        "longitudinal_camera": 0.20,
                        "longitudinal_radar": 0.80,
                        "lateral_camera": 0.65,
                        "lateral_radar": 0.35,
                    },
                    "position_reference": "CARLA target bounding-box ground center",
                    "evaluation_truth": "target vehicle poses used only for scoring; ego pose and ground height provide coordinate reference",
                }
            (path / "session.json").write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            records: Queue[dict[str, Any]] = Queue(maxsize=4096)
            images: Queue[tuple[CameraFrame, int]] = Queue(maxsize=32)
            visibility_masks: Queue[VisibilityMaskFrame] = Queue(maxsize=32)
            stop = threading.Event()
            thread = threading.Thread(
                target=self._run, args=(path, records, images, visibility_masks, stop),
                name="session-recorder"
            )
            self.path = path
            self._mode = mode
            self._last_mode = mode
            self._records = records
            self._images = images
            self._visibility_masks = visibility_masks
            self._stop = stop
            self._thread = thread
            self._saved_frames = 0
            self._dropped_frames = 0
            self._dropped_records = 0
            self._error = ""
            self._enabled = True
            thread.start()
            return path

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                "enabled": self._enabled,
                "mode": self._mode or self._last_mode,
                "path": self.path,
                "saved_frames": self._saved_frames,
                "dropped_frames": self._dropped_frames,
                "dropped_records": self._dropped_records,
                "error": self._error,
            }

    def record(self, data: dict[str, Any]) -> None:
        self.record_many((data,))

    def record_many(self, records: tuple[dict[str, Any], ...]) -> None:
        with self._lock:
            if not self._enabled or self._records is None:
                return
            for data in records:
                if data.get("kind") == "detection" and self._mode not in ("yolo", "tracking", "radar_fusion"):
                    continue
                if data.get("kind") == "tracking" and self._mode not in ("tracking", "radar_fusion"):
                    continue
                if data.get("kind") == "vision_tracking" and self._mode != "radar_fusion":
                    continue
                if data.get("kind") in ("positioning", "ground_truth") and self._mode not in ("tracking", "radar_fusion"):
                    continue
                if data.get("kind") == "radar_fusion" and self._mode != "radar_fusion":
                    continue
                try:
                    self._records.put_nowait(data)
                except Full:
                    self._dropped_records += 1

    def record_camera(self, frame: CameraFrame, sensor_id: int) -> None:
        with self._lock:
            if not self._enabled or self._images is None:
                return
            try:
                self._images.put_nowait((frame, sensor_id))
            except Full:
                self._dropped_frames += 1

    def record_visibility_mask(self, frame: VisibilityMaskFrame) -> None:
        with self._lock:
            if not self._enabled or self._visibility_masks is None:
                return
            try:
                self._visibility_masks.put_nowait(frame)
            except Full:
                self._dropped_frames += 1

    def record_detection(self, detection: DetectionFrame) -> None:
        frame = detection.camera
        records = [{
            "schema_version": 1,
            "kind": "detection",
            "sensor_id": frame.sensor_id,
            "frame": frame.frame,
            "simulation_timestamp": frame.timestamp,
            "receive_timestamp": frame.received_at,
            "detection_timestamp": time.time(),
            "processing_ms": detection.processing_ms,
            "image": f"camera/{frame.sensor_id}_{frame.frame:08d}.jpg",
            "boxes": [
                {
                    "x": box.x,
                    "y": box.y,
                    "width": box.width,
                    "height": box.height,
                    "class_id": box.class_id,
                    "confidence": box.confidence,
                }
                for box in detection.boxes
            ],
        }]
        if detection.positioning is not None:
            records.append(detection.positioning)
            records.append({**detection.positioning, "kind": "radar_fusion"})
        records.append({
            "schema_version": 1,
            "kind": "tracking",
            "sensor_id": frame.sensor_id,
            "frame": frame.frame,
            "simulation_timestamp": frame.timestamp,
            "receive_timestamp": frame.received_at,
            "image": f"camera/{frame.sensor_id}_{frame.frame:08d}.jpg",
            "tracks": [
                {
                    "track_id": track.track_id,
                    "x": track.x,
                    "y": track.y,
                    "width": track.width,
                    "height": track.height,
                    "class_id": track.class_id,
                    "confidence": track.confidence,
                    "predicted": track.predicted,
                    "radar_corrected": track.radar_corrected,
                    "radar_correction_points": track.radar_correction_points,
                    "missed_frames": track.missed_frames,
                    "hits": track.hits,
                    "velocity_x_px_s": track.velocity_x_px_s,
                    "velocity_y_px_s": track.velocity_y_px_s,
                    "radar_range_m": track.radar_range_m,
                    "radar_range_rate_mps": track.radar_range_rate_mps,
                    "radar_age_s": track.radar_age_s,
                }
                for track in detection.tracks
            ],
        })
        if detection.vision_tracks or self.mode == "radar_fusion":
            records.append({
                "schema_version": 1,
                "kind": "vision_tracking",
                "sensor_id": frame.sensor_id,
                "frame": frame.frame,
                "simulation_timestamp": frame.timestamp,
                "receive_timestamp": frame.received_at,
                "image": f"camera/{frame.sensor_id}_{frame.frame:08d}.jpg",
                "tracks": [
                    {
                        "track_id": track.track_id,
                        "x": track.x,
                        "y": track.y,
                        "width": track.width,
                        "height": track.height,
                        "class_id": track.class_id,
                        "confidence": track.confidence,
                        "predicted": track.predicted,
                        "missed_frames": track.missed_frames,
                        "hits": track.hits,
                        "velocity_x_px_s": track.velocity_x_px_s,
                        "velocity_y_px_s": track.velocity_y_px_s,
                    }
                    for track in detection.vision_tracks
                ],
            })
        self.record_many(tuple(records))

    def _run(
        self,
        session_path: Path,
        records: Queue[dict[str, Any]],
        images: Queue[tuple[CameraFrame, int]],
        visibility_masks: Queue[VisibilityMaskFrame],
        stop: threading.Event,
    ) -> None:
        outputs: dict[str, Any] = {}
        try:
            with (session_path / "measurements.jsonl").open("a", encoding="utf-8") as output:
                while (not stop.is_set() or not records.empty() or not images.empty()
                       or not visibility_masks.empty()):
                    wrote = False
                    for _ in range(20):
                        try:
                            record = records.get_nowait()
                        except Empty:
                            break
                        destination = output
                        filenames = {
                            "detection": "detections.jsonl",
                            "tracking": "tracks.jsonl",
                            "vision_tracking": "vision_tracks.jsonl",
                            "positioning": "positions.jsonl",
                            "radar_fusion": "radar_fusion.jsonl",
                            "ground_truth": "ground_truth.jsonl",
                        }
                        filename = filenames.get(record.get("kind"))
                        if filename is not None:
                            if filename not in outputs:
                                outputs[filename] = (session_path / filename).open("a", encoding="utf-8")
                            destination = outputs[filename]
                        destination.write(json.dumps(record, ensure_ascii=False) + "\n")
                        wrote = True
                    try:
                        frame, sensor_id = images.get_nowait()
                    except Empty:
                        pass
                    else:
                        relative_path = f"camera/{sensor_id}_{frame.frame:08d}.jpg"
                        image = QImage(
                            frame.bgra,
                            frame.width,
                            frame.height,
                            frame.width * 4,
                            QImage.Format.Format_ARGB32,
                        )
                        if image.save(str(session_path / relative_path), "JPEG", 82):
                            output.write(
                                json.dumps(
                                    {
                                        "kind": "camera",
                                        "sensor_id": sensor_id,
                                        "frame": frame.frame,
                                        "simulation_timestamp": frame.timestamp,
                                        "receive_timestamp": frame.received_at,
                                        "width": frame.width,
                                        "height": frame.height,
                                        "fov": frame.fov,
                                        "sensor_transform": frame.sensor_transform,
                                        "image": relative_path,
                                    }
                                )
                                + "\n"
                            )
                            with self._lock:
                                self._saved_frames += 1
                        else:
                            with self._lock:
                                self._dropped_frames += 1
                                self._error = "JPEG 저장 실패"
                        wrote = True
                    try:
                        mask = visibility_masks.get_nowait()
                    except Empty:
                        pass
                    else:
                        relative_path = (f"visibility_masks/"
                                         f"{mask.sensor_id}_{mask.frame:08d}.png")
                        mask_image = QImage(
                            mask.bgra, mask.width, mask.height, mask.width * 4,
                            QImage.Format.Format_ARGB32,
                        )
                        if mask_image.save(str(session_path / relative_path), "PNG"):
                            if "visibility_masks.jsonl" not in outputs:
                                outputs["visibility_masks.jsonl"] = (
                                    session_path / "visibility_masks.jsonl"
                                ).open("a", encoding="utf-8")
                            outputs["visibility_masks.jsonl"].write(json.dumps({
                                "kind": "visibility_mask",
                                "sensor_id": mask.sensor_id,
                                "frame": mask.frame,
                                "simulation_timestamp": mask.timestamp,
                                "receive_timestamp": mask.received_at,
                                "width": mask.width,
                                "height": mask.height,
                                "image": relative_path,
                            }) + "\n")
                        else:
                            with self._lock:
                                self._dropped_frames += 1
                                self._error = "가시성 마스크 PNG 저장 실패"
                        wrote = True
                    if wrote:
                        output.flush()
                        for handle in outputs.values():
                            handle.flush()
                    else:
                        stop.wait(0.02)
        except Exception as exc:
            with self._lock:
                self._error = f"기록 오류: {exc}"
        finally:
            for handle in outputs.values():
                handle.close()

    def stop_session(self) -> Path | None:
        with self._lock:
            if not self._enabled:
                return self.path
            self._enabled = False
            stop = self._stop
            thread = self._thread
            path = self.path
            mode = self._mode
            if stop is not None:
                stop.set()
        if thread is not None:
            thread.join()
        with self._lock:
            if self._thread is thread:
                self._thread = None
                self._stop = None
                self._records = None
                self._images = None
                self._visibility_masks = None
                self._mode = None
        if mode in ("tracking", "radar_fusion") and path is not None:
            try:
                from monitor.qt.positioning import write_accuracy_summary
                write_accuracy_summary(path)
            except Exception as exc:
                with self._lock:
                    self._error = f"정확도 요약 생성 실패: {exc}"
        return path

    def stop(self) -> None:
        self.stop_session()


class CarlaCollector:
    def __init__(
        self,
        state: LiveState,
        recorder: SessionRecorder,
        host: str = "127.0.0.1",
        port: int = 2000,
        role: str = "hero",
    ) -> None:
        self.state = state
        self.recorder = recorder
        self.host = host
        self.port = port
        self.role = role
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="carla-collector")
        self._sensors: list[carla.Sensor] = []
        self._world_callback_id: int | None = None
        self.on_camera_frame: Callable[[], None] | None = None
        self._actors_lock = threading.Lock()
        self._vehicle_metadata: dict[int, dict[str, Any]] = {}
        self._ego_vehicle_id: int | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()

    def _spawn_sensor(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
        blueprint_name: str,
        placement: carla.Transform,
        attributes: dict[str, str],
        callback: Any,
    ) -> None:
        blueprint = world.get_blueprint_library().find(blueprint_name)
        for key, value in attributes.items():
            blueprint.set_attribute(key, value)
        sensor = world.spawn_actor(blueprint, placement, attach_to=vehicle)
        self._sensors.append(sensor)
        sensor.listen(lambda measurement, sensor_id=sensor.id: callback(measurement, sensor_id))

    def _attach_sensors(
        self, client: carla.Client, world: carla.World, vehicle: carla.Vehicle
    ) -> None:
        origin = carla.Transform()
        sensor_tick = "0.066666"  # Two world ticks at a 30 FPS world rate.
        sensor_specs = (
            (
                "sensor.camera.rgb",
                carla.Transform(carla.Location(x=1.5, z=2.4)),
                {"image_size_x": "640", "image_size_y": "480", "fov": "90", "sensor_tick": sensor_tick},
                self._on_camera,
            ),
            (
                "sensor.camera.instance_segmentation",
                carla.Transform(carla.Location(x=1.5, z=2.4)),
                {"image_size_x": "640", "image_size_y": "480", "fov": "90", "sensor_tick": sensor_tick},
                self._on_visibility_mask,
            ),
            (
                "sensor.other.radar",
                carla.Transform(carla.Location(x=2.2, z=0.7)),
                {"range": "100", "horizontal_fov": "35", "vertical_fov": "20",
                 "points_per_second": "1500", "sensor_tick": sensor_tick},
                self._on_radar,
            ),
            ("sensor.other.imu", origin, {"sensor_tick": sensor_tick}, self._on_imu),
            ("sensor.other.gnss", origin, {"sensor_tick": sensor_tick}, self._on_gnss),
        )
        commands = []
        for name, placement, attributes, _callback in sensor_specs:
            blueprint = world.get_blueprint_library().find(name)
            for key, value in attributes.items():
                blueprint.set_attribute(key, value)
            commands.append(carla.command.SpawnActor(blueprint, placement, vehicle.id))
        responses = client.apply_batch_sync(commands, False)
        spawned = [response.actor_id for response in responses if not response.error]
        if len(responses) != len(sensor_specs) or any(response.error for response in responses):
            for actor_id in spawned:
                actor = world.get_actor(actor_id)
                if actor is not None:
                    actor.destroy()
            errors = [response.error for response in responses if response.error]
            raise RuntimeError(f"Sensor batch spawn failed: {errors}")
        for actor_id, (_name, _placement, _attributes, callback) in zip(spawned, sensor_specs):
            sensor = world.get_actor(actor_id)
            if sensor is None:
                raise RuntimeError(f"Sensor {actor_id} missing after batch spawn")
            self._sensors.append(sensor)
            sensor.listen(lambda measurement, sensor_id=actor_id, handler=callback:
                          handler(measurement, sensor_id))
        self._spawn_sensor(world, vehicle, "sensor.other.collision", origin, {}, self._on_collision)
        self._spawn_sensor(world, vehicle, "sensor.other.lane_invasion", origin, {}, self._on_lane)

    def _destroy_sensors(self) -> None:
        for sensor in reversed(self._sensors):
            try:
                sensor.stop()
            except RuntimeError:
                pass
            try:
                sensor.destroy()
            except RuntimeError:
                pass
        self._sensors.clear()

    def _base(self, kind: str, measurement: Any, sensor_id: int) -> dict[str, Any]:
        return {
            "kind": kind,
            "sensor_id": sensor_id,
            "frame": int(measurement.frame),
            "simulation_timestamp": float(measurement.timestamp),
            "receive_timestamp": time.time(),
            "sensor_transform": transform(measurement.transform),
        }

    def _on_camera(self, image: carla.Image, sensor_id: int) -> None:
        frame = CameraFrame(
            frame=int(image.frame),
            timestamp=float(image.timestamp),
            received_at=time.time(),
            width=int(image.width),
            height=int(image.height),
            bgra=bytes(image.raw_data),
            sensor_id=sensor_id,
            fov=float(image.fov),
            sensor_transform=transform(image.transform),
        )
        self.state.update("camera", frame)
        self.recorder.record_camera(frame, sensor_id)
        if self.on_camera_frame is not None:
            self.on_camera_frame()

    def _on_visibility_mask(self, image: carla.Image, sensor_id: int) -> None:
        if not self.recorder.is_enabled():
            return
        self.recorder.record_visibility_mask(VisibilityMaskFrame(
            frame=int(image.frame),
            timestamp=float(image.timestamp),
            received_at=time.time(),
            width=int(image.width),
            height=int(image.height),
            bgra=bytes(image.raw_data),
            sensor_id=sensor_id,
        ))

    def _on_radar(self, measurement: carla.RadarMeasurement, sensor_id: int) -> None:
        data = self._base("radar", measurement, sensor_id)
        data["points"] = [
            {
                "distance_m": float(point.depth),
                "azimuth_rad": float(point.azimuth),
                "altitude_rad": float(point.altitude),
                "relative_velocity_mps": float(point.velocity),
            }
            for point in measurement
        ]
        self.state.update("radar", data)
        self.recorder.record(data)

    def _on_imu(self, measurement: carla.IMUMeasurement, sensor_id: int) -> None:
        data = self._base("imu", measurement, sensor_id)
        data.update(
            accelerometer_mps2=vector(measurement.accelerometer),
            gyroscope_rps=vector(measurement.gyroscope),
            compass_deg=math.degrees(float(measurement.compass)),
        )
        self.state.update("imu", data)
        self.recorder.record(data)

    def _on_gnss(self, measurement: carla.GNSSMeasurement, sensor_id: int) -> None:
        data = self._base("gnss", measurement, sensor_id)
        data.update(
            latitude=float(measurement.latitude),
            longitude=float(measurement.longitude),
            altitude_m=float(measurement.altitude),
        )
        self.state.update("gnss", data)
        self.recorder.record(data)

    def _on_collision(self, event: carla.CollisionEvent, sensor_id: int) -> None:
        data = self._base("collision", event, sensor_id)
        data.update(other_actor=event.other_actor.type_id, normal_impulse=vector(event.normal_impulse))
        self.state.update("collision", data)
        self.recorder.record(data)

    def _on_lane(self, event: carla.LaneInvasionEvent, sensor_id: int) -> None:
        data = self._base("lane_invasion", event, sensor_id)
        data["markings"] = [str(marking.type) for marking in event.crossed_lane_markings]
        self.state.update("lane_invasion", data)
        self.recorder.record(data)

    def _update_vehicle(self, actor: carla.ActorSnapshot, snapshot: carla.WorldSnapshot, vehicle_id: int) -> None:
        pose = actor.get_transform()
        velocity = actor.get_velocity()
        data = {
            "kind": "vehicle",
            "vehicle_id": vehicle_id,
            "frame": int(snapshot.frame),
            "simulation_timestamp": float(snapshot.timestamp.elapsed_seconds),
            "receive_timestamp": time.time(),
            "transform": transform(pose),
            "velocity_mps": vector(velocity),
            "speed_kmh": 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2),
        }
        self.state.update("vehicle", data)
        self.recorder.record(data)

    def _refresh_vehicle_metadata(self, world: carla.World) -> None:
        with self._actors_lock:
            previous = dict(self._vehicle_metadata)
        metadata: dict[int, dict[str, Any]] = {}
        for vehicle in world.get_actors().filter("vehicle.*"):
            actor_id = int(vehicle.id)
            type_id = str(vehicle.type_id)
            cached = previous.get(actor_id)
            if cached is not None and cached["type_id"] == type_id:
                metadata[actor_id] = cached
                continue
            try:
                bounds = vehicle.bounding_box
            except RuntimeError:
                continue  # The vehicle may have disappeared after the actor list was read.
            metadata[actor_id] = {
                "actor_id": actor_id,
                "type_id": type_id,
                "bounding_box": {
                    "location": vector(bounds.location),
                    "extent": vector(bounds.extent),
                },
            }
        with self._actors_lock:
            self._vehicle_metadata = metadata

    @staticmethod
    def _rotate_local(point: tuple[float, float, float], rotation: Any) -> tuple[float, float, float]:
        pitch, yaw, roll = (math.radians(float(getattr(rotation, axis)))
                            for axis in ("pitch", "yaw", "roll"))
        cp, sp, cy, sy, cr, sr = (
            math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw),
            math.cos(roll), math.sin(roll),
        )
        matrix = (
            (cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr),
            (cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr),
            (sp, -cp * sr, cp * cr),
        )
        return tuple(sum(matrix[row][col] * point[col] for col in range(3)) for row in range(3))

    def _on_world_snapshot(self, snapshot: carla.WorldSnapshot) -> None:
        with self._actors_lock:
            metadata = dict(self._vehicle_metadata)
            ego_vehicle_id = self._ego_vehicle_id
        if ego_vehicle_id is None:
            return
        ego_snapshot = snapshot.find(ego_vehicle_id)
        if ego_snapshot is None:
            return
        ego_transform = ego_snapshot.get_transform()
        vehicles = []
        for actor_id, vehicle_data in metadata.items():
            actor_snapshot = snapshot.find(actor_id)
            if actor_snapshot is None:
                continue
            vehicles.append({
                **vehicle_data,
                "transform": transform(actor_snapshot.get_transform()),
                "velocity_mps": vector(actor_snapshot.get_velocity()),
            })
        ego_meta = metadata.get(ego_vehicle_id)
        if ego_meta is None:
            return
        bounds = ego_meta["bounding_box"]
        center = bounds["location"]
        extent = bounds["extent"]
        ground_offset = self._rotate_local(
            (center["x"], center["y"], center["z"] - extent["z"]),
            ego_transform.rotation,
        )
        ego_location = ego_transform.location
        data = {
            "schema_version": 1,
            "kind": "ground_truth",
            "frame": int(snapshot.frame),
            "simulation_timestamp": float(snapshot.timestamp.elapsed_seconds),
            "ego_actor_id": int(ego_vehicle_id),
            "ego_transform": transform(ego_transform),
            "ego_ground_z": float(ego_location.z + ground_offset[2]),
            "vehicles": vehicles,
        }
        self.state.update("ground_truth", data)
        self.recorder.record(data)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                client = carla.Client(self.host, self.port)
                client.set_timeout(3.0)
                world = client.get_world()
                self.state.set_weather(weather_parameters(world.get_weather()))
                vehicles = world.get_actors().filter("vehicle.*")
                vehicle = next((v for v in vehicles if v.attributes.get("role_name") == self.role), None)
                if vehicle is None:
                    self.state.clear(f"'{self.role}' 차량을 기다리는 중")
                    self._stop.wait(2.0)
                    continue
                self._attach_sensors(client, world, vehicle)
                self._ego_vehicle_id = int(vehicle.id)
                self._refresh_vehicle_metadata(world)
                self._world_callback_id = world.on_tick(self._on_world_snapshot)
                self.state.set_status(f"연결됨 · {world.get_map().name.split('/')[-1]} · 차량 {vehicle.id}")
                last_frame = -1
                last_actor_refresh = time.monotonic()
                while not self._stop.is_set():
                    snapshot = world.get_snapshot()
                    if snapshot.frame != last_frame:
                        actor = snapshot.find(vehicle.id)
                        if actor is None:
                            raise RuntimeError("모니터링 차량이 사라졌습니다")
                        self._update_vehicle(actor, snapshot, vehicle.id)
                        last_frame = snapshot.frame
                    if time.monotonic() - last_actor_refresh >= 1.0:
                        self._refresh_vehicle_metadata(world)
                        self.state.set_weather(weather_parameters(world.get_weather()))
                        last_actor_refresh = time.monotonic()
                    self._stop.wait(0.03)
            except Exception as exc:
                self.state.clear(f"CARLA 대기/재연결: {exc}")
                self._stop.wait(2.0)
            finally:
                if self._world_callback_id is not None:
                    try:
                        world.remove_on_tick(self._world_callback_id)
                    except (RuntimeError, UnboundLocalError):
                        pass
                    self._world_callback_id = None
                self._ego_vehicle_id = None
                with self._actors_lock:
                    self._vehicle_metadata.clear()
                self._destroy_sensors()
