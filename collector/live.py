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
from typing import Any

import carla
from PySide6.QtGui import QImage


@dataclass(frozen=True)
class CameraFrame:
    frame: int
    timestamp: float
    received_at: float
    width: int
    height: int
    bgra: bytes


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
        self.status = "CARLA 연결 중"
        self.camera: CameraFrame | None = None
        self.radar: dict[str, Any] | None = None
        self.vehicle: dict[str, Any] | None = None
        self.imu: dict[str, Any] | None = None
        self.gnss: dict[str, Any] | None = None
        self.events: deque[dict[str, Any]] = deque(maxlen=12)
        self.arrivals: dict[str, deque[float]] = {
            name: deque(maxlen=120)
            for name in ("camera", "radar", "vehicle", "imu", "gnss")
        }

    def set_status(self, text: str) -> None:
        with self._lock:
            self.status = text

    def clear(self, status: str) -> None:
        with self._lock:
            self.status = status
            self.camera = None
            self.radar = None
            self.vehicle = None
            self.imu = None
            self.gnss = None
            self.events.clear()
            for arrivals in self.arrivals.values():
                arrivals.clear()

    def update(self, name: str, value: Any) -> None:
        with self._lock:
            if name in ("collision", "lane_invasion"):
                self.events.appendleft(value)
            else:
                setattr(self, name, value)
                self.arrivals[name].append(time.monotonic())

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
                "radar": self.radar,
                "vehicle": self.vehicle,
                "imu": self.imu,
                "gnss": self.gnss,
                "events": list(self.events),
                "rates": rates,
            }


class SessionRecorder:
    """Write JSONL measurements and JPEG frames without blocking sensor callbacks."""

    def __init__(self, root: Path, host: str, port: int, role: str) -> None:
        self.path = root / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "camera").mkdir()
        (self.path / "session.json").write_text(
            json.dumps(
                {
                    "created_at": datetime.now().astimezone().isoformat(),
                    "carla_host": host,
                    "carla_port": port,
                    "vehicle_role": role,
                    "camera_format": "JPEG",
                    "measurements_format": "JSONL",
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        self._records: Queue[dict[str, Any]] = Queue(maxsize=4096)
        self._images: Queue[tuple[CameraFrame, int]] = Queue(maxsize=32)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._enabled = True
        self._saved_frames = 0
        self._dropped_frames = 0
        self._dropped_records = 0
        self._error = ""
        self._thread = threading.Thread(target=self._run, name="session-recorder")
        self._thread.start()

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def toggle(self) -> bool:
        with self._lock:
            self._enabled = not self._enabled
            return self._enabled

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                "enabled": self._enabled,
                "saved_frames": self._saved_frames,
                "dropped_frames": self._dropped_frames,
                "dropped_records": self._dropped_records,
                "error": self._error,
            }

    def record(self, data: dict[str, Any]) -> None:
        if not self.is_enabled():
            return
        try:
            self._records.put_nowait(data)
        except Full:
            with self._lock:
                self._dropped_records += 1

    def record_camera(self, frame: CameraFrame, sensor_id: int) -> None:
        if not self.is_enabled():
            return
        try:
            self._images.put_nowait((frame, sensor_id))
        except Full:
            with self._lock:
                self._dropped_frames += 1

    def _run(self) -> None:
        try:
            with (self.path / "measurements.jsonl").open("a", encoding="utf-8") as output:
                while not self._stop.is_set() or not self._records.empty() or not self._images.empty():
                    wrote = False
                    for _ in range(20):
                        try:
                            record = self._records.get_nowait()
                        except Empty:
                            break
                        output.write(json.dumps(record, ensure_ascii=False) + "\n")
                        wrote = True
                    try:
                        frame, sensor_id = self._images.get_nowait()
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
                        if image.save(str(self.path / relative_path), "JPEG", 82):
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
                    if wrote:
                        output.flush()
                    else:
                        self._stop.wait(0.02)
        except Exception as exc:
            with self._lock:
                self._error = f"기록 오류: {exc}"

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()


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

    def _attach_sensors(self, world: carla.World, vehicle: carla.Vehicle) -> None:
        origin = carla.Transform()
        self._spawn_sensor(
            world,
            vehicle,
            "sensor.camera.rgb",
            carla.Transform(carla.Location(x=1.5, z=2.4)),
            {"image_size_x": "640", "image_size_y": "480", "fov": "90", "sensor_tick": "0.05"},
            self._on_camera,
        )
        self._spawn_sensor(
            world,
            vehicle,
            "sensor.other.radar",
            carla.Transform(carla.Location(x=2.2, z=0.7)),
            {
                "range": "100",
                "horizontal_fov": "35",
                "vertical_fov": "20",
                "points_per_second": "1500",
                "sensor_tick": "0.05",
            },
            self._on_radar,
        )
        self._spawn_sensor(world, vehicle, "sensor.other.imu", origin, {"sensor_tick": "0.05"}, self._on_imu)
        self._spawn_sensor(world, vehicle, "sensor.other.gnss", origin, {"sensor_tick": "0.05"}, self._on_gnss)
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
        )
        self.state.update("camera", frame)
        self.recorder.record_camera(frame, sensor_id)

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

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                client = carla.Client(self.host, self.port)
                client.set_timeout(3.0)
                world = client.get_world()
                vehicles = world.get_actors().filter("vehicle.*")
                vehicle = next((v for v in vehicles if v.attributes.get("role_name") == self.role), None)
                if vehicle is None:
                    self.state.clear(f"'{self.role}' 차량을 기다리는 중")
                    self._stop.wait(2.0)
                    continue
                self._attach_sensors(world, vehicle)
                self.state.set_status(f"연결됨 · {world.get_map().name.split('/')[-1]} · 차량 {vehicle.id}")
                last_frame = -1
                while not self._stop.is_set():
                    snapshot = world.get_snapshot()
                    if snapshot.frame != last_frame:
                        actor = snapshot.find(vehicle.id)
                        if actor is None:
                            raise RuntimeError("모니터링 차량이 사라졌습니다")
                        self._update_vehicle(actor, snapshot, vehicle.id)
                        last_frame = snapshot.frame
                    self._stop.wait(0.03)
            except Exception as exc:
                self.state.clear(f"CARLA 대기/재연결: {exc}")
                self._stop.wait(2.0)
            finally:
                self._destroy_sensors()
