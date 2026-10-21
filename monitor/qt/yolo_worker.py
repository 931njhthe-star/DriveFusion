"""Run C++ YOLO11 inference without blocking CARLA callbacks or the Qt UI."""

from __future__ import annotations

from pathlib import Path
import threading
import time

from collector.live import DetectionFrame, LiveState, SessionRecorder
from core.yolo11_detector import Detector
from monitor.qt.positioning import PositioningEngine, match_radar_to_boxes
from monitor.qt.tracker import MultiObjectTracker


class YoloWorker:
    def __init__(
        self,
        state: LiveState,
        recorder: SessionRecorder,
        library_path: Path,
        engine_path: Path,
        dll_directories: tuple[Path, ...] = (),
    ) -> None:
        self.state = state
        self.recorder = recorder
        self.library_path = library_path
        self.engine_path = engine_path
        self.dll_directories = dll_directories
        self.status = "YOLO11n 모델 로딩 중"
        self._stop = threading.Event()
        self._enabled = threading.Event()
        self._tracking_enabled = threading.Event()
        self._radar_fusion_enabled = threading.Event()
        self._radar_fusion_enabled.set()
        self._control_lock = threading.Lock()
        self._tracker = MultiObjectTracker(max_lost_frames=10)
        self._vision_tracker = MultiObjectTracker(max_lost_frames=10)
        self._positioning = PositioningEngine()
        self._thread = threading.Thread(target=self._run, name="yolo11-worker")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._enabled.clear()
        self._thread.join()

    @property
    def enabled(self) -> bool:
        return self._enabled.is_set()

    def set_enabled(self, enabled: bool) -> None:
        with self._control_lock:
            if enabled and not self._stop.is_set():
                self._enabled.set()
                self.status = "YOLO11n 탐지 중"
            else:
                self._enabled.clear()
                self._tracking_enabled.clear()
                self._tracker.reset()
                self._vision_tracker.reset()
                self._positioning.reset()
                self.state.clear_detections()
                if not self._stop.is_set() and not self.status.startswith("YOLO11n 오류"):
                    self.status = "YOLO11n 탐지 꺼짐"

    @property
    def tracking_enabled(self) -> bool:
        return self._tracking_enabled.is_set()

    def set_tracking_enabled(self, enabled: bool) -> None:
        with self._control_lock:
            if enabled and self._enabled.is_set() and not self._stop.is_set():
                self._tracker.reset()
                self._vision_tracker.reset()
                self._positioning.reset()
                self._tracking_enabled.set()
                self.state.clear_detections()
            else:
                self._tracking_enabled.clear()
                self._tracker.reset()
                self._vision_tracker.reset()
                self._positioning.reset()
                self.state.clear_detections()

    @property
    def radar_fusion_enabled(self) -> bool:
        return self._radar_fusion_enabled.is_set()

    def set_radar_fusion_enabled(self, enabled: bool) -> None:
        with self._control_lock:
            if enabled and not self._stop.is_set():
                self._radar_fusion_enabled.set()
            else:
                self._radar_fusion_enabled.clear()
            self._tracker.reset()
            self._positioning.reset()
            self.state.clear_detections()

    def _run(self) -> None:
        try:
            with Detector(self.library_path, self.engine_path,
                          dll_directories=self.dll_directories) as detector:
                with self._control_lock:
                    if self._enabled.is_set():
                        self.status = "YOLO11n 탐지 중"
                    else:
                        self.status = "YOLO11n 준비됨 · 탐지 꺼짐"
                last_received = None
                while not self._stop.is_set():
                    if not self._enabled.wait(0.1):
                        continue
                    frame = self.state.get_camera()
                    if frame is None or frame.received_at == last_received:
                        self._stop.wait(0.01)
                        continue
                    last_received = frame.received_at
                    started = time.perf_counter()
                    boxes = detector.detect(frame.bgra, frame.width, frame.height)
                    if self._tracking_enabled.is_set() and self._radar_fusion_enabled.is_set():
                        radar, radar_delta, truth, truth_delta = (
                            self.state.wait_for_position_context(frame.frame)
                        )
                    else:
                        radar, radar_delta, truth, truth_delta = self.state.position_context(frame.frame)
                    with self._control_lock:
                        if self._enabled.is_set():
                            radar_matches = (
                                match_radar_to_boxes(frame, boxes, radar, truth)
                                if self._radar_fusion_enabled.is_set()
                                else None
                            )
                            vision_tracks = ()
                            tracks = ()
                            if self._tracking_enabled.is_set():
                                vision_tracks = self._vision_tracker.update(boxes, frame.received_at)
                                tracks = (
                                    self._tracker.update(boxes, frame.received_at, radar_matches)
                                    if self._radar_fusion_enabled.is_set()
                                    else vision_tracks
                                )
                            positioning = (
                                self._positioning.estimate(
                                    frame, boxes, tracks,
                                    radar if self._radar_fusion_enabled.is_set() else None,
                                    truth,
                                    radar_delta if self._radar_fusion_enabled.is_set() else None,
                                    truth_delta,
                                )
                                if self._tracking_enabled.is_set()
                                else None
                            )
                            processing_ms = (time.perf_counter() - started) * 1000.0
                            result = DetectionFrame(
                                frame, tuple(boxes), processing_ms, tracks, positioning,
                                vision_tracks=vision_tracks,
                            )
                            self.state.update("detections", result)
                            if positioning is not None:
                                self.state.update("positioning", positioning)
                            self.recorder.record_detection(result)
        except Exception as exc:
            with self._control_lock:
                self._enabled.clear()
                self._tracking_enabled.clear()
                self._tracker.reset()
                self._vision_tracker.reset()
                self._positioning.reset()
                self.state.clear_detections()
                self.status = f"YOLO11n 오류: {exc}"
