"""Create review images by drawing saved YOLO boxes onto source camera frames."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import sys
import subprocess
import statistics
from typing import Any

from PySide6.QtCore import QBuffer, QIODevice, QThread, Signal, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import QApplication

from core.yolo11_detector import Box
from monitor.qt.tracker import MultiObjectTracker


@dataclass(frozen=True)
class OverlayGenerationSummary:
    input_records: int
    generated_images: int
    missing_images: int
    invalid_records: int
    generated_tracking_images: int
    output_dir: Path
    generated_radar_fusion_images: int = 0
    comparison_video: Path | None = None
    video_frames: int = 0


def _record_image_path(session_dir: Path, record: dict[str, Any]) -> Path | None:
    relative_image = record.get("image")
    if not relative_image:
        sensor_id = record.get("sensor_id")
        frame = record.get("frame")
        if sensor_id is None or frame is None:
            return None
        relative_image = f"camera/{int(sensor_id)}_{int(frame):08d}.jpg"

    relative_path = Path(str(relative_image))
    if relative_path.is_absolute() or ".." in relative_path.parts:
        return None
    session_root = session_dir.resolve()
    image_path = (session_root / relative_path).resolve()
    try:
        image_path.relative_to(session_root)
    except ValueError:
        return None
    return image_path


def _draw_boxes(image: QImage, boxes: Any) -> None:
    if not isinstance(boxes, list):
        return
    painter = QPainter(image)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    line_width = max(2, image.width() // 320)
    for box in boxes:
        if not isinstance(box, dict):
            continue
        try:
            x = int(round(float(box["x"])))
            y = int(round(float(box["y"])))
            width = int(round(float(box["width"])))
            height = int(round(float(box["height"])))
            class_id = int(box["class_id"])
            confidence = float(box["confidence"])
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        if width <= 0 or height <= 0:
            continue
        predicted = bool(box.get("predicted", False))
        pen = QPen(QColor("#f5d76e") if predicted else QColor("#4ee38b"))
        pen.setWidth(line_width)
        if predicted:
            pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.drawRect(x, y, width, height)
        track_id = box.get("track_id")
        label = f"ID {track_id}  C{class_id} {confidence:.2f}" if track_id is not None else f"C{class_id} {confidence:.2f}"
        if predicted:
            label += f"  · 누락 {box.get('missed_frames', 0)}"
        painter.drawText(x, max(14, y - 5), label)
    painter.end()


def _read_frame_records(path: Path) -> dict[tuple[int | None, int], dict[str, Any]]:
    records: dict[tuple[int | None, int], dict[str, Any]] = {}
    if not path.is_file():
        return records
    with path.open("r", encoding="utf-8") as source:
        for line in source:
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                if not isinstance(record, dict) or record.get("frame") is None:
                    continue
                sensor_id = record.get("sensor_id")
                key = (int(sensor_id) if sensor_id is not None else None, int(record["frame"]))
                records[key] = record
            except (json.JSONDecodeError, TypeError, ValueError, OverflowError):
                continue
    return records


def _for_frame(
    records: dict[tuple[int | None, int], dict[str, Any]], sensor_id: int, frame: int
) -> dict[str, Any]:
    return records.get((sensor_id, frame), records.get((None, frame), {}))


def _draw_radar_fusion(image: QImage, tracks: Any, positioning: dict[str, Any]) -> None:
    if not isinstance(tracks, list):
        return
    estimates = {
        int(item["track_id"]): item
        for item in positioning.get("objects", [])
        if isinstance(item, dict) and item.get("track_id") is not None
    }
    painter = QPainter(image)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    pen_width = max(2, image.width() // 320)
    for track in tracks:
        if not isinstance(track, dict):
            continue
        try:
            track_id = int(track["track_id"])
            x, y = int(track["x"]), int(track["y"])
            width, height = int(track["width"]), int(track["height"])
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        if width <= 0 or height <= 0:
            continue
        estimate = estimates.get(track_id, {})
        radar_used = bool(estimate.get("radar_used", track.get("radar_range_m") is not None))
        pen = QPen(QColor("#51c9ff") if radar_used else QColor("#f5d76e"))
        pen.setWidth(pen_width)
        if track.get("predicted"):
            pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.drawRect(x, y, width, height)
        label = f"ID {track_id}"
        radar_range = estimate.get("radar_range_m", track.get("radar_range_m"))
        if radar_range is not None:
            label += f"  R {float(radar_range):.1f}m"
        fused = estimate.get("fused_xy_ego_m")
        if isinstance(fused, dict):
            label += f"  X {float(fused.get('forward_m', 0.0)):.1f} Y {float(fused.get('right_m', 0.0)):+.1f}m"
        elif not radar_used:
            label += "  radar wait"
        if track.get("predicted"):
            label += f"  lost {track.get('missed_frames', 0)}"
        painter.drawText(x, max(14, y - 4), label)
    painter.end()


def _render_saved_view(
    source_path: Path,
    view: str,
    sensor_id: int,
    frame_number: int,
    detections: dict[tuple[int | None, int], dict[str, Any]],
    tracks: dict[tuple[int | None, int], dict[str, Any]],
    positioning: dict[tuple[int | None, int], dict[str, Any]],
) -> QImage:
    image = QImage(str(source_path))
    if image.isNull():
        return image
    if view == "yolo":
        _draw_boxes(image, _for_frame(detections, sensor_id, frame_number).get("boxes", []))
    elif view == "tracking":
        _draw_boxes(image, _for_frame(tracks, sensor_id, frame_number).get("tracks", []))
    elif view == "radar_fusion":
        _draw_radar_fusion(
            image,
            _for_frame(tracks, sensor_id, frame_number).get("tracks", []),
            _for_frame(positioning, sensor_id, frame_number),
        )
    return image


def _labeled_panel(image: QImage, title: str, width: int, height: int) -> QImage:
    panel = QImage(width, height, QImage.Format.Format_RGB32)
    panel.fill(QColor("#07101c"))
    fitted = image.scaled(
        width, height, Qt.AspectRatioMode.KeepAspectRatio,
        Qt.TransformationMode.SmoothTransformation,
    )
    painter = QPainter(panel)
    painter.drawImage((width - fitted.width()) // 2, (height - fitted.height()) // 2, fitted)
    painter.fillRect(0, 0, width, 32, QColor(0, 0, 0, 185))
    painter.setPen(QColor("#ffffff"))
    painter.drawText(12, 22, title)
    painter.end()
    return panel


def _validate_comparison_session(session_dir: Path) -> None:
    if not session_dir.is_dir():
        raise NotADirectoryError(f"Recording session folder not found: {session_dir}")
    required_logs = (
        "measurements.jsonl",
        "detections.jsonl",
        "tracks.jsonl",
        "positions.jsonl",
    )
    missing_logs = [name for name in required_logs if not (session_dir / name).is_file()]
    if missing_logs:
        raise ValueError(
            "4분할 비교 영상에는 YOLO+트래킹 기록 세션이 필요합니다. "
            f"누락 파일: {', '.join(missing_logs)}"
        )
    metadata_path = session_dir / "session.json"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        metadata = {}
    if (
        not isinstance(metadata, dict)
        or metadata.get("recording_mode") != "radar_fusion"
    ) and not (session_dir / "radar_fusion.jsonl").is_file():
        raise ValueError("원본·YOLO·일반 트래킹·레이더 융합 비교를 위해 *_radar_fusion 세션을 선택하세요.")


def _vision_tracking_records(
    session_dir: Path,
    detections: dict[tuple[int | None, int], dict[str, Any]],
    tracks: dict[tuple[int | None, int], dict[str, Any]],
) -> dict[tuple[int | None, int], dict[str, Any]]:
    vision_path = session_dir / "vision_tracks.jsonl"
    if vision_path.is_file():
        return _read_frame_records(vision_path)

    metadata_path = session_dir / "session.json"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        metadata = {}
    is_radar_fusion = (
        isinstance(metadata, dict) and metadata.get("recording_mode") == "radar_fusion"
    ) or (session_dir / "radar_fusion.jsonl").is_file()
    if not is_radar_fusion:
        return tracks

    # Older radar-fusion sessions only saved the fused tracks. Re-run the
    # same visual tracker over their saved detections for a fair baseline.
    ordered = sorted(
        detections.items(),
        key=lambda item: (
            float(item[1].get("receive_timestamp", item[1].get("simulation_timestamp", item[0][1]))),
            -1 if item[0][0] is None else item[0][0],
            item[0][1],
        ),
    )
    tracker = MultiObjectTracker(max_lost_frames=10)
    current_sensor_id: int | None = None
    have_sensor_id = False
    reconstructed: dict[tuple[int | None, int], dict[str, Any]] = {}
    for key, record in ordered:
        sensor_id, frame_number = key
        if have_sensor_id and sensor_id != current_sensor_id:
            tracker.reset()
        current_sensor_id = sensor_id
        have_sensor_id = True
        boxes = []
        for box in record.get("boxes", []):
            if not isinstance(box, dict):
                continue
            try:
                boxes.append(Box(
                    int(box["x"]), int(box["y"]), int(box["width"]), int(box["height"]),
                    int(box["class_id"]), float(box["confidence"]),
                ))
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
        timestamp = record.get("receive_timestamp", record.get("simulation_timestamp", frame_number))
        try:
            received_at = float(timestamp)
        except (TypeError, ValueError, OverflowError):
            received_at = float(frame_number)
        tracked = tracker.update(boxes, received_at)
        reconstructed[key] = {
            **record,
            "tracks": [
                {
                    "track_id": item.track_id,
                    "x": item.x,
                    "y": item.y,
                    "width": item.width,
                    "height": item.height,
                    "class_id": item.class_id,
                    "confidence": item.confidence,
                    "predicted": item.predicted,
                    "missed_frames": item.missed_frames,
                }
                for item in tracked
            ],
        }
    return reconstructed


def generate_comparison_video(
    session_dir: Path,
    output_path: Path | None = None,
) -> tuple[Path, int]:
    """Render one four-panel MP4 containing every saved camera view."""
    session_dir = session_dir.resolve()
    _validate_comparison_session(session_dir)
    measurements_path = session_dir / "measurements.jsonl"
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("비교 영상 생성에는 ffmpeg가 필요합니다. ffmpeg를 설치하고 PATH에 추가한 뒤 다시 실행하세요.")

    camera_frames: list[dict[str, Any]] = []
    with measurements_path.open("r", encoding="utf-8") as source:
        for line in source:
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(record, dict) and record.get("kind") == "camera":
                camera_frames.append(record)
    camera_frames.sort(key=lambda row: (float(row.get("simulation_timestamp", 0.0)), int(row.get("frame", 0))))
    if not camera_frames:
        raise ValueError("비교 영상에 사용할 카메라 프레임이 없습니다.")

    detections = _read_frame_records(session_dir / "detections.jsonl")
    tracks = _read_frame_records(session_dir / "tracks.jsonl")
    vision_tracks = _vision_tracking_records(session_dir, detections, tracks)
    positioning = _read_frame_records(session_dir / "positions.jsonl")
    if output_path is None:
        output_dir = session_dir / "comparison_videos"
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "all_views.mp4"
    else:
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = output_path.with_name(f"{output_path.stem}.tmp{output_path.suffix}")
    temporary_output.unlink(missing_ok=True)

    timestamps = [float(row["simulation_timestamp"]) for row in camera_frames
                  if row.get("simulation_timestamp") is not None]
    intervals = [later - earlier for earlier, later in zip(timestamps, timestamps[1:])
                 if later > earlier]
    fps = max(1, min(60, round(1.0 / statistics.median(intervals)))) if intervals else 20
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-f", "image2pipe", "-framerate", str(fps), "-vcodec", "mjpeg", "-i", "pipe:0",
        "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary_output),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    assert process.stdin is not None
    written_frames = 0
    cell_size: tuple[int, int] | None = None
    views = (
        ("original", "Original", tracks),
        ("yolo", "YOLO detection", tracks),
        ("tracking", "Vision tracking", vision_tracks),
        ("radar_fusion", "Radar fusion tracking", tracks),
    )
    try:
        for record in camera_frames:
            try:
                sensor_id, frame_number = int(record["sensor_id"]), int(record["frame"])
            except (KeyError, TypeError, ValueError):
                continue
            source_path = _record_image_path(session_dir, record)
            if source_path is None or not source_path.is_file():
                continue
            panels = []
            for view, title, view_tracks in views:
                image = _render_saved_view(
                    source_path, view, sensor_id, frame_number,
                    detections, view_tracks, positioning,
                )
                if image.isNull():
                    panels = []
                    break
                if cell_size is None:
                    cell_width = image.width() + image.width() % 2
                    cell_height = image.height() + image.height() % 2
                    cell_size = (cell_width, cell_height)
                panels.append(_labeled_panel(image, title, *cell_size))
            if len(panels) != len(views) or cell_size is None:
                continue
            cell_width, cell_height = cell_size
            combined = QImage(cell_width * 2, cell_height * 2, QImage.Format.Format_RGB32)
            combined.fill(QColor("#07101c"))
            painter = QPainter(combined)
            painter.drawImage(0, 0, panels[0])
            painter.drawImage(cell_width, 0, panels[1])
            painter.drawImage(0, cell_height, panels[2])
            painter.drawImage(cell_width, cell_height, panels[3])
            painter.end()
            buffer = QBuffer()
            buffer.open(QIODevice.OpenModeFlag.WriteOnly)
            if not combined.save(buffer, "JPEG", 88):
                buffer.close()
                raise RuntimeError("비교 영상 프레임 JPEG 인코딩에 실패했습니다.")
            encoded = bytes(buffer.data())
            buffer.close()
            process.stdin.write(encoded)
            written_frames += 1
        process.stdin.close()
        error_text = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
        if process.stderr is not None:
            process.stderr.close()
        return_code = process.wait()
    except Exception:
        if process.poll() is None:
            process.kill()
            process.wait()
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        if process.stderr is not None:
            process.stderr.close()
        temporary_output.unlink(missing_ok=True)
        raise
    if return_code != 0:
        temporary_output.unlink(missing_ok=True)
        raise RuntimeError(f"ffmpeg 영상 인코딩 실패: {error_text.strip() or return_code}")
    if written_frames == 0:
        temporary_output.unlink(missing_ok=True)
        raise ValueError("비교 영상에 쓸 수 있는 원본 카메라 프레임이 없습니다.")
    temporary_output.replace(output_path)
    return output_path, written_frames


def generate_yolo_overlays(
    session_dir: Path,
    output_dir: Path | None = None,
) -> OverlayGenerationSummary:
    session_dir = session_dir.resolve()
    _validate_comparison_session(session_dir)
    detections_path = session_dir / "detections.jsonl"

    output_dir = (output_dir or session_dir / "annotated").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    tracking_dir = session_dir / "tracked"
    fused_tracking_path = session_dir / "tracks.jsonl"
    detection_records = _read_frame_records(detections_path)
    fused_tracking_records = _read_frame_records(fused_tracking_path)
    vision_tracking_records = _vision_tracking_records(
        session_dir, detection_records, fused_tracking_records
    )
    radar_fusion_dir = session_dir / "radar_fusion"
    positioning_records = _read_frame_records(session_dir / "positions.jsonl")
    input_records = generated_images = missing_images = invalid_records = 0
    generated_tracking_images = 0
    generated_radar_fusion_images = 0

    if detections_path.is_file():
        with detections_path.open("r", encoding="utf-8") as source:
            for line_number, line in enumerate(source, start=1):
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                    if not isinstance(record, dict):
                        raise ValueError("record must be a JSON object")
                    input_records += 1
                    image_path = _record_image_path(session_dir, record)
                    if image_path is None or not image_path.is_file():
                        missing_images += 1
                        continue
                    image = QImage(str(image_path))
                    if image.isNull():
                        missing_images += 1
                        continue
                    _draw_boxes(image, record.get("boxes", []))
                    output_path = output_dir / image_path.name
                    if not image.save(str(output_path), "JPEG", 92):
                        raise OSError(f"Could not save generated image: {output_path}")
                    generated_images += 1
                except (json.JSONDecodeError, ValueError, TypeError, OverflowError):
                    invalid_records += 1
                except Exception as exc:
                    raise RuntimeError(f"detections.jsonl line {line_number}: {exc}") from exc

    if vision_tracking_records:
        tracking_dir.mkdir(parents=True, exist_ok=True)
        radar_fusion_dir.mkdir(parents=True, exist_ok=True)
        for key, record in vision_tracking_records.items():
            input_records += 1
            image_path = _record_image_path(session_dir, record)
            if image_path is None or not image_path.is_file():
                missing_images += 1
                continue
            image = QImage(str(image_path))
            if image.isNull():
                missing_images += 1
                continue
            _draw_boxes(image, record.get("tracks", []))
            if not image.save(str(tracking_dir / image_path.name), "JPEG", 92):
                raise OSError(f"Could not save tracked image: {tracking_dir / image_path.name}")
            generated_tracking_images += 1
            sensor_id, frame_number = key
            radar_image = QImage(str(image_path))
            _draw_radar_fusion(
                radar_image,
                _for_frame(
                    fused_tracking_records,
                    -1 if sensor_id is None else sensor_id,
                    frame_number,
                ).get("tracks", []),
                _for_frame(
                    positioning_records,
                    -1 if sensor_id is None else sensor_id,
                    frame_number,
                ),
            )
            if not radar_image.save(str(radar_fusion_dir / image_path.name), "JPEG", 92):
                raise OSError(f"Could not save radar fusion image: {radar_fusion_dir / image_path.name}")
            generated_radar_fusion_images += 1

    comparison_video, video_frames = generate_comparison_video(
        session_dir
    )
    return OverlayGenerationSummary(
        input_records=input_records,
        generated_images=generated_images,
        missing_images=missing_images,
        invalid_records=invalid_records,
        generated_tracking_images=generated_tracking_images,
        output_dir=output_dir,
        generated_radar_fusion_images=generated_radar_fusion_images,
        comparison_video=comparison_video,
        video_frames=video_frames,
    )


class OverlayGeneratorThread(QThread):
    completed = Signal(object)
    failed = Signal(str)

    def __init__(
        self, session_dir: Path, parent: Any = None,
    ) -> None:
        super().__init__(parent)
        self.session_dir = session_dir

    def run(self) -> None:
        try:
            self.completed.emit(generate_yolo_overlays(self.session_dir))
        except Exception as exc:
            self.failed.emit(str(exc))


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate annotated images from a YOLO session")
    parser.add_argument("session_dir", type=Path, help="Folder containing detections.jsonl and camera/")
    parser.add_argument("--output", type=Path, help="Output folder (default: session_dir/annotated)")
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    try:
        summary = generate_yolo_overlays(args.session_dir, args.output)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    print(f"Generated: {summary.generated_images}")
    print(f"Missing source images: {summary.missing_images}")
    print(f"Invalid records: {summary.invalid_records}")
    print(f"Output: {summary.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
