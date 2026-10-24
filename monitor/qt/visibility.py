"""Read CARLA instance masks and score geometrically visible vehicle detections."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

from PySide6.QtGui import QImage

from monitor.qt.positioning import _box_iou


# CARLA 0.9.16 stores the actor ID in the instance mask's G/B channels.
# The sensor's raw BGRA bytes and a lossless ARGB32 PNG have B, G, R, A order.
VEHICLE_TAGS = (14, 15, 16, 17, 18, 19)
MIN_VISIBLE_PIXELS = 50
MIN_VISIBLE_BOX_SIDE_PX = 8
DETECTION_IOU = 0.5


def visible_vehicle_boxes(
    mask_path: Path, projected: dict[int, dict[str, float]],
    width: int, height: int,
) -> dict[int, dict[str, Any]]:
    """Return visible pixel counts and boxes for projected CARLA actor IDs."""
    image = QImage(str(mask_path))
    if image.isNull() or image.width() != width or image.height() != height:
        raise ValueError(f"가시성 마스크를 읽을 수 없거나 RGB 크기와 다릅니다: {mask_path}")
    image = image.convertToFormat(QImage.Format.Format_ARGB32)
    data = bytes(image.bits())
    stride = image.bytesPerLine()
    result: dict[int, dict[str, Any]] = {}
    for actor_id, box in projected.items():
        if not 0 < actor_id <= 0xffff:
            continue  # The two mask ID channels cannot represent this actor ID.
        left = max(0, math.floor(box["x"]))
        top = max(0, math.floor(box["y"]))
        right = min(width, math.ceil(box["x"] + box["width"]))
        bottom = min(height, math.ceil(box["y"] + box["height"]))
        if right <= left or bottom <= top:
            continue
        patterns = tuple(bytes((actor_id >> 8, actor_id & 255, tag, 255))
                         for tag in VEHICLE_TAGS)
        pixels = 0
        min_x, min_y, max_x, max_y = right, bottom, left - 1, top - 1
        for y in range(top, bottom):
            row = data[y * stride + 4 * left:y * stride + 4 * right]
            for pattern in patterns:
                count = row.count(pattern)
                if not count:
                    continue
                pixels += count
                first = row.find(pattern) // 4 + left
                last = row.rfind(pattern) // 4 + left
                min_x, min_y = min(min_x, first), min(min_y, y)
                max_x, max_y = max(max_x, last), max(max_y, y)
        if pixels:
            visible_box = {"x": min_x, "y": min_y,
                           "width": max_x - min_x + 1, "height": max_y - min_y + 1}
            result[actor_id] = {
                "pixels": pixels,
                "box": visible_box,
                "eligible": (pixels >= MIN_VISIBLE_PIXELS
                             and visible_box["width"] >= MIN_VISIBLE_BOX_SIDE_PX
                             and visible_box["height"] >= MIN_VISIBLE_BOX_SIDE_PX),
            }
    return result


def match_visible_detections(
    visible: dict[int, dict[str, Any]], boxes: Sequence[Any],
) -> set[int]:
    """Match YOLO vehicle boxes to visible mask boxes, one detection per actor."""
    candidates = []
    for index, box in enumerate(boxes):
        if int(box.class_id) not in (2, 3, 5, 7):
            continue
        detection_box = {"x": float(box.x), "y": float(box.y),
                         "width": float(box.width), "height": float(box.height)}
        for actor_id, item in visible.items():
            if not item["eligible"]:
                continue
            overlap = _box_iou(detection_box, item["box"])
            if overlap >= DETECTION_IOU:
                candidates.append((overlap, index, actor_id))
    matches: set[int] = set()
    used_boxes: set[int] = set()
    for _overlap, index, actor_id in sorted(candidates, reverse=True):
        if index in used_boxes or actor_id in matches:
            continue
        used_boxes.add(index)
        matches.add(actor_id)
    return matches
