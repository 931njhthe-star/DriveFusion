"""Conservative radar clusters for correcting tracks during YOLO gaps."""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Any, Sequence


MIN_TRACK_RADAR_POINTS = 3
MAX_TRACK_RADAR_SPREAD_M = 0.5
TRACK_RADAR_NEIGHBOR_M = 1.0
TRACK_RADAR_RATE_NEIGHBOR_MPS = 4.0


@dataclass(frozen=True)
class RadarTrackCluster:
    world_x_m: float
    world_y_m: float
    u_px: float
    v_px: float
    range_m: float
    range_rate_mps: float
    point_count: int


def radar_track_clusters(
    points: Sequence[dict[str, float]], boxes: Sequence[Any],
) -> tuple[RadarTrackCluster, ...]:
    """Group radar returns not already explained by a current YOLO box."""
    available = []
    for point in points:
        required = (
            "u", "v", "ground_u", "ground_v", "world_x_m", "world_y_m",
            "range_m", "range_rate_mps"
        )
        if not all(point.get(key) is not None
                   and math.isfinite(float(point[key])) for key in required):
            continue
        if any(box.x - 4 <= point["u"] <= box.x + box.width + 4
               and box.y - 4 <= point["v"] <= box.y + box.height + 4
               for box in boxes):
            continue
        available.append(point)

    used: set[int] = set()
    clusters = []
    for index, seed in enumerate(available):
        if index in used:
            continue
        neighbors = [
            other_index for other_index, other in enumerate(available)
            if other_index not in used
            and math.hypot(other["world_x_m"] - seed["world_x_m"],
                           other["world_y_m"] - seed["world_y_m"])
            <= TRACK_RADAR_NEIGHBOR_M
            and abs(other["range_m"] - seed["range_m"]) <= MAX_TRACK_RADAR_SPREAD_M
            and abs(other["range_rate_mps"] - seed["range_rate_mps"])
            <= TRACK_RADAR_RATE_NEIGHBOR_MPS
        ]
        if len(neighbors) < MIN_TRACK_RADAR_POINTS:
            continue
        group = [available[other_index] for other_index in neighbors]
        center_range = median(point["range_m"] for point in group)
        if max(abs(point["range_m"] - center_range) for point in group) > MAX_TRACK_RADAR_SPREAD_M:
            continue
        used.update(neighbors)
        clusters.append(RadarTrackCluster(
            world_x_m=median(point["world_x_m"] for point in group),
            world_y_m=median(point["world_y_m"] for point in group),
            u_px=median(point["ground_u"] for point in group),
            v_px=median(point["ground_v"] for point in group),
            range_m=center_range,
            range_rate_mps=median(point["range_rate_mps"] for point in group),
            point_count=len(group),
        ))
    return tuple(clusters)
