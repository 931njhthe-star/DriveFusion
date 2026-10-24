"""Apply CARLA traffic-light settings without blocking the dashboard."""

from __future__ import annotations

from dataclasses import dataclass

import carla
from PySide6.QtCore import QThread, Signal


GREEN_SECONDS = 10.0
YELLOW_SECONDS = 3.0
MIN_ALL_RED_SECONDS = 0.5


@dataclass(frozen=True)
class TrafficLightResult:
    light_count: int
    expected_red_min_seconds: float | None = None
    expected_red_max_seconds: float | None = None


def _phase_times(group: list[carla.TrafficLight], red_seconds: float
                 ) -> tuple[float, float, float, float]:
    """Keep driving phases and get as close as possible to the requested red.

    CARLA cycles one pole at a time. While another pole runs green/yellow/red,
    this pole stays red. Thus visible red is n*r + (n-1)*(g+y).
    """
    pole_count = len({light.get_pole_index() for light in group})
    if pole_count <= 1:
        return GREEN_SECONDS, YELLOW_SECONDS, red_seconds, red_seconds
    other_phases = (pole_count - 1) * (GREEN_SECONDS + YELLOW_SECONDS)
    all_red_seconds = max(
        MIN_ALL_RED_SECONDS, (red_seconds - other_phases) / pole_count)
    visible_red_seconds = pole_count * all_red_seconds + other_phases
    return GREEN_SECONDS, YELLOW_SECONDS, all_red_seconds, visible_red_seconds


def apply_traffic_lights(world: carla.World, use_red: bool,
                         red_seconds: float) -> TrafficLightResult:
    """Cycle with 10s green and a target red wait, or freeze all green."""
    if use_red and not 1 <= red_seconds <= 300:
        raise ValueError("Red-light duration must be between 1 and 300 seconds")
    lights = list(world.get_actors().filter("traffic.traffic_light*"))
    if not lights:
        raise RuntimeError("No traffic lights were found in the CARLA world")
    if use_red:
        groups: list[list[carla.TrafficLight]] = []
        seen: set[int] = set()
        for light in lights:
            if light.id in seen:
                continue
            group = list(light.get_group_traffic_lights())
            if not group:
                group = [light]
            groups.append(group)
            seen.update(member.id for member in group)
        plans = [(group, _phase_times(group, red_seconds)) for group in groups]
        was_frozen = all(light.is_frozen() for light in lights)
        world.freeze_all_traffic_lights(True)
        try:
            for group, times in plans:
                for light in group:
                    green, yellow, all_red, _ = times
                    light.set_green_time(green)
                    light.set_yellow_time(yellow)
                    light.set_red_time(all_red)
                group[0].reset_group()
        except Exception:
            world.freeze_all_traffic_lights(was_frozen)
            raise
        world.freeze_all_traffic_lights(False)
        expected = [times[3] for _, times in plans]
        return TrafficLightResult(len(lights), min(expected), max(expected))
    else:
        for light in lights:
            light.set_state(carla.TrafficLightState.Green)
        world.freeze_all_traffic_lights(True)
    return TrafficLightResult(len(lights))


class TrafficLightControlThread(QThread):
    applied = Signal(object, bool, float)
    failed = Signal(str)

    def __init__(self, host: str, port: int, use_red: bool, red_seconds: float,
                 parent=None) -> None:
        super().__init__(parent)
        self.host = host
        self.port = port
        self.use_red = use_red
        self.red_seconds = red_seconds

    def run(self) -> None:
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(10.0)
            result = apply_traffic_lights(client.get_world(), self.use_red, self.red_seconds)
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.applied.emit(result, self.use_red, self.red_seconds)
