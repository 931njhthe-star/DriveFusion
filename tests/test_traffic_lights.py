"""Traffic-light modes should preserve the intended CARLA signal behavior."""

import unittest
from unittest.mock import MagicMock, call

import carla

from monitor.qt.traffic_lights import apply_traffic_lights


class TrafficLightSettingsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.world = MagicMock()
        self.lights = [MagicMock(), MagicMock()]
        for index, light in enumerate(self.lights):
            light.id = index + 1
            light.get_pole_index.return_value = index
            light.get_group_traffic_lights.return_value = self.lights
            light.is_frozen.return_value = False
        self.world.get_actors.return_value.filter.return_value = self.lights

    def test_no_red_sets_all_green_then_freezes(self) -> None:
        result = apply_traffic_lights(self.world, False, 15)
        self.assertEqual(result.light_count, 2)
        self.assertIsNone(result.expected_red_min_seconds)
        for light in self.lights:
            light.set_state.assert_called_once_with(carla.TrafficLightState.Green)
            light.set_red_time.assert_not_called()
        self.world.freeze_all_traffic_lights.assert_called_once_with(True)

    def test_red_uses_cycle_and_requested_duration(self) -> None:
        result = apply_traffic_lights(self.world, True, 23)
        self.assertEqual(result.light_count, 2)
        self.assertEqual(result.expected_red_min_seconds, 23.0)
        for light in self.lights:
            light.set_green_time.assert_called_once_with(10.0)
            light.set_yellow_time.assert_called_once_with(3.0)
            light.set_red_time.assert_called_once_with(5.0)
            light.set_state.assert_not_called()
        self.lights[0].reset_group.assert_called_once_with()
        self.lights[1].reset_group.assert_not_called()
        self.assertEqual(self.world.freeze_all_traffic_lights.call_args_list,
                         [call(True), call(False)])

    def test_three_phase_junction_preserves_driving_time(self) -> None:
        third = MagicMock()
        third.id = 3
        third.get_pole_index.return_value = 2
        third.is_frozen.return_value = False
        self.lights.append(third)
        for light in self.lights:
            light.get_group_traffic_lights.return_value = self.lights
        green, yellow, all_red = 10.0, 3.0, 0.5
        result = apply_traffic_lights(self.world, True, 10)
        self.assertEqual(result.light_count, 3)
        self.assertEqual(result.expected_red_min_seconds, 27.5)
        self.assertEqual(3 * all_red + 2 * (green + yellow), 27.5)
        for light in self.lights:
            light.set_green_time.assert_called_once_with(green)
            light.set_yellow_time.assert_called_once_with(yellow)
            light.set_red_time.assert_called_once_with(all_red)

    def test_short_red_request_clamped_to_junction_minimum(self) -> None:
        result = apply_traffic_lights(self.world, True, 2)
        self.assertEqual(result.expected_red_min_seconds, 14.0)
        for light in self.lights:
            light.set_red_time.assert_called_once_with(0.5)

    def test_no_lights_reports_failure_without_freezing(self) -> None:
        self.world.get_actors.return_value.filter.return_value = []
        with self.assertRaisesRegex(RuntimeError, "No traffic lights"):
            apply_traffic_lights(self.world, False, 15)
        self.world.freeze_all_traffic_lights.assert_not_called()

    def test_invalid_duration_does_not_change_world(self) -> None:
        with self.assertRaises(ValueError):
            apply_traffic_lights(self.world, True, 0)
        self.world.get_actors.assert_not_called()


if __name__ == "__main__":
    unittest.main()
