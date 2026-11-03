"""Integer route timing regressions; pure computation, no serial or camera IO."""

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.general_transfer_entry import _duration
from arm_project.motion_scene import SceneValidator
from arm_project.route_timing import partition_joint_route


ROOT = Path(__file__).resolve().parents[1]


class RouteTimingTests(unittest.TestCase):
    def partition(self, route, **changes):
        options = {
            "max_segments": 8,
            "maximum_segment_ms": 9000,
            "duration_fn": lambda a, b: 1500,
            "state_valid": lambda p: True,
            "resolution_p": 5,
        }
        options.update(changes)
        return partition_joint_route(route, **options)

    def test_real_400p_edge_uses_three_parts_instead_of_requiring_binary_four(self):
        def read(name):
            return json.loads((ROOT / "configs" / name).read_text(encoding="utf-8"))

        model = read("kinematics.arduino_nominal.json")
        transfer = plan_fixed_transfer(model, read("transfer_task_request.example.json"))
        scene = SceneValidator(model, read("motion_scene.example.json"))
        settings = read("general_entry_planner.example.json")
        settings["maximum_velocity_p_s"] = [32, 100000, 100000, 100000]
        settings["maximum_acceleration_p_s2"] = [100000] * 4
        settings["maximum_jerk_p_s3"] = [100000] * 4
        goal = transfer["nominal_endpoints_p"]["pick_hover"]
        start = [goal[0] - 400, *goal[1:]]

        def duration(a, b):
            return _duration(a, b, settings)

        self.assertEqual(duration(start, goal), 23438)
        route = self.partition(
            [start, goal], max_segments=3, duration_fn=duration, state_valid=scene
        )
        self.assertEqual([point[0] for point in route], [1433, 1566, 1700, 1833])
        self.assertEqual([duration(a, b) for a, b in zip(route, route[1:])], [7793, 7852, 7793])
        for a, b in zip(route, route[1:]):
            for pulse in range(a[0], b[0] + 1):
                self.assertTrue(scene([pulse, *a[1:]]))

    def test_original_rrt_corner_and_both_endpoints_are_preserved(self):
        route = [[1500] * 4, [1530, 1500, 1500, 1500], [1530, 1530, 1500, 1500]]
        before = deepcopy(route)

        def duration(a, b):
            return 100 * max(abs(y - x) for x, y in zip(a, b))

        expanded = self.partition(
            route,
            max_segments=4,
            maximum_segment_ms=2000,
            duration_fn=duration,
            state_valid=lambda p: p[1] == 1500 or p[0] == 1530,
        )
        self.assertEqual(route, before)
        self.assertEqual(len(expanded), 5)
        self.assertEqual(expanded[0], route[0])
        self.assertEqual(expanded[2], route[1])
        self.assertEqual(expanded[-1], route[-1])

    def test_scene_rejection_of_interior_point_cannot_be_overridden_by_timing(self):
        seen = []

        def valid(point):
            seen.append(point)
            return point[0] != 1550

        with self.assertRaisesRegex(ValueError, "scene validation"):
            self.partition([[1500] * 4, [1600, 1500, 1500, 1500]], state_valid=valid)
        self.assertIn((1550, 1500, 1500, 1500), seen)

    def test_infeasible_timing_fails_with_bounded_callback_count(self):
        durations = []

        def duration(a, b):
            durations.append((a, b))
            return 9001

        with self.assertRaisesRegex(ValueError, "finite entry segment budget"):
            self.partition(
                [[1500] * 4, [1600, 1500, 1500, 1500]],
                max_segments=3,
                duration_fn=duration,
            )
        self.assertEqual(len(durations), 6)

    def test_duplicate_integer_subedges_cannot_create_fake_time_capacity(self):
        with self.assertRaisesRegex(ValueError, "finite entry segment budget"):
            self.partition([[1500] * 4, [1501, 1500, 1500, 1500]], duration_fn=lambda a, b: 9001)

    def test_invalid_inputs_and_nonboolean_scene_callbacks_fail_closed(self):
        route = [[1500] * 4, [1600, 1500, 1500, 1500]]
        for changes in (
            {"max_segments": 9},
            {"max_segments": True},
            {"maximum_segment_ms": 9001},
            {"resolution_p": 0},
            {"duration_fn": None},
            {"state_valid": lambda p: 1},
            {"duration_fn": lambda a, b: math.nan},
            {"duration_fn": lambda a, b: True},
            {"duration_fn": lambda a, b: 0},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.partition(route, **changes)
        for invalid in ([], [[1500] * 3], [[True] * 4], [[1500] * 4] * 2):
            with self.subTest(route=invalid), self.assertRaises(ValueError):
                self.partition(invalid)

    def test_callback_exceptions_and_scene_check_budget_produce_no_partial_route(self):
        route = [[1500] * 4, [1600, 1500, 1500, 1500]]

        def failed(*args):
            raise RuntimeError("synthetic callback failure")

        for callback in ("state_valid", "duration_fn"):
            with self.subTest(callback=callback), self.assertRaises(ValueError):
                self.partition(route, **{callback: failed})
        with patch("arm_project.route_timing._MAX_VALIDITY_CHECKS", 1):
            with self.assertRaisesRegex(ValueError, "scene-check budget"):
                self.partition(route)

    def test_single_state_route_is_checked_and_returned_as_detached_lists(self):
        route = [[1500] * 4]
        expanded = self.partition(route)
        self.assertEqual(expanded, route)
        self.assertIsNot(expanded[0], route[0])
        with self.assertRaises(ValueError):
            self.partition(route, state_valid=lambda p: False)
