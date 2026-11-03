"""纯关节P空间算法测试；不访问相机、串口或ROS，不证明实物安全。"""

import math
import unittest
from unittest.mock import patch

from arm_project.joint_path import plan_joint_path


BOUNDS = [(500, 2500)] * 4
PLANAR_BOUNDS = [(1000, 2000), (1000, 2000), (1500, 1500), (1500, 1500)]
START = [1100, 1500, 1500, 1500]
GOAL = [1900, 1500, 1500, 1500]


def rectangle_free(p):
    # 中间矩形占据直接通道，上下两侧保留绕行空间；后两轴固定。
    return not (1400 <= p[0] <= 1600 and 1250 <= p[1] <= 1750)


class JointPathTests(unittest.TestCase):
    def assert_valid_path(self, result, start, goal, bounds, validator):
        self.assertEqual(result["status"], "PATH_FOUND", result)
        path = result["waypoints4_p"]
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        self.assertFalse(result["continuous_collision_check"])
        self.assertFalse(result["motion_authorized"])
        self.assertLessEqual(result["stats"]["validity_checks"], 100000)
        for index, point in enumerate(path):
            self.assertTrue(all(type(p) is int for p in point))
            self.assertTrue(all(low <= p <= high for p, (low, high) in zip(point, bounds)))
            self.assertTrue(validator(point))
            if index:
                self.assertNotEqual(point, path[index - 1])
        for a, b in zip(path, path[1:]):
            count = max(1, max(abs(q - p) for p, q in zip(a, b)))
            for i in range(count + 1):
                self.assertTrue(
                    validator(tuple(round(p + (q - p) * i / count) for p, q in zip(a, b)))
                )

    def test_direct_from_upright_and_unrecognized_four_axis_postures(self):
        for start in ([1500] * 4, [1777, 1423, 1568, 1321]):
            goal = [1840, 1620, 1700, 1400]
            result = plan_joint_path(start, goal, BOUNDS, lambda _: True)
            self.assert_valid_path(result, start, goal, BOUNDS, lambda _: True)
            self.assertEqual(result["waypoints4_p"], [start, goal])
            self.assertEqual(result["stats"]["method"], "DIRECT")
            self.assertEqual(result["stats"]["iterations"], 0)

    def test_rectangle_obstacle_requires_reproducible_detour(self):
        args = (START, GOAL, PLANAR_BOUNDS, rectangle_free)
        first = plan_joint_path(*args, time_budget_s=5)
        second = plan_joint_path(*args, time_budget_s=5)
        self.assert_valid_path(first, START, GOAL, PLANAR_BOUNDS, rectangle_free)
        self.assertGreater(len(first["waypoints4_p"]), 2)
        self.assertLessEqual(len(first["waypoints4_p"]), 9)
        self.assertEqual(first["waypoints4_p"], second["waypoints4_p"])
        self.assertEqual(first["stats"]["validity_checks"], second["stats"]["validity_checks"])

    def test_disconnected_space_exhausts_finite_iterations(self):
        result = plan_joint_path(
            START,
            GOAL,
            PLANAR_BOUNDS,
            lambda p: not 1400 <= p[0] <= 1600,
            max_iterations=30,
            max_nodes=100,
            time_budget_s=5,
        )
        self.assertEqual(result["status"], "SEARCH_EXHAUSTED")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertLessEqual(result["stats"]["iterations"], 30)
        self.assertLessEqual(result["stats"]["nodes"], 100)

    def test_invalid_endpoints_fail_before_search(self):
        for start, goal, validator, status in (
            ([999, 1500, 1500, 1500], GOAL, lambda _: True, "INVALID_START"),
            (START, [2001, 1500, 1500, 1500], lambda _: True, "INVALID_GOAL"),
            (START, GOAL, lambda p: p != tuple(START), "INVALID_START"),
            (START, GOAL, lambda p: p != tuple(GOAL), "INVALID_GOAL"),
        ):
            with self.subTest(status=status):
                result = plan_joint_path(start, goal, PLANAR_BOUNDS, validator)
                self.assertEqual(result["status"], status)
                self.assertEqual(result["waypoints4_p"], [])
                self.assertEqual(result["stats"]["iterations"], 0)

    def test_equal_start_goal_and_single_p_quantization(self):
        position = [1500] * 4
        same = plan_joint_path(position, position, BOUNDS, lambda _: True)
        self.assertEqual(same["status"], "PATH_FOUND")
        self.assertEqual(same["waypoints4_p"], [position])
        goal = [1501, 1499, 1501, 1500]
        checked = []
        result = plan_joint_path(
            position, goal, BOUNDS, lambda p: checked.append(p) is None, resolution_p=1
        )
        self.assert_valid_path(result, position, goal, BOUNDS, lambda _: True)
        self.assertGreaterEqual(checked.count(tuple(position)), 3)
        self.assertGreaterEqual(checked.count(tuple(goal)), 3)

    def test_node_and_complexity_caps_are_enforced(self):
        capped = plan_joint_path(START, GOAL, PLANAR_BOUNDS, rectangle_free, max_nodes=2)
        self.assertEqual(capped["status"], "SEARCH_EXHAUSTED")
        self.assertEqual(capped["stats"]["nodes"], 2)
        complex_path = plan_joint_path(
            START, GOAL, PLANAR_BOUNDS, rectangle_free, max_segments=1, time_budget_s=5
        )
        self.assertEqual(complex_path["status"], "PATH_TOO_COMPLEX")
        self.assertEqual(complex_path["waypoints4_p"], [])

    def test_timeout_checked_after_validator_and_during_connection(self):
        now = [0.0]

        def slow(_):
            now[0] = 2.0
            return True

        with patch("arm_project.joint_path.time.monotonic", side_effect=lambda: now[0]):
            result = plan_joint_path(START, GOAL, PLANAR_BOUNDS, slow)
        self.assertEqual(result["status"], "PLANNING_TIMEOUT")
        self.assertEqual(result["stats"]["validity_checks"], 1)
        calls = [0]

        def during_connect(p):
            calls[0] += 1
            if calls[0] > 70:
                now[0] = 2.0
            return rectangle_free(p)

        now[0] = 0.0
        with patch("arm_project.joint_path.time.monotonic", side_effect=lambda: now[0]):
            result = plan_joint_path(START, GOAL, PLANAR_BOUNDS, during_connect)
        self.assertEqual(result["status"], "PLANNING_TIMEOUT")
        self.assertGreater(result["stats"]["nodes"], 2)
        self.assertEqual(result["waypoints4_p"], [])

    def test_output_is_rechecked_and_failure_never_exposes_path(self):
        count = [0]

        def changes_after_direct_check(_):
            count[0] += 1
            return count[0] <= 5

        result = plan_joint_path(
            [1000] * 4, [1020] * 4, BOUNDS, changes_after_direct_check, resolution_p=10
        )
        self.assertEqual(result["status"], "SEARCH_EXHAUSTED")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertIn("final validation", result["stats"]["reason"])

    def test_validity_call_budget_is_enforced_inside_edges(self):
        with patch("arm_project.joint_path._MAX_VALIDITY_CHECKS", 4):
            result = plan_joint_path([500] * 4, [2500] * 4, BOUNDS, lambda _: True, resolution_p=1)
        self.assertEqual(result["status"], "SEARCH_EXHAUSTED")
        self.assertEqual(result["stats"]["validity_checks"], 4)
        self.assertEqual(result["waypoints4_p"], [])

    def test_callback_exceptions_and_non_bool_fail_closed(self):
        def broken(_):
            raise RuntimeError("synthetic validator error")

        for callback in (broken, lambda _: None, lambda _: 1, lambda _: "valid"):
            with self.subTest(callback=callback), self.assertRaises(ValueError):
                plan_joint_path(START, GOAL, PLANAR_BOUNDS, callback)

    def test_malformed_inputs_and_unbounded_options_are_rejected(self):
        for name, values in {
            "step_p": [True, 0, 2001, 1.5],
            "resolution_p": [False, 0, 2001],
            "max_iterations": [True, 0, 10001, 10**100],
            "max_nodes": [True, 1, 10001],
            "max_segments": [True, 0, 33],
            "seed": [False, -1, 2**32],
            "time_budget_s": [True, 0, math.inf, math.nan, 11, 10**1000],
        }.items():
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    plan_joint_path(START, GOAL, PLANAR_BOUNDS, lambda _: True, **{name: value})
        for start, goal, bounds, callback in (
            ([True, 1500, 1500, 1500], GOAL, BOUNDS, lambda _: True),
            (START[:3], GOAL, BOUNDS, lambda _: True),
            (START, [2501] * 4, BOUNDS, lambda _: True),
            (START, GOAL, [(500, 2500)] * 3, lambda _: True),
            (START, GOAL, [(True, 2500)] * 4, lambda _: True),
            (START, GOAL, [(2000, 1000)] * 4, lambda _: True),
            (START, GOAL, BOUNDS, None),
        ):
            with self.subTest(start=start, bounds=bounds), self.assertRaises(ValueError):
                plan_joint_path(start, goal, bounds, callback)


if __name__ == "__main__":
    unittest.main()
