"""有界桌面脱离搜索的名义几何与预算；不连接串口或操作实物。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.clearance_recovery import TableRecovery
from arm_project.motion_scene import SceneValidator
from arm_project.table_escape import plan_table_escape


ROOT = Path(__file__).resolve().parents[1]
START = [1833, 1331, 2163, 1221]
HOVER = [1833, 1344, 1893, 938]
DIPPING_GOAL = [1833, 1513, 2204, 906]


def scene_config():
    # 上移假设桌面，原pick姿态的胶囊余量由8.4405mm变为2mm；不代表实测位置。
    return {
        "schema": "jixiebi.motion_scene.v1",
        "axis_bounds_p": [[500, 2500] for _ in range(4)],
        "table_z_mm": 6.440537650113015,
        "minimum_clearance_mm": 5,
        "maximum_radial_mm": 365,
        "link_radii_mm": [8, 8, 4],
        "self_collision_margin_mm": 2,
        "obstacles_aabb_mm": [],
        "physical_geometry_verified": False,
    }


class TableEscapeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )

    def setUp(self):
        self.config = scene_config()
        self.scene = SceneValidator(self.model, self.config)

    def assert_valid_escape(self, result, scene=None):
        scene = self.scene if scene is None else scene
        self.assertEqual(result["status"], "ESCAPE_FOUND", result)
        route = result["waypoints4_p"]
        self.assertEqual(route[0], START)
        self.assertTrue(scene(route[-1]))
        check = TableRecovery(scene, START).validate_route(route)
        self.assertTrue(check["valid"], check)
        self.assertTrue(check["normal_margin_reached"])
        self.assertTrue(result["recovery"]["route_check"]["valid"])
        self.assertLessEqual(len(route) - 1, result["stats"]["max_segments"])

    def test_soft_margin_direct_escape_keeps_normal_scene_and_never_opens_hardware(self):
        before = deepcopy((self.model, self.config))
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            result = plan_table_escape(self.scene, START, HOVER)
        self.assert_valid_escape(result)
        self.assertEqual(result["waypoints4_p"], [START, HOVER])
        self.assertEqual(result["stats"]["method"], "DIRECT")
        self.assertEqual(result["recovery"]["hard_clearance_mm"], 0.5)
        self.assertAlmostEqual(result["recovery"]["initial_minimum_clearance_mm"], 2)
        self.assertFalse(self.scene(START))
        self.assertEqual(self.scene.minimum_clearance_mm, 5)
        self.assertEqual((self.model, self.config), before)
        for field in (
            "motion_authorized",
            "physical_geometry_verified",
            "physical_collision_free_verified",
            "physical_path_verified",
            "continuous_path_verified",
            "continuous_collision_check",
        ):
            self.assertFalse(result[field], field)
        self.assertEqual(json.loads(json.dumps(result)), result)

    def test_interpolated_dip_uses_validated_one_axis_escape_instead(self):
        recovery = TableRecovery(self.scene, START)
        direct = recovery.validate_route([START, DIPPING_GOAL])
        self.assertFalse(direct["valid"])
        self.assertTrue(self.scene(DIPPING_GOAL))
        result = plan_table_escape(self.scene, START, DIPPING_GOAL)
        self.assert_valid_escape(result)
        self.assertEqual(result["stats"]["method"], "BOUNDED_BEST_FIRST")
        self.assertFalse(result["recovery"]["direct_route_check"]["valid"])
        route = result["waypoints4_p"]
        for start, end in zip(route, route[1:]):
            self.assertEqual(start[0], end[0])
            changed = [index for index, (a, b) in enumerate(zip(start, end)) if a != b]
            self.assertEqual(len(changed), 1)
            self.assertIn(changed[0], (1, 2, 3))
            self.assertIn(abs(end[changed[0]] - start[changed[0]]), (2, 5, 10, 20, 40))

    def test_negative_touch_and_subhard_start_are_not_recoverable(self):
        for clearance in (-1, 0, 0.499):
            with self.subTest(clearance=clearance):
                config = scene_config()
                config["table_z_mm"] = 8.440537650113015 - clearance
                scene = SceneValidator(self.model, config)
                result = plan_table_escape(scene, START, HOVER)
                self.assertEqual(result["status"], "INVALID_START")
                self.assertEqual(result["waypoints4_p"], [])
                self.assertFalse(result["recovery"]["hard_scene_check"]["valid"])

    def test_other_collision_hidden_by_soft_table_failure_is_not_recoverable(self):
        self.config["obstacles_aabb_mm"] = [{"min": [-1, -1, 95], "max": [1, 1, 110]}]
        scene = SceneValidator(self.model, self.config)
        self.assertEqual(scene.inspect(START)["code"], "TABLE_CLEARANCE")
        result = plan_table_escape(scene, START, HOVER)
        self.assertEqual(result["status"], "INVALID_START")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertEqual(result["recovery"]["hard_scene_check"]["code"], "OBSTACLE_COLLISION")

    def test_regular_valid_start_is_not_given_a_recovery_exception(self):
        result = plan_table_escape(self.scene, HOVER, DIPPING_GOAL)
        self.assertEqual(result["status"], "INVALID_START")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertTrue(result["recovery"]["regular_scene_check"]["valid"])

    def test_no_candidate_in_explicit_bounds_returns_empty_path(self):
        self.config["axis_bounds_p"] = [[p, p] for p in START]
        scene = SceneValidator(self.model, self.config)
        result = plan_table_escape(scene, START, HOVER)
        self.assertEqual(result["status"], "NO_RECOVERY_PATH")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertLessEqual(result["stats"]["nodes"], 250)
        self.assertEqual(result["stats"]["expanded_nodes"], 1)

    def test_partial_prefix_can_extend_but_depth_limit_never_forces_a_finish(self):
        # 各单轴的允许增量均不足以脱离，组合肩＋腕后才达到普通余量。
        self.config["axis_bounds_p"] = [[1833, 1833], [1331, 1336], [2163, 2163], [1221, 1226]]
        scene = SceneValidator(self.model, self.config)
        short = plan_table_escape(scene, START, HOVER, max_segments=1)
        self.assertEqual(short["status"], "NO_RECOVERY_PATH")
        self.assertEqual(short["waypoints4_p"], [])
        result = plan_table_escape(scene, START, HOVER, max_segments=3)
        self.assert_valid_escape(result, scene)
        self.assertGreater(len(result["waypoints4_p"]), 2)
        self.assertGreater(result["stats"]["expanded_nodes"], 1)

    def test_candidate_node_budget_is_combined_and_failure_returns_no_partial_path(self):
        result = plan_table_escape(self.scene, START, DIPPING_GOAL, max_nodes=3)
        self.assertEqual(result["status"], "NO_RECOVERY_PATH")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertEqual(result["stats"]["nodes"], 3)
        self.assertGreater(result["stats"]["accepted_nodes"], 1)

    def test_deadline_is_checked_inside_sampled_route_validation(self):
        clock = [0.0]
        original_inspect = self.scene.inspect

        def inspect_with_cost(*args, **kwargs):
            result = original_inspect(*args, **kwargs)
            clock[0] += 0.01
            return result

        with (
            patch("arm_project.table_escape.time.monotonic", side_effect=lambda: clock[0]),
            patch.object(self.scene, "inspect", side_effect=inspect_with_cost),
        ):
            result = plan_table_escape(self.scene, START, HOVER, time_budget_s=0.035)
        self.assertEqual(result["status"], "RECOVERY_TIMEOUT")
        self.assertEqual(result["waypoints4_p"], [])
        self.assertEqual(result["stats"]["route_checks"], 1)
        self.assertEqual(result["stats"]["scene_checks"], 4)
        self.assertLess(result["stats"]["elapsed_s"], 0.05)
        self.assertFalse(self.scene(START))

    def test_malformed_arguments_fail_before_any_search(self):
        arguments = (
            {"scene": None},
            {"start4_p": None},
            {"start4_p": [True, *START[1:]]},
            {"goal4_p": [1500] * 3},
            {"goal4_p": [499] * 4},
            {"resolution_p": 0},
            {"resolution_p": True},
            {"resolution_p": 2001},
            {"max_segments": 0},
            {"max_segments": 9},
            {"max_segments": 3.0},
            {"max_nodes": 0},
            {"max_nodes": 10001},
            {"max_nodes": True},
            {"time_budget_s": 0},
            {"time_budget_s": -0.1},
            {"time_budget_s": 11},
            {"time_budget_s": float("nan")},
            {"time_budget_s": float("inf")},
            {"time_budget_s": True},
        )
        for changes in arguments:
            kwargs = {"scene": self.scene, "start4_p": START, "goal4_p": HOVER, **changes}
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                plan_table_escape(**kwargs)


if __name__ == "__main__":
    unittest.main()
