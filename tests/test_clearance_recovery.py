"""桌面软余量脱离仅作名义模型检查；所有测试均不操作硬件。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.clearance_recovery import RecoverySceneGuard, TableRecovery
from arm_project.motion_scene import SceneValidator


ROOT = Path(__file__).resolve().parents[1]
START = [1833, 1331, 2163, 1221]
HOVER = [1833, 1344, 1893, 938]


def scene_config(table_z_mm=6.440537650113015):
    # 原始模型pick的胶囊余量8.4405mm；上移假设桌面后明确构造2mm软余量起点。
    return {
        "schema": "jixiebi.motion_scene.v1",
        "axis_bounds_p": [[500, 2500] for _ in range(4)],
        "table_z_mm": table_z_mm,
        "minimum_clearance_mm": 5,
        "maximum_radial_mm": 365,
        "link_radii_mm": [8, 8, 4],
        "self_collision_margin_mm": 2,
        "obstacles_aabb_mm": [],
        "physical_geometry_verified": False,
    }


class ClearanceRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )

    def setUp(self):
        self.config = scene_config()
        self.scene = SceneValidator(self.model, self.config)
        self.recovery = TableRecovery(self.scene, START)

    def test_only_positive_soft_margin_start_is_eligible_and_scene_stays_unchanged(self):
        before = deepcopy(self.config)
        check = self.recovery.inspect_start()
        self.assertTrue(self.recovery.eligible)
        self.assertTrue(check["eligible"])
        self.assertFalse(check["regular_scene_check"]["valid"])
        self.assertEqual(check["regular_scene_check"]["code"], "TABLE_CLEARANCE")
        self.assertTrue(check["hard_scene_check"]["valid"])
        self.assertAlmostEqual(check["initial_minimum_clearance_mm"], 2)
        self.assertEqual(len(check["initial_link_clearances_mm"]), 3)
        self.assertEqual(self.scene.minimum_clearance_mm, 5)
        self.assertEqual(self.config, before)
        check["hard_scene_check"]["links"][0]["minimum_table_clearance_mm"] = -999
        self.assertGreater(
            self.recovery.inspect_start()["hard_scene_check"]["links"][0][
                "minimum_table_clearance_mm"
            ],
            0,
        )
        self.assertFalse(TableRecovery(self.scene, HOVER).eligible)

    def test_penetration_touch_and_subhard_margin_are_never_eligible(self):
        original_tip_clearance = 8.440537650113015
        for desired_clearance in (-1, 0, 0.499):
            with self.subTest(clearance=desired_clearance):
                scene = SceneValidator(
                    self.model,
                    scene_config(original_tip_clearance - desired_clearance),
                )
                recovery = TableRecovery(scene, START)
                self.assertFalse(recovery.eligible)
                self.assertFalse(recovery.state_valid(START))
                self.assertFalse(recovery.validate_route([START, HOVER])["valid"])
        # 正好达到0.5mm与SceneValidator既有“<阈值拒绝”语义相同，不能暗中扩大容差。
        exact = SceneValidator(self.model, scene_config(original_tip_clearance - 0.5))
        self.assertEqual(exact.inspect(START)["minimum_table_clearance_mm"], 0.5)
        self.assertTrue(TableRecovery(exact, START).eligible)

    def test_other_collision_hidden_behind_ordinary_table_failure_is_rejected(self):
        # 默认直立本身不会自碰撞；显式加大自碰撞余量，构造原TABLE提前返回会隐藏的失败。
        config = scene_config(90)
        config["self_collision_margin_mm"] = 64
        scene = SceneValidator(self.model, config)
        start = [1500] * 4
        recovery = TableRecovery(scene, start)
        check = recovery.inspect_start()
        self.assertEqual(check["regular_scene_check"]["code"], "TABLE_CLEARANCE")
        self.assertEqual(check["hard_scene_check"]["code"], "SELF_COLLISION")
        self.assertFalse(recovery.eligible)
        config = scene_config()
        config["maximum_radial_mm"] = 100
        scene = SceneValidator(self.model, config)
        self.assertFalse(TableRecovery(scene, START).eligible)

    def test_recovery_cannot_mask_axis_or_obstacle_failure(self):
        self.assertFalse(TableRecovery(self.scene, [500, *START[1:]]).eligible)
        config = scene_config()
        config["obstacles_aabb_mm"] = [{"min": [-1, -1, 140], "max": [1, 1, 160]}]
        scene = SceneValidator(self.model, config)
        check = TableRecovery(scene, [1500] * 4).inspect_start()
        self.assertEqual(check["hard_scene_check"]["code"], "OBSTACLE_COLLISION")
        self.assertFalse(check["eligible"])
        # 从合法软余量起点出发，候选超限也必须失败。
        invalid = self.recovery.inspect_state([500, *START[1:]])
        self.assertFalse(invalid["valid"])
        self.assertEqual(invalid["scene_code"], "AXIS_LIMIT")

    def test_link_specific_floor_rejects_regression_even_when_global_minimum_improves(self):
        point = [1833, 1321, 2153, 1226]
        hard_check = self.scene.inspect(point, minimum_clearance_mm=0.5)
        self.assertTrue(hard_check["valid"])
        self.assertGreater(hard_check["minimum_table_clearance_mm"], 2)
        check = self.recovery.inspect_state(point)
        self.assertFalse(check["valid"])
        self.assertEqual(check["code"], "RECOVERY_LINK_CLEARANCE_DECREASED")
        self.assertEqual(check["link_index"], 1)

    def test_valid_escape_is_sampled_and_final_margin_is_mandatory(self):
        check = self.recovery.validate_route([START, HOVER])
        self.assertTrue(check["valid"], check)
        self.assertGreater(check["sampled_state_count"], 2)
        self.assertIsNotNone(check["normal_transition_sample_index"])
        self.assertFalse(check["physical_collision_free_verified"])
        self.assertFalse(check["continuous_path_verified"])
        self.assertFalse(check["motion_authorized"])
        partial = self.recovery.validate_route([START, [1833, 1333, 2163, 1221]])
        self.assertFalse(partial["valid"])
        self.assertEqual(partial["code"], "RECOVERY_FINAL_MARGIN_NOT_REACHED")

    def test_interpolated_dip_is_rejected_even_when_endpoint_is_ordinary_valid(self):
        endpoint = [1833, 1513, 2204, 906]
        self.assertTrue(self.scene(endpoint))
        self.assertTrue(self.recovery.state_valid(endpoint))
        check = self.recovery.validate_route([START, endpoint])
        self.assertFalse(check["valid"])
        self.assertEqual(check["failure"]["edge_index"], 0)
        self.assertGreater(check["failure"]["sample_index"], 0)
        self.assertEqual(check["code"], "RECOVERY_LINK_CLEARANCE_DECREASED")

    def test_partial_search_prefix_requires_explicit_false_and_reports_missing_margin(self):
        route = [START, [1833, 1333, 2163, 1221]]
        self.assertFalse(self.recovery.validate_route(route)["valid"])
        partial = self.recovery.validate_route(route, require_normal_end=False)
        self.assertTrue(partial["valid"], partial)
        self.assertFalse(partial["normal_margin_reached"])
        self.assertIsNone(partial["normal_transition_sample_index"])
        self.assertEqual(partial["code"], "TABLE_RECOVERY_PARTIAL_ROUTE_VALID")
        self.assertFalse(partial["motion_authorized"])
        self.assertFalse(partial["physical_collision_free_verified"])
        dip = [START, [1833, 1513, 2204, 906]]
        self.assertFalse(self.recovery.validate_route(dip, require_normal_end=False)["valid"])

    def test_normal_end_flag_is_a_strict_bool_and_full_route_reports_margin_reached(self):
        for value in (None, 0, 1, "false", [], {}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.recovery.validate_route([START, HOVER], require_normal_end=value)
        for require_normal_end in (True, False):
            result = self.recovery.validate_route(
                [START, HOVER], require_normal_end=require_normal_end
            )
            self.assertTrue(result["valid"], result)
            self.assertTrue(result["normal_margin_reached"])
            self.assertEqual(result["code"], "TABLE_RECOVERY_ROUTE_VALID")

    def test_progress_cannot_retreat_from_an_improved_soft_margin(self):
        guard = RecoverySceneGuard(self.scene, self.recovery)
        high_soft = [1833, 1335, 2163, 1221]
        lower_soft = [1833, 1334, 2163, 1221]
        self.assertTrue(self.recovery.state_valid(lower_soft))
        self.assertTrue(guard(high_soft))
        self.assertFalse(guard(lower_soft))
        self.assertEqual(guard.last_check["code"], "RECOVERY_PROGRESS_DECREASED")
        self.assertTrue(guard(high_soft))
        self.assertFalse(guard.normal_scene_active)

    def test_unaffected_higher_link_may_lower_above_its_initial_floor(self):
        guard = RecoverySceneGuard(self.scene, self.recovery)
        first = [1833, 1334, 2163, 1221]
        second = [1833, 1331, 2160, 1223]
        first_check = self.scene.inspect(first, minimum_clearance_mm=0.5)
        second_check = self.scene.inspect(second, minimum_clearance_mm=0.5)
        self.assertGreater(
            second_check["minimum_table_clearance_mm"], first_check["minimum_table_clearance_mm"]
        )
        self.assertLess(
            second_check["links"][1]["minimum_table_clearance_mm"],
            first_check["links"][1]["minimum_table_clearance_mm"],
        )
        self.assertTrue(guard(first))
        self.assertTrue(guard(second))

    def test_normal_latch_accepts_ordinary_lowering_but_never_reenters_soft_margin(self):
        guard = RecoverySceneGuard(self.scene, self.recovery)
        lower_normal = [1833, 1338, 2163, 1221]
        soft = [1833, 1336, 2163, 1221]
        self.assertTrue(guard(START))
        self.assertTrue(guard(HOVER))
        self.assertTrue(guard.normal_scene_active)
        self.assertTrue(guard(lower_normal))
        self.assertFalse(guard(soft))
        self.assertEqual(guard.last_check["code"], "TABLE_CLEARANCE")
        self.assertTrue(guard.normal_scene_active)
        self.assertFalse(guard(START))
        self.assertTrue(guard(HOVER))
        check = self.recovery.validate_route([START, HOVER, lower_normal])
        self.assertTrue(check["valid"], check)
        check = self.recovery.validate_route([START, HOVER, START, HOVER])
        self.assertFalse(check["valid"])
        self.assertEqual(check["code"], "TABLE_CLEARANCE")

    def test_first_ordinary_valid_observation_releases_extra_initial_high_link_floor(self):
        # 前臂仍有152.87mm余量，略低于起点153.86mm；末端已达5.20mm，普通全套检查通过。
        # 这些离散观察只验运行门禁交接；不把两点之间未经逐边审阅的运动声明为有效路径。
        handoff = [1833, 1321, 2153, 1240]
        regular = self.scene.inspect(handoff)
        self.assertTrue(regular["valid"])
        self.assertLess(
            regular["links"][1]["minimum_table_clearance_mm"],
            self.recovery.initial_link_clearances_mm[1],
        )
        self.assertFalse(self.recovery.state_valid(handoff))
        guard = RecoverySceneGuard(self.scene, self.recovery)
        self.assertTrue(guard(START))
        self.assertFalse(guard.normal_scene_active)
        self.assertTrue(guard(handoff), guard.last_check)
        self.assertTrue(guard.normal_scene_active)
        self.assertEqual(guard.last_check["code"], "NOMINAL_POSE_VALID")
        self.assertFalse(guard(START))
        self.assertTrue(guard.normal_scene_active)
        self.assertEqual(guard.last_check["code"], "TABLE_CLEARANCE")

    def test_runtime_noise_does_not_lower_initial_or_positive_hard_floors(self):
        guard = RecoverySceneGuard(self.scene, self.recovery)
        self.assertFalse(guard([1833, 1330, 2163, 1221]))
        self.assertFalse(guard([1833, 1325, 2163, 1221]))
        self.assertTrue(guard(START))
        self.assertFalse(guard.normal_scene_active)

    def test_malformed_arguments_fail_explicitly_and_all_checks_are_offline(self):
        for hard in (-1, 0, 0.49, True, float("nan"), float("inf")):
            with self.subTest(hard=hard), self.assertRaises(ValueError):
                TableRecovery(self.scene, START, hard_clearance_mm=hard)
        for resolution in (0, True, 5.0, 2001):
            with self.subTest(resolution=resolution), self.assertRaises(ValueError):
                TableRecovery(self.scene, START, resolution_p=resolution)
        for route in (None, [], [HOVER, START], [START, [1500] * 3], [START, [True] * 4]):
            with self.subTest(route=route), self.assertRaises(ValueError):
                self.recovery.validate_route(route)
        with self.assertRaises(ValueError):
            RecoverySceneGuard(SceneValidator(self.model, self.config), self.recovery)
        with self.assertRaises(ValueError):
            RecoverySceneGuard(self.scene, TableRecovery(self.scene, HOVER))
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            recovery = TableRecovery(self.scene, START)
            result = recovery.validate_route([START, HOVER])
            self.assertTrue(result["valid"])
            self.assertEqual(json.loads(json.dumps(result)), result)


if __name__ == "__main__":
    unittest.main()
