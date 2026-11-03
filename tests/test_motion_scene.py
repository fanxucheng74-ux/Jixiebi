"""名义静态场景的几何和拒绝条件；半径为显式假设，不进行实物验收。"""

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.kinematics import from_nominal_pulses
from arm_project.motion_scene import SceneValidator, _segment_distance


ROOT = Path(__file__).resolve().parents[1]


def scene():
    return {
        "schema": "jixiebi.motion_scene.v1",
        "axis_bounds_p": [[500, 2500] for _ in range(4)],
        "table_z_mm": 0,
        "minimum_clearance_mm": 5,
        "maximum_radial_mm": 365,
        "link_radii_mm": [8, 8, 4],
        "self_collision_margin_mm": 2,
        "obstacles_aabb_mm": [],
        "physical_geometry_verified": False,
    }


class MotionSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal_model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )
        request = json.loads(
            (ROOT / "configs/fixed_transfer_nominal_example.json").read_text(encoding="utf-8")
        )
        cls.endpoints = plan_fixed_transfer(cls.nominal_model, request)["nominal_endpoints_p"]

    def setUp(self):
        self.model = deepcopy(self.nominal_model)
        self.config = scene()

    def test_upright_and_nominal_transfer_endpoints_satisfy_assumed_capsules(self):
        validator = SceneValidator(self.model, self.config)
        for positions in ([1500] * 4, *self.endpoints.values()):
            with self.subTest(positions=positions):
                result = validator.inspect(positions)
                self.assertTrue(result["valid"], result)
                self.assertTrue(validator(positions))
                self.assertEqual(len(result["chain_points_mm"]), 4)
                self.assertEqual(len(result["links"]), 3)
                self.assertGreaterEqual(result["minimum_table_clearance_mm"], 5)
                self.assertLessEqual(result["maximum_capsule_radial_mm"], 365)
                self.assertGreater(result["nonadjacent_centerline_distance_mm"], 14)
        pick = validator.inspect(self.endpoints["pick"])
        self.assertAlmostEqual(pick["minimum_table_clearance_mm"], 8.440537650113015)

    def test_integer_bounds_intersect_explicit_and_actual_nominal_angle_limits(self):
        validator = SceneValidator(self.model, self.config)
        self.assertEqual(validator.bounds4_p, ((834, 2166), (834, 2166), (500, 2500), (834, 2166)))
        self.model["nominal_angular_limits_deg"]["yaw_deg"] = [-12, 24]
        self.config["axis_bounds_p"][1] = [1400, 1600]
        validator = SceneValidator(self.model, self.config)
        self.assertEqual(validator.bounds4_p[0], (1323, 1588))
        self.assertEqual(validator.bounds4_p[1], (1400, 1600))
        for pulse in range(1322, 1590):
            yaw = from_nominal_pulses([pulse, 1500, 1500, 1500]).yaw_deg
            self.assertEqual(1323 <= pulse <= 1588, -12 <= yaw <= 24)
        self.assertFalse(validator([1322, 1500, 1500, 1500]))
        self.assertFalse(validator([1500, 1399, 1500, 1500]))

    def test_bounds_do_not_round_outwards_and_empty_intersection_is_rejected(self):
        self.model["nominal_angular_limits_deg"]["yaw_deg"] = [-0.135, 0.135]
        validator = SceneValidator(self.model, self.config)
        self.assertEqual(validator.bounds4_p[0], (1499, 1501))
        self.config["axis_bounds_p"][0] = [1400, 1410]
        with self.assertRaisesRegex(ValueError, "no overlap"):
            SceneValidator(self.model, self.config)

    def test_table_and_radial_checks_include_capsule_radius(self):
        self.config["table_z_mm"] = 90
        result = SceneValidator(self.model, self.config).inspect([1500] * 4)
        self.assertEqual(result["code"], "TABLE_CLEARANCE")
        self.assertEqual(result["link_index"], 0)
        self.assertEqual(result["minimum_table_clearance_mm"], 2)
        # 中心线最大径向约360mm，小于363mm；工具半径4mm让胶囊超过包络。
        self.config = scene()
        self.config["maximum_radial_mm"] = 363
        result = SceneValidator(self.model, self.config).inspect([1500, 834, 1500, 1500])
        self.assertFalse(result["valid"])
        self.assertEqual(result["code"], "RADIAL_ENVELOPE")
        self.assertLess(max(math.hypot(*p[:2]) for p in result["chain_points_mm"]), 363)

    def test_table_clearance_override_is_scoped_and_preserves_regular_threshold(self):
        self.config["table_z_mm"] = 90
        before = deepcopy(self.config)
        validator = SceneValidator(self.model, self.config)
        positions = [1500] * 4
        strict = validator.inspect(positions)
        self.assertEqual(strict["code"], "TABLE_CLEARANCE")
        self.assertEqual(strict["required_table_clearance_mm"], 5)
        self.assertEqual(strict["regular_table_clearance_mm"], 5)
        scoped = validator.inspect(positions, minimum_clearance_mm=0.5)
        self.assertTrue(scoped["valid"], scoped)
        self.assertEqual(scoped["minimum_table_clearance_mm"], 2)
        self.assertEqual(scoped["required_table_clearance_mm"], 0.5)
        self.assertEqual(scoped["regular_table_clearance_mm"], 5)
        self.assertEqual(validator.minimum_clearance_mm, 5)
        self.assertEqual(self.config, before)
        self.assertEqual(validator.inspect(positions), strict)
        self.assertFalse(validator(positions))

    def test_table_clearance_override_does_not_bypass_other_scene_checks(self):
        # 此姿态距桌面2mm；放宽该次余量之后，仍需检查自碰撞、障碍与运动域。
        self.config["table_z_mm"] = 90
        cases = (
            ({"self_collision_margin_mm": 64}, [1500] * 4, "SELF_COLLISION"),
            (
                {"obstacles_aabb_mm": [{"min": [-0.01, -0.01, 150], "max": [0.01, 0.01, 151]}]},
                [1500] * 4,
                "OBSTACLE_COLLISION",
            ),
            ({}, [500, 1500, 1500, 1500], "AXIS_LIMIT"),
            ({"maximum_radial_mm": 363}, [1500, 834, 1500, 1500], "RADIAL_ENVELOPE"),
        )
        for changes, positions, code in cases:
            with self.subTest(code=code):
                validator = SceneValidator(self.model, {**self.config, **changes})
                result = validator.inspect(positions, minimum_clearance_mm=0.5)
                self.assertFalse(result["valid"], result)
                self.assertEqual(result["code"], code)
                self.assertEqual(result["required_table_clearance_mm"], 0.5)
                self.assertEqual(result["regular_table_clearance_mm"], 5)

    def test_table_clearance_override_rejects_invalid_values_before_positions(self):
        validator = SceneValidator(self.model, self.config)
        for value in (float("nan"), float("inf"), True, False, -0.1, 100001, "0.5"):
            for positions in (None, [1500] * 4):
                with self.subTest(value=value, positions=positions):
                    with self.assertRaisesRegex(ValueError, "minimum_clearance_mm"):
                        validator.inspect(positions, minimum_clearance_mm=value)
        self.assertEqual(validator.minimum_clearance_mm, 5)

    def test_table_clearance_override_accepts_numeric_domain_and_checks_exact_boundary(self):
        self.config["table_z_mm"] = 90
        validator = SceneValidator(self.model, self.config)
        for value in (0, 0.5, 2):
            with self.subTest(value=value):
                result = validator.inspect([1500] * 4, minimum_clearance_mm=value)
                self.assertTrue(result["valid"], result)
                self.assertEqual(result["required_table_clearance_mm"], value)
        for value in (2.001, 100000):
            with self.subTest(value=value):
                result = validator.inspect([1500] * 4, minimum_clearance_mm=value)
                self.assertEqual(result["code"], "TABLE_CLEARANCE")
        invalid = validator.inspect(None, minimum_clearance_mm=0.5)
        self.assertEqual(invalid["code"], "INVALID_POSITIONS")
        self.assertEqual(invalid["required_table_clearance_mm"], 0.5)
        self.assertEqual(invalid["regular_table_clearance_mm"], 5)

    def test_nonadjacent_capsules_detect_folded_pose_without_table_collision(self):
        result = SceneValidator(self.model, self.config).inspect([1500, 1500, 2500, 834])
        self.assertFalse(result["valid"])
        self.assertEqual(result["code"], "SELF_COLLISION")
        self.assertGreater(result["minimum_table_clearance_mm"], 5)
        self.assertLess(result["nonadjacent_centerline_distance_mm"], 1)
        self.assertEqual(result["nonadjacent_required_distance_mm"], 14)
        # 相邻连杆接头中心重合是模型定义；默认直立姿态不因此被误报为自碰撞。
        self.assertTrue(SceneValidator(self.model, self.config)([1500] * 4))

    def test_explicit_self_collision_margin_is_not_ignored(self):
        self.config["self_collision_margin_mm"] = 64
        result = SceneValidator(self.model, self.config).inspect([1500] * 4)
        self.assertEqual(result["code"], "SELF_COLLISION")
        self.assertAlmostEqual(result["nonadjacent_centerline_distance_mm"], 75)
        self.assertEqual(result["nonadjacent_required_distance_mm"], 76)

    def test_obstacle_intersection_checks_interior_of_thin_boxes(self):
        # 盒体仅0.0001mm厚，上臂端点均在外面，仍必须拒绝线段内部穿过。
        self.config["obstacles_aabb_mm"] = [
            {"min": [-0.01, -0.01, 150.0001], "max": [0.01, 0.01, 150.0002]}
        ]
        result = SceneValidator(self.model, self.config).inspect([1500] * 4)
        self.assertFalse(result["valid"])
        self.assertEqual(result["code"], "OBSTACLE_COLLISION")
        self.assertEqual(result["obstacle_index"], 0)
        self.assertEqual(result["link_index"], 0)
        self.assertLess(result["chain_points_mm"][0][2], 150)
        self.assertGreater(result["chain_points_mm"][1][2], 151)

    def test_obstacle_radius_expansion_includes_touch_and_clear_box_stays_valid(self):
        # 中心线位于X=0；盒体X=8处与半径8胶囊相切，也按碰撞拒绝。
        self.config["obstacles_aabb_mm"] = [{"min": [8, -0.01, 150], "max": [8.01, 0.01, 151]}]
        result = SceneValidator(self.model, self.config).inspect([1500] * 4)
        self.assertEqual(result["code"], "OBSTACLE_COLLISION")
        self.config["obstacles_aabb_mm"][0]["min"][0] = 8.001
        self.assertTrue(SceneValidator(self.model, self.config)([1500] * 4))
        self.config["obstacles_aabb_mm"] = [{"min": [100, 100, 150], "max": [101, 101, 151]}]
        self.assertTrue(SceneValidator(self.model, self.config)([1500] * 4))

    def test_segment_distance_handles_crossing_parallel_skew_and_degenerate_segments(self):
        cases = (
            ((-2, 0, 0), (2, 0, 0), (0, -2, 0), (0, 2, 0), 0),
            ((-2, 0, 0), (2, 0, 0), (0, -2, 3), (0, 2, 3), 3),
            ((0, 0, 0), (4, 0, 0), (1, 2, 0), (3, 2, 0), 2),
            ((0, 0, 0), (4, 0, 0), (1, 0, 0), (3, 0, 0), 0),
            ((0, 0, 0), (0, 0, 0), (0, 3, 0), (0, 6, 0), 3),
            # 近乎平行的长段在内部交叉，不得因分母阈值而漏掉碰撞。
            ((0, 0, 0), (10000, 0, 0), (0, -0.001, 0), (10000, 0.001, 0), 0),
        )
        for a, b, c, d, expected in cases:
            with self.subTest(a=a, b=b, c=c, d=d):
                self.assertAlmostEqual(_segment_distance(a, b, c, d), expected, places=9)
                self.assertAlmostEqual(_segment_distance(c, d, a, b), expected, places=9)

    def test_invalid_positions_return_false_without_old_success(self):
        validator = SceneValidator(self.model, self.config)
        self.assertTrue(validator([1500] * 4))
        for positions in (None, [1500] * 3, [1500] * 5, [True, 1500, 1500, 1500], [499] * 4):
            with self.subTest(positions=positions):
                self.assertFalse(validator(positions))
                self.assertEqual(validator.inspect(positions)["code"], "INVALID_POSITIONS")
        self.assertEqual(validator.inspect([500, 1500, 1500, 1500])["code"], "AXIS_LIMIT")

    def test_invalid_scene_inputs_fail_before_planning(self):
        mutations = (
            ("schema", "unknown"),
            ("physical_geometry_verified", True),
            ("physical_geometry_verified", 0),
            ("table_z_mm", float("nan")),
            ("minimum_clearance_mm", -1),
            ("maximum_radial_mm", 0),
            ("axis_bounds_p", [[500, 2500]] * 3),
            ("axis_bounds_p", [[500, 2500]] * 3 + [[1501, 1500]]),
            ("axis_bounds_p", [[500, 2500]] * 3 + [[True, 2500]]),
            ("axis_bounds_p", [[500, 2500]] * 3 + [[499, 2500]]),
            ("link_radii_mm", [8, 8]),
            ("link_radii_mm", [8, 8, 0]),
            ("link_radii_mm", [8, 8, True]),
            ("self_collision_margin_mm", -1),
            ("obstacles_aabb_mm", {}),
            ("obstacles_aabb_mm", [{"min": [0, 0, 0], "max": [1, 1, 1], "extra": 1}]),
            ("obstacles_aabb_mm", [{"min": [0, 0], "max": [1, 1, 1]}]),
            ("obstacles_aabb_mm", [{"min": [0, 0, 0], "max": [0, 1, 1]}]),
            ("obstacles_aabb_mm", [{"min": [0, 0, 0], "max": [1, float("inf"), 1]}]),
        )
        for field, value in mutations:
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                SceneValidator(self.model, {**self.config, field: value})
        for config in (
            {k: v for k, v in self.config.items() if k != "table_z_mm"},
            {**self.config, "extra": 1},
        ):
            with self.assertRaises(ValueError):
                SceneValidator(self.model, config)

    def test_wrong_model_source_geometry_or_joint_limit_fields_rejected(self):
        for field, value in (
            ("source", "CALIBRATED"),
            ("frame", "unknown"),
            ("endpoint", "grasp_center"),
            ("physical_geometry_verified", True),
            ("motion_authorized", True),
            ("geometry_mm", {}),
            ("nominal_angular_limits_deg", {}),
        ):
            with self.subTest(field=field), self.assertRaises(ValueError):
                SceneValidator({**self.model, field: value}, self.config)

    def test_no_io_no_mutation_and_nominal_notes_do_not_claim_calibration(self):
        before = deepcopy((self.model, self.config))
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            validator = SceneValidator(self.model, self.config)
            result = validator.inspect([1500] * 4)
        self.assertEqual((self.model, self.config), before)
        self.assertTrue(result["valid"])
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["physical_geometry_verified"])
        self.assertFalse(result["physical_collision_free_verified"])
        self.assertFalse(result["continuous_path_verified"])
        self.assertEqual(result["link_radii_source"], "EXPLICIT_UNMEASURED_SCENE_ASSUMPTIONS")
        self.assertIn("Base body", " ".join(result["notes"]))
        self.assertIn("calibration", " ".join(result["notes"]))
        self.assertEqual(json.loads(json.dumps(result)), result)


if __name__ == "__main__":
    unittest.main()
