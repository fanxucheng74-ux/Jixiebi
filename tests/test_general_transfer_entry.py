"""通用入口只验名义搜索、时序和新鲜状态边界；不接串口或相机。"""

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.general_transfer_entry import (
    plan_general_transfer_entry,
    plan_simulated_general_entry,
)
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_plan import compose_transfer_task
from test_transfer_entry import BOOT, policy, state_for


ROOT = Path(__file__).resolve().parents[1]


class GeneralTransferEntryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model_template = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )
        cls.request_template = json.loads(
            (ROOT / "configs/fixed_transfer_nominal_example.json").read_text(encoding="utf-8")
        )
        cls.scene_template = json.loads(
            (ROOT / "configs/motion_scene.example.json").read_text(encoding="utf-8")
        )
        cls.settings_template = json.loads(
            (ROOT / "configs/general_entry_planner.example.json").read_text(encoding="utf-8")
        )

    def setUp(self):
        self.model = deepcopy(self.model_template)
        self.transfer = plan_fixed_transfer(self.model, deepcopy(self.request_template))
        self.scene = deepcopy(self.scene_template)
        self.settings = deepcopy(self.settings_template)
        self.rules = policy()
        self.left = self.transfer["nominal_endpoints_p"]["pick_hover"] + [1430, 1711]
        self.right = self.transfer["nominal_endpoints_p"]["place_hover"] + [1430, 1711]

    def simulated(self, positions):
        return plan_simulated_general_entry(
            self.model, self.transfer, positions, self.rules, self.scene, self.settings
        )

    def live(self, positions=None, payload=None):
        return plan_general_transfer_entry(
            self.model,
            self.transfer,
            state_for(self.right if positions is None else positions)
            if payload is None
            else payload,
            self.rules,
            self.scene,
            self.settings,
            now_monotonic_s=101,
            host_boot_id=BOOT,
            expected_session_id="test-session",
        )

    def assert_proposal(self, result, positions):
        self.assertEqual(result["status"], "NOMINAL_ENTRY_PROPOSAL", result.get("reason"))
        self.assertEqual(result["route4_p"][0], positions[:4])
        self.assertEqual(result["route4_p"][-1], self.left[:4])
        self.assertEqual(len(result["entry_segments"]), len(result["route4_p"]) - 1)
        self.assertEqual(
            result["entry_declared_ms"], sum(row["duration_ms"] for row in result["entry_segments"])
        )
        self.assertEqual(
            result["entry_and_transfer_declared_ms"],
            result["entry_declared_ms"] + self.transfer["declared_ms_per_transfer"],
        )
        self.assertEqual(
            result["held_axes_without_new_targets"], {"4": positions[4], "5": positions[5]}
        )
        for index, row in enumerate(result["entry_segments"]):
            self.assertEqual(row["from4_p"], result["route4_p"][index])
            self.assertEqual(row["to4_p"], result["route4_p"][index + 1])
            self.assertGreaterEqual(row["duration_ms"], self.settings["minimum_segment_ms"])
            self.assertLessEqual(row["duration_ms"], self.settings["maximum_segment_ms"])
        for key in (
            "motion_authorized",
            "physical_geometry_verified",
            "physical_path_verified",
            "hardware_interpolation_implemented",
            "asynchronous_motion_envelope_verified",
            "gripper_contents_known",
            "gripper_open_state_verified",
            "task_ready_verified",
        ):
            self.assertFalse(result[key], key)

    def test_factory_upright_is_planned_without_legacy_template_classification(self):
        current = [1500] * 4 + [1430, 1711]
        with patch(
            "arm_project.transfer_entry._classify",
            side_effect=AssertionError("legacy classification"),
        ):
            result = self.simulated(current)
        self.assert_proposal(result, current)
        self.assertEqual(result["search"]["stats"]["method"], "DIRECT")
        self.assertEqual(result["source"], "SIMULATION")
        self.assertIsNone(result["freshness"])

    def test_left_hover_is_zero_motion_and_right_hover_has_finite_entry(self):
        left = self.simulated(self.left)
        self.assertEqual(left["status"], "ALREADY_AT_START")
        self.assertEqual(left["entry_declared_ms"], 0)
        self.assertEqual(left["entry_segments"], [])
        self.assertEqual(left["route4_p"], [self.left[:4]])
        self.assertNotIn("search", left)
        self.assertFalse(left["gripper_contents_known"])
        self.assert_proposal(self.simulated(self.right), self.right)

    def test_low_pick_pose_is_checked_by_scene_and_not_rejected_by_old_height_template(self):
        current = self.transfer["nominal_endpoints_p"]["pick"] + [1430, 1711]
        result = self.simulated(current)
        self.assertLess(
            result["current_pose_nominal_mm"]["z_mm"],
            self.rules["minimum_tip_height_above_table_mm"],
        )
        self.assert_proposal(result, current)

    def test_other_valid_pose_is_planned_without_matching_left_right_or_upright(self):
        current = [1610, 1510, 1490, 1510, 1401, 1599]
        self.assertTrue(SceneValidator(self.model, self.scene)(current[:4]))
        self.assert_proposal(self.simulated(current), current)

    def test_invalid_start_table_and_self_collision_return_no_route(self):
        for current, code in (
            ([1500, 834, 1800, 1500, 1430, 1711], "TABLE_CLEARANCE"),
            ([1500, 1500, 2500, 834, 1430, 1711], "SELF_COLLISION"),
        ):
            with self.subTest(code=code):
                result = self.simulated(current)
                self.assertEqual(result["status"], "NOMINAL_PATH_BLOCKED")
                self.assertIn("INVALID_START", result["reason"])
                self.assertEqual(result["start_scene_check"]["code"], code)
                self.assertEqual(result["entry_segments"], [])
                self.assertEqual(result["route4_p"], [])
                self.assertEqual(result["samples"], [])
                self.assertEqual(result["entry_declared_ms"], 0)

    def test_invalid_goal_is_reported_before_search(self):
        self.scene["axis_bounds_p"][0] = [1400, 1600]
        with patch("arm_project.general_transfer_entry.plan_joint_path") as search:
            result = self.simulated([1500] * 4 + [1430, 1711])
            search.assert_not_called()
        self.assertEqual(result["status"], "NOMINAL_PATH_BLOCKED")
        self.assertIn("INVALID_GOAL", result["reason"])
        self.assertTrue(result["start_scene_check"]["valid"])
        self.assertFalse(result["goal_scene_check"]["valid"])
        self.assertEqual(result["entry_segments"], [])

    def test_live_entry_keeps_same_boot_freshness_and_all_physical_flags_unverified(self):
        result = self.live()
        self.assert_proposal(result, self.right)
        self.assertEqual(result["source"], "HARDWARE_STATE")
        self.assertEqual(result["freshness"]["host_boot_id"], BOOT)
        self.assertEqual(result["freshness"]["session_id"], "test-session")
        self.assertFalse(result["freshness"]["metadata_is_source_authentication"])

    def test_stale_future_cross_boot_session_and_incomplete_state_remain_rejected(self):
        for edit in (
            lambda p: p["state"]["servos"][0].update(received_monotonic_s=97),
            lambda p: p["state"]["servos"][-1].update(received_monotonic_s=102),
            lambda p: p["state"].update(host_boot_id="aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"),
            lambda p: p.update(session_id="old-session"),
            lambda p: p["state"].update(source="REPLAY"),
            lambda p: p["state"]["servos"][3].update(servo_id=2),
            lambda p: p["state"]["servos"][2].update(position_p=True),
            lambda p: p["state"]["servos"][0].update(status="NOT_READ"),
        ):
            payload = state_for(self.right)
            edit(payload)
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.live(payload=payload)

    def test_obstacle_detour_returned_by_search_keeps_every_segment_through_composition(self):
        # 固定躯干左右扫掠会穿过前方小盒；用真实场景检查绕行点与每个预览姿态。
        self.scene["obstacles_aabb_mm"] = [{"min": [-5, 190, 45], "max": [5, 210, 80]}]
        route = [
            self.right[:4],
            [self.right[0], 1500, 1500, 1500],
            [self.left[0], 1500, 1500, 1500],
            self.left[:4],
        ]
        middle = [1500] + self.left[1:4]
        validator = SceneValidator(self.model, self.scene)
        self.assertFalse(validator(middle))
        self.assertTrue(validator(route[0]))
        self.assertTrue(validator(route[-1]))

        def routed_search(start, goal, bounds, scene, **kwargs):
            self.assertEqual(list(start), route[0])
            self.assertEqual(list(goal), route[-1])
            self.assertFalse(scene(middle))
            self.assertEqual(bounds, validator.bounds4_p)
            return {
                "status": "PATH_FOUND",
                "waypoints4_p": deepcopy(route),
                "continuous_collision_check": False,
                "motion_authorized": False,
                "stats": {"method": "RRT_CONNECT", "reason": "simulated selected detour"},
            }

        # 真实RRT搜索独立验收；此处固定搜索输出，以专门检验包装与编排没有拉直绕行。
        with patch("arm_project.general_transfer_entry.plan_joint_path", side_effect=routed_search):
            result = self.simulated(self.right)
        self.assert_proposal(result, self.right)
        self.assertEqual(result["route4_p"], route)
        self.assertEqual(result["search"]["stats"]["method"], "RRT_CONNECT")
        task = compose_transfer_task(
            self.transfer,
            result,
            {"open_p": 1200, "close_p": 1570, "duration_ms": 1000},
            "SIM-route",
        )
        self.assertEqual(
            [p["name"] for p in task["phases"][:4]],
            ["entry_01", "entry_02", "entry_03", "prepare_gripper"],
        )
        for index, waypoint in enumerate(route[1:]):
            phase = task["phases"][index]
            self.assertEqual([phase["targets"][str(sid)] for sid in range(4)], waypoint)
            self.assertEqual(phase["targets"]["4"], self.right[4])
            self.assertEqual(phase["targets"]["5"], self.right[5])
        self.assertFalse(task["motion_authorized"])

    def test_duration_satisfies_nominal_velocity_acceleration_and_jerk_formula(self):
        current = list(self.left)
        current[0] -= 400
        for key, expected_seconds in (
            ("maximum_velocity_p_s", 1.875 * 400 / 100),
            ("maximum_acceleration_p_s2", math.sqrt((10 * math.sqrt(3) / 3) * 400 / 100)),
            ("maximum_jerk_p_s3", (60 * 400 / 100) ** (1 / 3)),
        ):
            for field in ("maximum_velocity_p_s", "maximum_acceleration_p_s2", "maximum_jerk_p_s3"):
                self.settings[field] = [100000] * 4
            self.settings[key][0] = 100
            result = self.simulated(current)
            self.assert_proposal(result, current)
            self.assertEqual(len(result["entry_segments"]), 1)
            duration = result["entry_segments"][0]["duration_ms"]
            self.assertEqual(duration, math.ceil(expected_seconds * 1000))
            seconds = duration / 1000
            self.assertLessEqual(1.875 * 400 / seconds, self.settings["maximum_velocity_p_s"][0])
            self.assertLessEqual(
                (10 * math.sqrt(3) / 3) * 400 / seconds**2,
                self.settings["maximum_acceleration_p_s2"][0],
            )
            self.assertLessEqual(60 * 400 / seconds**3, self.settings["maximum_jerk_p_s3"][0])
            self.assertFalse(result["hardware_interpolation_implemented"])

    def test_dynamic_duration_over_budget_and_failed_search_expose_no_segments(self):
        current = list(self.left)
        current[0] -= 400
        self.settings["maximum_velocity_p_s"] = [0.001] * 4
        result = self.simulated(current)
        self.assertEqual(result["status"], "NOMINAL_PATH_BLOCKED")
        self.assertEqual(result["entry_segments"], [])
        self.assertEqual(result["route4_p"], [])
        self.assertEqual(result["samples"], [])
        self.assertEqual(result["entry_declared_ms"], 0)
        self.settings = deepcopy(self.settings_template)
        for status in ("PLANNING_TIMEOUT", "SEARCH_EXHAUSTED", "PATH_TOO_COMPLEX"):
            with patch(
                "arm_project.general_transfer_entry.plan_joint_path",
                return_value={
                    "status": status,
                    "waypoints4_p": [],
                    "stats": {"reason": "finite search budget"},
                },
            ):
                failed = self.simulated(self.right)
            self.assertEqual(failed["status"], "NOMINAL_PATH_BLOCKED")
            self.assertIn(status, failed["reason"])
            self.assertEqual(failed["entry_segments"], [])
            self.assertEqual(failed["route4_p"], [])
            self.assertEqual(failed["entry_declared_ms"], 0)

    def test_preview_revalidation_rejects_colliding_search_output_without_partial_route(self):
        self.scene["obstacles_aabb_mm"] = [{"min": [-5, 190, 45], "max": [5, 210, 80]}]
        with patch(
            "arm_project.general_transfer_entry.plan_joint_path",
            return_value={
                "status": "PATH_FOUND",
                "waypoints4_p": [self.right[:4], self.left[:4]],
                "stats": {"method": "SIMULATED_INVALID_SEARCH"},
            },
        ):
            result = self.simulated(self.right)
        self.assertEqual(result["status"], "NOMINAL_PATH_BLOCKED")
        self.assertEqual(result["entry_segments"], [])
        self.assertEqual(result["samples"], [])
        self.assertEqual(result["route4_p"], [])

    def test_invalid_settings_and_scene_table_mismatch_fail_closed(self):
        for field, value in (
            ("maximum_velocity_p_s", [0] * 4),
            ("maximum_acceleration_p_s2", [True] * 4),
            ("maximum_jerk_p_s3", [float("inf")] * 4),
            ("minimum_segment_ms", 1499),
            ("maximum_segment_ms", 9001),
            ("max_segments", 9),
            ("seed", True),
            ("extra", 1),
        ):
            self.settings = {**self.settings_template, field: value}
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.simulated([1500] * 6)
        self.settings = deepcopy(self.settings_template)
        self.scene["table_z_mm"] = 1
        with self.assertRaisesRegex(ValueError, "桌面"):
            self.simulated([1500] * 6)

    def test_invalid_search_settings_are_rejected_even_when_already_at_start(self):
        for field, value in (
            ("step_p", 0),
            ("step_p", True),
            ("resolution_p", 2001),
            ("max_iterations", 0),
            ("max_nodes", 1),
            ("seed", True),
            ("seed", 2**32),
            ("time_budget_s", 0.0009),
            ("time_budget_s", float("nan")),
            ("time_budget_s", False),
        ):
            self.settings = {**self.settings_template, field: value}
            with self.subTest(field=field, value=value):
                with patch("arm_project.general_transfer_entry.plan_joint_path") as search:
                    with self.assertRaises(ValueError):
                        self.simulated(self.left)
                    search.assert_not_called()

    def test_no_io_input_mutation_or_unknown_roll_grip_inference(self):
        current = [1500] * 4 + [1311, 1699]
        before = deepcopy(
            (self.model, self.transfer, self.rules, self.scene, self.settings, current)
        )
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            result = self.simulated(current)
        self.assert_proposal(result, current)
        self.assertEqual(
            (self.model, self.transfer, self.rules, self.scene, self.settings, current), before
        )
        self.assertEqual(result["current_positions6_p"][4:], [1311, 1699])
        self.assertFalse(result["gripper_contents_known"])
        self.assertEqual(json.loads(json.dumps(result)), result)

    def test_left_offset_four_p_requires_explicit_entry_beyond_strict_no_motion_band(self):
        self.assertEqual(self.rules["at_start_tolerance_p"], 5)
        near = list(self.left)
        near[0] += 3
        self.assertEqual(self.simulated(near)["status"], "ALREADY_AT_START")
        current = list(self.left)
        current[0] += 4
        result = self.simulated(current)
        self.assert_proposal(result, current)
        self.assertEqual(len(result["entry_segments"]), 1)
        self.assertEqual(result["entry_segments"][0]["from4_p"], current[:4])
        self.assertEqual(result["entry_segments"][0]["to4_p"], self.left[:4])
        self.assertGreaterEqual(result["entry_declared_ms"], 1500)
        task = compose_transfer_task(
            self.transfer,
            result,
            {"open_p": 1200, "close_p": 1570, "duration_ms": 1000},
            "SIM-four-p",
        )
        self.assertEqual(task["motion_phases"][0]["name"], "entry_01")
        self.assertEqual(task["motion_phases"][0]["targets"]["0"], self.left[0])

    def test_long_direct_edge_is_split_into_valid_connected_composed_segments(self):
        current = list(self.left)
        current[0] -= 400
        for field in ("maximum_velocity_p_s", "maximum_acceleration_p_s2", "maximum_jerk_p_s3"):
            self.settings[field] = [100000] * 4
        self.settings["maximum_velocity_p_s"][0] = 100
        self.settings["maximum_segment_ms"] = 3000
        self.settings["max_segments"] = 3
        # 原单段需要7500ms；三等分可满足预算，不能固定二分成4段后误拒。
        self.assertEqual(math.ceil(1.875 * 400 / 100 * 1000), 7500)
        result = self.simulated(current)
        self.assert_proposal(result, current)
        self.assertEqual(result["search"]["stats"]["method"], "DIRECT")
        self.assertEqual(result["search"]["waypoints4_p"], [current[:4], self.left[:4]])
        self.assertEqual(len(result["entry_segments"]), 3)
        scene = SceneValidator(self.model, self.scene)
        for index, segment in enumerate(result["entry_segments"]):
            self.assertEqual(segment["from4_p"], result["route4_p"][index])
            self.assertEqual(segment["to4_p"], result["route4_p"][index + 1])
            self.assertEqual(segment["duration_ms"], [2494, 2513, 2494][index])
            start, goal = segment["from4_p"], segment["to4_p"]
            count = math.ceil(
                max(abs(b - a) for a, b in zip(start, goal)) / self.settings["resolution_p"]
            )
            for step in range(count + 1):
                self.assertTrue(
                    scene([round(a + (b - a) * step / count) for a, b in zip(start, goal)])
                )
        task = compose_transfer_task(
            self.transfer,
            result,
            {"open_p": 1200, "close_p": 1570, "duration_ms": 1000},
            "SIM-split-edge",
        )
        self.assertEqual(
            [p["name"] for p in task["motion_phases"][:4]],
            ["entry_01", "entry_02", "entry_03", "prepare_gripper"],
        )
        for index, segment in enumerate(result["entry_segments"]):
            self.assertEqual(
                task["motion_phases"][index]["targets"],
                {str(sid): position for sid, position in enumerate(segment["to4_p"])},
            )
        self.assertEqual(task["entry_declared_ms"], 7501)
        self.assertFalse(task["motion_authorized"])


if __name__ == "__main__":
    unittest.main()
