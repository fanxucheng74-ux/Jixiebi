"""整趟任务仅验证名义编排、边界与声明时间，不打开设备。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.transfer_entry import plan_simulated_transfer_entry
from arm_project.transfer_task_motion import motion_phases_sha256
from arm_project.transfer_task_plan import compose_transfer_task


ROOT = Path(__file__).resolve().parents[1]


class TransferTaskPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )
        cls.request = json.loads(
            (ROOT / "configs/fixed_transfer_nominal_example.json").read_text(encoding="utf-8")
        )
        cls.policy = json.loads(
            (ROOT / "configs/transfer_entry_policy.example.json").read_text(encoding="utf-8")
        )
        cls.transfer = plan_fixed_transfer(cls.model, cls.request)

    def setUp(self):
        # 明确采用模拟P；004与005保留不相同的值，防止整趟编排悄悄重置未知轴。
        self.plan = deepcopy(self.transfer)
        self.positions = self.plan["nominal_endpoints_p"]["place_hover"] + [1430, 1711]
        self.entry = plan_simulated_transfer_entry(
            self.model, self.plan, self.positions, self.policy
        )
        self.grip = {"open_p": 1300, "close_p": 1550, "duration_ms": 800}

    def compose(self):
        return compose_transfer_task(self.plan, self.entry, self.grip, "nominal_transfer_01")

    def set_route(self, *, count=3, duration_ms=2000):
        start = self.entry["current_positions6_p"][:4]
        goal = self.plan["nominal_endpoints_p"]["pick_hover"]
        if count == 3:
            # 两个绕行点偏离首末点直线；编排器不能因终点相同而丢掉它们。
            first = [start[0], start[1] + 10, start[2], start[3]]
            second = [goal[0], goal[1] + 10, goal[2], goal[3]]
            waypoints = [start, first, second, goal]
        else:
            waypoints = [
                [round(a + (b - a) * index / count) for a, b in zip(start, goal)]
                for index in range(count + 1)
            ]
        self.entry["entry_segments"] = [
            {"from4_p": list(a), "to4_p": list(b), "duration_ms": duration_ms}
            for a, b in zip(waypoints, waypoints[1:])
        ]
        self.entry["entry_declared_ms"] = count * duration_ms
        self.entry["entry_and_transfer_declared_ms"] = (
            count * duration_ms + self.plan["declared_ms_per_transfer"]
        )
        return waypoints

    def test_finite_named_order_all_targets_and_continuity(self):
        result = self.compose()
        self.assertEqual(result["status"], "NOMINAL_TASK_PROPOSAL")
        self.assertEqual(result["schema"], "jixiebi.transfer_task.v1")
        self.assertEqual(result["source"], "SIMULATION")
        phases = result["phases"]
        self.assertEqual(len(phases), 2 + 8 * 5)
        self.assertEqual([p["name"] for p in phases[:2]], ["entry", "prepare_gripper"])
        expected = [
            "descend_left",
            "close",
            "lift_left",
            "rotate_right",
            "descend_right",
            "open",
            "lift_right",
            "rotate_left",
        ]
        self.assertEqual([p["name"] for p in phases[2:]], expected * 5)
        self.assertEqual(len({p["phase_id"] for p in phases}), len(phases))
        for phase in phases:
            self.assertEqual(set(phase["targets"]), {str(i) for i in range(6)})
            self.assertEqual(set(phase["start_targets"]), {str(i) for i in range(6)})
            self.assertTrue(all(type(p) is int for p in phase["targets"].values()))
            self.assertEqual(phase["targets"]["4"], 1430)
        self.assertEqual(
            phases[0]["start_targets"], {str(i): p for i, p in enumerate(self.positions)}
        )
        for previous, following in zip(phases, phases[1:]):
            self.assertEqual(previous["targets"], following["start_targets"])
            self.assertEqual(previous["declared_end_ms"], following["declared_start_ms"])
        for cycle in range(5):
            start, finish = phases[2 + cycle * 8], phases[2 + cycle * 8 + 7]
            self.assertEqual(start["start_targets"], finish["targets"])

    def test_entry_is_one_target_and_gripper_preparation_follows(self):
        result = self.compose()
        phases = result["phases"]
        entry, preparation = phases[:2]
        self.assertEqual(len(self.entry["samples"]), 65)
        self.assertEqual(entry["duration_ms"], self.entry["entry_declared_ms"])
        self.assertEqual(
            [entry["targets"][str(i)] for i in range(4)],
            self.plan["nominal_endpoints_p"]["pick_hover"],
        )
        self.assertEqual(entry["targets"]["5"], self.positions[5])
        self.assertEqual(preparation["targets"]["5"], self.grip["open_p"])
        self.assertEqual(preparation["duration_ms"], 800)
        self.assertEqual(phases[3]["targets"]["5"], self.grip["close_p"])
        self.assertEqual(phases[7]["targets"]["5"], self.grip["open_p"])
        # 夹爪开合与底座旋转各自独立，循环目标并未从65个入口采样点生成。
        for index, expected_ids in ((1, {"5"}), (3, {"5"}), (5, {"0"})):
            phase = phases[index]
            changed = {
                sid
                for sid in phase["targets"]
                if phase["targets"][sid] != phase["start_targets"][sid]
            }
            self.assertEqual(changed, expected_ids)

    def test_motion_phases_separate_new_targets_from_full_nominal_state(self):
        result = self.compose()
        self.assertEqual(len(result["motion_phases"]), len(result["phases"]))
        for phase, motion in zip(result["phases"], result["motion_phases"]):
            self.assertEqual(
                motion,
                {
                    "name": phase["phase_id"],
                    "duration_ms": phase["duration_ms"],
                    "targets": phase["motion_targets"],
                },
            )
            if phase["name"] == "entry":
                expected_ids = {"0", "1", "2", "3"}
            elif phase["name"] in {"prepare_gripper", "close", "open"}:
                expected_ids = {"5"}
            elif phase["name"] in {"rotate_right", "rotate_left"}:
                expected_ids = {"0"}
            else:
                expected_ids = {"1", "2", "3"}
            self.assertEqual(set(motion["targets"]), expected_ids)
            self.assertNotIn("4", motion["targets"])

    def test_exact_motion_targets_do_not_depend_on_current_roll_or_grip_noise(self):
        original = self.compose()
        changed_positions = self.positions[:4] + [1460, 1530]
        self.entry = plan_simulated_transfer_entry(
            self.model, self.plan, changed_positions, self.policy
        )
        changed = self.compose()
        self.assertEqual(original["motion_phases"], changed["motion_phases"])
        self.assertNotEqual(original["phases"][0]["targets"], changed["phases"][0]["targets"])

    def test_declared_timing_includes_entry_preparation_and_every_cycle(self):
        result = self.compose()
        entry_ms = self.entry["entry_declared_ms"]
        self.assertEqual(result["declared_ms_per_transfer"], 7400)
        self.assertEqual(result["total_declared_ms"], entry_ms + 800 + 5 * 7400)
        self.assertEqual(
            result["total_declared_ms"], sum(p["duration_ms"] for p in result["phases"])
        )
        self.assertEqual(result["cycle_timing"][0]["declared_start_ms"], entry_ms + 800)
        self.assertEqual(result["cycle_timing"][-1]["declared_end_ms"], result["total_declared_ms"])
        for cycle in result["cycle_timing"]:
            self.assertEqual(cycle["declared_duration_ms"], 7400)
        # 原提案中的500ms开合不可因准备时间800ms或硬件执行器下限而被暗改。
        self.assertEqual(result["phases"][3]["duration_ms"], 500)
        self.assertEqual(result["phases"][7]["duration_ms"], 500)
        self.assertTrue(result["within_10s_per_transfer_declared_only"])
        self.assertFalse(result["within_10s_total_declared_only"])
        self.assertFalse(result["execution_overhead_included"])

    def test_already_at_start_remains_zero_duration_acceptance(self):
        positions = self.plan["nominal_endpoints_p"]["pick_hover"] + [1430, 1711]
        positions[0] += 1  # 入口允许窗口中的偏差，不冒充零时长精确物理复位。
        self.entry = plan_simulated_transfer_entry(self.model, self.plan, positions, self.policy)
        self.assertEqual(self.entry["status"], "ALREADY_AT_START")
        result = self.compose()
        self.assertEqual(result["phases"][0]["duration_ms"], 0)
        self.assertFalse(result["phases"][0]["motion_required"])
        self.assertEqual(result["phases"][0]["motion_targets"], {})
        self.assertEqual(result["motion_phases"][0]["name"], "prepare_gripper")
        self.assertEqual(len(result["motion_phases"]), len(result["phases"]) - 1)
        self.assertTrue(result["phases"][0]["accepted_with_nominal_tolerance_only"])
        self.assertIn("does not command repositioning", result["phases"][0]["caveat"])
        self.assertEqual(result["total_declared_ms"], 800 + 5 * 7400)

    def test_blocked_entry_has_no_partial_task_phases(self):
        for status in (
            "REQUIRES_REVIEWED_LIFT",
            "REQUIRES_POSE_REVIEW",
            "NOMINAL_PATH_BLOCKED",
            "UNKNOWN",
        ):
            with self.subTest(status=status):
                self.entry["status"] = status
                result = self.compose()
                self.assertEqual(result["status"], "BLOCKED")
                self.assertEqual(result["phases"], [])
                self.assertEqual(result["motion_phases"], [])
                self.assertEqual(result["cycle_timing"], [])
                self.assertIsNone(result["total_declared_ms"])
                self.assertFalse(result["motion_authorized"])

    def test_invalid_gripper_and_unbounded_identifiers_fail_closed(self):
        original = deepcopy(self.grip)
        for key, value in (
            ("open_p", True),
            ("open_p", 499),
            ("close_p", 2501),
            ("close_p", original["open_p"]),
            ("duration_ms", 99),
            ("duration_ms", 10000),
            ("duration_ms", 800.0),
            ("unexpected", 1),
        ):
            self.grip = {**original, key: value}
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.compose()
        self.grip = {"open_p": 1300, "close_p": 1550}
        with self.assertRaises(ValueError):
            self.compose()
        for name in ("", "x" * 81, "path/to/task", True):
            with self.subTest(name=name), self.assertRaises(ValueError):
                compose_transfer_task(self.plan, self.entry, original, name)

    def test_cycle_count_bound_and_inconsistent_transfer_rejected(self):
        for count in (0, 6, True, 1.0):
            self.plan = deepcopy(self.transfer)
            self.plan["cycle_count"] = count
            with self.subTest(count=count), self.assertRaises(ValueError):
                self.compose()
        self.plan = deepcopy(self.transfer)
        self.plan["cycle_count"] = 1
        self.assertEqual(len(self.compose()["phases"]), 10)
        for edit in (
            lambda p: p.update(motion_authorized=True),
            lambda p: p.update(declared_ms_per_transfer=7401),
            lambda p: p["stages"].reverse(),
            lambda p: p["stages"][0].update(duration_ms=True),
            lambda p: p["nominal_endpoints_p"].update(pick=[1500] * 3),
        ):
            self.plan = deepcopy(self.transfer)
            edit(self.plan)
            with self.assertRaises(ValueError):
                self.compose()

    def test_mismatched_or_relabelled_entry_rejected(self):
        original = deepcopy(self.entry)
        for edit in (
            lambda p: p.update(source="REPLAY"),
            lambda p: p.update(motion_authorized=True),
            lambda p: p.update(entry_declared_ms=0),
            lambda p: p.update(entry_and_transfer_declared_ms=1),
            lambda p: p["target_pick_hover_p"].__setitem__(0, 1500),
            lambda p: p["held_axes_without_new_targets"].update({"5": 1500}),
            lambda p: p["current_positions6_p"].__setitem__(5, None),
        ):
            self.entry = deepcopy(original)
            edit(self.entry)
            with self.assertRaises(ValueError):
                self.compose()

    def test_no_io_no_input_mutation_and_all_physical_claims_remain_false(self):
        before = deepcopy((self.plan, self.entry, self.grip))
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            result = self.compose()
        self.assertEqual((self.plan, self.entry, self.grip), before)
        self.assertEqual(result["source"], "SIMULATION")
        self.assertIsNone(result["entry_freshness"])
        for key in (
            "motion_authorized",
            "hardware_execution_allowed",
            "current_timed_cycle_executor_compatible",
            "hardware_interpolation_implemented",
            "physical_geometry_verified",
            "physical_angles_verified",
            "physical_path_verified",
            "physical_grasp_verified",
            "physical_stop_verified",
            "physical_cycles_verified",
            "gripper_contents_known",
            "gripper_open_state_verified",
        ):
            self.assertIs(result[key], False, key)
        self.assertIn("ID 0", result["executor_compatibility_reason"])
        self.assertIn("mathematical previews", " ".join(result["caveats"]))
        self.assertEqual(json.loads(json.dumps(result)), result)

    def test_entry_route_preserves_every_detour_phase_state_and_motion_target(self):
        legacy_hash = motion_phases_sha256(self.compose())
        waypoints = self.set_route()
        original = deepcopy(self.entry)
        result = self.compose()
        self.assertEqual(
            [p["phase_id"] for p in result["phases"][:4]],
            ["entry_01", "entry_02", "entry_03", "prepare_gripper"],
        )
        for index, phase in enumerate(result["phases"][:3]):
            self.assertEqual(
                [phase["start_targets"][str(sid)] for sid in range(4)], waypoints[index]
            )
            self.assertEqual([phase["targets"][str(sid)] for sid in range(4)], waypoints[index + 1])
            self.assertEqual(set(phase["motion_targets"]), {"0", "1", "2", "3"})
            self.assertEqual(phase["targets"]["4"], self.positions[4])
            self.assertEqual(phase["targets"]["5"], self.positions[5])
            self.assertEqual(phase["duration_ms"], 2000)
            self.assertEqual(phase["profile"], "JOINT_ROUTE_WAYPOINT_MODEL_ONLY")
            self.assertFalse(phase["hardware_interpolation_implemented"])
            self.assertFalse(phase["physical_path_verified"])
            self.assertEqual(result["motion_phases"][index]["targets"], phase["motion_targets"])
        self.assertEqual(result["phases"][3]["declared_start_ms"], 6000)
        self.assertEqual(result["total_declared_ms"], 6000 + 800 + 5 * 7400)
        self.assertNotEqual(motion_phases_sha256(result), legacy_hash)
        self.assertEqual(self.entry, original)
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["hardware_execution_allowed"])

    def test_factory_upright_route_can_be_composed_without_relabeling_physical_evidence(self):
        self.entry["current_positions6_p"][:4] = [1500] * 4
        self.entry["source"] = "SIMULATION"
        self.set_route()
        result = self.compose()
        self.assertEqual(result["status"], "NOMINAL_TASK_PROPOSAL")
        self.assertEqual(
            result["phases"][0]["start_targets"],
            {str(sid): position for sid, position in enumerate([1500] * 4 + self.positions[4:])},
        )
        self.assertEqual(
            result["phases"][-1]["motion_targets"],
            {"0": self.plan["nominal_endpoints_p"]["pick_hover"][0]},
        )
        self.assertFalse(result["physical_angles_verified"])
        self.assertFalse(result["physical_path_verified"])
        self.assertFalse(result["physical_cycles_verified"])

    def test_eight_segment_entry_keeps_finite_phase_count_and_aggregate_time_above_wire_t(self):
        self.set_route(count=8, duration_ms=1500)
        result = self.compose()
        self.assertEqual(result["entry_declared_ms"], 12000)
        self.assertEqual(len(result["phases"]), 49)
        self.assertEqual(len(result["motion_phases"]), 49)
        self.assertEqual(result["phases"][7]["name"], "entry_08")
        self.assertEqual(result["cycle_timing"][0]["declared_start_ms"], 12800)
        self.assertTrue(all(phase["duration_ms"] == 1500 for phase in result["phases"][:8]))
        self.set_route(count=9, duration_ms=1500)
        with self.assertRaises(ValueError):
            self.compose()

    def test_bad_route_chain_endpoint_schema_or_duration_is_rejected(self):
        self.set_route()
        original = deepcopy(self.entry)
        edits = (
            lambda e: e["entry_segments"][0]["from4_p"].__setitem__(0, 1500),
            lambda e: e["entry_segments"][1]["from4_p"].__setitem__(1, 1500),
            lambda e: e["entry_segments"][-1]["to4_p"].__setitem__(1, 1500),
            lambda e: e["entry_segments"][0]["to4_p"].__setitem__(0, 499),
            lambda e: e["entry_segments"][0]["to4_p"].__setitem__(0, True),
            lambda e: e["entry_segments"][0].update(duration_ms=1499),
            lambda e: e["entry_segments"][0].update(duration_ms=9001),
            lambda e: e["entry_segments"][0].update(duration_ms=0),
            lambda e: e["entry_segments"][0].update(extra="not allowed"),
            lambda e: e["entry_segments"][0].update(from4_p=tuple(self.positions[:4])),
            lambda e: e["entry_segments"][0].update(to4_p=list(self.positions[:4])),
            lambda e: e.update(entry_segments=[]),
            lambda e: e.update(entry_segments=None),
            lambda e: e.update(entry_declared_ms=6001, entry_and_transfer_declared_ms=13401),
        )
        for index, edit in enumerate(edits):
            self.entry = deepcopy(original)
            edit(self.entry)
            with self.subTest(index=index), self.assertRaises(ValueError):
                self.compose()

    def test_already_at_start_accepts_only_empty_explicit_route(self):
        current = self.plan["nominal_endpoints_p"]["pick_hover"] + self.positions[4:]
        self.entry = plan_simulated_transfer_entry(self.model, self.plan, current, self.policy)
        self.entry["entry_segments"] = []
        result = self.compose()
        self.assertEqual(result["phases"][0]["name"], "entry")
        self.assertEqual(result["phases"][0]["duration_ms"], 0)
        self.assertEqual(result["motion_phases"][0]["name"], "prepare_gripper")
        self.entry["entry_segments"] = [
            {"from4_p": current[:4], "to4_p": current[:4], "duration_ms": 1500}
        ]
        with self.assertRaises(ValueError):
            self.compose()


if __name__ == "__main__":
    unittest.main()
