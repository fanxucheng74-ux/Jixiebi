"""统一有限任务的独立审阅、底座运动与异常门禁；不访问串口或相机。"""

from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import unittest

from arm_project.arm_link import ServoLink
from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_entry import plan_simulated_transfer_entry
from arm_project.transfer_task_motion import (
    REVIEW_SCHEMA,
    TaskPhase,
    TaskTransport,
    TransferTaskController,
    TransferTaskSpec,
    compile_motion_phases,
    motion_phases_sha256,
    validate_task_review,
)
from arm_project.transfer_task_plan import compose_transfer_task
from test_arm_driver import FakeClock
from test_timed_cycle import Device


def phase(name, targets, duration_ms=1500, tolerance=20):
    return {
        "name": name,
        "duration_ms": duration_ms,
        "dwell_ms": 0,
        "targets": [
            {"servo_id": sid, "target_p": target, "end_tolerance_p": tolerance}
            for sid, target in targets.items()
        ],
    }


def review_payload():
    value = {
        "schema": REVIEW_SCHEMA,
        "review_id": "SIM-TRANSFER-TASK",
        "whole_path_reviewed_for_motion": True,
        "evidence": "simulation fixture only; no real path evidence",
        "start_intervals_p": [[1498, 1502] for _ in range(6)],
        "observation_bounds_p": [[1000, 2100] for _ in range(6)],
        "deadline_ms": 12000,
        "geometry_mm": {
            "base_height_mm": 100,
            "upper_arm_mm": 105,
            "forearm_mm": 75,
            "tool_mm": 180,
        },
        "table_z_mm": 0,
        "phases": [
            phase("entry", {0: 1520}),
            phase("prepare_gripper", {5: 1570}, 1000),
            phase("cycle_01_rotate_left", {0: 1500}),
        ],
    }
    value["motion_phases_sha256"] = motion_phases_sha256(value["phases"])
    return value


def composed_payload(review=None):
    review = review or review_payload()
    return {
        "schema": "jixiebi.transfer_task.v1",
        "status": "NOMINAL_TASK_PROPOSAL",
        "entry_status": "NOMINAL_ENTRY_PROPOSAL",
        "motion_authorized": False,
        "hardware_execution_allowed": False,
        "task_id": "SIM-NOMINAL-TASK",
        "model_geometry_mm": deepcopy(review["geometry_mm"]),
        "motion_phases": [
            {
                "name": row["name"],
                "duration_ms": row["duration_ms"],
                "targets": {str(t["servo_id"]): t["target_p"] for t in row["targets"]},
            }
            for row in review["phases"]
        ],
    }


class TransferTaskMotionTests(unittest.TestCase):
    def controller(
        self, *, scenario="normal", reserve=None, record=None, review=None, state_validator=None
    ):
        clock = FakeClock()
        raw = Device(clock, scenario)
        spec = TransferTaskSpec.from_dict(review or review_payload())
        gate = TaskTransport("NEVER_OPENED", spec, raw=raw)
        link = ServoLink(gate, clock=clock, record=record)
        controller = TransferTaskController(
            link, spec, reserve or (lambda value: None), state_validator=state_validator
        )
        return controller, raw

    @staticmethod
    def motions(raw):
        return [command for command in raw.writes if len(command) == 15 and b"T" in command]

    def test_exact_review_and_hash_bind_every_phase_without_authorizing_nominal_plan(self):
        review = review_payload()
        task = composed_payload(review)
        before = deepcopy(task)
        spec = validate_task_review(task, review)
        self.assertEqual(spec.motion_phases_sha256, motion_phases_sha256(task))
        self.assertEqual(task, before)
        self.assertFalse(task["motion_authorized"])
        self.assertEqual(spec, TransferTaskSpec.from_dict(spec.to_dict()))
        self.assertEqual(spec.phases[1].continue_tracking_p, ((0, 20),))
        self.assertEqual(spec.phases[2].continue_tracking_p, ((5, 20),))

    def test_hash_excludes_task_name_freshness_and_unsent_axis_noise(self):
        task = composed_payload()
        original = motion_phases_sha256(task)
        task.update(task_id="ANOTHER-TASK", entry_freshness={"age_ms": 10})
        task["current_positions6_p"] = [1500, 1500, 1500, 1500, 1537, 1492]
        self.assertEqual(motion_phases_sha256(task), original)
        task["motion_phases"][0]["targets"]["0"] += 1
        self.assertNotEqual(motion_phases_sha256(task), original)

    def test_review_missing_false_or_not_matching_composed_motion_is_rejected(self):
        for change in ("unreviewed", "hash", "duration", "target", "unused", "order"):
            with self.subTest(change=change):
                review = review_payload()
                task = composed_payload(review)
                if change == "unreviewed":
                    review["whole_path_reviewed_for_motion"] = False
                elif change == "hash":
                    review["motion_phases_sha256"] = "0" * 64
                elif change == "duration":
                    task["motion_phases"][0]["duration_ms"] += 1
                elif change == "target":
                    task["motion_phases"][0]["targets"]["0"] += 1
                elif change == "unused":
                    review["phases"].append(phase("unused", {0: 1500}))
                    review["motion_phases_sha256"] = motion_phases_sha256(review["phases"])
                else:
                    task["motion_phases"].reverse()
                with self.assertRaises(ValueError):
                    validate_task_review(task, review)
        review = review_payload()
        del review["evidence"]
        with self.assertRaises(ValueError):
            validate_task_review(composed_payload(), review)

    def test_blocked_or_already_authorized_payload_does_not_supply_motion_authorization(self):
        for key, value in (
            ("status", "BLOCKED"),
            ("entry_status", "REQUIRES_REVIEWED_LIFT"),
            ("motion_authorized", True),
            ("hardware_execution_allowed", True),
        ):
            task = composed_payload()
            task[key] = value
            with self.assertRaises(ValueError):
                compile_motion_phases(task)

    def test_base_and_four_body_axes_allowed_but_gripper_must_be_separate(self):
        value = phase("entry", {0: 1520, 1: 1400, 2: 1500, 3: 1500})
        parsed = TaskPhase.from_dict(value)
        self.assertEqual([target[0] for target in parsed.targets], [0, 1, 2, 3])
        self.assertEqual(parsed.commands[0], b"#000P1520T1500!")
        for targets in ({0: 1520, 5: 1570}, {0: 1520, 1: 1400, 2: 1500, 3: 1500, 4: 1500}):
            with self.assertRaises(ValueError):
                TaskPhase.from_dict(phase("bad", targets))

    def test_body_and_gripper_time_floors_remain_enforced(self):
        for sid, accepted, rejected in ((0, 1500, 1499), (3, 1500, 500), (5, 1000, 999)):
            TaskPhase.from_dict(phase("ok", {sid: 1520}, accepted))
            with self.assertRaises(ValueError):
                TaskPhase.from_dict(phase("bad", {sid: 1520}, rejected))

    def test_finite_phase_frame_and_deadline_caps(self):
        for kind in ("phases", "frames", "deadline", "feedback_slack"):
            review = review_payload()
            review["deadline_ms"] = 180000
            if kind == "phases":
                review["phases"] = [phase(f"phase_{n}", {0: 1500}) for n in range(51)]
            elif kind == "frames":
                review["phases"] = [
                    phase(f"phase_{n}", {0: 1500, 1: 1500, 2: 1500, 3: 1500}) for n in range(46)
                ]
            elif kind == "deadline":
                review["deadline_ms"] += 1
            else:
                review["deadline_ms"] = 4999
            review["motion_phases_sha256"] = motion_phases_sha256(review["phases"])
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                TransferTaskSpec.from_dict(review)

    def test_only_explicit_zero_table_and_complete_start_envelope_are_supported(self):
        for key, value in (
            ("table_z_mm", 1),
            ("table_z_mm", float("nan")),
            ("table_z_mm", False),
            ("start_intervals_p", [[1498, 1502]] * 5),
            ("start_intervals_p", [[499, 1502]] * 6),
            ("observation_bounds_p", [[1600, 1700]] * 6),
        ):
            review = review_payload()
            review[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                TransferTaskSpec.from_dict(review)

    def test_unknown_fields_duplicate_ids_and_unsafe_numbers_are_rejected(self):
        for change in ("field", "duplicate", "bool", "target", "tolerance"):
            value = phase("entry", {0: 1520})
            if change == "field":
                value["raw_command"] = "#000P1520T1500!"
            elif change == "duplicate":
                value["targets"].append(dict(value["targets"][0]))
            elif change == "bool":
                value["targets"][0]["servo_id"] = False
            elif change == "target":
                value["targets"][0]["target_p"] = 2501
            else:
                value["targets"][0]["end_tolerance_p"] = 81
            with self.subTest(change=change), self.assertRaises(ValueError):
                TaskPhase.from_dict(value)

    def test_continuation_cannot_invent_a_goal_or_loosen_prior_corridor(self):
        for sid, tolerance in (("1", 20), ("0", 21)):
            review = review_payload()
            review["phases"][1]["continue_tracking_p"] = {sid: tolerance}
            with self.assertRaises(ValueError):
                TransferTaskSpec.from_dict(review)
        review = review_payload()
        review["phases"][1]["continue_tracking_p"] = {"0": 10}
        spec = TransferTaskSpec.from_dict(review)
        self.assertEqual(spec.phases[1].continue_tracking_p, ((0, 10),))

    def test_review_geometry_must_match_nominal_model_and_export_is_independent(self):
        review = review_payload()
        task = composed_payload(review)
        task["model_geometry_mm"]["tool_mm"] += 1
        with self.assertRaisesRegex(ValueError, "geometry"):
            validate_task_review(task, review)
        spec = TransferTaskSpec.from_dict(review)
        exported = spec.to_dict()
        exported["geometry_mm"]["tool_mm"] += 1
        self.assertEqual(spec.geometry.tool_mm, 180)

    def test_held_base_requires_same_prior_goal_and_original_tracking_corridor(self):
        for target, tolerance in ((1521, 20), (1520, 21)):
            review = review_payload()
            review["phases"][2] = phase("cycle_01_rotate_left", {0: target}, tolerance=tolerance)
            review["phases"][2]["hold_tracking_p"] = {"0": 20}
            review["motion_phases_sha256"] = motion_phases_sha256(review["phases"])
            with self.assertRaises(ValueError):
                TransferTaskSpec.from_dict(review)

    def test_direct_dataclass_construction_cannot_bypass_validation(self):
        spec = TransferTaskSpec.from_dict(review_payload())
        with self.assertRaises(ValueError):
            TaskTransport("NEVER_OPENED", replace(spec, deadline_ms=180001))

    def test_exact_ordered_six_state_rejects_missing_duplicate_wrong_id_or_invalid_status(self):
        state = {
            "status": "OK",
            "servos": [{"servo_id": sid, "status": "OK", "position_p": 1500} for sid in range(6)],
        }
        self.assertEqual(TransferTaskController.positions(state), [1500] * 6)
        for change in ("missing", "duplicate", "reversed", "invalid_status", "bool"):
            bad = deepcopy(state)
            if change == "missing":
                bad["servos"].pop()
            elif change == "duplicate":
                bad["servos"][1]["servo_id"] = 0
            elif change == "reversed":
                bad["servos"].reverse()
            elif change == "invalid_status":
                bad["servos"][2]["status"] = "NOT_READ"
            else:
                bad["servos"][0]["position_p"] = True
            with self.subTest(change=change), self.assertRaises(ValueError):
                TransferTaskController.positions(bad)

    def test_simulated_base_task_reserves_once_and_retains_separate_physical_flags(self):
        claims = []
        controller, raw = self.controller(reserve=claims.append)
        with controller.link:
            result = controller.run()
        self.assertTrue(result["response_pass"], result["error"])
        self.assertTrue(result["within_deadline"])
        self.assertEqual(len(claims), 1)
        self.assertEqual(
            self.motions(raw),
            [b"#000P1520T1500!", b"#005P1570T1000!", b"#000P1500T1500!"],
        )
        self.assertFalse(result["physical_task_verified"])
        self.assertFalse(result["physical_stop_verified"])
        self.assertFalse(result["strict_position_verified"])
        self.assertTrue(raw.closed)
        with self.assertRaises(RuntimeError):
            controller.run()

    def test_composed_right_entry_and_all_five_complete_cycles_run_under_one_budget(self):
        root = Path(__file__).resolve().parents[1]
        model = json.loads((root / "configs/kinematics.arduino_nominal.json").read_text("utf-8"))
        request = json.loads(
            (root / "configs/fixed_transfer_nominal_example.json").read_text("utf-8")
        )
        policy = json.loads(
            (root / "configs/transfer_entry_policy.example.json").read_text("utf-8")
        )
        # 原快速例子仍只供预览；本测试显式使用现有执行器允许的臂/爪声明时长。
        request["stage_times_ms"] = {
            name: 1000 if name in {"open", "close"} else 1500 for name in request["stage_times_ms"]
        }
        transfer = plan_fixed_transfer(model, request)
        current = transfer["nominal_endpoints_p"]["place_hover"] + [1534, 1218]
        entry = plan_simulated_transfer_entry(model, transfer, current, policy)
        task = compose_transfer_task(
            transfer, entry, {"open_p": 1200, "close_p": 1570, "duration_ms": 1000}, "SIM-FIVE"
        )
        review = review_payload()
        review.update(
            start_intervals_p=[[value - 2, value + 2] for value in current],
            observation_bounds_p=[[500, 2500] for _ in range(6)],
            deadline_ms=70000,
            phases=[
                phase(
                    item["name"],
                    {int(sid): target for sid, target in item["targets"].items()},
                    item["duration_ms"],
                )
                for item in task["motion_phases"]
            ],
        )
        review["motion_phases_sha256"] = motion_phases_sha256(task)
        spec = validate_task_review(task, review)
        claims = []
        controller, raw = self.controller(reserve=claims.append, review=spec.to_dict())
        raw.initial = current
        with controller.link:
            result = controller.run()
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(len(result["phases"]), 42)
        self.assertEqual(len(claims), 1)
        self.assertEqual(result["phases"][-1]["name"], "cycle_05_rotate_left")
        self.assertEqual(len(self.motions(raw)), sum(len(p.targets) for p in spec.phases))
        self.assertEqual(
            [servo["position_p"] for servo in result["final_state"]["servos"][:4]],
            transfer["nominal_endpoints_p"]["pick_hover"],
        )
        self.assertFalse(result["physical_task_verified"])

    def test_cancel_during_motion_tx_logging_stops_before_write_and_preserves_one_shot_stops(self):
        controller = None

        def record(event, **values):
            if event == "motion_tx" and values["command"].startswith("#005"):
                controller.cancel()

        controller, raw = self.controller(record=record)
        with controller.link:
            result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(self.motions(raw), [b"#000P1520T1500!"])
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)
        self.assertEqual(raw.writes.count(b"#005PDST!"), 0)
        self.assertEqual(controller.link.transport.command_index, 0)

    def test_failed_budget_sends_no_motion_and_short_write_never_retries_or_advances(self):
        def fail_reservation(value):
            raise OSError("exclusive persistent budget unavailable")

        controller, raw = self.controller(reserve=fail_reservation)
        result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(self.motions(raw), [])
        controller, raw = self.controller(scenario="short_second")
        result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(len(self.motions(raw)), 2)
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)
        self.assertEqual(raw.writes.count(b"#005PDST!"), 1)

    def test_stuck_or_opposite_base_cancels_later_gripper_and_return(self):
        for scenario in ("stuck", "opposite"):
            controller, raw = self.controller(scenario=scenario)
            result = controller.run()
            self.assertFalse(result["response_pass"])
            self.assertEqual(self.motions(raw), [b"#000P1520T1500!"])
            self.assertEqual(raw.writes.count(b"#000PDST!"), 1)

    def test_custom_scene_rejects_initial_state_before_consuming_budget(self):
        claims = []
        controller, raw = self.controller(reserve=claims.append, state_validator=lambda _: False)
        with controller.link:
            result = controller.run()
        self.assertEqual(result["status"], "PREFLIGHT_FAILED")
        self.assertIn("configured nominal scene", result["error"])
        self.assertEqual(claims, [])
        self.assertEqual(self.motions(raw), [])
        self.assertFalse(controller.link.transport.armed)

    def test_custom_scene_failure_during_motion_stops_once_before_later_phases(self):
        calls = []
        raw = None

        def state_validator(positions):
            calls.append(positions)
            return not bool(raw.moves)

        controller, raw = self.controller(state_validator=state_validator)
        with controller.link:
            result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        self.assertIn("configured nominal scene", result["error"])
        self.assertEqual(self.motions(raw), [b"#000P1520T1500!"])
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)
        self.assertEqual(raw.writes.count(b"#005PDST!"), 0)
        self.assertEqual(result["phases"][-1]["name"], "entry")
        self.assertGreaterEqual(len(calls), 3)

    def test_custom_scene_requires_exact_bool_result_and_callable_interface(self):
        for value in (None, 0, 1, "true", {"valid": True}):
            claims = []
            controller, raw = self.controller(
                reserve=claims.append, state_validator=lambda _, answer=value: answer
            )
            with self.subTest(value=value), controller.link:
                result = controller.run()
            self.assertFalse(result["response_pass"])
            self.assertEqual(claims, [])
            self.assertEqual(self.motions(raw), [])
            self.assertIn("configured nominal scene", result["error"])
        with self.assertRaisesRegex(ValueError, "callable"):
            self.controller(state_validator=True)

    def test_scene_hook_does_not_bypass_direction_or_command_envelope_guards(self):
        controller, raw = self.controller(scenario="opposite", state_validator=lambda _: True)
        with controller.link:
            result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertIn("opposite", result["error"])
        self.assertEqual(self.motions(raw), [b"#000P1520T1500!"])
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)

    def test_scene_callback_checks_initial_phase_sample_handoff_and_final_feedback(self):
        root = Path(__file__).resolve().parents[1]
        model = json.loads((root / "configs/kinematics.arduino_nominal.json").read_text("utf-8"))
        config = json.loads((root / "configs/motion_scene.example.json").read_text("utf-8"))
        scene = SceneValidator(model, config)
        checked, sample_events = [], []

        def state_validator(positions):
            checked.append(positions)
            return scene(positions)

        def record(event, **values):
            if event == "cycle_sample":
                positions = tuple(servo["position_p"] for servo in values["state"]["servos"][:4])
                self.assertEqual(checked[-1], positions)
                sample_events.append(positions)

        controller, _ = self.controller(record=record, state_validator=state_validator)
        with controller.link:
            result = controller.run()
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(len(checked), 2 + 2 * len(result["phases"]) + len(sample_events))
        self.assertEqual(checked[0], (1500, 1500, 1500, 1500))
        self.assertEqual(
            checked[-1], tuple(servo["position_p"] for servo in result["final_state"]["servos"][:4])
        )
        self.assertTrue(all(type(value) is tuple and len(value) == 4 for value in checked))
        self.assertGreater(len(sample_events), 0)

    def test_scene_table_rejection_is_applied_before_budget_or_motion(self):
        root = Path(__file__).resolve().parents[1]
        model = json.loads((root / "configs/kinematics.arduino_nominal.json").read_text("utf-8"))
        config = json.loads((root / "configs/motion_scene.example.json").read_text("utf-8"))
        scene = SceneValidator(model, config)
        current = [1500, 834, 1800, 1500, 1500, 1500]
        review = review_payload()
        review["start_intervals_p"] = [[value - 2, value + 2] for value in current]
        review["observation_bounds_p"] = [[500, 2500] for _ in range(6)]
        checks, claims = [], []

        def state_validator(positions):
            checks.append(scene.inspect(positions))
            return checks[-1]["valid"]

        controller, raw = self.controller(
            review=review, reserve=claims.append, state_validator=state_validator
        )
        raw.initial = current
        with controller.link:
            result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(checks[0]["code"], "TABLE_CLEARANCE")
        self.assertEqual(claims, [])
        self.assertEqual(self.motions(raw), [])

    def test_default_geometry_guard_remains_and_explicit_scene_handles_other_valid_pose(self):
        root = Path(__file__).resolve().parents[1]
        model = json.loads((root / "configs/kinematics.arduino_nominal.json").read_text("utf-8"))
        config = json.loads((root / "configs/motion_scene.example.json").read_text("utf-8"))
        scene = SceneValidator(model, config)
        current = [1500, 1550, 1500, 1500, 1500, 1500]
        self.assertTrue(scene(current[:4]))
        review = review_payload()
        review["start_intervals_p"] = [[value - 2, value + 2] for value in current]
        controller, raw = self.controller(review=review)
        raw.initial = current
        with controller.link:
            default = controller.run()
        self.assertFalse(default["response_pass"])
        self.assertIn("folds behind", default["error"])
        self.assertEqual(self.motions(raw), [])
        controller, raw = self.controller(review=review, state_validator=scene)
        raw.initial = current
        with controller.link:
            configured = controller.run()
        self.assertTrue(configured["response_pass"], configured["error"])
        self.assertEqual(len(self.motions(raw)), 3)


if __name__ == "__main__":
    unittest.main()
