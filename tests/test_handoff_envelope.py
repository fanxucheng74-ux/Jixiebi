"""交接误差盒覆盖连续角区间；测试不读取私有日志，也不连接硬件。"""

from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.handoff_envelope import (
    HandoffEnvelopeError,
    check_handoff_envelopes,
    require_handoff_envelopes,
)
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import (
    REVIEW_SCHEMA,
    TransferTaskSpec,
    motion_phases_sha256,
)


ROOT = Path(__file__).resolve().parents[1]


def config(name):
    return json.loads((ROOT / "configs" / name).read_text(encoding="utf-8"))


def phase(name, targets, *, tolerance=3, holds=None, continuing=None):
    value = {
        "name": name,
        "duration_ms": 1000 if set(targets) == {5} else 1500,
        "dwell_ms": 0,
        "targets": [
            {"servo_id": sid, "target_p": target, "end_tolerance_p": tolerance}
            for sid, target in targets.items()
        ],
    }
    if holds is not None:
        value["hold_tracking_p"] = {str(sid): value for sid, value in holds.items()}
    if continuing is not None:
        value["continue_tracking_p"] = {str(sid): value for sid, value in continuing.items()}
    return value


def spec_and_scene(phases=None, *, starts=None, observation=None, base_height=None):
    model = config("kinematics.arduino_nominal.json")
    if base_height is not None:
        model["geometry_mm"]["base_height_mm"] = base_height
    scene = SceneValidator(model, config("motion_scene.example.json"))
    phases = phases or [phase("prepare_gripper", {5: 1500})]
    value = {
        "schema": REVIEW_SCHEMA,
        "review_id": "SIM-HANDOFF-ENVELOPE",
        "whole_path_reviewed_for_motion": True,
        "evidence": "Synthetic offline fixture, no physical motion or measurement",
        "start_intervals_p": starts or [[1498, 1502] for _ in range(6)],
        "observation_bounds_p": observation or [[500, 2500] for _ in range(6)],
        "deadline_ms": 180000,
        "geometry_mm": model["geometry_mm"],
        "table_z_mm": 0,
        "phases": phases,
        "motion_phases_sha256": motion_phases_sha256(phases),
    }
    return TransferTaskSpec.from_dict(value), scene


class HandoffEnvelopeTests(unittest.TestCase):
    def test_recorded_error_combination_passes_individual_tolerances_but_fails_scene(self):
        # 数值摘自已结束任务的反馈，仅作离线回归；不把模型毫米数当实测间隙。
        spec, scene = spec_and_scene()
        target = [1833, 1331, 2163, 1221]
        observed = [1820, 1320, 2170, 1200]
        self.assertTrue(all(abs(a - b) <= t for a, b, t in zip(target, observed, (20, 20, 35, 35))))
        self.assertAlmostEqual(scene.inspect(target)["minimum_table_clearance_mm"], 8.4405376501)
        rejected = scene.inspect(observed)
        self.assertEqual(rejected["code"], "TABLE_CLEARANCE")
        self.assertEqual(rejected["link_index"], 2)
        self.assertAlmostEqual(rejected["minimum_table_clearance_mm"], -3.1679075571)
        self.assertFalse(rejected["physical_geometry_verified"])
        self.assertEqual(spec.table_z_mm, 0)

    def test_required_check_preserves_report_and_never_narrows_a_rejected_spec(self):
        row = phase("cycle_01_descend_left", {0: 1833, 1: 1331, 2: 2163, 3: 1221}, tolerance=35)
        spec, scene = spec_and_scene([row])
        before = spec.to_dict()
        with self.assertRaises(HandoffEnvelopeError) as failure:
            require_handoff_envelopes(spec, scene)
        self.assertIn("phase=cycle_01_descend_left", str(failure.exception))
        self.assertIn("required_mm=5", str(failure.exception))
        self.assertFalse(failure.exception.report["valid"])
        self.assertEqual(spec.to_dict(), before)

    def test_required_check_returns_only_actual_success_without_mutating_spec(self):
        spec, scene = spec_and_scene()
        before = spec.to_dict()
        checked = require_handoff_envelopes(spec, scene)
        self.assertTrue(checked["valid"])
        self.assertEqual(checked["scope"], "WORK_PHASE_HANDOFF_TABLE_ONLY")
        self.assertFalse(checked["asynchronous_motion_envelope_verified"])
        self.assertEqual(spec.to_dict(), before)

    def test_current_profile_allowances_have_a_real_table_counterexample(self):
        row = phase("cycle_01_descend_left", {0: 1833, 1: 1331, 2: 2163, 3: 1221})
        for target in row["targets"]:
            target["end_tolerance_p"] = {0: 20, 1: 20, 2: 35, 3: 35}[target["servo_id"]]
        spec, scene = spec_and_scene([row])
        result = check_handoff_envelopes(spec, scene)
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], "COUNTEREXAMPLE")
        counterexample = result["failure"]["counterexample"]
        self.assertLess(counterexample["minimum_table_clearance_mm"], 5)
        self.assertFalse(counterexample["physical_collision_verified"])
        checked = scene.inspect(counterexample["positions4_p"])
        self.assertAlmostEqual(
            checked["minimum_table_clearance_mm"],
            counterexample["minimum_table_clearance_mm"],
        )

    def test_three_p_profile_is_mathematically_compatible_without_changing_spec(self):
        row = phase("cycle_01_descend_left", {0: 1833, 1: 1331, 2: 2163, 3: 1221})
        spec, scene = spec_and_scene([row])
        before = deepcopy(spec.to_dict())
        result = check_handoff_envelopes(spec, scene)
        self.assertTrue(result["valid"])
        self.assertGreater(result["phases"][0]["certified_table_clearance_lower_bound_mm"], 5)
        self.assertEqual(spec.to_dict(), before)
        self.assertFalse(result["asynchronous_motion_envelope_verified"])
        self.assertFalse(result["other_scene_constraints_verified"])

    def test_interior_sine_extremum_can_fail_while_both_box_corners_pass_table(self):
        # 工具俯仰区间跨过-90度；正弦最低点在内部，不能拿两个端点当证明。
        starts = [[1500, 1500], [834, 834], [2167, 2167], [1500, 1500]] + [[1500, 1500]] * 2
        observation = deepcopy(starts)
        observation[3] = [1420, 1580]
        spec, scene = spec_and_scene(
            [phase("work_wrist", {3: 1500}, tolerance=80)],
            starts=starts,
            observation=observation,
            base_height=262,
        )
        for wrist in (1420, 1580):
            check = scene.inspect([1500, 834, 2167, wrist])
            self.assertGreater(check["minimum_table_clearance_mm"], 5)
        result = check_handoff_envelopes(spec, scene)
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], "COUNTEREXAMPLE")
        self.assertEqual(result["failure"]["counterexample"]["positions4_p"][3], 1500)

    def test_held_continuing_and_untargeted_axes_retain_exact_interval_meanings(self):
        spec, scene = spec_and_scene(
            [
                phase("entry_01", {1: 1500}, tolerance=20),
                phase("work_01", {3: 1500}, tolerance=10, continuing={1: 5}),
                phase("work_02", {3: 1500}, tolerance=3, holds={3: 3}),
            ]
        )
        result = check_handoff_envelopes(spec, scene)
        self.assertTrue(result["valid"])
        rows = result["phases"]
        self.assertEqual(rows[0]["status"], "SKIPPED_ENTRY_RECOVERY_SCOPE")
        self.assertEqual(rows[1]["pulse_windows6_p"][1], [1495, 1505])
        self.assertEqual(rows[1]["pulse_windows6_p"][2], [1498, 1502])
        self.assertEqual(rows[1]["pulse_windows6_p"][3], [1490, 1510])
        self.assertEqual(rows[2]["pulse_windows6_p"][1], [1480, 1520])
        self.assertEqual(rows[2]["pulse_windows6_p"][3], [1497, 1503])

    def test_ranges_clip_to_existing_observation_bounds(self):
        observation = [[1495, 1505] for _ in range(6)]
        spec, scene = spec_and_scene(
            [phase("work", {1: 1500}, tolerance=20)], observation=observation
        )
        result = check_handoff_envelopes(spec, scene)
        self.assertTrue(result["valid"])
        self.assertEqual(result["phases"][0]["pulse_windows6_p"][1], [1495, 1505])

    def test_repeated_body_windows_reuse_results_without_more_scene_checks(self):
        spec, scene = spec_and_scene(
            [phase("prepare", {5: 1500}), phase("work_01", {5: 1500}), phase("work_02", {5: 1500})]
        )
        result = check_handoff_envelopes(spec, scene)
        self.assertTrue(result["valid"])
        self.assertEqual(result["check_counts"]["cache_hits"], 2)
        self.assertEqual(result["check_counts"]["unique_envelopes"], 1)
        self.assertEqual(result["check_counts"]["interval_boxes"], 1)

    def test_recovery_entry_is_outside_work_only_certification_scope(self):
        spec, scene = spec_and_scene([phase("entry_01", {1: 834, 2: 2167, 3: 1500}, tolerance=80)])
        result = check_handoff_envelopes(spec, scene)
        self.assertTrue(result["valid"])
        self.assertEqual(result["scope"], "WORK_PHASE_HANDOFF_TABLE_ONLY")
        self.assertIsNone(result["phases"][0]["valid"])
        self.assertEqual(result["check_counts"]["interval_boxes"], 0)

    def test_invalid_types_unvalidated_dataclass_and_geometry_mismatch_are_rejected(self):
        spec, scene = spec_and_scene()
        for invalid_spec, invalid_scene in (({}, scene), (spec, {})):
            with self.assertRaises(ValueError):
                check_handoff_envelopes(invalid_spec, invalid_scene)
        with self.assertRaisesRegex(ValueError, "check_budget"):
            check_handoff_envelopes(spec, scene, check_budget=True)
        with self.assertRaises(ValueError):
            check_handoff_envelopes(replace(spec, deadline_ms=1), scene)
        _, different_scene = spec_and_scene(base_height=101)
        with self.assertRaisesRegex(ValueError, "same geometry"):
            check_handoff_envelopes(spec, different_scene)

    def test_work_phase_count_is_explicitly_bounded(self):
        spec, scene = spec_and_scene([phase("work_%02d" % i, {5: 1500}) for i in range(42)])
        with self.assertRaisesRegex(ValueError, "41 non-entry"):
            check_handoff_envelopes(spec, scene)

    def test_per_envelope_budget_exhaustion_is_not_proven_not_fake_collision(self):
        spec, scene = spec_and_scene()
        with (
            patch("arm_project.handoff_envelope._table_bounds", return_value=(4.0,) * 3),
            patch("arm_project.handoff_envelope._inspect_counterexample", return_value=None),
            patch("arm_project.handoff_envelope._MAX_BOXES_PER_ENVELOPE", 2),
        ):
            result = check_handoff_envelopes(spec, scene)
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], "NOT_PROVEN")
        self.assertEqual(result["check_counts"]["interval_boxes"], 2)
        self.assertIsNone(result["failure"]["counterexample"])

    def test_whole_function_budget_counts_boxes_and_actual_scene_inspections(self):
        spec, scene = spec_and_scene([phase("work", {1: 834, 2: 2167, 3: 1500})])
        with patch("arm_project.handoff_envelope._MAX_CHECKS_TOTAL", 1):
            result = check_handoff_envelopes(spec, scene)
        self.assertFalse(result["valid"])
        self.assertEqual(result["status"], "NOT_PROVEN")
        self.assertEqual(result["check_counts"]["checks_total"], 1)

    def test_callback_runs_immediately_after_scene_inspection_and_errors_propagate(self):
        spec, scene = spec_and_scene([phase("work", {1: 834, 2: 2167, 3: 1500})])
        inspections = 0
        original = scene.inspect

        def inspect(*args, **kwargs):
            nonlocal inspections
            inspections += 1
            return original(*args, **kwargs)

        def budget():
            if inspections:
                raise TimeoutError("shared execution budget")

        with patch.object(scene, "inspect", side_effect=inspect):
            with self.assertRaisesRegex(TimeoutError, "shared execution budget"):
                check_handoff_envelopes(spec, scene, check_budget=budget)
        self.assertEqual(inspections, 1)

    def test_certification_does_not_open_serial(self):
        spec, scene = spec_and_scene()
        with patch("serial.Serial", side_effect=AssertionError("hardware IO")):
            self.assertTrue(check_handoff_envelopes(spec, scene)["valid"])


if __name__ == "__main__":
    unittest.main()
