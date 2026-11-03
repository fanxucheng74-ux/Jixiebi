"""恢复预检与普通路径复核必须共用时间/检查预算；均为离线名义模型。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.general_transfer_entry import plan_simulated_general_entry
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import validate_task_review
from arm_project.transfer_task_plan import compose_transfer_task
from arm_project.transfer_task_policy import (
    EVIDENCE_PREFIX,
    build_policy_review,
    task_scene_validator,
)
from test_table_recovery_integration import SOFT_START, task_policy


ROOT = Path(__file__).resolve().parents[1]


def config(name):
    return json.loads((ROOT / "configs" / name).read_text(encoding="utf-8"))


class RecoveryPolicyBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = config("kinematics.arduino_nominal.json")
        cls.scene = config("motion_scene.example.json")
        cls.planner = config("general_entry_planner.example.json")
        transfer = plan_fixed_transfer(cls.model, config("transfer_task_request.example.json"))
        entry_policy = config("transfer_entry_policy.example.json")
        cls.tasks = {}
        for name, start in (("soft", SOFT_START), ("normal", [1500] * 4)):
            entry = plan_simulated_general_entry(
                cls.model,
                transfer,
                start + [1500, 1200],
                entry_policy,
                cls.scene,
                cls.planner,
            )
            if entry["status"] != "NOMINAL_ENTRY_PROPOSAL":
                raise AssertionError(entry)
            cls.tasks[name] = compose_transfer_task(
                transfer,
                entry,
                {"open_p": 1200, "close_p": 1570, "duration_ms": 1000},
                "SIM-recovery-budget-" + name,
            )

    def build(self, name="soft"):
        task = self.tasks[name]
        return build_policy_review(
            task,
            self.model,
            self.scene,
            self.planner,
            task_policy(self.model, self.scene, self.planner, task),
        )

    def test_slow_model_inspections_stop_inside_recovery_helper_at_shared_deadline(self):
        elapsed = 0.0
        checks = 0
        helper_completed = False
        original_inspect = SceneValidator.inspect

        def slow_inspect(scene, positions4_p, *, minimum_clearance_mm=None):
            nonlocal elapsed, checks
            result = original_inspect(
                scene, positions4_p, minimum_clearance_mm=minimum_clearance_mm
            )
            checks += 1
            elapsed += 0.05
            return result

        def record_helper(*args, **kwargs):
            nonlocal helper_completed
            result = task_scene_validator(*args, **kwargs)
            helper_completed = True
            return result

        with (
            patch("arm_project.transfer_task_policy.time.monotonic", side_effect=lambda: elapsed),
            patch.object(SceneValidator, "inspect", new=slow_inspect),
            patch(
                "arm_project.transfer_task_policy.task_scene_validator", side_effect=record_helper
            ),
            patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")),
        ):
            with self.assertRaisesRegex(ValueError, "3-second budget"):
                self.build()
        self.assertFalse(helper_completed)
        self.assertGreaterEqual(elapsed, 3)
        self.assertLessEqual(elapsed, 3.05 + 1e-9)
        self.assertLessEqual(checks, 61)

    def test_one_inspection_cap_is_consumed_inside_helper_before_any_motion_or_outer_loop(self):
        calls = []
        helper_completed = False
        original_inspect = SceneValidator.inspect

        def record_inspect(scene, positions4_p, *, minimum_clearance_mm=None):
            calls.append((list(positions4_p), minimum_clearance_mm))
            return original_inspect(scene, positions4_p, minimum_clearance_mm=minimum_clearance_mm)

        def record_helper(*args, **kwargs):
            nonlocal helper_completed
            result = task_scene_validator(*args, **kwargs)
            helper_completed = True
            return result

        with (
            patch("arm_project.transfer_task_policy._MAX_VALIDITY_CHECKS", 1),
            patch.object(SceneValidator, "inspect", new=record_inspect),
            patch(
                "arm_project.transfer_task_policy.task_scene_validator", side_effect=record_helper
            ),
            patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")),
        ):
            with self.assertRaisesRegex(ValueError, "scene-check budget"):
                self.build()
        self.assertEqual(calls, [(SOFT_START, None)])
        self.assertFalse(helper_completed)

    def test_successful_normal_and_recovery_reviews_count_every_actual_inspection_once(self):
        original_inspect = SceneValidator.inspect
        before = deepcopy((self.tasks, self.model, self.scene, self.planner))
        for name in ("normal", "soft"):
            calls = 0

            def record_inspect(scene, positions4_p, *, minimum_clearance_mm=None):
                nonlocal calls
                calls += 1
                return original_inspect(
                    scene, positions4_p, minimum_clearance_mm=minimum_clearance_mm
                )

            with (
                self.subTest(start=name),
                patch.object(SceneValidator, "inspect", new=record_inspect),
                patch(
                    "arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")
                ),
            ):
                review = self.build(name)
                validate_task_review(self.tasks[name], review)
                evidence = json.loads(review["evidence"][len(EVIDENCE_PREFIX) :])
                self.assertEqual(evidence["sampled_scene_checks"], calls)
                self.assertGreater(calls, 1000)
                self.assertEqual(evidence["table_recovery"] is not None, name == "soft")
                self.assertFalse(evidence["physical_observation_evidence"])
                self.assertFalse(evidence["physical_geometry_verified"])
                self.assertFalse(evidence["physical_path_verified"])
                self.assertFalse(evidence["asynchronous_motion_envelope_verified"])
        self.assertEqual((self.tasks, self.model, self.scene, self.planner), before)


if __name__ == "__main__":
    unittest.main()
