"""CLI预检与默认不访问硬件；硬件构造器均mock，不连接串口。"""

import contextlib
import io
import json
import unittest
from unittest.mock import patch

from arm_project.cli import arm_cli


class ArmCliTests(unittest.TestCase):
    def test_full_plan_builds_only_explicit_exact_motion_entries(self):
        steps, moves = arm_cli.build_plan(
            {
                "schema_version": 1,
                "steps": [
                    {"kind": "capture_pose", "name": "initial"},
                    {"kind": "move_joint", "servo_id": 4, "target_p": 1540, "duration_ms": 1500},
                    {"kind": "grip", "action": "open"},
                    {"kind": "read_state"},
                ],
            }
        )
        self.assertEqual(len(steps), 4)
        self.assertEqual(
            [(m.servo_id, m.target_p, m.duration_ms) for m in moves],
            [(4, 1540, 1500), (5, 1480, 1000)],
        )

    def test_invalid_plan_and_boundary_types_are_rejected_before_hardware(self):
        cases = [
            {"schema_version": True, "steps": [{"kind": "read_state"}]},
            {"schema_version": 1, "steps": []},
            {"schema_version": 1, "steps": [{"kind": "read_state", "command": "#255P1500T1000!"}]},
            {"schema_version": 1, "steps": [{"kind": "capture_pose", "name": "../escape"}]},
            {"schema_version": 1, "steps": [{"kind": "capture_pose", "name": "same"}] * 2},
            {"schema_version": 1, "steps": [{"kind": "grip", "action": "close", "force": 999}]},
            {
                "schema_version": 1,
                "steps": [
                    {"kind": "move_joint", "servo_id": True, "target_p": 1540, "duration_ms": 2000}
                ],
            },
            {
                "schema_version": 1,
                "steps": [
                    {"kind": "move_joint", "servo_id": 1, "target_p": 1600, "duration_ms": 2000}
                ],
            },
            {
                "schema_version": 1,
                "steps": [
                    {"kind": "move_joint", "servo_id": 4, "target_p": 1540, "duration_ms": 1000}
                ],
            },
            {"schema_version": 1, "steps": [{"kind": "grip", "action": "open"}] * 21},
        ]
        with patch.object(arm_cli, "HardwareArmDriver") as hardware:
            for case in cases:
                with self.subTest(case=case), self.assertRaises((ValueError, TypeError)):
                    arm_cli.build_plan(case)
            hardware.assert_not_called()

    def test_dry_run_validates_without_constructing_driver(self):
        contents = json.dumps({"schema_version": 1, "steps": [{"kind": "grip", "action": "open"}]})
        with (
            patch("pathlib.Path.read_text", return_value=contents),
            patch("sys.argv", ["arm_cli", "--plan", "plan.json"]),
            patch.object(arm_cli, "HardwareArmDriver") as hardware,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(arm_cli.main(), 0)
            hardware.assert_not_called()

    def test_invalid_later_step_prevents_any_driver_creation(self):
        contents = json.dumps(
            {
                "schema_version": 1,
                "steps": [{"kind": "grip", "action": "open"}, {"kind": "unknown"}],
            }
        )
        argv = [
            "arm_cli",
            "--plan",
            "plan.json",
            "--port",
            "NOT_A_DEVICE",
            "--run-id",
            "test",
            "--execute",
        ]
        with (
            patch("pathlib.Path.read_text", return_value=contents),
            patch("sys.argv", argv),
            patch.object(arm_cli, "HardwareArmDriver") as hardware,
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit),
        ):
            arm_cli.main()
        hardware.assert_not_called()

    def test_motion_response_acceptance_does_not_claim_precision_or_grasp(self):
        r = {
            "status": "RESPONSE_OBSERVED",
            "response_pass": True,
            "position_reached": False,
            "object_grasped": False,
            "error": None,
        }
        self.assertTrue(arm_cli.operation_accepted("grip", r))
        self.assertFalse(r["position_reached"])
        self.assertFalse(r["object_grasped"])
        self.assertFalse(arm_cli.operation_accepted("grip", {**r, "error": "serial fault"}))
        self.assertFalse(arm_cli.operation_accepted("read_state", {"status": "INCOMPLETE"}))


if __name__ == "__main__":
    unittest.main()
