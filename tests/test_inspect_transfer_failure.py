"""失败检查只读小型构造日志；不依赖用户实验、不打开设备。"""

from copy import deepcopy
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.inspect_transfer_failure import inspect_failure, main


ROOT = Path(__file__).resolve().parents[1]
ERROR = "ValueError: feedback rejected by the task's configured nominal scene"
START6 = [1820, 1338, 1899, 934, 1504, 1233]
SAMPLE6 = [1820, 1320, 2102, 1135, 1504, 1220]
END6 = [1820, 1320, 2170, 1200, 1504, 1220]


def state(values):
    return {
        "status": "OK",
        "simultaneous": False,
        "servos": [
            {"servo_id": sid, "position_p": value, "received_monotonic_s": sid * 0.05}
            for sid, value in enumerate(values)
        ],
    }


class InspectTransferFailureTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.folder = self.root / "fixture"
        self.folder.mkdir()
        self.output = self.root / "inspection"
        self.snapshot = {
            "model": json.loads(
                (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
            ),
            "scene_config": json.loads(
                (ROOT / "configs/motion_scene.example.json").read_text(encoding="utf-8")
            ),
        }
        self.phase = {
            "name": "cycle_01_descend_left",
            "starts_p": list(START6),
            "response_pass": False,
            "commands_sent": ["#001P1331T1500!", "#002P2163T1500!", "#003P1221T1500!"],
            "continuing_goals": {"0": {"target_p": 1833}, "5": {"target_p": 1200}},
            "end_active": [
                {
                    "servo_id": sid,
                    "target_p": target,
                    "feedback_p": END6[sid],
                    "in_handoff_corridor": True,
                    "direction_response": True,
                }
                for sid, target in ((1, 1331), (2, 2163), (3, 1221))
            ],
        }
        self.events = [
            {"event": "cycle_phase_started", "name": "cycle_01_descend_left"},
            {"event": "arm_state", "state": state(SAMPLE6)},
            {"event": "cycle_sample", "phase": "cycle_01_descend_left", "state": state(SAMPLE6)},
            *[
                {
                    "event": "reply",
                    "servo_id": sid,
                    "field": "position",
                    "value": END6[sid],
                    "received_monotonic_s": 10 + sid * 0.01,
                }
                for sid in (1, 2, 3)
            ],
            {"event": "cycle_stop_attempt", "servo_id": 1},
        ]

    def write_fixture(self):
        values = {
            "result.json": {
                "source": "SIMULATED_FIXTURE",
                "status": "FAILED_AFTER_ATTEMPT",
                "error": ERROR,
                "sequence": {"phases": [self.phase]},
            },
            "input_snapshot.json": self.snapshot,
            "task_plan.json": {"task_id": "fixture_only", "motion_authorized": False},
        }
        for name, value in values.items():
            (self.folder / name).write_text(json.dumps(value), encoding="utf-8")
        (self.folder / "events.jsonl").write_text(
            "".join(json.dumps(e) + "\n" for e in self.events), encoding="utf-8"
        )

    def test_endpoint_reconstruction_does_not_mistake_last_successful_sample_for_failure(self):
        self.write_fixture()
        before = {p.name: p.read_bytes() for p in self.folder.iterdir()}
        report = inspect_failure(self.folder)
        self.assertEqual(report["reconstruction"]["status"], "RECONSTRUCTED_ENDPOINT_GUARD")
        self.assertEqual(report["reconstruction"]["positions6_p"], END6)
        self.assertEqual(report["recomputed_static_scene"]["code"], "TABLE_CLEARANCE")
        self.assertEqual(report["recomputed_static_scene"]["link_index"], 2)
        self.assertAlmostEqual(
            report["recomputed_static_scene"]["minimum_table_clearance_mm"], -3.1679075571
        )
        self.assertTrue(report["target_static_scene"]["valid"])
        self.assertAlmostEqual(report["active_reply_host_receive_span_ms"], 20)
        self.assertEqual(
            [row["feedback_minus_target_p"] for row in report["active_axis_errors_p"]],
            [-11, 7, -21],
        )
        self.assertFalse(report["hardware_io"])
        self.assertFalse(report["simultaneous"])
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.folder.iterdir()})

    def test_incomplete_or_unmatched_endpoint_replies_are_uncertain(self):
        original_phase, original_events = deepcopy(self.phase), deepcopy(self.events)
        for change in ("partial", "missing_reply", "changed_reply", "missing_start"):
            with self.subTest(change=change):
                self.phase, self.events = deepcopy(original_phase), deepcopy(original_events)
                if change == "partial":
                    self.phase["end_active"].pop()
                elif change == "missing_reply":
                    self.events.pop(5)
                elif change == "changed_reply":
                    self.events[5]["value"] += 1
                else:
                    self.events.pop(0)
                self.write_fixture()
                report = inspect_failure(self.folder)
                self.assertEqual(report["reconstruction"]["status"], "UNCERTAIN")
                self.assertIsNone(report["recomputed_static_scene"])

    def test_snapshot_not_current_workspace_scene_controls_recalculation(self):
        self.snapshot["scene_config"]["table_z_mm"] = 2
        self.write_fixture()
        report = inspect_failure(self.folder)
        self.assertAlmostEqual(
            report["recomputed_static_scene"]["minimum_table_clearance_mm"], -5.1679075571
        )

    def test_explicit_rejection_event_takes_precedence_and_preserves_saved_check(self):
        self.events.append(
            {
                "event": "task_scene_rejected",
                "phase_name": self.phase["name"],
                "positions6_p": END6,
                "scene_check": {"code": "TABLE_CLEARANCE", "valid": False},
            }
        )
        self.phase["end_active"] = []
        self.write_fixture()
        report = inspect_failure(self.folder)
        reconstruction = report["reconstruction"]
        self.assertEqual(reconstruction["status"], "EXPLICIT_REJECTION_EVENT")
        self.assertEqual(reconstruction["saved_scene_check"]["code"], "TABLE_CLEARANCE")
        self.assertEqual(reconstruction["positions6_p"], END6)

    def test_unaccepted_sample_is_reconstructed_but_a_successful_sample_is_not(self):
        self.phase["end_active"] = []
        self.events = self.events[:2]
        self.events[1]["state"] = state(END6)
        self.write_fixture()
        self.assertEqual(
            inspect_failure(self.folder)["reconstruction"]["status"],
            "RECONSTRUCTED_SAMPLE_GUARD",
        )
        self.events.append({"event": "cycle_sample", "phase": self.phase["name"]})
        self.write_fixture()
        self.assertEqual(inspect_failure(self.folder)["reconstruction"]["status"], "UNCERTAIN")

    def test_cli_uses_new_output_and_refuses_original_or_existing_directory(self):
        self.write_fixture()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--run-dir", str(self.folder), "--output", str(self.output)]), 0)
        saved = (self.output / "report.json").read_bytes()
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(["--run-dir", str(self.folder), "--output", str(self.output)]), 1)
            self.assertEqual(
                main(["--run-dir", str(self.folder), "--output", str(self.folder / "new")]), 1
            )
        self.assertEqual(saved, (self.output / "report.json").read_bytes())
        self.assertFalse((self.folder / "new").exists())

    def test_non_scene_or_postcompletion_failure_is_not_given_a_guessed_pose(self):
        self.phase["response_pass"] = True
        self.write_fixture()
        report = inspect_failure(self.folder)
        self.assertEqual(report["reconstruction"]["status"], "UNCERTAIN")

    def test_invalid_endpoint_rows_are_uncertain_and_invalid_json_root_is_rejected(self):
        self.phase["end_active"] = [None, None, None]
        self.write_fixture()
        report = inspect_failure(self.folder)
        self.assertEqual(report["reconstruction"]["status"], "UNCERTAIN")
        (self.folder / "input_snapshot.json").write_text("[]", encoding="utf-8")
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(["--run-dir", str(self.folder), "--output", str(self.output)]), 1)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
