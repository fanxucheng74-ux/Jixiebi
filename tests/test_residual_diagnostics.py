"""残差诊断离线验收：仅内存设备和临时文件，不连接真实串口。"""

import json
from pathlib import Path
import shutil
import unittest
import uuid

from arm_project.arm_link import ServoLink
from arm_project.arm_motion import MotionController, load_limits
from arm_project.cli.diagnose_base_residual import STEPS, check_previous, claim_step
from test_arm_driver import FakeClock, FakeDevice


ROOT = Path(__file__).resolve().parents[1]


class ResidualObservationTests(unittest.TestCase):
    def controller(self, scenario="normal", before_motion=None):
        clock = FakeClock()
        device = FakeDevice(clock, scenario)
        events = []
        link = ServoLink(
            device, clock=clock, record=lambda event, **values: events.append((event, values))
        )
        controller = MotionController(
            link, load_limits(ROOT / "configs" / "joint_limits.json"), before_motion=before_motion
        )
        return controller, device, events

    def motion_writes(self, device):
        return [command for command in device.writes if len(command) == 15 and b"T" in command]

    def test_extended_observation_has_eleven_spaced_samples(self):
        controller, device, _ = self.controller()
        result = controller.move_joint(0, 1550, feedback_samples=11, feedback_interval_s=0.5)
        self.assertEqual(result["status"], "TARGET_REACHED")
        self.assertEqual(result["feedback_samples_requested"], 11)
        self.assertEqual(result["feedback_interval_s"], 0.5)
        self.assertEqual(len(result["feedback"]), 11)
        times = [sample["received_monotonic_s"] for sample in result["feedback"]]
        for earlier, later in zip(times, times[1:]):
            self.assertGreaterEqual(later - earlier, 0.5 - 1e-9)
            self.assertLess(later - earlier, 0.51)
        self.assertGreaterEqual(result["observation_span_s"], 5.0)
        self.assertLess(result["observation_span_s"], 5.1)
        self.assertEqual(self.motion_writes(device), [b"#000P1550T1500!"])

    def test_invalid_observation_policy_rejected_without_io_or_budget_claim(self):
        invalid = [
            {"feedback_samples": 4},
            {"feedback_samples": 62},
            {"feedback_samples": True},
            {"feedback_samples": 11.0},
            {"feedback_samples": float("nan")},
            {"feedback_interval_s": 0.09},
            {"feedback_interval_s": 1.01},
            {"feedback_interval_s": True},
            {"feedback_interval_s": False},
            {"feedback_interval_s": "0.5"},
            {"feedback_interval_s": float("nan")},
            {"feedback_interval_s": float("inf")},
            {"feedback_interval_s": float("-inf")},
            {"feedback_samples": 61, "feedback_interval_s": 0.51},
        ]
        for parameters in invalid:
            with self.subTest(parameters=parameters):
                claims = []
                controller, device, events = self.controller(before_motion=claims.append)
                with self.assertRaises(ValueError):
                    controller.move_joint(0, 1550, **parameters)
                self.assertEqual(device.writes, [])
                self.assertEqual(device.reads, 0)
                self.assertEqual(events, [])
                self.assertEqual(claims, [])

    def test_stable_twelve_p_residual_is_not_success_or_compensated(self):
        controller, device, _ = self.controller("undershoot")
        result = controller.move_joint(0, 1550, feedback_samples=11, feedback_interval_s=0.5)
        self.assertEqual(result["status"], "OBSERVED_NOT_AT_TARGET")
        self.assertEqual([sample["value"] for sample in result["feedback"]], [1538] * 11)
        self.assertEqual(result["error_p"], 12)
        self.assertEqual(result["tail_span_p"], 0)
        self.assertTrue(result["stable_residual_observed"])
        self.assertTrue(result["directed_feedback_change"])
        self.assertFalse(result["target_reached"])
        self.assertFalse(result["physical_stop_verified"])
        self.assertIsNone(result["stop_attempt"])
        self.assertEqual(self.motion_writes(device), [b"#000P1550T1500!"])

    def test_write_count_and_raw_command_logged_for_full_and_short_write(self):
        for scenario, expected_written in (("normal", 15), ("short_motion", 14)):
            with self.subTest(scenario=scenario):
                controller, device, events = self.controller(scenario)
                result = controller.move_joint(0, 1550)
                transmissions = [values for event, values in events if event == "motion_tx"]
                returns = [values for event, values in events if event == "motion_write_returned"]
                self.assertEqual(len(transmissions), 1)
                self.assertEqual(len(returns), 1)
                self.assertEqual(bytes.fromhex(transmissions[0]["tx_hex"]), b"#000P1550T1500!")
                self.assertEqual(returns[0]["expected_bytes"], 15)
                self.assertEqual(returns[0]["bytes_written"], expected_written)
                self.assertGreaterEqual(returns[0]["monotonic_s"], transmissions[0]["monotonic_s"])
                self.assertEqual(result["motion_write_complete"], expected_written == 15)
                self.assertEqual(len(self.motion_writes(device)), 1)
                if expected_written != 15:
                    self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
                    with self.assertRaises(RuntimeError):
                        controller.move_joint(0, 1550)


class DiagnosticBudgetTests(unittest.TestCase):
    def setUp(self):
        # 项目内隔离临时目录，避免依赖系统Temp的额外权限。
        self.experiment = ROOT / "tests" / (".residual-test-" + uuid.uuid4().hex)
        self.experiment.mkdir()
        self.addCleanup(self.cleanup_experiment)

    def cleanup_experiment(self):
        # 删除前校验实际路径，清理范围仅限本测试创建的目录。
        target = self.experiment.resolve()
        if target.parent != (ROOT / "tests").resolve() or not target.name.startswith(
            ".residual-test-"
        ):
            raise RuntimeError("unexpected test directory; refusing cleanup")
        shutil.rmtree(target)

    def write_previous(self, phase, summary):
        directory = self.experiment / STEPS[phase][2]
        directory.mkdir(exist_ok=True)
        (directory / "result.json").write_text(json.dumps(summary), encoding="utf-8")

    def clean_observation(self):
        return {
            "error": None,
            "transport_closed": True,
            "logging_errors": [],
            "final_state": {"status": "OK"},
            "motion": None,
            "observation": [{"value": 1538} for _ in range(11)],
        }

    def clean_motion(self):
        return {
            "error": None,
            "transport_closed": True,
            "logging_errors": [],
            "final_state": {"status": "OK"},
            "motion": {
                "status": "OBSERVED_NOT_AT_TARGET",
                "motion_write_complete": True,
                "logging_errors": [],
                "directed_feedback_change": True,
                "tail_span_p": 0,
                "feedback": [{"value": 1520} for _ in range(11)],
            },
        }

    def plan(self, phase):
        target, duration, _ = STEPS[phase]
        return {
            "servo_id": 0,
            "start_p": 1512,
            "target_p": target,
            "duration_ms": duration,
            "command": "#000P%04dT%04d!" % (target, duration),
        }

    def test_observe_bootstrap_and_clean_stable_observation_allowed(self):
        check_previous("observe", self.experiment)
        self.assertEqual(list(self.experiment.iterdir()), [])
        self.write_previous("negative_fast", self.clean_observation())
        check_previous("negative_fast", self.experiment)

    def test_missing_short_or_unstable_initial_observation_blocks_motion(self):
        for values in ([], [1538] * 10, [1538] * 10 + [1541]):
            with self.subTest(values=values):
                summary = self.clean_observation()
                summary["observation"] = [{"value": value} for value in values]
                self.write_previous("negative_fast", summary)
                with self.assertRaises(ValueError):
                    check_previous("negative_fast", self.experiment)

    def test_clean_observed_or_reached_motion_allows_next_phase(self):
        for status in ("OBSERVED_NOT_AT_TARGET", "TARGET_REACHED"):
            summary = self.clean_motion()
            summary["motion"]["status"] = status
            self.write_previous("positive_fast", summary)
            check_previous("positive_fast", self.experiment)

    def test_previous_motion_fault_or_incomplete_write_blocks_next_phase(self):
        changes = [
            ("status", "FAILED_AFTER_ATTEMPT"),
            ("status", "PREFLIGHT_FAILED"),
            ("status", "CANCELLED_STOP_UNVERIFIED"),
            ("status", "NO_MOTION_NEEDED"),
            ("motion_write_complete", False),
            ("logging_errors", ["disk full"]),
            ("directed_feedback_change", False),
            ("feedback", [{"value": 1520}] * 10),
            ("tail_span_p", 3),
        ]
        for key, value in changes:
            with self.subTest(key=key, value=value):
                summary = self.clean_motion()
                summary["motion"][key] = value
                self.write_previous("positive_fast", summary)
                with self.assertRaises(ValueError):
                    check_previous("positive_fast", self.experiment)

    def test_phase_level_faults_block_both_observation_and_motion_successors(self):
        changes = [
            ("error", "serial timeout"),
            ("transport_closed", False),
            ("logging_errors", ["close log failed"]),
            ("final_state", {"status": "INCOMPLETE"}),
        ]
        for phase, factory in (
            ("negative_fast", self.clean_observation),
            ("positive_fast", self.clean_motion),
        ):
            for key, value in changes:
                with self.subTest(phase=phase, key=key):
                    summary = factory()
                    summary[key] = value
                    self.write_previous(phase, summary)
                    with self.assertRaises(ValueError):
                        check_previous(phase, self.experiment)

    def test_missing_or_corrupt_previous_result_cannot_advance(self):
        with self.assertRaises(FileNotFoundError):
            check_previous("positive_fast", self.experiment)
        directory = self.experiment / "negative_fast"
        directory.mkdir()
        (directory / "result.json").write_text("{incomplete", encoding="utf-8")
        with self.assertRaises(ValueError):
            check_previous("positive_fast", self.experiment)
        self.write_previous("positive_fast", self.clean_observation())
        with self.assertRaises(ValueError):
            check_previous("positive_fast", self.experiment)

    def test_each_fixed_step_claim_is_persistent_and_cannot_be_replaced(self):
        for phase in STEPS:
            with self.subTest(phase=phase):
                plan = self.plan(phase)
                claim_step(phase, plan, self.experiment)
                marker = self.experiment / (phase + "_attempt.json")
                before = marker.read_bytes()
                self.assertEqual(json.loads(before)["plan"], plan)
                self.assertTrue(json.loads(before)["time"])
                with self.assertRaises(FileExistsError):
                    claim_step(phase, plan, self.experiment)
                self.assertEqual(marker.read_bytes(), before)

    def test_mismatched_id_target_or_duration_cannot_create_marker(self):
        for key, value in (("servo_id", 1), ("target_p", 1550), ("duration_ms", 3000)):
            with self.subTest(key=key):
                plan = self.plan("negative_fast")
                plan[key] = value
                with self.assertRaises(ValueError):
                    claim_step("negative_fast", plan, self.experiment)
                self.assertFalse((self.experiment / "negative_fast_attempt.json").exists())


if __name__ == "__main__":
    unittest.main()
