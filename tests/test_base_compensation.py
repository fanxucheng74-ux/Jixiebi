"""局部分方向补偿离线验收；仅模拟协议回包和隔离临时文件。"""

import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch
import uuid

from arm_project.cli import accept_base_compensation as acceptance
from arm_project.arm_link import ServoLink
from arm_project.arm_motion import load_limits
from arm_project.base_compensation import CompensatedBaseController, compensated_command
from arm_project.servo_protocol import validate_read_query, validate_motion_command
from test_arm_driver import FakeClock, FakeDevice


ROOT = Path(__file__).resolve().parents[1]


class DirectionalDevice(FakeDevice):
    """绝对目标的简化设备模型；正向少12P、负向多15P，可注入故障。"""

    def __init__(self, clock, start_p, scenario="normal", start_samples=None):
        super().__init__(clock, scenario)
        self.start_p = self.current_p = start_p
        self.motion_origin = None
        self.start_samples = list(start_samples or [])

    def write(self, command):
        if command == b"#000PDST!":
            return super().write(command)
        try:
            servo_id, field = validate_read_query(command)
        except ValueError:
            validate_motion_command(command)
            self.motion_origin = self.current_p
            return super().write(command)
        if servo_id != 0 or field != "position":
            return super().write(command)
        self.writes.append(command)
        self.reads += 1
        if self.motion and self.scenario == "disconnect":
            raise OSError("simulated device disconnected")
        if self.motion and self.scenario == "silent":
            return len(command)
        if not self.motion:
            position = self.start_samples.pop(0) if self.start_samples else self.current_p
        elif self.scenario == "stuck":
            position = self.motion_origin
        elif self.scenario == "wrong_model":
            position = self.motion[1]  # 实际设备没有被假设的方向偏差。
        elif self.scenario == "out_of_window":
            position = 1560
        else:
            target = self.motion[1]
            position = target - 12 if target > self.motion_origin else target + 15
        self.current_p = position
        frame = ("#000P%04d!" % position).encode("ascii")
        self.chunks.extend([frame[:1], frame[1:]])
        return len(command)


class CompensationTests(unittest.TestCase):
    def make_driver(self, start_p=1543, scenario="normal", samples=None, before=None):
        clock = FakeClock()
        device = DirectionalDevice(clock, start_p, scenario, samples)
        events, claims = [], []
        link = ServoLink(
            device, clock=clock, record=lambda event, **values: events.append((event, values))
        )
        driver = CompensatedBaseController(
            link,
            load_limits(ROOT / "configs" / "joint_limits.json"),
            before_motion=claims.append if before is None else before,
        )
        return driver, device, events, claims

    def motions(self, device):
        return [command for command in device.writes if len(command) == 15 and b"T" in command]

    def test_direction_selection_and_interior_edges_stay_in_reviewed_window(self):
        for reference, start, command in (
            (1530, 1543, 1515),
            (1540, 1530, 1552),
            (1520, 1543, 1505),
            (1543, 1520, 1555),
        ):
            with self.subTest(reference=reference, start=start):
                self.assertEqual(compensated_command(reference, start), command)
        self.assertIsNone(compensated_command(1530, 1532))
        for start in (1504, 1556, True, 1530.0, float("nan")):
            with self.subTest(start=start), self.assertRaises(ValueError):
                compensated_command(1530, start)

    def test_invalid_reference_rejected_before_any_io(self):
        for reference in (1519, 1544, True, 1530.0, float("nan"), "1530"):
            driver, device, events, claims = self.make_driver()
            with self.subTest(reference=reference), self.assertRaises(ValueError):
                driver.move_reference(reference)
            self.assertEqual(device.writes, [])
            self.assertEqual(events, [])
            self.assertEqual(claims, [])

    def test_directional_model_reaches_reference_but_preserves_raw_residual(self):
        for start, reference, command, raw_error in (
            (1543, 1530, 1515, -15),
            (1530, 1540, 1552, 12),
        ):
            with self.subTest(start=start, reference=reference):
                driver, device, _, claims = self.make_driver(start)
                result = driver.move_reference(reference)
                raw = result["command_result"]
                self.assertEqual(result["status"], "REFERENCE_REACHED")
                self.assertTrue(result["reference_reached"])
                self.assertEqual(result["reference_error_p"], 0)
                self.assertEqual(result["command_p"], command)
                self.assertEqual(result["reference_errors_p"], [0] * 11)
                self.assertEqual(raw["status"], "OBSERVED_NOT_AT_TARGET")
                self.assertEqual(raw["error_p"], raw_error)
                self.assertFalse(raw["target_reached"])
                self.assertFalse(raw["physical_stop_verified"])
                self.assertFalse(result["physical_accuracy_verified"])
                self.assertEqual(self.motions(device), [("#000P%04dT1500!" % command).encode()])
                self.assertEqual(len(claims), 1)
                self.assertEqual(claims[0]["reference_p"], reference)
                self.assertEqual(claims[0]["offset_p"], command - reference)
                with self.assertRaises(RuntimeError):
                    driver.move_reference(reference)
                self.assertEqual(len(self.motions(device)), 1)

    def test_wrong_model_raw_target_success_does_not_mean_reference_success(self):
        for start, reference in ((1543, 1530), (1530, 1540)):
            with self.subTest(start=start):
                driver, device, _, _ = self.make_driver(start, "wrong_model")
                result = driver.move_reference(reference)
                self.assertEqual(result["command_result"]["status"], "TARGET_REACHED")
                self.assertEqual(result["status"], "REFERENCE_NOT_REACHED")
                self.assertFalse(result["reference_reached"])
                self.assertGreater(abs(result["reference_error_p"]), 3)
                self.assertEqual(len(self.motions(device)), 1)

    def test_stuck_short_write_disconnect_silence_and_outside_feedback_cannot_succeed(self):
        for scenario in ("stuck", "short_motion", "disconnect", "silent", "out_of_window"):
            with self.subTest(scenario=scenario):
                driver, device, _, _ = self.make_driver(scenario=scenario)
                result = driver.move_reference(1530)
                self.assertFalse(result["reference_reached"])
                self.assertNotEqual(result["status"], "REFERENCE_REACHED")
                self.assertEqual(len(self.motions(device)), 1)
                self.assertFalse(result["command_result"]["physical_stop_verified"])
                if scenario != "stuck":
                    self.assertEqual(result["status"], "REFERENCE_ATTEMPT_FAILED")
                    self.assertEqual(device.writes.count(b"#000PDST!"), 1)
                with self.assertRaises(RuntimeError):
                    driver.move_reference(1530)

    def test_bad_initial_state_or_changed_second_preflight_cannot_move(self):
        driver, device, _, claims = self.make_driver(samples=[1543, 1546, 1543])
        with self.assertRaises(ValueError):
            driver.move_reference(1530)
        self.assertEqual(self.motions(device), [])
        self.assertEqual(claims, [])
        driver, device, _, claims = self.make_driver(samples=[1543] * 3 + [1540] * 3)
        result = driver.move_reference(1530)
        self.assertEqual(result["status"], "REFERENCE_ATTEMPT_FAILED")
        self.assertEqual(self.motions(device), [])
        self.assertEqual(claims, [])

    def test_already_within_tolerance_requires_all_three_samples_without_moving(self):
        for values, reached in (([1531, 1531, 1531], True), ([1525, 1526, 1527], False)):
            with self.subTest(values=values):
                driver, device, _, claims = self.make_driver(values[0], samples=values)
                result = driver.move_reference(1530)
                self.assertEqual(result["reference_reached"], reached)
                expected = (
                    "REFERENCE_ALREADY_WITHIN_TOLERANCE" if reached else "REFERENCE_NOT_REACHED"
                )
                self.assertEqual(result["status"], expected)
                self.assertEqual(self.motions(device), [])
                self.assertEqual(claims, [])

    def test_motion_logging_failure_cannot_be_reference_success(self):
        driver, device, _, _ = self.make_driver()

        def log(event, **values):
            if event == "motion_result":
                raise OSError("simulated result logging failure")

        driver.link.record = log
        result = driver.move_reference(1530)
        self.assertFalse(result["reference_reached"])
        self.assertNotEqual(result["status"], "REFERENCE_REACHED")
        self.assertTrue(result["command_result"]["logging_errors"])
        self.assertEqual(len(self.motions(device)), 1)


class CompensationBudgetTests(unittest.TestCase):
    def setUp(self):
        self.experiment = ROOT / "tests" / (".compensation-test-" + uuid.uuid4().hex)
        self.experiment.mkdir()
        self.addCleanup(self.cleanup_experiment)

    def cleanup_experiment(self):
        target = self.experiment.resolve()
        if target.parent != (ROOT / "tests").resolve() or not target.name.startswith(
            ".compensation-test-"
        ):
            raise RuntimeError("unexpected test directory; refusing cleanup")
        shutil.rmtree(target)

    def plan(self, phase):
        reference, command, _ = acceptance.STEPS[phase]
        return {
            "servo_id": 0,
            "reference_p": reference,
            "start_p": 1543,
            "target_p": command,
            "duration_ms": 1500,
        }

    def clean_previous(self):
        return {
            "error": None,
            "logging_errors": [],
            "transport_closed": True,
            "final_state": {
                "status": "OK",
                "servos": [{"servo_id": 0, "status": "OK", "position_p": 1530}],
            },
            "reference": {
                "status": "REFERENCE_REACHED",
                "reference_reached": True,
                "reference_p": 1530,
                "command_result": {"logging_errors": []},
            },
        }

    def write_previous(self, summary):
        directory = self.experiment / "negative_1"
        directory.mkdir(exist_ok=True)
        (directory / "result.json").write_text(json.dumps(summary), encoding="utf-8")

    def test_three_fixed_steps_have_exclusive_markers_and_duplicates_cannot_move(self):
        for phase in acceptance.STEPS:
            with self.subTest(phase=phase):
                acceptance.reserve_step(phase, self.plan(phase), self.experiment)
                marker = self.experiment / (phase + "_attempt.json")
                original = marker.read_bytes()
                with self.assertRaises(FileExistsError):
                    acceptance.reserve_step(phase, self.plan(phase), self.experiment)
                self.assertEqual(marker.read_bytes(), original)
        clock = FakeClock()
        device = DirectionalDevice(clock, 1543)
        link = ServoLink(device, clock=clock)
        driver = CompensatedBaseController(
            link,
            load_limits(ROOT / "configs" / "joint_limits.json"),
            before_motion=lambda plan: acceptance.reserve_step("negative_1", plan, self.experiment),
        )
        result = driver.move_reference(1530)
        self.assertEqual(result["status"], "REFERENCE_ATTEMPT_FAILED")
        self.assertFalse(result["reference_reached"])
        self.assertEqual([w for w in device.writes if len(w) == 15], [])

    def test_wrong_id_reference_command_or_duration_cannot_reserve(self):
        for key, value in (
            ("servo_id", 1),
            ("reference_p", 1540),
            ("target_p", 1516),
            ("duration_ms", 3000),
        ):
            with self.subTest(key=key):
                plan = self.plan("negative_1")
                plan[key] = value
                with self.assertRaises(ValueError):
                    acceptance.reserve_step("negative_1", plan, self.experiment)
                self.assertEqual(list(self.experiment.iterdir()), [])

    def test_previous_failed_reference_or_session_does_not_advance(self):
        self.write_previous(self.clean_previous())
        acceptance.require_previous("positive_1", self.experiment)
        for kind in (
            "reference_status",
            "reference_flag",
            "motion_log",
            "session_error",
            "session_log",
            "open_port",
            "incomplete_state",
        ):
            with self.subTest(kind=kind):
                data = self.clean_previous()
                if kind == "reference_status":
                    data["reference"]["status"] = "REFERENCE_NOT_REACHED"
                elif kind == "reference_flag":
                    data["reference"]["reference_reached"] = False
                elif kind == "motion_log":
                    data["reference"]["command_result"]["logging_errors"] = ["disk full"]
                elif kind == "session_error":
                    data["error"] = "serial failure"
                elif kind == "session_log":
                    data["logging_errors"] = ["log failure"]
                elif kind == "open_port":
                    data["transport_closed"] = False
                else:
                    data["final_state"]["status"] = "INCOMPLETE"
                self.write_previous(data)
                with self.assertRaises(ValueError):
                    acceptance.require_previous("positive_1", self.experiment)

    def test_final_snapshot_drift_prevents_acceptance_and_next_phase(self):
        for position, allowed in ((1527, True), (1533, True), (1526, False), (1534, False)):
            with self.subTest(position=position):
                data = self.clean_previous()
                data["final_state"]["servos"][0]["position_p"] = position
                self.assertEqual(acceptance.final_base_p(data), position)
                self.assertEqual(acceptance.acceptance_ok(data), allowed)
                self.write_previous(data)
                if allowed:
                    acceptance.require_previous("positive_1", self.experiment)
                else:
                    with self.assertRaises(ValueError):
                        acceptance.require_previous("positive_1", self.experiment)

    def test_missing_duplicate_failed_or_noninteger_base_is_not_accepted(self):
        invalid_rows = [
            [],
            [{"servo_id": 1, "status": "OK", "position_p": 1530}],
            [{"servo_id": 0, "status": "READ_FAILED", "position_p": 1530}],
            [{"servo_id": 0, "status": "OK", "position_p": 1530}] * 2,
        ]
        invalid_rows.extend(
            [
                [{"servo_id": 0, "status": "OK", "position_p": value}]
                for value in (None, True, 1530.0, "1530", float("nan"), 1504, 1556)
            ]
        )
        for rows in invalid_rows:
            with self.subTest(rows=rows):
                data = self.clean_previous()
                data["final_state"]["servos"] = rows
                with self.assertRaises(ValueError):
                    acceptance.final_base_p(data)
                self.assertFalse(acceptance.acceptance_ok(data))
                self.write_previous(data)
                with self.assertRaises(ValueError):
                    acceptance.require_previous("positive_1", self.experiment)

    def test_first_step_checks_isolated_diagnostic_result_not_real_hardware_logs(self):
        path = self.experiment / "runs/base_residual_20261031/positive_slow/result.json"
        path.parent.mkdir(parents=True)
        data = {
            "error": None,
            "logging_errors": [],
            "transport_closed": True,
            "final_state": {
                "status": "OK",
                "servos": [{"servo_id": 0, "status": "OK", "position_p": 1543}],
            },
            "motion": {
                "status": "OBSERVED_NOT_AT_TARGET",
                "motion_write_complete": True,
                "logging_errors": [],
                "tail_span_p": 0,
            },
        }
        path.write_text(json.dumps(data), encoding="utf-8")
        with patch.object(acceptance, "ROOT", self.experiment):
            acceptance.require_previous("negative_1", self.experiment)
            data["motion"]["motion_write_complete"] = False
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ValueError):
                acceptance.require_previous("negative_1", self.experiment)


if __name__ == "__main__":
    unittest.main()
