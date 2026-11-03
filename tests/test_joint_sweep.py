"""Continuous policy tests use an interpolating memory servo, never hardware."""

from collections import deque
from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import contextlib
import io

from arm_project.arm_link import ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.joint_sweep import ExactSweepTransport, JointSweepController, SweepSpec
from arm_project.servo_protocol import validate_motion_command, validate_read_query
from test_arm_driver import FakeClock


def spec(**changes):
    return SweepSpec.from_dict(
        {
            "review_id": "SIMULATION-CONTINUOUS",
            "whole_path_reviewed_for_motion": True,
            "evidence": "simulation only, not a physical path review",
            "servo_id": 2,
            "min_p": 1480,
            "max_p": 1710,
            "start_min_p": 1690,
            "start_max_p": 1694,
            "target_p": 1503,
            "duration_ms": 8000,
            "max_step_p": 200,
            **changes,
        }
    )


class LinearServo:
    source = "SIMULATION"

    def __init__(self, clock, scenario="normal"):
        self.clock, self.scenario = clock, scenario
        self.pending = deque()
        self.writes, self.motion = [], None
        self.started = None
        self.closed = False

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return sum(len(chunk) for chunk in self.pending)

    def read(self, count):
        self.clock.sleep(0.001)
        if not self.pending:
            return b""
        value = self.pending.popleft()
        if value[count:]:
            self.pending.appendleft(value[count:])
        return value[:count]

    def write(self, command):
        self.writes.append((self.clock.now, command))
        if command.endswith(b"PDST!"):
            self.pending.append(b"#OK!")
            return len(command)
        try:
            axis, field = validate_read_query(command)
        except ValueError:
            self.motion = validate_motion_command(command)
            self.started = self.clock.now
            return 14 if self.scenario == "short_motion" else 15
        if field == "version":
            body = b"@ " + EXPECTED_FIRMWARE.encode("ascii")
        elif field == "mode":
            body = b"MOD5" if self.scenario == "wrong_mode" else b"MOD1"
        else:
            value = 1692 if axis == 2 else 1500
            if self.motion and axis == self.motion[0]:
                goal = self.motion[1] + (6 if self.scenario == "residual" else 0)
                fraction = min(1.0, (self.clock.now - self.started) / (self.motion[2] / 1000))
                value = round(1692 + (goal - 1692) * fraction)
                if self.scenario == "stuck":
                    value = 1692
                elif self.scenario == "reverse":
                    value = 1697
                elif self.scenario == "outside":
                    value = 1750
            body = f"P{value:04d}".encode("ascii")
        self.pending.append(f"#{axis:03d}".encode("ascii") + body + b"!")
        return len(command)


class SweepTests(unittest.TestCase):
    def controller(self, scenario="normal", before=None, hardware=False):
        clock = FakeClock()
        device = LinearServo(clock, scenario)
        value = spec()
        transport = device
        if hardware:
            transport = ExactSweepTransport("NEVER_OPENED", value)
            transport.port = device
            transport.open = device.open
        events = []
        link = ServoLink(transport, clock=clock, record=lambda e, **v: events.append((e, v)))
        return JointSweepController(link, value, before), device, events

    def motions(self, device):
        return [c for _, c in device.writes if len(c) == 15 and b"T" in c]

    def test_200p_8second_move_is_one_frame_with_feedback_during_motion(self):
        claims = []
        controller, device, events = self.controller(before=claims.append, hardware=True)
        with controller.link:
            result = controller.run()
        self.assertTrue(result["response_pass"])
        self.assertTrue(result["position_reached"])
        self.assertEqual(result["final_p"], 1503)
        self.assertEqual(self.motions(device), [b"#002P1503T8000!"])
        self.assertFalse(any(c.endswith(b"PDST!") for _, c in device.writes))
        self.assertGreater(result["samples_during_declared_motion"], 10)
        self.assertLess(result["feedback"][0]["elapsed_s"], 1)
        self.assertTrue(any(1503 < s["value"] < 1692 for s in result["feedback"]))
        self.assertEqual(len(claims), 1)
        self.assertTrue(device.closed)
        with self.assertRaises(RuntimeError):
            controller.run()

    def test_residual_is_response_without_claiming_precise_position(self):
        controller, device, _ = self.controller("residual")
        result = controller.run()
        self.assertEqual(result["status"], "RESPONSE_OBSERVED")
        self.assertTrue(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertEqual(result["error_p"], -6)
        self.assertEqual(len(self.motions(device)), 1)

    def test_failure_stops_only_current_axis_once_and_never_retries(self):
        for scenario in ("stuck", "reverse", "outside", "short_motion"):
            controller, device, _ = self.controller(scenario)
            result = controller.run()
            with self.subTest(scenario=scenario):
                self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
                self.assertFalse(result["response_pass"])
                self.assertTrue(controller.link.faulted)
                self.assertEqual(len(self.motions(device)), 1)
                self.assertEqual(sum(c == b"#002PDST!" for _, c in device.writes), 1)
                self.assertFalse(result["physical_stop_verified"])
                self.assertLess(device.clock.now, 14)

    def test_budget_failure_and_mode_mismatch_do_not_move(self):
        controller, device, _ = self.controller("wrong_mode")
        self.assertEqual(controller.run()["status"], "PREFLIGHT_FAILED")
        self.assertEqual(self.motions(device), [])
        controller, device, _ = self.controller(
            before=lambda _: (_ for _ in ()).throw(FileExistsError("used"))
        )
        self.assertEqual(controller.run()["status"], "PREFLIGHT_FAILED")
        self.assertEqual(self.motions(device), [])

    def test_cancellation_during_continuous_move_has_no_intermediate_target(self):
        controller, device, _ = self.controller()
        device.clock.hook = lambda: (
            controller.cancel() if device.motion and device.clock.now > device.started + 1 else None
        )
        result = controller.run()
        self.assertEqual(result["status"], "CANCELLED")
        self.assertLess(device.clock.now - device.started, 2)
        self.assertEqual(len(self.motions(device)), 1)
        self.assertEqual(sum(c == b"#002PDST!" for _, c in device.writes), 1)

    def test_cancel_file_and_bad_logging_prevent_or_interrupt_motion(self):
        controller, device, _ = self.controller()
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "cancel.request"
            marker.write_text("cancel", encoding="utf-8")
            controller.cancel_file = marker
            self.assertEqual(controller.run()["status"], "CANCELLED")
        self.assertEqual(device.writes, [])
        controller, device, _ = self.controller()

        def bad_log(event, **values):
            if event == "sweep_sample":
                raise OSError("disk failure")

        controller.link.record = bad_log
        result = controller.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(len(self.motions(device)), 1)
        self.assertEqual(sum(c == b"#002PDST!" for _, c in device.writes), 1)

    def test_exact_gate_rejects_changed_target_duration_other_axis_and_config(self):
        controller, device, _ = self.controller(before=lambda _: None, hardware=True)
        transport = controller.link.transport
        for command in (
            b"#002P1502T8000!",
            b"#002P1503T7999!",
            b"#001P1503T8000!",
            b"#002PCSR!",
            b"#255P1503T8000!",
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                transport.write(command)
        self.assertEqual(device.writes, [])
        transport.write(spec().command)
        with self.assertRaises(RuntimeError):
            transport.write(spec().command)
        self.assertEqual(len(self.motions(device)), 1)

    def test_review_bounds_are_explicit_immutable_and_not_old_local_limits(self):
        value = spec()
        self.assertEqual(value.duration_ms, 8000)
        self.assertEqual(value.max_step_p, 200)
        with self.assertRaises(FrozenInstanceError):
            value.target_p = 1500
        for changes in (
            {"whole_path_reviewed_for_motion": False},
            {"whole_path_reviewed_for_motion": 1},
            {"evidence": ""},
            {"max_step_p": 50},
            {"duration_ms": 10000},
            {"sample_interval_s": 0.01},
            {"settling_s": float("nan")},
            {"servo_id": True},
            {"extra": 1},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                spec(**changes)

    def test_cli_dry_run_never_opens_serial(self):
        from arm_project.cli.run_joint_sweep import main

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "spec.json"
            from dataclasses import asdict

            path.write_text(json.dumps(asdict(spec())), encoding="utf-8")
            with (
                patch("sys.argv", ["run_joint_sweep", "--spec", str(path)]),
                patch(
                    "arm_project.arm_link.SerialTransport.open",
                    side_effect=AssertionError("must not open"),
                ),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(), 0)


if __name__ == "__main__":
    unittest.main()
