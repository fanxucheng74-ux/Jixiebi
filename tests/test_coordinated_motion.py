"""Group motion tests use only in-memory interpolated servos and fake clocks."""

from collections import deque
from dataclasses import asdict
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arm_project.arm_link import ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.coordinated_motion import (
    CoordinatedController,
    CoordinatedSpec,
    CoordinatedTransport,
)
from arm_project.joint_sweep import SweepSpec
from arm_project.servo_protocol import validate_motion_command, validate_read_query
from test_arm_driver import FakeClock


def spec():
    axes = []
    for sid, target in ((1, 1400), (2, 1900), (3, 1200)):
        axes.append(
            SweepSpec(
                f"SIM-GROUP-{sid}",
                True,
                "simulation only; no real physical path review",
                sid,
                1000,
                2000,
                1498,
                1502,
                target,
                8000,
                600,
            )
        )
    return CoordinatedSpec("SIM-GROUP", True, "simulation group swept-path fixture", tuple(axes))


class GroupDevice:
    source = "SIMULATION"

    def __init__(self, clock, scenario="normal"):
        self.clock, self.scenario = clock, scenario
        self.pending = deque()
        self.writes = []
        self.moves = {}
        self.closed = False

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return sum(len(c) for c in self.pending)

    def read(self, n):
        self.clock.sleep(0.001)
        if not self.pending:
            return b""
        c = self.pending.popleft()
        if c[n:]:
            self.pending.appendleft(c[n:])
        return c[:n]

    def write(self, c):
        self.writes.append((self.clock.now, c))
        if c.endswith(b"PDST!"):
            return len(c)
        try:
            sid, field = validate_read_query(c)
        except ValueError:
            sid, target, duration = validate_motion_command(c)
            self.moves[sid] = (target, duration, self.clock.now)
            return 14 if self.scenario == "partial_second" and sid == 2 else len(c)
        if field == "version":
            body = b"@ " + EXPECTED_FIRMWARE.encode("ascii")
        elif field == "mode":
            body = b"MOD5" if self.scenario == "wrong_mode" else b"MOD1"
        else:
            value = 1500
            if sid in self.moves:
                target, duration, start = self.moves[sid]
                fraction = min(1, (self.clock.now - start) / (duration / 1000))
                goal = target + (7 if self.scenario == "residual" else 0)
                value = round(1500 + (goal - 1500) * fraction)
                if self.scenario == "tail_jitter6" and fraction >= 1:
                    value += 3 if int((self.clock.now - start) * 2) % 2 else -3
                if self.scenario == "stuck_two" and sid == 2:
                    value = 1500
                elif self.scenario == "opposite" and sid == 1:
                    value = 1505
            if self.moves and self.scenario == "held_drift" and sid == 5:
                value = 1520
            body = f"P{value:04d}".encode("ascii")
        self.pending.append(f"#{sid:03d}".encode("ascii") + body + b"!")
        return len(c)


class GroupTests(unittest.TestCase):
    def controller(self, scenario="normal", before=None):
        clock = FakeClock()
        device = GroupDevice(clock, scenario)
        value = spec()
        gate = CoordinatedTransport("NEVER_OPENED", value, raw=device)
        events = []
        link = ServoLink(gate, clock=clock, record=lambda e, **v: events.append((e, v)))
        return CoordinatedController(link, value, before), device, events

    def motions(self, d):
        return [c for _, c in d.writes if len(c) == 15 and b"T" in c]

    def stops(self, d):
        return [c for _, c in d.writes if c.endswith(b"PDST!")]

    def test_three_targets_start_before_observation_and_are_not_intermediate_steps(self):
        claims = []
        c, d, events = self.controller(before=claims.append)
        with c.link:
            r = c.run()
        self.assertEqual(r["status"], "TARGET_REACHED")
        self.assertTrue(r["response_pass"])
        self.assertTrue(r["position_reached"])
        self.assertEqual(len(self.motions(d)), 3)
        self.assertEqual(self.stops(d), [])
        self.assertEqual(len(claims), 1)
        self.assertEqual(len(claims[0]["commands"]), 3)
        self.assertLess(r["host_write_skew_ms"], 1)
        self.assertTrue(all(row["state"]["status"] == "OK" for row in r["samples"]))
        self.assertTrue(
            any(1500 < row["state"]["servos"][2]["position_p"] < 1900 for row in r["samples"])
        )
        self.assertTrue(d.closed)
        with self.assertRaises(RuntimeError):
            c.run()

    def test_partial_second_motion_stops_attempted_axes_only_and_no_third_write(self):
        c, d, _ = self.controller("partial_second")
        r = c.run()
        self.assertEqual(r["status"], "FAILED_AFTER_ATTEMPT")
        self.assertEqual(len(self.motions(d)), 2)
        self.assertEqual(set(self.stops(d)), {b"#001PDST!", b"#002PDST!"})
        self.assertFalse(r["physical_stop_verified"])
        with self.assertRaises(RuntimeError):
            c.link.transport.write(spec().axes[2].command)

    def test_stuck_reverse_and_held_axis_drift_cancel_the_entire_active_group(self):
        for scenario in ("stuck_two", "opposite", "held_drift"):
            c, d, _ = self.controller(scenario)
            r = c.run()
            with self.subTest(scenario=scenario):
                self.assertEqual(r["status"], "FAILED_AFTER_ATTEMPT")
                self.assertFalse(r["response_pass"])
                self.assertEqual(len(self.motions(d)), 3)
                self.assertEqual(set(self.stops(d)), {b"#001PDST!", b"#002PDST!", b"#003PDST!"})
                self.assertTrue(c.link.faulted)

    def test_residual_does_not_become_precise_group_success(self):
        c, d, _ = self.controller("residual")
        r = c.run()
        self.assertTrue(r["response_pass"])
        self.assertFalse(r["position_reached"])
        self.assertEqual(r["status"], "RESPONSE_OBSERVED")
        self.assertEqual(self.stops(d), [])

    def test_budget_failure_and_mode_failure_never_start_group(self):
        for scenario, before in (
            ("wrong_mode", None),
            ("normal", lambda _: (_ for _ in ()).throw(FileExistsError("used"))),
        ):
            c, d, _ = self.controller(scenario, before)
            r = c.run()
            self.assertEqual(r["status"], "PREFLIGHT_FAILED")
            self.assertEqual(self.motions(d), [])
            self.assertEqual(self.stops(d), [])

    def test_gate_requires_reservation_exact_order_and_consumes_frames(self):
        c, d, _ = self.controller()
        gate = c.link.transport
        value = spec()
        with self.assertRaises(RuntimeError):
            gate.write(value.axes[0].command)
        gate.arm_after_reservation()
        for command in (
            value.axes[1].command,
            b"#255P1400T8000!",
            b"#001PCSR!",
            b"#005P1500T8000!",
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                gate.write(command)
        gate.write(value.axes[0].command)
        with self.assertRaises(ValueError):
            gate.write(value.axes[0].command)
        gate.write(b"#001PDST!")
        with self.assertRaises(RuntimeError):
            gate.write(value.axes[1].command)
        with self.assertRaises(RuntimeError):
            gate.write(b"#001PDST!")
        self.assertEqual(len(self.motions(d)), 1)

    def test_cancel_before_motion_and_logging_failure_do_not_silently_continue(self):
        c, d, _ = self.controller()
        c.cancel()
        r = c.run()
        self.assertEqual(r["status"], "CANCELLED")
        self.assertEqual(d.writes, [])
        c, d, _ = self.controller()

        def fail(event, **values):
            if event == "coordinated_sample":
                raise OSError("disk failure")

        c.link.record = fail
        r = c.run()
        self.assertFalse(r["response_pass"])
        self.assertEqual(len(self.stops(d)), 3)

    def test_cancellation_during_group_stops_all_attempted_axes(self):
        c, d, _ = self.controller()
        d.clock.hook = lambda: c.cancel() if len(d.moves) == 3 else None
        r = c.run()
        self.assertEqual(r["status"], "CANCELLED")
        self.assertEqual(len(self.motions(d)), 3)
        self.assertEqual(len(self.stops(d)), 3)

    def test_group_rejects_duplicate_axis_mixed_time_gripper_and_nominal_proposal(self):
        data = asdict(spec())
        data["axes"] = list(data["axes"])
        for change in (
            {"whole_path_reviewed_for_motion": False},
            {"axes": [data["axes"][0], data["axes"][0]]},
            {"held_max_drift_p": True},
        ):
            with self.assertRaises(ValueError):
                CoordinatedSpec.from_dict({**data, **change})
        for field, value in (("duration_ms", 9000), ("servo_id", 5), ("sample_interval_s", 0.2)):
            axes = [dict(a) for a in data["axes"]]
            axes[1][field] = value
            with self.assertRaises(ValueError):
                CoordinatedSpec.from_dict({**data, "axes": axes})
        with self.assertRaises(ValueError):
            CoordinatedSpec.from_dict({"motion_authorized": False, "segments": []})

    def test_cli_dry_run_does_not_open_port(self):
        from arm_project.cli.run_coordinated import main

        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "spec.json"
            p.write_text(json.dumps(asdict(spec())), encoding="utf-8")
            with (
                patch(
                    "arm_project.arm_link.SerialTransport.open",
                    side_effect=AssertionError("no hardware"),
                ),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--spec", str(p)]), 0)

    def test_reviewed_response_window_never_changes_strict_position_rule(self):
        c, d, _ = self.controller("tail_jitter6")
        old = c.run()
        self.assertFalse(old["response_pass"])
        c, d, _ = self.controller("tail_jitter6")
        original = spec()
        review = CoordinatedSpec(original.review_id, True, original.evidence, original.axes, 10, 8)
        c.spec = review
        c.link.transport.spec = review
        result = c.run()
        self.assertTrue(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertTrue(any(not a["strict_settled"] for a in result["axes"]))
        self.assertEqual(self.stops(d), [])
        self.assertFalse(old["response_pass"])

    def test_invalid_response_bounds_rejected_and_legacy_spec_keeps_two_p(self):
        value = asdict(spec())
        value["axes"] = list(value["axes"])
        value.pop("response_tail_tolerance_p")
        self.assertEqual(CoordinatedSpec.from_dict(value).response_tail_tolerance_p, 2)
        for bound in (True, 1, 11, 2.0):
            with self.subTest(bound=bound), self.assertRaises(ValueError):
                CoordinatedSpec.from_dict({**value, "response_tail_tolerance_p": bound})


if __name__ == "__main__":
    unittest.main()
