"""Finite cycle and fault containment tests: no physical ports or cameras."""

from collections import deque
from copy import deepcopy
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arm_project.arm_link import ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.cli.run_timed_cycle import main
from arm_project.servo_protocol import validate_motion_command, validate_read_query
from arm_project.timed_cycle import CycleController, CycleSpec, CycleTransport
from test_arm_driver import FakeClock


def load_sim_plan(path):
    # 文件本身禁止实物执行；仅用于模拟测试时在内存中满足解析器的审阅字段。
    value = json.loads(path.read_text(encoding="utf-8"))
    if value["whole_path_reviewed_for_motion"] is not False:
        raise ValueError("fixture must not authorize motion")
    value["whole_path_reviewed_for_motion"] = True
    return value


def payload():
    return {
        "schema_version": 1,
        "review_id": "SIM-CYCLE",
        "whole_path_reviewed_for_motion": True,
        "evidence": "simulation fixture, no physical path authorization",
        "start_intervals_p": [[1498, 1502] for _ in range(6)],
        "observation_bounds_p": [[1000, 2100] for _ in range(6)],
        "deadline_ms": 10000,
        "geometry_mm": {
            "base_height_mm": 100,
            "upper_arm_mm": 105,
            "forearm_mm": 75,
            "tool_mm": 180,
        },
        "phases": [
            {
                "name": "lean",
                "duration_ms": 1500,
                "dwell_ms": 0,
                "targets": [{"servo_id": 1, "target_p": 1400, "end_tolerance_p": 20}],
            },
            {
                "name": "home",
                "duration_ms": 1500,
                "dwell_ms": 0,
                "targets": [{"servo_id": 1, "target_p": 1490, "end_tolerance_p": 20}],
            },
            {
                "name": "clamp",
                "duration_ms": 1500,
                "dwell_ms": 100,
                "targets": [{"servo_id": 5, "target_p": 1570, "end_tolerance_p": 80}],
            },
        ],
    }


class Device:
    source = "SIMULATION"

    def __init__(self, clock, scenario="normal"):
        self.clock, self.scenario = clock, scenario
        self.pending, self.writes, self.moves = deque(), [], {}
        self.count = 0
        self.initial = [1500] * 6
        self.closed = True

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return sum(len(x) for x in self.pending)

    def read(self, n):
        self.clock.sleep(0.001)
        if not self.pending:
            return b""
        data = self.pending.popleft()
        if data[n:]:
            self.pending.appendleft(data[n:])
        return data[:n]

    def position(self, sid):
        if self.scenario == "hold_escape" and sid == 1 and self.count >= 2:
            return 1440
        if self.moves and self.scenario == "held_drift" and sid == 4:
            return 1520
        if sid not in self.moves:
            return self.initial[sid]
        start, target, duration, when = self.moves[sid]
        if self.scenario == "stuck":
            return start
        if self.scenario == "opposite":
            return start + 5 if target < start else start - 5
        if self.scenario in ("hardware_like", "shoulder_tail", "shoulder_escape"):
            if (
                self.scenario in ("shoulder_tail", "shoulder_escape")
                and sid == 1
                and target == 1152
            ):
                elapsed = self.clock.now - when
                if elapsed <= duration / 1000:
                    return round(start + (1157 - start) * elapsed / (duration / 1000))
                if elapsed <= duration / 1000 + 0.1:
                    return 1157
                return 1117 if self.scenario == "shoulder_escape" else 1137
            residual = {
                1: {1152: -9, 1500: -1},
                2: {1805: 7, 1503: 13, 1500: 2},
                3: {1995: -7, 1050: -2, 1500: 5},
                5: {1570: -33, 1200: 20},
            }.get(sid, {}).get(target, 0)
            lag = 0.0
            if sid == 3 and target == 1995:
                lag = 0.1 if start < 1400 else 0.05
            if sid == 2 and target == 1503:
                lag = 0.05
            f = min(1, (self.clock.now - when) / (duration / 1000 + lag))
            return round(start + (target + residual - start) * f)
        late = (
            self.scenario in ("wrist_late_tail", "late_and_loaded")
            and sid == 3
            and target == 1995
            and duration in (1500, 3000)
        )
        f = min(1, (self.clock.now - when) / (duration / 1000 + (0.1 if late else 0)))
        residual = 7 if self.scenario == "residual" else 0
        if self.scenario == "late_and_loaded" and sid == 2:
            returning = (
                self.moves.get(1, (0, 0))[1] == 1497 and self.moves.get(3, (0, 0))[1] == 1496
            )
            residual = -4 if returning else 10
        return round(start + (target + residual - start) * f)

    def write(self, command):
        self.writes.append(command)
        if command.endswith(b"PDST!"):
            return len(command)
        try:
            sid, field = validate_read_query(command)
        except ValueError:
            sid, goal, duration = validate_motion_command(command)
            self.count += 1
            self.moves[sid] = self.position(sid), goal, duration, self.clock.now
            return 14 if self.scenario == "short_second" and self.count == 2 else len(command)
        if field == "version":
            body = b"@ " + EXPECTED_FIRMWARE.encode()
        elif field == "mode":
            body = b"MOD5" if self.scenario == "mode" else b"MOD1"
        else:
            if self.scenario == "slow" and self.moves:
                self.clock.sleep(0.6)
            body = f"P{self.position(sid):04d}".encode()
        self.pending.append(f"#{sid:03d}".encode() + body + b"!")
        return len(command)


class TimedCycleTests(unittest.TestCase):
    def controller(self, scenario="normal", reserve=None, record=None):
        clock = FakeClock()
        device = Device(clock, scenario)
        spec = CycleSpec.from_dict(payload())
        gate = CycleTransport("NEVER_OPENED", spec, raw=device)
        link = ServoLink(gate, clock=clock, record=record)
        c = CycleController(link, spec, reserve or (lambda value: None))
        return c, device

    @staticmethod
    def motions(d):
        return [c for c in d.writes if len(c) == 15 and b"T" in c]

    def test_complete_one_budget_and_no_extra_settling_or_physical_success(self):
        claims = []
        c, d = self.controller(reserve=claims.append)
        with c.link:
            r = c.run()
        self.assertEqual(r["status"], "RESPONSE_SEQUENCE_COMPLETED")
        self.assertTrue(r["within_deadline"])
        self.assertLess(r["elapsed_s"], 5)
        self.assertEqual(len(claims), 1)
        self.assertEqual(len(self.motions(d)), 3)
        self.assertFalse(r["physical_task_verified"])
        self.assertFalse(r["strict_position_verified"])
        self.assertTrue(d.closed)
        with self.assertRaises(RuntimeError):
            c.run()

    def test_residual_is_handoff_response_not_strict_position(self):
        c, _ = self.controller("residual")
        r = c.run()
        self.assertTrue(r["response_pass"])
        self.assertFalse(r["phases"][0]["end_active"][0]["strict_within3p"])

    def test_full_eight_phase_reviewed_demo_stays_within_deadline_in_simulation(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "tests/fixtures/sim_cycles/timed_put_regrasp_01.json"
        )
        value = CycleSpec.from_dict(load_sim_plan(path))
        c, d = self.controller()
        d.initial = [1500, 1492, 1508, 1500, 1534, 1537]
        c.link.transport = CycleTransport("NEVER_OPENED", value, raw=d)
        c.spec = value
        r = c.run()
        self.assertTrue(r["response_pass"], r["error"])
        self.assertTrue(r["within_deadline"])
        self.assertLess(r["elapsed_s"], 20)
        self.assertEqual(len(r["phases"]), 8)
        self.assertEqual(len(self.motions(d)), 10)
        self.assertFalse(r["physical_task_verified"])
        self.assertEqual(r["stops"], [])

    def test_residual_cannot_override_nominal_workspace_guard(self):
        c, d = self.controller("residual")
        bad = payload()
        bad["phases"][1]["targets"][0]["target_p"] = 1540
        spec = CycleSpec.from_dict(bad)
        c.link.transport = CycleTransport("NEVER_OPENED", spec, raw=d)
        c.spec = spec
        r = c.run()
        self.assertFalse(r["response_pass"])
        self.assertIn("folds behind", r["error"])

    def test_late_wrist_tail_is_sampled_after_reviewed_dwell_before_handoff(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "tests/fixtures/sim_cycles/timed_put_regrasp_02.json"
        )
        value = CycleSpec.from_dict(load_sim_plan(path))
        c, d = self.controller("wrist_late_tail")
        d.initial = [1500, 1492, 1508, 1500, 1534, 1537]
        c.link.transport = CycleTransport("NEVER_OPENED", value, raw=d)
        c.spec = value
        r = c.run()
        self.assertTrue(r["response_pass"], r["error"])
        self.assertLess(r["elapsed_s"], 20)
        self.assertEqual(r["stops"], [])
        raised = next(p for p in r["phases"] if p["name"] == "raise_wrist")
        retract = next(p for p in r["phases"] if p["name"] == "retract_elbow")
        self.assertEqual(raised["end_active"][0]["feedback_p"], 1995)
        self.assertEqual(retract["starts_p"][3], 1995)

    def test_full_home_phase_commands_elbow_instead_of_misclassifying_load_change(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "tests/fixtures/sim_cycles/timed_put_regrasp_02.json"
        )
        value = CycleSpec.from_dict(load_sim_plan(path))
        c, d = self.controller("late_and_loaded")
        d.initial = [1500, 1492, 1508, 1500, 1534, 1537]
        c.link.transport = CycleTransport("NEVER_OPENED", value, raw=d)
        c.spec = value
        r = c.run()
        self.assertTrue(r["response_pass"], r["error"])
        self.assertLess(r["elapsed_s"], 20)
        self.assertEqual(len(self.motions(d)), 11)
        self.assertIn(b"#002P1503T2500!", self.motions(d))
        self.assertFalse(r["physical_task_verified"])

    def test_new_empty_start_cycle_and_separate_setup_with_late_load_feedback(self):
        root = Path(__file__).resolve().parents[1]
        for filename, initial_grip, phases, count in (
            ("setup_empty_upright_01.json", 1537, 6, 11),
            ("empty_pick_place_01.json", 1220, 12, 22),
        ):
            with self.subTest(filename=filename):
                value = CycleSpec.from_dict(
                    load_sim_plan(root / "tests/fixtures/sim_cycles" / filename)
                )
                c, d = self.controller("late_and_loaded")
                d.initial = [1500, 1492, 1508, 1500, 1534, initial_grip]
                c.link.transport = CycleTransport("NEVER_OPENED", value, raw=d)
                c.spec = value
                r = c.run()
                self.assertTrue(r["response_pass"], r["error"])
                self.assertLess(r["elapsed_s"], 20)
                self.assertEqual(len(r["phases"]), phases)
                self.assertEqual(len(self.motions(d)), count)
                self.assertEqual(r["final_state"]["servos"][5]["position_p"], 1200)
                self.assertFalse(r["physical_task_verified"])
                self.assertEqual(r["stops"], [])

    def test_ordered_hold_requires_the_same_prior_reserved_setpoint(self):
        bad = payload()
        bad["phases"][0]["hold_tracking_p"] = {"1": 20}
        with self.assertRaises(ValueError):
            CycleSpec.from_dict(bad)
        bad = payload()
        bad["phases"][1]["hold_tracking_p"] = {"1": 20}
        with self.assertRaises(ValueError):
            CycleSpec.from_dict(bad)

    def test_unchanged_hold_accepts_no_motion_but_rejects_excess_tracking_error(self):
        for scenario, expected in (("normal", True), ("hold_escape", False)):
            c, d = self.controller(scenario)
            p = payload()
            p["phases"][1]["targets"][0]["target_p"] = 1400
            p["phases"][1]["hold_tracking_p"] = {"1": 20}
            value = CycleSpec.from_dict(p)
            c.link.transport = CycleTransport("NEVER_OPENED", value, raw=d)
            c.spec = value
            r = c.run()
            self.assertEqual(r["response_pass"], expected, r["error"])
            if expected:
                row = r["phases"][1]["end_active"][0]
                self.assertTrue(row["unchanged_ordered_target_hold"])
                self.assertFalse(row["direction_response"])
            else:
                self.assertIn("tracking corridor", r["error"])

    def test_twenty_five_frames_exceeds_the_explicit_finite_budget(self):
        p = payload()
        p["deadline_ms"] = 20000
        p["phases"] = []
        for i in range(12):
            ids = (1, 2, 3) if i == 11 else (1, 2)
            p["phases"].append(
                {
                    "name": f"phase_{i}",
                    "duration_ms": 1500,
                    "dwell_ms": 0,
                    "targets": [
                        {"servo_id": sid, "target_p": 1400, "end_tolerance_p": 20} for sid in ids
                    ],
                }
            )
        with self.assertRaisesRegex(ValueError, "finite24"):
            CycleSpec.from_dict(p)

    def test_short_observed_gripper_policy_does_not_lower_body_motion_minimum(self):
        p = payload()
        p["phases"][2]["duration_ms"] = 1200
        CycleSpec.from_dict(p)
        for phase in (0, 2):
            p = payload()
            p["phases"][phase]["duration_ms"] = 1200 if phase == 0 else 999
            with self.assertRaises(ValueError):
                CycleSpec.from_dict(p)

    def test_revised_cycle_converges_with_observed_directional_residuals_and_elbow_delay(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "tests/fixtures/sim_cycles/empty_pick_place_03.json"
        )
        spec = CycleSpec.from_dict(load_sim_plan(path))
        c, d = self.controller("hardware_like")
        d.initial = [1502, 1499, 1502, 1505, 1535, 1218]
        c.link.transport = CycleTransport("NEVER_OPENED", spec, raw=d)
        c.spec = spec
        r = c.run()
        self.assertTrue(r["response_pass"], r["error"])
        self.assertLess(r["elapsed_s"], 20)
        self.assertEqual(len(self.motions(d)), 22)
        self.assertEqual(r["final_state"]["servos"][5]["position_p"], 1220)
        self.assertFalse(r["physical_task_verified"])
        self.assertEqual(r["stops"], [])

    def test_target_history_reproduces_and_fixes_real_twenty_p_shoulder_tail(self):
        root = Path(__file__).resolve().parents[1] / "tests/fixtures/sim_cycles"
        for filename, expected in (
            ("empty_pick_place_03.json", False),
            ("empty_pick_place_04.json", True),
        ):
            c, d = self.controller("shoulder_tail")
            d.initial = [1502, 1499, 1502, 1505, 1535, 1218]
            spec = CycleSpec.from_dict(load_sim_plan(root / filename))
            c.link.transport = CycleTransport("NEVER_OPENED", spec, raw=d)
            c.spec = spec
            r = c.run()
            self.assertEqual(r["response_pass"], expected, r["error"])
            if expected:
                self.assertLess(r["elapsed_s"], 20)
                self.assertFalse(r["physical_task_verified"])
                self.assertEqual(len(self.motions(d)), 20)
            else:
                self.assertIn("held axis drift", r["error"])

    def test_continued_target_still_rejects_error_outside_original_corridor(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "tests/fixtures/sim_cycles/empty_pick_place_04.json"
        )
        c, d = self.controller("shoulder_escape")
        d.initial = [1502, 1499, 1502, 1505, 1535, 1218]
        spec = CycleSpec.from_dict(load_sim_plan(path))
        c.link.transport = CycleTransport("NEVER_OPENED", spec, raw=d)
        c.spec = spec
        r = c.run()
        self.assertFalse(r["response_pass"])
        self.assertIn("tracking corridor", r["error"])
        self.assertTrue(r["stops"])

    def test_continuation_requires_prior_goal_and_cannot_disguise_a_new_motion(self):
        for phase, sid in ((0, "2"), (1, "1"), (1, "4")):
            p = payload()
            p["phases"][phase]["continue_tracking_p"] = {sid: 20}
            with self.assertRaises(ValueError):
                CycleSpec.from_dict(p)
        p = payload()
        p["initial_setpoints_p"] = {"0": 1500}
        with self.assertRaises(ValueError):
            CycleSpec.from_dict(p)

    def test_reviewed_initial_reference_is_checked_before_any_motion_and_not_sent(self):
        for goal, expected in ((1500, True), (1600, False)):
            c, d = self.controller()
            p = payload()
            p["initial_setpoints_p"] = {"2": goal}
            p["phases"][0]["continue_tracking_p"] = {"2": 20}
            spec = CycleSpec.from_dict(p)
            c.link.transport = CycleTransport("NEVER_OPENED", spec, raw=d)
            c.spec = spec
            r = c.run()
            self.assertEqual(r["response_pass"], expected, r["error"])
            self.assertEqual(len(self.motions(d)), 3 if expected else 0)
            self.assertFalse(any(c.startswith(b"#002P") for c in self.motions(d)))

    def test_stuck_opposite_and_held_drift_cancel_remaining_phases(self):
        for scenario in ("stuck", "opposite", "held_drift"):
            with self.subTest(scenario=scenario):
                c, d = self.controller(scenario)
                r = c.run()
                self.assertFalse(r["response_pass"])
                self.assertEqual(len(self.motions(d)), 1)
                self.assertEqual(d.writes.count(b"#001PDST!"), 1)

    def test_partial_second_write_never_runs_third_phase(self):
        c, d = self.controller("short_second")
        r = c.run()
        self.assertFalse(r["response_pass"])
        self.assertEqual(len(self.motions(d)), 2)
        self.assertEqual(d.writes.count(b"#001PDST!"), 1)
        self.assertFalse(r["physical_stop_verified"])

    def test_bad_mode_or_failed_persistent_budget_sends_no_motion(self):
        for scenario in ("mode", "normal"):

            def fail(value):
                raise OSError("budget storage unavailable")

            c, d = self.controller(scenario, reserve=fail)
            r = c.run()
            self.assertFalse(r["response_pass"])
            self.assertEqual(self.motions(d), [])

    def test_slow_link_hits_deadline_without_finishing_sequence(self):
        c, d = self.controller("slow")
        r = c.run()
        self.assertFalse(r["response_pass"])
        self.assertLess(len(self.motions(d)), 3)

    def test_scheduler_delay_after_last_guard_cannot_return_success_past_deadline(self):
        c, d = self.controller()
        original = c.guard

        def paused_after_guard(positions, starts, moving, holds=None):
            original(positions, starts, moving, holds)
            if c.started is not None and not moving and len(d.moves) == 2:
                c.link.clock.sleep(11)

        c.guard = paused_after_guard
        r = c.run()
        self.assertFalse(r["response_pass"])
        self.assertFalse(r["within_deadline"])
        self.assertEqual(r["status"], "FAILED_AFTER_ATTEMPT")
        self.assertIn("deadline", r["error"])
        self.assertTrue(r["stops"])

    def test_cancel_before_motion_or_during_motion(self):
        for during in (False, True):
            c, d = self.controller()
            if during:
                c.link.clock.hook = lambda: c.cancel() if d.moves else None
            else:
                c.cancel()
            r = c.run()
            self.assertFalse(r["response_pass"])
            self.assertEqual(len(self.motions(d)), int(during))

    def test_logging_failure_including_final_record_requests_stops(self):
        for failed_event in ("cycle_phase_started", "cycle_result"):

            def record(event, **values):
                if event == failed_event:
                    raise OSError("log storage full")

            c, d = self.controller(record=record)
            r = c.run()
            self.assertFalse(r["response_pass"])
            self.assertTrue(any(w.endswith(b"PDST!") for w in d.writes))

    def test_transport_blocks_replay_future_frames_and_configuration(self):
        c, d = self.controller()
        gate = c.link.transport
        with self.assertRaises(RuntimeError):
            gate.write(gate.spec.phases[0].commands[0])
        gate.arm_after_reservation()
        gate.begin_phase(0)
        for wrong in (gate.spec.phases[1].commands[0], b"#001PMOD2!"):
            with self.assertRaises(ValueError):
                gate.write(wrong)
        command = gate.spec.phases[0].commands[0]
        gate.write(command)
        with self.assertRaises(ValueError):
            gate.write(command)
        gate.write(b"#001PDST!")
        with self.assertRaises(RuntimeError):
            gate.begin_phase(1)
        self.assertEqual(len(self.motions(d)), 1)

    def test_invalid_budget_and_phase_schema_rejected(self):
        for edit in (
            lambda p: p.update(deadline_ms=5000),
            lambda p: p.update(whole_path_reviewed_for_motion=False),
            lambda p: p["phases"][0]["targets"].append(deepcopy(p["phases"][0]["targets"][0])),
            lambda p: p["phases"][0]["targets"][0].update(target_p=True),
        ):
            p = payload()
            edit(p)
            with self.assertRaises(ValueError):
                CycleSpec.from_dict(p)

    def test_dry_run_never_constructs_transport(self):
        with tempfile.TemporaryDirectory() as temp:
            file = Path(temp) / "cycle.json"
            file.write_text(json.dumps(payload()), encoding="utf-8")
            with patch(
                "arm_project.cli.run_timed_cycle.CycleTransport",
                side_effect=AssertionError("hardware opened"),
            ):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(["--spec", str(file)]), 0)


if __name__ == "__main__":
    unittest.main()
