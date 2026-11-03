"""Pure-memory cross-layer integration; HARDWARE poses below are test fixtures.

No fixture is exported as a teaching record. Transport and all run results are
SIMULATION. These tests do not open serial ports or verify physical grasping.
"""

from collections import deque
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch
from uuid import uuid4

from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.arm_tasks import TaskStep, TaughtPose
from arm_project.servo_protocol import validate_motion_command, validate_read_query
from arm_project.task_adapter import ReviewedTaskAdapter, compile_task
from arm_project.unified_arm import HardwareArmDriver, MoveSpec


ROOT = Path(__file__).resolve().parents[1]
INITIAL = [1520, 1505, 1494, 1495, 1495, 1500]


class MemoryClock:
    def __init__(self):
        self.seconds = 0.0

    def monotonic(self):
        return self.seconds

    def now(self):
        return datetime(2026, 10, 31, tzinfo=timezone.utc) + timedelta(seconds=self.seconds)

    def timestamp(self):
        return self.now().isoformat()

    def sleep(self, seconds):
        self.seconds += seconds


class MemorySerial:
    source = "SIMULATION"

    def __init__(self, clock, scenario="exact"):
        self.clock, self.scenario = clock, scenario
        self.positions = INITIAL.copy()
        self.pending = deque()
        self.closed = True
        self.motions, self.writes = [], []
        self.on_motion = None
        self.open_calls = 0

    def open(self):
        self.closed = False
        self.open_calls += 1

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return sum(map(len, self.pending))

    def read(self, count):
        self.clock.sleep(0.001)
        if not self.pending:
            return b""
        frame = self.pending.popleft()
        if frame[count:]:
            self.pending.appendleft(frame[count:])
        return frame[:count]

    def write(self, command):
        if self.closed:
            raise OSError("closed simulation transport")
        self.writes.append(command)
        if command.endswith(b"PDST!"):
            return len(command)
        try:
            axis, field = validate_read_query(command)
        except ValueError:
            axis, target, _ = validate_motion_command(command)
            self.motions.append(command)
            previous = self.positions[axis]
            self.positions[axis] = target
            if self.scenario == "gripper_residual" and axis == 5:
                self.positions[axis] = target + 8 if target < previous else target - 8
            if self.scenario == "arm_residual" and axis != 5:
                self.positions[axis] = target + 8 if target < previous else target - 8
            if self.on_motion:
                self.on_motion(command)
            return len(command) - 1 if self.scenario == "short" else len(command)
        if self.motions and self.scenario == "disconnect":
            raise OSError("simulated disconnection")
        if field == "version":
            payload = b"@ " + EXPECTED_FIRMWARE.encode()
        elif field == "mode":
            payload = b"MOD1"
        elif field == "id":
            payload = b"P"
        else:
            payload = ("P%04d" % self.positions[axis]).encode()
        frame = ("#%03d" % axis).encode() + payload + b"!"
        self.pending.extend((frame[:2], frame[2:]))
        return len(command)


def fixture_task():
    """Invented geometry only inside this SIMULATION test module."""
    positions = {
        "above": [1520, 1505, 1494, 1495, 1510, 1500],
        "pick": [1520, 1505, 1494, 1510, 1510, 1480],
        "place": [1540, 1505, 1494, 1495, 1510, 1570],
        "retract": [1540, 1505, 1494, 1495, 1495, 1480],
    }
    poses = {
        name: TaughtPose(name, tuple(p), "HARDWARE", "2026-10-31T00:00:00+00:00", True)
        for name, p in positions.items()
    }
    steps = (
        TaskStep("approach", "move", pose_name="above"),
        TaskStep("open", "grip", action="open"),
        TaskStep("descend", "move", pose_name="pick"),
        TaskStep("close", "grip", action="close"),
        TaskStep("lift", "move", pose_name="above"),
        TaskStep("place", "move", pose_name="place"),
        TaskStep("open", "grip", action="open"),
        TaskStep("retract", "move", pose_name="retract"),
    )
    paths = {}
    starts = INITIAL.copy()
    segments = {
        0: MoveSpec(4, 1510, 1500),
        2: MoveSpec(3, 1510, 1500),
        4: MoveSpec(3, 1495, 1500),
        5: MoveSpec(0, 1540, 1500),
        7: MoveSpec(4, 1495, 1500),
    }
    for index, step in enumerate(steps):
        if step.kind == "grip":
            starts[5] = 1480 if step.action == "open" else 1570
        else:
            spec = segments[index]
            paths["%02d:%s" % (index, step.name)] = {
                "reviewed": True,
                "pose_name": step.pose_name,
                "evidence": "in-memory simulation fixture, not a hardware path review",
                "start_positions_p": starts.copy(),
                "segments": [spec],
            }
            starts[spec.servo_id] = spec.target_p
    return poses, steps, paths


class TaskAdapterTests(unittest.TestCase):
    def setUp(self):
        self.container = ROOT / ("_task_adapter_tests_" + uuid4().hex)
        self.container.mkdir()
        self.addCleanup(self.cleanup)
        self.real_open = patch(
            "arm_project.unified_arm._RawSerialTransport.open",
            side_effect=AssertionError("real serial access is forbidden"),
        )
        self.real_open.start()
        self.addCleanup(self.real_open.stop)
        self.counter = 0

    def cleanup(self):
        resolved = self.container.resolve()
        if resolved.parent != ROOT.resolve() or not resolved.name.startswith(
            "_task_adapter_tests_"
        ):
            raise AssertionError("cleanup must stay inside its own test directory")
        shutil.rmtree(resolved)

    def setup_task(self, scenario="exact", observer="valid"):
        compiled = compile_task(*fixture_task())
        clock = MemoryClock()
        serial = MemorySerial(clock, scenario)
        self.counter += 1
        run_dir = self.container / str(self.counter)
        driver = HardwareArmDriver(
            "NEVER_OPENED", run_dir, compiled.planned_moves, transport=serial, clock=clock
        )
        if observer == "valid":

            def observe(context):
                return {
                    "object_grasped": True,
                    "evidence": {
                        "source": "EXTERNAL_OBSERVATION",
                        "execution_id": context["execution_id"],
                        "step_id": context["step_id"],
                        "observation_id": context["observation_id"],
                        "observed_at": clock.timestamp(),
                        "description": "simulated observer fixture only",
                    },
                }
        else:
            observe = observer
        adapter = ReviewedTaskAdapter(driver, compiled, observer=observe, now=clock.now)
        runner = adapter.create_runner()
        return compiled, clock, serial, driver, adapter, runner, run_dir

    def finish(self, runner):
        state = runner.start()
        for _ in range(8):
            if state["state"] != "RUNNING":
                break
            state = runner.tick()
        return state

    def test_complete_eight_phase_cross_layer_run_is_simulation_only(self):
        compiled, _, serial, driver, _, runner, run_dir = self.setup_task()
        self.assertEqual(serial.open_calls, 0)
        with driver:
            state = self.finish(runner)
        self.assertEqual(state["state"], "SUCCEEDED")
        self.assertEqual(state["source"], "SIMULATION")
        self.assertFalse(state["physical_task_verified"])
        self.assertFalse(state["physical_stop_verified"])
        self.assertEqual(serial.motions, [spec.command for spec in compiled.planned_moves])
        self.assertEqual(len(serial.motions), 8)
        self.assertTrue(driver.closed)
        self.assertEqual(
            json.loads((run_dir / "session.json").read_text(encoding="utf-8"))["source"],
            "SIMULATION",
        )
        self.assertEqual(len(list(run_dir.glob("attempt_*.json"))), 8)

    def test_missing_last_path_fails_before_any_driver_construction(self):
        poses, steps, paths = fixture_task()
        del paths["07:retract"]
        with patch("arm_project.unified_arm.HardwareArmDriver") as constructor:
            with self.assertRaises(ValueError):
                compile_task(poses, steps, paths)
            constructor.assert_not_called()

    def test_complete_budget_counts_noops_and_gripper_and_rejects_over_twenty(self):
        for count, valid in ((13, True), (14, False)):
            poses, steps, paths = fixture_task()
            paths["00:approach"]["segments"] *= count
            if valid:
                self.assertEqual(len(compile_task(poses, steps, paths).planned_moves), 20)
            else:
                with self.assertRaises(ValueError):
                    compile_task(poses, steps, paths)

    def test_same_pose_uses_different_step_paths_and_review_input_is_copied(self):
        poses, steps, paths = fixture_task()
        compiled = compile_task(poses, steps, paths)
        approach, lift = compiled.paths["00:approach"], compiled.paths["04:lift"]
        self.assertEqual(approach.pose_name, lift.pose_name)
        self.assertNotEqual(approach.start_positions_p, lift.start_positions_p)
        paths["00:approach"]["start_positions_p"][0] = 2500
        self.assertEqual(approach.start_positions_p[0], INITIAL[0])
        with self.assertRaises(TypeError):
            compiled.paths["00:approach"] = lift

    def test_inconsistent_or_unsafe_review_is_rejected_statically(self):
        for variant in (
            "start",
            "gripper_hold",
            "gripper_segment",
            "review",
            "boolean",
            "endpoint",
        ):
            poses, steps, paths = fixture_task()
            if variant == "start":
                paths["02:descend"]["start_positions_p"][0] += 1
            elif variant == "gripper_hold":
                paths["02:descend"]["start_positions_p"][5] = 1500
            elif variant == "gripper_segment":
                paths["00:approach"]["segments"].append(MoveSpec(5, 1480, 1000))
            elif variant == "review":
                paths["07:retract"]["reviewed"] = False
            elif variant == "boolean":
                paths["00:approach"]["start_positions_p"][0] = True
            else:
                paths["07:retract"]["segments"] = [MoveSpec(4, 1500, 1500)]
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                compile_task(poses, steps, paths)

    def test_unverified_or_simulation_pose_is_not_promoted_to_hardware(self):
        poses, steps, paths = fixture_task()
        original = poses["above"]
        poses["above"] = TaughtPose(
            original.name, original.positions_p, "HARDWARE", original.captured_at, False
        )
        with self.assertRaises(ValueError):
            compile_task(poses, steps, paths)
        with self.assertRaises(ValueError):
            TaughtPose("fake", tuple(INITIAL), "SIMULATION", original.captured_at, True)

    def test_wrong_driver_budget_rejected_without_opening(self):
        compiled, clock, serial, _, _, _, _ = self.setup_task()
        other = HardwareArmDriver(
            "NEVER_OPENED", self.container / "wrong", (), transport=serial, clock=clock
        )
        with self.assertRaises(ValueError):
            ReviewedTaskAdapter(other, compiled)
        self.assertEqual(serial.open_calls, 0)

    def test_response_only_gripper_does_not_advance_to_descend(self):
        _, _, serial, driver, _, runner, _ = self.setup_task("gripper_residual")
        with driver:
            state = self.finish(runner)
        self.assertEqual(state["state"], "FAILED")
        self.assertIn("RESPONSE_OBSERVED", state["reason"])
        self.assertEqual(len(serial.motions), 2)
        self.assertEqual(state["next_step_index"], 1)

    def test_response_only_pose_short_write_or_disconnect_stops_remaining_steps(self):
        for scenario in ("arm_residual", "short", "disconnect"):
            _, _, serial, driver, _, runner, _ = self.setup_task(scenario)
            with driver:
                state = self.finish(runner)
            with self.subTest(scenario=scenario):
                self.assertEqual(state["state"], "FAILED")
                self.assertEqual(len(serial.motions), 1)
                self.assertTrue(driver.closed)

    def test_without_observer_close_does_not_lift(self):
        _, _, serial, driver, _, runner, _ = self.setup_task(observer=None)
        with driver:
            state = self.finish(runner)
        self.assertEqual(state["state"], "GRIP_UNVERIFIED")
        self.assertEqual(len(serial.motions), 4)
        self.assertFalse(state["grasp_confirmed"])
        self.assertTrue(driver.closed)

    def test_arbitrary_true_stale_time_wrong_identity_and_driver_feedback_are_not_evidence(self):
        for variant in ("true", "old_time", "wrong_step", "old_challenge", "feedback", "future"):
            _, clock, serial, driver, adapter, runner, _ = self.setup_task()
            valid = adapter.observer

            def observer(context, variant=variant):
                if variant == "true":
                    return True
                result = valid(context)
                evidence = result["evidence"]
                if variant == "old_time":
                    evidence["observed_at"] = "2020-01-01T00:00:00+00:00"
                elif variant == "wrong_step":
                    evidence["step_id"] = "01:open"
                elif variant == "old_challenge":
                    evidence["observation_id"] = "previous-observation"
                elif variant == "feedback":
                    evidence["source"] = "SERVO_POSITION"
                else:
                    evidence["observed_at"] = (clock.now() + timedelta(seconds=1)).isoformat()
                return result

            adapter.observer = observer
            with driver:
                state = self.finish(runner)
            with self.subTest(variant=variant):
                self.assertEqual(state["state"], "GRIP_UNVERIFIED")
                self.assertEqual(len(serial.motions), 4)

    def test_cancel_during_motion_reaches_driver_and_never_starts_next_step(self):
        _, _, serial, driver, _, runner, _ = self.setup_task()
        serial.on_motion = lambda command: runner.cancel()
        with driver:
            state = self.finish(runner)
        self.assertEqual(state["state"], "CANCELLED")
        self.assertEqual(len(serial.motions), 1)
        self.assertTrue(driver.snapshot()["cancel_requested"])
        self.assertTrue(driver.closed)
        self.assertFalse(state["physical_stop_verified"])

    def test_cancel_during_observation_discards_true_result(self):
        _, _, serial, driver, adapter, runner, _ = self.setup_task()
        valid = adapter.observer

        def observer(context):
            result = valid(context)
            runner.cancel()
            return result

        adapter.observer = observer
        with driver:
            state = self.finish(runner)
        self.assertEqual(state["state"], "CANCELLED")
        self.assertEqual(len(serial.motions), 4)
        self.assertFalse(state["grasp_confirmed"])

    def test_observer_exception_and_duplicate_runner_or_restart_cannot_resume(self):
        def broken(context):
            raise RuntimeError("simulated observer failure")

        _, _, serial, driver, adapter, runner, _ = self.setup_task(observer=broken)
        with self.assertRaises(RuntimeError):
            adapter.create_runner()
        with driver:
            state = self.finish(runner)
            self.assertEqual(state["state"], "FAILED")
            before = len(serial.motions)
            runner.tick()
            with self.assertRaises(RuntimeError):
                runner.start()
            self.assertEqual(len(serial.motions), before)
        self.assertEqual(len(serial.motions), 4)

    def test_out_of_order_adapter_callback_never_moves(self):
        _, _, serial, driver, adapter, _, _ = self.setup_task()
        with driver:
            result = adapter.grip("close", execution_id="fixture", step_id="03:close")
        self.assertEqual(result["status"], "FAILED")
        self.assertEqual(serial.motions, [])
        self.assertTrue(driver.closed)


if __name__ == "__main__":
    unittest.main()
