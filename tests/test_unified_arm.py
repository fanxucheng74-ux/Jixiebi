"""统一驱动纯内存测试；所有HARDWARE姿态标签仅为测试夹具，无实物I/O。"""

from collections import deque
from dataclasses import FrozenInstanceError
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
from threading import Event, Thread
import unittest
from unittest.mock import patch
from uuid import uuid4

from arm_project.arm_tasks import TaughtPose
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.servo_protocol import validate_read_query, validate_motion_command
from arm_project.unified_arm import HardwareArmDriver, MoveSpec


ROOT = Path(__file__).resolve().parents[1]
INITIAL = [1520, 1505, 1494, 1495, 1495, 1500]


class TestClock:
    def __init__(self):
        self.now = 0.0
        self.hook = None

    def monotonic(self):
        return self.now

    def timestamp(self):
        return (
            datetime(2026, 10, 31, tzinfo=timezone.utc) + timedelta(seconds=self.now)
        ).isoformat()

    def sleep(self, seconds):
        self.now += seconds
        if self.hook:
            self.hook()


class MemoryDevice:
    source = "SIMULATION"

    def __init__(self, clock, scenario="normal", positions=None):
        self.clock = clock
        self.scenario = scenario
        self.positions = list(positions or INITIAL)
        self.pending = deque()
        self.writes = []
        self.motions = []
        self.closed = True
        self.open_calls = 0
        self.close_calls = 0
        self.position_reads = 0
        self.before_write = None
        self.motion_observer = None

    def open(self):
        self.open_calls += 1
        self.closed = False

    def close(self):
        self.close_calls += 1
        self.closed = True

    @property
    def in_waiting(self):
        return sum(map(len, self.pending))

    def read(self, count):
        self.clock.sleep(0.001)
        if not self.pending:
            return b""
        chunk = self.pending.popleft()
        if chunk[count:]:
            self.pending.appendleft(chunk[count:])
        return chunk[:count]

    def write(self, command):
        if self.closed:
            raise OSError("memory transport is closed")
        if self.before_write:
            self.before_write(command)
        self.writes.append(command)
        if command.endswith(b"PDST!"):
            self.pending.append(b"#OK!")
            return len(command)
        try:
            axis, field = validate_read_query(command)
        except ValueError:
            axis, target, duration = validate_motion_command(command)
            if self.motion_observer:
                self.motion_observer(command)
            self.motions.append(command)
            old = self.positions[axis]
            if self.scenario == "stuck":
                pass
            elif self.scenario == "opposite":
                self.positions[axis] = old - 5 if target > old else old + 5
            elif self.scenario == "undershoot":
                self.positions[axis] = target - 8 if target > old else target + 8
            elif self.scenario == "outside":
                self.positions[axis] = 2500
            else:
                self.positions[axis] = target
            return len(command) - 1 if self.scenario == "short_motion" else len(command)
        if self.motions and self.scenario == "disconnect":
            raise OSError("simulated disconnected cable")
        if self.motions and self.scenario == "silent":
            return len(command)
        if field == "version":
            value = "WRONG-FIRMWARE" if self.scenario == "wrong_firmware" else EXPECTED_FIRMWARE
            body = b"@ " + value.encode()
        elif field == "mode":
            body = b"MOD5" if self.scenario == "wrong_mode" else b"MOD1"
        elif field == "id":
            body = b"P"
        elif field == "telemetry":
            body = b"@T090V082"
        else:
            self.position_reads += 1
            p = self.positions[axis]
            if self.scenario == "unstable" and axis == 0:
                p += 4 if self.position_reads % 2 else 0
            body = ("P%04d" % p).encode()
        frame = ("#%03d" % axis).encode() + body + b"!"
        self.pending.extend((frame[:1], frame[1:]))
        return len(command)


class UnifiedArmTests(unittest.TestCase):
    def setUp(self):
        # 此Windows沙箱中TemporaryDirectory的0700 ACL不可访问；普通mkdir继承工作区ACL。
        self.container = ROOT / ("_unified_tests_" + uuid4().hex)
        self.container.mkdir()
        self.addCleanup(self.cleanup_directory)
        self.counter = 0

    def cleanup_directory(self):
        resolved = self.container.resolve()
        if resolved.parent != ROOT.resolve() or not resolved.name.startswith("_unified_tests_"):
            raise AssertionError("test cleanup escaped its own workspace directory")
        shutil.rmtree(resolved)

    def driver(self, moves=(), scenario="normal", positions=None):
        self.counter += 1
        run = self.container / ("session_%02d" % self.counter)
        clock = TestClock()
        device = MemoryDevice(clock, scenario, positions)
        driver = HardwareArmDriver(
            "NEVER_OPENED", run, planned_moves=moves, transport=device, clock=clock
        )
        return driver, device, run

    def move(self):
        return MoveSpec(0, 1540, 1500)

    def test_move_spec_is_frozen_and_invalid_plan_does_not_create_directory(self):
        plan = self.move()
        with self.assertRaises(FrozenInstanceError):
            plan.target_p = 1550
        for args in (
            (True, 1540, 1500),
            (6, 1540, 1500),
            (0, 1556, 1500),
            (0, 1540.0, 1500),
            (0, 1540, float("nan")),
            (0, 1540, 1000),
        ):
            path = self.container / "invalid"
            with self.subTest(args=args), self.assertRaises((ValueError, TypeError)):
                HardwareArmDriver("NEVER_OPENED", path, planned_moves=(MoveSpec(*args),))
            self.assertFalse(path.exists())
        with self.assertRaises(ValueError):
            HardwareArmDriver(
                "NEVER_OPENED", self.container / "too_many", planned_moves=(plan,) * 21
            )
        self.assertFalse((self.container / "too_many").exists())

    def test_construction_does_not_open_and_existing_run_directory_rejected(self):
        driver, device, run = self.driver()
        self.assertEqual(device.open_calls, 0)
        with self.assertRaises(FileExistsError):
            HardwareArmDriver("NEVER_OPENED", run, transport=device, clock=device.clock)
        self.assertEqual(device.open_calls, 0)
        with driver:
            driver.read_state()
        self.assertTrue(driver.closed)
        self.assertEqual(device.open_calls, 1)
        self.assertEqual(device.motions, [])
        self.assertTrue((run / "session.json").is_file())

    def test_read_state_and_capture_pose_are_read_only_and_not_verified(self):
        driver, device, _ = self.driver()
        with driver:
            state = driver.read_state()
            pose = driver.capture_pose("simulation_only")
        self.assertEqual(state["status"], "OK")
        self.assertEqual([s["position_p"] for s in state["servos"]], INITIAL)
        self.assertEqual(pose["source"], "SIMULATION")
        self.assertIs(pose["verified"], False)
        self.assertEqual(device.motions, [])
        for command in device.writes:
            validate_read_query(command)

    def test_budget_exists_before_motion_and_single_exact_plan_cannot_be_reused(self):
        driver, device, run = self.driver((self.move(),))
        seen = []

        def inspect_budget(command):
            path = run / "attempt_01.json"
            self.assertTrue(path.is_file())
            seen.append(json.loads(path.read_text(encoding="utf-8")))

        device.motion_observer = inspect_budget
        with driver:
            result = driver.move_joint(0, 1540, 1500, execution_id="exec", step_id="one")
            self.assertEqual(result["status"], "SUCCEEDED")
            self.assertTrue(result["response_pass"])
            self.assertTrue(result["position_reached"])
            self.assertEqual(result["execution_id"], "exec")
            self.assertEqual(result["step_id"], "one")
            repeated = driver.move_joint(0, 1540, 1500)
            self.assertEqual(repeated["status"], "FAILED")
        self.assertEqual(device.motions, [b"#000P1540T1500!"])
        self.assertEqual(len(seen), 1)

    def test_wrong_axis_unplanned_target_and_configuration_text_never_move(self):
        for args in (
            (5, 1480, 1000),
            (0, 1550, 1500),
            ("#005PMOD2!", 1540, 1500),
            (255, 1540, 1500),
            (0, 1540, 1000),
        ):
            driver, device, _ = self.driver((self.move(),))
            with driver:
                result = driver.move_joint(*args)
                self.assertEqual(result["status"], "FAILED")
            self.assertEqual(device.motions, [])
            self.assertFalse(
                any(b"PMOD2" in cmd or cmd.startswith(b"#255") for cmd in device.writes)
            )

    def test_wire_gate_rejects_configuration_broadcast_and_unreserved_motion(self):
        driver, device, _ = self.driver((self.move(),))
        with driver:
            before = list(device.writes)
            for command in (
                b"#005PMOD2!",
                b"#005PULK!",
                b"#255PRAD!",
                b"#000P1540T1500!",
                b"#005P1480T1000!",
                b"#000P1540T1500!\r\n",
            ):
                with self.subTest(command=command), self.assertRaises(ValueError):
                    driver.transport.write(command)
            self.assertEqual(device.writes, before)
        self.assertEqual(device.motions, [])

    def test_noop_consumes_plan_and_budget_without_motion_frame(self):
        driver, device, run = self.driver((MoveSpec(0, 1520, 1500), self.move()))
        with driver:
            first = driver.move_joint(0, 1520, 1500)
            self.assertEqual(device.motions, [])
            self.assertFalse(first["motion_attempted"])
            self.assertEqual(first["status"], "SUCCEEDED")
            first_budget = json.loads((run / "attempt_01.json").read_text(encoding="utf-8"))
            self.assertIn("NO_MOTION_NEEDED", json.dumps(first_budget))
            second = driver.move_joint(0, 1540, 1500)
            self.assertEqual(second["status"], "SUCCEEDED")
        self.assertTrue((run / "attempt_02.json").is_file())
        self.assertEqual(device.motions, [b"#000P1540T1500!"])

    def test_cancel_during_persistent_reservation_prevents_motion(self):
        from arm_project import unified_arm

        driver, device, run = self.driver((self.move(),))
        actual_fsync = unified_arm.os.fsync

        def cancel_then_sync(fd):
            driver.cancel()
            return actual_fsync(fd)

        with driver:
            with patch("arm_project.unified_arm.os.fsync", side_effect=cancel_then_sync):
                result = driver.move_joint(0, 1540, 1500)
        self.assertEqual(result["status"], "CANCELLED")
        self.assertEqual(device.motions, [])
        self.assertTrue((run / "attempt_01.json").exists())
        self.assertFalse(any(cmd.endswith(b"PDST!") for cmd in device.writes))

    def test_cancel_file_created_during_reservation_prevents_motion(self):
        from arm_project import unified_arm

        driver, device, run = self.driver((self.move(),))
        actual_fsync = unified_arm.os.fsync

        def mark_then_sync(fd):
            (run / "cancel.request").write_text("test cancellation", encoding="utf-8")
            return actual_fsync(fd)

        with driver:
            with patch("arm_project.unified_arm.os.fsync", side_effect=mark_then_sync):
                result = driver.move_joint(0, 1540, 1500)
        self.assertEqual(result["status"], "CANCELLED")
        self.assertEqual(device.motions, [])

    def test_cancel_in_final_wire_log_sends_neither_motion_nor_stop(self):
        driver, device, _ = self.driver((self.move(),))
        with driver:
            original_record = driver._record

            def cancel_after_motion_log(event, **values):
                original_record(event, **values)
                if event == "wire_tx" and values.get("kind") == "motion":
                    driver.cancel()

            driver._record = cancel_after_motion_log
            result = driver.move_joint(0, 1540, 1500)
        self.assertEqual(result["status"], "CANCELLED")
        self.assertEqual(device.motions, [])
        self.assertFalse(any(cmd.endswith(b"PDST!") for cmd in device.writes))
        self.assertEqual(driver.snapshot()["motion_attempts"], 0)

    def test_noop_position_change_during_reservation_cannot_reuse_old_success(self):
        from arm_project import unified_arm

        driver, device, run = self.driver((MoveSpec(0, 1520, 1500),))
        actual_fsync = unified_arm.os.fsync

        def drift_then_sync(fd):
            device.positions[0] = 1525
            return actual_fsync(fd)

        with driver:
            with patch("arm_project.unified_arm.os.fsync", side_effect=drift_then_sync):
                result = driver.move_joint(0, 1520, 1500)
        self.assertEqual(result["status"], "FAILED")
        self.assertFalse(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertEqual(device.motions, [])
        self.assertTrue((run / "attempt_01.json").exists())

    def test_final_result_logging_failure_clears_success_flags(self):
        driver, device, _ = self.driver((self.move(),))
        with driver:
            original_record = driver._record

            def broken_result(event, **values):
                if event == "move_result":
                    raise OSError("simulated full disk at final result")
                return original_record(event, **values)

            driver._record = broken_result
            result = driver.move_joint(0, 1540, 1500)
        self.assertEqual(result["status"], "FAILED")
        self.assertFalse(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertEqual(device.motions, [b"#000P1540T1500!"])

    def test_precision_and_response_are_independent(self):
        driver, device, _ = self.driver((self.move(),), "undershoot")
        with driver:
            result = driver.move_joint(0, 1540, 1500)
        self.assertEqual(result["status"], "RESPONSE_OBSERVED")
        self.assertTrue(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertFalse(result["physical_stop_verified"])
        self.assertEqual(device.positions[0], 1532)

    def test_unstable_start_and_wrong_mode_prevent_any_motion(self):
        for scenario in ("unstable", "wrong_mode", "wrong_firmware"):
            driver, device, _ = self.driver((self.move(),), scenario)
            with driver:
                result = driver.move_joint(0, 1540, 1500)
            self.assertEqual(result["status"], "FAILED")
            self.assertEqual(device.motions, [])
            self.assertTrue(driver.closed)

    def test_legal_gripper_target_still_rejects_excessive_fresh_step(self):
        initial = list(INITIAL)
        initial[5] = 1460
        driver, device, _ = self.driver((MoveSpec(5, 1570, 1000),), positions=initial)
        with driver:
            result = driver.grip("close")
        self.assertEqual(result["status"], "FAILED")
        self.assertEqual(device.motions, [])

    def test_each_axis_retains_its_local_command_and_duration_limits(self):
        invalid = (
            (0, 1504, 1500),
            (1, 1541, 2000),
            (1, 1540, 1500),
            (2, 1531, 2000),
            (2, 1530, 1500),
            (3, 1489, 1500),
            (4, 1541, 1500),
            (5, 1479, 1000),
            (5, 1571, 1000),
            (5, 1500, 999),
        )
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(ValueError):
                MoveSpec(*args)

    def test_short_write_timeout_reverse_no_change_and_outside_feedback_abort_remaining(self):
        for scenario in ("short_motion", "silent", "disconnect", "opposite", "stuck", "outside"):
            driver, device, run = self.driver((self.move(), MoveSpec(0, 1510, 1500)), scenario)
            with driver:
                result = driver.move_joint(0, 1540, 1500)
                self.assertEqual(result["status"], "FAILED")
                with self.assertRaises(RuntimeError):
                    driver.move_joint(0, 1510, 1500)
            self.assertTrue(driver.closed)
            self.assertEqual(device.motions, [b"#000P1540T1500!"])
            self.assertTrue((run / "attempt_01.json").exists())
            self.assertFalse((run / "attempt_02.json").exists())

    def test_grip_uses_only_declared_preset_and_never_claims_object_grasped(self):
        moves = (MoveSpec(5, 1480, 1000), MoveSpec(5, 1570, 1000))
        driver, device, _ = self.driver(moves)
        with driver:
            opened = driver.grip("open", execution_id="e", step_id="open")
            closed = driver.grip("close", execution_id="e", step_id="close")
        self.assertEqual(opened["status"], "SUCCEEDED")
        self.assertEqual(closed["status"], "SUCCEEDED")
        self.assertIsNot(closed.get("object_grasped"), True)
        self.assertFalse(closed["grasp_verified"])
        self.assertEqual(
            closed["meaning"], "local gripper response only; load and grasp not inferred"
        )
        self.assertEqual(device.motions, [b"#005P1480T1000!", b"#005P1570T1000!"])

    def pose(self, positions=None, verified=True):
        return TaughtPose(
            "fixture_pose",
            tuple(positions or [1540, *INITIAL[1:]]),
            "HARDWARE",
            "2026-10-31T00:00:00+00:00",
            verified,
        )

    def review(self, pose, segments=None):
        return {
            "reviewed": True,
            "pose_name": pose.name,
            "evidence": "pure simulation fixture",
            "start_positions_p": list(INITIAL),
            "segments": segments or [self.move()],
        }

    def test_pose_cannot_enable_path_without_review_or_when_not_verified(self):
        unverified = self.pose(verified=False)
        for pose, review in ((self.pose(), None), (unverified, self.review(unverified))):
            driver, device, _ = self.driver((self.move(),))
            with driver:
                result = driver.move_pose(
                    pose, execution_id="e", step_id="invalid_pose", path_review=review
                )
            self.assertEqual(result["status"], "FAILED")
            self.assertEqual(device.motions, [])

    def test_pose_path_cannot_sneak_in_gripper_segment(self):
        gripper_move = MoveSpec(5, 1570, 1000)
        pose = self.pose()
        driver, device, _ = self.driver((self.move(), gripper_move))
        with driver:
            result = driver.move_pose(
                pose,
                execution_id="e",
                step_id="bad_gripper_path",
                path_review=self.review(pose, [self.move(), gripper_move]),
            )
        self.assertEqual(result["status"], "FAILED")
        self.assertEqual(device.motions, [])

    def test_pose_review_cannot_override_live_start_or_exact_plan(self):
        pose = self.pose()
        for change in (
            "old_start",
            "wrong_segment",
            "wrong_pose_name",
            "no_evidence",
            "not_reviewed",
        ):
            review = self.review(pose)
            if change == "old_start":
                review["start_positions_p"][0] += 10
            elif change == "wrong_segment":
                review["segments"] = [MoveSpec(0, 1550, 1500)]
            elif change == "wrong_pose_name":
                review["pose_name"] = "other_pose"
            elif change == "no_evidence":
                review["evidence"] = ""
            else:
                review["reviewed"] = False
            driver, device, _ = self.driver((self.move(),))
            with driver:
                result = driver.move_pose(
                    pose, execution_id="e", step_id="bad_review", path_review=review
                )
            with self.subTest(change=change):
                self.assertEqual(result["status"], "FAILED")
                self.assertEqual(device.motions, [])

    def test_reviewed_pose_moves_only_arm_axes_and_holds_gripper(self):
        pose = self.pose([1540, *INITIAL[1:5], 1570])
        driver, device, _ = self.driver((self.move(),))
        with driver:
            result = driver.move_pose(
                pose, execution_id="e", step_id="pose", path_review=self.review(pose)
            )
        self.assertEqual(result["status"], "SUCCEEDED")
        self.assertEqual(device.motions, [b"#000P1540T1500!"])
        self.assertEqual(device.positions[5], INITIAL[5])

    def test_parallel_operation_is_rejected_and_cancel_does_not_write_from_second_thread(self):
        driver, device, _ = self.driver((self.move(),))
        entered, release = Event(), Event()
        blocked = [False]
        failures = []

        def block_one_query(command):
            if command == b"#000PRAD!" and not blocked[0]:
                blocked[0] = True
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test thread timed out")

        def reader():
            try:
                driver.read_state()
            except Exception as error:
                failures.append(error)

        with driver:
            device.before_write = block_one_query
            worker = Thread(target=reader)
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                with self.assertRaises(RuntimeError):
                    driver.move_joint(0, 1540, 1500)
                driver.cancel()
            finally:
                release.set()
                worker.join(2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(device.motions, [])
        for command in device.writes:
            validate_read_query(command)


if __name__ == "__main__":
    unittest.main()
