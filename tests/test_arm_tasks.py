"""任务层纯软件测试。姿态和外部观察均为虚构测试夹具，不供实物执行。"""

from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
import unittest
import uuid

from arm_project.arm_tasks import (
    ArmTaskRunner,
    PHASES,
    TaskStep,
    TaughtPose,
    load_poses,
    save_poses,
)


ROOT = Path(__file__).resolve().parents[1]


class FakeTime:
    def __init__(self):
        self.value = datetime(2026, 10, 31, 6, 0, tzinfo=timezone.utc)

    def __call__(self):
        return self.value

    def advance(self):
        self.value += timedelta(milliseconds=50)


def fixtures():
    poses = {
        name: TaughtPose(name, (1500,) * 6, "HARDWARE", "2026-10-31T13:00:00+08:00", True)
        for name in ("approach", "descend", "lift", "place", "retract")
    }
    steps = [
        TaskStep(name, "move", pose_name=name)
        if name in poses
        else TaskStep(name, "grip", action=name)
        for name in PHASES
    ]
    return poses, steps


class FakeTaskDriver:
    def __init__(self, clock, scenario="normal", fail_step="02:descend"):
        self.clock = clock
        self.scenario = scenario
        self.fail_step = fail_step
        self.calls = []
        self.hook = None

    def move_pose(self, pose, *, execution_id, step_id):
        return self._execute("move", pose.name, execution_id, step_id)

    def grip(self, action, *, execution_id, step_id):
        return self._execute("grip", action, execution_id, step_id)

    def _execute(self, kind, target, execution_id, step_id):
        self.calls.append((kind, target, execution_id, step_id))
        self.clock.advance()
        if self.hook:
            self.hook()
        if step_id == self.fail_step and self.scenario == "exception":
            raise OSError("simulated device failure")
        result = {"status": "SUCCEEDED", "execution_id": execution_id, "step_id": step_id}
        if step_id == self.fail_step and self.scenario == "failure":
            result["status"] = "FAILED"
        if step_id == self.fail_step and self.scenario == "wrong_result_id":
            result["execution_id"] = "old-execution"
        if step_id == self.fail_step and self.scenario == "interrupt":
            raise KeyboardInterrupt
        if kind == "grip" and target == "close":
            evidence = {
                "source": "EXTERNAL_OBSERVATION",
                "execution_id": execution_id,
                "step_id": step_id,
                "observed_at": self.clock().isoformat(),
                "description": "SIMULATED observation: test object held between jaws",
            }
            result.update(object_grasped=True, evidence=evidence)
            if self.scenario == "no_evidence":
                result.pop("evidence")
            elif self.scenario == "unknown_grip":
                result["object_grasped"] = None
            elif self.scenario == "no_object":
                result["object_grasped"] = False
            elif self.scenario == "position_only":
                evidence["source"] = "GRIPPER_POSITION"
            elif self.scenario == "old_task_evidence":
                evidence["execution_id"] = "old-execution"
            elif self.scenario == "old_step_evidence":
                evidence["step_id"] = "01:open"
            elif self.scenario == "old_time_evidence":
                evidence["observed_at"] = (self.clock() - timedelta(seconds=1)).isoformat()
            elif self.scenario == "future_evidence":
                evidence["observed_at"] = (self.clock() + timedelta(seconds=1)).isoformat()
            elif self.scenario == "naive_time_evidence":
                evidence["observed_at"] = self.clock().replace(tzinfo=None).isoformat()
            elif self.scenario == "empty_description":
                evidence["description"] = " "
            elif self.scenario == "lowercase_source":
                evidence["source"] = "external_observation"
        return result


class TaskRunnerTests(unittest.TestCase):
    def runner(self, scenario="normal", fail_step="02:descend"):
        clock = FakeTime()
        driver = FakeTaskDriver(clock, scenario, fail_step)
        poses, steps = fixtures()
        return ArmTaskRunner(driver, poses, steps, now=clock), driver, clock

    def run_to_terminal(self, runner):
        runner.start()
        for _ in range(8):
            if runner.state != "RUNNING":
                break
            runner.tick()
        return runner.snapshot()

    def test_construct_start_and_idle_tick_do_not_call_driver(self):
        runner, driver, _ = self.runner()
        self.assertEqual(runner.tick()["state"], "READY")
        self.assertEqual(driver.calls, [])
        runner.start()
        self.assertEqual(driver.calls, [])

    def test_normal_task_moves_one_step_per_tick_with_distinct_step_ids(self):
        runner, driver, _ = self.runner()
        runner.start()
        execution_id = runner.execution_id
        for index in range(8):
            state = runner.tick()
            self.assertEqual(len(driver.calls), index + 1)
            self.assertEqual(state["next_step_index"], index + 1)
        self.assertEqual(state["state"], "SUCCEEDED")
        self.assertEqual([call[1] for call in driver.calls], list(PHASES))
        self.assertEqual({call[2] for call in driver.calls}, {execution_id})
        self.assertEqual(len({call[3] for call in driver.calls}), 8)
        self.assertEqual(state["grasp_evidence"]["step_id"], "03:close")
        self.assertFalse(state["physical_stop_verified"])
        self.assertFalse(state["physical_task_verified"])
        runner.tick()
        self.assertEqual(len(driver.calls), 8)
        with self.assertRaises(RuntimeError):
            runner.start()

    def test_preflight_rejects_incomplete_missing_unverified_or_wrong_kind_without_io(self):
        for case in (
            "incomplete",
            "missing",
            "unverified",
            "wrong_kind",
            "wrong_action",
            "reordered",
        ):
            with self.subTest(case=case):
                clock = FakeTime()
                driver = FakeTaskDriver(clock)
                poses, steps = fixtures()
                if case == "incomplete":
                    steps.pop()
                elif case == "missing":
                    del poses["retract"]
                elif case == "unverified":
                    poses["lift"] = TaughtPose(
                        "lift", (1500,) * 6, "HARDWARE", "2026-10-31T13:00:00+08:00", False
                    )
                elif case == "wrong_kind":
                    steps[4] = TaskStep("lift", "grip", action="close")
                elif case == "wrong_action":
                    steps[3] = TaskStep("close", "grip", action="open")
                else:
                    steps[3], steps[4] = steps[4], steps[3]
                with self.assertRaises(ValueError):
                    ArmTaskRunner(driver, poses, steps, now=clock)
                self.assertEqual(driver.calls, [])

    def test_invalid_raw_protocol_positions_cannot_reach_driver(self):
        for value in (-1, 499, 2501, True, 1500.0):
            with self.subTest(value=value):
                clock = FakeTime()
                driver = FakeTaskDriver(clock)
                poses, steps = fixtures()
                with self.assertRaises(ValueError):
                    poses["lift"] = TaughtPose(
                        "lift",
                        (value,) + (1500,) * 5,
                        "HARDWARE",
                        "2026-10-31T13:00:00+08:00",
                        True,
                    )
                    ArmTaskRunner(driver, poses, steps, now=clock)
                self.assertEqual(driver.calls, [])

    def test_callback_failure_exception_or_mismatched_response_never_progresses(self):
        for scenario in ("failure", "exception", "wrong_result_id"):
            with self.subTest(scenario=scenario):
                runner, driver, _ = self.runner(scenario)
                result = self.run_to_terminal(runner)
                self.assertEqual(result["state"], "FAILED")
                self.assertEqual(len(driver.calls), 3)
                self.assertEqual(result["next_step_index"], 2)
                runner.tick()
                self.assertEqual(len(driver.calls), 3)
                with self.assertRaises(RuntimeError):
                    runner.start()

    def test_no_unknown_position_only_stale_or_wrong_grasp_evidence_blocks_lift(self):
        scenarios = (
            "no_evidence",
            "unknown_grip",
            "no_object",
            "position_only",
            "old_task_evidence",
            "old_step_evidence",
            "old_time_evidence",
            "future_evidence",
            "naive_time_evidence",
            "empty_description",
        )
        for scenario in scenarios:
            with self.subTest(scenario=scenario):
                runner, driver, _ = self.runner(scenario)
                result = self.run_to_terminal(runner)
                self.assertEqual(result["state"], "GRIP_UNVERIFIED")
                self.assertFalse(result["grasp_confirmed"])
                self.assertEqual(len(driver.calls), 4)
                self.assertNotIn("lift", [call[1] for call in driver.calls])
                runner.tick()
                self.assertEqual(len(driver.calls), 4)

    def test_lowercase_external_source_and_timezone_aware_evidence_are_accepted(self):
        runner, _, _ = self.runner("lowercase_source")
        self.assertEqual(self.run_to_terminal(runner)["state"], "SUCCEEDED")

    def test_cancel_before_start_and_between_steps_stops_further_calls(self):
        runner, driver, _ = self.runner()
        self.assertEqual(runner.cancel()["state"], "CANCELLED")
        with self.assertRaises(RuntimeError):
            runner.start()
        runner.tick()
        self.assertEqual(driver.calls, [])
        runner, driver, _ = self.runner()
        runner.start()
        runner.tick()
        runner.cancel()
        runner.tick()
        self.assertEqual(len(driver.calls), 1)
        self.assertEqual(runner.state, "CANCELLED")

    def test_cancel_during_close_discards_success_and_never_lifts(self):
        runner, driver, _ = self.runner()

        def cancel_on_close():
            if driver.calls[-1][1] == "close":
                runner.cancel()

        driver.hook = cancel_on_close
        result = self.run_to_terminal(runner)
        self.assertEqual(result["state"], "CANCELLED")
        self.assertEqual(len(driver.calls), 4)
        self.assertFalse(result["grasp_confirmed"])
        self.assertEqual(result["history"][-1]["status"], "CANCELLED_RESULT_DISCARDED")

    def test_keyboard_interrupt_cancels_without_reset_or_return_motion(self):
        runner, driver, _ = self.runner("interrupt")
        result = self.run_to_terminal(runner)
        self.assertEqual(result["state"], "CANCELLED")
        self.assertEqual([call[1] for call in driver.calls], ["approach", "open", "descend"])
        runner.tick()
        self.assertEqual(len(driver.calls), 3)

    def test_reentrant_tick_cannot_execute_the_current_step_twice(self):
        runner, driver, _ = self.runner()

        def nested_tick():
            with self.assertRaises(RuntimeError):
                runner.tick()

        driver.hook = nested_tick
        self.assertEqual(self.run_to_terminal(runner)["state"], "SUCCEEDED")
        self.assertEqual(len(driver.calls), 8)

    def test_execution_ids_are_unique_and_snapshot_cannot_mutate_runner(self):
        first, _, _ = self.runner()
        second, _, _ = self.runner()
        first.start()
        second.start()
        self.assertNotEqual(first.execution_id, second.execution_id)
        view = first.tick()
        view["history"][0]["status"] = "FORGED"
        self.assertEqual(first.history[0]["status"], "SUCCEEDED")


class TaughtPoseTests(unittest.TestCase):
    def setUp(self):
        self.directory = ROOT / "tests" / (".arm-tasks-test-" + uuid.uuid4().hex)
        self.directory.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        target = self.directory.resolve()
        if target.parent != (ROOT / "tests").resolve() or not target.name.startswith(
            ".arm-tasks-test-"
        ):
            raise RuntimeError("unexpected test directory; refusing cleanup")
        shutil.rmtree(target)

    def test_round_trip_keeps_six_raw_positions_source_time_and_verification(self):
        values = [1512, 1503, 1492, 1497, 1495, 1521]
        pose = TaughtPose(
            "TEST_FIXTURE_ONLY", values, "hardware", "2026-10-31T13:00:00+08:00", False
        )
        values[0] = 9999
        self.assertEqual(pose.positions_p[0], 1512)
        self.assertEqual(pose.source, "HARDWARE")
        path = self.directory / "poses.json"
        save_poses(path, [pose])
        self.assertEqual(load_poses(path)[pose.name], pose)
        before = path.read_bytes()
        with self.assertRaises(FileExistsError):
            save_poses(path, [pose])
        self.assertEqual(path.read_bytes(), before)

    def test_bad_pose_provenance_shape_time_or_flag_rejected(self):
        base = {
            "name": "TEST_FIXTURE_ONLY",
            "positions_p": [1500] * 6,
            "source": "HARDWARE",
            "captured_at": "2026-10-31T13:00:00+08:00",
            "verified": True,
        }
        changes = (
            ("name", ""),
            ("positions_p", [1500] * 5),
            ("positions_p", [True] + [1500] * 5),
            ("positions_p", [1500.0] * 6),
            ("positions_p", [-1] + [1500] * 5),
            ("positions_p", [2501] + [1500] * 5),
            ("source", "SIMULATION"),
            ("captured_at", "2026-10-31T13:00:00"),
            ("captured_at", "invalid"),
            ("verified", 1),
        )
        for key, value in changes:
            with self.subTest(key=key, value=value):
                data = deepcopy(base)
                data[key] = value
                with self.assertRaises(ValueError):
                    TaughtPose.from_dict(data)
        del base["captured_at"]
        with self.assertRaises(ValueError):
            TaughtPose.from_dict(base)

    def test_utc_z_timestamp_is_valid(self):
        pose = TaughtPose("TEST_UTC_ONLY", (1500,) * 6, "HARDWARE", "2026-10-31T05:00:00Z", True)
        self.assertEqual(pose.captured_at, "2026-10-31T05:00:00Z")

    def test_duplicate_pose_names_and_incomplete_file_rejected(self):
        poses, _ = fixtures()
        path = self.directory / "poses.json"
        with self.assertRaises(ValueError):
            save_poses(path, [poses["approach"], poses["approach"]])
        self.assertFalse(path.exists())
        path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "poses": [poses["approach"].to_dict(), poses["approach"].to_dict()],
                }
            ),
            encoding="utf-8",
        )
        with self.assertRaises(ValueError):
            load_poses(path)
        path.write_text(json.dumps({"schema_version": 1, "poses": []}), encoding="utf-8")
        with self.assertRaises(ValueError):
            load_poses(path)


if __name__ == "__main__":
    unittest.main()
