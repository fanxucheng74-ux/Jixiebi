"""Local-path tests use only memory transports, including all HARDWARE labels."""

from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import Mock, patch
from uuid import uuid4

from arm_project.cli.run_local_experiment import PoseCamera, main
from arm_project.local_experiments import (
    experiment_steps,
    require_midpoint,
    run_local,
    summarize_local,
)
from arm_project.servo_protocol import validate_motion_command
from arm_project.unified_arm import HardwareArmDriver, PROFILES
from test_unified_arm import MemoryDevice, TestClock

ROOT = Path(__file__).resolve().parents[1]


class OffsetDevice(MemoryDevice):
    def write(self, command):
        try:
            axis, target, _ = validate_motion_command(command)
        except ValueError:
            return super().write(command)
        old = self.positions[axis]
        count = super().write(command)
        self.positions[axis] = target - 4 if target > old else target + 6
        return count


class LocalExperimentTests(unittest.TestCase):
    def setUp(self):
        self.folder = ROOT / ("_local_tests_" + uuid4().hex)
        self.folder.mkdir()
        self.addCleanup(self.cleanup)
        self.n = 0

    def cleanup(self):
        path = self.folder.resolve()
        if path.parent != ROOT.resolve() or not path.name.startswith("_local_tests_"):
            raise AssertionError("cleanup escaped owned directory")
        shutil.rmtree(path)

    def driver(self, name="midpoint", scenario=None):
        self.n += 1
        clock = TestClock()
        device = OffsetDevice(clock) if scenario is None else MemoryDevice(clock, scenario)
        if name in ("midpoint_wide", "empty_path"):
            device.positions[1] = 1526
        steps = experiment_steps(name)
        driver = HardwareArmDriver(
            "NEVER_OPENED",
            self.folder / str(self.n),
            tuple(s.move for s in steps),
            transport=device,
            clock=clock,
        )
        return driver, device

    def experiment(self, name="midpoint"):
        driver, device = self.driver(name)
        result = {"experiment": name, "source": "SIMULATION", "error": None}
        with driver:
            run_local(driver, name, "test_only", result)
        result["driver"] = driver.snapshot()
        return result, driver, device

    def test_midpoint_uses_twelve_exact_budgeted_moves_with_both_directions(self):
        result, driver, device = self.experiment()
        self.assertEqual(len(device.motions), 12)
        self.assertEqual(device.motions, [s.move.command for s in experiment_steps("midpoint")])
        self.assertEqual(driver.snapshot()["consumed_count"], 12)
        self.assertTrue(result["analysis"]["reference_repeatability_pass"])
        self.assertEqual(result["analysis"]["directions"]["from_above"]["medians_p"], [1521] * 3)
        self.assertEqual(result["analysis"]["directions"]["from_below"]["medians_p"], [1511] * 3)
        self.assertEqual(result["analysis"]["directional_gap_p"], 10)

    def test_repeatability_never_promotes_strict_position_or_task_success(self):
        result, _, _ = self.experiment()
        self.assertTrue(result["analysis"]["reference_repeatability_pass"])
        self.assertTrue(all(not t["move"]["position_reached"] for t in result["trials"]))
        self.assertTrue(all(t["pose"]["verified"] is False for t in result["trials"]))
        self.assertFalse(result["analysis"]["task_path_verified"])
        self.assertFalse(result["analysis"]["compensation_authorized"])

    def test_all_commands_remain_in_existing_profiles_and_gripper_has_no_command(self):
        for name in ("midpoint", "midpoint_wide", "empty_path"):
            for step in experiment_steps(name):
                self.assertNotEqual(step.move.servo_id, 5)
                low, high, _, _, _, duration = PROFILES[step.move.servo_id]
                self.assertLessEqual(low, step.move.target_p)
                self.assertLessEqual(step.move.target_p, high)
                self.assertGreaterEqual(step.move.duration_ms, duration)
        self.assertEqual(len(experiment_steps("empty_path")), 17)

    def test_local_path_is_two_serial_cycles_not_a_grasp_or_approved_task(self):
        result, driver, device = self.experiment("empty_path")
        self.assertEqual(len(device.motions), 17)
        self.assertEqual(driver.snapshot()["remaining_count"], 0)
        self.assertTrue(result["analysis"]["local_response_sequence_complete"])
        self.assertFalse(result["analysis"]["task_path_verified"])
        self.assertFalse(result["analysis"]["grasp_verified"])
        self.assertEqual(device.motions[1:9], device.motions[9:])
        self.assertTrue(all(validate_motion_command(m)[0] in (1, 2, 3, 4) for m in device.motions))

    def test_partial_failed_trial_is_preserved_and_not_retried(self):
        driver, device = self.driver(scenario="stuck")
        result = {}
        with driver:
            with self.assertRaisesRegex(RuntimeError, "step response failed"):
                run_local(driver, "midpoint", "test_only", result)
        self.assertEqual(len(device.motions), 1)
        self.assertEqual(len(result["trials"]), 1)
        self.assertFalse(result["trials"][0]["move"]["response_pass"])
        self.assertTrue(driver.closed)

    def test_wrong_initial_side_prevents_all_motion(self):
        driver, device = self.driver()
        device.positions[1] = 1525
        with driver:
            with self.assertRaisesRegex(ValueError, "lower-side"):
                run_local(driver, "midpoint", "test_only", {})
        self.assertEqual(device.motions, [])

    def test_wider_midpoint_preserves_old_plan_and_uses_existing_extremes(self):
        result, _, device = self.experiment("midpoint_wide")
        self.assertTrue(result["analysis"]["reference_repeatability_pass"])
        self.assertEqual(result["analysis"]["directions"]["from_above"]["medians_p"], [1526] * 3)
        self.assertEqual(result["analysis"]["directions"]["from_below"]["medians_p"], [1516] * 3)
        self.assertEqual(
            [validate_motion_command(m)[1] for m in device.motions], [1500, 1520, 1540, 1520] * 3
        )
        self.assertEqual(experiment_steps("midpoint")[1].move.target_p, 1515)

    def test_uncommanded_axis_change_stops_after_first_step(self):
        driver, device = self.driver()
        original = driver.sample_joint_at

        def drift(*args):
            value = original(*args)
            device.positions[2] += 4
            return value

        result = {}
        with driver, patch.object(driver, "sample_joint_at", side_effect=drift):
            with self.assertRaisesRegex(RuntimeError, "uncommanded-axis"):
                run_local(driver, "midpoint", "test_only", result)
        self.assertEqual(len(device.motions), 1)
        self.assertFalse(result["trials"][0]["held_axes_ok"])

    def test_late_drift_or_loss_of_direction_prevents_next_step(self):
        for kind in ("unstable", "direction"):
            driver, device = self.driver()
            original = driver.sample_joint_at

            def change(*args):
                value = original(*args)
                if kind == "unstable":
                    value["samples"][0]["value"] += 4
                else:
                    for sample in value["samples"]:
                        sample["value"] = 1505
                return value

            with driver, patch.object(driver, "sample_joint_at", side_effect=change):
                with self.assertRaisesRegex(RuntimeError, "late feedback"):
                    run_local(driver, "midpoint", "test_only", {})
            self.assertEqual(len(device.motions), 1)

    def test_incomplete_late_wrong_order_or_wide_span_never_passes(self):
        for kind in ("incomplete", "late", "order", "span"):
            result, _, _ = self.experiment()
            trials = deepcopy(result["trials"])
            if kind == "incomplete":
                trials.pop()
            elif kind == "late":
                trials[0]["observation"]["samples"][0]["received_monotonic_s"] += 0.3
            elif kind == "order":
                trials[1]["step"]["approach"] = "from_below"
            else:
                for sample in trials[9]["observation"]["samples"]:
                    sample["value"] += 4
            report = summarize_local("midpoint", trials)
            self.assertFalse(report["reference_repeatability_pass"], kind)

    def hardware_labelled_fixture(self):
        # Metadata only: all records were produced by the memory device above.
        result, _, _ = self.experiment()
        result["source"] = result["driver"]["source"] = "HARDWARE"
        path = self.folder / ("fixture_%d.json" % self.n)
        path.write_text(json.dumps(result), encoding="utf-8")
        return result, path

    def test_path_prerequisite_recomputes_analysis_and_rejects_fault_or_simulation(self):
        for kind in ("valid", "fault", "tampered", "simulation", "partial"):
            result, path = self.hardware_labelled_fixture()
            if kind == "fault":
                result["driver"]["faulted"] = True
            elif kind == "tampered":
                result["analysis"]["directional_gap_p"] = 0
            elif kind == "simulation":
                result["source"] = "SIMULATION"
            elif kind == "partial":
                result["trials"].pop()
            path.write_text(json.dumps(result), encoding="utf-8")
            if kind == "valid":
                self.assertEqual(require_midpoint(path), result)
            else:
                with self.assertRaises(ValueError):
                    require_midpoint(path)

    def test_both_cli_previews_do_not_construct_driver_or_camera(self):
        for name in ("midpoint", "empty_path"):
            with patch("arm_project.cli.run_local_experiment.HardwareArmDriver") as driver:
                with patch("arm_project.cli.run_local_experiment.PoseCamera") as camera:
                    with redirect_stdout(StringIO()):
                        self.assertEqual(main(["--experiment", name, "--camera", "0"]), 0)
            driver.assert_not_called()
            camera.assert_not_called()

    def test_empty_path_requires_prerequisite_before_driver_construction(self):
        with patch("arm_project.cli.run_local_experiment.HardwareArmDriver") as driver:
            with redirect_stdout(StringIO()), self.assertRaises(SystemExit):
                main(
                    [
                        "--experiment",
                        "empty_path",
                        "--execute",
                        "--port",
                        "NO_IO",
                        "--run-id",
                        "test",
                    ]
                )
        driver.assert_not_called()

    def test_camera_releases_on_open_or_read_failure(self):
        for opened in (False, True):
            fake = Mock()
            cap = fake.VideoCapture.return_value
            cap.isOpened.return_value = opened
            cap.read.return_value = (False, None)
            with patch.dict("sys.modules", {"cv2": fake}):
                camera = PoseCamera(0, self.folder / ("images_" + str(opened)))
                with self.assertRaises(RuntimeError):
                    camera.__enter__()
            cap.release.assert_called_once()
            self.assertTrue(camera.released)

    def test_camera_release_failure_is_not_reported_as_released(self):
        fake = Mock()
        cap = fake.VideoCapture.return_value
        cap.isOpened.return_value = False
        cap.release.side_effect = OSError("release failed")
        with patch.dict("sys.modules", {"cv2": fake}):
            camera = PoseCamera(0, self.folder / "images_release")
            with self.assertRaises(OSError):
                camera.__enter__()
        self.assertFalse(camera.released)


if __name__ == "__main__":
    unittest.main()
