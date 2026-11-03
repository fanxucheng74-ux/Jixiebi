"""No hardware: timing, finite budgets, cancellation and diagnostic distinctions."""

from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch
from uuid import uuid4

from arm_project.cli.run_joint_settling import main
from arm_project.joint_settling import MOVES, OFFSETS_S, run_experiment, summarize_trials
from arm_project.unified_arm import HardwareArmDriver, MoveSpec
from test_unified_arm import MemoryDevice, TestClock


ROOT = Path(__file__).resolve().parents[1]


class SettlingTests(unittest.TestCase):
    def setUp(self):
        self.folder = ROOT / ("_settling_tests_" + uuid4().hex)
        self.folder.mkdir()
        self.addCleanup(self.cleanup)
        self.number = 0

    def cleanup(self):
        resolved = self.folder.resolve()
        if resolved.parent != ROOT.resolve() or not resolved.name.startswith("_settling_tests_"):
            raise AssertionError("cleanup outside owned directory")
        shutil.rmtree(resolved)

    def driver(self, moves=(), scenario="normal"):
        self.number += 1
        clock = TestClock()
        device = MemoryDevice(clock, scenario=scenario)
        driver = HardwareArmDriver(
            "NEVER_OPENED", self.folder / str(self.number), moves, transport=device, clock=clock
        )
        return driver, device, clock

    def test_scheduled_reads_follow_absolute_deadlines_without_budget_or_motion(self):
        driver, device, clock = self.driver()
        with driver:
            now = clock.monotonic()
            deadlines = [now + 0.3, now + 0.5, now + 1.0]
            result = driver.sample_joint_at(1, deadlines)
        self.assertEqual(result["status"], "OK")
        self.assertEqual(len(result["samples"]), 3)
        self.assertEqual(device.motions, [])
        self.assertEqual(driver.snapshot()["consumed_count"], 0)
        for sample, deadline in zip(result["samples"], deadlines):
            self.assertAlmostEqual(sample["query_monotonic_s"], deadline)
            self.assertLess(sample["received_monotonic_s"] - deadline, 0.01)

    def test_invalid_schedule_never_queries_or_moves(self):
        for axis, deadlines in (
            (True, [1.0]),
            (6, [1.0]),
            (1, []),
            (1, [float("nan")]),
            (1, [True]),
            (1, [1.0, 1.01]),
            (1, [1.0, 0.5]),
        ):
            with self.subTest(axis=axis, deadlines=deadlines):
                driver, device, _ = self.driver()
                with driver, self.assertRaises(ValueError):
                    driver.sample_joint_at(axis, deadlines)
                self.assertEqual(device.writes, [])

    def test_expired_or_excessive_schedule_faults_session_without_moving(self):
        for deadlines in ([-1.0], [16.0]):
            driver, device, _ = self.driver()
            with driver:
                with self.assertRaises(ValueError):
                    driver.sample_joint_at(1, deadlines)
            self.assertTrue(driver.closed)
            self.assertTrue(driver.snapshot()["faulted"])
            self.assertEqual(device.motions, [])

    def test_delayed_query_does_not_compress_remaining_schedule(self):
        driver, device, clock = self.driver()
        with driver:
            now = clock.monotonic()
            device.before_write = lambda _: clock.sleep(0.25)
            with self.assertRaisesRegex(RuntimeError, "reception missed"):
                driver.sample_joint_at(1, [now + 0.3, now + 0.6])
        self.assertEqual(len(device.writes), 1)
        self.assertEqual(device.motions, [])

    def test_cancellation_while_waiting_prevents_queries_and_future_motion(self):
        driver, device, clock = self.driver((MoveSpec(1, 1530, 2000),))
        with driver:
            clock.hook = lambda: driver.cancel()
            with self.assertRaises(InterruptedError):
                driver.sample_joint_at(1, [clock.monotonic() + 1.0])
        self.assertEqual(device.writes, [])
        self.assertEqual(device.motions, [])
        self.assertTrue(driver.closed)

    def test_read_error_and_out_of_window_cancel_remaining_plan(self):
        for kind in ("error", "outside"):
            driver, device, clock = self.driver((MoveSpec(1, 1530, 2000),))
            with driver:
                if kind == "error":

                    def broken(_):
                        raise OSError("test cable failure")

                    device.before_write = broken
                else:
                    device.positions[1] = 2500
                with self.assertRaises((OSError, ValueError)):
                    driver.sample_joint_at(1, [clock.monotonic() + 0.3])
            self.assertTrue(driver.snapshot()["faulted"])
            self.assertEqual(device.motions, [])

    def test_move_records_host_write_return_and_default_precision_unchanged(self):
        driver, device, clock = self.driver((MoveSpec(1, 1530, 2000),), "undershoot")
        with driver:
            result = driver.move_joint(1, 1530, 2000)
        self.assertLess(result["motion_write_monotonic_s"], clock.monotonic())
        self.assertEqual(result["final_p"], 1522)
        self.assertTrue(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertEqual(len(device.motions), 1)

    def experiment(self, scenario="normal"):
        driver, device, _ = self.driver(MOVES, scenario)
        with driver:
            result = run_experiment(driver, "simulation_only")
        return result, driver, device

    def test_six_exact_motions_and_twelve_scheduled_samples_each(self):
        result, driver, device = self.experiment()
        self.assertEqual(device.motions, [spec.command for spec in MOVES])
        self.assertEqual(driver.snapshot()["consumed_count"], 6)
        self.assertEqual(len(result["trials"]), 6)
        self.assertTrue(result["analysis"]["sampling_valid"])
        self.assertTrue(all(c["repeatability_pass"] for c in result["analysis"]["comparisons"]))
        for trial in result["trials"]:
            anchor = trial["move"]["motion_write_monotonic_s"]
            samples = trial["observation"]["samples"]
            self.assertEqual(len(samples), 12)
            for sample, offset in zip(samples, OFFSETS_S):
                self.assertAlmostEqual(sample["deadline_monotonic_s"] - anchor, offset)
        self.assertFalse(result["analysis"]["physical_positioning_verified"])

    def test_stable_offset_passes_repeatability_but_not_target_precision(self):
        result, _, _ = self.experiment("undershoot")
        self.assertTrue(result["analysis"]["sampling_valid"])
        self.assertTrue(all(c["repeatability_pass"] for c in result["analysis"]["comparisons"]))
        self.assertTrue(all(not t["move"]["position_reached"] for t in result["trials"]))
        self.assertFalse(result["analysis"]["compensation_authorized"])

    def test_same_endpoint_span_over_three_is_not_hidden_by_median(self):
        result, _, _ = self.experiment()
        for sample in result["trials"][5]["observation"]["samples"]:
            sample["value"] += 4
        report = summarize_trials(result["trials"])
        reverse = [c for c in report["comparisons"] if c["target_p"] == 1500]
        self.assertTrue(report["sampling_valid"])
        self.assertTrue(all(c["span_p"] == 4 and not c["repeatability_pass"] for c in reverse))

    def test_incomplete_late_unstable_or_wrong_order_experiment_never_passes(self):
        for kind in ("incomplete", "late", "unstable", "order"):
            result, _, _ = self.experiment()
            trials = result["trials"]
            if kind == "incomplete":
                trials.pop()
            elif kind == "late":
                trials[0]["observation"]["samples"][0]["received_monotonic_s"] += 0.3
            elif kind == "unstable":
                trials[0]["observation"]["samples"][0]["value"] += 4
            else:
                trials[0]["move"]["target_p"] = 1500
            report = summarize_trials(trials)
            self.assertFalse(report["sampling_valid"], kind)
            self.assertFalse(any(c["repeatability_pass"] for c in report["comparisons"]), kind)

    def test_failed_motion_cannot_trigger_observation_or_retry(self):
        driver, device, _ = self.driver(MOVES, "stuck")
        with driver, patch.object(driver, "sample_joint_at") as sample:
            with self.assertRaisesRegex(RuntimeError, "motion response failed"):
                run_experiment(driver, "simulation_only")
        self.assertEqual(len(device.motions), 1)
        sample.assert_not_called()
        self.assertTrue(driver.closed)

    def test_uncommanded_axis_drift_blocks_second_move(self):
        driver, device, _ = self.driver(MOVES)
        original = driver.sample_joint_at

        def drift(*args):
            result = original(*args)
            device.positions[2] += 4
            return result

        with driver, patch.object(driver, "sample_joint_at", side_effect=drift):
            with self.assertRaisesRegex(RuntimeError, "uncommanded-axis"):
                run_experiment(driver, "simulation_only")
        self.assertEqual(len(device.motions), 1)

    def test_cli_default_has_no_hardware_io(self):
        with patch("arm_project.cli.run_joint_settling.HardwareArmDriver") as driver:
            with redirect_stdout(StringIO()) as output:
                self.assertEqual(main([]), 0)
        driver.assert_not_called()
        self.assertIn("DRY_RUN", output.getvalue())

    def test_raw_sample_and_move_times_remain_auditable(self):
        _, driver, _ = self.experiment()
        events = [
            json.loads(line) for line in (driver.run_dir / "events.jsonl").read_text().splitlines()
        ]
        samples = [e for e in events if e["event"] == "scheduled_position_sample"]
        self.assertEqual(len(samples), 72)
        self.assertEqual(len([e for e in events if e["event"] == "move_result"]), 6)
        self.assertTrue(all(s["received_monotonic_s"] >= s["query_monotonic_s"] for s in samples))


if __name__ == "__main__":
    unittest.main()
