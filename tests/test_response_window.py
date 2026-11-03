"""Longer response observation adds read queries only; all movement guards remain."""

from pathlib import Path
import shutil
import unittest
from uuid import uuid4

from arm_project.servo_protocol import validate_motion_command, validate_read_query
from arm_project.unified_arm import HardwareArmDriver, MoveSpec
from test_unified_arm import MemoryDevice, TestClock

ROOT = Path(__file__).resolve().parents[1]


class DelayedDevice(MemoryDevice):
    def __init__(self, clock):
        super().__init__(clock)
        self.delayed = None

    def write(self, command):
        try:
            axis, target, _ = validate_motion_command(command)
        except ValueError:
            try:
                queried, field = validate_read_query(command)
            except ValueError:
                queried, field = None, None
            if self.delayed and field == "position" and queried == self.delayed[0]:
                axis, target, when = self.delayed
                if self.clock.monotonic() >= when:
                    self.positions[axis] = target
            return super().write(command)
        old = self.positions[axis]
        count = super().write(command)
        self.positions[axis] = old
        self.delayed = (
            axis,
            target - 4 if target > old else target + 6,
            self.clock.monotonic() + 6,
        )
        return count


class ResponseWindowTests(unittest.TestCase):
    def setUp(self):
        self.folder = ROOT / ("_response_tests_" + uuid4().hex)
        self.folder.mkdir()
        self.addCleanup(self.cleanup)
        self.n = 0

    def cleanup(self):
        path = self.folder.resolve()
        if path.parent != ROOT.resolve() or not path.name.startswith("_response_tests_"):
            raise AssertionError("cleanup escaped owned directory")
        shutil.rmtree(path)

    def driver(self, seconds, scenario="delayed", interval=0.1):
        self.n += 1
        clock = TestClock()
        device = DelayedDevice(clock) if scenario == "delayed" else MemoryDevice(clock, scenario)
        driver = HardwareArmDriver(
            "NEVER_OPENED",
            self.folder / str(self.n),
            (MoveSpec(1, 1530, 2000),),
            transport=device,
            clock=clock,
            response_observation_s=seconds,
            response_poll_interval_s=interval,
        )
        return driver, device, clock

    def test_default_still_fails_unchanged_feedback_without_retry(self):
        driver, device, _ = self.driver(0)
        with driver:
            result = driver.move_joint(1, 1530, 2000)
        self.assertFalse(result["response_pass"])
        self.assertEqual(len(result["feedback"]), 11)
        self.assertEqual(len(device.motions), 1)

    def test_explicit_long_window_observes_delayed_response_but_not_exact_arrival(self):
        driver, device, clock = self.driver(9)
        with driver:
            result = driver.move_joint(1, 1530, 2000)
        self.assertTrue(result["response_pass"])
        self.assertFalse(result["position_reached"])
        self.assertEqual(result["status"], "RESPONSE_OBSERVED")
        self.assertEqual(result["final_p"], 1526)
        self.assertEqual(len(device.motions), 1)
        self.assertGreater(len(result["feedback"]), 11)
        self.assertLessEqual(len(result["feedback"]), 90)
        self.assertLess(clock.monotonic() - result["motion_write_monotonic_s"], 9.3)

    def test_long_window_still_rejects_a_stuck_joint(self):
        driver, device, _ = self.driver(9, "stuck")
        with driver:
            result = driver.move_joint(1, 1530, 2000)
        self.assertFalse(result["response_pass"])
        self.assertIn("no distinct", result["error"])
        self.assertEqual(len(device.motions), 1)

    def test_opposite_and_outside_feedback_abort_immediately_even_with_long_window(self):
        for scenario in ("opposite", "outside"):
            driver, device, clock = self.driver(9, scenario)
            with driver:
                result = driver.move_joint(1, 1530, 2000)
            self.assertFalse(result["response_pass"])
            self.assertLess(clock.monotonic() - result["motion_write_monotonic_s"], 3)
            self.assertEqual(len(device.motions), 1)

    def test_cancel_during_extended_observation_prevents_further_reads(self):
        driver, device, clock = self.driver(9)
        with driver:
            clock.hook = lambda: driver.cancel() if clock.monotonic() > 5 else None
            result = driver.move_joint(1, 1530, 2000)
        self.assertEqual(result["status"], "CANCELLED")
        self.assertEqual(len(device.motions), 1)
        self.assertLess(clock.monotonic(), 6)
        self.assertTrue(driver.closed)

    def test_invalid_policy_never_constructs_session_or_opens_transport(self):
        for value in (True, -1, 10, float("inf"), float("nan"), "9"):
            with self.assertRaises(ValueError):
                self.driver(value)
        self.assertFalse(any(self.folder.iterdir()))

    def test_low_polling_keeps_tail_density_and_reduces_middle_queries(self):
        results = []
        for interval in (0.1, 1.0):
            driver, device, _ = self.driver(9, interval=interval)
            with driver:
                result = driver.move_joint(1, 1530, 2000)
            self.assertTrue(result["response_pass"])
            self.assertEqual(len(device.motions), 1)
            times = [s["received_monotonic_s"] for s in result["feedback"][-5:]]
            self.assertLessEqual(times[-1] - times[0], 0.41)
            results.append(result)
        self.assertLess(len(results[1]["feedback"]), len(results[0]["feedback"]))
        self.assertEqual(results[0]["final_p"], results[1]["final_p"])

    def test_invalid_poll_interval_never_creates_session(self):
        for value in (True, 0, 1.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.driver(9, interval=value)
        self.assertFalse(any(self.folder.iterdir()))

    def test_wait_never_sleeps_negative_when_clock_crosses_deadline_between_reads(self):
        driver, _, clock = self.driver(9)
        sleeps = []

        def ticking():
            value = clock.now
            clock.now += 0.006
            return value

        def checked_sleep(seconds):
            self.assertGreaterEqual(seconds, 0)
            sleeps.append(seconds)
            clock.now += seconds

        clock.monotonic = ticking
        clock.sleep = checked_sleep
        driver._wait(0.01)
        self.assertTrue(sleeps)
        self.assertTrue(all(0 <= s <= 0.02 for s in sleeps))
        self.assertEqual(driver.snapshot()["motion_attempts"], 0)


if __name__ == "__main__":
    unittest.main()
