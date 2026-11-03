"""T starts after target transmission; logging/write delays cannot shorten it."""

import unittest

from arm_project.arm_link import ServoLink
from arm_project.servo_protocol import validate_read_query
from arm_project.transfer_task_motion import (
    TaskTransport,
    TransferTaskController,
    TransferTaskSpec,
    motion_phases_sha256,
)
from test_arm_driver import FakeClock
from test_timed_cycle import Device
from test_transfer_task_motion import phase, review_payload


class _TimedDevice(Device):
    def __init__(self, clock, *, write_delay=0, query_delay=0):
        super().__init__(clock)
        self.write_delay = write_delay
        self.query_delay = query_delay
        self.position_ages = []

    def write(self, command):
        try:
            sid, field = validate_read_query(command)
        except ValueError:
            if b"T" in command:
                self.clock.sleep(self.write_delay)
        else:
            if self.moves and field == "position":
                self.clock.sleep(self.query_delay)
                if sid in self.moves:
                    self.position_ages.append((sid, self.clock.now - self.moves[sid][3]))
        return super().write(command)


class CycleSendTimingTests(unittest.TestCase):
    def run_phase(
        self,
        *,
        log_delay=0,
        write_delay=0,
        query_delay=0,
        targets=None,
        deadline_ms=5000,
    ):
        clock = FakeClock()
        raw = _TimedDevice(clock, write_delay=write_delay, query_delay=query_delay)
        review = review_payload()
        review["phases"] = [phase("entry", targets or {0: 1600}, 1500, 3)]
        review["deadline_ms"] = deadline_ms
        review["motion_phases_sha256"] = motion_phases_sha256(review["phases"])
        spec = TransferTaskSpec.from_dict(review)

        def record(event, **values):
            if event == "motion_tx":
                clock.sleep(log_delay)

        gate = TaskTransport("NEVER_OPENED", spec, raw=raw)
        link = ServoLink(gate, clock=clock, record=record)
        controller = TransferTaskController(link, spec, lambda value: None)
        return controller.run(), raw

    def test_300ms_motion_log_delay_still_gives_the_motor_its_entire_t(self):
        result, raw = self.run_phase(log_delay=0.3)
        self.assertTrue(result["response_pass"], result["error"])
        endpoint = result["phases"][0]["end_active"][0]
        self.assertEqual(endpoint["feedback_p"], 1600)
        self.assertGreaterEqual(raw.position_ages[-1][1], 1.5)
        self.assertGreaterEqual(result["elapsed_s"], 1.8)

    def test_multi_axis_write_delays_do_not_shorten_the_last_axes_t(self):
        result, raw = self.run_phase(write_delay=0.2, targets={0: 1600, 1: 1400})
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(
            [item["feedback_p"] for item in result["phases"][0]["end_active"]],
            [1600, 1400],
        )
        self.assertGreaterEqual(raw.position_ages[-1][1], 1.5)
        self.assertGreaterEqual(result["elapsed_s"], 1.9)

    def test_delayed_send_remains_inside_the_original_whole_task_deadline(self):
        result, raw = self.run_phase(log_delay=2, deadline_ms=3000)
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        self.assertIn("deadline", result["error"])
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)

    def test_slow_reads_skip_overdue_samples_instead_of_catching_up_all_old_slots(self):
        result, _ = self.run_phase(query_delay=0.2)
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(len(result["phases"][0]["samples"]), 1)
