"""控制链路离线验收；FakeDevice不导入串口，不访问实物。"""

from collections import deque
import json
from pathlib import Path
import unittest

from arm_project.arm_link import ReplayTransport, SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE, MotionController, load_limits
from arm_project.arm_state import ArmStateReader
from arm_project.servo_protocol import validate_read_query, validate_motion_command


ROOT = Path(__file__).resolve().parents[1]


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.hook = None

    def monotonic(self):
        return self.now

    def timestamp(self):
        return "SIMULATED-HOST-TIME-%.6f" % self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.hook:
            self.hook()


class FakeDevice:
    source = "SIMULATION"

    def __init__(self, clock, scenario="normal"):
        self.clock = clock
        self.scenario = scenario
        self.chunks = deque()
        self.writes = []
        self.motion = None
        self.closed = False
        self.reads = 0

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return sum(len(c) for c in self.chunks)

    def read(self, count):
        self.clock.sleep(0.001)
        if not self.chunks:
            return b""
        chunk = self.chunks.popleft()
        result = chunk[:count]
        if chunk[count:]:
            self.chunks.appendleft(chunk[count:])
        return result

    def write(self, command):
        self.writes.append(command)
        if command == b"#000PDST!":
            self.chunks.extend([b"#OK!"])
            return len(command)
        try:
            servo_id, field = validate_read_query(command)
        except ValueError:
            self.motion = validate_motion_command(command)
            return len(command) - 1 if self.scenario == "short_motion" else len(command)
        if self.scenario == "short_query":
            return len(command) - 1
        if self.scenario == "missing_2" and servo_id == 2:
            return len(command)
        if self.scenario == "disconnect" and self.motion:
            raise OSError("simulated unplug")
        if field == "version":
            value = b"@ " + EXPECTED_FIRMWARE.encode("ascii")
        elif field == "mode":
            value = b"MOD5" if self.scenario == "wrong_mode" else b"MOD1"
        elif field == "id":
            value = b"P"
        else:
            self.reads += 1
            p = [1512, 1503, 1492, 1497, 1495, 1521][servo_id]
            if self.scenario == "unstable" and self.reads == 2:
                p += 5
            if self.motion:
                p = self.motion[1]
                if self.scenario == "undershoot":
                    p -= 12
                if self.scenario == "stuck":
                    p = 1512
                if self.scenario == "out_of_window":
                    p = 1560
            value = ("P%04d" % p).encode("ascii")
        frame = ("#%03d" % servo_id).encode("ascii") + value + b"!"
        if self.scenario == "wrong_then_right":
            self.chunks.append(b"#005P1500!#000PRAD!#000P9999!")
        if self.scenario == "late":
            self.clock.sleep(0.6)
        self.chunks.extend([frame[:1], frame[1:]])
        return len(command)


class DriverTests(unittest.TestCase):
    def link(self, scenario="normal"):
        clock = FakeClock()
        device = FakeDevice(clock, scenario)
        events = []
        link = ServoLink(device, clock=clock, record=lambda e, **v: events.append((e, v)))
        return link, device, events

    def controller(self, scenario="normal", before=None):
        link, device, events = self.link(scenario)
        controller = MotionController(
            link, load_limits(ROOT / "configs" / "joint_limits.json"), before_motion=before
        )
        return controller, device, events

    def motion_writes(self, device):
        return [w for w in device.writes if len(w) == 15 and b"T" in w]

    def test_replay_synthetic_state(self):
        fixture = json.loads(
            (ROOT / "data/fixtures/position_replay_20261031.json").read_text(encoding="utf-8")
        )
        transport = ReplayTransport(fixture["exchanges"])
        with ServoLink(transport, clock=FakeClock()) as link:
            state = ArmStateReader(link).read_state()
        self.assertEqual(state["source"], "REPLAY")
        self.assertEqual(state["status"], "OK")
        self.assertEqual(
            [s["position_p"] for s in state["servos"]], [1500, 1501, 1502, 1503, 1504, 1505]
        )
        self.assertFalse(state["simultaneous"])
        self.assertGreater(
            state["servos"][0]["age_ms_at_finish"], state["servos"][-1]["age_ms_at_finish"]
        )
        self.assertEqual(len(transport.writes), 6)

    def test_fragmented_six_axis_state_and_port_cleanup(self):
        link, device, _ = self.link()
        with link:
            state = ArmStateReader(link).read_state()
        self.assertTrue(device.closed)
        self.assertEqual(state["status"], "OK")
        self.assertEqual(device.writes, [("#%03dPRAD!" % i).encode() for i in range(6)])

    def test_identity_then_state_uses_one_session_and_24_read_queries(self):
        link, device, _ = self.link()
        with link:
            reader = ArmStateReader(link)
            identities = reader.read_identity()
            state = reader.read_state()
        self.assertEqual(len(identities), 6)
        self.assertTrue(
            all(i["replies"]["version"]["value"] == EXPECTED_FIRMWARE for i in identities)
        )
        self.assertTrue(all(i["replies"]["mode"]["value"] == 1 for i in identities))
        self.assertEqual(state["status"], "OK")
        self.assertEqual(len(device.writes), 24)
        for command in device.writes:
            validate_read_query(command)

    def test_midway_failure_keeps_partial_results_without_retry(self):
        link, device, _ = self.link("missing_2")
        state = ArmStateReader(link).read_state()
        self.assertEqual(state["status"], "INCOMPLETE")
        self.assertEqual(
            [s["status"] for s in state["servos"]],
            ["OK", "OK", "READ_FAILED", "NOT_READ", "NOT_READ", "NOT_READ"],
        )
        self.assertEqual(
            [s["position_p"] for s in state["servos"]], [1512, 1503, None, None, None, None]
        )
        self.assertEqual(len(device.writes), 3)
        ArmStateReader(link).read_state()
        self.assertEqual(len(device.writes), 3)

    def test_short_query_never_retried(self):
        link, device, _ = self.link("short_query")
        state = ArmStateReader(link).read_state()
        self.assertEqual(state["servos"][0]["status"], "READ_FAILED")
        self.assertEqual(len(device.writes), 1)

    def test_wrong_id_echo_and_bad_value_do_not_update_position(self):
        link, _, events = self.link("wrong_then_right")
        reply = link.query(0)
        self.assertEqual(reply["value"], 1512)
        self.assertEqual(sum(e == "rejected" for e, _ in events), 3)

    def test_late_reply_does_not_extend_deadline(self):
        link, device, _ = self.link("late")
        with self.assertRaises(TimeoutError):
            link.query(0)
        self.assertEqual(len(device.writes), 1)

    def test_pending_old_frame_is_not_accepted(self):
        link, device, events = self.link()
        device.chunks.append(b"#000P1000!")
        self.assertEqual(link.query(0)["value"], 1512)
        self.assertTrue(any(e == "pending_discarded" for e, _ in events))

    def test_disabled_axis_and_outside_target_rejected_before_io(self):
        for params in ((1, 1500, 1500), (0, 1600, 1500), (0, 1550, 100)):
            controller, device, _ = self.controller()
            with self.subTest(params=params), self.assertRaises(ValueError):
                controller.move_joint(*params)
            self.assertEqual(device.writes, [])

    def test_normal_motion_is_single_write_and_five_samples(self):
        controller, device, _ = self.controller()
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "TARGET_REACHED")
        self.assertEqual(len(result["feedback"]), 5)
        self.assertEqual(self.motion_writes(device), [b"#000P1550T1500!"])
        self.assertFalse(result["physical_stop_verified"])
        with self.assertRaises(RuntimeError):
            controller.move_joint(0, 1512)
        self.assertEqual(len(self.motion_writes(device)), 1)

    def test_actual_like_undershoot_is_reported_without_compensation(self):
        controller, device, _ = self.controller("undershoot")
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "OBSERVED_NOT_AT_TARGET")
        self.assertEqual(result["error_p"], 12)
        self.assertTrue(result["directed_feedback_change"])
        self.assertFalse(result["target_reached"])
        self.assertEqual(len(self.motion_writes(device)), 1)

    def test_stuck_feedback_not_called_motion_success(self):
        controller, device, _ = self.controller("stuck")
        result = controller.move_joint(0, 1550)
        self.assertFalse(result["directed_feedback_change"])
        self.assertEqual(result["status"], "OBSERVED_NOT_AT_TARGET")
        self.assertEqual(len(self.motion_writes(device)), 1)

    def test_wrong_mode_and_unstable_start_prevent_motion(self):
        for scenario in ("wrong_mode", "unstable"):
            controller, device, _ = self.controller(scenario)
            result = controller.move_joint(0, 1550)
            self.assertEqual(result["status"], "PREFLIGHT_FAILED")
            self.assertEqual(self.motion_writes(device), [])

    def test_persistent_attempt_marker_failure_prevents_write(self):
        def claimed(_):
            raise FileExistsError("prior attempt exists")

        controller, device, _ = self.controller(before=claimed)
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "PREFLIGHT_FAILED")
        self.assertEqual(self.motion_writes(device), [])
        self.assertTrue(controller.used)

    def test_short_motion_disconnect_or_bad_feedback_lock_session(self):
        for scenario in ("short_motion", "disconnect", "out_of_window"):
            controller, device, _ = self.controller(scenario)
            result = controller.move_joint(0, 1550)
            self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
            self.assertEqual(len(self.motion_writes(device)), 1)
            self.assertEqual(device.writes.count(b"#000PDST!"), 1)
            self.assertFalse(result["physical_stop_verified"])
            with self.assertRaises(RuntimeError):
                controller.move_joint(0, 1512)

    def test_cancel_before_write_has_no_stop_or_motion(self):
        controller, device, _ = self.controller()
        controller.cancel()
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "CANCELLED_BEFORE_MOTION")
        self.assertEqual(device.writes, [])

    def test_cancel_after_write_attempts_stop_but_does_not_claim_stop(self):
        controller, device, _ = self.controller()

        def cancel_after_command():
            if device.motion:
                controller.cancel()

        device.clock.hook = cancel_after_command
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "CANCELLED_STOP_UNVERIFIED")
        self.assertEqual(result["stop_attempt"], "ACK_RECEIVED_UNVERIFIED")
        self.assertFalse(result["physical_stop_verified"])
        self.assertEqual(len(self.motion_writes(device)), 1)

    def test_transport_hard_limits_block_unreviewed_commands(self):
        clock = FakeClock()
        device = FakeDevice(clock)
        transport = SerialTransport("NEVER_OPENED")
        transport.port = device
        for command in (
            b"#001P1500T1500!",
            b"#000P2000T1500!",
            b"#000PMOD5!",
            b"#255PRAD!",
            b"#000PULK!",
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                transport.write(command)
        self.assertEqual(device.writes, [])

    def test_enter_logging_failure_still_closes_port(self):
        clock = FakeClock()
        device = FakeDevice(clock)

        def broken_log(*args, **kwargs):
            raise OSError("disk full")

        with self.assertRaisesRegex(OSError, "disk full"):
            with ServoLink(device, clock=clock, record=broken_log):
                self.fail("must not enter")
        self.assertTrue(device.closed)
        self.assertEqual(device.writes, [])

    def test_cancel_during_persistence_prevents_motion(self):
        controller, device, _ = self.controller()
        controller.before_motion = lambda plan: controller.cancel()
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "CANCELLED_BEFORE_MOTION")
        self.assertTrue(controller.used)
        self.assertEqual(self.motion_writes(device), [])

    def test_logging_failure_after_motion_does_not_block_stop(self):
        controller, device, _ = self.controller()

        def failing_after_motion(event, **values):
            if device.motion:
                raise OSError("disk full after motion")

        controller.link.record = failing_after_motion
        result = controller.move_joint(0, 1550)
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        self.assertIn("disk full after motion", result["error"])
        self.assertEqual(device.writes.count(b"#000PDST!"), 1)
        self.assertTrue(result["logging_errors"])

    def test_hardware_controller_requires_budget_and_cannot_widen_limits(self):
        link, device, _ = self.link()
        device.source = "HARDWARE"
        limits = load_limits(ROOT / "configs" / "joint_limits.json")
        with self.assertRaises(ValueError):
            MotionController(link, limits)
        limits[0]["min_p"] = 500
        with self.assertRaises(ValueError):
            MotionController(link, limits, before_motion=lambda plan: None)
        self.assertEqual(device.writes, [])


if __name__ == "__main__":
    unittest.main()
