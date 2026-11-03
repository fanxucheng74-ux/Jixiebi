"""内存设备验证逐轴探测门槛；任何窗口/数值都仅是测试夹具。"""

from collections import deque
from dataclasses import FrozenInstanceError
import unittest

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.joint_probe import ProbeSpec, ExactProbeTransport, JointProbeController
from arm_project.servo_protocol import validate_read_query, validate_motion_command
from test_arm_driver import FakeClock


def spec(**changes):
    return ProbeSpec.from_dict(
        {
            "review_id": "SIMULATION-ONLY-REVIEW",
            "reviewed_for_motion": True,
            "evidence": "unit-test fixture, not a real joint review",
            "servo_id": 5,
            "min_p": 1490,
            "max_p": 1550,
            "start_min_p": 1498,
            "start_max_p": 1502,
            "target_p": 1530,
            "duration_ms": 1500,
            "max_step_p": 40,
            **changes,
        }
    )


def timing_spec(**changes):
    # 复核精确的例外门槛；这里的review_id是测试输入，不会开设备。
    return spec(
        **{
            "review_id": "gripper_timing_1000_20261031_01",
            "servo_id": 5,
            "min_p": 1435,
            "max_p": 1510,
            "start_min_p": 1490,
            "start_max_p": 1494,
            "target_p": 1450,
            "max_step_p": 50,
            "duration_ms": 1000,
            **changes,
        }
    )


class ProbeDevice:
    source = "SIMULATION"

    def __init__(self, clock, scenario="normal", start=1500):
        self.clock, self.scenario, self.start = clock, scenario, start
        self.pending = deque()
        self.writes, self.motion = [], None
        self.position_reads = 0
        self.closed = False

    @property
    def in_waiting(self):
        return sum(len(chunk) for chunk in self.pending)

    def read(self, count):
        self.clock.sleep(0.001)
        if not self.pending:
            return b""
        chunk = self.pending.popleft()
        if chunk[count:]:
            self.pending.appendleft(chunk[count:])
        return chunk[:count]

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    def write(self, command):
        self.writes.append(command)
        if command.endswith(b"PDST!"):
            if self.scenario == "no_stop_ack":
                return len(command)
            if self.scenario == "stop_write_failed":
                raise OSError("simulated stop write failure")
            self.pending.append(b"#OK!")
            return len(command)
        try:
            axis, field = validate_read_query(command)
        except ValueError:
            self.motion = validate_motion_command(command)
            if self.scenario == "write_exception":
                raise OSError("simulated motion write error")
            return len(command) - 1 if self.scenario == "short_motion" else len(command)
        if self.scenario == "disconnect" and self.motion:
            raise OSError("simulated unplug")
        if self.scenario == "silent" and self.motion:
            return len(command)
        if self.scenario == "short_query":
            return len(command) - 1
        if field == "version":
            version = "UNEXPECTED" if self.scenario == "wrong_firmware" else EXPECTED_FIRMWARE
            body = b"@ " + version.encode("ascii")
        elif field == "mode":
            body = b"MOD5" if self.scenario == "wrong_mode" else b"MOD1"
        else:
            self.position_reads += 1
            p = self.start
            if self.scenario == "unstable" and self.position_reads == 2:
                p += 4
            if self.motion:
                p = self.motion[1]
                if self.scenario == "undershoot":
                    p -= 12
                elif self.scenario in ("stuck", "no_stop_ack", "stop_write_failed"):
                    p = self.start
                elif self.scenario == "two_p_only":
                    p = self.start + 2
                elif self.scenario == "opposite":
                    p = self.start - 5
                elif self.scenario == "out_of_window":
                    p = 1560
                elif self.scenario == "unstable_tail":
                    p += 2 if self.position_reads % 2 else -2
            body = ("P%04d" % p).encode("ascii")
        frame = ("#%03d" % axis).encode("ascii") + body + b"!"
        self.pending.extend((frame[:1], frame[1:]))
        return len(command)


class ProbeSpecTests(unittest.TestCase):
    def test_valid_spec_is_immutable_and_commands_follow_its_id(self):
        value = spec()
        self.assertEqual(value.command, b"#005P1530T1500!")
        self.assertEqual(value.stop_command, b"#005PDST!")
        with self.assertRaises(FrozenInstanceError):
            value.servo_id = 0

    def test_missing_review_unknown_keys_and_unsafe_parameters_rejected(self):
        for changes in (
            {"review_id": " "},
            {"reviewed_for_motion": False},
            {"reviewed_for_motion": 1},
            {"evidence": ""},
            {"servo_id": True},
            {"servo_id": 6},
            {"min_p": 499},
            {"max_p": 2501},
            {"min_p": 1550},
            {"start_min_p": 1489},
            {"start_max_p": 1551},
            {"start_min_p": 1503},
            {"target_p": 1551},
            {"target_p": float("nan")},
            {"duration_ms": 1499},
            {"duration_ms": 5001},
            {"max_step_p": 51},
            {"max_step_p": True},
            {"tolerance_p": 4},
            {"extra": 1},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                spec(**changes)
        with self.assertRaises(ValueError):
            ProbeSpec.from_dict({"servo_id": 5})

    def test_timing_exception_requires_every_exact_reviewed_field(self):
        value = timing_spec()
        self.assertEqual(value.command, b"#005P1450T1000!")
        changes = [{"servo_id": axis} for axis in range(5)] + [
            {"review_id": "gripper_timing_1000_20261031_02"},
            {"review_id": "gripper_timing_1000_20261031_01 "},
            {"review_id": "SIMULATION-ONLY-REVIEW"},
            {"target_p": 1449},
            {"target_p": 1451},
            {"min_p": 1434},
            {"min_p": 1436},
            {"max_p": 1509},
            {"max_p": 1511},
            {"max_step_p": 49},
            {"max_step_p": 51},
            {"duration_ms": 999},
            {"duration_ms": 1001},
            {"duration_ms": 1499},
            {"duration_ms": 1000.0},
            {"max_step_p": 50.0},
            {"reviewed_for_motion": 1},
            {"tolerance_p": 4},
        ]
        for override in changes:
            with self.subTest(override=override), self.assertRaises(ValueError):
                timing_spec(**override)

    def test_ordinary_specs_still_reject_sub_1500_durations_on_every_axis(self):
        for axis in range(6):
            for duration in (1000, 1499):
                with self.subTest(axis=axis, duration=duration), self.assertRaises(ValueError):
                    spec(servo_id=axis, duration_ms=duration)
        self.assertEqual(spec(duration_ms=1500).duration_ms, 1500)
        self.assertEqual(spec(duration_ms=5000).duration_ms, 5000)


class ExactTransportTests(unittest.TestCase):
    def transport(self, scenario="normal"):
        device = ProbeDevice(FakeClock(), scenario)
        transport = ExactProbeTransport("NEVER_OPENED", spec())
        transport.port = device
        return transport, device

    def test_only_exact_motion_and_own_axis_stop_are_permitted_once(self):
        transport, device = self.transport()
        for command in (
            b"#005P1531T1500!",
            b"#000P1530T1500!",
            b"#000PDST!",
            b"#005PMOD5!",
            b"#255PRAD!",
            b"#005PULK!",
            b"#005P1530T1500!\r\n",
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                transport.write(command)
        self.assertEqual(device.writes, [])
        self.assertEqual(transport.write(b"#002PRAD!"), 9)
        self.assertEqual(transport.write(spec().command), 15)
        with self.assertRaises(RuntimeError):
            transport.write(spec().command)
        transport.write(spec().stop_command)
        with self.assertRaises(RuntimeError):
            transport.write(spec().stop_command)
        self.assertEqual(device.writes.count(spec().command), 1)

    def test_short_write_or_exception_always_consumes_motion_allowance(self):
        for scenario in ("short_motion", "write_exception"):
            transport, device = self.transport(scenario)
            try:
                transport.write(spec().command)
            except OSError:
                pass
            with self.subTest(scenario=scenario), self.assertRaises(RuntimeError):
                transport.write(spec().command)
            self.assertEqual(device.writes, [spec().command])

    def test_stop_before_motion_prevents_later_start(self):
        transport, device = self.transport()
        transport.write(spec().stop_command)
        with self.assertRaises(RuntimeError):
            transport.write(spec().command)
        self.assertEqual(device.writes, [spec().stop_command])


class JointProbeTests(unittest.TestCase):
    def test_wait_handles_clock_crossing_deadline_without_negative_sleep(self):
        controller, device, _ = self.controller()
        clock = controller.link.clock
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
        controller._wait(0.01)
        self.assertTrue(sleeps)
        self.assertEqual(device.writes, [])

    def controller(
        self, scenario="normal", probe=None, before=None, hardware=False, feedback_samples=11
    ):
        clock = FakeClock()
        device = ProbeDevice(clock, scenario)
        probe = probe or spec()
        events = []
        transport = device
        if hardware:
            transport = ExactProbeTransport("NEVER_OPENED", probe)
            transport.port = device
            # 仅测试硬件后端的白名单/预算，绝不调用真实serial.open。
            transport.open = device.open
        link = ServoLink(transport, clock=clock, record=lambda e, **v: events.append((e, v)))
        controller = JointProbeController(
            link, probe, before_motion=before, feedback_samples=feedback_samples
        )
        return controller, device, events

    def motions(self, device):
        return [c for c in device.writes if len(c) == 15 and b"T" in c]

    def test_arbitrary_id5_normal_motion_observes_11_points_and_is_one_shot(self):
        claims = []
        controller, device, events = self.controller(before=claims.append, hardware=True)
        with controller.link:
            result = controller.run()
        self.assertEqual(result["status"], "TARGET_REACHED")
        self.assertEqual(result["final_p"], 1530)
        self.assertEqual(len(result["feedback"]), 11)
        self.assertGreaterEqual(result["observation_span_s"], 5)
        self.assertTrue(result["directed_feedback_change"])
        self.assertFalse(result["physical_stop_verified"])
        self.assertTrue(device.closed)
        self.assertEqual(self.motions(device), [b"#005P1530T1500!"])
        self.assertEqual(claims[0]["review_id"], "SIMULATION-ONLY-REVIEW")
        self.assertEqual(claims[0]["start_p"], 1500)
        with self.assertRaises(RuntimeError):
            controller.run()

    def test_exact_timing_exception_and_31_sample_observation_use_one_motion(self):
        claims = []
        controller, device, events = self.controller(
            probe=timing_spec(), before=claims.append, hardware=True, feedback_samples=31
        )
        device.start = 1492  # 内存设备的模拟起点，不是读取实物。
        result = controller.run()
        self.assertEqual(result["status"], "TARGET_REACHED")
        self.assertEqual(result["feedback_samples_requested"], 31)
        self.assertEqual(len(result["feedback"]), 31)
        self.assertGreaterEqual(result["observation_span_s"], 15)
        motion_time = next(value["monotonic_s"] for event, value in events if event == "motion_tx")
        self.assertGreaterEqual(result["feedback"][0]["received_monotonic_s"] - motion_time, 1.3)
        self.assertEqual(self.motions(device), [b"#005P1450T1000!"])
        self.assertEqual(len(claims), 1)
        self.assertEqual(result["final_p"], 1450)
        with self.assertRaises(RuntimeError):
            controller.run()
        with self.assertRaises(RuntimeError):
            controller.link.transport.write(b"#005P1450T1000!")
        self.assertEqual(len(self.motions(device)), 1)

    def test_invalid_observation_counts_rejected_before_any_io(self):
        clock = FakeClock()
        device = ProbeDevice(clock)
        link = ServoLink(device, clock=clock)
        for value in (-1, 0, 10, 62, True, False, 11.0, float("nan"), float("inf"), "31", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                JointProbeController(link, spec(), feedback_samples=value)
            self.assertEqual(device.writes, [])

    def test_long_observation_preserves_failure_and_has_a_hard_upper_bound(self):
        for samples, scenario in ((31, "undershoot"), (61, "stuck")):
            controller, device, _ = self.controller(scenario, feedback_samples=samples)
            result = controller.run()
            with self.subTest(samples=samples, scenario=scenario):
                self.assertEqual(
                    result["status"],
                    "FAILED_AFTER_ATTEMPT" if scenario == "stuck" else "OBSERVED_NOT_AT_TARGET",
                )
                self.assertFalse(result["target_reached"])
                self.assertEqual(len(result["feedback"]), samples)
                self.assertEqual(len(self.motions(device)), 1)
                self.assertGreaterEqual(result["observation_span_s"], (samples - 1) * 0.5)

    def test_cancellation_still_interrupts_extended_observation(self):
        controller, device, _ = self.controller(feedback_samples=31)
        device.clock.hook = lambda: (
            controller.cancel() if device.motion and device.position_reads >= 20 else None
        )
        result = controller.run()
        self.assertEqual(result["status"], "CANCELLED_STOP_UNVERIFIED")
        self.assertLess(len(result["feedback"]), 31)
        self.assertEqual(len(self.motions(device)), 1)
        self.assertEqual(device.writes.count(b"#005PDST!"), 1)

    def test_hardware_requires_matching_exact_transport_and_persistent_budget(self):
        with self.assertRaises(ValueError):
            self.controller(hardware=True)
        base = SerialTransport("NEVER_OPENED")
        with self.assertRaises(ValueError):
            JointProbeController(ServoLink(base), spec(), before_motion=lambda p: None)
        exact = ExactProbeTransport("NEVER_OPENED", spec())
        with self.assertRaises(ValueError):
            JointProbeController(ServoLink(exact), spec(servo_id=4), before_motion=lambda p: None)

    def test_mode_firmware_instability_start_interval_and_step_prevent_motion(self):
        for scenario, probe in (
            ("wrong_mode", spec()),
            ("wrong_firmware", spec()),
            ("unstable", spec(start_min_p=1490, start_max_p=1510)),
            ("normal", spec(start_min_p=1501)),
            ("normal", spec(max_step_p=20)),
        ):
            controller, device, _ = self.controller(scenario, probe)
            result = controller.run()
            self.assertEqual(result["status"], "PREFLIGHT_FAILED")
            self.assertEqual(self.motions(device), [])
            self.assertFalse(any(c.endswith(b"PDST!") for c in device.writes))

    def test_budget_failure_and_cancel_in_budget_never_write_motion(self):
        controller, device, _ = self.controller()
        controller.before_motion = lambda plan: (_ for _ in ()).throw(FileExistsError("used"))
        self.assertEqual(controller.run()["status"], "PREFLIGHT_FAILED")
        self.assertEqual(self.motions(device), [])
        controller, device, _ = self.controller()
        controller.before_motion = lambda plan: controller.cancel()
        self.assertEqual(controller.run()["status"], "CANCELLED_BEFORE_MOTION")
        self.assertEqual(self.motions(device), [])

    def test_cancel_before_queries_has_no_io(self):
        controller, device, _ = self.controller()
        controller.cancel()
        self.assertEqual(controller.run()["status"], "CANCELLED_BEFORE_MOTION")
        self.assertEqual(device.writes, [])

    def test_existing_logging_error_prevents_any_io(self):
        controller, device, _ = self.controller()
        controller.link.log_errors.append("prior session log failure")
        with self.assertRaises(RuntimeError):
            controller.run()
        self.assertEqual(device.writes, [])

    def test_cancel_during_wait_stops_only_current_axis_and_ack_is_not_physical_stop(self):
        controller, device, _ = self.controller()
        device.clock.hook = lambda: controller.cancel() if device.motion else None
        result = controller.run()
        self.assertEqual(result["status"], "CANCELLED_STOP_UNVERIFIED")
        self.assertEqual(result["stop_attempt"], "ACK_RECEIVED_UNVERIFIED")
        self.assertFalse(result["physical_stop_verified"])
        self.assertEqual(device.writes.count(b"#005PDST!"), 1)
        self.assertNotIn(b"#000PDST!", device.writes)

    def test_short_motion_disconnect_silence_outside_window_and_reverse_fault(self):
        for scenario in ("short_motion", "disconnect", "silent", "out_of_window", "opposite"):
            controller, device, _ = self.controller(scenario)
            result = controller.run()
            with self.subTest(scenario=scenario):
                self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
                self.assertEqual(len(self.motions(device)), 1)
                self.assertEqual(device.writes.count(b"#005PDST!"), 1)
                self.assertTrue(controller.link.faulted)
                self.assertFalse(result["target_reached"])

    def test_logging_failure_does_not_prevent_targeted_stop_or_claim_success(self):
        for fail_event in ("rx", "probe_result"):
            controller, device, _ = self.controller()

            def bad_log(event, **values):
                if device.motion and event == fail_event:
                    raise OSError("simulated disk full")

            controller.link.record = bad_log
            result = controller.run()
            with self.subTest(event=fail_event):
                self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
                self.assertFalse(result["target_reached"])
                self.assertEqual(device.writes.count(b"#005PDST!"), 1)
                self.assertTrue(controller.link.faulted)

    def test_stable_residual_stuck_and_unstable_tail_not_success(self):
        for scenario in ("undershoot", "stuck", "unstable_tail"):
            controller, device, _ = self.controller(scenario)
            result = controller.run()
            with self.subTest(scenario=scenario):
                self.assertEqual(
                    result["status"],
                    "FAILED_AFTER_ATTEMPT" if scenario == "stuck" else "OBSERVED_NOT_AT_TARGET",
                )
                self.assertFalse(result["target_reached"])
                self.assertEqual(len(result["feedback"]), 11)
                self.assertEqual(len(self.motions(device)), 1)

    def test_only_two_p_change_is_a_fault_not_a_stable_residual(self):
        controller, device, _ = self.controller("two_p_only")
        result = controller.run()
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        self.assertEqual(result["observed_delta_p"], 2)
        self.assertFalse(result["response_pass"])
        self.assertFalse(result["stable_residual_observed"])
        self.assertTrue(controller.link.faulted)
        self.assertEqual(device.writes.count(b"#005PDST!"), 1)
        self.assertEqual(len(self.motions(device)), 1)

    def test_no_motion_when_already_within_tolerance_consumes_controller(self):
        controller, device, _ = self.controller(probe=spec(target_p=1502))
        result = controller.run()
        self.assertEqual(result["status"], "NO_MOTION_NEEDED")
        self.assertEqual(self.motions(device), [])
        with self.assertRaises(RuntimeError):
            controller.run()

    def test_stop_failure_or_silence_remains_unverified_and_never_retried(self):
        for scenario in ("no_stop_ack", "stop_write_failed"):
            controller, device, _ = self.controller(scenario)
            device.clock.hook = lambda: controller.cancel() if device.motion else None
            result = controller.run()
            self.assertFalse(result["physical_stop_verified"])
            self.assertEqual(device.writes.count(b"#005PDST!"), 1)
            self.assertNotEqual(result["stop_attempt"], "ACK_RECEIVED_UNVERIFIED")


if __name__ == "__main__":
    unittest.main()
