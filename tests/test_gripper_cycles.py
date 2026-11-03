"""固定夹爪周期的纯内存检查；不打开串口、不访问相机、不创建实物执行目录。"""

from contextlib import redirect_stdout
from io import StringIO
import unittest

from arm_project.arm_link import ServoLink
from arm_project.cli.run_gripper_cycles import COMMANDS, CycleTransport, run_cycle
from test_arm_driver import FakeClock
from test_joint_probe import ProbeDevice


class FakeRunDir:
    def __init__(self):
        self.cancelled = False

    def __truediv__(self, name):
        if name != "cancel.request":
            raise AssertionError("unexpected filesystem access")
        return self

    def exists(self):
        return self.cancelled


class GripperCycleTests(unittest.TestCase):
    def setup_cycle(self, scenario="normal", start=1500, reserve=None):
        clock = FakeClock()
        device = ProbeDevice(clock, scenario, start=start)
        reservations = []
        transport = CycleTransport(
            "NEVER_OPENED", reserve or (lambda i, c: reservations.append((i, c)))
        )
        transport.port = device
        transport.open = device.open  # 此测试即使进入context也只能打开内存设备。
        events = []

        def record(e, **v):
            events.append((e, v))

        link = ServoLink(transport, clock=clock, record=record)
        return link, device, FakeRunDir(), record, reservations

    def motions(self, device):
        return [command for command in device.writes if len(command) == 15 and b"T" in command]

    def execute(self, link, directory, record):
        with redirect_stdout(StringIO()):
            return run_cycle(link, directory, record)

    def test_exact_twenty_alternating_commands_and_no_twenty_first(self):
        link, device, _, _, reservations = self.setup_cycle()
        for command in COMMANDS:
            self.assertEqual(link.transport.write(command), 15)
        self.assertEqual(self.motions(device), list(COMMANDS))
        self.assertEqual([index for index, _ in reservations], list(range(20)))
        self.assertEqual(link.transport.index, 20)
        with self.assertRaises(ValueError):
            link.transport.write(COMMANDS[0])
        self.assertEqual(len(self.motions(device)), 20)

    def test_wrong_order_axis_broadcast_configuration_and_target_rejected(self):
        link, device, _, _, reservations = self.setup_cycle()
        for command in (
            COMMANDS[1],
            b"#004PRAD!",
            b"#000P1480T1000!",
            b"#255PRAD!",
            b"#005PMOD2!",
            b"#005PULK!",
            b"#005P1481T1000!",
            b"#005P1480T1500!",
            b"#000PDST!",
            COMMANDS[0] + b"\n",
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                link.transport.write(command)
        self.assertEqual(device.writes, [])
        self.assertEqual(reservations, [])
        self.assertEqual(link.transport.write(b"#005PRAD!"), 9)
        self.assertEqual(link.transport.index, 0)

    def test_reservation_happens_before_write_and_failure_blocks_motion(self):
        order = []
        link, device, _, _, _ = self.setup_cycle(reserve=lambda i, c: order.append(("reserve", i)))
        original_write = device.write

        def tracked_write(command):
            order.append(("write", command))
            return original_write(command)

        device.write = tracked_write
        link.transport.write(COMMANDS[0])
        self.assertEqual(order, [("reserve", 0), ("write", COMMANDS[0])])

        def denied(index, command):
            raise FileExistsError("persistent budget already consumed")

        link, device, _, _, _ = self.setup_cycle(reserve=denied)
        with self.assertRaises(FileExistsError):
            link.transport.write(COMMANDS[0])
        self.assertTrue(link.transport.halted)
        self.assertEqual(link.transport.index, 0)
        with self.assertRaises(ValueError):
            link.transport.write(COMMANDS[0])
        self.assertEqual(device.writes, [])

    def test_short_write_consumes_allowance_and_blocks_both_retry_and_next(self):
        link, device, _, _, reservations = self.setup_cycle("short_motion")
        self.assertEqual(link.transport.write(COMMANDS[0]), 14)
        self.assertEqual(link.transport.index, 1)
        self.assertTrue(link.transport.halted)
        for command in (COMMANDS[0], COMMANDS[1]):
            with self.assertRaises(ValueError):
                link.transport.write(command)
        self.assertEqual(len(reservations), 1)
        self.assertEqual(self.motions(device), [COMMANDS[0]])

    def test_full_ten_cycles_complete_only_as_observed_not_physical_verification(self):
        link, device, directory, record, reservations = self.setup_cycle()
        result = self.execute(link, directory, record)
        self.assertIsNone(result["error"])
        self.assertEqual(result["completed_cycles"], 10)
        self.assertEqual(len(result["steps"]), 20)
        self.assertEqual(result["status"], "TEN_CYCLES_OBSERVED_NOT_PHYSICALLY_VERIFIED")
        self.assertFalse(result["physical_motion_verified"])
        self.assertFalse(result["grasp_verified"])
        self.assertEqual(self.motions(device), list(COMMANDS))
        self.assertEqual(len(reservations), 20)
        self.assertNotIn(b"#005PDST!", device.writes)
        self.assertTrue(
            all(
                len(row["start_samples"]) == 3 and len(row["feedback"]) == 3
                for row in result["steps"]
            )
        )

    def test_unstable_preflight_over_step_wrong_mode_stop_before_motion(self):
        for scenario, start in (
            ("unstable", 1500),
            ("normal", 1590),
            ("wrong_mode", 1500),
            ("wrong_firmware", 1500),
        ):
            link, device, directory, record, _ = self.setup_cycle(scenario, start)
            result = self.execute(link, directory, record)
            with self.subTest(scenario=scenario, start=start):
                self.assertEqual(result["completed_cycles"], 0)
                self.assertEqual(result["status"], "ABORTED_REMAINING_COMMANDS_CANCELLED")
                self.assertIsNotNone(result["error"])
                self.assertEqual(self.motions(device), [])
                self.assertNotIn(b"#005PDST!", device.writes)

    def test_communication_failure_or_no_progress_aborts_without_following_step(self):
        for scenario in ("short_motion", "disconnect", "silent", "stuck"):
            link, device, directory, record, _ = self.setup_cycle(scenario)
            result = self.execute(link, directory, record)
            with self.subTest(scenario=scenario):
                self.assertEqual(result["completed_cycles"], 0)
                self.assertEqual(result["status"], "ABORTED_REMAINING_COMMANDS_CANCELLED")
                self.assertEqual(self.motions(device), [COMMANDS[0]])
                self.assertEqual(device.writes.count(b"#005PDST!"), 1)
                self.assertFalse(result["stop_attempt"]["physical_stop_verified"])
                self.assertTrue(link.transport.halted)

    def test_cancel_before_motion_does_not_append_any_motion(self):
        link, device, directory, record, _ = self.setup_cycle()
        directory.cancelled = True
        result = self.execute(link, directory, record)
        self.assertEqual(result["completed_cycles"], 0)
        self.assertEqual(self.motions(device), [])
        self.assertIn("InterruptedError", result["error"])

    def test_cancel_during_first_wait_sends_only_one_motion_and_one_local_stop(self):
        link, device, directory, record, _ = self.setup_cycle()

        def cancel_after_write():
            if device.motion:
                directory.cancelled = True

        link.clock.hook = cancel_after_write
        result = self.execute(link, directory, record)
        self.assertEqual(result["completed_cycles"], 0)
        self.assertEqual(self.motions(device), [COMMANDS[0]])
        self.assertEqual(device.writes.count(b"#005PDST!"), 1)
        self.assertNotIn(b"#000PDST!", device.writes)
        self.assertTrue(link.transport.halted)
        with self.assertRaises(ValueError):
            link.transport.write(COMMANDS[1])
        with self.assertRaises(RuntimeError):
            link.transport.write(b"#005PDST!")


if __name__ == "__main__":
    unittest.main()
