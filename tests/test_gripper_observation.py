"""夹爪延长只读观察的回归测试；模拟设备不会访问串口或实物。"""

from pathlib import Path
import shutil
import unittest
from uuid import uuid4

from arm_project.servo_protocol import validate_read_query
from arm_project.unified_arm import HardwareArmDriver, MoveSpec
from test_unified_arm import MemoryDevice, TestClock


ROOT = Path(__file__).resolve().parents[1]


class SettlingGripper(MemoryDevice):
    """首段模拟回读跨3P；之后稳定或持续波动，不模拟真实物理过程。"""

    def __init__(self, clock, *, stable):
        super().__init__(clock, positions=[1506, 1504, 1493, 1496, 1504, 1498])
        self.stable = stable
        self.post_reads = 0

    def write(self, command):
        if command == b"#005P1570T1000!":
            count = super().write(command)
            # MemoryDevice默认立即到达目标；这里改成等待后续读数脚本驱动。
            self.positions[5] = 1498
            return count
        if self.motions:
            try:
                axis, field = validate_read_query(command)
            except ValueError:
                axis, field = None, None
            if axis == 5 and field == "position":
                # 前11次构造方向正确但末5点跨3P的轨迹，用于复现原判定。
                self.post_reads += 1
                first_window = (1507, 1507, 1507, 1507, 1507, 1507, 1507, 1506, 1506, 1504, 1505)
                if self.post_reads <= len(first_window):
                    self.positions[5] = first_window[self.post_reads - 1]
                elif self.stable:
                    # 后续稳定于历史合爪反馈附近，距离目标仍为20P。
                    self.positions[5] = 1550
                else:
                    # 留在合法反馈窗口内，持续跨3P，不能被误判稳定。
                    self.positions[5] = 1547 if self.post_reads % 2 else 1550
        return super().write(command)


class GripperObservationTests(unittest.TestCase):
    def setUp(self):
        self.parent = ROOT / "artifacts"
        self.folder = self.parent / ("_gripper_observation_test_" + uuid4().hex)
        self.folder.mkdir(parents=True)
        self.addCleanup(self.cleanup)
        self.counter = 0

    def cleanup(self):
        # 只移除本测试独占创建的目录，绝不修改已有验收或实物日志。
        resolved = self.folder.resolve()
        if resolved.parent != self.parent.resolve() or not resolved.name.startswith(
            "_gripper_observation_test_"
        ):
            raise AssertionError("cleanup escaped the test directory")
        shutil.rmtree(resolved)

    def run_gripper(self, observation_s, *, stable):
        self.counter += 1
        clock = TestClock()
        device = SettlingGripper(clock, stable=stable)
        driver = HardwareArmDriver(
            "NEVER_OPENED",
            self.folder / f"session_{self.counter}",
            (MoveSpec(5, 1570, 1000),),
            transport=device,
            clock=clock,
            response_observation_s=observation_s,
        )
        with driver:
            result = driver.move_joint(5, 1570, 1000)
        self.assertEqual(device.motions, [b"#005P1570T1000!"])
        self.assertTrue(driver.closed)
        return result

    def test_default_unstable_then_five_second_observation_passes_without_exact_arrival(self):
        default = self.run_gripper(0, stable=True)
        self.assertEqual(default["status"], "FAILED")
        self.assertIn("post-motion tail is not stable", default["error"])
        self.assertEqual(
            [row["value"] for row in default["feedback"][-5:]], [1507, 1506, 1506, 1504, 1505]
        )

        extended = self.run_gripper(5, stable=True)
        self.assertEqual(extended["status"], "RESPONSE_OBSERVED")
        self.assertTrue(extended["response_pass"])
        self.assertFalse(extended["position_reached"])
        self.assertEqual(extended["final_p"], 1550)
        self.assertLessEqual(extended["tail_span_p"], 2)
        self.assertGreaterEqual(
            extended["feedback"][-1]["received_monotonic_s"] - extended["motion_write_monotonic_s"],
            5.0,
        )

    def test_five_second_observation_still_rejects_continued_three_p_wobble(self):
        result = self.run_gripper(5, stable=False)
        self.assertEqual(result["status"], "FAILED")
        self.assertFalse(result["response_pass"])
        self.assertIn("post-motion tail is not stable", result["error"])
        self.assertEqual(
            max(row["value"] for row in result["feedback"][-5:])
            - min(row["value"] for row in result["feedback"][-5:]),
            3,
        )


if __name__ == "__main__":
    unittest.main()
