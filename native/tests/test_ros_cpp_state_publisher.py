"""第六课ROS+PTY集成检查：仅访问测试新建的伪终端与本机ROS话题。"""

import argparse
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import unittest
import uuid

ROS_AVAILABLE = False
ROS_IMPORT_ERROR = ""
if sys.platform == "linux":
    import fcntl
    import pty
    import termios

    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        from jixiebi_interfaces.msg import ServoStateRaw, ServoStatesRaw

        ROS_AVAILABLE = True
    except ImportError as error:
        ROS_IMPORT_ERROR = str(error)


BINARY = Path("ros2_ws/install/jixiebi_hardware/lib/jixiebi_hardware/servo_state_publisher")


@unittest.skipUnless(sys.platform == "linux" and ROS_AVAILABLE, "需要Linux及已构建ROS环境")
class StatePublisherTests(unittest.TestCase):
    """真正订阅C++节点；PTY主端模拟舵机，不用伪造的ROS消息替代节点输出。"""

    @classmethod
    def setUpClass(cls):
        # 测试进程及其C++子进程只用本机发现；独立域减少与开发中的节点互相影响。
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ.setdefault("ROS_DOMAIN_ID", str(20 + os.getpid() % 70))
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        suffix = uuid.uuid4().hex[:10]
        self.topic = f"/jixiebi_test_{suffix}/states"
        self.node = rclpy.create_node(f"cpp_state_subscriber_{suffix}")
        self.messages = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 订阅先于C++发布端建立；失败批次只发布一次，不靠持久化旧状态补消息。
        self.subscription = self.node.create_subscription(
            ServoStatesRaw, self.topic, self.messages.append, qos
        )
        self.processes = []
        self.wire_bytes = bytearray()
        self.consumed_bytes = 0

    def tearDown(self):
        for process in self.processes:
            self.stop_process(process)
        self.node.destroy_node()
        os.close(self.master_fd)
        os.close(self.slave_fd)

    def stop_process(self, process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            stdout, stderr = process.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate(timeout=2)
            self.fail(f"ROS节点未按期退出: {(stdout + stderr).decode('utf-8', errors='replace')}")
        return stdout.decode("utf-8"), stderr.decode("utf-8")

    def start_publisher(self, *, period_ms=500, timeout_ms=1000):
        process = subprocess.Popen(
            [
                str(BINARY),
                "--ros-args",
                "-r",
                f"/jixiebi/servo_states_cpp:={self.topic}",
                "-p",
                f"port:={self.slave_name}",
                "-p",
                "pty_simulation:=true",
                "-p",
                f"period_ms:={period_ms}",
                "-p",
                f"query_timeout_ms:={timeout_ms}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.processes.append(process)
        return process

    @staticmethod
    def query(servo_id):
        return f"#{servo_id:03d}PRAD!".encode("ascii")

    def pump(self, wait_s=0.01):
        readable, _, _ = select.select([self.master_fd], [], [], wait_s)
        if readable:
            self.wire_bytes.extend(os.read(self.master_fd, 4096))
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def pump_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not predicate() and time.monotonic() < deadline:
            self.pump()
        return predicate()

    def wait_for_query(self, process, servo_id):
        expected_end = self.consumed_bytes + 9
        found = self.pump_until(
            lambda: len(self.wire_bytes) >= expected_end or process.poll() is not None, 4.0
        )
        if not found or process.poll() is not None:
            stdout, stderr = self.stop_process(process)
            self.fail(f"没有等到编号{servo_id}的查询: {stdout}{stderr}")
        actual = bytes(self.wire_bytes[self.consumed_bytes : expected_end])
        self.assertEqual(actual, self.query(servo_id))
        self.consumed_bytes = expected_end

    def reply(self, servo_id, position_p):
        os.write(self.master_fd, f"#{servo_id:03d}P{position_p:04d}!".encode("ascii"))

    def serve_complete_batch(self, process, positions, *, fragmented=False):
        for index, position in enumerate(positions):
            self.wait_for_query(process, index)
            if fragmented and index == 2:
                # 回显/错编号均不能冒充002；合法半帧仍要跨read保留。
                os.write(self.master_fd, self.query(index))
                self.reply(1, 1999)
                frame = f"#{index:03d}P{position:04d}!".encode("ascii")
                os.write(self.master_fd, frame[:7])
                time.sleep(0.015)
                os.write(self.master_fd, frame[7:])
            else:
                self.reply(index, position)

    def assert_complete_message(self, message, positions):
        self.assertEqual(message.source, "PTY_SIMULATION")
        self.assertFalse(message.simultaneous)
        self.assertTrue(message.complete)
        self.assertEqual(message.valid_count, 6)
        self.assertGreater(message.stamp.sec, 0)
        self.assertEqual([sample.servo_id for sample in message.servos], list(range(6)))
        self.assertEqual([sample.position_p for sample in message.servos], positions)
        previous_finish = 0.0
        for sample in message.servos:
            self.assertEqual(sample.status, ServoStateRaw.OK)
            self.assertEqual(sample.error, "")
            self.assertTrue(math.isfinite(sample.elapsed_ms) and sample.elapsed_ms >= 0.0)
            self.assertTrue(
                math.isfinite(sample.finished_offset_ms)
                and sample.finished_offset_ms >= previous_finish
            )
            previous_finish = sample.finished_offset_ms
        self.assertTrue(math.isfinite(message.elapsed_ms) and message.elapsed_ms >= previous_finish)

    def test_subscriber_receives_repeated_typed_six_axis_messages(self):
        process = self.start_publisher()
        expected_by_sequence = {}
        served_batches = 0
        # 给发现过程留余量；验证至少两批真实消息，不强求订阅端必然收到第一帧。
        for batch_index in range(4):
            positions = [1510 + batch_index, 1503, 1493, 1497, 1503, 1480]
            expected_by_sequence[batch_index + 1] = positions
            self.serve_complete_batch(process, positions, fragmented=batch_index == 0)
            served_batches += 1
            if self.pump_until(lambda: len(self.messages) >= 2, 0.2):
                break
        self.assertGreaterEqual(len(self.messages), 2)
        first, second = self.messages[:2]
        self.assertGreater(second.sequence, first.sequence)
        for message in self.messages:
            self.assertIn(message.sequence, expected_by_sequence)
            self.assert_complete_message(message, expected_by_sequence[message.sequence])
        self.stop_process(process)
        self.pump(0.0)
        # 直接核对线上字节，不以节点的TX日志代替“没有重发/没有NUL”。
        self.assertEqual(
            bytes(self.wire_bytes),
            b"".join(self.query(index) for _ in range(served_batches) for index in range(6)),
        )

    def test_failed_batch_is_published_once_then_querying_pauses(self):
        process = self.start_publisher(period_ms=600, timeout_ms=200)
        positions = [1510, 1503, 1493, 1497, 1503, 1480]
        self.serve_complete_batch(process, positions)
        self.assertTrue(self.pump_until(lambda: bool(self.messages), 0.4))
        self.assert_complete_message(self.messages[0], positions)

        self.wait_for_query(process, 0)
        self.reply(0, 1511)
        self.wait_for_query(process, 1)
        self.reply(1, 1504)
        self.wait_for_query(process, 2)
        # 故意不回复002；先前完整批次已建立订阅，避免把发现延迟当失败消息丢失。
        self.assertTrue(self.pump_until(lambda: any(not m.complete for m in self.messages), 3.0))
        incomplete = next(message for message in self.messages if not message.complete)
        self.assertEqual(incomplete.valid_count, 2)
        self.assertEqual(incomplete.sequence, self.messages[0].sequence + 1)
        self.assertEqual([sample.position_p for sample in incomplete.servos[:2]], [1511, 1504])
        for sample in incomplete.servos[:2]:
            self.assertEqual(sample.status, ServoStateRaw.OK)
        self.assertEqual(incomplete.servos[2].status, ServoStateRaw.FAILED)
        self.assertIn("timeout", incomplete.servos[2].error.lower())
        for sample in incomplete.servos[2:]:
            self.assertEqual(sample.position_p, 0)
        for sample in incomplete.servos[3:]:
            self.assertEqual(sample.status, ServoStateRaw.NOT_QUERIED)

        expected_wire = b"".join(self.query(index) for index in range(6)) + b"".join(
            self.query(index) for index in range(3)
        )
        messages_before_pause = len(self.messages)
        # 大于一个period：失败后不能启动新批次，也不能发送002补查。
        deadline = time.monotonic() + 0.85
        while time.monotonic() < deadline:
            self.pump()
        self.assertIsNone(process.poll(), "节点应保持可检查状态，等待用户退出")
        self.assertEqual(bytes(self.wire_bytes), expected_wire)
        self.assertEqual(len(self.messages), messages_before_pause)
        # 暂停的节点仍持有串口锁，不能悄悄释放给第二个所有者。
        with self.assertRaises(BlockingIOError):
            fcntl.flock(self.slave_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_invalid_startup_parameters_do_not_query_or_change_tty_configuration(self):
        initial_attributes = termios.tcgetattr(self.slave_fd)
        fcntl.flock(self.slave_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # 已持锁时仍应先报告参数错误，而非触碰设备后才报告flock错误。
        for name, value in (("period_ms", 0), ("query_timeout_ms", 0)):
            with self.subTest(parameter=name):
                result = subprocess.run(
                    [
                        str(BINARY),
                        "--ros-args",
                        "-p",
                        f"port:={self.slave_name}",
                        "-p",
                        "pty_simulation:=true",
                        "-p",
                        f"{name}:={value}",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=4,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(name, result.stdout + result.stderr)
                self.pump(0.0)
                self.assertEqual(bytes(self.wire_bytes), b"")
                self.assertEqual(termios.tcgetattr(self.slave_fd), initial_attributes)

    def test_empty_default_port_fails_without_querying(self):
        result = subprocess.run([str(BINARY)], capture_output=True, text=True, timeout=4)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("port", result.stdout + result.stderr)
        self.pump(0.0)
        self.assertEqual(bytes(self.wire_bytes), b"")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=BINARY)
    args, remaining = parser.parse_known_args()
    BINARY = args.binary.resolve()
    if sys.platform == "linux" and not ROS_AVAILABLE:
        parser.error(f"先加载ROS和本项目install/setup.bash：{ROS_IMPORT_ERROR}")
    if sys.platform == "linux" and not BINARY.is_file():
        parser.error(f"请先构建C++节点：{BINARY}")
    unittest.main(argv=[sys.argv[0], *remaining])
