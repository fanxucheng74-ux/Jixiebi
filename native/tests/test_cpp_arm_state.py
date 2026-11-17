"""第五课六轴查询的 Linux PTY 检查：只打开本测试创建的伪终端。"""

import argparse
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import unittest

if sys.platform == "linux":
    import pty


BINARY = Path("build/cpp_lesson05/read_arm_state")
SINGLE_BINARY = Path("build/cpp_lesson04/read_position")


@unittest.skipUnless(sys.platform == "linux", "PTY 检查仅在 Linux/WSL 运行")
class ArmStateTests(unittest.TestCase):
    """PTY 主端模拟六个地址，逐条核对请求与批次结果，不访问机械臂。"""

    def setUp(self):
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        self.processes = []
        self.wire_bytes = bytearray()
        self.consumed_bytes = 0

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.kill()
            process.communicate(timeout=2)
        if self.master_fd is not None:
            os.close(self.master_fd)
        os.close(self.slave_fd)

    @staticmethod
    def query(servo_id):
        return f"#{servo_id:03d}PRAD!".encode("ascii")

    def start_batch(self, timeout_ms=700):
        process = subprocess.Popen(
            [
                str(BINARY),
                "--port",
                self.slave_name,
                "--timeout-ms",
                str(timeout_ms),
                "--pty-simulation",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.processes.append(process)
        return process

    def collect_wire(self, wait_s=0.0):
        if self.master_fd is not None:
            readable, _, _ = select.select([self.master_fd], [], [], wait_s)
            if readable:
                self.wire_bytes.extend(os.read(self.master_fd, 4096))

    def wait_for_query(self, process, servo_id):
        deadline = time.monotonic() + 2.0
        expected_end = self.consumed_bytes + 9
        while len(self.wire_bytes) < expected_end and time.monotonic() < deadline:
            self.collect_wire(0.01)
            if process.poll() is not None:
                break
        actual = bytes(self.wire_bytes[self.consumed_bytes : expected_end])
        self.assertEqual(actual, self.query(servo_id))
        self.consumed_bytes = expected_end

    def reply(self, servo_id, position):
        os.write(self.master_fd, f"#{servo_id:03d}P{position:04d}!".encode("ascii"))

    def finish(self, process):
        # 保留主端直到子进程退出；只有断线测试会主动关闭主端。
        deadline = time.monotonic() + 3.0
        while process.poll() is None and time.monotonic() < deadline:
            if self.master_fd is None:
                time.sleep(0.01)
            else:
                self.collect_wire(0.01)
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=2)
            self.fail("六轴查询未在有限期限内退出")
        self.collect_wire()
        stdout, stderr = process.communicate(timeout=2)
        return process.returncode, stdout.decode("utf-8"), stderr.decode("utf-8")

    def samples(self, stdout):
        rows = []
        for line in stdout.splitlines():
            if line.startswith("ID="):
                rows.append(dict(token.split("=", 1) for token in line.split() if "=" in token))
        self.assertEqual(len(rows), 6, stdout)
        self.assertEqual([row["ID"] for row in rows], [f"{index:03d}" for index in range(6)])
        return rows

    def assert_wire(self, count):
        # 精确字节对照同时检出重发、NUL及额外运动帧，不能只数打印出来的 TX 行。
        self.assertEqual(
            bytes(self.wire_bytes), b"".join(self.query(index) for index in range(count))
        )

    def assert_failed_batch(self, process, failed_id, positions):
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertIn("BATCH_STATUS=INCOMPLETE", stdout)
        self.assertIn(f"VALID_COUNT={failed_id}", stdout)
        rows = self.samples(stdout)
        for index, row in enumerate(rows):
            if index < failed_id:
                self.assertEqual(row["STATUS"], "OK")
                self.assertEqual(row["P"], str(positions[index]))
            else:
                expected = "FAILED" if index == failed_id else "NOT_QUERIED"
                self.assertEqual(row["STATUS"], expected)
                self.assertEqual(row["P"], "--")
        self.assert_wire(failed_id + 1)
        return stdout, stderr

    def test_six_sequential_queries_accept_fragmented_matching_replies(self):
        process = self.start_batch()
        positions = [1510, 1500, 1492, 1498, 500, 2500]
        for index, position in enumerate(positions):
            self.wait_for_query(process, index)
            # 回显和错 ID 不能当作本轴结果，合法半帧必须跨 read 保留下来。
            wrong_id = (index + 1) % 6
            os.write(self.master_fd, b"noise" + self.query(index))
            self.reply(wrong_id, 1999)
            frame = f"#{index:03d}P{position:04d}!".encode("ascii")
            os.write(self.master_fd, frame[:7])
            time.sleep(0.01)
            os.write(self.master_fd, frame[7:])
        status, stdout, stderr = self.finish(process)
        self.assertEqual(status, 0, stdout + stderr)
        self.assertIn("SOURCE=PTY_SIMULATION", stdout)
        self.assertIn("SIMULTANEOUS=false", stdout)
        self.assertIn("BATCH_STATUS=OK", stdout)
        self.assertIn("VALID_COUNT=6", stdout)
        rows = self.samples(stdout)
        previous_finish_ms = 0.0
        for index, row in enumerate(rows):
            self.assertEqual(row["STATUS"], "OK")
            self.assertEqual(row["P"], str(positions[index]))
            elapsed_ms = float(row["ELAPSED_MS"])
            finished_ms = float(row["FINISHED_OFFSET_MS"])
            self.assertTrue(math.isfinite(elapsed_ms) and elapsed_ms >= 0.0)
            self.assertTrue(math.isfinite(finished_ms) and finished_ms >= previous_finish_ms)
            previous_finish_ms = finished_ms
        self.assert_wire(6)

    def test_first_timeout_does_not_invent_positions_or_query_later_axes(self):
        process = self.start_batch(timeout_ms=150)
        self.wait_for_query(process, 0)
        stdout, stderr = self.assert_failed_batch(process, 0, [])
        self.assertIn("timeout", (stdout + stderr).lower())

    def test_middle_timeout_preserves_earlier_samples_and_stops_without_retry(self):
        process = self.start_batch(timeout_ms=150)
        positions = [1510, 1500]
        for index, position in enumerate(positions):
            self.wait_for_query(process, index)
            self.reply(index, position)
        self.wait_for_query(process, 2)
        stdout, stderr = self.assert_failed_batch(process, 2, positions)
        self.assertIn("timeout", (stdout + stderr).lower())

    def test_lock_stays_owned_between_axis_queries(self):
        process = self.start_batch(timeout_ms=1000)
        self.wait_for_query(process, 0)
        self.reply(0, 1510)
        self.wait_for_query(process, 1)
        competitor = subprocess.run(
            [
                str(SINGLE_BINARY),
                "--port",
                self.slave_name,
                "--id",
                "2",
                "--pty-simulation",
            ],
            capture_output=True,
            text=True,
            timeout=2,
        )
        self.assertNotEqual(competitor.returncode, 0, competitor.stdout + competitor.stderr)
        self.assertIn("flock", competitor.stderr)
        self.reply(1, 1500)
        for index in range(2, 6):
            self.wait_for_query(process, index)
            self.reply(index, 1490 + index)
        status, stdout, stderr = self.finish(process)
        self.assertEqual(status, 0, stdout + stderr)
        self.assert_wire(6)

    def test_disconnection_preserves_partial_batch(self):
        process = self.start_batch(timeout_ms=1000)
        self.wait_for_query(process, 0)
        self.reply(0, 1510)
        self.wait_for_query(process, 1)
        os.close(self.master_fd)
        self.master_fd = None
        self.assert_failed_batch(process, 1, [1510])

    def test_no_arguments_and_help_only_show_usage(self):
        for arguments in ([], ["--help"]):
            with self.subTest(arguments=arguments):
                result = subprocess.run(
                    [str(BINARY), *arguments], capture_output=True, text=True, timeout=2
                )
                self.assertEqual(result.returncode == 0, arguments == ["--help"])
                self.collect_wire()
                self.assertEqual(bytes(self.wire_bytes), b"")

    def test_bad_cli_values_are_rejected_before_query(self):
        for arguments in (
            ["--timeout-ms", "0"],
            ["--timeout-ms", "5001"],
            ["--timeout-ms", "150abc"],
            ["--id", "2"],
            ["--unknown"],
        ):
            with self.subTest(arguments=arguments):
                result = subprocess.run(
                    [str(BINARY), "--port", self.slave_name, "--pty-simulation", *arguments],
                    capture_output=True,
                    text=True,
                    timeout=2,
                )
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                self.collect_wire()
                self.assertEqual(bytes(self.wire_bytes), b"")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=BINARY)
    parser.add_argument("--single-binary", type=Path, default=SINGLE_BINARY)
    args, remaining = parser.parse_known_args()
    BINARY = args.binary.resolve()
    SINGLE_BINARY = args.single_binary.resolve()
    if sys.platform == "linux" and (not BINARY.is_file() or not SINGLE_BINARY.is_file()):
        parser.error("请先构建第四课和第五课可执行程序")
    unittest.main(argv=[sys.argv[0], *remaining])
