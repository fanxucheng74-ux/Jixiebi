"""第四课的 Linux PTY 集成检查：只连接本测试新建的伪终端，不访问实物。"""

import argparse
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time
import unittest

if sys.platform == "linux":
    import fcntl
    import pty
    import termios


BINARY = Path("build/cpp_lesson04/read_position")


@unittest.skipUnless(sys.platform == "linux", "PTY 检查仅在 Linux/WSL 运行")
class PositionReaderTests(unittest.TestCase):
    """把 PTY 主端当成模拟舵机，检查 C++ 程序的真实字节读写。"""

    def setUp(self):
        self.master_fd, self.slave_fd = pty.openpty()
        # 设备名只能来自自己刚创建的 PTY，不能通过参数指定实物设备。
        self.slave_name = os.ttyname(self.slave_fd)
        self.processes = []
        self.wire_bytes = bytearray()

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.kill()
            process.communicate(timeout=2)
        if self.master_fd is not None:
            os.close(self.master_fd)
        os.close(self.slave_fd)

    def start_reader(self, *, servo_id="2", timeout_ms=700):
        # --pty-simulation 只用于模拟，不能把未验证的电气控制线当成实测通过。
        command = [
            str(BINARY),
            "--port",
            self.slave_name,
            "--id",
            str(servo_id),
            "--timeout-ms",
            str(timeout_ms),
            "--pty-simulation",
        ]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.processes.append(process)
        return process

    def collect_wire(self, wait_s=0.0):
        """只读模拟舵机端收到的数据，保存下来核对有没有多发或发送 NUL。"""
        if self.master_fd is None:
            return
        readable, _, _ = select.select([self.master_fd], [], [], wait_s)
        if readable:
            self.wire_bytes.extend(os.read(self.master_fd, 4096))

    def wait_for_query(self, process):
        deadline = time.monotonic() + 2.0
        while len(self.wire_bytes) < 9 and time.monotonic() < deadline:
            self.collect_wire(0.02)
            if process.poll() is not None:
                break
        self.assertEqual(bytes(self.wire_bytes), b"#002PRAD!")

    def finish(self, process, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        # 主端持续存在直到子进程结束，避免测试自身造成假断线。
        while process.poll() is None and time.monotonic() < deadline:
            if self.master_fd is None:
                time.sleep(0.01)
            else:
                self.collect_wire(0.01)
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=2)
            self.fail("C++ 子进程未在有限等待时间内退出")
        self.collect_wire()
        stdout, stderr = process.communicate(timeout=2)
        return process.returncode, stdout.decode("utf-8"), stderr.decode("utf-8")

    def assert_success(self, process, position=1495):
        status, stdout, stderr = self.finish(process)
        self.assertEqual(status, 0, stdout + stderr)
        self.assertIn("SOURCE=PTY_SIMULATION", stdout)
        self.assertIn(f"POSITION_P={position}", stdout)
        # 本次事务只有一条查询；\0、查询重发以及运动指令都不能出现。
        self.assertEqual(bytes(self.wire_bytes), b"#002PRAD!")

    def test_fragmented_reply_keeps_receiver_state(self):
        process = self.start_reader()
        self.wait_for_query(process)
        os.write(self.master_fd, b"#002P14")
        time.sleep(0.03)
        os.write(self.master_fd, b"95!")
        self.assert_success(process)

    def test_echo_wrong_id_and_correct_reply_in_one_chunk(self):
        process = self.start_reader()
        self.wait_for_query(process)
        os.write(self.master_fd, b"#002PRAD!#001P1400!#002P1495!")
        self.assert_success(process)

    def test_malformed_and_overlong_frames_resynchronize(self):
        process = self.start_reader()
        self.wait_for_query(process)
        # 格式错误和超过 64 字节的帧不得冒充有效位置；下一条 # 重新同步。
        os.write(self.master_fd, b"#002P1x95!#" + b"x" * 80 + b"!#002P1495!")
        self.assert_success(process)

    def test_valid_wire_position_may_be_protocol_endpoint(self):
        process = self.start_reader()
        self.wait_for_query(process)
        os.write(self.master_fd, b"#002P0500!")
        self.assert_success(process, 500)

    def test_raw_115200_8n1_configuration_is_visible_on_slave(self):
        process = self.start_reader()
        self.wait_for_query(process)
        # 收到查询时程序已经配置 TTY；读取内核配置，不从源码字符串推断通过。
        # PTY 仅能核对 termios 设置，不能证明真实波特率、电平或 DTR/RTS。
        iflag, _, cflag, lflag, input_speed, output_speed, cc = termios.tcgetattr(self.slave_fd)
        self.assertEqual(input_speed, termios.B115200)
        self.assertEqual(output_speed, termios.B115200)
        self.assertEqual(cflag & termios.CSIZE, termios.CS8)
        self.assertEqual(cflag & (termios.CREAD | termios.CLOCAL), termios.CREAD | termios.CLOCAL)
        self.assertEqual(
            cflag & (termios.PARENB | termios.CSTOPB | termios.CRTSCTS | termios.HUPCL), 0
        )
        self.assertEqual(iflag & (termios.IXON | termios.IXOFF | termios.IXANY), 0)
        self.assertEqual(lflag & (termios.ICANON | termios.ECHO), 0)
        self.assertIn(cc[termios.VMIN], (1, b"\x01"))
        self.assertIn(cc[termios.VTIME], (0, b"\x00"))
        os.write(self.master_fd, b"#002P1495!")
        self.assert_success(process)

    def test_no_reply_times_out_without_retransmission(self):
        began = time.monotonic()
        process = self.start_reader(timeout_ms=150)
        self.wait_for_query(process)
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertNotIn("POSITION_P=", stdout)
        self.assertLess(time.monotonic() - began, 2.0)
        self.assertEqual(bytes(self.wire_bytes), b"#002PRAD!")

    def test_wrong_id_and_bad_position_never_become_success(self):
        process = self.start_reader(timeout_ms=200)
        self.wait_for_query(process)
        os.write(self.master_fd, b"#001P1495!#002P9999!#002P1495x!")
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertNotIn("POSITION_P=", stdout)
        self.assertEqual(bytes(self.wire_bytes), b"#002PRAD!")

    def test_invalid_traffic_does_not_restart_transaction_deadline(self):
        began = time.monotonic()
        process = self.start_reader(timeout_ms=150)
        self.wait_for_query(process)
        deadline = time.monotonic() + 0.6
        while process.poll() is None and time.monotonic() < deadline:
            os.write(self.master_fd, b"#001P1495!")
            self.collect_wire(0.015)
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertNotIn("POSITION_P=", stdout)
        self.assertLess(time.monotonic() - began, 0.6)
        self.assertEqual(bytes(self.wire_bytes), b"#002PRAD!")

    def test_noise_over_receive_budget_aborts_without_retransmission(self):
        process = self.start_reader(timeout_ms=2000)
        self.wait_for_query(process)
        # 主端采用非阻塞写，避免子进程拒绝噪声后测试仍阻塞在写入上。
        os.set_blocking(self.master_fd, False)
        noise = b"x" * 8192
        sent = 0
        deadline = time.monotonic() + 1.0
        while sent < len(noise) and process.poll() is None and time.monotonic() < deadline:
            try:
                sent += os.write(self.master_fd, noise[sent:])
            except BlockingIOError:
                time.sleep(0.005)
        self.assertGreater(sent, 4096)
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertIn("4096", stderr)
        self.assertNotIn("POSITION_P=", stdout)
        self.assertEqual(bytes(self.wire_bytes), b"#002PRAD!")

    def test_disconnected_master_is_not_a_valid_reply(self):
        process = self.start_reader(timeout_ms=1000)
        self.wait_for_query(process)
        os.close(self.master_fd)
        self.master_fd = None
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertNotIn("POSITION_P=", stdout)

    def test_other_flock_owner_prevents_query(self):
        fcntl.flock(self.slave_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = self.start_reader()
        status, stdout, stderr = self.finish(process)
        self.assertNotEqual(status, 0, stdout + stderr)
        self.assertNotIn("POSITION_P=", stdout)
        self.assertEqual(bytes(self.wire_bytes), b"")

    def test_no_arguments_or_help_do_not_open_a_device(self):
        for arguments in ([], ["--help"]):
            with self.subTest(arguments=arguments):
                completed = subprocess.run(
                    [str(BINARY), *arguments], capture_output=True, text=True, timeout=2
                )
                if arguments:
                    self.assertEqual(completed.returncode, 0, completed.stderr)
                else:
                    self.assertNotEqual(completed.returncode, 0)
                self.collect_wire()
                self.assertEqual(bytes(self.wire_bytes), b"")

    def test_bad_cli_values_are_rejected_before_query(self):
        # 必须先验证完整文本再缩窄为 uint8_t，256 不能绕回编号 0。
        for option, value in (
            ("--id", "256"),
            ("--id", "-1"),
            ("--id", "6"),
            ("--id", "2abc"),
            ("--timeout-ms", "0"),
            ("--timeout-ms", "5001"),
        ):
            with self.subTest(option=option, value=value):
                if option == "--id":
                    process = self.start_reader(servo_id=value)
                else:
                    process = self.start_reader(timeout_ms=value)
                status, stdout, stderr = self.finish(process)
                self.assertNotEqual(status, 0, stdout + stderr)
                self.assertEqual(bytes(self.wire_bytes), b"")

    def test_non_tty_rejected_and_no_lock_remains_after_process_exit(self):
        with tempfile.TemporaryDirectory(prefix="jixiebi-cpp-pty-") as folder:
            ordinary_file = Path(folder) / "ordinary-file.txt"
            ordinary_file.write_text("", encoding="utf-8")
            completed = subprocess.run(
                [str(BINARY), "--port", str(ordinary_file), "--id", "2"],
                capture_output=True,
                text=True,
                timeout=2,
            )
            self.assertNotEqual(completed.returncode, 0)
            # 这里只证明失败进程退出后没有遗留锁；进程退出本身会关闭描述符，
            # 因此不能用本测试声称类在构造异常路径中主动关闭了描述符。
            fd = os.open(ordinary_file, os.O_RDWR)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            finally:
                os.close(fd)
            self.assertEqual(ordinary_file.read_bytes(), b"")

    def test_pty_mode_rejects_ordinary_file_before_writing(self):
        with tempfile.TemporaryDirectory(prefix="jixiebi-cpp-pty-") as folder:
            ordinary_file = Path(folder) / "not-a-pty.txt"
            ordinary_file.write_text("", encoding="utf-8")
            completed = subprocess.run(
                [
                    str(BINARY),
                    "--port",
                    str(ordinary_file),
                    "--id",
                    "2",
                    "--pty-simulation",
                ],
                capture_output=True,
                text=True,
                timeout=2,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertEqual(ordinary_file.read_bytes(), b"")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=BINARY, help="已编译的 C++ 只读程序")
    args, remaining = parser.parse_known_args()
    BINARY = args.binary.resolve()
    if sys.platform == "linux" and not BINARY.is_file():
        parser.error(f"程序不存在，请先编译：{BINARY}")
    unittest.main(argv=[sys.argv[0], *remaining])
