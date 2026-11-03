"""ROS 节点共用会话的离线测试，不导入 rclpy 或打开真实串口。"""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from arm_project.ros_readonly import ReadOnlySerialTransport, ReadOnlySession
from arm_project.servo_protocol import validate_read_query


class Device:
    def __init__(self):
        self.commands = []

    def write(self, command):
        self.commands.append(command)
        return len(command)


class FakeLink:
    def __init__(self, transport, record):
        self.transport = transport
        self.record = record
        self.exits = 0

    def __enter__(self):
        self.record("opened", source="SIMULATION")
        return self

    def __exit__(self, *_):
        self.exits += 1
        self.record("closed")

    def record_best_effort(self, event, **values):
        self.record(event, **values)


class FakeReader:
    state = {"status": "OK", "servos": [{"position_p": 1500}]}
    fail_identity = False

    def __init__(self, link):
        self.link = link

    def read_identity(self):
        if self.fail_identity:
            raise TimeoutError("identity timeout")
        return []

    def read_state(self):
        return self.state


class SyntheticTransport:
    """合成查询回复：仅用于检查真实 ServoLink/Reader/日志的数据流。"""

    source = "SIMULATION"

    def __init__(self, port):
        self.port_name = port
        self.pending = bytearray()
        self.writes = []
        self.closed = True

    def open(self):
        self.closed = False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return len(self.pending)

    def read(self, count):
        chunk = bytes(self.pending[:count])
        del self.pending[:count]
        return chunk

    def write(self, command):
        servo_id, field = validate_read_query(command)
        self.writes.append(command)
        body = {"version": "PV1", "id": "P", "mode": "MOD1", "position": "P1500"}[field]
        self.pending.extend(("#%03d%s!" % (servo_id, body)).encode("ascii"))
        return len(command)


class ReadOnlyTests(unittest.TestCase):
    def test_synthetic_protocol_reader_and_logs_end_to_end(self):
        with (
            tempfile.TemporaryDirectory() as tmp,
            patch("arm_project.ros_readonly.ReadOnlySerialTransport", SyntheticTransport),
        ):
            session = ReadOnlySession("sim", Path(tmp))
            self.assertEqual(len(session.start()), 6)
            state = session.poll()
            self.assertEqual(state["status"], "OK")
            self.assertEqual([s["position_p"] for s in state["servos"]], [1500] * 6)
            self.assertEqual(session.batch_index, 1)
            self.assertEqual(len(session.transport.writes), 24)
            session.close()
            self.assertTrue(session.transport.closed)
            events = (session.run_dir / "events.jsonl").read_text(encoding="utf-8")
            self.assertIn('"event": "tx"', events)
            self.assertIn('"event": "rx"', events)

    def test_transport_rejects_motion_and_stop_before_serial_write(self):
        transport = ReadOnlySerialTransport("not-opened")
        transport.port = Device()
        for command in (b"#000P1550T1500!", b"#000PDST!", b"#255PRAD!"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                transport.write(command)
        self.assertEqual(transport.port.commands, [])
        transport.write(b"#000PRAD!")
        self.assertEqual(transport.port.commands, [b"#000PRAD!"])

    def test_incomplete_result_closes_and_prevents_poll_retry(self):
        with (
            tempfile.TemporaryDirectory() as tmp,
            patch("arm_project.ros_readonly.ServoLink", FakeLink),
            patch("arm_project.ros_readonly.ArmStateReader", FakeReader),
        ):
            session = ReadOnlySession("fake", Path(tmp))
            session.start()
            session.reader.state = {"status": "INCOMPLETE", "error": "timeout"}
            result = session.poll()
            self.assertEqual(result["status"], "INCOMPLETE")
            self.assertTrue(session.closed)
            self.assertEqual(session.link.exits, 1)
            with self.assertRaises(RuntimeError):
                session.poll()
            self.assertIn('"closed"', (session.run_dir / "events.jsonl").read_text())

    def test_start_failure_closes_and_preserves_fresh_log(self):
        with (
            tempfile.TemporaryDirectory() as tmp,
            patch("arm_project.ros_readonly.ServoLink", FakeLink),
            patch("arm_project.ros_readonly.ArmStateReader", FakeReader),
        ):
            with patch.object(FakeReader, "fail_identity", True):
                session = ReadOnlySession("fake", Path(tmp))
                with self.assertRaises(TimeoutError):
                    session.start()
                self.assertTrue(session.closed)
                self.assertEqual(session.link.exits, 1)
                self.assertTrue((session.run_dir / "events.jsonl").exists())


if __name__ == "__main__":
    unittest.main()
