"""串口打开配置与失败清理的离线测试，不访问真实设备。"""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from arm_project import arm_link
from arm_project.arm_link import SerialTransport


class MemorySerialPort:
    """只记录打开时状态；所有字节操作均保留在内存中。"""

    def __init__(self, *, allow_exclusive=True, exclusive_error=None, open_error=None):
        self.events = []
        self.allow_exclusive = allow_exclusive
        self.exclusive_error = exclusive_error
        self.open_error = open_error
        self.is_open = False
        self.open_calls = 0
        self.close_calls = 0
        self.open_snapshot = None

    def __setattr__(self, name, value):
        if name in {"exclusive", "dtr", "rts", "port"}:
            self.events.append((name, value))
            if name == "exclusive":
                if not self.allow_exclusive:
                    raise AssertionError("non-Linux path must not set exclusive")
                if self.exclusive_error is not None:
                    raise self.exclusive_error
        object.__setattr__(self, name, value)

    def open(self):
        self.open_calls += 1
        self.open_snapshot = {
            name: self.__dict__.get(name) for name in ("port", "dtr", "rts", "exclusive")
        }
        self.events.append(("open", None))
        # 模拟open已取得部分资源后失败，要求上层仍关闭而不是重新打开。
        self.is_open = True
        if self.open_error is not None:
            raise self.open_error

    def close(self):
        self.close_calls += 1
        self.is_open = False
        self.events.append(("close", None))


class SerialTransportTests(unittest.TestCase):
    def serial_module(self, port):
        factory = Mock(return_value=port)
        return SimpleNamespace(Serial=factory), factory

    def test_import_and_construction_do_not_construct_or_open_serial(self):
        port = MemorySerialPort()
        module, factory = self.serial_module(port)
        spec = importlib.util.spec_from_file_location(
            "serial_transport_import_probe", Path(arm_link.__file__)
        )
        imported = importlib.util.module_from_spec(spec)
        with patch.dict("sys.modules", {"serial": module}):
            spec.loader.exec_module(imported)
            transport = imported.SerialTransport("NEVER_OPENED")
        factory.assert_not_called()
        self.assertIsNone(transport.port)
        self.assertTrue(transport.closed)
        self.assertEqual(port.open_calls, 0)

    def test_linux_configures_lock_and_control_lines_before_open(self):
        port = MemorySerialPort()
        module, factory = self.serial_module(port)
        transport = SerialTransport("/dev/verified-zlink")
        with (
            patch.dict("sys.modules", {"serial": module}),
            patch("arm_project.arm_link.sys.platform", "linux"),
        ):
            transport.open()
        factory.assert_called_once()
        config = factory.call_args.kwargs
        self.assertIsNone(config["port"])
        self.assertEqual((config["baudrate"], config["bytesize"]), (115200, 8))
        self.assertEqual((config["parity"], config["stopbits"]), ("N", 1))
        self.assertEqual((config["timeout"], config["write_timeout"]), (0.02, 0.3))
        self.assertFalse(any(config[name] for name in ("xonxoff", "rtscts", "dsrdtr")))
        self.assertEqual(
            port.open_snapshot,
            {"port": "/dev/verified-zlink", "dtr": False, "rts": False, "exclusive": True},
        )
        self.assertLess(port.events.index(("exclusive", True)), port.events.index(("open", None)))
        self.assertIs(transport.port, port)
        self.assertFalse(transport.closed)
        transport.close()
        self.assertTrue(transport.closed)
        self.assertEqual(port.close_calls, 1)

    def test_non_linux_does_not_request_posix_lock(self):
        for platform in ("win32", "darwin"):
            with self.subTest(platform=platform):
                port = MemorySerialPort(allow_exclusive=False)
                module, factory = self.serial_module(port)
                transport = SerialTransport("OFFLINE_PORT")
                with (
                    patch.dict("sys.modules", {"serial": module}),
                    patch("arm_project.arm_link.sys.platform", platform),
                ):
                    transport.open()
                factory.assert_called_once()
                self.assertFalse(any(name == "exclusive" for name, _ in port.events))
                self.assertEqual(port.open_snapshot["dtr"], False)
                self.assertEqual(port.open_snapshot["rts"], False)
                self.assertEqual(port.open_calls, 1)
                transport.close()

    def test_open_failure_closes_partial_port_and_never_retries(self):
        for error in (OSError("port busy"), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                port = MemorySerialPort(open_error=error)
                module, factory = self.serial_module(port)
                transport = SerialTransport("/dev/verified-zlink")
                with (
                    patch.dict("sys.modules", {"serial": module}),
                    patch("arm_project.arm_link.sys.platform", "linux"),
                ):
                    with self.assertRaises(type(error)) as raised:
                        transport.open()
                self.assertIs(raised.exception, error)
                factory.assert_called_once()
                self.assertEqual((port.open_calls, port.close_calls), (1, 1))
                self.assertFalse(port.is_open)
                self.assertIsNone(transport.port)
                self.assertTrue(transport.closed)

    def test_lock_configuration_failure_closes_without_open_or_retry(self):
        error = OSError("exclusive setting failed")
        port = MemorySerialPort(exclusive_error=error)
        module, factory = self.serial_module(port)
        transport = SerialTransport("/dev/verified-zlink")
        with (
            patch.dict("sys.modules", {"serial": module}),
            patch("arm_project.arm_link.sys.platform", "linux"),
        ):
            with self.assertRaises(OSError) as raised:
                transport.open()
        self.assertIs(raised.exception, error)
        factory.assert_called_once()
        self.assertEqual((port.open_calls, port.close_calls), (0, 1))
        self.assertIsNone(transport.port)
        self.assertTrue(transport.closed)


if __name__ == "__main__":
    unittest.main()
