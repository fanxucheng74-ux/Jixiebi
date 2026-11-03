"""以 ROS 消息桩验证只读标称桥回调；不加载 ROS，也不访问串口。"""

from datetime import datetime, timedelta, timezone
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


BOOT_ID = "a3c01061-3e90-4c40-a770-bade891e01cd"
SESSION_ID = "14dd54ab4e3e474c868897a45ac4358a"
START = datetime(2026, 11, 3, 5, 30, 22, tzinfo=timezone.utc)
ADAPTER_PATH = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/jixiebi_driver/jixiebi_driver/nominal_estimate.py"
)


class FakeJointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
        self.name = []
        self.position = []


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.parameters = {"use_sim_time": False}
        self.logger = FakeLogger()
        self.outputs = []
        self.inputs = []

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger

    def create_publisher(self, message_type, topic, depth):
        publisher = FakePublisher()
        self.outputs.append((message_type, topic, depth, publisher))
        return publisher

    def create_subscription(self, message_type, topic, callback, depth):
        subscription = (message_type, topic, callback, depth)
        self.inputs.append(subscription)
        return subscription


def load_adapter():
    # 只在模块导入时注入内存桩，测试不会启动 ROS 执行器或连接硬件。
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = FakeJointState
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("FakeString", (), {})
    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    spec = importlib.util.spec_from_file_location("nominal_estimate_stub_test", ADAPTER_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def state_message(batch=1, offset=0.0):
    # 合成消息保持真实协议的六轴顺序，各轴主机时间分别相差 50ms。
    pulses = (1510, 1503, 1495, 1495, 1504, 1514)
    servos = []
    for servo_id, pulse in enumerate(pulses):
        stamp = START + timedelta(seconds=offset, milliseconds=50 * servo_id)
        servos.append(
            {
                "servo_id": servo_id,
                "status": "OK",
                "position_p": pulse,
                "received_at": stamp.isoformat(),
                "received_monotonic_s": 1000.0 + offset + 0.05 * servo_id,
            }
        )
    return {
        "schema": "jixiebi.raw_state.v1",
        "session_id": SESSION_ID,
        "batch_index": batch,
        "state": {
            "source": "HARDWARE",
            "status": "OK",
            "error": None,
            "position_unit": "protocol_P",
            "simultaneous": False,
            "timestamp_origin": "host_receive",
            "host_boot_id": BOOT_ID,
            "servos": servos,
        },
    }


class NominalEstimateRosAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter = load_adapter()

    def setUp(self):
        # 注入固定 boot_id，阻止依赖测试机器的真实 /proc。
        fake_path = types.SimpleNamespace(read_text=lambda: BOOT_ID)
        with patch.object(self.adapter, "Path", return_value=fake_path):
            self.node = self.adapter.NominalEstimateNode()

    def send(self, payload, *, mono=1000.5, wall=None, literal=False):
        if wall is None:
            wall = START.timestamp() + (mono - 1000.0)
        text = payload if literal else json.dumps(payload)
        with (
            patch.object(self.adapter.time, "monotonic", return_value=mono),
            patch.object(self.adapter.time, "time", return_value=wall),
        ):
            self.node.on_state(types.SimpleNamespace(data=text))

    def test_callback_publishes_each_joint_with_its_own_sample_time(self):
        self.assertEqual(self.node.inputs[0][1], "/jixiebi/servo_states_raw")
        self.assertEqual(self.node.outputs[0][1], "/nominal_estimate/joint_states")
        self.send(state_message())
        messages = self.node.publisher.messages
        self.assertEqual(len(messages), 4)
        names = ("base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch")
        angles = (1.35, 90.405, 0.675, -0.675)
        for index, message in enumerate(messages):
            with self.subTest(joint=index):
                self.assertEqual(message.name, [names[index]])
                self.assertAlmostEqual(message.position[0], math.radians(angles[index]))
                stamp = START + timedelta(milliseconds=50 * index)
                self.assertEqual(message.header.stamp.sec, int(stamp.timestamp()))
                self.assertEqual(message.header.stamp.nanosec, stamp.microsecond * 1000)

    def test_invalid_stale_and_duplicate_messages_do_not_publish(self):
        self.send(state_message())
        self.assertEqual(len(self.node.publisher.messages), 4)
        self.send(state_message())  # 批次重复
        invalid = state_message(batch=2, offset=1)
        invalid["state"]["status"] = "INCOMPLETE"
        self.send(invalid, mono=1001.5)
        self.send(state_message(batch=2, offset=1), mono=1005.0)  # 陈旧
        self.send('{"schema":"one","schema":"two"}', literal=True)  # 重复键
        self.send('{"value":NaN}', literal=True)  # 非有限 JSON 常量
        self.assertEqual(len(self.node.publisher.messages), 4)
        self.assertGreaterEqual(len(self.node.logger.warnings), 6)
        self.send(state_message(batch=2, offset=1), mono=1001.5)
        self.assertEqual(len(self.node.publisher.messages), 8)

    def test_sim_time_rejects_without_consuming_batch(self):
        self.node.parameters["use_sim_time"] = True
        self.send(state_message())
        self.assertEqual(self.node.publisher.messages, [])
        self.node.parameters["use_sim_time"] = False
        self.send(state_message())
        self.assertEqual(len(self.node.publisher.messages), 4)


if __name__ == "__main__":
    unittest.main()
