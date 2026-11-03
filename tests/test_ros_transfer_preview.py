"""ROS 搬运名义规划节点测试：使用 ROS 桩，不连接设备。"""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from test_fixed_transfer import request


ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = ROOT / "ros2_ws/src/jixiebi_driver/jixiebi_driver/transfer_preview.py"


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    overrides = {}
    publishers = []

    def __init__(self, name):
        self.node_name = name
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        self.publishers.append((topic, qos, publisher))
        return publisher

    def get_logger(self):
        return types.SimpleNamespace(info=lambda _: None)


def load_node_module():
    # 仅模拟 ROS 层接口；规划算法仍使用项目中的真实实现。
    rclpy = types.ModuleType("rclpy")
    node_module = types.ModuleType("rclpy.node")
    node_module.Node = FakeNode
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="TRANSIENT_LOCAL")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="RELIABLE")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
    messages = types.ModuleType("std_msgs.msg")
    messages.String = type("FakeString", (), {})
    modules = {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.qos": qos,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": messages,
    }
    spec = importlib.util.spec_from_file_location("transfer_preview_stub_test", NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class TransferPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def setUp(self):
        FakeNode.overrides = {}
        FakeNode.publishers = []

    def test_real_planner_result_published_with_no_motion_authorization(self):
        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "request.json"
            input_path.write_text(json.dumps(request()), encoding="utf-8")
            FakeNode.overrides = {
                "model_path": str(ROOT / "configs/kinematics.vendor_nominal.json"),
                "request_path": str(input_path),
            }
            with patch(
                "arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("port")
            ):
                self.module.TransferPreviewNode()
        self.assertEqual(len(FakeNode.publishers), 1)
        topic, qos, publisher = FakeNode.publishers[0]
        self.assertEqual(topic, "/jixiebi/planning/fixed_transfer_nominal")
        self.assertEqual(qos.durability, "TRANSIENT_LOCAL")
        self.assertEqual(qos.reliability, "RELIABLE")
        self.assertEqual(len(publisher.messages), 1)
        output = json.loads(publisher.messages[0].data)
        self.assertEqual(output["source"], "LOCAL_STATIC_PLAN")
        self.assertIs(output["motion_authorized"], False)
        self.assertIs(output["plan"]["motion_authorized"], False)
        self.assertEqual(output["plan"]["cycle_count"], 5)
        self.assertEqual(output["plan"]["declared_ms_per_transfer"], 7400)

    def test_missing_request_or_false_planner_claim_does_not_publish(self):
        FakeNode.overrides = {"model_path": str(ROOT / "configs/kinematics.vendor_nominal.json")}
        with self.assertRaisesRegex(ValueError, "request_path"):
            self.module.TransferPreviewNode()
        self.assertEqual(FakeNode.publishers, [])

        with tempfile.TemporaryDirectory() as directory:
            input_path = Path(directory) / "request.json"
            input_path.write_text(json.dumps(request()), encoding="utf-8")
            FakeNode.overrides["request_path"] = str(input_path)
            with (
                patch.object(
                    self.module, "plan_fixed_transfer", return_value={"motion_authorized": True}
                ),
                self.assertRaisesRegex(ValueError, "禁止运动"),
            ):
                self.module.TransferPreviewNode()
        self.assertEqual(FakeNode.publishers, [])


if __name__ == "__main__":
    unittest.main()
