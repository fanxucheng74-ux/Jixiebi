"""ROS 静态图像节点离线测试；使用内存 PNG 与 ROS 桩，不访问相机。"""

import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import cv2
import numpy as np


NODE_PATH = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/jixiebi_vision/jixiebi_vision/static_image.py"
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    overrides = {}
    created_publishers = []

    def __init__(self, name):
        self.node_name = name
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        self.created_publishers.append((topic, qos, publisher))
        return publisher

    def get_logger(self):
        return types.SimpleNamespace(info=lambda _: None)


def load_node_module():
    # 在模块导入时临时提供 ROS API 桩，不启动 ROS 进程或依赖设备。
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
    spec = importlib.util.spec_from_file_location("static_vision_stub_test", NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class StaticVisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def setUp(self):
        FakeNode.overrides = {}
        FakeNode.created_publishers = []

    def test_synthetic_image_decodes_detects_and_publishes_once(self):
        # 合成色块经过 PNG 解码和真实 HSV 函数，不代表实物木块识别验收。
        frame = np.zeros((100, 180, 3), dtype=np.uint8)
        frame[20:60, 10:50] = (0, 0, 255)
        frame[20:60, 70:110] = (255, 0, 0)
        frame[20:60, 130:170] = (0, 255, 0)
        ok, encoded = cv2.imencode(".png", frame)
        self.assertTrue(ok)
        FakeNode.overrides = {"image_path": "synthetic.png"}
        with (
            patch.object(Path, "read_bytes", return_value=encoded.tobytes()),
            patch.object(cv2, "VideoCapture", side_effect=AssertionError("camera forbidden")),
        ):
            self.module.StaticImageNode()
        self.assertEqual(len(FakeNode.created_publishers), 1)
        topic, qos, publisher = FakeNode.created_publishers[0]
        self.assertEqual(topic, "/jixiebi/vision/static_candidates")
        self.assertEqual(qos.durability, "TRANSIENT_LOCAL")
        self.assertEqual(len(publisher.messages), 1)
        output = json.loads(publisher.messages[0].data)
        self.assertEqual(output["source"], "STATIC_IMAGE")
        self.assertEqual(output["image_size_wh"], [180, 100])
        self.assertEqual(output["coordinate_units"], "pixels")
        self.assertFalse(output["motion_authorized"])
        self.assertEqual([d["color"] for d in output["detections"]], ["red", "blue", "green"])
        self.assertEqual(output["detections"][0]["center_px"], [29.5, 39.5])

    def test_empty_path_and_invalid_threshold_do_not_publish(self):
        for overrides in ({}, {"image_path": "x.png", "min_area_px2": float("nan")}):
            FakeNode.overrides = overrides
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.module.StaticImageNode()
            self.assertEqual(FakeNode.created_publishers, [])

    def test_unreadable_image_does_not_publish(self):
        FakeNode.overrides = {"image_path": "broken.png"}
        with (
            patch.object(Path, "read_bytes", return_value=b"not an image"),
            self.assertRaises(ValueError),
        ):
            self.module.StaticImageNode()
        self.assertEqual(FakeNode.created_publishers, [])


if __name__ == "__main__":
    unittest.main()
