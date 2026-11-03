"""ROS入口节点使用合成状态与ROS桩；真实规划器参与，无设备IO。"""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = ROOT / "ros2_ws/src/jixiebi_driver/jixiebi_driver/transfer_entry_preview.py"
BOOT = "8befb5b8-248c-4c14-a672-942590920b65"


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.node_name = name
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message_type, topic, qos):
        self.output_topic, self.output_qos = topic, qos
        return FakePublisher()

    def create_subscription(self, message_type, topic, callback, depth):
        return types.SimpleNamespace(topic=topic, callback=callback, depth=depth)

    def create_timer(self, period, callback):
        return types.SimpleNamespace(period=period, callback=callback)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda _: None)


def load_node_module():
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
    spec = importlib.util.spec_from_file_location("transfer_entry_ros_stub_test", NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def policy():
    return {
        "max_age_s": 3.0,
        "at_start_tolerance_p": 5,
        "high_pose_tolerance_p": 8,
        "minimum_tip_height_above_table_mm": 50.0,
        "minimum_centerline_height_mm": 5.0,
        "maximum_radial_mm": 400.0,
        "duration_ms": 3000,
        "max_joint_delta_p": 900,
    }


def raw_payload(positions, *, batch=1, session="synthetic_session"):
    return {
        "schema": "jixiebi.raw_state.v1",
        "session_id": session,
        "batch_index": batch,
        "state": {
            "source": "HARDWARE",
            "status": "OK",
            "error": None,
            "position_unit": "protocol_P",
            "simultaneous": False,
            "timestamp_origin": "host_receive",
            "host_boot_id": BOOT,
            "servos": [
                {
                    "servo_id": sid,
                    "status": "OK",
                    "position_p": position,
                    "received_monotonic_s": 99.6 + sid * 0.05,
                }
                for sid, position in enumerate(positions)
            ],
            "finished_monotonic_s": 99.9,
        },
    }


class TransferEntryROSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        settings = root / "entry.json"
        settings.write_text(json.dumps(policy()), encoding="utf-8")
        FakeNode.overrides = {
            "model_path": str(ROOT / "configs/kinematics.arduino_nominal.json"),
            "request_path": str(ROOT / "configs/fixed_transfer_nominal_example.json"),
            "policy_path": str(settings),
        }
        with (
            patch.object(self.module, "_host_boot_id", return_value=BOOT),
            patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("port")),
        ):
            self.node = self.module.TransferEntryPreviewNode()
        self.positions = [
            *self.node.transfer_plan["nominal_endpoints_p"]["place_hover"],
            1500,
            1500,
        ]

    def send(self, payload, now=100.0):
        message = types.SimpleNamespace(data=json.dumps(payload))
        with patch.object(self.module.time, "monotonic", return_value=now):
            self.node.on_state(message)
        return self.node.publisher.messages[-1]

    def test_fresh_right_hover_and_expiry_replace_the_cached_plan(self):
        self.assertEqual(self.node.publisher.messages[-1]["status"], "WAITING_FOR_FRESH_STATE")
        result = self.send(raw_payload(self.positions))
        self.assertEqual(result["status"], "NOMINAL_ENTRY_PROPOSAL")
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["source_authenticated"])
        self.assertEqual(self.node.output_topic, "/jixiebi/planning/transfer_entry_nominal")
        self.assertEqual(self.node.output_qos.durability, "TRANSIENT_LOCAL")
        with patch.object(self.module.time, "monotonic", return_value=102.61):
            self.node.check_expiry()
        invalid = self.node.publisher.messages[-1]
        self.assertEqual(invalid["status"], "STALE_STATE")
        self.assertIsNone(invalid["entry"])
        self.assertFalse(self.node.plan_cached)

    def test_stale_duplicate_or_switched_session_never_reuses_valid_trajectory(self):
        payload = raw_payload(self.positions)
        self.send(payload)
        for invalid in (payload, raw_payload(self.positions, batch=2, session="other_session")):
            result = self.send(invalid)
            self.assertEqual(result["status"], "INVALID_STATE")
            self.assertIsNone(result["entry"])
        result = self.send(raw_payload(self.positions, batch=2), now=110.0)
        self.assertEqual(result["status"], "INVALID_STATE")
        self.assertIsNone(result["entry"])
        self.assertEqual(self.node.session_id, "synthetic_session")
        self.assertEqual(self.node.last_batch, 1)

    def test_bad_boot_malformed_or_authorizing_result_invalidates_output(self):
        payload = raw_payload(self.positions)
        payload["state"]["host_boot_id"] = "72ff978c-0930-4afc-a009-e0454d10fd23"
        result = self.send(payload)
        self.assertEqual(result["status"], "INVALID_STATE")
        self.assertIsNone(result["entry"])
        with patch.object(self.module.time, "monotonic", return_value=100.0):
            self.node.on_state(types.SimpleNamespace(data="{broken"))
        self.assertEqual(self.node.publisher.messages[-1]["status"], "INVALID_STATE")
        with patch.object(
            self.module, "plan_transfer_entry", return_value={"motion_authorized": True}
        ):
            result = self.send(raw_payload(self.positions))
        self.assertEqual(result["status"], "INVALID_STATE")
        self.assertIsNone(result["entry"])

    def test_a_bad_message_does_not_advance_the_session_or_batch(self):
        valid = raw_payload(self.positions)
        broken = deepcopy(valid)
        broken["state"]["servos"][2]["status"] = "NOT_READ"
        self.assertEqual(self.send(broken)["status"], "INVALID_STATE")
        self.assertIsNone(self.node.session_id)
        result = self.send(valid)
        self.assertEqual(result["status"], "NOMINAL_ENTRY_PROPOSAL")
        self.assertEqual(self.node.last_batch, 1)


if __name__ == "__main__":
    unittest.main()
