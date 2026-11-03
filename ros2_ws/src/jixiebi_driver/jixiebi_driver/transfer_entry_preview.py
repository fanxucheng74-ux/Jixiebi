"""订阅六轴反馈并规划名义入口；不持有串口，也不发送运动指令。"""

import json
import math
from pathlib import Path
import time
from uuid import UUID

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.transfer_entry import plan_transfer_entry


def _unique_object(pairs):
    # 重复JSON键可能掩盖错误的来源或坐标，明确拒绝而不取最后一个值。
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("JSON contains duplicate keys")
        result[key] = value
    return result


def _read_object(filename, label):
    if not isinstance(filename, str) or not filename.strip():
        raise ValueError(f"请明确指定 {label} 参数")
    value = json.loads(
        Path(filename).expanduser().read_text(encoding="utf-8-sig"),
        object_pairs_hook=_unique_object,
    )
    if not isinstance(value, dict):
        raise ValueError(f"{label} 必须指向JSON对象")
    return value


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + " must be finite")
    return float(value)


def _host_boot_id():
    return str(UUID(Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()))


class TransferEntryPreviewNode(Node):
    """当前状态→名义入口提案；失效消息替换缓存，不能继续复用旧路径。"""

    def __init__(self):
        super().__init__("jixiebi_transfer_entry_preview")
        self.declare_parameter("model_path", "")
        self.declare_parameter("request_path", "")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("state_topic", "/jixiebi/servo_states_raw")
        self.model = _read_object(self.get_parameter("model_path").value, "model_path")
        request = _read_object(self.get_parameter("request_path").value, "request_path")
        self.policy = _read_object(self.get_parameter("policy_path").value, "policy_path")
        topic = self.get_parameter("state_topic").value
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError("state_topic 必须是明确的话题名称")
        self.max_age_s = _number(self.policy.get("max_age_s"), "max_age_s")
        if not 0 < self.max_age_s <= 60:
            raise ValueError("max_age_s must be >0 and <=60")
        # 状态节点与本节点必须在相同Linux启动周期；此标识不构成身份认证。
        self.host_boot_id = _host_boot_id()
        # 两份规划使用同一模型，避免把100mm与170mm基座变体的结果拼接。
        self.transfer_plan = plan_fixed_transfer(self.model, request)
        if self.transfer_plan.get("motion_authorized") is not False:
            raise ValueError("搬运规划必须禁止直接运动")
        self.session_id = None
        self.last_batch = None
        self.valid_until_monotonic_s = None
        self.plan_cached = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, "/jixiebi/planning/transfer_entry_nominal", qos
        )
        self.subscription = self.create_subscription(String, topic, self.on_state, 1)
        # watchdog只更新失效状态，不根据缓存自行运动、回零或重复规划。
        self.timer = self.create_timer(0.25, self.check_expiry)
        self._publish_invalid("WAITING_FOR_FRESH_STATE", "等待同机新鲜六轴反馈")
        self.get_logger().info("入口规划节点待命；只订阅状态，不连接串口或执行动作")

    def _publish(self, value):
        message = String()
        message.data = json.dumps(value, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(message)

    def _publish_invalid(self, status, reason):
        self.valid_until_monotonic_s = None
        self.plan_cached = False
        self._publish(
            {
                "schema": "jixiebi.transfer_entry_nominal.v1",
                "source": "RAW_STATE_TOPIC",
                "status": status,
                "reason": reason,
                "entry": None,
                "motion_authorized": False,
                "source_authenticated": False,
            }
        )

    def on_state(self, message):
        try:
            now = time.monotonic()
            payload = json.loads(message.data, object_pairs_hook=_unique_object)
            if not isinstance(payload, dict):
                raise ValueError("raw state must be a JSON object")
            batch = payload.get("batch_index")
            if type(batch) is not int or batch < 1:
                raise ValueError("invalid batch_index")
            if self.last_batch is not None and batch <= self.last_batch:
                raise ValueError("duplicate or out-of-order batch")
            entry = plan_transfer_entry(
                self.model,
                self.transfer_plan,
                payload,
                self.policy,
                now_monotonic_s=now,
                host_boot_id=self.host_boot_id,
                expected_session_id=self.session_id,
            )
            if not isinstance(entry, dict) or entry.get("motion_authorized") is not False:
                raise ValueError("入口规划必须禁止直接运动")
            # 以最早读取的轴计算寿命，而非消息刚送达本节点的时间。
            received = [
                _number(row["received_monotonic_s"], "received_monotonic_s")
                for row in payload["state"]["servos"]
            ]
            if len(received) != 6 or any(t > now or now - t > self.max_age_s for t in received):
                raise ValueError("raw samples stale, in the future, or incomplete")
            self.session_id = payload["session_id"]
            self.last_batch = batch
            self.valid_until_monotonic_s = min(received) + self.max_age_s
            self.plan_cached = True
            self._publish(
                {
                    "schema": "jixiebi.transfer_entry_nominal.v1",
                    "source": "RAW_STATE_TOPIC",
                    "status": entry.get("status"),
                    "session_id": self.session_id,
                    "batch_index": batch,
                    "entry": entry,
                    "motion_authorized": False,
                    # HARDWARE是消息中的来源声明，不是认证过的硬件事实。
                    "source_authenticated": False,
                }
            )
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self._publish_invalid("INVALID_STATE", str(error))

    def check_expiry(self):
        if (
            self.plan_cached
            and self.valid_until_monotonic_s is not None
            and time.monotonic() >= self.valid_until_monotonic_s
        ):
            self._publish_invalid("STALE_STATE", "状态已过期，旧入口轨迹失效")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TransferEntryPreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
