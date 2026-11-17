"""只读标称估计话题适配，不打开串口，不提供机器人运动接口。"""

import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_project.nominal_state_bridge import NominalStateBridge


def _unique_object(pairs):
    # JSON重复键会造成不同解析器对状态含义理解不一致，直接拒绝。
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError("non-finite JSON constant: " + value)


class NominalEstimateNode(Node):
    def __init__(self):
        super().__init__("jixiebi_nominal_estimate")
        self.declare_parameter("max_age_s", 3.0)
        age = self.get_parameter("max_age_s").value
        # 仅Linux同一启动、同一单调时钟命名空间可比较；ID并不认证消息来源。
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.bridge = NominalStateBridge(self.boot_id, max_age_s=age)
        self.publisher = self.create_publisher(JointState, "/nominal_estimate/joint_states", 10)
        self.subscription = self.create_subscription(
            String, "/jixiebi/servo_states_raw", self.on_state, 10
        )
        self.get_logger().warning("仅厂家标称估计，不是实测角度，不可用于运动控制")

    def on_state(self, message):
        try:
            if self.get_parameter("use_sim_time").value:
                raise ValueError("use_sim_time is unsupported for host reception timestamps")
            if len(message.data) > 65536:
                raise ValueError("raw message too large")
            payload = json.loads(
                message.data, object_pairs_hook=_unique_object, parse_constant=_invalid_constant
            )
            # 每一批只转换一次；状态不可用时不重发上次的估计。
            samples = self.bridge.accept(
                payload,
                now_monotonic=time.monotonic(),
                now_wall=time.time(),
                host_boot_id=self.boot_id,
            )
            for sample in samples:
                msg = JointState()
                msg.header.stamp.sec = sample["stamp_sec"]
                msg.header.stamp.nanosec = sample["stamp_nanosec"]
                msg.name = [sample["name"]]
                msg.position = [sample["position_rad"]]
                self.publisher.publish(msg)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.get_logger().warning("拒绝无效/过期状态，不发布估计: " + str(error))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NominalEstimateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
