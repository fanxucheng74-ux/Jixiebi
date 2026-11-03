"""ROS 2 固定点搬运名义方案发布；仅读显式文件，不连接串口或驱动机械臂。"""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from arm_project.fixed_transfer import plan_fixed_transfer


def _read_object(filename, label):
    # 路径来自参数，读取前要求显式指定；不自行寻找旧计划或打开任何设备。
    if not isinstance(filename, str) or not filename.strip():
        raise ValueError(f"请指定 {label} 参数：JSON 文件路径")
    path = Path(filename).expanduser()
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError(f"{label} 必须指向 JSON 对象")
    return data


class TransferPreviewNode(Node):
    """读取输入并发布一次名义方案；ROS Topic 不提供任何运动授权或执行能力。"""

    def __init__(self):
        super().__init__("jixiebi_fixed_transfer_preview")
        self.declare_parameter("model_path", "")
        self.declare_parameter("request_path", "")
        model = _read_object(self.get_parameter("model_path").value, "model_path")
        request = _read_object(self.get_parameter("request_path").value, "request_path")
        # 算法和几何由项目统一实现；任何错误应中止发布，不能复用上次缓存方案。
        plan = plan_fixed_transfer(model, request)
        if not isinstance(plan, dict) or plan.get("motion_authorized") is not False:
            raise ValueError("规划器必须明确返回禁止运动的名义方案")
        payload = {
            "schema": "jixiebi.fixed_transfer_nominal.v1",
            "source": "LOCAL_STATIC_PLAN",
            "plan": plan,
            "motion_authorized": False,
        }
        # transient_local 允许节点存活时读取最近的一次结果；它不是实时硬件状态。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, "/jixiebi/planning/fixed_transfer_nominal", qos
        )
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(message)
        self.get_logger().info("名义搬运方案已发布；未连接串口或发送运动指令")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TransferPreviewNode()
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
