"""Publish raw position feedback; never expose motion commands through ROS."""

import json
from pathlib import Path
from uuid import UUID

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from arm_project.paths import PROJECT_ROOT
from arm_project.ros_readonly import ReadOnlySession


class RawStateNode(Node):
    def __init__(self):
        super().__init__("jixiebi_raw_state")
        self.declare_parameter("port", "")
        self.declare_parameter("period_s", 2.0)
        port = self.get_parameter("port").value
        period = self.get_parameter("period_s").value
        if not isinstance(port, str) or not port:
            raise ValueError("请明确指定 port 参数；默认不连接硬件")
        if not isinstance(period, (int, float)) or not 1.0 <= period <= 60.0:
            raise ValueError("period_s 必须在 1 到 60 秒之间")
        self.publisher = self.create_publisher(String, "/jixiebi/servo_states_raw", 10)
        # 同机同启动才可比较单调时钟；无法取得时保留原始查询功能但不提供标称桥接资格。
        try:
            self.host_boot_id = str(
                UUID(Path("/proc/sys/kernel/random/boot_id").read_text().strip())
            )
        except (OSError, ValueError):
            self.host_boot_id = None
            self.get_logger().warning("无法读取启动标识；标称估计桥会拒绝本会话")
        self.session = ReadOnlySession(port, PROJECT_ROOT / "runs")
        self.timer = None
        try:
            # 通电后先读取六轴身份；查询失败即关闭会话，不自动重试。
            self.session.start()
            self.timer = self.create_timer(float(period), self.poll_once)
        except BaseException:
            self.session.close()
            raise
        self.get_logger().info("只读串口会话已打开，日志: " + str(self.session.run_dir))

    def poll_once(self):
        try:
            state = self.session.poll()
            if self.host_boot_id is not None:
                # 附加元数据不改变原始P值、真实采样记录或v1消息结构。
                state["host_boot_id"] = self.host_boot_id
            # JSON保留每轴P、主机接收时间与状态；这不是弧度制JointState。
            payload = {
                "schema": "jixiebi.raw_state.v1",
                "session_id": self.session.session_id,
                "batch_index": self.session.batch_index,
                "state": state,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.publisher.publish(msg)
            if state["status"] != "OK":
                self.timer.cancel()
                self.get_logger().error("读数不完整，已停止轮询并关闭串口: " + str(state["error"]))
        except Exception as exc:
            self.timer.cancel()
            self.session.close()
            self.get_logger().error("读取失败，已停止轮询并关闭串口: " + str(exc))

    def destroy_node(self):
        try:
            if self.timer is not None:
                self.timer.cancel()
            self.session.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RawStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
