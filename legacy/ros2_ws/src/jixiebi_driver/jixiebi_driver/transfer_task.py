"""ROS有限搬运任务入口；节点待命不打开串口，实际硬件搬运仍需独立验收。"""

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arm_project.transfer_task_runtime import TransferTaskRunner


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


class TransferTaskNode(Node):
    """Trigger只启动本地已指定的一整趟任务，不接收任意P目标或串口帧。"""

    def __init__(self, config, port):
        super().__init__("jixiebi_transfer_task")
        self.runner = None
        try:
            self.publisher = self.create_publisher(
                String,
                "/jixiebi/task/status",
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            # 构造runner只校验本地配置；串口所有权由run在一次有限任务期间独占。
            self.runner = TransferTaskRunner(config, port, progress=self.on_progress)
            self.start_group = MutuallyExclusiveCallbackGroup()
            self.cancel_group = MutuallyExclusiveCallbackGroup()
            self.start_service = self.create_service(
                Trigger,
                "/jixiebi/task/start",
                self.on_start,
                callback_group=self.start_group,
            )
            self.cancel_service = self.create_service(
                Trigger,
                "/jixiebi/task/cancel",
                self.on_cancel,
                callback_group=self.cancel_group,
            )
            self.on_progress(
                {
                    "status": "READY_FOR_REQUEST",
                    "source": "ROS_TASK_WRAPPER",
                    "motion_started": False,
                    "motion_authorized": False,
                    "physical_task_verified": False,
                    "note": "节点待命不打开串口；响应完成不等于物体搬运或物理停止验收。",
                }
            )
        except BaseException:
            # 部分初始化失败也释放ROS实体，不调用run或尝试打开设备。
            super().destroy_node()
            raise
        self.get_logger().info(
            "任务请求服务待命：/jixiebi/task/start；不表示运动获准，实物搬运未验证。"
        )

    def on_progress(self, progress):
        # 原样保留runtime的真实来源及状态，不把模拟或待命状态标成HARDWARE。
        if not isinstance(progress, dict):
            raise ValueError("task progress must be a dictionary")
        message = String()
        message.data = json.dumps(progress, ensure_ascii=False, default=str)
        self.publisher.publish(message)
        # 证据位置来自运行器返回值；节点不创建、覆盖或删除预算与raw日志。
        paths = {
            key: value
            for key, value in progress.items()
            if value is not None
            and (key.endswith("_path") or key.endswith("_dir") or key == "evidence_paths")
        }
        if paths:
            self.get_logger().info(
                "任务证据位置: " + json.dumps(paths, ensure_ascii=False, default=str)
            )

    def on_start(self, request, response):
        del request
        try:
            # 重复调用交由runner返回ALREADY_USED，不建立第二个串口所有者或自动重试。
            result = self.runner.run()
        except Exception as error:
            self.runner.cancel()
            result = {
                "status": "FAILED",
                "response_pass": False,
                "physical_task_verified": False,
                "error": str(error),
            }
            self.get_logger().error("有限搬运任务异常: " + str(error))
        self.on_progress(result)
        response.success = (
            result.get("status") == "RESPONSE_SEQUENCE_COMPLETED"
            and result.get("response_pass") is True
        )
        response.message = json.dumps(result, ensure_ascii=False, default=str)
        return response

    def on_cancel(self, request, response):
        del request
        # 独立回调组允许run阻塞期间仅设置取消事件；不在这里读写串口或宣称物理急停。
        result = self.runner.cancel()
        self.on_progress(result)
        response.success = result.get("status") == "CANCEL_REQUESTED"
        response.message = json.dumps(result, ensure_ascii=False, default=str)
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="本地有限任务配置JSON，含显式预审信息")
    parser.add_argument("--port", required=True, help="明确指定唯一ZLink端口")
    options, ros_args = parser.parse_known_args(args)
    config = json.loads(
        Path(options.config).read_text(encoding="utf-8-sig"), object_pairs_hook=_unique_pairs
    )
    if not isinstance(config, dict):
        raise ValueError("task config must be a JSON object")
    rclpy.init(args=ros_args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = TransferTaskNode(config, options.port)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 先设置取消事件，再等待工作线程退出，最后销毁ROS实体。串口关闭归runner所有。
        try:
            if node is not None and node.runner is not None:
                node.runner.cancel()
        finally:
            try:
                executor.shutdown(wait_for_threads=True)
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    main()
