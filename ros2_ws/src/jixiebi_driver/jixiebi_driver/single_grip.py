"""ROS 单次预审夹爪服务；网络请求不携带目标或原始串口命令。"""

import argparse
import json
import os
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from arm_project.ros_single_grip import OneShotGrip, SingleGripPlan
from arm_project.unified_arm import HardwareArmDriver


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


class SingleGripNode(Node):
    """一个唯一预审任务对应一个串口所有者，完成后不接受再次执行。"""

    def __init__(self, plan, plan_data, port):
        super().__init__("jixiebi_single_grip")
        self.driver = None
        self.runner = None
        # review_id绑定独占目录，即使节点异常退出也不允许重用同一预算。
        # RK的多个代码快照共用原始项目的预算目录，避免通过切换快照重用同一review_id。
        ledger_root = (Path.home() / "Jixiebi").resolve(strict=True)
        if not (ledger_root / ".arm-project-root").is_file():
            raise ValueError("缺少RK原始项目标记，不能建立跨快照一次性预算")
        folder = ledger_root / "runs" / "ros_single_grip" / plan.review_id
        # 上次夹爪在约2.5秒时仍轻微变化；将写出后的只读观察窗口延长至5秒。
        # 沿用驱动原有方向、范围、末5点稳定和±3P到位判据，不重复发送动作。
        self.driver = HardwareArmDriver(
            port,
            folder,
            planned_moves=plan.planned_moves,
            response_observation_s=5.0,
            response_poll_interval_s=0.5,
        )
        try:
            with (folder / "reviewed_plan.json").open("x", encoding="utf-8") as stream:
                json.dump(plan_data, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            self.driver.__enter__()
            self.runner = OneShotGrip(plan, self.driver)
            # 取消可在动作回调阻塞期间被另一个执行器线程处理。
            self.exec_group = MutuallyExclusiveCallbackGroup()
            self.cancel_group = MutuallyExclusiveCallbackGroup()
            self.execute_service = self.create_service(
                Trigger, "/jixiebi/gripper/execute", self.on_execute, callback_group=self.exec_group
            )
            self.cancel_service = self.create_service(
                Trigger, "/jixiebi/gripper/cancel", self.on_cancel, callback_group=self.cancel_group
            )
        except BaseException:
            self.driver.close()
            raise
        self.get_logger().info(f"预审夹爪计划待命: {plan.review_id}; 日志: {folder}")

    def on_execute(self, request, response):
        # Trigger请求本身没有目标参数；runner只会执行本地已审核且一次性的计划。
        del request
        result = self.runner.execute()
        # RESPONSE_OBSERVED表示有正确方向反馈，不等于严格到位或成功夹持。
        response.success = result["status"] in ("SUCCEEDED", "RESPONSE_OBSERVED")
        response.message = json.dumps(result, ensure_ascii=False, default=str)
        return response

    def on_cancel(self, request, response):
        # 与执行服务在不同回调组，允许动作期间发出取消请求；取消不等于物理急停。
        del request
        result = self.runner.cancel()
        response.success = result["status"] == "CANCELLED"
        response.message = json.dumps(result, ensure_ascii=False)
        return response

    def destroy_node(self):
        if self.driver is not None:
            self.driver.close()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, help="本地已审阅的一次性夹爪计划JSON")
    parser.add_argument("--port", required=True, help="明确指定ZLink端口")
    options, ros_args = parser.parse_known_args(args)
    plan_data = json.loads(
        Path(options.plan).read_text(encoding="utf-8-sig"), object_pairs_hook=_unique_pairs
    )
    plan = SingleGripPlan.from_dict(plan_data)
    rclpy.init(args=ros_args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = SingleGripNode(plan, plan_data, options.port)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node is not None and node.runner is not None:
            node.runner.cancel()
    finally:
        # 先请求取消，等待已有回调结束，才能释放其唯一串口。
        if node is not None and node.runner is not None:
            node.runner.cancel()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    # 同一入口既可由ros2 run调用，也可在维护时用python -m启动。
    main()
