"""ROS 预审单轴连续动作；网络请求不携带目标或原始串口帧。"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from arm_project.arm_link import ServoLink
from arm_project.arm_state import ArmStateReader
from arm_project.joint_sweep import ExactSweepTransport, JointSweepController
from arm_project.ros_sweep import OneShotSweep, parse_review


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("审阅文件存在重复字段")
        result[key] = value
    return result


def save_new(path, value):
    # 排他创建并落盘：尝试写失败也不能重用本次预算。
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())


class SweepNode(Node):
    def __init__(self, data, port):
        super().__init__("jixiebi_single_sweep")
        spec, ranges = parse_review(data)
        root = (Path.home() / "Jixiebi").resolve(strict=True)
        if not (root / ".arm-project-root").is_file():
            raise ValueError("缺少原项目标记，无法共用独占预算")
        self.folder = root / "runs" / "joint_sweeps" / spec.review_id
        # 与 CLI 共用预算目录；已创建即视为消费，禁止切换代码快照重复执行。
        self.folder.mkdir(parents=True, exist_ok=False)
        save_new(self.folder / "spec.json", data)
        self.log = (self.folder / "events.jsonl").open("x", encoding="utf-8")
        self.link = ServoLink(ExactSweepTransport(port, spec), record=self.record)
        try:
            self.link.__enter__()
            self.controller = JointSweepController(
                self.link, spec, self.reserve, cancel_file=self.folder / "cancel.request"
            )
            self.runner = OneShotSweep(
                spec, ranges, self.link, self.controller, ArmStateReader(self.link)
            )
            self.exec_group = MutuallyExclusiveCallbackGroup()
            self.cancel_group = MutuallyExclusiveCallbackGroup()
            self.create_service(
                Trigger, "/jixiebi/sweep/execute", self.execute, callback_group=self.exec_group
            )
            self.create_service(
                Trigger, "/jixiebi/sweep/cancel", self.cancel, callback_group=self.cancel_group
            )
        except BaseException:
            self.link.__exit__(None, None, None)
            self.log.close()
            raise
        self.get_logger().info("预审单轴扫掠待命：%s；日志：%s" % (spec.review_id, self.folder))

    def record(self, event, **values):
        self.log.write(
            json.dumps(
                {"time": datetime.now().astimezone().isoformat(), "event": event, **values},
                ensure_ascii=False,
            )
            + "\n"
        )
        self.log.flush()

    def reserve(self, plan):
        # 在帧发送前记录实际起点与唯一允许的命令；不可自动重发。
        save_new(self.folder / "motion_attempt.json", {"plan": plan})

    def execute(self, request, response):
        del request
        result = self.runner.execute()
        if result["status"] != "ALREADY_USED":
            try:
                save_new(self.folder / "result.json", result)
            except OSError as error:
                result.update(status="LOGGING_FAILED", error=str(error))
        response.success = result["status"] in (
            "TARGET_REACHED",
            "RESPONSE_OBSERVED",
            "NO_MOTION_NEEDED",
        )
        response.message = json.dumps(result, ensure_ascii=False, default=str)
        return response

    def cancel(self, request, response):
        del request
        result = self.runner.cancel()
        response.success = result["status"] == "CANCEL_REQUESTED"
        response.message = json.dumps(result, ensure_ascii=False)
        return response

    def destroy_node(self):
        self.link.__exit__(None, None, None)
        self.log.close()
        return super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", required=True, help="本地已审阅的扫掠和六轴起始区间 JSON")
    parser.add_argument("--port", required=True, help="ZLink 串口设备")
    options, ros_args = parser.parse_known_args(args)
    data = json.loads(
        Path(options.spec).read_text(encoding="utf-8-sig"), object_pairs_hook=unique_pairs
    )
    parse_review(data)
    rclpy.init(args=ros_args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = SweepNode(data, options.port)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.runner.cancel()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
