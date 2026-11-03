"""ROS 单轴连续动作的审阅和单次执行；导入本模块不操作硬件。"""

import re
from threading import Lock

from arm_project.joint_sweep import SweepSpec


def parse_review(data):
    """审阅文件同时固定完整扫掠参数和每一轴的起始范围，单位均为协议 P。"""
    if type(data) is not dict or set(data) != {"sweep", "start_ranges_p"}:
        raise ValueError("审阅文件必须包含 sweep 与 start_ranges_p")
    spec = SweepSpec.from_dict(data["sweep"])
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", spec.review_id):
        raise ValueError("review_id 只能包含字母、数字、下划线和连字符")
    ranges = data["start_ranges_p"]
    if type(ranges) is not dict or set(ranges) != {str(i) for i in range(6)}:
        raise ValueError("必须提供 ID 0 至 5 的起始区间")
    for bounds in ranges.values():
        if (
            type(bounds) is not list
            or len(bounds) != 2
            or any(type(p) is not int or not 500 <= p <= 2500 for p in bounds)
            or bounds[0] > bounds[1]
        ):
            raise ValueError("起始区间必须是协议 P 的有效整数上下界")
    lo, hi = ranges[str(spec.servo_id)]
    if lo < spec.start_min_p or hi > spec.start_max_p:
        raise ValueError("运动轴起始区间必须落在 sweep 的起始范围内")
    return spec, {k: tuple(v) for k, v in ranges.items()}


class OneShotSweep:
    """一个审阅只执行一次；取消仅请求停止，不能证明机械臂物理停止。"""

    def __init__(self, spec, ranges, link, controller, reader):
        self.spec, self.ranges = spec, ranges
        self.link, self.controller, self.reader = link, controller, reader
        self.lock = Lock()
        self.used = False
        self.finished = False
        self.cancel_requested = False

    def cancel(self):
        with self.lock:
            if self.finished:
                return {"status": "FINISHED", "physical_stop_verified": False}
            self.cancel_requested = True
            self.controller.cancel()
            return {"status": "CANCEL_REQUESTED", "physical_stop_verified": False}

    def execute(self):
        with self.lock:
            if self.used:
                return {"status": "ALREADY_USED", "physical_stop_verified": False}
            self.used = True
        result = {
            "status": "FAILED",
            "initial_state": None,
            "motion": None,
            "physical_stop_verified": False,
            "error": None,
        }
        try:
            # 使用同一持有串口的链路逐轴读取；不是同步采样或角度标定。
            state = self.reader.read_state()
            result["initial_state"] = state
            if state.get("source") != "HARDWARE" or state.get("status") != "OK":
                raise ValueError("六轴状态读取不完整")
            servos = state.get("servos", [])
            if len(servos) != 6 or {s.get("servo_id") for s in servos} != set(range(6)):
                raise ValueError("六轴数据缺失或重复")
            for servo in servos:
                value = servo.get("position_p")
                lo, hi = self.ranges[str(servo["servo_id"])]
                if servo.get("status") != "OK" or type(value) is not int or not lo <= value <= hi:
                    raise ValueError("六轴起始状态超出审阅范围")
            motion = self.controller.run()
            result["motion"] = motion
            status = motion.get("status")
            with self.lock:
                self.finished = True
                cancelled = self.cancel_requested
            # 方向响应、严格到位和无需运动是不同结果，不据此宣称夹持或停止。
            if cancelled:
                result["status"] = "CANCELLED"
            elif not motion.get("error") and (
                (
                    status == "TARGET_REACHED"
                    and motion.get("response_pass") is True
                    and motion.get("position_reached") is True
                )
                or (
                    status == "RESPONSE_OBSERVED"
                    and motion.get("response_pass") is True
                    and motion.get("position_reached") is False
                )
                or (
                    status == "NO_MOTION_NEEDED"
                    and motion.get("position_reached") is True
                    and motion.get("motion_attempted") is False
                )
            ):
                result["status"] = status
            else:
                result["status"] = "FAILED"
        except Exception as error:
            result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            with self.lock:
                self.finished = True
        return result
