"""固定预审夹爪任务；ROS无关，导入和编译均不连接串口。"""

from dataclasses import dataclass
import re
from threading import Event, Lock

from arm_project.unified_arm import MoveSpec


@dataclass(frozen=True)
class SingleGripPlan:
    review_id: str
    evidence: str
    action: str
    start_intervals_p: tuple
    reviewed: bool

    def __post_init__(self):
        if self.reviewed is not True:
            raise ValueError("explicit physical path and work area review required")
        if not isinstance(self.review_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_-]{1,80}", self.review_id
        ):
            raise ValueError("invalid unique review_id")
        if (
            not isinstance(self.evidence, str)
            or not self.evidence.strip()
            or len(self.evidence) > 1000
        ):
            raise ValueError("nonempty review evidence required")
        if self.action not in ("open", "close", "open_close"):
            raise ValueError("invalid action")
        if not isinstance(self.start_intervals_p, tuple) or len(self.start_intervals_p) != 6:
            raise ValueError("six reviewed starting intervals required")
        for row in self.start_intervals_p:
            if not isinstance(row, tuple) or len(row) != 2:
                raise ValueError("invalid start interval")
            low, high = row
            if type(low) is not int or type(high) is not int or not 500 <= low <= high <= 2500:
                raise ValueError("invalid protocol P interval")

    @classmethod
    def from_dict(cls, value):
        fields = {
            "schema_version",
            "review_id",
            "evidence",
            "action",
            "start_intervals_p",
            "reviewed",
        }
        if not isinstance(value, dict) or set(value) != fields:
            raise ValueError("exact reviewed plan fields required")
        if type(value["schema_version"]) is not int or value["schema_version"] != 1:
            raise ValueError("unsupported plan version")
        if value["reviewed"] is not True:
            raise ValueError("explicit physical path and work area review required")
        review_id, evidence, action = value["review_id"], value["evidence"], value["action"]
        if not isinstance(review_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", review_id):
            raise ValueError("invalid unique review_id")
        if not isinstance(evidence, str) or not evidence.strip() or len(evidence) > 1000:
            raise ValueError("nonempty review evidence required")
        if action not in ("open", "close", "open_close"):
            raise ValueError("action must be open, close or open_close")
        rows = value["start_intervals_p"]
        if not isinstance(rows, list) or len(rows) != 6:
            raise ValueError("six reviewed starting intervals required")
        intervals = []
        for row in rows:
            if not isinstance(row, list) or len(row) != 2:
                raise ValueError("invalid start interval")
            low, high = row
            if type(low) is not int or type(high) is not int or not 500 <= low <= high <= 2500:
                raise ValueError("invalid protocol P interval")
            intervals.append((low, high))
        return cls(review_id, evidence, action, tuple(intervals), True)

    @property
    def actions(self):
        return ("open", "close") if self.action == "open_close" else (self.action,)

    @property
    def planned_moves(self):
        # 固定夹爪协议目标沿用统一驱动，网络请求无法指定原始帧或任意数值。
        return tuple(MoveSpec(5, 1480 if item == "open" else 1570, 1000) for item in self.actions)


class OneShotGrip:
    """单会话、单请求、精确有限预算；取消不表示物理停止。"""

    def __init__(self, plan, driver):
        if not isinstance(plan, SingleGripPlan):
            raise ValueError("a prevalidated local plan is required")
        self.plan, self.driver = plan, driver
        self._lock = Lock()
        self._state_lock = Lock()
        self._cancelled = Event()
        self._used = False
        self._finished = False

    def cancel(self):
        with self._state_lock:
            if self._finished:
                return {"status": "ALREADY_USED", "physical_stop_verified": False}
            self._cancelled.set()
            self.driver.cancel()
        return {"status": "CANCELLED", "physical_stop_verified": False}

    def execute(self):
        if not self._lock.acquire(blocking=False):
            return {"status": "BUSY", "physical_stop_verified": False}
        try:
            with self._state_lock:
                if self._used:
                    return {"status": "ALREADY_USED", "physical_stop_verified": False}
                self._used = True
            if self._cancelled.is_set():
                return {"status": "CANCELLED", "physical_stop_verified": False}
            # 本次重新读取六轴并核对操作者审阅的起点；六轴是依次采样，并非同步。
            state = self.driver.read_state()
            rows = state.get("servos") if isinstance(state, dict) else None
            if state.get("source") != "HARDWARE" or state.get("status") != "OK":
                raise ValueError("fresh hardware state unavailable")
            if not isinstance(rows, list) or len(rows) != 6:
                raise ValueError("six fresh servo records required")
            for sid, (row, interval) in enumerate(zip(rows, self.plan.start_intervals_p)):
                if (
                    not isinstance(row, dict)
                    or type(row.get("servo_id")) is not int
                    or row["servo_id"] != sid
                    or row.get("status") != "OK"
                    or type(row.get("position_p")) is not int
                    or not interval[0] <= row["position_p"] <= interval[1]
                ):
                    raise ValueError("fresh state differs from reviewed starting interval")
            outputs = []
            for index, action in enumerate(self.plan.actions):
                if self._cancelled.is_set():
                    return {
                        "status": "CANCELLED",
                        "moves": outputs,
                        "physical_stop_verified": False,
                    }
                result = self.driver.grip(
                    action, execution_id=self.plan.review_id, step_id=f"{index:02d}:{action}"
                )
                outputs.append(result)
                if self._cancelled.is_set():
                    return {
                        "status": "CANCELLED",
                        "moves": outputs,
                        "physical_stop_verified": False,
                    }
                if (
                    not isinstance(result, dict)
                    or result.get("status") not in ("SUCCEEDED", "RESPONSE_OBSERVED")
                    or result.get("execution_id") != self.plan.review_id
                    or result.get("step_id") != f"{index:02d}:{action}"
                    or result.get("response_pass") is not True
                    or type(result.get("position_reached")) is not bool
                    or (result.get("status") == "SUCCEEDED" and not result["position_reached"])
                    or (result.get("status") == "RESPONSE_OBSERVED" and result["position_reached"])
                    or result.get("error")
                ):
                    return {"status": "FAILED", "moves": outputs, "physical_stop_verified": False}
            status = (
                "SUCCEEDED"
                if all(row.get("position_reached") is True for row in outputs)
                else "RESPONSE_OBSERVED"
            )
            # 成功判定和终态提交须与cancel()互斥，避免先答已取消又返回执行成功。
            with self._state_lock:
                if self._cancelled.is_set():
                    return {
                        "status": "CANCELLED",
                        "moves": outputs,
                        "physical_stop_verified": False,
                    }
                self._finished = True
                return {
                    "status": status,
                    "moves": outputs,
                    "physical_stop_verified": False,
                    "grasp_verified": False,
                }
        except Exception as error:
            return {
                "status": "CANCELLED" if self._cancelled.is_set() else "FAILED",
                "error": f"{type(error).__name__}: {error}",
                "physical_stop_verified": False,
            }
        finally:
            with self._state_lock:
                self._finished = True
            self._lock.release()
