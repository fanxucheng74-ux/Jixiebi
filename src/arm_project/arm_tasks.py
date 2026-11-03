"""可移植的示教姿态与抓放任务层；不导入串口、不自动控制设备。

driver 合同（仅 tick 显式调用）：
  move_pose(pose, *, execution_id, step_id) -> dict
  grip(action, *, execution_id, step_id) -> dict
两者均须返回 status='SUCCEEDED' 及相同 execution_id、step_id 才能推进。
move_pose 只移动机械臂轴000--004，保持005夹爪；六个P全部保存在姿态中供
追溯，夹爪动作只能走 grip。驱动负责标定、各轴限位、路径和实际到位检查；
本模块不生成安全角度、速度或夹爪阈值，verified姿态也不代表路径已经安全。

close还须 object_grasped=True 及 evidence 字典：source='EXTERNAL_OBSERVATION'、
execution_id、step_id、observed_at（带时区ISO）、description。观察时间必须在
本次close调用起止之间；舵机位置到位不能替代抓到物证据。回调可等待外部确认，
但 cancel 仅禁止后续步骤，不能中断正在阻塞的回调或证明机械臂物理停止。

顺序固定为 approach/open/descend/close/lift/place/open/retract，具体示教姿态
可配置。每次tick最多一步，每个runner只运行一次；失败不复位、不重发、不回程。
SUCCEEDED表示此合同下步骤完成，不证明放置精度或整项物理任务验收。
"""

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from threading import Lock
from typing import Optional
from uuid import uuid4


PHASES = ("approach", "open", "descend", "close", "lift", "place", "open", "retract")
# PHASES是固定八步顺序；两次open由步骤下标区分，不共享同一个执行身份。
MOVE_PHASES = frozenset(("approach", "descend", "lift", "place", "retract"))


def _timestamp(value, field):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(field + " must be an offset-aware ISO timestamp")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00" if value.endswith("Z") else value)
    except ValueError as error:
        raise ValueError(field + " must be an offset-aware ISO timestamp") from error
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError(field + " must include a UTC offset")
    return parsed.astimezone(timezone.utc)


@dataclass(frozen=True)
class TaughtPose:
    # positions_p按000–005地址保存六个原始P值；verified只表明姿态经过外部确认。
    # 路径、工具坐标与抓住物体仍需另外验证，不能由姿态名称推断。
    name: str
    positions_p: tuple
    source: str
    captured_at: str
    verified: bool = False

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("pose needs a nonempty name")
        if not isinstance(self.positions_p, (tuple, list)) or len(self.positions_p) != 6:
            raise ValueError("pose needs exactly six raw protocol P values")
        if any(type(value) is not int for value in self.positions_p):
            raise ValueError("pose P values must be integers, not angles or booleans")
        if any(not 500 <= value <= 2500 for value in self.positions_p):
            raise ValueError(
                "pose P values must be inside protocol bounds [500, 2500]; these are not mechanical limits"
            )
        if not isinstance(self.source, str) or self.source.upper() != "HARDWARE":
            raise ValueError("taught poses must originate from HARDWARE acquisition")
        if type(self.verified) is not bool:
            raise ValueError("verified must be a boolean")
        _timestamp(self.captured_at, "captured_at")
        object.__setattr__(self, "positions_p", tuple(self.positions_p))
        object.__setattr__(self, "source", "HARDWARE")

    def to_dict(self):
        return {
            "name": self.name,
            "positions_p": list(self.positions_p),
            "source": self.source,
            "captured_at": self.captured_at,
            "verified": self.verified,
        }

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError("pose must be an object")
        fields = ("name", "positions_p", "source", "captured_at", "verified")
        if any(field not in data for field in fields):
            raise ValueError("pose metadata is incomplete")
        return cls(**{field: data[field] for field in fields})


def save_poses(path, poses):
    """保存新的示教文件，不覆盖已有记录；未验证姿态可保存但不能执行。"""
    entries = list(poses)
    if not entries or any(not isinstance(pose, TaughtPose) for pose in entries):
        raise ValueError("expected a nonempty sequence of TaughtPose")
    if len({pose.name for pose in entries}) != len(entries):
        raise ValueError("duplicate pose names")
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(
            {"schema_version": 1, "poses": [pose.to_dict() for pose in entries]},
            stream,
            ensure_ascii=False,
            indent=2,
        )


def load_poses(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if (
        not isinstance(data, dict)
        or type(data.get("schema_version")) is not int
        or data["schema_version"] != 1
    ):
        raise ValueError("unsupported pose schema")
    if not isinstance(data.get("poses"), list) or not data["poses"]:
        raise ValueError("pose file is empty or incomplete")
    poses = [TaughtPose.from_dict(entry) for entry in data["poses"]]
    if len({pose.name for pose in poses}) != len(poses):
        raise ValueError("duplicate pose names")
    return {pose.name: pose for pose in poses}


@dataclass(frozen=True)
class TaskStep:
    name: str
    kind: str
    pose_name: Optional[str] = None
    action: Optional[str] = None


class ArmTaskRunner:
    def __init__(self, driver, poses, steps, *, now=None):
        # driver是满足move_pose/grip合同的对象引用；任务层不直接操作串口。
        self.driver = driver
        self.poses = dict(poses)
        self.steps = tuple(steps)
        self.now = now or (lambda: datetime.now(timezone.utc))
        self._preflight()
        self.state = "READY"
        self.execution_id = None
        # index指向下一步；execution_id标识整趟任务，step_id标识其中某次回调。
        self.index = 0
        self.reason = ""
        self.cancel_requested = False
        self.grasp_confirmed = False
        self.grasp_evidence = None
        self.history = []
        self._tick_lock = Lock()

    def _preflight(self):
        # 在start/tick之前检查八步是否齐全、姿态是否存在，避免做了一半才发现缺配置。
        if not callable(getattr(self.driver, "move_pose", None)) or not callable(
            getattr(self.driver, "grip", None)
        ):
            raise ValueError("driver must provide move_pose and grip callbacks")
        if (
            any(not isinstance(step, TaskStep) for step in self.steps)
            or tuple(s.name for s in self.steps) != PHASES
        ):
            raise ValueError("task must contain the complete configured eight-phase sequence")
        for step in self.steps:
            if step.name in MOVE_PHASES:
                if (
                    step.kind != "move"
                    or step.action is not None
                    or not isinstance(step.pose_name, str)
                ):
                    raise ValueError("move step needs a pose name and no gripper action")
                pose = self.poses.get(step.pose_name)
                if (
                    not isinstance(pose, TaughtPose)
                    or pose.name != step.pose_name
                    or not pose.verified
                ):
                    raise ValueError("all task poses must exist and be verified hardware captures")
            elif step.kind != "grip" or step.action != step.name or step.pose_name is not None:
                raise ValueError("gripper step must explicitly open or close without a motion pose")

    def _now(self):
        value = self.now()
        if not isinstance(value, datetime) or value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("task clock must return an offset-aware datetime")
        return value.astimezone(timezone.utc)

    def snapshot(self):
        return deepcopy(
            {
                "state": self.state,
                "execution_id": self.execution_id,
                "next_step_index": self.index,
                "step_count": len(self.steps),
                "reason": self.reason,
                "grasp_confirmed": self.grasp_confirmed,
                "grasp_evidence": self.grasp_evidence,
                "history": self.history,
                "physical_stop_verified": False,
                "physical_task_verified": False,
            }
        )

    def start(self):
        if self.state != "READY":
            raise RuntimeError("this task session cannot be started again")
        self._preflight()
        self.execution_id = uuid4().hex
        self.state = "RUNNING"
        return self.snapshot()

    def cancel(self):
        if self.state in ("READY", "RUNNING"):
            self.cancel_requested = True
            self.state = "CANCELLED"
            self.reason = (
                "later steps cancelled; in-flight physical motion is not stopped by this task layer"
            )
        return self.snapshot()

    def _grip_evidence(self, result, step_id, started, finished):
        # 夹爪位置变化不能证明夹到了物体；证据必须属于本次任务、本次close时间段。
        evidence = result.get("evidence")
        if result.get("object_grasped") is not True or not isinstance(evidence, dict):
            raise ValueError("close succeeded without external evidence of an object being grasped")
        source = evidence.get("source")
        if not isinstance(source, str) or source.upper() != "EXTERNAL_OBSERVATION":
            raise ValueError("gripper position or command acknowledgement is not grasp evidence")
        if evidence.get("execution_id") != self.execution_id or evidence.get("step_id") != step_id:
            raise ValueError("grasp evidence belongs to another execution or step")
        observed = _timestamp(evidence.get("observed_at"), "observed_at")
        if not started <= observed <= finished:
            raise ValueError("grasp evidence must be observed during this close operation")
        if not isinstance(evidence.get("description"), str) or not evidence["description"].strip():
            raise ValueError("grasp evidence needs an observation description")
        return {
            "source": "EXTERNAL_OBSERVATION",
            "execution_id": self.execution_id,
            "step_id": step_id,
            "observed_at": observed.isoformat(),
            "description": evidence["description"],
        }

    def tick(self):
        # 非阻塞锁拒绝两个调用者同时推进同一任务；每次tick最多执行一个步骤。
        if not self._tick_lock.acquire(blocking=False):
            raise RuntimeError("a task step is already executing")
        try:
            if self.state != "RUNNING":
                return self.snapshot()
            step = self.steps[self.index]
            # 例如00:approach、01:open、06:open；同名动作也有不同的唯一位置。
            step_id = "%02d:%s" % (self.index, step.name)
            if step.name in ("lift", "place") and not self.grasp_confirmed:
                self.state, self.reason = (
                    "GRIP_UNVERIFIED",
                    "cannot lift or transport without current grasp evidence",
                )
                return self.snapshot()
            record = {"step_id": step_id, "name": step.name, "status": "STARTED"}
            self.history.append(record)
            try:
                started = self._now()
                record["started_at"] = started.isoformat()
                # 把身份随调用传给驱动，再核对返回值，防止误用上一趟任务的结果。
                identity = {"execution_id": self.execution_id, "step_id": step_id}
                if self.cancel_requested:
                    record["status"] = "CANCELLED_BEFORE_CALLBACK"
                    return self.snapshot()
                if step.kind == "move":
                    result = self.driver.move_pose(self.poses[step.pose_name], **identity)
                else:
                    result = self.driver.grip(step.action, **identity)
                finished = self._now()
                record["finished_at"] = finished.isoformat()
                if self.cancel_requested:
                    record["status"] = "CANCELLED_RESULT_DISCARDED"
                    return self.snapshot()
                if not isinstance(result, dict) or any(
                    result.get(key) != value for key, value in identity.items()
                ):
                    raise ValueError("driver result does not match the current execution and step")
                # 驱动仅报告“有响应”时不推进严格任务；SUCCEEDED后还需检查close证据。
                if result.get("status") != "SUCCEEDED":
                    raise RuntimeError(
                        "driver did not confirm step success: " + str(result.get("status"))
                    )
                if step.name == "close":
                    try:
                        self.grasp_evidence = self._grip_evidence(
                            result, step_id, started, finished
                        )
                    except ValueError as error:
                        self.state, self.reason = "GRIP_UNVERIFIED", str(error)
                        record.update(status=self.state, error=self.reason)
                        return self.snapshot()
                    self.grasp_confirmed = True
                elif step.name == "open":
                    self.grasp_confirmed = False
                record["status"] = "SUCCEEDED"
                # 全部检查通过才进入下一步；失败分支保留当前状态，不自动重发或回正。
                self.index += 1
                if self.index == len(self.steps):
                    self.state = "SUCCEEDED"
                return self.snapshot()
            except KeyboardInterrupt:
                self.cancel()
                record["status"] = "CANCELLED"
                return self.snapshot()
            except Exception as error:
                if self.cancel_requested:
                    record.update(status="CANCELLED", error=str(error))
                else:
                    self.state, self.reason = "FAILED", "%s: %s" % (type(error).__name__, error)
                    record.update(status="FAILED", error=self.reason)
                return self.snapshot()
        finally:
            self._tick_lock.release()
