"""经独立审阅的一次性搬运任务；复用有限阶段执行器，不在导入时打开设备。

每阶段按清单为每轴发送一条目标 P 和声明 T，运动中只读反馈。本模块没有
实现预览中的五次曲线流式插补，也不证明多轴同步、实物夹持或物理停止。
"""

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import re
from threading import Lock

from arm_project.clearance_recovery import RecoverySceneGuard
from arm_project.kinematics import Geometry
from arm_project.timed_cycle import CycleController, CycleSpec, CycleTransport, Phase, integer


REVIEW_SCHEMA = "jixiebi.reviewed_transfer_task.v1"
_AXIS_KEYS = {str(sid) for sid in range(6)}
_BODY_KEYS = {str(sid) for sid in range(5)}


def _tracking(value, allowed):
    if not isinstance(value, dict) or any(key not in allowed for key in value):
        raise ValueError("invalid reviewed tracking map")
    return tuple((int(key), integer(value[key], 3, 80)) for key in sorted(value))


@dataclass(frozen=True)
class TaskPhase(Phase):
    """支持底座 000 的有限阶段；夹爪 005 始终独立。"""

    @classmethod
    def from_dict(cls, value):
        required = {"name", "duration_ms", "dwell_ms", "targets"}
        if (
            not isinstance(value, dict)
            or required - set(value)
            or set(value) - required - {"hold_tracking_p", "continue_tracking_p"}
        ):
            raise ValueError("invalid task phase schema")
        if not isinstance(value["name"], str) or not re.fullmatch(
            r"[a-z0-9_]{1,60}", value["name"]
        ):
            raise ValueError("invalid task phase name")
        if not isinstance(value["targets"], list):
            raise ValueError("task phase targets must be a list")
        targets = []
        for target in value["targets"]:
            if not isinstance(target, dict) or set(target) != {
                "servo_id",
                "target_p",
                "end_tolerance_p",
            }:
                raise ValueError("invalid task target schema")
            targets.append(
                (
                    integer(target["servo_id"], 0, 5),
                    integer(target["target_p"], 500, 2500),
                    integer(target["end_tolerance_p"], 3, 80),
                )
            )
        ids = [sid for sid, _, _ in targets]
        if not 1 <= len(ids) <= 4 or len(set(ids)) != len(ids) or (5 in ids and ids != [5]):
            raise ValueError("one..four distinct body axes; gripper needs its own phase")
        holds = _tracking(value.get("hold_tracking_p", {}), _BODY_KEYS)
        continuing = _tracking(value.get("continue_tracking_p", {}), _AXIS_KEYS)
        if any(sid not in ids for sid, _ in holds):
            raise ValueError("held setpoint must be an explicit task target")
        if any(sid in ids for sid, _ in continuing):
            raise ValueError("continuation cannot send a new target")
        return cls(
            value["name"],
            integer(value["duration_ms"], 1000 if ids == [5] else 1500, 9000),
            integer(value["dwell_ms"], 0, 500),
            tuple(targets),
            holds,
            continuing,
        )


def compile_motion_phases(composed_task):
    """提取组合器明确列出的实际目标；不从六轴预期状态自动补发保持帧。"""
    if (
        not isinstance(composed_task, dict)
        or composed_task.get("schema") != "jixiebi.transfer_task.v1"
        or composed_task.get("status") != "NOMINAL_TASK_PROPOSAL"
        or composed_task.get("motion_authorized") is not False
        or composed_task.get("hardware_execution_allowed") is not False
        or composed_task.get("entry_status") not in {"ALREADY_AT_START", "NOMINAL_ENTRY_PROPOSAL"}
    ):
        raise ValueError("a complete non-executable composed task proposal is required")
    return _canonical_phases(composed_task.get("motion_phases"))


def _canonical_phases(phases):
    if not isinstance(phases, list) or not phases:
        raise ValueError("nonempty explicit motion phases required")
    result = []
    for phase in phases:
        if not isinstance(phase, dict):
            raise ValueError("invalid canonical motion phase")
        name = phase.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"[a-z0-9_]{1,60}", name):
            raise ValueError("invalid canonical phase name")
        duration = integer(phase.get("duration_ms"), 1, 9999)
        targets = phase.get("targets")
        if isinstance(targets, list):
            # 审阅阶段的容差约束留在审阅文件中；哈希只绑定实际发送的轴、P、T与顺序。
            mapped = {}
            for target in targets:
                if not isinstance(target, dict):
                    raise ValueError("invalid canonical target")
                sid = str(integer(target.get("servo_id"), 0, 5))
                if sid in mapped:
                    raise ValueError("duplicate canonical target axis")
                mapped[sid] = integer(target.get("target_p"), 500, 2500)
            targets = mapped
        if (
            not isinstance(targets, dict)
            or not targets
            or any(sid not in _AXIS_KEYS for sid in targets)
        ):
            raise ValueError("canonical targets require explicit string axis IDs")
        result.append(
            {
                "name": name,
                "duration_ms": duration,
                "targets": {sid: integer(targets[sid], 500, 2500) for sid in sorted(targets)},
            }
        )
    return result


def motion_phases_sha256(task_or_phases):
    """稳定绑定完整运动清单；不把新鲜时间、任务名称或保持轴噪声纳入运动哈希。"""
    phases = (
        compile_motion_phases(task_or_phases)
        if isinstance(task_or_phases, dict)
        else _canonical_phases(task_or_phases)
    )
    payload = json.dumps(
        phases, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _intervals(rows):
    if not isinstance(rows, list) or len(rows) != 6:
        raise ValueError("six explicit task intervals required")
    checked = []
    for row in rows:
        if not isinstance(row, list) or len(row) != 2:
            raise ValueError("invalid task interval")
        low, high = [integer(value, 500, 2500) for value in row]
        if low > high:
            raise ValueError("reversed task interval")
        checked.append((low, high))
    return tuple(checked)


def _phase_dict(phase):
    return {
        "name": phase.name,
        "duration_ms": phase.duration_ms,
        "dwell_ms": phase.dwell_ms,
        "targets": [
            {"servo_id": sid, "target_p": target, "end_tolerance_p": tolerance}
            for sid, target, tolerance in phase.targets
        ],
        "hold_tracking_p": {str(sid): tolerance for sid, tolerance in phase.hold_tracking_p},
        "continue_tracking_p": {
            str(sid): tolerance for sid, tolerance in phase.continue_tracking_p
        },
    }


@dataclass(frozen=True)
class TransferTaskSpec(CycleSpec):
    """独立审阅的一整趟有限预算；旧 CycleSpec 的局部上限保持原样。"""

    motion_phases_sha256: str = ""
    table_z_mm: float = 0.0

    @classmethod
    def from_dict(cls, value):
        required = {
            "schema",
            "review_id",
            "whole_path_reviewed_for_motion",
            "evidence",
            "start_intervals_p",
            "observation_bounds_p",
            "geometry_mm",
            "table_z_mm",
            "phases",
            "deadline_ms",
            "motion_phases_sha256",
        }
        if (
            not isinstance(value, dict)
            or required - set(value)
            or set(value) - required - {"initial_setpoints_p"}
        ):
            raise ValueError("provide the complete explicit reviewed task schema")
        if value["schema"] != REVIEW_SCHEMA or value["whole_path_reviewed_for_motion"] is not True:
            raise ValueError("an independently reviewed whole task is required")
        if not isinstance(value["review_id"], str) or not re.fullmatch(
            r"[A-Za-z0-9_-]{1,80}", value["review_id"]
        ):
            raise ValueError("invalid bounded task review id")
        if not isinstance(value["evidence"], str) or not value["evidence"].strip():
            raise ValueError("task review evidence required")
        table = value["table_z_mm"]
        if type(table) not in (int, float) or not math.isfinite(table) or table != 0:
            raise ValueError("task executor supports only explicit table_z_mm=0")
        starts = _intervals(value["start_intervals_p"])
        observation = _intervals(value["observation_bounds_p"])
        if any(
            not lo <= start_lo <= start_hi <= hi
            for (start_lo, start_hi), (lo, hi) in zip(starts, observation)
        ):
            raise ValueError("task start outside observation bounds")
        if not isinstance(value["phases"], list) or not 1 <= len(value["phases"]) <= 50:
            raise ValueError("one..50 task phases required")
        phases = tuple(TaskPhase.from_dict(phase) for phase in value["phases"])
        if len({phase.name for phase in phases}) != len(phases):
            raise ValueError("task phase names must be unique")
        if sum(len(phase.targets) for phase in phases) > 180:
            raise ValueError("task exceeds finite180-frame budget")
        deadline = integer(value["deadline_ms"], 2000, 180000)
        if sum(phase.duration_ms + phase.dwell_ms for phase in phases) > deadline - 1000:
            raise ValueError("reserve at least1s for task feedback within deadline")
        digest = value["motion_phases_sha256"]
        if (
            not isinstance(digest, str)
            or not re.fullmatch(r"[a-f0-9]{64}", digest)
            or digest != motion_phases_sha256(value["phases"])
        ):
            raise ValueError("review motion_phases_sha256 does not match exact phases")
        initial_data = value.get("initial_setpoints_p", {})
        if not isinstance(initial_data, dict) or any(sid not in _AXIS_KEYS for sid in initial_data):
            raise ValueError("invalid reviewed initial setpoint history")
        initial = tuple(
            (int(sid), integer(initial_data[sid], 500, 2500)) for sid in sorted(initial_data)
        )
        if any(not observation[sid][0] <= target <= observation[sid][1] for sid, target in initial):
            raise ValueError("initial goal outside task envelope")
        previous = dict(initial)
        # 初始目标未带交接容差时使用3P；后续旧目标一律保留其首次审阅的阶段交接走廊。
        limits = {sid: 3 for sid, _ in initial}
        tracked_phases = []
        for phase in phases:
            ids = {sid for sid, _, _ in phase.targets}
            continuing = dict(phase.continue_tracking_p)
            if any(sid not in previous for sid in continuing):
                raise ValueError("continuation requires a prior reviewed target")
            if any(tolerance > limits[sid] for sid, tolerance in continuing.items()):
                raise ValueError("continuation cannot loosen the original target corridor")
            for sid in previous.keys() - ids:
                continuing.setdefault(sid, limits[sid])
            holds = dict(phase.hold_tracking_p)
            for sid, target, tolerance in phase.targets:
                if not observation[sid][0] <= target <= observation[sid][1]:
                    raise ValueError("task target outside reviewed axis bound")
                if sid in holds and previous.get(sid) != target:
                    raise ValueError("hold cannot disguise a new task target")
                if sid in holds and holds[sid] > limits[sid]:
                    raise ValueError("hold cannot loosen the original target corridor")
                if sid in holds and tolerance > limits[sid]:
                    raise ValueError("held target cannot loosen its later tracking corridor")
                previous[sid] = target
                limits[sid] = tolerance
            tracked_phases.append(
                TaskPhase(
                    phase.name,
                    phase.duration_ms,
                    phase.dwell_ms,
                    phase.targets,
                    phase.hold_tracking_p,
                    tuple(sorted(continuing.items())),
                )
            )
        geometry_data = value["geometry_mm"]
        if not isinstance(geometry_data, dict) or set(geometry_data) != {
            "base_height_mm",
            "upper_arm_mm",
            "forearm_mm",
            "tool_mm",
        }:
            raise ValueError("exact task geometry fields required")
        return cls(
            review_id=value["review_id"],
            evidence=value["evidence"],
            start_intervals_p=starts,
            observation_bounds_p=observation,
            phases=tuple(tracked_phases),
            deadline_ms=deadline,
            geometry=Geometry(**geometry_data),
            initial_setpoints_p=initial,
            motion_phases_sha256=digest,
            table_z_mm=0.0,
        )

    def to_dict(self):
        """导出排他预算记录所需的完整审阅内容，包括新增的只读持续跟踪条件。"""
        return {
            "schema": REVIEW_SCHEMA,
            "review_id": self.review_id,
            "whole_path_reviewed_for_motion": True,
            "evidence": self.evidence,
            "start_intervals_p": [list(row) for row in self.start_intervals_p],
            "observation_bounds_p": [list(row) for row in self.observation_bounds_p],
            "geometry_mm": dict(vars(self.geometry)),
            "table_z_mm": self.table_z_mm,
            "phases": [_phase_dict(phase) for phase in self.phases],
            "deadline_ms": self.deadline_ms,
            "initial_setpoints_p": {str(sid): target for sid, target in self.initial_setpoints_p},
            "motion_phases_sha256": self.motion_phases_sha256,
        }


def validate_task_review(composed_task, review):
    """独立审阅必须精确匹配组合清单；名义规划中的 False 授权标志不会被改写。"""
    compiled = compile_motion_phases(composed_task)
    spec = TransferTaskSpec.from_dict(review)
    reviewed = _canonical_phases(review["phases"])
    if compiled != reviewed or motion_phases_sha256(compiled) != spec.motion_phases_sha256:
        raise ValueError("review does not match this exact composed task motion sequence")
    geometry = composed_task.get("model_geometry_mm")
    if not isinstance(geometry, dict) or geometry != dict(vars(spec.geometry)):
        raise ValueError("review geometry does not match the composed task model")
    return spec


class TaskTransport(CycleTransport):
    """沿用精确单次帧门禁，在真实 write 前再检查取消和总期限。"""

    def __init__(self, port, spec, *, raw=None, cancel_check=None):
        if not isinstance(spec, TransferTaskSpec):
            raise ValueError("validated transfer task spec required")
        if cancel_check is not None and not callable(cancel_check):
            raise ValueError("cancel_check must be callable")
        # 防止绕过 from_dict 直接构造数据类，把非法参数伪装成已经验证的审阅。
        if TransferTaskSpec.from_dict(spec.to_dict()) != spec:
            raise ValueError("task spec does not match its complete validated representation")
        super().__init__(port, spec, raw=raw)
        self.cancel_check = cancel_check

    def write(self, command):
        stopping = any(command == f"#{sid:03d}PDST!".encode() for sid in self.attempted_ids)
        if not stopping and self.cancel_check is not None:
            try:
                self.cancel_check()
            except (Exception, KeyboardInterrupt):
                self.cancelled = True
                raise
        return super().write(command)


class TransferTaskController(CycleController):
    """一次有限任务；保持旧阶段的方向/几何/预算/响应检查与异常停止行为。"""

    def __init__(self, link, spec, reserve, *, cancel_file=None, state_validator=None):
        if not isinstance(link.transport, TaskTransport) or not isinstance(spec, TransferTaskSpec):
            raise ValueError("matching transfer task transport and spec required")
        super().__init__(link, spec, reserve, cancel_file=cancel_file)
        if state_validator is not None and not callable(state_validator):
            raise ValueError("state_validator must be callable")
        self.state_validator = state_validator
        self._run_lock = Lock()
        external_cancel_check = link.transport.cancel_check

        def check_before_write():
            self._check_cancel()
            if external_cancel_check is not None:
                external_cancel_check()
            self._check_cancel()
            index = self.link.transport.phase_index
            if index >= 0:
                name = self.spec.phases[index].name
                if name != "entry" and not name.startswith("entry_"):
                    # 恢复例外只属于入口。端点即使落在P交接容差内，也必须先由
                    # 新鲜反馈证实达到普通场景，再允许准备或循环阶段的首条帧。
                    self._require_recovery_complete()

        link.transport.cancel_check = check_before_write

    def _require_recovery_complete(self):
        if (
            isinstance(self.state_validator, RecoverySceneGuard)
            and not self.state_validator.normal_scene_active
        ):
            raise ValueError("task entry recovery has not reached the ordinary scene")

    def check_completion(self):
        # 只有入口的有限任务同样需要完成脱离，不能把软余量里的P到位算作完成。
        # 此处发生在父类成功赋值和结果日志之前，保留统一失败、停止与记录流程。
        self._check_cancel()
        self._require_recovery_complete()

    def guard_geometry(self, positions):
        if self.state_validator is None:
            return super().guard_geometry(positions)
        # 方向、整趟P边界和旧目标走廊仍由父类guard检查；这里只替换名义场景判断。
        # 输入是依次回读的四轴组合，不能声称为硬件同步姿态或连续碰撞检测。
        positions4_p = tuple(positions[:4])
        inspector = getattr(self.state_validator, "inspect", None)
        inspection_available = callable(inspector)
        # RecoverySceneGuard.inspect会推进恢复锁存器，所以一次观察只能调用一次；
        # 不先调用__call__再inspect，否则同一反馈会被重复当作两个观察。
        if inspection_available:
            inspected = inspector(positions4_p)
            scene_check = inspected if isinstance(inspected, dict) else None
            valid = scene_check.get("valid") if scene_check is not None else None
        else:
            # 测试/扩展检查器可以只返回bool，但没有完整inspect时不编造碰撞诊断。
            scene_check = None
            valid = self.state_validator(positions4_p)
        if type(valid) is bool and valid:
            return

        index = self.link.transport.phase_index
        name = self.spec.phases[index].name if 0 <= index < len(self.spec.phases) else None
        # 这里保留真正被guard拒绝的组合；它可能尚未进入phase.samples，不能拿
        # 最后一个成功样本冒充失败姿态。端点补读与顺序六轴也不当作同步硬件快照。
        diagnostic = {
            "positions6_p": list(positions),
            "positions4_p": list(positions4_p),
            "phase_index": index,
            "phase_name": name,
            "simultaneous": False,
            "inspection_available": inspection_available,
            "validation_return_type": type(valid).__name__,
            "scene_check": deepcopy(scene_check),
        }
        # 持久化失败直接传播，仍交统一异常停止流程处理；不吞日志错误后继续发帧。
        self.link.record("task_scene_rejected", **diagnostic)
        details = []
        if scene_check is not None:
            for key in (
                "code",
                "link_index",
                "minimum_table_clearance_mm",
                "required_table_clearance_mm",
            ):
                if key in scene_check:
                    details.append(f"{key}={scene_check[key]}")
        if not inspection_available:
            details.append("inspection_available=False")
        if type(valid) is not bool:
            details.append(f"validation_return_type={type(valid).__name__}")
        suffix = "; " + "; ".join(details) if details else ""
        raise ValueError("feedback rejected by the task's configured nominal scene" + suffix)

    def cancel(self):
        self.cancel_requested = True
        self.link.transport.cancelled = True

    @staticmethod
    def positions(state):
        if not isinstance(state, dict) or state.get("status") != "OK":
            raise ValueError("incomplete six-axis task feedback")
        servos = state.get("servos")
        if not isinstance(servos, list) or len(servos) != 6:
            raise ValueError("task feedback requires exactly six ordered axes")
        positions = []
        for sid, servo in enumerate(servos):
            if (
                not isinstance(servo, dict)
                or type(servo.get("servo_id")) is not int
                or servo["servo_id"] != sid
                or servo.get("status") != "OK"
            ):
                raise ValueError("task feedback axes must be ordered000..005, unique and valid")
            positions.append(integer(servo.get("position_p"), 500, 2500))
        return positions

    def run(self):
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("another transfer task run is already active")
        try:
            return super().run()
        finally:
            self._run_lock.release()
