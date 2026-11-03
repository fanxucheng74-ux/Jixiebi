"""显式有界任务策略的离线校验；许可范围来自策略，不伪造实物观测或标定。"""

import hashlib
import json
import math
import re
import time

from arm_project.general_transfer_entry import _duration, _settings
from arm_project.clearance_recovery import RecoverySceneGuard, task_scene_validator
from arm_project.handoff_envelope import require_handoff_envelopes
from arm_project.kinematics import finite
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import (
    REVIEW_SCHEMA,
    TransferTaskSpec,
    compile_motion_phases,
    motion_phases_sha256,
)


EVIDENCE_PREFIX = (
    "BOUNDED_POLICY_VALIDATION_ONLY; sampled nominal scene validation, "
    "not observed physical success or calibration. "
)
_MAX_VALIDITY_CHECKS = 100000
_VALIDATION_BUDGET_S = 3.0
_AXES = {str(sid) for sid in range(6)}
_BODY_AXES = {"0", "1", "2", "3"}
_TOLERANCE_AXES = _BODY_AXES | {"5"}
_POLICY_FIELDS = {
    "schema",
    "enabled",
    "evidence",
    "model_sha256",
    "scene_sha256",
    "planner_sha256",
    "cycle_motion_sha256",
    "observation_bounds_p",
    "handoff_tolerance_p",
    "maximum_entry_segments",
    "deadline_ms",
}
_CYCLE_NAMES = (
    "descend_left",
    "close",
    "lift_left",
    "rotate_right",
    "descend_right",
    "open",
    "lift_right",
    "rotate_left",
)


class _BudgetedScene(SceneValidator):
    # 恢复资格/逐边预检和后续整趟复核共用一个检查预算，不能先无界跑完恢复再检查时钟。
    # 保留SceneValidator类型及普通配置，__call__继承后仍进入本层inspect计数。
    def __init__(self, scene, guard):
        self._scene = scene
        self._guard = guard
        self.checks = 0

    def __getattr__(self, name):
        return getattr(self._scene, name)

    def inspect(self, positions4_p, *, minimum_clearance_mm=None):
        self._guard()
        if self.checks >= _MAX_VALIDITY_CHECKS:
            raise ValueError("task policy exceeded its 100000 scene-check budget")
        self.checks += 1
        inspected = self._scene.inspect(positions4_p, minimum_clearance_mm=minimum_clearance_mm)
        self._guard()
        return inspected


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def canonical_sha256(obj):
    """绑定完整JSON语义；键顺序不影响哈希，NaN和不可序列化值被拒绝。"""
    try:
        payload = json.dumps(
            obj, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("canonical hash requires finite JSON data") from error
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def cycle_motion_sha256(task):
    """绑定开爪准备与全部循环；排除每次起点不同而重新规划的entry前缀阶段。"""
    phases = [p for p in compile_motion_phases(task) if not p["name"].startswith("entry")]
    if not phases:
        raise ValueError("policy requires a nonempty fixed transfer cycle template")
    return canonical_sha256(phases)


def _policy(value):
    if (
        not isinstance(value, dict)
        or set(value) != _POLICY_FIELDS
        or value.get("schema") != "jixiebi.transfer_task_policy.v1"
        or value.get("enabled") is not True
    ):
        raise ValueError("an exact explicitly enabled transfer_task_policy.v1 is required")
    if not isinstance(value["evidence"], str) or not value["evidence"].strip():
        raise ValueError("explicit task policy evidence is required")
    for name in ("model_sha256", "scene_sha256", "planner_sha256", "cycle_motion_sha256"):
        if not isinstance(value[name], str) or not re.fullmatch(r"[a-f0-9]{64}", value[name]):
            raise ValueError(name + " must be a lowercase SHA256")
    bounds = value["observation_bounds_p"]
    if not isinstance(bounds, list) or len(bounds) != 6:
        raise ValueError("policy requires six explicit observation intervals")
    checked = []
    for pair in bounds:
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError("policy observation interval must be an integer pair")
        low, high = [_integer(p, "observation bound", 500, 2500) for p in pair]
        if low > high:
            raise ValueError("policy observation interval is reversed")
        checked.append((low, high))
    tolerance = value["handoff_tolerance_p"]
    if not isinstance(tolerance, dict) or set(tolerance) != _TOLERANCE_AXES:
        raise ValueError("policy requires exact handoff tolerances for axes 0,1,2,3,5")
    tolerance = {sid: _integer(v, "handoff tolerance", 3, 80) for sid, v in tolerance.items()}
    maximum = _integer(value["maximum_entry_segments"], "maximum_entry_segments", 1, 8)
    deadline = _integer(value["deadline_ms"], "deadline_ms", 2000, 180000)
    return checked, tolerance, maximum, deadline


def _planner(value):
    checked = _settings(value)
    for name, low, high in (
        ("step_p", 1, 2000),
        ("resolution_p", 1, 2000),
        ("max_iterations", 1, 10000),
        ("max_nodes", 2, 10000),
        ("seed", 0, 2**32 - 1),
    ):
        _integer(checked[name], name, low, high)
    if not 0.001 <= finite(checked["time_budget_s"], "time_budget_s") <= 10:
        raise ValueError("planner time budget outside 0.001..10 seconds")
    return checked


def _positions(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(name + " requires exact protocol positions")
    return [_integer(p, name, 500, 2500) for p in value]


def _full_state(value):
    if not isinstance(value, dict) or set(value) != _AXES:
        raise ValueError("task phase requires complete string-keyed six-axis states")
    return {str(sid): _integer(value[str(sid)], "phase position", 500, 2500) for sid in range(6)}


def _chain(task, compiled, bounds, maximum, planner):
    current = _positions(task.get("current_positions6_p"), 6, "current_positions6_p")
    if any(not low <= p <= high for p, (low, high) in zip(current, bounds)):
        raise ValueError("current task state outside policy observation bounds")
    phases = task.get("phases")
    if not isinstance(phases, list) or not 1 <= len(phases) <= 49:
        raise ValueError("policy requires one finite complete composed phase chain")
    count = _integer(task.get("cycle_count"), "cycle_count", 1, 5)
    expected = ["prepare_gripper"] + [
        f"cycle_{cycle:02d}_{name}" for cycle in range(1, count + 1) for name in _CYCLE_NAMES
    ]
    previous = {str(sid): p for sid, p in enumerate(current)}
    canonical, entries, nonentry, states = [], [], [], []
    seen_nonentry = False
    for row in phases:
        if not isinstance(row, dict):
            raise ValueError("invalid complete task phase")
        phase_id = row.get("phase_id")
        if not isinstance(phase_id, str) or not re.fullmatch(r"[a-z0-9_]{1,60}", phase_id):
            raise ValueError("invalid complete task phase identifier")
        start, target = _full_state(row.get("start_targets")), _full_state(row.get("targets"))
        if start != previous:
            raise ValueError("task phase chain must begin at current state and remain connected")
        if any(
            not low <= state[str(sid)] <= high
            for state in (start, target)
            for sid, (low, high) in enumerate(bounds)
        ):
            raise ValueError("task phase state outside policy observation bounds")
        duration = _integer(row.get("duration_ms"), "phase duration_ms", 0, 9000)
        motion = row.get("motion_targets")
        if not isinstance(motion, dict) or any(sid not in _TOLERANCE_AXES for sid in motion):
            raise ValueError("motion targets must contain only modeled axes or separate gripper")
        for sid, position in motion.items():
            if _integer(position, "motion target", 500, 2500) != target[sid]:
                raise ValueError("motion frames and complete nominal targets differ")
        entry = phase_id.startswith("entry")
        if entry:
            if seen_nonentry:
                raise ValueError("entry phases must form a contiguous task prefix")
            entries.append((start, target, duration, motion))
            if duration:
                if set(motion) != _BODY_AXES:
                    raise ValueError("moving entry requires exactly four modeled body axes")
                a, b = [start[str(i)] for i in range(4)], [target[str(i)] for i in range(4)]
                if a == b or duration != _duration(a, b, planner):
                    raise ValueError("entry duration does not match the bound planner dynamics")
                if not planner["minimum_segment_ms"] <= duration <= planner["maximum_segment_ms"]:
                    raise ValueError("entry duration exceeds bound planner segment limits")
            elif motion or task["entry_status"] != "ALREADY_AT_START":
                raise ValueError("zero-time entry must be an already-at-start acceptance")
        else:
            seen_nonentry = True
            nonentry.append(phase_id)
            if not duration or not motion:
                raise ValueError("fixed transfer stages must have explicit finite motion")
        for sid in _AXES - set(motion):
            if start[sid] != target[sid]:
                if not (
                    entry
                    and not duration
                    and sid in _BODY_AXES
                    # 零时长不发回位帧；只能接受与本次±3P起点走廊一致的窄偏差，
                    # 不能把最高80P的阶段交接容差当成已完成的位置移动。
                    and abs(start[sid] - target[sid]) <= 3
                ):
                    raise ValueError("unsent axis changes across the complete nominal phase")
        if duration and motion:
            canonical.append(
                {"name": phase_id, "duration_ms": duration, "targets": dict(sorted(motion.items()))}
            )
        states.append((start, target))
        previous = target
    if nonentry != expected or canonical != compiled:
        raise ValueError("complete phase chain does not match canonical motion/cycle sequence")
    if not entries or len(entries) > min(maximum, planner["max_segments"]):
        raise ValueError("entry segment count exceeds the explicit policy or planner cap")
    route = task.get("entry_route4_p")
    if not isinstance(route, list) or not route:
        raise ValueError("general entry requires its complete explicit route")
    route = [_positions(p, 4, "entry_route4_p") for p in route]
    actual = [current[:4]] + [
        [target[str(i)] for i in range(4)] for _, target, duration, _ in entries if duration
    ]
    if route != actual:
        raise ValueError("entry route differs from the complete task phases")
    if task["entry_status"] == "ALREADY_AT_START" and (len(entries) != 1 or entries[0][2] != 0):
        raise ValueError("already-at-start entry cannot contain route motion")
    goal = task.get("nominal_endpoints_p", {}).get("pick_hover")
    if _positions(goal, 4, "pick_hover") != [entries[-1][1][str(i)] for i in range(4)]:
        raise ValueError("entry route must finish at the nominal pick hover")
    return current, states


def build_policy_review(task, model, scene_config, planner_config, policy):
    """根据已启用策略编译本次精确任务；不读取设备、文件或补造物理证据。

    whole_path_reviewed_for_motion表示策略范围内的算法校验已通过，不表示新的人工作业
    审阅或实物成功。每个声明关节直线以resolution_p采样，不能证明异步P/T真实扫掠。
    """
    started = time.monotonic()

    def guard():
        if time.monotonic() - started >= _VALIDATION_BUDGET_S:
            raise ValueError("task policy validation exceeded its monotonic 3-second budget")

    bounds, tolerance, maximum, deadline = _policy(policy)
    compiled = compile_motion_phases(task)
    if task.get("entry_planner_version") != "GENERAL_JOINT_SPACE_V2":
        raise ValueError("policy requires the general joint-space entry planner")
    if task.get("source") not in {"SIMULATION", "HARDWARE_STATE"}:
        raise ValueError("task must retain explicit simulation or hardware-state provenance")
    for key, obj in (
        ("model_sha256", model),
        ("scene_sha256", scene_config),
        ("planner_sha256", planner_config),
    ):
        if canonical_sha256(obj) != policy[key]:
            raise ValueError(key + " does not match the explicitly enabled policy")
    if cycle_motion_sha256(task) != policy["cycle_motion_sha256"]:
        raise ValueError("fixed cycle motion does not match the bound task policy")
    planner = _planner(planner_config)
    scene = _BudgetedScene(SceneValidator(model, scene_config), guard)
    if task.get("model_geometry_mm") != model["geometry_mm"] or scene.table_z_mm != 0:
        raise ValueError("policy model or table frame differs from the supported task model")
    current, states = _chain(task, compiled, bounds, maximum, planner)
    # 软余量恢复仅用于入口前缀；达到普通余量后锁存原场景。不能规划放行而策略仍拒绝。
    validate_scene = task_scene_validator(scene, task)
    for start, target in states:
        guard()
        a, b = [start[str(i)] for i in range(4)], [target[str(i)] for i in range(4)]
        count = max(1, math.ceil(max(abs(y - x) for x, y in zip(a, b)) / planner["resolution_p"]))
        for index in range(count + 1):
            guard()
            pulses = [round(x + (y - x) * index / count) for x, y in zip(a, b)]
            if isinstance(validate_scene, RecoverySceneGuard):
                inspected = validate_scene.inspect(pulses)
            else:
                inspected = scene.inspect(pulses)
            guard()
            if not inspected["valid"]:
                raise ValueError("task policy path collision or limit: " + inspected["reason"])
    # 历史目标只由先前已编译运动帧建立，不把本次位置回读伪装成舵机内部既有目标。
    goals, phases = {}, []
    for phase in compiled:
        targets, holds = [], {}
        for sid, target in phase["targets"].items():
            targets.append(
                {"servo_id": int(sid), "target_p": target, "end_tolerance_p": tolerance[sid]}
            )
            if sid in _BODY_AXES and goals.get(sid) == target:
                holds[sid] = tolerance[sid]
            goals[sid] = target
        phases.append(
            {
                "name": phase["name"],
                "duration_ms": phase["duration_ms"],
                "dwell_ms": 0,
                "targets": targets,
                "hold_tracking_p": holds,
            }
        )
    provenance = {
        "policy_evidence": policy["evidence"],
        "model_sha256": policy["model_sha256"],
        "scene_sha256": policy["scene_sha256"],
        "planner_sha256": policy["planner_sha256"],
        "cycle_motion_sha256": policy["cycle_motion_sha256"],
        "sampled_scene_checks": scene.checks,
        "resolution_p": planner["resolution_p"],
        "physical_observation_evidence": False,
        "physical_geometry_verified": False,
        "physical_path_verified": False,
        "asynchronous_motion_envelope_verified": False,
        "table_recovery": (
            validate_scene.recovery.inspect_start()
            if isinstance(validate_scene, RecoverySceneGuard)
            else None
        ),
    }
    raw = {
        "schema": REVIEW_SCHEMA,
        "review_id": task.get("task_id"),
        "whole_path_reviewed_for_motion": True,
        "evidence": EVIDENCE_PREFIX + json.dumps(provenance, ensure_ascii=False, sort_keys=True),
        "start_intervals_p": [
            [max(low, p - 3), min(high, p + 3)] for p, (low, high) in zip(current, bounds)
        ],
        "observation_bounds_p": [list(pair) for pair in bounds],
        "geometry_mm": dict(model["geometry_mm"]),
        "table_z_mm": scene.table_z_mm,
        "phases": phases,
        "deadline_ms": deadline,
        "initial_setpoints_p": {},
        "motion_phases_sha256": motion_phases_sha256(compiled),
    }
    spec = TransferTaskSpec.from_dict(raw)
    # 名义目标通过不表示其允许的反馈组合也通过。例如肩/腕各自偏差合法，
    # 叠加后仍可能使工具包络穿过模型桌面。交接误差盒与同一3秒预算一起复核；
    # 不把3P数学对照自动替换现场20/35P容差，也不降低桌面或恢复底线。
    handoff = require_handoff_envelopes(spec, scene, check_budget=guard)
    provenance["handoff_table_envelope"] = handoff
    provenance["sampled_scene_checks"] = scene.checks
    raw["evidence"] = EVIDENCE_PREFIX + json.dumps(provenance, ensure_ascii=False, sort_keys=True)
    spec = TransferTaskSpec.from_dict(raw)
    guard()
    return spec.to_dict()
