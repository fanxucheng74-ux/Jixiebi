"""组合入口与固定取放为一个有限名义任务；纯计算，不包含设备或执行入口。"""

from copy import deepcopy
import re


# 左右命名用于整趟任务阅读；原阶段名保留，时间仍取自固定取放规划。
_CYCLE_STAGES = (
    ("descend_left", "descend_pick", "pick", "open"),
    ("close", "close", "pick", "close"),
    ("lift_left", "lift_pick", "pick_hover", "close"),
    ("rotate_right", "rotate_to_place", "place_hover", "close"),
    ("descend_right", "descend_place", "place", "close"),
    ("open", "open", "place", "open"),
    ("lift_right", "lift_place", "place_hover", "open"),
    ("rotate_left", "rotate_to_pick", "pick_hover", "open"),
)
_ENTRY_READY = {"ALREADY_AT_START", "NOMINAL_ENTRY_PROPOSAL"}


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def _pulses(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} requires {size} protocol positions")
    return [_integer(p, name, 500, 2500) for p in value]


def _transfer(value):
    if (
        not isinstance(value, dict)
        or value.get("status") != "NOMINAL_PROPOSAL_ONLY"
        or value.get("motion_authorized") is not False
    ):
        raise ValueError("provide a non-executable nominal fixed transfer plan")
    count = _integer(value.get("cycle_count"), "cycle_count", 1, 5)
    endpoints = value.get("nominal_endpoints_p")
    if not isinstance(endpoints, dict):
        raise ValueError("transfer plan requires named nominal endpoints")
    targets = {
        name: _pulses(endpoints.get(name), 4, name)
        for name in ("pick", "pick_hover", "place", "place_hover")
    }
    if (
        targets["pick"][0] != targets["pick_hover"][0]
        or targets["place"][0] != targets["place_hover"][0]
        or targets["pick"][1:] != targets["place"][1:]
        or targets["pick_hover"][1:] != targets["place_hover"][1:]
    ):
        raise ValueError("transfer endpoints do not preserve separate body and base stages")
    stages = value.get("stages")
    expected = [stage for _, stage, _, _ in _CYCLE_STAGES]
    if (
        not isinstance(stages, list)
        or len(stages) != len(expected)
        or any(not isinstance(row, dict) for row in stages)
        or [row.get("name") for row in stages] != expected
    ):
        raise ValueError("transfer plan requires all eight ordered stages")
    times = {
        row["name"]: _integer(row.get("duration_ms"), "stage duration_ms", 100, 9999)
        for row in stages
    }
    per_cycle = sum(times.values())
    if (
        type(value.get("declared_ms_per_transfer")) is not int
        or value["declared_ms_per_transfer"] != per_cycle
    ):
        raise ValueError("transfer declared duration differs from its eight stages")
    return count, targets, times, per_cycle


def _gripper(value):
    if not isinstance(value, dict) or set(value) != {"open_p", "close_p", "duration_ms"}:
        raise ValueError("provide exact explicit gripper open_p, close_p and duration_ms")
    checked = {
        "open_p": _integer(value["open_p"], "open_p", 500, 2500),
        "close_p": _integer(value["close_p"], "close_p", 500, 2500),
        "duration_ms": _integer(value["duration_ms"], "gripper duration_ms", 100, 9999),
    }
    if checked["open_p"] == checked["close_p"]:
        raise ValueError("gripper open_p and close_p must differ")
    return checked


def _six_targets(arm, roll, grip):
    # 000..003是名义位置四轴，004滚转保持当前P，005为独立夹爪；全量目标不等于发送帧。
    return {str(sid): p for sid, p in enumerate([*arm, roll, grip])}


def _entry_segments(entry_plan, current, goal, already):
    if "entry_segments" not in entry_plan:
        return None
    segments = entry_plan["entry_segments"]
    if not isinstance(segments, list):
        raise ValueError("entry_segments must be an explicit finite list")
    if already:
        if segments:
            raise ValueError("already-at-start entry cannot contain motion segments")
        return []
    if not 1 <= len(segments) <= 8:
        raise ValueError("moving entry requires one..eight route segments")
    previous = list(current[:4])
    checked = []
    for segment in segments:
        if not isinstance(segment, dict) or set(segment) != {"from4_p", "to4_p", "duration_ms"}:
            raise ValueError("entry segment requires exact from4_p, to4_p and duration_ms")
        if not isinstance(segment["from4_p"], list) or not isinstance(segment["to4_p"], list):
            raise ValueError("entry segment endpoints must be four-position lists")
        start = _pulses(segment["from4_p"], 4, "entry segment from4_p")
        target = _pulses(segment["to4_p"], 4, "entry segment to4_p")
        if start != previous:
            raise ValueError(
                "entry route must start at current state and preserve every connection"
            )
        if start == target:
            raise ValueError("entry route must not contain duplicate consecutive waypoints")
        duration = _integer(segment["duration_ms"], "entry segment duration_ms", 1500, 9000)
        checked.append({"from4_p": start, "to4_p": target, "duration_ms": duration})
        previous = target
    if previous != list(goal):
        raise ValueError("entry route must end at the requested pick hover")
    return checked


def compose_transfer_task(transfer_plan, entry_plan, gripper, task_id):
    """组合两个已有规划，不查询状态、不发帧、不产生运动授权。

    entry_plan可来自显式六轴SIMULATION入口API，或已校验状态的入口规划；
    此函数保留来源，绝不把模拟值包装成硬件反馈。gripper.duration_ms仅用于
    入口后的开爪准备，循环开合时间继续采用transfer_plan中原有阶段时间。
    """
    if not isinstance(task_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", task_id):
        raise ValueError("task_id must be a bounded identifier")
    count, endpoints, times, per_cycle = _transfer(transfer_plan)
    grip = _gripper(gripper)
    if (
        not isinstance(entry_plan, dict)
        or entry_plan.get("schema") != "jixiebi.transfer_entry.v1"
        or entry_plan.get("motion_authorized") is not False
        or entry_plan.get("source") not in {"SIMULATION", "HARDWARE_STATE"}
    ):
        raise ValueError("provide an explicit non-executable transfer entry plan")
    result = {
        "schema": "jixiebi.transfer_task.v1",
        "task_id": task_id,
        "status": "BLOCKED",
        "motion_authorized": False,
        "hardware_execution_allowed": False,
        "source": entry_plan["source"],
        "entry_status": entry_plan.get("status"),
        "entry_reason": entry_plan.get("reason"),
        "entry_freshness": deepcopy(entry_plan.get("freshness")),
        "entry_planner_version": entry_plan.get("planner_version"),
        "entry_route4_p": deepcopy(entry_plan.get("route4_p")),
        # 阻塞时同样保留反馈和诊断，不把关键证据丢在提前return之前。
        "current_positions6_p": deepcopy(entry_plan.get("current_positions6_p")),
        "entry_diagnostics": {
            key: deepcopy(entry_plan.get(key))
            for key in ("start_scene_check", "goal_scene_check", "table_recovery", "search")
        },
        "cycle_count": count,
        "gripper": grip,
        "model_parameter_variant": transfer_plan.get("model_parameter_variant"),
        "model_geometry_mm": deepcopy(transfer_plan.get("model_geometry_mm")),
        "declared_ms_per_transfer": per_cycle,
        "within_10s_per_transfer_declared_only": per_cycle <= 10000,
        "entry_declared_ms": None,
        "preparation_declared_ms": grip["duration_ms"],
        "total_declared_ms": None,
        "within_10s_total_declared_only": None,
        "phases": [],
        "motion_phases": [],
        "cycle_timing": [],
        "execution_overhead_included": False,
        "supply_and_clearing_time_included": False,
        "current_timed_cycle_executor_compatible": False,
        "executor_compatibility_reason": (
            "Nominal task includes base ID 0, full six-axis state targets and up to 49 phases; "
            "existing timed_cycle excludes ID 0, limits phase/frame counts, and requires "
            "arm T>=1500ms and gripper T>=1000ms. The separate transfer_task_motion executor "
            "requires an independent exact whole-task review; this proposal is not authorization."
        ),
        "hardware_interpolation_implemented": False,
        "physical_geometry_verified": False,
        "physical_angles_verified": False,
        "physical_path_verified": False,
        "physical_grasp_verified": False,
        "physical_stop_verified": False,
        "physical_cycles_verified": False,
        "gripper_contents_known": False,
        "gripper_open_state_verified": False,
        "requires_physical_review_before_execution": True,
        "caveats": [
            "All P values are protocol positions from an unmeasured nominal model; "
            "500..2500 is a protocol boundary, not a verified mechanical range.",
            "Quintic profiles describe mathematical previews only; targets do not promise "
            "actual smoothness, synchronization, path clearance or elapsed time.",
            "Entry preserves current roll and gripper P. Gripper contents remain unknown; "
            "opening after entry is an explicit proposal and can release an unknown object.",
            "Full six-axis target dictionaries describe nominal phase states; they are "
            "not an executable frame schema or a reviewed motion budget.",
            "Object replenishment, destination clearing, feedback, communication and "
            "host scheduling time are excluded from declared timing.",
            transfer_plan.get("caveat", "Transfer collision and grasp evidence remain unverified."),
            entry_plan.get("caveat", "Entry physical path remains unverified."),
        ],
    }
    # 不可接受入口必须整趟阻塞，不能只跳过入口并留下可误读为可运行的搬运阶段。
    if entry_plan.get("status") not in _ENTRY_READY:
        result["reason"] = "Entry requires review or is blocked; no task phases were composed."
        return result
    current = _pulses(entry_plan.get("current_positions6_p"), 6, "current_positions6_p")
    if (
        _pulses(entry_plan.get("target_pick_hover_p"), 4, "target_pick_hover_p")
        != endpoints["pick_hover"]
    ):
        raise ValueError("entry target does not match transfer pick_hover")
    if entry_plan.get("held_axes_without_new_targets") != {
        "4": current[4],
        "5": current[5],
    }:
        raise ValueError("entry held axes do not match explicit current positions")
    already = entry_plan["status"] == "ALREADY_AT_START"
    segments = _entry_segments(entry_plan, current, endpoints["pick_hover"], already)
    # 每个路由段独立受舵机T上限约束；整段入口允许累加，但不能无限追加绕行段。
    entry_ms = _integer(
        entry_plan.get("entry_declared_ms"), "entry_declared_ms", 0, 72000 if segments else 9999
    )
    if (already and entry_ms != 0) or (not already and entry_ms < 100):
        raise ValueError("entry duration does not match entry classification")
    if segments is not None and sum(row["duration_ms"] for row in segments) != entry_ms:
        raise ValueError("entry declared duration differs from its exact route segments")
    if entry_plan.get("entry_and_transfer_declared_ms") != entry_ms + per_cycle:
        raise ValueError("entry timing does not match the supplied transfer plan")
    result["current_positions6_p"] = current
    result["nominal_endpoints_p"] = deepcopy(endpoints)
    previous = _six_targets(current[:4], current[4], current[5])
    elapsed = 0

    def add_phase(name, target, duration, *, cycle_index=None, transfer_stage=None):
        nonlocal previous, elapsed
        phase_id = name if cycle_index is None else f"cycle_{cycle_index:02d}_{name}"
        # motion_targets只列该阶段新设目标的轴；其余全量targets用于预期状态审阅。
        # 004不发送，入口不操作005，下降/抬升与底座转动不混为一批发送。
        if name == "entry" or name.startswith("entry_"):
            axes = range(4) if duration else ()
        elif name in {"prepare_gripper", "close", "open"}:
            axes = (5,)
        elif name in {"rotate_right", "rotate_left"}:
            axes = (0,)
        else:
            axes = (1, 2, 3)
        motion_targets = {str(sid): target[str(sid)] for sid in axes}
        row = {
            "phase_id": phase_id,
            "name": name,
            "cycle_index": cycle_index,
            "start_targets": dict(previous),
            "targets": dict(target),
            "motion_targets": motion_targets,
            "duration_ms": duration,
            "declared_start_ms": elapsed,
            "declared_end_ms": elapsed + duration,
            "profile": "QUINTIC_PROGRESS_MODEL_ONLY",
            "physical_path_verified": False,
            "hardware_interpolation_implemented": False,
        }
        if transfer_stage is not None:
            row["transfer_stage"] = transfer_stage
        if name in {"prepare_gripper", "close", "open"}:
            row["profile"] = "DECLARED_GRIPPER_DURATION_ONLY"
            row["gripper_position_and_force_calibrated"] = False
        result["phases"].append(row)
        if duration > 0 and motion_targets:
            result["motion_phases"].append(
                {"name": phase_id, "duration_ms": duration, "targets": dict(motion_targets)}
            )
        previous = dict(target)
        elapsed += duration
        return row

    if segments:
        # 保留规划器给出的每个绕行点，不把RRT路线折叠为可能碰撞的首末点直连。
        # 每段仍只是独立轴P/T目标，不能据此宣称实际同步插补或连续碰撞保证。
        for index, segment in enumerate(segments, 1):
            entry = add_phase(
                f"entry_{index:02d}",
                _six_targets(segment["to4_p"], current[4], current[5]),
                segment["duration_ms"],
            )
            entry.update(
                motion_required=True,
                accepted_with_nominal_tolerance_only=False,
                profile="JOINT_ROUTE_WAYPOINT_MODEL_ONLY",
            )
    else:
        entry = add_phase(
            "entry", _six_targets(endpoints["pick_hover"], current[4], current[5]), entry_ms
        )
        entry["motion_required"] = not already
        entry["accepted_with_nominal_tolerance_only"] = already
        if already:
            entry["profile"] = "NOMINAL_START_ACCEPTANCE_NO_ENTRY_MOTION"
            entry["caveat"] = (
                "Zero duration accepts the entry planner's nominal start tolerance; "
                "it does not command repositioning to the exact nominal target."
            )
    add_phase(
        "prepare_gripper",
        _six_targets(endpoints["pick_hover"], current[4], grip["open_p"]),
        grip["duration_ms"],
    )
    for cycle in range(1, count + 1):
        cycle_start = elapsed
        for name, stage, point, action in _CYCLE_STAGES:
            add_phase(
                name,
                _six_targets(endpoints[point], current[4], grip[action + "_p"]),
                times[stage],
                cycle_index=cycle,
                transfer_stage=stage,
            )
        result["cycle_timing"].append(
            {
                "cycle_index": cycle,
                "declared_start_ms": cycle_start,
                "declared_end_ms": elapsed,
                "declared_duration_ms": elapsed - cycle_start,
            }
        )
    result.update(
        status="NOMINAL_TASK_PROPOSAL",
        entry_declared_ms=entry_ms,
        total_declared_ms=elapsed,
        within_10s_total_declared_only=elapsed <= 10000,
        reason="One finite nominal entry, preparation and repeated transfer task; review only.",
    )
    return result
