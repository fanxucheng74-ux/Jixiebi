"""从任一模型有效姿态规划入口；直连优先、RRT-Connect回退，不持有设备。"""

from dataclasses import asdict
import math

from arm_project.joint_path import plan_joint_path
from arm_project.clearance_recovery import TableRecovery, RecoverySceneGuard
from arm_project.kinematics import forward, from_nominal_pulses, finite
from arm_project.motion_scene import SceneValidator
from arm_project.route_timing import partition_joint_route
from arm_project.table_escape import plan_table_escape
from arm_project.transfer_entry import _fresh_positions, _model_and_targets, _policy, _pulses


_PLANNER_FIELDS = {
    "schema",
    "step_p",
    "resolution_p",
    "max_iterations",
    "max_nodes",
    "time_budget_s",
    "seed",
    "max_segments",
    "maximum_velocity_p_s",
    "maximum_acceleration_p_s2",
    "maximum_jerk_p_s3",
    "minimum_segment_ms",
    "maximum_segment_ms",
}


def _settings(value):
    if not isinstance(value, dict) or set(value) != _PLANNER_FIELDS:
        raise ValueError("完整的通用入口规划参数缺失或有未知字段")
    if value["schema"] != "jixiebi.general_entry_planner.v1":
        raise ValueError("通用入口参数版本错误")
    for key, low, high in (
        ("step_p", 1, 2000),
        ("resolution_p", 1, 2000),
        ("max_iterations", 1, 10000),
        ("max_nodes", 2, 10000),
        ("seed", 0, 2**32 - 1),
    ):
        if type(value[key]) is not int or not low <= value[key] <= high:
            raise ValueError("通用入口搜索参数越界: " + key)
    if not 0.001 <= finite(value["time_budget_s"], "time_budget_s") <= 10:
        raise ValueError("通用入口搜索时限越界")
    for key in ("maximum_velocity_p_s", "maximum_acceleration_p_s2", "maximum_jerk_p_s3"):
        rows = value[key]
        if not isinstance(rows, list) or len(rows) != 4:
            raise ValueError("四轴速度/加速度/jerk参数须显式提供")
        if any(not 0 < finite(v, key) <= 100000 for v in rows):
            raise ValueError("四轴动态参数须为有限正数，单位为协议P而非实测角度")
    if any(
        type(value[k]) is not int
        for k in ("minimum_segment_ms", "maximum_segment_ms", "max_segments")
    ):
        raise ValueError("阶段时长和数量须为整数")
    if not 1500 <= value["minimum_segment_ms"] <= value["maximum_segment_ms"] <= 9000:
        raise ValueError("通用入口每段须在现有1500..9000ms执行范围内")
    if not 1 <= value["max_segments"] <= 8:
        raise ValueError("入口最多8段，不能超出完整任务预算")
    return value


def _duration(start, goal, settings):
    # 五次进度s(u)的导数最大值依次为1.875、10√3/3、60。
    # 这些约束只约束名义数学轨迹；P/T舵机内部没有因此自动采用该插补曲线。
    seconds = settings["minimum_segment_ms"] / 1000
    for axis, delta in enumerate(abs(b - a) for a, b in zip(start, goal)):
        seconds = max(
            seconds,
            1.875 * delta / settings["maximum_velocity_p_s"][axis],
            math.sqrt(
                (10 * math.sqrt(3) / 3) * delta / settings["maximum_acceleration_p_s2"][axis]
            ),
            (60 * delta / settings["maximum_jerk_p_s3"][axis]) ** (1 / 3),
        )
    return math.ceil(seconds * 1000)


def _plan(model, transfer, positions, policy, scene_config, settings, *, source, freshness):
    settings = _settings(settings)
    geometry, _, targets, table_z = _model_and_targets(model, transfer)
    scene = SceneValidator(model, scene_config)
    if table_z != scene_config["table_z_mm"]:
        raise ValueError("入口场景和搬运方案的桌面坐标不一致")
    start, goal = positions[:4], tuple(targets["pick_hover"])
    result = {
        "schema": "jixiebi.transfer_entry.v1",
        "planner_version": "GENERAL_JOINT_SPACE_V2",
        "source": source,
        "freshness": freshness,
        "motion_authorized": False,
        "current_positions6_p": list(positions),
        "target_pick_hover_p": list(goal),
        "held_axes_without_new_targets": {"4": positions[4], "5": positions[5]},
        "current_pose_nominal_mm": asdict(forward(geometry, from_nominal_pulses(start))),
        "samples": [],
        "entry_segments": [],
        "route4_p": [],
        "entry_declared_ms": 0,
        "entry_and_transfer_declared_ms": None,
        "physical_geometry_verified": False,
        "physical_path_verified": False,
        "hardware_interpolation_implemented": False,
        "asynchronous_motion_envelope_verified": False,
        "gripper_contents_known": False,
        "gripper_open_state_verified": False,
        "task_ready_verified": False,
        "caveat": "Sampled nominal capsules and configured obstacles only; base/roll/open claw/load/cables and actual servo interpolation are not certified.",
    }
    first, final = scene.inspect(start), scene.inspect(goal)
    result.update(start_scene_check=first, goal_scene_check=final, table_recovery=None)
    if not final["valid"]:
        return {
            **result,
            "status": "NOMINAL_PATH_BLOCKED",
            "reason": "INVALID_GOAL: " + final["reason"],
        }
    recovery = None
    prefix = [list(start)]
    if not first["valid"]:
        # 只有普通桌面预留余量不足、且全套硬底线/其他碰撞检查通过，才可尝试脱离。
        # 不把TABLE_CLEARANCE直接改成valid，也不让模型穿桌起点绕过检查。
        recovery = TableRecovery(scene, start, resolution_p=1)
        result["table_recovery"] = recovery.inspect_start()
        if not recovery.eligible:
            return {
                **result,
                "status": "NOMINAL_PATH_BLOCKED",
                "reason": "INVALID_START: "
                + first["reason"]
                + "; minimum_clearance_mm="
                + str(first.get("minimum_table_clearance_mm"))
                + "; recovery_hard_floor_mm="
                + str(recovery.hard_clearance_mm),
            }
        escape = plan_table_escape(
            scene,
            start,
            goal,
            resolution_p=1,
            max_segments=min(3, settings["max_segments"]),
            time_budget_s=min(0.5, settings["time_budget_s"]),
        )
        result["table_recovery"]["escape_search"] = escape
        if escape["status"] != "ESCAPE_FOUND":
            return {
                **result,
                "status": "NOMINAL_PATH_BLOCKED",
                "reason": "TABLE_RECOVERY: " + escape["status"],
            }
        prefix = escape["waypoints4_p"]
    # 只把“已在目标容差内”当作可省略动作；其它姿态不再与右高位或直立模板匹配。
    if first["valid"] and max(abs(a - b) for a, b in zip(start, goal)) <= min(
        3, policy["at_start_tolerance_p"]
    ):
        return {
            **result,
            "status": "ALREADY_AT_START",
            "reason": "当前反馈已在工作起点容差内",
            "route4_p": [list(start)],
            "entry_and_transfer_declared_ms": transfer["declared_ms_per_transfer"],
        }
    remaining = settings["max_segments"] - (len(prefix) - 1)
    if prefix[-1] == list(goal):
        search = {
            "status": "PATH_FOUND",
            "waypoints4_p": [list(goal)],
            "stats": {"method": "CONSTRAINED_TABLE_ESCAPE"},
        }
    elif remaining < 1:
        return {**result, "status": "NOMINAL_PATH_BLOCKED", "reason": "脱离路径已占满入口段数预算"}
    else:
        controls = {
            key: settings[key]
            for key in (
                "step_p",
                "resolution_p",
                "max_iterations",
                "max_nodes",
                "time_budget_s",
                "seed",
            )
        }
        search = plan_joint_path(
            prefix[-1], goal, scene.bounds4_p, scene, max_segments=remaining, **controls
        )
    result["search"] = search
    if search["status"] != "PATH_FOUND":
        return {
            **result,
            "status": "NOMINAL_PATH_BLOCKED",
            "reason": search["status"] + ": " + search["stats"]["reason"],
        }
    # 保留原转折，用满足时长的最少等分；不能因二分多造一段而误判预算不足。
    # 正常空间中的后续路径只需原场景规则，不继续背负软余量起点的逐链高度底线。
    # 这个无状态并集仅用于候选几何检查；下面有序validate_route仍防止提前回落。
    validity = scene if recovery is None else lambda p: scene(p) or recovery.state_valid(p)
    try:
        route = partition_joint_route(
            prefix + search["waypoints4_p"][1:],
            max_segments=settings["max_segments"],
            maximum_segment_ms=settings["maximum_segment_ms"],
            duration_fn=lambda a, b: _duration(a, b, settings),
            state_valid=validity,
            resolution_p=settings["resolution_p"],
        )
    except ValueError as error:
        return {**result, "status": "NOMINAL_PATH_BLOCKED", "reason": str(error)}
    if recovery is not None:
        checked = recovery.validate_route(route)
        result["table_recovery"]["route_check"] = checked
        if not checked["valid"]:
            return {**result, "status": "NOMINAL_PATH_BLOCKED", "reason": checked["reason"]}
    preview_guard = scene if recovery is None else RecoverySceneGuard(scene, recovery)
    segments, elapsed, samples = [], 0, []
    for start, goal in zip(route, route[1:]):
        duration = _duration(start, goal, settings)
        if duration > settings["maximum_segment_ms"]:
            return {
                **result,
                "status": "NOMINAL_PATH_BLOCKED",
                "reason": "名义动态限值要求单段时长超过预算；未输出可执行路线",
            }
        segments.append({"from4_p": start, "to4_p": goal, "duration_ms": duration})
        count = max(
            1, math.ceil(max(abs(b - a) for a, b in zip(start, goal)) / settings["resolution_p"])
        )
        if any(
            not validity([round(a + (b - a) * i / count) for a, b in zip(start, goal)])
            for i in range(count + 1)
        ):
            return {
                **result,
                "status": "NOMINAL_PATH_BLOCKED",
                "reason": "细分后的整数路径重新校验失败",
            }
        # 这些仅为阅读用预览；组合器执行entry_segments，不能丢弃RRT中间路点。
        for i in range(17):
            u = i / 16
            s = u * u * u * (10 + u * (-15 + 6 * u))
            pulses = [round(a + (b - a) * s) for a, b in zip(start, goal)]
            if not preview_guard(pulses):
                return {
                    **result,
                    "status": "NOMINAL_PATH_BLOCKED",
                    "reason": "时间参数化后的离散状态重新校验失败",
                }
            samples.append({"time_ms": elapsed + round(duration * u, 3), "positions4_p": pulses})
        elapsed += duration
    return {
        **result,
        "status": "NOMINAL_ENTRY_PROPOSAL",
        "reason": "从当前关节位置规划到工作起点，无预设姿态分类前置",
        "route4_p": route,
        "entry_segments": segments,
        "entry_declared_ms": elapsed,
        "samples": samples,
        "entry_and_transfer_declared_ms": elapsed + transfer["declared_ms_per_transfer"],
    }


def plan_general_transfer_entry(
    model,
    transfer,
    payload,
    policy,
    scene_config,
    settings,
    *,
    now_monotonic_s,
    host_boot_id,
    expected_session_id=None,
):
    """保留同机新鲜六轴校验，再从当前位置搜索，不用旧日志猜姿态。"""
    checked = _policy(policy)
    positions, freshness = _fresh_positions(
        payload, checked, now_monotonic_s, host_boot_id, expected_session_id
    )
    return _plan(
        model,
        transfer,
        positions,
        checked,
        scene_config,
        settings,
        source="HARDWARE_STATE",
        freshness=freshness,
    )


def plan_simulated_general_entry(model, transfer, positions6_p, policy, scene_config, settings):
    """显式模拟路径，测试与演示不构造虚假硬件反馈。"""
    return _plan(
        model,
        transfer,
        _pulses(positions6_p, 6),
        _policy(policy),
        scene_config,
        settings,
        source="SIMULATION",
        freshness=None,
    )
