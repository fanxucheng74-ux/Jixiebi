"""从新鲜六轴反馈计算搬运入口提案；不连接设备，也不授权运动。"""

from dataclasses import asdict
import math
from uuid import UUID

from arm_project.approach_planning import chain_points
from arm_project.kinematics import Geometry, finite, forward, from_nominal_pulses, validate_limits


POLICY_FIELDS = {
    "max_age_s",
    "at_start_tolerance_p",
    "high_pose_tolerance_p",
    "minimum_tip_height_above_table_mm",
    "minimum_centerline_height_mm",
    "maximum_radial_mm",
    "duration_ms",
    "max_joint_delta_p",
}


def _bounded(value, name, low, high, *, integer=False):
    if integer and type(value) is not int:
        raise ValueError(name + " must be an integer")
    number = finite(value, name)
    if not low <= number <= high:
        raise ValueError(name + " outside the entry planning domain")
    return int(number) if integer else number


def _policy(value):
    if not isinstance(value, dict) or set(value) != POLICY_FIELDS:
        raise ValueError("provide exact explicit entry policy fields")
    result = {
        "max_age_s": _bounded(value["max_age_s"], "max_age_s", 0.01, 60),
        "at_start_tolerance_p": _bounded(
            value["at_start_tolerance_p"], "at_start_tolerance_p", 0, 30, integer=True
        ),
        "high_pose_tolerance_p": _bounded(
            value["high_pose_tolerance_p"], "high_pose_tolerance_p", 0, 100, integer=True
        ),
        "minimum_tip_height_above_table_mm": _bounded(
            value["minimum_tip_height_above_table_mm"],
            "minimum_tip_height_above_table_mm",
            5,
            1000,
        ),
        "minimum_centerline_height_mm": _bounded(
            value["minimum_centerline_height_mm"],
            "minimum_centerline_height_mm",
            0,
            100,
        ),
        "maximum_radial_mm": _bounded(value["maximum_radial_mm"], "maximum_radial_mm", 20, 1000),
        "duration_ms": _bounded(value["duration_ms"], "duration_ms", 100, 9999, integer=True),
        "max_joint_delta_p": _bounded(
            value["max_joint_delta_p"], "max_joint_delta_p", 1, 2000, integer=True
        ),
    }
    if result["at_start_tolerance_p"] > result["high_pose_tolerance_p"]:
        raise ValueError("start tolerance cannot exceed recognized high-pose tolerance")
    return result


def _pulses(value, size):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != size
        or any(type(p) is not int or not 500 <= p <= 2500 for p in value)
    ):
        raise ValueError(f"expected {size} protocol integer positions in 500..2500")
    return tuple(value)


def _boot(value):
    if not isinstance(value, str):
        raise ValueError("host_boot_id must be a UUID string")
    try:
        return str(UUID(value))
    except (ValueError, AttributeError) as error:
        raise ValueError("invalid host_boot_id") from error


def _fresh_positions(payload, policy, now_monotonic_s, host_boot_id, expected_session_id):
    # monotonic只可比较同一主机启动内的接收时间；boot标识不提供来源身份认证。
    now = _bounded(now_monotonic_s, "now_monotonic_s", 0, 1e15)
    boot = _boot(host_boot_id)
    if not isinstance(payload, dict) or payload.get("schema") != "jixiebi.raw_state.v1":
        raise ValueError("expected raw_state.v1 envelope")
    session = payload.get("session_id")
    if not isinstance(session, str) or not session or len(session) > 128:
        raise ValueError("missing bounded session_id")
    if expected_session_id is not None and session != expected_session_id:
        raise ValueError("state session differs from the expected session")
    if type(payload.get("batch_index")) is not int or payload["batch_index"] < 1:
        raise ValueError("invalid batch_index")
    state = payload.get("state")
    if (
        not isinstance(state, dict)
        or state.get("source") != "HARDWARE"
        or state.get("status") != "OK"
        or "error" not in state
        or state["error"] is not None
        or state.get("position_unit") != "protocol_P"
        or state.get("simultaneous") is not False
        or state.get("timestamp_origin") != "host_receive"
        or _boot(state.get("host_boot_id")) != boot
    ):
        raise ValueError("state is incomplete, not hardware, or from a different host boot")
    rows = state.get("servos")
    if not isinstance(rows, list) or len(rows) != 6:
        raise ValueError("six current servo rows are required")
    pulses, times = [], []
    for sid, row in enumerate(rows):
        if (
            not isinstance(row, dict)
            or type(row.get("servo_id")) is not int
            or row["servo_id"] != sid
            or row.get("status") != "OK"
        ):
            raise ValueError("servo records must be complete and ordered 0 through 5")
        pulse = _pulses([row.get("position_p")], 1)[0]
        received = _bounded(row.get("received_monotonic_s"), "received_monotonic_s", 0, 1e15)
        if received > now or now - received > policy["max_age_s"]:
            raise ValueError("servo sample stale or from the future")
        if times and received <= times[-1]:
            raise ValueError("servo receive times must increase")
        pulses.append(pulse)
        times.append(received)
    finished = _bounded(state.get("finished_monotonic_s"), "finished_monotonic_s", 0, 1e15)
    if not times[-1] <= finished <= now:
        raise ValueError("state finish does not bound receive times")
    return tuple(pulses), {
        "session_id": session,
        "batch_index": payload["batch_index"],
        "host_boot_id": boot,
        "oldest_sample_age_s": now - times[0],
        "newest_sample_age_s": now - times[-1],
        "sequential_samples": True,
        "metadata_is_source_authentication": False,
    }


def _model_and_targets(model, transfer_plan):
    if (
        not isinstance(model, dict)
        or model.get("source") != "VENDOR_NOMINAL_UNMEASURED"
        or model.get("frame") != "vendor_X_right_Y_forward_Z_up"
        or model.get("endpoint") != "closed_gripper_tip"
    ):
        raise ValueError("entry requires the explicit unmeasured closed-tip model")
    geometry = Geometry(**model["geometry_mm"])
    limits = validate_limits(model["nominal_angular_limits_deg"])
    if (
        not isinstance(transfer_plan, dict)
        or transfer_plan.get("status") != "NOMINAL_PROPOSAL_ONLY"
        or transfer_plan.get("motion_authorized") is not False
    ):
        raise ValueError("entry requires the matching non-executable nominal transfer plan")
    if transfer_plan.get("model_geometry_mm") != model["geometry_mm"] or transfer_plan.get(
        "model_parameter_variant"
    ) != model.get("parameter_variant"):
        raise ValueError("transfer plan and geometry model do not match")
    _bounded(
        transfer_plan.get("declared_ms_per_transfer"),
        "declared_ms_per_transfer",
        800,
        80000,
        integer=True,
    )
    endpoints = transfer_plan.get("nominal_endpoints_p")
    modeled = transfer_plan.get("nominal_endpoints_model")
    if not isinstance(endpoints, dict) or not isinstance(modeled, dict):
        raise ValueError("transfer plan lacks named endpoints")
    targets = {}
    for name in ("pick_hover", "place_hover"):
        targets[name] = _pulses(endpoints.get(name), 4)
        calculated = asdict(forward(geometry, from_nominal_pulses(targets[name])))
        reference = modeled.get(name)
        if not isinstance(reference, dict) or set(reference) != set(calculated):
            raise ValueError("transfer plan endpoint model is incomplete")
        if any(abs(finite(reference[k], k) - v) > 0.001 for k, v in calculated.items()):
            raise ValueError("transfer plan and geometry model do not match")
    assumptions = transfer_plan.get("input_assumptions")
    if not isinstance(assumptions, dict):
        raise ValueError("transfer plan lacks table assumptions")
    table_z = _bounded(assumptions.get("table_z_mm"), "table_z_mm", -1000, 1000)
    return geometry, limits, targets, table_z


def _inspect(geometry, limits, pulses, table_z, policy):
    joints = from_nominal_pulses(pulses)
    if any(not limits[k][0] <= v <= limits[k][1] for k, v in vars(joints).items()):
        raise ValueError("nominal angular limit exceeded")
    points = chain_points(geometry, pulses)
    centerline = min(p[2] - table_z for p in points)
    radial = max(math.hypot(p[0], p[1]) for p in points)
    if centerline < policy["minimum_centerline_height_mm"]:
        raise ValueError("sampled nominal centerline below table clearance")
    if radial > policy["maximum_radial_mm"]:
        raise ValueError("sampled nominal centerline outside radial envelope")
    return forward(geometry, joints), centerline, radial


def _classify(model, transfer_plan, positions, policy, *, source, freshness):
    geometry, limits, targets, table_z = _model_and_targets(model, transfer_plan)
    start = positions[:4]
    result = {
        "schema": "jixiebi.transfer_entry.v1",
        "source": source,
        "motion_authorized": False,
        "current_positions6_p": list(positions),
        "target_pick_hover_p": list(targets["pick_hover"]),
        "freshness": freshness,
        "hardware_sample_freshness_checked": source == "HARDWARE_STATE",
        "physical_geometry_verified": False,
        "physical_angle_verified": False,
        "physical_path_verified": False,
        "hardware_interpolation_implemented": False,
        "gripper_contents_known": False,
        "gripper_open_state_verified": False,
        "task_ready_verified": False,
        "held_axes_without_new_targets": {"4": positions[4], "5": positions[5]},
        "samples": [],
        "entry_declared_ms": 0,
        "entry_and_transfer_declared_ms": None,
        "within_10s_total_declared_only": None,
        "execution_overhead_included": False,
        "initial_gripper_preparation_included": False,
        "requires_physical_review_before_execution": True,
        "caveat": "Nominal joint interpolation only; no object, link-width, obstacle, cable or physical-stop proof.",
    }
    try:
        pose, centerline, radial = _inspect(geometry, limits, start, table_z, policy)
    except ValueError as error:
        return {**result, "status": "NOMINAL_PATH_BLOCKED", "reason": str(error)}
    result["current_pose_nominal_mm"] = asdict(pose)
    result["current_nominal_tip_height_above_table_mm"] = pose.z_mm - table_z
    result["current_minimum_centerline_height_mm"] = centerline
    result["current_maximum_centerline_radial_mm"] = radial
    if pose.z_mm - table_z < policy["minimum_tip_height_above_table_mm"]:
        return {
            **result,
            "status": "REQUIRES_REVIEWED_LIFT",
            "reason": "Current nominal tip is low; do not guess a horizontal sweep or an unreviewed lift.",
        }
    if (
        max(abs(a - b) for a, b in zip(start, targets["pick_hover"]))
        <= policy["at_start_tolerance_p"]
    ):
        return {
            **result,
            "status": "ALREADY_AT_START",
            "entry_and_transfer_declared_ms": transfer_plan["declared_ms_per_transfer"],
            "within_10s_total_declared_only": transfer_plan["declared_ms_per_transfer"] <= 10000,
            "reason": "Position feedback is within the nominal left-hover window; gripper and scene remain unverified.",
        }
    if (
        max(abs(a - b) for a, b in zip(start, targets["place_hover"]))
        > policy["high_pose_tolerance_p"]
    ):
        return {
            **result,
            "status": "REQUIRES_POSE_REVIEW",
            "reason": "High posture does not match the recognized right hover; no arbitrary return trajectory is generated.",
        }
    target = targets["pick_hover"]
    if max(abs(a - b) for a, b in zip(start, target)) > policy["max_joint_delta_p"]:
        return {
            **result,
            "status": "NOMINAL_PATH_BLOCKED",
            "reason": "Entry exceeds the explicit nominal joint-distance budget.",
        }
    samples = []
    try:
        for index in range(65):
            u = index / 64
            # 五次进度的端点速度/加速度为0；此处只生成预览，不向舵机流式发目标。
            s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
            pulses = tuple(round(a + (b - a) * s) for a, b in zip(start, target))
            pose, centerline, radial = _inspect(geometry, limits, pulses, table_z, policy)
            if pose.z_mm - table_z < policy["minimum_tip_height_above_table_mm"]:
                raise ValueError("entry leaves the nominal high-tip corridor")
            samples.append(
                {
                    "time_ms": round(policy["duration_ms"] * u, 3),
                    "positions4_p": list(pulses),
                    "tip_nominal_mm": asdict(pose),
                    "minimum_centerline_height_mm": centerline,
                    "maximum_centerline_radial_mm": radial,
                }
            )
    except ValueError as error:
        return {**result, "status": "NOMINAL_PATH_BLOCKED", "reason": str(error)}
    return {
        **result,
        "status": "NOMINAL_ENTRY_PROPOSAL",
        "reason": "Recognized right hover to left hover, with sampled nominal high-corridor checks only.",
        "entry_declared_ms": policy["duration_ms"],
        "entry_and_transfer_declared_ms": policy["duration_ms"]
        + transfer_plan["declared_ms_per_transfer"],
        "within_10s_total_declared_only": policy["duration_ms"]
        + transfer_plan["declared_ms_per_transfer"]
        <= 10000,
        "profile": "QUINTIC_PROGRESS_MODEL_ONLY",
        "samples": samples,
        "maximum_nominal_joint_delta_p": max(abs(a - b) for a, b in zip(start, target)),
        "minimum_sampled_centerline_height_mm": min(
            s["minimum_centerline_height_mm"] for s in samples
        ),
    }


def plan_transfer_entry(
    model,
    transfer_plan,
    payload,
    policy,
    *,
    now_monotonic_s,
    host_boot_id,
    expected_session_id=None,
):
    """使用同机新鲜六轴状态计算名义入口；元数据校验不等于运动或来源授权。"""
    checked = _policy(policy)
    positions, freshness = _fresh_positions(
        payload, checked, now_monotonic_s, host_boot_id, expected_session_id
    )
    return _classify(
        model, transfer_plan, positions, checked, source="HARDWARE_STATE", freshness=freshness
    )


def plan_simulated_transfer_entry(model, transfer_plan, positions6_p, policy):
    """仅用显式模拟P值演示入口，不构造伪造HARDWARE时间或反馈。"""
    return _classify(
        model,
        transfer_plan,
        _pulses(positions6_p, 6),
        _policy(policy),
        source="SIMULATION",
        freshness=None,
    )
