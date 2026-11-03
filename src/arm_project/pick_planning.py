"""Known-position paper pickup planning; nominal, offline, and never executable.

Task frame: origin at the base centre on the tabletop, +Y in the current
nominal arm bearing, +X right, +Z up. A measured upright tip height can align
the table vertically, but cannot calibrate joint angles, links or the TCP.
"""

from dataclasses import asdict
import math

from arm_project.approach_planning import chain_points, is_nominal_upright_reference
from arm_project.kinematics import (
    Geometry,
    Joints,
    Pose,
    finite,
    forward,
    from_nominal_pulses,
    inverse,
    to_nominal_pulses,
    validate_limits,
)


class PickPlanningError(ValueError):
    pass


# 本文件输出候选几何方案，不打开串口，也不把motion_authorized改成True。
# 任务坐标以桌面底座中心为原点；厂家模型坐标的桌面高度可能需要table_z_mm偏移。
def shoulder_solutions_for_z(geometry, limits, reference_pulses, target_z_mm):
    """Analytic height IK with yaw/elbow/wrist held; XY is reported, not forced.

    This supports a deliberately chosen reachable stopping point for an object
    placed afterward. It cannot claim to reach an arbitrary prescribed XYZ.
    """
    if not isinstance(geometry, Geometry):
        raise ValueError("Geometry required")
    bounds = validate_limits(limits)
    q = from_nominal_pulses(reference_pulses)
    target = finite(target_z_mm, "target_z_mm")
    # 保持底座/肘/腕参考角不变，只求肩角；因此只约束Z，求出的XY会随肩角一起变化。
    elbow, wrist = math.radians(q.elbow_flex_deg), math.radians(q.wrist_deg)
    a = (
        geometry.upper_arm_mm
        + geometry.forearm_mm * math.cos(-elbow)
        + geometry.tool_mm * math.cos(-elbow + wrist)
    )
    b = geometry.forearm_mm * math.sin(-elbow) + geometry.tool_mm * math.sin(-elbow + wrist)
    # 高度式整理为 a*sin(肩角)+b*cos(肩角)，再合成为一个带相位的正弦函数。
    magnitude = math.hypot(a, b)
    z = target - geometry.base_height_mm
    if magnitude < 1e-9:
        angles = [q.shoulder_deg] if abs(z) < 1e-8 else []
    elif abs(z) > magnitude + 1e-8:
        return ()
    else:
        value = math.asin(max(-1, min(1, z / magnitude)))
        phase = math.atan2(b, a)
        # asin有两个分支，再枚举整圈等价角；后面仍按显式肩轴限位过滤。
        angles = [
            math.degrees(root - phase) + turn
            for root in (value, math.pi - value)
            for turn in (-360, 0, 360)
        ]
    found = {}
    for shoulder in angles:
        if not bounds["shoulder_deg"][0] - 1e-8 <= shoulder <= bounds["shoulder_deg"][1] + 1e-8:
            continue
        solution = Joints(
            q.yaw_deg,
            min(bounds["shoulder_deg"][1], max(bounds["shoulder_deg"][0], shoulder)),
            q.elbow_flex_deg,
            q.wrist_deg,
        )
        if any(
            not bounds[n][0] - 1e-8 <= v <= bounds[n][1] + 1e-8 for n, v in vars(solution).items()
        ):
            continue
        # 先量化为真正可发送的整数P，再重算名义高度，保留量化后的高度误差。
        pulses = to_nominal_pulses(solution)
        pose = forward(geometry, from_nominal_pulses(pulses))
        found[pulses] = {
            "pulses_p": list(pulses),
            "pose_model": asdict(pose),
            "height_error_mm": pose.z_mm - target,
            "motion_authorized": False,
        }
    # 优先肩轴P变化较小的候选；排序不等于已经审阅了从当前姿态到候选的路径。
    return tuple(sorted(found.values(), key=lambda v: abs(v["pulses_p"][1] - reference_pulses[1])))


def _number(request, name, low, high):
    value = finite(request[name], name)
    if not low <= value <= high:
        raise ValueError(name + " outside planning domain")
    return value


def task_to_model(point, yaw_deg, table_z_mm):
    """Right/forward/up to vendor XY/Z; yaw is measured from +Y toward +X."""
    if not isinstance(point, (list, tuple)) or len(point) != 3:
        raise ValueError("point requires three coordinates")
    x, y, z = [finite(v, "point") for v in point]
    # 用当前任务朝向旋转水平XY，再给Z加桌面在模型坐标中的高度偏移；长度均为mm。
    angle = math.radians(finite(yaw_deg, "yaw_deg"))
    return [
        x * math.cos(angle) + y * math.sin(angle),
        -x * math.sin(angle) + y * math.cos(angle),
        z + finite(table_z_mm, "table_z_mm"),
    ]


def tip_for_grasp(center, bearing_deg, pitch_deg, tip_to_grasp_mm):
    """grasp TCP = tip - offset * tool direction; never silently confuse them."""
    yaw, pitch = math.radians(bearing_deg), math.radians(pitch_deg)
    distance = finite(tip_to_grasp_mm, "tip_to_grasp_mm")
    if distance < 0:
        raise ValueError("tip-to-grasp distance must be nonnegative")
    # center是希望物体处于的抓取中心；FK/IK定义的端点却是爪尖，需沿工具方向补偏置。
    # direction是单位方向向量，distance是“爪尖到抓取中心”的mm距离。
    direction = [math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), math.sin(pitch)]
    return Pose(*(c + distance * u for c, u in zip(center, direction)), pitch_deg)


def continuous_segment(
    geometry,
    limits,
    start,
    target,
    table_z_mm,
    duration_ms,
    *,
    plane_margin_mm=5,
    behind_base_margin_mm=2,
):
    """Sample joint-linear progress at <=1P per axis, without small-stop commands.

    This is a geometric preview, not a proof of real servo synchronization,
    continuous collision clearance, link widths, or object contact.
    """
    from_nominal_pulses(start)
    from_nominal_pulses(target)
    # 以最大轴P差决定抽样数，使每个轴相邻样点最多变化1P；这些样点不会逐条发给舵机。
    count = max(1, max(abs(a - b) for a, b in zip(start, target)))
    rows = []
    minimum = float("inf")
    maximum_radius = 0.0
    yaw = math.radians(from_nominal_pulses(start).yaw_deg)
    for i in range(count + 1):
        # 假设各关节按相同比例线性推进，用于名义几何预览，不代表真实机械臂严格同步。
        fraction = i / count
        pulses = [round(a + (b - a) * fraction) for a, b in zip(start, target)]
        joints = from_nominal_pulses(pulses)
        if any(
            not limits[n][0] - 1e-8 <= v <= limits[n][1] + 1e-8 for n, v in vars(joints).items()
        ):
            raise PickPlanningError("joint-linear preview leaves nominal angular bounds")
        points = chain_points(geometry, pulses)
        # 检查整条连杆中心线节点，而非只看爪尖；不包含连杆粗细和物体外形。
        clearance = min(p[2] - table_z_mm for p in points)
        minimum = min(minimum, clearance)
        if clearance < plane_margin_mm:
            raise PickPlanningError("sampled link centreline crosses the assumed tabletop margin")
        # Restrict this demonstration to the front half-plane; no blind fold
        # behind the base merely to make the endpoint mathematically reachable.
        if any(
            p[0] * math.sin(yaw) + p[1] * math.cos(yaw) < -behind_base_margin_mm for p in points
        ) and not is_nominal_upright_reference(geometry, pulses, points=points):
            raise PickPlanningError("sampled link folds behind the current forward plane")
        # 保存整段最大水平半径，便于调用方核对已清空的桌面区域。
        maximum_radius = max(maximum_radius, *(math.hypot(p[0], p[1]) for p in points))
        rows.append(
            {
                "fraction": fraction,
                "time_s": duration_ms / 1000 * fraction,
                "pulses_p": pulses,
                "points_mm": points,
                "tip_model": asdict(forward(geometry, joints)),
            }
        )
    return {
        "start_p": list(start),
        "target_p": list(target),
        "duration_ms": duration_ms,
        # 执行候选每个变化轴仅一帧目标+T；几何samples不是“小步走停”的运动指令。
        "commands": [
            {
                "servo_id": axis,
                "target_p": b,
                "duration_ms": duration_ms,
                "ascii": f"#{axis:03d}P{b:04d}T{duration_ms:04d}!",
            }
            for axis, (a, b) in enumerate(zip(start, target))
            if a != b
        ],
        "minimum_sampled_table_clearance_mm": minimum,
        "maximum_sampled_radius_mm": maximum_radius,
        "sampling_max_axis_increment_p": 1,
        "samples": rows,
        "progress_model": "JOINT_LINEAR_GEOMETRIC_PREVIEW",
        "physical_path_verified": False,
        "motion_authorized": False,
    }


def plan_paper_pick(model, request, start_pulses):
    """Produce a full candidate pickup/return with explicit frame assumptions."""
    if (
        model.get("source") != "VENDOR_NOMINAL_UNMEASURED"
        or model.get("endpoint") != "closed_gripper_tip"
    ):
        raise ValueError("explicit nominal closed-tip model required")
    if (
        not isinstance(request, dict)
        or type(request.get("schema_version")) is not int
        or request["schema_version"] != 1
    ):
        raise ValueError("request schema_version must be1")
    required = {
        "schema_version",
        "base_front_edge_offset_mm",
        "base_edge_source",
        "outside_edge_distance_mm",
        "distance_reference",
        "object_radius_mm",
        "measured_home_tip_height_mm",
        "tip_to_grasp_offset_mm",
        "tcp_offset_source",
        "approach_clearance_mm",
        "jaw_opening_margin_mm",
        "duration_ms",
    }
    if set(request) != required:
        raise ValueError("unknown or missing paper request fields")
    # 来源标签保留“测量、估计、假设”的区别，避免把用户估计或样机参数包装成实测标定。
    for name in ("base_edge_source", "tcp_offset_source"):
        if request[name] not in ("USER_MEASURED", "USER_ESTIMATE", "ASSUMED_DEMO"):
            raise ValueError(name + " must distinguish measured from assumed")
    if (
        not isinstance(start_pulses, (list, tuple))
        or len(start_pulses) != 6
        or any(type(p) is not int or not 500 <= p <= 2500 for p in start_pulses)
    ):
        raise ValueError("six recorded integer protocol positions required")
    # edge是底座中心到前边缘；outside是从该边缘向外量的距离，不能直接当转轴半径。
    edge = _number(request, "base_front_edge_offset_mm", 1, 500)
    outside = _number(request, "outside_edge_distance_mm", 0, 1000)
    radius = _number(request, "object_radius_mm", 1, 100)
    offset = _number(request, "tip_to_grasp_offset_mm", 0, 150)
    clearance = _number(request, "approach_clearance_mm", 10, 200)
    opening_margin = _number(request, "jaw_opening_margin_mm", 0, 50)
    duration = request["duration_ms"]
    if type(duration) is not int or not 1500 <= duration <= 9999:
        raise ValueError("duration must be integer1500..9999ms")
    reference = request["distance_reference"]
    if reference not in ("BASE_EDGE_TO_OBJECT_CENTER", "BASE_EDGE_TO_OBJECT_NEAR_EDGE"):
        raise ValueError("specify whether edge distance ends at object centre or near edge")
    # 若距离量到物体近侧边缘，还要加物体半径，才能得到底座中心到物体中心的距离。
    radial = edge + outside + (radius if reference.endswith("NEAR_EDGE") else 0)
    geometry = Geometry(**model["geometry_mm"])
    limits = validate_limits(model["nominal_angular_limits_deg"])
    # 六轴快照中仅前四轴进入FK；004末端旋转、005夹爪开合不参与当前平面几何。
    initial = from_nominal_pulses(start_pulses[:4])
    home_tip = forward(geometry, initial)
    height = request["measured_home_tip_height_mm"]
    # 单次竖直爪尖高度可对齐桌面Z偏移，不能同时标定关节零点、连杆长度和TCP。
    table_z = (
        0.0
        if height is None
        else home_tip.z_mm - _number(request, "measured_home_tip_height_mm", 50, 1500)
    )
    table_source = "ASSUMED_MODEL_Z_ZERO" if height is None else "SINGLE_USER_HOME_HEIGHT_REFERENCE"
    # 球状物的中心高度暂取radius；上方接近点再加clearance，所有值仍是候选假设。
    grasp_task = [0.0, radial, radius]
    approach_task = [0.0, radial, radius + clearance]
    grasp_center = task_to_model(grasp_task, initial.yaw_deg, table_z)
    approach_center = task_to_model(approach_task, initial.yaw_deg, table_z)
    candidates = []
    # Prefer a downward grasp, but allow a ball to be approached at an inclined
    # pitch when nominal wrist limits make a vertical hover unreachable.
    # 按5deg枚举向下到倾斜的抓取俯仰角，寻找同时有接近点和抓取点的逆解组合。
    for grasp_pitch in range(-90, -29, 5):
        grasp_tip = tip_for_grasp(grasp_center, initial.yaw_deg, grasp_pitch, offset)
        for grasp_q in inverse(geometry, grasp_tip, limits, base_hint_deg=initial.yaw_deg):
            grasp_p = to_nominal_pulses(grasp_q)
            for approach_pitch in range(grasp_pitch, -29, 5):
                approach_tip = tip_for_grasp(
                    approach_center, initial.yaw_deg, approach_pitch, offset
                )
                for approach_q in inverse(
                    geometry, approach_tip, limits, base_hint_deg=initial.yaw_deg
                ):
                    # Keep5deg margin for the candidate wrist endpoint, without
                    # changing the vendor algorithm's reported bounds.
                    if abs(approach_q.wrist_deg) > 85 or abs(grasp_q.wrist_deg) > 85:
                        continue
                    approach_p = to_nominal_pulses(approach_q)
                    # 两段都要通过中心线抽样检查；只保证两个端点可达仍不够。
                    try:
                        down1 = continuous_segment(
                            geometry, limits, start_pulses[:4], approach_p, table_z, duration
                        )
                        down2 = continuous_segment(
                            geometry, limits, approach_p, grasp_p, table_z, duration
                        )
                    except PickPlanningError:
                        continue
                    candidates.append((grasp_tip, grasp_p, approach_tip, approach_p, down1, down2))
                    break
                if candidates:
                    break
            if candidates:
                break
        if candidates:
            break
    if not candidates:
        raise PickPlanningError(
            "no front-only nominal approach+grasp path under current geometry, TCP and table reference"
        )
    # 当前采用搜索顺序中的首个候选，不是对时间/平滑度/误差进行全局最优规划。
    grasp_tip, grasp_p, approach_tip, approach_p, down1, down2 = candidates[0]
    lift = continuous_segment(geometry, limits, grasp_p, approach_p, table_z, duration)
    retract = continuous_segment(geometry, limits, approach_p, start_pulses[:4], table_z, duration)
    # 抓取后的抬升与回正也重新做几何预览，不能只审阅去程就假定回程有效。
    segments = []
    for name, value in zip(
        ("approach", "descend", "lift", "return_upright"), (down1, down2, lift, retract)
    ):
        # held_servo_ids标记本段未命令的轴，供后续实物执行层监测它们是否漂移。
        moving = {c["servo_id"] for c in value["commands"]}
        value = {**value, "name": name, "held_servo_ids": [i for i in range(6) if i not in moving]}
        segments.append(value)
    # 返回的是方案及待核实条件；夹口宽度、夹持力、实物角度/尺寸/TCP均单独标未验证。
    return {
        "schema_version": 1,
        "status": "NOMINAL_PICK_PROPOSAL",
        "motion_authorized": False,
        "initial_pulses_p": list(start_pulses),
        "request": dict(request),
        "task_frame": "base_centre_on_table_right_forward_up",
        "task_forward_yaw_in_model_deg": initial.yaw_deg,
        "distance_to_centre_mm": radial,
        "table_z_in_model_mm": table_z,
        "table_reference_source": table_source,
        "grasp_center_task_mm": grasp_task,
        "grasp_center_model_mm": grasp_center,
        "grasp_tip_model": asdict(grasp_tip),
        "grasp_pulses_p": list(grasp_p),
        "approach_center_task_mm": approach_task,
        "approach_tip_model": asdict(approach_tip),
        "approach_pulses_p": list(approach_p),
        "segments": segments,
        "phases": [
            "approach",
            "open_gripper",
            "descend",
            "WAIT_OBJECT_PLACED_AND_HANDS_CLEAR",
            "close_gripper",
            "WAIT_INDEPENDENT_GRASP_CONFIRMATION",
            "lift",
            "return_upright_holding",
        ],
        "required_opening_mm": 2 * radius + opening_margin,
        "gripper_width_to_p_verified": False,
        "grip_force_verified": False,
        "physical_joint_angles_verified": False,
        "physical_geometry_verified": False,
        "physical_tcp_verified": False,
        "sampled_centreline_only": True,
        "maximum_sampled_radius_mm": max(s["maximum_sampled_radius_mm"] for s in segments),
        "minimum_sampled_table_clearance_mm": min(
            s["minimum_sampled_table_clearance_mm"] for s in segments
        ),
        "execution_preconditions": [
            "actual base-edge/table references",
            "physical angle/TCP and full swept-path review",
            "jaw opening sufficient for object",
            "fresh stable six-axis state",
            "independent grasp evidence before lift",
        ],
    }
