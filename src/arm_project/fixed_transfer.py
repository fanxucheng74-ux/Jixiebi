"""固定左右点搬运的离线名义规划；不连接硬件，也不产生运动授权。"""

from dataclasses import asdict
import math

from arm_project.approach_planning import chain_points
from arm_project.kinematics import (
    Geometry,
    Pose,
    finite,
    forward,
    from_nominal_pulses,
    inverse,
    to_nominal_pulses,
)


def _number(value, label, low, high):
    number = finite(value, label)
    if not low <= number <= high:
        raise ValueError(label + " outside bounded planning domain")
    return number


def _coordinate(distance, reference, yaw_deg, height):
    """厂家坐标 +X右/+Y前/+Z上；距离参考必须由调用者显式选择。"""
    angle = math.radians(yaw_deg)
    if reference == "RADIAL_FROM_BASE":
        radial = distance
    elif reference == "FORWARD_Y":
        if math.cos(angle) <= 0.01:
            raise ValueError("forward-Y distance is undefined for this yaw")
        radial = distance / math.cos(angle)
    else:
        raise ValueError("choose RADIAL_FROM_BASE or FORWARD_Y")
    return (radial * math.sin(angle), radial * math.cos(angle), height)


def _tip(center, yaw_deg, pitch_deg, offset):
    # 模型末端是闭合爪尖；offset是假设的抓取中心至爪尖偏移，不是实测标定。
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    direction = (math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), math.sin(pitch))
    return Pose(*(v + offset * d for v, d in zip(center, direction)), pitch_deg)


def _smoothstep(u):
    # 五次多项式仅用于离线预测：两端速度及加速度为零，舵机是否按此运动尚未实现。
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def _preview(geometry, start, end, table_z, duration_ms):
    # 四轴共享进度是数学假设；多舵机实际接收和执行不同步，不能把结果当碰撞证明。
    samples = []
    for index in range(33):
        u = index / 32
        s = _smoothstep(u)
        pulses = tuple(round(a + (b - a) * s) for a, b in zip(start, end))
        points = chain_points(geometry, pulses)
        samples.append(
            {
                "time_ms": round(duration_ms * u, 3),
                "pulses_p": list(pulses),
                "minimum_centerline_height_above_table_mm": min(p[2] - table_z for p in points),
            }
        )
    return {
        "profile": "QUINTIC_PROGRESS_MODEL_ONLY",
        "samples": samples,
        "minimum_centerline_height_mm": min(
            row["minimum_centerline_height_above_table_mm"] for row in samples
        ),
        "physical_path_verified": False,
        "hardware_interpolation_implemented": False,
    }


def plan_fixed_transfer(model, request):
    """生成最多五次重复搬运的名义目标与时间方案；绝不授权硬件运行。

    每轮从左侧高位出发，取物、旋转到右侧放置、空爪回左侧高位。
    时间为各阶段的理想声明值之和，不含通信、反馈或物料补充时间。
    """
    keys = {
        "distance_reference",
        "distance_mm",
        "left_yaw_deg",
        "right_yaw_deg",
        "block_height_mm",
        "tip_to_grasp_mm",
        "table_z_mm",
        "approach_clearance_mm",
        "cycle_count",
        "stage_times_ms",
    }
    if (
        not isinstance(request, dict)
        or keys - set(request)
        or set(request) - keys - {"block_footprint_xy_mm"}
    ):
        raise ValueError("fixed transfer requires exact explicit planning fields")
    if not isinstance(model, dict) or model.get("source") != "VENDOR_NOMINAL_UNMEASURED":
        raise ValueError("explicit unmeasured vendor nominal model required")
    geom = Geometry(**model["geometry_mm"])
    limits = model["nominal_angular_limits_deg"]
    distance = _number(request["distance_mm"], "distance_mm", 1, 1000)
    left = _number(request["left_yaw_deg"], "left_yaw_deg", -89, 89)
    right = _number(request["right_yaw_deg"], "right_yaw_deg", -89, 89)
    if left >= right:
        raise ValueError("left yaw must be less than right yaw")
    height = _number(request["block_height_mm"], "block_height_mm", 1, 200)
    footprint = request.get("block_footprint_xy_mm")
    if footprint is not None:
        if not isinstance(footprint, (list, tuple)) or len(footprint) != 2:
            raise ValueError("block footprint requires explicit width and length")
        footprint = [_number(v, "block_footprint_xy_mm", 1, 200) for v in footprint]
    offset = _number(request["tip_to_grasp_mm"], "tip_to_grasp_mm", 0, 150)
    table = _number(request["table_z_mm"], "table_z_mm", -1000, 1000)
    clearance = _number(request["approach_clearance_mm"], "approach_clearance_mm", 10, 300)
    count = request["cycle_count"]
    if type(count) is not int or not 1 <= count <= 5:
        raise ValueError("cycle_count must be an integer in 1..5")
    names = (
        "descend_pick",
        "close",
        "lift_pick",
        "rotate_to_place",
        "descend_place",
        "open",
        "lift_place",
        "rotate_to_pick",
    )
    times = request["stage_times_ms"]
    if not isinstance(times, dict) or set(times) != set(names):
        raise ValueError("all eight exact stage times are required")
    if any(type(times[n]) is not int or not 100 <= times[n] <= 9999 for n in names):
        raise ValueError("stage times must be bounded integer milliseconds")
    # 物块中心为桌面以上半高；不得将25mm边长自动视为抓取TCP的标定偏移。
    z_pick, z_hover = table + height / 2, table + height / 2 + clearance
    xyz = {
        "pick": _coordinate(distance, request["distance_reference"], left, z_pick),
        "pick_hover": _coordinate(distance, request["distance_reference"], left, z_hover),
        "place": _coordinate(distance, request["distance_reference"], right, z_pick),
        "place_hover": _coordinate(distance, request["distance_reference"], right, z_hover),
    }
    selected = None
    # 在有限俯仰范围中找共同可达解；仅数学端点匹配，不能代表实际抓取姿态。
    for pitch in range(-90, -24, 5):
        options = {}
        for name, yaw in (
            ("pick", left),
            ("pick_hover", left),
            ("place", right),
            ("place_hover", right),
        ):
            pose = _tip(xyz[name], yaw, pitch, offset)
            options[name] = []
            for joints in inverse(geom, pose, limits, base_hint_deg=yaw):
                if abs(joints.yaw_deg - yaw) > 0.01:
                    continue
                try:
                    pulses = to_nominal_pulses(joints)
                except ValueError:
                    continue
                options[name].append((joints, pulses))
        if any(not options[name] for name in options):
            continue
        # 首先挑选同肘分支、接近姿态变化最小的左侧逆解；不代表全局最优或无碰撞。
        pairs = [
            (a, b)
            for a in options["pick"]
            for b in options["pick_hover"]
            if a[0].elbow_flex_deg * b[0].elbow_flex_deg >= 0
        ]
        pairs.sort(key=lambda p: sum(abs(x - y) for x, y in zip(p[0][1], p[1][1])))
        for pick, hover in pairs:
            place = min(
                options["place"],
                key=lambda q: sum(abs(x - y) for x, y in zip(q[1][1:], pick[1][1:])),
            )
            place_hover = min(
                options["place_hover"],
                key=lambda q: sum(abs(x - y) for x, y in zip(q[1][1:], hover[1][1:])),
            )
            targets = {
                "pick": pick[1],
                "pick_hover": hover[1],
                "place": place[1],
                "place_hover": place_hover[1],
            }
            if (
                targets["pick"][1:] != targets["place"][1:]
                or targets["pick_hover"][1:] != targets["place_hover"][1:]
            ):
                continue
            paths = (
                ("descend_pick", "pick_hover", "pick"),
                ("lift_pick", "pick", "pick_hover"),
                ("rotate_to_place", "pick_hover", "place_hover"),
                ("descend_place", "place_hover", "place"),
                ("lift_place", "place", "place_hover"),
                ("rotate_to_pick", "place_hover", "pick_hover"),
            )
            previews = {
                n: _preview(geom, targets[a], targets[b], table, times[n]) for n, a, b in paths
            }
            if min(x["minimum_centerline_height_mm"] for x in previews.values()) < 5:
                continue
            selected = pitch, targets, previews
            break
        if selected:
            break
    if selected is None:
        raise ValueError("no nominal common-pitch endpoints with sampled centerline clearance")
    pitch, targets, previews = selected
    stages = []
    for name in names:
        row = {"name": name, "duration_ms": times[name]}
        if name in previews:
            row["preview"] = previews[name]
        else:
            row["gripper_action"] = "close" if name == "close" else "open"
            row["gripper_position_and_force_calibrated"] = False
        stages.append(row)
    total = sum(times.values())
    return {
        "status": "NOMINAL_PROPOSAL_ONLY",
        "motion_authorized": False,
        "model_geometry_mm": dict(model["geometry_mm"]),
        "model_parameter_variant": model.get("parameter_variant"),
        "coordinate_frame": "+X right, +Y forward, +Z up; millimetres",
        "input_assumptions": dict(request),
        "block_footprint_xy_mm": footprint,
        "points_mm": {name: list(point) for name, point in xyz.items()},
        "nominal_pitch_deg": pitch,
        "nominal_endpoints_p": {name: list(p) for name, p in targets.items()},
        "nominal_endpoints_model": {
            name: asdict(forward(geom, from_nominal_pulses(p))) for name, p in targets.items()
        },
        "stages": stages,
        "cycle_count": count,
        "declared_ms_per_transfer": total,
        "within_10s_declared_only": total <= 10000,
        "execution_overhead_included": False,
        "initial_move_to_pick_hover_included": False,
        "initial_state_assumption": "cycle timing starts at left hover with gripper open; not verified",
        "object_supply": "each round requires a new object at the same pickup point; not verified",
        "destination_management": "same fixed drop point must be cleared between rounds, or use separately validated drop points; not verified",
        "supply_and_clearing_time_included": False,
        "dynamic_vision_selection_implemented": False,
        "current_timed_cycle_executor_compatible": False,
        "executor_compatibility_reason": "existing timed_cycle excludes base ID 0 and requires arm T>=1500ms and gripper T>=1000ms",
        "hardware_interpolation_implemented": False,
        "physical_geometry_verified": False,
        "physical_angles_verified": False,
        "physical_grasp_verified": False,
        "physical_five_cycles_verified": False,
        "caveat": "sampled centerlines only; no link width, block bounding-box/neighbour obstacle collision, actual timing or synchronization proof",
    }
