"""有限工作阶段的交接误差盒检查；不改变容差，也不访问硬件。

这里只证明给定名义模型下的桌面余量。各轴独立运动过程、自碰撞、障碍、
真实尺寸及物理停止均不由交接误差盒证明。
"""

from copy import deepcopy
from itertools import product
import math

from arm_project.approach_planning import _sin_bounds, chain_points
from arm_project.motion_scene import SceneValidator


_MAX_WORK_PHASES = 41
_MAX_BOXES_PER_ENVELOPE = 4096
_MAX_CHECKS_TOTAL = 32768
_SCOPE = "WORK_PHASE_HANDOFF_TABLE_ONLY"


class HandoffEnvelopeError(ValueError):
    """交接误差未通过名义桌面约束；保留完整报告，供运行器在预算前落盘。"""

    def __init__(self, report):
        self.report = deepcopy(report)
        failure = report["failure"]
        witness = failure.get("counterexample") or {}
        clearance = witness.get("minimum_table_clearance_mm")
        if clearance is None:
            clearance = min(failure.get("initial_link_clearance_lower_bounds_mm") or [math.nan])
        super().__init__(
            "HANDOFF_TABLE_ENVELOPE_REJECTED: "
            f"phase={failure['phase']}; status={report['status']}; "
            f"nominal_clearance_mm={clearance}; "
            f"required_mm={report['required_table_clearance_mm']}; "
            "configured handoff tolerances require calibration or a revised target; no motion"
        )


def require_handoff_envelopes(spec, scene, *, check_budget=None):
    """交接检查必须在帧许可/独占运动预算前完成；不静默缩小操作者容差。"""
    report = check_handoff_envelopes(spec, scene, check_budget=check_budget)
    if not report["valid"]:
        raise HandoffEnvelopeError(report)
    return report


class _LimitReached(Exception):
    """内部有限检查预算耗尽；不能把未证明报告为碰撞或通过。"""


class _Checks:
    def __init__(self, callback):
        self.callback = callback
        self.boxes = 0
        self.inspections = 0

    def guard(self):
        if self.callback is not None:
            self.callback()

    def before(self, kind):
        self.guard()
        if self.boxes + self.inspections >= _MAX_CHECKS_TOTAL:
            raise _LimitReached("total finite envelope-check budget exhausted")
        if kind == "box":
            self.boxes += 1
        else:
            self.inspections += 1


def _table_bounds(scene, windows):
    """连续P区间→保守的三段胶囊最低桌面余量，单位毫米。

    正弦范围包含区间内全部极值，不能用角点代替。角之间的相关性被放宽，
    因此下界可能过低；这种情形只能细分或报告未证明，不能直接断言碰撞。
    """
    factor = 270.0 / 2000.0
    shoulder = tuple(90 + (p - 1500) * factor for p in windows[1])
    elbow = tuple((p - 1500) * factor for p in windows[2])
    wrist = tuple((p - 1500) * factor for p in windows[3])
    angles = (
        shoulder,
        (shoulder[0] - elbow[1], shoulder[1] - elbow[0]),
        (shoulder[0] - elbow[1] + wrist[0], shoulder[1] - elbow[0] + wrist[1]),
    )
    lengths = (
        scene.geometry.upper_arm_mm,
        scene.geometry.forearm_mm,
        scene.geometry.tool_mm,
    )
    # 向外误差余量只用于避免浮点边界误放，不降低场景要求，也不是实物精度声明。
    epsilon = 1e-9 + sum(lengths) * 2e-12
    endpoint_lows = [scene.geometry.base_height_mm - epsilon]
    for length, angle in zip(lengths, angles):
        endpoint_lows.append(endpoint_lows[-1] + length * _sin_bounds(*angle)[0] - epsilon)
    return tuple(
        min(a, b) - radius - scene.table_z_mm - epsilon
        for a, b, radius in zip(endpoint_lows, endpoint_lows[1:], scene.link_radii_mm)
    )


def _integer_candidates(windows):
    """角点及中点只用于寻找真实整数P反例，绝不作为全区间通过依据。"""
    integer_windows = tuple((math.ceil(low), math.floor(high)) for low, high in windows)
    if any(low > high for low, high in integer_windows):
        return ()
    yaw = round(sum(integer_windows[0]) / 2)
    corners = product(*(tuple(sorted(set(pair))) for pair in integer_windows[1:]))
    middle = tuple(round((low + high) / 2) for low, high in integer_windows)
    return tuple(dict.fromkeys([(yaw, *point) for point in corners] + [middle]))


def _inspect_counterexample(scene, windows, checks):
    for point in _integer_candidates(windows):
        checks.before("inspection")
        checked = scene.inspect(point)
        checks.guard()
        # inspect可能先返回其他场景错误，独立计算桌面值以保持本函数的TABLE范围。
        points = chain_points(scene.geometry, point)
        clearance = min(
            min(start[2], end[2]) - radius - scene.table_z_mm
            for start, end, radius in zip(points, points[1:], scene.link_radii_mm)
        )
        if clearance < scene.minimum_clearance_mm:
            return {
                "positions4_p": list(point),
                "minimum_table_clearance_mm": clearance,
                "scene_check": checked,
                "physical_collision_verified": False,
            }
    return None


def _check_box(scene, windows, checks):
    stack = [tuple(tuple(pair) for pair in windows)]
    evaluated = 0
    certified_lows = []
    initial_lows = None
    result = {"valid": False, "status": "NOT_PROVEN", "counterexample": None}
    try:
        while stack:
            checks.guard()
            if evaluated >= _MAX_BOXES_PER_ENVELOPE:
                raise _LimitReached("per-envelope interval-box budget exhausted")
            checks.before("box")
            cell = stack.pop()
            evaluated += 1
            lows = _table_bounds(scene, cell)
            checks.guard()
            if initial_lows is None:
                initial_lows = lows
            if min(lows) >= scene.minimum_clearance_mm:
                certified_lows.append(min(lows))
                continue
            counterexample = _inspect_counterexample(scene, cell, checks)
            if counterexample is not None:
                result.update(
                    status="COUNTEREXAMPLE",
                    counterexample=counterexample,
                    reason="an allowed integer-P handoff combination violates the nominal table margin",
                )
                break
            # Z与底座yaw无关，仅细分001..003；重叠中点覆盖完整连续区间。
            axis = max(range(1, 4), key=lambda i: cell[i][1] - cell[i][0])
            low, high = cell[axis]
            middle = (low + high) / 2
            if high - low <= 1e-6 or middle in (low, high):
                result["reason"] = "interval lower bound remains unresolved at numerical limit"
                break
            left, right = list(cell), list(cell)
            left[axis], right[axis] = (low, middle), (middle, high)
            stack.extend((tuple(right), tuple(left)))
        else:
            result.update(
                valid=True,
                status="CERTIFIED",
                certified_table_clearance_lower_bound_mm=min(certified_lows),
            )
    except _LimitReached as error:
        result["reason"] = str(error)
    result.update(
        interval_boxes=evaluated,
        initial_link_clearance_lower_bounds_mm=list(initial_lows or ()),
    )
    return result


def check_handoff_envelopes(spec, scene, *, check_budget=None):
    """检查非入口阶段的允许交接区间；调用方的时间预算异常原样传播。

    spec必须是最终TransferTaskSpec。入口仍传播目标与保持区间，但不在此
    重复认证软余量恢复；其资格及整段路线由已有恢复检查器独立处理。
    """
    # 放在函数内部避免策略/控制器接入本模块时形成循环导入。
    from arm_project.transfer_task_motion import TransferTaskSpec

    if not isinstance(spec, TransferTaskSpec) or not isinstance(scene, SceneValidator):
        raise ValueError("validated TransferTaskSpec and SceneValidator required")
    if check_budget is not None and not callable(check_budget):
        raise ValueError("check_budget must be callable or None")
    checks = _Checks(check_budget)
    checks.guard()
    if TransferTaskSpec.from_dict(spec.to_dict()) != spec:
        raise ValueError("handoff spec differs from its complete validated representation")
    if spec.geometry != scene.geometry or spec.table_z_mm != scene.table_z_mm:
        raise ValueError("handoff spec and scene must use the same geometry and table frame")
    work_count = sum(
        phase.name != "entry" and not phase.name.startswith("entry_") for phase in spec.phases
    )
    if work_count > _MAX_WORK_PHASES:
        raise ValueError("at most 41 non-entry work phases are supported")
    windows = [tuple(pair) for pair in spec.start_intervals_p]
    goals = dict(spec.initial_setpoints_p)
    cache = {}
    rows = []
    cache_hits = 0
    failure = None
    for index, phase in enumerate(spec.phases):
        checks.guard()
        held = dict(phase.hold_tracking_p)
        for sid, tolerance in phase.continue_tracking_p:
            target = goals[sid]
            low, high = spec.observation_bounds_p[sid]
            windows[sid] = (max(low, target - tolerance), min(high, target + tolerance))
        for sid, target, tolerance in phase.targets:
            tolerance = min(tolerance, held.get(sid, tolerance))
            low, high = spec.observation_bounds_p[sid]
            windows[sid] = (max(low, target - tolerance), min(high, target + tolerance))
            goals[sid] = target
        row = {
            "phase_index": index,
            "phase": phase.name,
            "pulse_windows6_p": [list(pair) for pair in windows],
        }
        if phase.name == "entry" or phase.name.startswith("entry_"):
            rows.append({**row, "status": "SKIPPED_ENTRY_RECOVERY_SCOPE", "valid": None})
            continue
        key = tuple(windows[:4])
        if key in cache:
            checked = deepcopy(cache[key])
            cache_hits += 1
            row["cached"] = True
        else:
            checked = _check_box(scene, key, checks)
            cache[key] = deepcopy(checked)
            row["cached"] = False
        rows.append({**row, **checked})
        if not checked["valid"]:
            failure = deepcopy(rows[-1])
            break
    checks.guard()
    return {
        "valid": failure is None,
        "status": "CERTIFIED" if failure is None else failure["status"],
        "scope": _SCOPE,
        "required_table_clearance_mm": scene.minimum_clearance_mm,
        "failure": failure,
        "phases": rows,
        "check_counts": {
            "phase_count": len(spec.phases),
            "work_phase_count": work_count,
            "unique_envelopes": len(cache),
            "cache_hits": cache_hits,
            "interval_boxes": checks.boxes,
            "scene_inspections": checks.inspections,
            "checks_total": checks.boxes + checks.inspections,
        },
        "asynchronous_motion_envelope_verified": False,
        "other_scene_constraints_verified": False,
        "physical_geometry_verified": False,
        "physical_collision_verified": False,
    }
