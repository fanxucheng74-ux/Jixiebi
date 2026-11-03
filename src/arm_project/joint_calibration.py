"""纯软件逐轴标定统计；不访问设备，不产生运动许可或机械限位。

残差行：{servo_id, source:'HARDWARE', start_p, target_p, final_p, duration_ms}。
外部测量行：{servo_id, source:'HARDWARE', external_measurement:True,
             sample_id:唯一非空字符串, p:整数, angle_deg:数值}
夹爪测量将 angle_deg 换成 width_mm。

source/external_measurement 是调用者对数据来源的声明，本模块不能自行证明
实物或量具存在。sample_id 和 P 均须唯一，防止重复同一点冒充多点标定。
P 的 500..2500 检查仅是协议数值边界，绝不是可运动范围。
"""

from bisect import bisect_left
import math
from statistics import median


def _record(value):
    if not isinstance(value, dict):
        raise ValueError("each sample/model must be a dictionary")
    return value


def _rows(samples, minimum=1):
    if not isinstance(samples, (list, tuple)) or len(samples) < minimum:
        raise ValueError("at least %d sample rows are required" % minimum)
    return [_record(row) for row in samples]


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("%s must be an integer in [%d, %d]" % (name, low, high))
    return value


def _p(value):
    return _integer(value, "p", 500, 2500)


def _number(value, name):
    if type(value) not in (int, float):
        raise ValueError(name + " must be a finite real number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ValueError(name + " is not a finite float") from None
    if not math.isfinite(result):
        raise ValueError(name + " must be finite")
    return result


def summarize_directional_residuals(samples):
    """按 servo_id 和 duration_ms 分组，避免把不同轴/时长的数据混合。

    方向由 target-start 定义；残差是 target-final，不是实际角度误差。
    返回 candidate_only 数据统计，不输出补偿推荐值或自动应用配置。
    """
    groups = {}
    for row in _rows(samples):
        if row.get("source") != "HARDWARE":
            raise ValueError("residual summary requires declared HARDWARE samples")
        axis = _integer(row.get("servo_id"), "servo_id", 0, 5)
        duration = _integer(row.get("duration_ms"), "duration_ms", 20, 9999)
        start, target, final = (_p(row.get(key)) for key in ("start_p", "target_p", "final_p"))
        direction = (
            "positive" if target > start else "negative" if target < start else "zero_target_delta"
        )
        group = groups.setdefault((axis, duration), {})
        group.setdefault(direction, []).append((target - final, final - start))
    result = []
    for (axis, duration), directions in sorted(groups.items()):
        stats = {}
        for direction, values in directions.items():
            errors = [error for error, _ in values]
            deltas = [delta for _, delta in values]
            stats[direction] = {
                "sample_count": len(values),
                "error_median_p": median(errors),
                "error_mean_p": sum(errors) / len(errors),
                "error_min_p": min(errors),
                "error_max_p": max(errors),
                "error_span_p": max(errors) - min(errors),
                "observed_delta_median_p": median(deltas),
            }
        result.append({"servo_id": axis, "duration_ms": duration, "directions": stats})
    return {
        "candidate_only": True,
        "unit": "protocol_P",
        "source": "HARDWARE",
        "physical_accuracy_verified": False,
        "groups": result,
    }


def _external_samples(samples, value_key):
    parsed = []
    ids, positions, axes = set(), set(), set()
    for row in _rows(samples, minimum=3):
        if row.get("source") != "HARDWARE" or row.get("external_measurement") is not True:
            raise ValueError("external HARDWARE measurements are required")
        axis = _integer(row.get("servo_id"), "servo_id", 0, 5)
        sample_id = row.get("sample_id")
        if type(sample_id) is not str or not sample_id.strip() or sample_id in ids:
            raise ValueError("sample_id must be nonempty and unique")
        p = _p(row.get("p"))
        if p in positions:
            raise ValueError("distinct P values are required for independent calibration points")
        value = _number(row.get(value_key), value_key)
        if value_key == "width_mm" and value < 0:
            raise ValueError("measured gripper width must be nonnegative")
        ids.add(sample_id)
        positions.add(p)
        axes.add(axis)
        parsed.append((p, value, sample_id))
    if len(axes) != 1:
        raise ValueError("a calibration must not combine different servo IDs")
    if len({value for _, value, _ in parsed}) == 1:
        raise ValueError("constant measured values do not define a calibration")
    return next(iter(axes)), sorted(parsed)


def fit_angle_mapping(samples):
    """拟合 angle_deg=a*p+b，返回区间、残差及候选模型；不自动判断精度合格。"""
    axis, rows = _external_samples(samples, "angle_deg")
    count = len(rows)
    try:
        mean_p = sum(p for p, _, _ in rows) / count
        mean_y = math.fsum(y / count for _, y, _ in rows)
        denominator = math.fsum((p - mean_p) ** 2 for p, _, _ in rows)
        a = math.fsum((p - mean_p) * (y - mean_y) for p, y, _ in rows) / denominator
        b = mean_y - a * mean_p
        residuals = [y - (a * p + b) for p, y, _ in rows]
        rms = math.sqrt(math.fsum(r * r / count for r in residuals))
        for name, number in (("slope", a), ("intercept", b), ("RMS", rms)):
            _number(number, name)
    except (OverflowError, ZeroDivisionError):
        raise ValueError("measurements cannot form a numerically finite linear model") from None
    if a == 0:
        raise ValueError("zero fitted slope cannot define an angle mapping")
    return {
        "kind": "linear_angle_mapping",
        "candidate_only": True,
        "servo_id": axis,
        "source": "HARDWARE_EXTERNAL_MEASUREMENTS",
        "sample_count": count,
        "sample_ids": [sample_id for _, _, sample_id in rows],
        "p_min": rows[0][0],
        "p_max": rows[-1][0],
        "a_deg_per_p": a,
        "b_deg": b,
        "rms_error_deg": rms,
        "max_abs_error_deg": max(abs(r) for r in residuals),
        "measurement_residuals_deg": residuals,
        "defines_safe_motion_limits": False,
    }


def angle_from_p(model, p):
    """仅在测量P区间内换算角度；不校验碰撞，也不表示允许运动。"""
    model = _record(model)
    if model.get("kind") != "linear_angle_mapping" or model.get("candidate_only") is not True:
        raise ValueError("not a supported candidate angle model")
    _integer(model.get("servo_id"), "servo_id", 0, 5)
    low, high = _p(model.get("p_min")), _p(model.get("p_max"))
    p = _p(p)
    if not low < high or not low <= p <= high:
        raise ValueError("angle conversion does not extrapolate beyond measured P")
    a = _number(model.get("a_deg_per_p"), "a_deg_per_p")
    b = _number(model.get("b_deg"), "b_deg")
    if a == 0:
        raise ValueError("zero fitted slope")
    return _number(a * p + b, "converted angle")


def _monotonic(values):
    changes = [b - a for a, b in zip(values, values[1:])]
    if not (all(delta > 0 for delta in changes) or all(delta < 0 for delta in changes)):
        raise ValueError("width calibration must be strictly monotonic")
    return "increasing" if changes[0] > 0 else "decreasing"


def build_gripper_table(samples):
    """保留外测夹爪宽度的单调插值表；宽度不是夹力，也不是物体夹持证明。"""
    axis, rows = _external_samples(samples, "width_mm")
    points = [{"p": p, "width_mm": width} for p, width, _ in rows]
    direction = _monotonic([point["width_mm"] for point in points])
    return {
        "kind": "gripper_width_table",
        "candidate_only": True,
        "servo_id": axis,
        "source": "HARDWARE_EXTERNAL_MEASUREMENTS",
        "sample_count": len(rows),
        "sample_ids": [sample_id for _, _, sample_id in rows],
        "points": points,
        "width_direction": direction,
        "defines_safe_motion_limits": False,
        "force_or_grasp_measurement": False,
    }


def gripper_width_from_p(table, p):
    """分段线性内插，只接受已测量范围；不会从位置推出是否抓到物。"""
    table = _record(table)
    if table.get("kind") != "gripper_width_table" or table.get("candidate_only") is not True:
        raise ValueError("not a supported candidate gripper table")
    _integer(table.get("servo_id"), "servo_id", 0, 5)
    rows = _rows(table.get("points"), minimum=3)
    positions = [_p(row.get("p")) for row in rows]
    widths = [_number(row.get("width_mm"), "width_mm") for row in rows]
    if any(b <= a for a, b in zip(positions, positions[1:])) or any(w < 0 for w in widths):
        raise ValueError("invalid gripper calibration points")
    _monotonic(widths)
    p = _p(p)
    if not positions[0] <= p <= positions[-1]:
        raise ValueError("width conversion does not extrapolate beyond measured P")
    index = bisect_left(positions, p)
    if positions[index] == p:
        return widths[index]
    ratio = (p - positions[index - 1]) / (positions[index] - positions[index - 1])
    return _number(
        widths[index - 1] + ratio * (widths[index] - widths[index - 1]), "interpolated width"
    )
