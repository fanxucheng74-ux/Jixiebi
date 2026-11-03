"""Nominal Jibot four-axis geometry, independent of any hardware transport.

Vendor frame: +Y forward, +X right, +Z up; yaw=atan2(X,Y). Shoulder is
elevation from horizontal, positive elbow flexion subtracts elevation, and
pitch=shoulder-elbow+wrist. The endpoint is the CLOSED GRIPPER TIP, not a
measured grasp center. Nominal pulses are not calibrated physical angles.
"""

from dataclasses import dataclass
import math


# 本模块只做几何计算：输入/输出长度为 mm，公开角度为 deg，三角函数内部使用 rad。
# 四轴顺序对应 000底座、001肩、002肘、003腕；004/005不参与这里的四轴几何模型。
def finite(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError(name + " must be a finite real number")
    return float(value)


def wrap_degrees(angle):
    # 将等价方向归一到 [-180, 180)，便于比较俯仰角和去除重复逆解。
    return (angle + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Geometry:
    # 基座高度指模型原点到肩轴的垂直距离；三段长度依次指上臂、前臂、腕到闭合爪尖。
    # 字段是模型参数，数值合法不表示这些尺寸已经在实物上测准。
    base_height_mm: float
    upper_arm_mm: float
    forearm_mm: float
    tool_mm: float

    def __post_init__(self):
        for name, value in vars(self).items():
            number = finite(value, name)
            if number < 0 or (name != "base_height_mm" and number == 0):
                raise ValueError("link lengths must be positive; base height may be zero")
            if name != "base_height_mm" and number < 0.001:
                raise ValueError("link length below the numerical desktop model domain")
            if number > 10000:
                raise ValueError("length outside this desktop model's numerical domain")
            object.__setattr__(self, name, number)


@dataclass(frozen=True)
class Joints:
    # yaw是底座水平朝向，shoulder是上臂相对水平面的仰角。
    # elbow_flex采用“从肩角中减去”的弯曲约定；wrist再叠加到末端俯仰角。
    yaw_deg: float
    shoulder_deg: float
    elbow_flex_deg: float
    wrist_deg: float

    def __post_init__(self):
        for name, value in vars(self).items():
            number = finite(value, name)
            if abs(number) > 360:
                raise ValueError("joint angle outside the nominal single-turn domain")
            object.__setattr__(self, name, number)


@dataclass(frozen=True)
class Pose:
    # XYZ是闭合爪尖在模型坐标系中的位置，不是图像像素或实测抓取中心。
    # 厂家约定 +X向右、+Y向前、+Z向上；pitch表示末端相对水平面的俯仰。
    x_mm: float
    y_mm: float
    z_mm: float
    pitch_deg: float

    def __post_init__(self):
        for name, value in vars(self).items():
            number = finite(value, name)
            if name == "pitch_deg" and not -180 <= number <= 180:
                raise ValueError("pitch_deg must be in [-180,180]")
            if name != "pitch_deg" and abs(number) > 100000:
                raise ValueError("position outside desktop model's numerical domain")
            object.__setattr__(self, name, number)


def forward(geometry, joints):
    if not isinstance(geometry, Geometry) or not isinstance(joints, Joints):
        raise ValueError("forward requires Geometry and Joints")
    # 数据类字段的声明顺序就是下面四个角度的解包顺序；进入sin/cos前统一转弧度。
    yaw, shoulder, elbow, wrist = map(math.radians, vars(joints).values())
    forearm_pitch = shoulder - elbow
    tool_pitch = forearm_pitch + wrist
    # 先在机械臂竖直剖面中求水平伸出距离radial和高度z，再按底座yaw转回XYZ。
    radial = (
        geometry.upper_arm_mm * math.cos(shoulder)
        + geometry.forearm_mm * math.cos(forearm_pitch)
        + geometry.tool_mm * math.cos(tool_pitch)
    )
    z = (
        geometry.base_height_mm
        + geometry.upper_arm_mm * math.sin(shoulder)
        + geometry.forearm_mm * math.sin(forearm_pitch)
        + geometry.tool_mm * math.sin(tool_pitch)
    )
    # yaw=0朝向+Y，所以X用sin、Y用cos，不能照搬常见“+X为零度”的公式。
    return Pose(
        radial * math.sin(yaw), radial * math.cos(yaw), z, wrap_degrees(math.degrees(tool_pitch))
    )


def validate_limits(limits):
    # limits必须明确给出四个关节各自的名义角度区间；缺字段时不能猜默认机械限位。
    names = tuple(Joints.__dataclass_fields__)
    if not isinstance(limits, dict) or set(limits) != set(names):
        raise ValueError("four explicit nominal angular limits are required")
    parsed = {}
    for name in names:
        pair = limits[name]
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError("limit must be a pair")
        low, high = [finite(v, name) for v in pair]
        if not -180 <= low < high <= 180:
            raise ValueError("invalid nominal angular limit")
        parsed[name] = (low, high)
    return parsed


def inverse(geometry, target, limits, *, base_hint_deg=0.0):
    """Return all bounded mathematical solutions; unreachable means an empty tuple.

    No mutable last-solution state and no pulse/ASCII output. Both elbow branches
    and signed radial directions are considered. At X=Y=0 yaw is underdetermined;
    the explicit hint selects a representative. No division by radial distance.
    Angular limits describe a nominal model, not collision-free mechanical travel.
    """
    if not isinstance(geometry, Geometry) or not isinstance(target, Pose):
        raise ValueError("inverse requires Geometry and Pose")
    bounds = validate_limits(limits)
    hint = finite(base_hint_deg, "base_hint_deg")
    if not -180 <= hint <= 180:
        raise ValueError("base_hint_deg outside [-180,180]")
    radius = math.hypot(target.x_mm, target.y_mm)
    if radius <= 1e-9:
        # 目标在竖直轴线上时，XY不能决定底座角；用调用方的hint选择一个代表方向。
        radial_options = [(hint, 0.0)]
    else:
        # 同一点可以对应“朝向目标伸出”或“底座反向、径向取负”两种数学表示。
        yaw = math.degrees(math.atan2(target.x_mm, target.y_mm))
        radial_options = [(yaw, radius), (wrap_degrees(yaw + 180), -radius)]
    pitch = math.radians(target.pitch_deg)
    upper, forearm = geometry.upper_arm_mm, geometry.forearm_mm
    # 先扣掉基座高度和工具段投影，把末端目标还原为上臂/前臂二连杆的腕部目标。
    z = target.z_mm - geometry.base_height_mm - geometry.tool_mm * math.sin(pitch)
    found = {}
    for yaw, radial in radial_options:
        if not bounds["yaw_deg"][0] - 1e-8 <= yaw <= bounds["yaw_deg"][1] + 1e-8:
            continue
        r = radial - geometry.tool_mm * math.cos(pitch)
        # 余弦定理判断二连杆可达性；超过[-1,1]的真实距离无解，不能硬夹成可达。
        cosine = (r * r + z * z - upper * upper - forearm * forearm) / (2 * upper * forearm)
        if abs(cosine) > 1 + 1e-10:
            continue
        # 仅吸收浮点舍入误差；正/负肘角分别枚举两个肘部数学分支。
        elbow_magnitude = math.acos(max(-1.0, min(1.0, cosine)))
        for elbow in (elbow_magnitude, -elbow_magnitude):
            shoulder = math.atan2(z, r) + math.atan2(
                forearm * math.sin(elbow), upper + forearm * math.cos(elbow)
            )
            for turn in (-360, 0, 360):
                # 枚举等价肩角后，按传入的名义角度区间筛选；不自动选择“上一次解”。
                shoulder_deg = math.degrees(shoulder) + turn
                elbow_deg = math.degrees(elbow)
                wrist_deg = wrap_degrees(target.pitch_deg - shoulder_deg + elbow_deg)
                values = (yaw, shoulder_deg, elbow_deg, wrist_deg)
                if not all(
                    bounds[n][0] - 1e-8 <= v <= bounds[n][1] + 1e-8
                    for n, v in zip(Joints.__dataclass_fields__, values)
                ):
                    continue
                # Snap numerical boundary excursions, never widen a supplied limit.
                values = tuple(
                    min(bounds[n][1], max(bounds[n][0], v))
                    for n, v in zip(Joints.__dataclass_fields__, values)
                )
                # 逆解重新送入正解复核XYZ和pitch，避免只满足公式中间量却产生错误端点。
                solution = Joints(*values)
                check = forward(geometry, solution)
                error = math.dist(
                    (target.x_mm, target.y_mm, target.z_mm), (check.x_mm, check.y_mm, check.z_mm)
                )
                if error > 1e-5 or abs(wrap_degrees(check.pitch_deg - target.pitch_deg)) > 1e-6:
                    continue
                # 以角度值去掉数值重复解；这里不比较碰撞、舵机负载或真实运动代价。
                found[tuple(round(v, 8) for v in values)] = solution
    return tuple(found.values())


def from_nominal_pulses(pulses):
    # pulses只接受000..003四轴的协议P值；500..2500仅是协议数值域。
    if (
        not isinstance(pulses, (list, tuple))
        or len(pulses) != 4
        or any(type(p) is not int or not 500 <= p <= 2500 for p in pulses)
    ):
        raise ValueError("four protocol integers in500..2500 are required")
    # 厂家名义比例为2000P对应270deg。1500P是参考点，肩轴额外加90deg表示竖直。
    # 底座方向带负号；这个比例和零点尚不能替代实测角度标定。
    factor = 270.0 / 2000.0
    return Joints(
        -(pulses[0] - 1500) * factor,
        90 + (pulses[1] - 1500) * factor,
        (pulses[2] - 1500) * factor,
        (pulses[3] - 1500) * factor,
    )


def to_nominal_pulses(joints):
    if not isinstance(joints, Joints):
        raise ValueError("Joints required")
    # 与from_nominal_pulses反向换算，单位从deg变为协议P，不是微秒或毫米。
    factor = 2000.0 / 270.0
    # Nearest protocol integer, rather than vendor's truncation. Not compensation.
    values = (
        1500 - joints.yaw_deg * factor,
        1500 + (joints.shoulder_deg - 90) * factor,
        1500 + joints.elbow_flex_deg * factor,
        1500 + joints.wrist_deg * factor,
    )
    # 取最近的协议整数会带来量化误差，因此下游仍需用真实回读核对实际位置。
    pulses = tuple(round(p) for p in values)
    if any(not 500 <= p <= 2500 for p in pulses):
        raise ValueError("nominal solution is outside protocol limits")
    return pulses
