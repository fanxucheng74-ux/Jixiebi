"""三连杆胶囊与显式场景的名义静态检查；不提供实物无碰撞证明或运动授权。"""

from dataclasses import asdict
import math

from arm_project.approach_planning import chain_points
from arm_project.kinematics import Geometry, finite, from_nominal_pulses, validate_limits


_SCENE_FIELDS = {
    "schema",
    "axis_bounds_p",
    "table_z_mm",
    "minimum_clearance_mm",
    "maximum_radial_mm",
    "link_radii_mm",
    "self_collision_margin_mm",
    "obstacles_aabb_mm",
    "physical_geometry_verified",
}
_NOTES = (
    "Nominal geometry, P-to-angle mapping, link radii and scene coordinates are assumptions; "
    "millimetre calculations do not establish physical calibration or safe mechanical travel.",
    "Three capsules model shoulder-elbow, elbow-wrist and wrist-closed-tip only. "
    "Base body, joint housings, roll, open gripper, carried objects and cables are not modeled.",
    "Adjacent links share endpoints and are excluded from self-collision checks; "
    "only nonadjacent links 0 and 2 are checked.",
    "AABB expansion is conservative and can reject a capsule near a box corner. "
    "A valid static pose is not a continuous path, actuator synchronization or physical-stop proof.",
)


def _number(value, name, low, high):
    number = finite(value, name)
    if not low <= number <= high:
        raise ValueError(name + " outside the nominal scene numerical domain")
    return number


def _vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(name + " requires three finite millimetre coordinates")
    return tuple(_number(v, name, -100000, 100000) for v in value)


def _difference(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _point_segment_distance(point, start, end):
    direction = _difference(end, start)
    length2 = _dot(direction, direction)
    if length2 == 0:
        return math.dist(point, start)
    progress = min(1.0, max(0.0, _dot(_difference(point, start), direction) / length2))
    closest = tuple(a + progress * d for a, d in zip(start, direction))
    return math.dist(point, closest)


def _segment_distance(a, b, c, d):
    # 最近点位于端点投影，或两条直线的内部公垂线；平行与零长度段也由端点分支覆盖。
    distance = min(
        _point_segment_distance(a, c, d),
        _point_segment_distance(b, c, d),
        _point_segment_distance(c, a, b),
        _point_segment_distance(d, a, b),
    )
    u, v, offset = _difference(b, a), _difference(d, c), _difference(c, a)
    normal = _cross(u, v)
    denominator = _dot(normal, normal)
    if denominator > 0:
        s = _dot(_cross(offset, v), normal) / denominator
        t = _dot(_cross(offset, u), normal) / denominator
        if 0 <= s <= 1 and 0 <= t <= 1:
            first = tuple(x + s * delta for x, delta in zip(a, u))
            second = tuple(x + t * delta for x, delta in zip(c, v))
            distance = min(distance, math.dist(first, second))
    return distance


def _intersects_expanded_aabb(start, end, lower, upper, radius):
    # slab求交检查整条线段，不能只比较端点或稀疏采样；扩大盒体包含真实胶囊碰撞。
    entry, leave = 0.0, 1.0
    for a, b, low, high in zip(start, end, lower, upper):
        low, high = low - radius, high + radius
        direction = b - a
        if direction == 0:
            if a < low or a > high:
                return False
            continue
        first, second = (low - a) / direction, (high - a) / direction
        entry = max(entry, min(first, second))
        leave = min(leave, max(first, second))
        if entry > leave:
            return False
    return True


class SceneValidator:
    """按显式名义角限、胶囊半径和场景检查单个000..003协议P姿态。"""

    def __init__(self, model, scene_config):
        if (
            not isinstance(model, dict)
            or model.get("source") != "VENDOR_NOMINAL_UNMEASURED"
            or model.get("frame") != "vendor_X_right_Y_forward_Z_up"
            or model.get("endpoint") != "closed_gripper_tip"
            or model.get("physical_geometry_verified") is not False
            or model.get("motion_authorized") is not False
        ):
            raise ValueError(
                "scene requires an explicitly unmeasured, non-authorizing vendor model"
            )
        if not isinstance(model.get("geometry_mm"), dict):
            raise ValueError("scene requires explicit nominal geometry")
        try:
            self.geometry = Geometry(**model["geometry_mm"])
        except TypeError as error:
            raise ValueError("scene requires the four exact nominal geometry fields") from error
        self.limits = validate_limits(model.get("nominal_angular_limits_deg"))
        if (
            not isinstance(scene_config, dict)
            or set(scene_config) != _SCENE_FIELDS
            or scene_config.get("schema") != "jixiebi.motion_scene.v1"
            or scene_config.get("physical_geometry_verified") is not False
        ):
            raise ValueError("provide exact explicit unverified motion_scene.v1 fields")
        self.table_z_mm = _number(scene_config["table_z_mm"], "table_z_mm", -100000, 100000)
        self.minimum_clearance_mm = _number(
            scene_config["minimum_clearance_mm"], "minimum_clearance_mm", 0, 100000
        )
        self.maximum_radial_mm = _number(
            scene_config["maximum_radial_mm"], "maximum_radial_mm", 0.001, 100000
        )
        radii = scene_config["link_radii_mm"]
        if not isinstance(radii, (list, tuple)) or len(radii) != 3:
            raise ValueError("provide three explicit assumed capsule radii")
        self.link_radii_mm = tuple(_number(r, "link radius", 0.001, 10000) for r in radii)
        self.self_collision_margin_mm = _number(
            scene_config["self_collision_margin_mm"], "self_collision_margin_mm", 0, 100000
        )
        obstacles = scene_config["obstacles_aabb_mm"]
        if not isinstance(obstacles, list) or len(obstacles) > 256:
            raise ValueError("provide an explicit bounded list of at most 256 obstacle boxes")
        parsed = []
        for obstacle in obstacles:
            if not isinstance(obstacle, dict) or set(obstacle) != {"min", "max"}:
                raise ValueError("each obstacle requires exact min and max vectors")
            low, high = (
                _vector(obstacle["min"], "obstacle min"),
                _vector(obstacle["max"], "obstacle max"),
            )
            if any(a >= b for a, b in zip(low, high)):
                raise ValueError("obstacle min must be strictly below max on every axis")
            parsed.append((low, high))
        self.obstacles_aabb_mm = tuple(parsed)
        self.bounds4_p = self._bounds(scene_config["axis_bounds_p"])

    def _bounds(self, explicit):
        if not isinstance(explicit, (list, tuple)) or len(explicit) != 4:
            raise ValueError("provide four explicit axis P intervals")
        # 枚举2001个协议整数而不四舍五入角限，避免把界外整数纳入规划器搜索域。
        # 名义映射是仿射的，每轴可接受P集合必为连续区间；这里只在构造器计算一次。
        parsed = []
        joint_names = tuple(self.limits)
        for axis, pair in enumerate(explicit):
            if (
                not isinstance(pair, (list, tuple))
                or len(pair) != 2
                or any(type(p) is not int or not 500 <= p <= 2500 for p in pair)
                or pair[0] > pair[1]
            ):
                raise ValueError("axis bounds must be ordered protocol integer pairs in 500..2500")
            joint_name = joint_names[axis]
            low, high = self.limits[joint_name]
            allowed = []
            for pulse in range(pair[0], pair[1] + 1):
                values = [1500] * 4
                values[axis] = pulse
                angle = getattr(from_nominal_pulses(values), joint_name)
                if low <= angle <= high:
                    allowed.append(pulse)
            if not allowed:
                raise ValueError("axis P interval has no overlap with nominal angular limits")
            parsed.append((allowed[0], allowed[-1]))
        return tuple(parsed)

    def __call__(self, positions4_p):
        return self.inspect(positions4_p)["valid"]

    def inspect(self, positions4_p, *, minimum_clearance_mm=None):
        """返回单姿态检查结果；非法姿态拒绝，非法场景或阈值抛出ValueError。"""
        # 覆盖值只用于本次桌面检查，不修改普通工作阈值或跳过其它碰撞条件。
        # 先验证阈值，再处理姿态，避免非法姿态掩盖调用方的阈值配置错误。
        required_clearance_mm = (
            self.minimum_clearance_mm
            if minimum_clearance_mm is None
            else _number(minimum_clearance_mm, "minimum_clearance_mm", 0, 100000)
        )
        result = {
            "valid": False,
            "reason": None,
            "source": "NOMINAL_SCENE_ASSUMPTIONS",
            "motion_authorized": False,
            "physical_geometry_verified": False,
            "physical_collision_free_verified": False,
            "continuous_path_verified": False,
            "link_radii_source": "EXPLICIT_UNMEASURED_SCENE_ASSUMPTIONS",
            "required_table_clearance_mm": required_clearance_mm,
            "regular_table_clearance_mm": self.minimum_clearance_mm,
            "notes": list(_NOTES),
        }
        try:
            joints = from_nominal_pulses(positions4_p)
        except ValueError as error:
            return {**result, "code": "INVALID_POSITIONS", "reason": str(error)}
        result["positions4_p"] = list(positions4_p)
        result["nominal_joints_deg"] = asdict(joints)
        if any(not low <= p <= high for p, (low, high) in zip(positions4_p, self.bounds4_p)):
            return {
                **result,
                "code": "AXIS_LIMIT",
                "reason": "Pose exceeds the intersected nominal P bounds.",
            }
        if any(not self.limits[k][0] <= v <= self.limits[k][1] for k, v in vars(joints).items()):
            return {
                **result,
                "code": "NOMINAL_ANGLE_LIMIT",
                "reason": "Pose exceeds nominal angular limits.",
            }
        points = chain_points(self.geometry, positions4_p)
        result["chain_points_mm"] = points
        links = []
        for index, (start, end, radius) in enumerate(zip(points, points[1:], self.link_radii_mm)):
            links.append(
                {
                    "link_index": index,
                    "radius_mm": radius,
                    "minimum_table_clearance_mm": min(start[2], end[2]) - radius - self.table_z_mm,
                    "maximum_radial_mm": max(math.hypot(*start[:2]), math.hypot(*end[:2])) + radius,
                }
            )
        result["links"] = links
        result["minimum_table_clearance_mm"] = min(
            row["minimum_table_clearance_mm"] for row in links
        )
        result["maximum_capsule_radial_mm"] = max(row["maximum_radial_mm"] for row in links)
        for row in links:
            if row["minimum_table_clearance_mm"] < required_clearance_mm:
                return {
                    **result,
                    "code": "TABLE_CLEARANCE",
                    "reason": "Assumed capsule violates table clearance.",
                    "link_index": row["link_index"],
                }
            if row["maximum_radial_mm"] > self.maximum_radial_mm:
                return {
                    **result,
                    "code": "RADIAL_ENVELOPE",
                    "reason": "Assumed capsule exceeds radial envelope.",
                    "link_index": row["link_index"],
                }
        separation = _segment_distance(points[0], points[1], points[2], points[3])
        required = self.link_radii_mm[0] + self.link_radii_mm[2] + self.self_collision_margin_mm
        result["nonadjacent_centerline_distance_mm"] = separation
        result["nonadjacent_required_distance_mm"] = required
        if separation <= required:
            return {
                **result,
                "code": "SELF_COLLISION",
                "reason": "Nonadjacent assumed capsules 0 and 2 collide or violate margin.",
            }
        for link, (start, end, radius) in enumerate(zip(points, points[1:], self.link_radii_mm)):
            for index, (low, high) in enumerate(self.obstacles_aabb_mm):
                if _intersects_expanded_aabb(start, end, low, high, radius):
                    return {
                        **result,
                        "code": "OBSTACLE_COLLISION",
                        "reason": "Assumed capsule intersects a radius-expanded obstacle box.",
                        "link_index": link,
                        "obstacle_index": index,
                    }
        return {
            **result,
            "valid": True,
            "code": "NOMINAL_POSE_VALID",
            "reason": "Pose satisfies the explicit assumed static scene only.",
        }
