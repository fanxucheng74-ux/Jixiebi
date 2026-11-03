"""Offline nominal approach proposals. No serial, camera or motion authorization."""

from dataclasses import asdict
import math

from arm_project.kinematics import (
    Geometry,
    Pose,
    finite,
    forward,
    from_nominal_pulses,
    inverse,
    to_nominal_pulses,
    validate_limits,
)
from arm_project.unified_arm import PROFILES


def chain_points(geometry, pulses):
    joints = from_nominal_pulses(pulses)
    yaw, shoulder, elbow, wrist = map(math.radians, vars(joints).values())
    points = [[0.0, 0.0, geometry.base_height_mm]]
    radial, z = 0.0, geometry.base_height_mm
    for length, angle in (
        (geometry.upper_arm_mm, shoulder),
        (geometry.forearm_mm, shoulder - elbow),
        (geometry.tool_mm, shoulder - elbow + wrist),
    ):
        radial += length * math.cos(angle)
        z += length * math.sin(angle)
        points.append([radial * math.sin(yaw), radial * math.cos(yaw), z])
    return points


def is_nominal_upright_reference(geometry, pulses, *, points=None):
    """A narrow high column for the uncalibrated upright reference only.

    All nodes must fit a20mm-radius column, with shoulder90+/-5deg and
    elbow/wrist+/-5deg. Any rearward node must be above200mm. With the
    task's35mm object allowance this occupies55mm versus the vendor CAD's
    nearest85mm plate edge. This is nominal evidence, not a collision test.
    """
    q = from_nominal_pulses(pulses)
    if not (abs(q.shoulder_deg - 90) <= 5 and abs(q.elbow_flex_deg) <= 5 and abs(q.wrist_deg) <= 5):
        return False
    points = chain_points(geometry, pulses) if points is None else points
    if max(math.hypot(p[0], p[1]) for p in points) > 20:
        return False
    yaw = math.radians(q.yaw_deg)
    return all(p[2] >= 200 for p in points if p[0] * math.sin(yaw) + p[1] * math.cos(yaw) < -1e-9)


def _sin_bounds(low, high):
    values = [math.sin(math.radians(low)), math.sin(math.radians(high))]
    for n in range(math.ceil((low - 90) / 180), math.floor((high - 90) / 180) + 1):
        values.append(math.sin(math.radians(90 + 180 * n)))
    return min(values), max(values)


def nominal_height_bounds(geometry, pulse_windows):
    """Conservative continuous tip-Z bounds, allowing correlated-angle overbounds."""
    if (
        not isinstance(geometry, Geometry)
        or not isinstance(pulse_windows, (list, tuple))
        or len(pulse_windows) != 4
    ):
        raise ValueError("Geometry and four pulse intervals required")
    windows = []
    for pair in pulse_windows:
        if (
            not isinstance(pair, (list, tuple))
            or len(pair) != 2
            or any(type(p) is not int or not 500 <= p <= 2500 for p in pair)
            or pair[0] > pair[1]
        ):
            raise ValueError("invalid pulse interval")
        windows.append(pair)
    factor = 270 / 2000
    s = tuple(90 + (p - 1500) * factor for p in windows[1])
    e = tuple((p - 1500) * factor for p in windows[2])
    w = tuple((p - 1500) * factor for p in windows[3])
    angles = (s, (s[0] - e[1], s[1] - e[0]), (s[0] - e[1] + w[0], s[1] - e[0] + w[1]))
    lengths = (geometry.upper_arm_mm, geometry.forearm_mm, geometry.tool_mm)
    bounds = [_sin_bounds(*pair) for pair in angles]
    return {
        "lower_mm": geometry.base_height_mm
        + sum(length * bound[0] for length, bound in zip(lengths, bounds)),
        "upper_mm": geometry.base_height_mm
        + sum(length * bound[1] for length, bound in zip(lengths, bounds)),
        "source": "CONSERVATIVE_NOMINAL_INTERVAL",
        "physical_height_verified": False,
    }


def transition(geometry, limits, start, target, *, max_step_p=20, floor_z_mm=0, margin_mm=15):
    """Interleave one-axis updates, inspect every integer-P centerline sample.

    Centerline/assumed-plane checks are not full continuous collision checking:
    finite link width, obstacles, base fixation and angle calibration are unknown.
    """
    bounds = validate_limits(limits)
    from_nominal_pulses(start)
    from_nominal_pulses(target)
    if type(max_step_p) is not int or not 1 <= max_step_p <= 40:
        raise ValueError("max_step_p must be an integer in1..40")
    floor = finite(floor_z_mm, "floor_z_mm")
    margin = finite(margin_mm, "margin_mm")
    if not -10000 <= floor <= 10000 or not 0 <= margin <= 1000:
        raise ValueError("floor or margin outside numerical domain")
    current = list(start)
    steps = []
    minimum = float("inf")

    def inspect(pulses):
        nonlocal minimum
        joint = from_nominal_pulses(pulses)
        if any(not bounds[n][0] <= v <= bounds[n][1] for n, v in vars(joint).items()):
            raise ValueError("nominal angular bound exceeded")
        points = chain_points(geometry, pulses)
        height = min(p[2] for p in points)
        minimum = min(minimum, height)
        if height < floor + margin:
            raise ValueError("sampled centerline violates the assumed plane margin")

    inspect(current)
    while current != list(target):
        for axis in (1, 2, 3, 0):
            delta = target[axis] - current[axis]
            if not delta:
                continue
            ending = current[axis] + max(-max_step_p, min(max_step_p, delta))
            sign = 1 if ending > current[axis] else -1
            for p in range(current[axis] + sign, ending + sign, sign):
                check = current.copy()
                check[axis] = p
                inspect(check)
            before = current.copy()
            current[axis] = ending
            low, high, _, _, maximum, duration = PROFILES[axis]
            steps.append(
                {
                    "servo_id": axis,
                    "target_p": ending,
                    "duration_ms": max(duration, 2000),
                    "start_nominal_p": before,
                    "end_nominal_p": current.copy(),
                    "within_current_command_window": low <= ending <= high,
                    "within_current_step_limit": abs(ending - before[axis]) <= maximum,
                    "nominal_tip": asdict(forward(geometry, from_nominal_pulses(current))),
                }
            )
            if len(steps) > 2000:
                raise ValueError("proposal exceeds2000 one-axis updates")
    return {
        "steps": steps,
        "minimum_sampled_centerline_z_mm": minimum,
        "end_pulses": current,
        "sample_step_p": 1,
        "continuous_collision_check": False,
    }


def plan_approach(model, specification, start):
    if (
        not isinstance(model, dict)
        or model.get("source") != "VENDOR_NOMINAL_UNMEASURED"
        or model.get("frame") != "vendor_X_right_Y_forward_Z_up"
        or model.get("endpoint") != "closed_gripper_tip"
    ):
        raise ValueError("explicit vendor nominal closed-tip model required")
    if (
        not isinstance(specification, dict)
        or type(specification.get("schema_version")) is not int
        or specification["schema_version"] != 1
        or specification.get("source") != "OFFLINE_ASSUMED"
        or specification.get("physical_target_verified") is not False
    ):
        raise ValueError("only declared offline assumed targets are supported here")
    stages = specification.get("stages")
    if not isinstance(stages, list) or not 1 <= len(stages) <= 12:
        raise ValueError("provide1..12 named Cartesian stages")
    geometry = Geometry(**model["geometry_mm"])
    limits = validate_limits(model["nominal_angular_limits_deg"])
    from_nominal_pulses(start)
    current = list(start)
    transition(
        geometry,
        limits,
        current,
        current,
        max_step_p=specification["max_step_p"],
        floor_z_mm=specification["assumed_table_z_mm"],
        margin_mm=specification["centerline_margin_mm"],
    )
    rows, steps, names = [], [], set()
    for stage in stages:
        if not isinstance(stage, dict) or set(stage) != {"name", "pose"}:
            raise ValueError("stage needs exactly name and pose")
        name = stage["name"]
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError("stage names must be nonempty and unique")
        names.add(name)
        target = Pose(**stage["pose"])
        solutions = inverse(
            geometry, target, limits, base_hint_deg=from_nominal_pulses(current).yaw_deg
        )
        candidates, rejected = [], []
        for joint in solutions:
            try:
                pulses = list(to_nominal_pulses(joint))
                path = transition(
                    geometry,
                    limits,
                    current,
                    pulses,
                    max_step_p=specification["max_step_p"],
                    floor_z_mm=specification["assumed_table_z_mm"],
                    margin_mm=specification["centerline_margin_mm"],
                )
                candidates.append(
                    (sum((p - q) ** 2 for p, q in zip(pulses, current)), pulses, path)
                )
            except ValueError as error:
                rejected.append({"joints": asdict(joint), "reason": str(error)})
        if not candidates:
            rows.append(
                {
                    "name": name,
                    "status": "NO_NOMINAL_CANDIDATE",
                    "target": asdict(target),
                    "ik_solution_count": len(solutions),
                    "rejected": rejected,
                }
            )
            break
        _, pulses, path = min(candidates, key=lambda row: row[0])
        rows.append(
            {
                "name": name,
                "status": "NOMINAL_PROPOSAL",
                "target": asdict(target),
                "candidate_count": len(candidates),
                "target_pulses": pulses,
                "quantized_tip": asdict(forward(geometry, from_nominal_pulses(pulses))),
                "first_step": len(steps),
                "step_count": len(path["steps"]),
                "minimum_sampled_centerline_z_mm": path["minimum_sampled_centerline_z_mm"],
            }
        )
        steps.extend({"stage": name, **step} for step in path["steps"])
        current = pulses
    solved = len(rows) == len(stages) and all(r["status"] == "NOMINAL_PROPOSAL" for r in rows)
    return {
        "schema_version": 1,
        "status": "OFFLINE_PROPOSAL" if solved else "NOMINAL_PATH_BLOCKED",
        "model_source": model["source"],
        "target_source": specification["source"],
        "start_pulses": list(start),
        "stages": rows,
        "steps": steps,
        "review_batches": [
            {
                "first_step": n,
                "step_count": min(20, len(steps) - n),
                "fresh_state_and_review_required": True,
            }
            for n in range(0, len(steps), 20)
        ],
        "outside_current_window_steps": sum(not s["within_current_command_window"] for s in steps),
        "current_window_nominal_tip_z_bounds": nominal_height_bounds(
            geometry, [PROFILES[i][:2] for i in range(4)]
        ),
        "physical_target_verified": False,
        "physical_geometry_verified": False,
        "obstacles_checked": False,
        "continuous_collision_check": False,
        "base_fixation_verified": False,
        "motion_authorized": False,
        "held_axes": [4, 5],
        "branch_policy": "nearest pulse-space candidate per stage; no global optimality claim",
    }
