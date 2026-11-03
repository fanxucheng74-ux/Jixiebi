"""Offline nominal FK/IK and local-envelope analysis. Never opens a device."""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

from arm_project.kinematics import (
    Geometry,
    Pose,
    forward,
    from_nominal_pulses,
    inverse,
    to_nominal_pulses,
    validate_limits,
)
from arm_project.paths import PROJECT_ROOT
from arm_project.arm_tasks import TaughtPose
from arm_project.unified_arm import MoveSpec, PROFILES


def inspect_model(config, *, pulses=None, target=None, pitch=None, duration_ms=2000):
    if (
        not isinstance(config, dict)
        or type(config.get("schema_version")) is not int
        or config["schema_version"] != 1
        or config.get("source") != "VENDOR_NOMINAL_UNMEASURED"
        or config.get("frame") != "vendor_X_right_Y_forward_Z_up"
        or config.get("endpoint") != "closed_gripper_tip"
    ):
        raise ValueError("unsupported nominal model identity/frame/endpoint")
    geometry = Geometry(**config["geometry_mm"])
    limits = validate_limits(config["nominal_angular_limits_deg"])
    current = from_nominal_pulses(pulses) if pulses is not None else None
    current_pose = forward(geometry, current) if current is not None else None
    if target is None:
        if current_pose is None:
            raise ValueError("provide pulses/pose-file or a target and pitch")
        target_pose = Pose(
            current_pose.x_mm,
            current_pose.y_mm,
            current_pose.z_mm,
            current_pose.pitch_deg if pitch is None else pitch,
        )
    else:
        if not isinstance(target, (list, tuple)) or len(target) != 3 or pitch is None:
            raise ValueError("target needs three coordinates and an explicit pitch")
        target_pose = Pose(*target, pitch)
    solutions = inverse(
        geometry, target_pose, limits, base_hint_deg=current.yaw_deg if current else 0
    )
    rows = []
    for solution in solutions:
        candidate = to_nominal_pulses(solution)
        fk = forward(geometry, solution)
        quantized = forward(geometry, from_nominal_pulses(candidate))
        issues = []
        for axis, p in enumerate(candidate):
            try:
                MoveSpec(axis, p, duration_ms)
                if pulses is not None:
                    _, _, low, high, step, _ = PROFILES[axis]
                    if not low <= pulses[axis] <= high or abs(p - pulses[axis]) > step:
                        raise ValueError("recorded start/window or maximum step mismatch")
            except ValueError as error:
                issues.append({"servo_id": axis, "reason": str(error)})
        rows.append(
            {
                "joints_deg": asdict(solution),
                "nominal_pulses_000_003": list(candidate),
                "numerical_fk_error_mm": math.dist(
                    (fk.x_mm, fk.y_mm, fk.z_mm),
                    (target_pose.x_mm, target_pose.y_mm, target_pose.z_mm),
                ),
                "nominal_quantization_error_mm": math.dist(
                    (quantized.x_mm, quantized.y_mm, quantized.z_mm),
                    (target_pose.x_mm, target_pose.y_mm, target_pose.z_mm),
                ),
                "local_command_window_ok": not issues,
                "local_issues": issues,
                "recorded_start_checked": pulses is not None,
            }
        )
    return {
        "schema_version": 1,
        "status": "SOLVED_GEOMETRICALLY" if rows else "UNREACHABLE",
        "model_source": config["source"],
        "geometry_mm": asdict(geometry),
        "nominal_angular_limits_deg": limits,
        "frame": config["frame"],
        "endpoint": config["endpoint"],
        "current_nominal_pose": asdict(current_pose) if current_pose else None,
        "target": asdict(target_pose),
        "solutions": rows,
        "local_candidate_count": sum(row["local_command_window_ok"] for row in rows),
        "held_axes": [4, 5],
        "physical_geometry_verified": False,
        "physical_angle_calibration_verified": False,
        "grasp_center_verified": False,
        "endpoint_assumption": "Closed-gripper reference length; actual aperture and grasp center are not modelled.",
        "motion_authorized": False,
        "limitations": "Nominal closed-tip math only; no collision/path/actual angle or grasp validation. Local checks do not authorize execution.",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=PROJECT_ROOT / "configs/kinematics.vendor_nominal.json"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--pulses", type=int, nargs=4)
    source.add_argument("--pose-file", type=Path)
    parser.add_argument("--pose-name")
    parser.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--pitch", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        pulses = args.pulses
        recorded_at = None
        pose_hash = None
        if args.pose_name and not args.pose_file:
            raise ValueError("pose-name requires pose-file")
        if args.pose_file:
            pose_bytes = args.pose_file.read_bytes()
            data = json.loads(pose_bytes.decode("utf-8"))
            pose_hash = hashlib.sha256(pose_bytes).hexdigest()
            if (
                not isinstance(data, dict)
                or type(data.get("schema_version")) is not int
                or data["schema_version"] != 1
                or not isinstance(data.get("poses"), list)
                or any(not isinstance(p, dict) for p in data["poses"])
            ):
                raise ValueError(
                    "pose-file must contain schema_version1 and a list of pose objects"
                )
            poses = data.get("poses", [])
            selected = [
                p for p in poses if args.pose_name is None or p.get("name") == args.pose_name
            ]
            if len(selected) != 1 or selected[0].get("source") != "HARDWARE":
                raise ValueError("select exactly one hardware snapshot (still nominal analysis)")
            pose = TaughtPose.from_dict(selected[0])
            pulses, recorded_at = list(pose.positions_p[:4]), pose.captured_at
        config_bytes = args.config.read_bytes()
        config = json.loads(config_bytes.decode("utf-8"))
        report = inspect_model(config, pulses=pulses, target=args.target, pitch=args.pitch)
        report["recorded_at"] = recorded_at
        report["config_sha256"] = hashlib.sha256(config_bytes).hexdigest()
        report["pose_file_sha256"] = pose_hash
        if args.output:
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if report["solutions"] else 2
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(
            json.dumps(
                {"status": "INVALID_INPUT", "error": str(error), "motion_authorized": False},
                ensure_ascii=False,
            )
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
