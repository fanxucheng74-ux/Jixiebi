"""Create an offline nominal approach proposal; no execution option is provided."""

import argparse
import hashlib
import json
from pathlib import Path

from arm_project.approach_planning import plan_approach
from arm_project.arm_tasks import TaughtPose
from arm_project.paths import PROJECT_ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-file", type=Path, required=True)
    parser.add_argument("--pose-name")
    parser.add_argument(
        "--model", type=Path, default=PROJECT_ROOT / "configs/kinematics.vendor_nominal.json"
    )
    parser.add_argument("--spec", type=Path, default=PROJECT_ROOT / "configs/approach.example.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        payloads = [p.read_bytes() for p in (args.pose_file, args.model, args.spec)]
        poses, model, spec = [json.loads(p.decode("utf-8-sig")) for p in payloads]
        if (
            not isinstance(poses, dict)
            or type(poses.get("schema_version")) is not int
            or poses["schema_version"] != 1
            or not isinstance(poses.get("poses"), list)
            or any(not isinstance(p, dict) for p in poses["poses"])
        ):
            raise ValueError("invalid snapshot schema")
        selected = [
            p for p in poses["poses"] if args.pose_name is None or p.get("name") == args.pose_name
        ]
        if len(selected) != 1 or selected[0].get("source") != "HARDWARE":
            raise ValueError("select exactly one recorded hardware snapshot for nominal analysis")
        pose = TaughtPose.from_dict(selected[0])
        report = plan_approach(model, spec, list(pose.positions_p[:4]))
        report["recorded_at"] = pose.captured_at
        report["input_sha256"] = {
            name: hashlib.sha256(data).hexdigest()
            for name, data in zip(("pose", "model", "spec"), payloads)
        }
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "stages": len(report["stages"]),
                    "one_axis_steps": len(report["steps"]),
                    "outside_current_window_steps": report["outside_current_window_steps"],
                    "review_batches": len(report["review_batches"]),
                    "motion_authorized": False,
                    "output": str(args.output),
                },
                ensure_ascii=False,
            )
        )
        return 0 if report["status"] == "OFFLINE_PROPOSAL" else 2
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
