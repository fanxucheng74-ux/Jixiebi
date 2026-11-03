"""Finite shoulder fixed-time sampling. Default is dry-run with no device I/O."""

import argparse
import hashlib
import json
import re

from arm_project.joint_settling import CHECKPOINTS_S, MOVES, OFFSETS_S, run_experiment
from arm_project.paths import PROJECT_ROOT
from arm_project.unified_arm import HardwareArmDriver


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--run-id")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    plan = {
        "schema_version": 1,
        "servo_id": 1,
        "targets_p": [m.target_p for m in MOVES],
        "duration_ms": 2000,
        "checkpoint_offsets_s": list(CHECKPOINTS_S),
        "sample_offsets_s": list(OFFSETS_S),
        "time_origin": "host motion write return, not servo acknowledgement",
        "maximum_reception_lateness_s": 0.2,
        "within_window_span_limit_p": 2,
        "across_repetitions_span_limit_p": 3,
        "position_tolerance_p": 3,
        "no_compensation": True,
        "no_range_expansion": True,
    }
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    if not args.execute:
        print("DRY_RUN: no device connection or motion")
        return 0
    if not args.port or not args.run_id or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.run_id):
        parser.error("execution requires an explicit port and simple run-id")
    root = PROJECT_ROOT / "runs/unified" / args.run_id
    driver = HardwareArmDriver(args.port, root, planned_moves=MOVES)
    result = {"source": "HARDWARE", "error": None, "physical_task_verified": False, "trials": []}
    try:
        with (root / "settling_plan.json").open("x", encoding="utf-8") as stream:
            json.dump(plan, stream, ensure_ascii=False, indent=2)
        result["plan_sha256"] = hashlib.sha256(
            (root / "settling_plan.json").read_bytes()
        ).hexdigest()

        def progress(trial):
            result["trials"].append(trial)
            with (root / ("trial_%02d.json" % len(result["trials"]))).open(
                "x", encoding="utf-8"
            ) as stream:
                json.dump(trial, stream, ensure_ascii=False, indent=2, allow_nan=False)
            move = trial["move"]
            print(
                "TRIAL",
                move["step_id"],
                "TARGET",
                move["target_p"],
                "FINAL",
                move["final_p"],
                "OBSERVATIONS",
                [s["value"] for s in trial["observation"]["samples"]],
                flush=True,
            )

        with driver:
            result.update(run_experiment(driver, args.run_id, progress=progress))
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = type(error).__name__ + ": " + str(error)
        driver.cancel()
    finally:
        driver.close()
        result["driver"] = driver.snapshot()
        with (root / "settling_result.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print("RESULT:", root / "settling_result.json")
    print("ERROR:", result["error"], "CLOSED:", driver.closed)
    state = result["driver"]
    return (
        0
        if (
            not result["error"]
            and driver.closed
            and not state["faulted"]
            and not state["logging_errors"]
            and state["consumed_count"] == len(MOVES)
            and result.get("analysis", {}).get("sampling_valid")
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
