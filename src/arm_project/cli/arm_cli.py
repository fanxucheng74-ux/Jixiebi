"""统一机械臂命令入口：默认仅检查计划，显式--execute才连接硬件。"""

from arm_project.paths import PROJECT_ROOT
import argparse
import json
from pathlib import Path
import re

from arm_project.unified_arm import HardwareArmDriver, MoveSpec

ROOT = PROJECT_ROOT


def build_plan(data):
    if (
        not isinstance(data, dict)
        or type(data.get("schema_version")) is not int
        or data["schema_version"] != 1
    ):
        raise ValueError("plan schema_version must be1")
    if set(data) - {"schema_version", "description", "steps"}:
        raise ValueError("unknown plan fields")
    if not isinstance(data.get("steps"), list) or not 1 <= len(data["steps"]) <= 40:
        raise ValueError("plan requires1..40 steps")
    steps, moves, names = [], [], set()
    for index, raw in enumerate(data["steps"]):
        if not isinstance(raw, dict):
            raise ValueError("step must be an object")
        kind = raw.get("kind")
        if kind == "read_state":
            keys = {"kind"}
        elif kind == "capture_pose":
            keys = {"kind", "name"}
            name = raw.get("name")
            if (
                not isinstance(name, str)
                or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name)
                or name in names
            ):
                raise ValueError("capture names must be unique simple names")
            names.add(name)
        elif kind == "move_joint":
            keys = {"kind", "servo_id", "target_p", "duration_ms"}
            if set(raw) != keys:
                raise ValueError("move_joint requires exact servo_id/target_p/duration_ms fields")
            moves.append(MoveSpec(raw["servo_id"], raw["target_p"], raw["duration_ms"]))
        elif kind == "grip":
            keys = {"kind", "action"}
            if raw.get("action") not in ("open", "close"):
                raise ValueError("grip action must be open or close")
            moves.append(MoveSpec(5, 1480 if raw["action"] == "open" else 1570, 1000))
        else:
            raise ValueError("unknown step kind: " + str(kind))
        if set(raw) != keys:
            raise ValueError("unknown or missing fields in step" + str(index + 1))
        steps.append(dict(raw))
    if len(moves) > 20:
        raise ValueError("at most20 exact motion plan entries")
    return tuple(steps), tuple(moves)


def operation_accepted(kind, result):
    if kind == "read_state":
        return isinstance(result, dict) and result.get("status") == "OK"
    if kind == "capture_pose":
        return (
            isinstance(result, dict)
            and result.get("source") == "HARDWARE"
            and result.get("verified") is False
        )
    return (
        isinstance(result, dict)
        and (result.get("response_pass") is True or result.get("position_reached") is True)
        and not result.get("error")
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--port")
    parser.add_argument("--run-id")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--cancel-run",
        help="request cancellation of an existing named session; no new serial connection",
    )
    args = parser.parse_args()
    if args.cancel_run:
        if args.plan or args.execute or args.port or args.run_id:
            parser.error("--cancel-run cannot be combined with execution arguments")
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.cancel_run):
            parser.error("invalid run-id")
        folder = ROOT / "runs" / "unified" / args.cancel_run
        if not folder.is_dir() or not (folder / "request_plan.json").exists():
            parser.error("unknown CLI session")
        if (folder / "cli_result.json").exists():
            print("Session already ended; no cancellation needed")
            return 0
        marker = folder / "cancel.request"
        try:
            with marker.open("x", encoding="utf-8") as out:
                out.write("User requested cancellation of remaining commands.\n")
        except FileExistsError:
            pass
        print("CANCEL REQUESTED; physical stop is not verified:", marker)
        return 0
    if args.plan is None:
        parser.error("--plan is required")
    try:
        data = json.loads(args.plan.read_text(encoding="utf-8-sig"))
        steps, moves = build_plan(data)
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    print("PLAN:", len(steps), "steps,", len(moves), "exact motion entries")
    if not args.execute:
        print("DRY_RUN: no serial connection or motion")
        for number, step in enumerate(steps, 1):
            print(number, json.dumps(step, ensure_ascii=False))
        return 0
    if not args.port or not args.run_id or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.run_id):
        parser.error("--execute requires explicit --port and simple --run-id")
    parent = ROOT / "runs" / "unified"
    parent.mkdir(parents=True, exist_ok=True)
    folder = parent / args.run_id
    result = {
        "source": "HARDWARE",
        "run_id": args.run_id,
        "steps": [],
        "poses": [],
        "error": None,
        "grasp_verified": False,
        "physical_task_verified": False,
    }
    driver = None
    try:
        driver = HardwareArmDriver(args.port, folder, planned_moves=moves)
        with (folder / "request_plan.json").open("x", encoding="utf-8") as out:
            json.dump(data, out, ensure_ascii=False, indent=2)
        with driver:
            for index, step in enumerate(steps):
                kind = step["kind"]
                identity = {"execution_id": args.run_id, "step_id": "%02d:%s" % (index, kind)}
                if kind == "read_state":
                    value = driver.read_state()
                elif kind == "capture_pose":
                    pose = driver.capture_pose(step["name"])
                    value = pose.to_dict() if hasattr(pose, "to_dict") else pose
                    result["poses"].append(value)
                elif kind == "move_joint":
                    value = driver.move_joint(
                        step["servo_id"], step["target_p"], step["duration_ms"], **identity
                    )
                else:
                    value = driver.grip(step["action"], **identity)
                result["steps"].append({"step": step, "result": value})
                print(
                    "STEP",
                    index + 1,
                    kind,
                    "STATUS:",
                    value.get("status", "CAPTURED"),
                    "RESPONSE:",
                    value.get("response_pass"),
                    "POSITION:",
                    value.get("position_reached"),
                    flush=True,
                )
                if not operation_accepted(kind, value):
                    raise RuntimeError(
                        "step did not meet its acceptance; remaining steps cancelled"
                    )
                current = driver.snapshot()
                if current.get("faulted") or current.get("cancel_requested"):
                    raise RuntimeError("driver fault/cancellation prevents further steps")
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = type(error).__name__ + ": " + (str(error) or "interrupted")
        if driver is not None:
            driver.cancel()
    finally:
        if driver is not None:
            driver.close()
            result["driver"] = driver.snapshot()
            result["closed"] = driver.closed
            with (folder / "cli_result.json").open("x", encoding="utf-8") as out:
                json.dump(result, out, ensure_ascii=False, indent=2)
            if result["poses"]:
                with (folder / "captured_poses.json").open("x", encoding="utf-8") as out:
                    json.dump(
                        {"schema_version": 1, "poses": result["poses"]},
                        out,
                        ensure_ascii=False,
                        indent=2,
                    )
    print("ERROR:", result["error"], "CLOSED:", result.get("closed"))
    if driver is not None:
        print("RESULT:", folder / "cli_result.json")
    state = result.get("driver", {})
    return (
        0
        if (
            not result["error"]
            and result.get("closed")
            and not state.get("logging_errors")
            and not state.get("faulted")
            and not state.get("cancel_requested")
            and state.get("consumed_count") == len(moves)
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
