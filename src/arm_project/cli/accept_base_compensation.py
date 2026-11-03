"""12P排查后的局部补偿验收：固定3条运动预算，每次独立执行并检查前次结果。"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json
import os

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE, load_limits
from arm_project.arm_state import ArmStateReader
from arm_project.base_compensation import CompensatedBaseController

ROOT = PROJECT_ROOT
EXPERIMENT = ROOT / "runs" / "base_compensation_20261031"
STEPS = {
    "negative_1": (1530, 1515, None),
    "positive_1": (1540, 1552, "negative_1"),
    "negative_2": (1530, 1515, "positive_1"),
}


def final_base_p(data):
    state = data.get("final_state") or {}
    rows = [s for s in state.get("servos", []) if s.get("servo_id") == 0]
    if state.get("status") != "OK" or len(rows) != 1 or rows[0].get("status") != "OK":
        raise ValueError("missing successful final base observation")
    p = rows[0].get("position_p")
    if type(p) is not int or not 1505 <= p <= 1555:
        raise ValueError("final base observation outside window")
    return p


def acceptance_ok(data):
    try:
        p = final_base_p(data)
    except ValueError:
        return False
    ref = data.get("reference") or {}
    return (
        not data.get("error")
        and not data.get("logging_errors")
        and data.get("transport_closed") is True
        and ref.get("status") == "REFERENCE_REACHED"
        and ref.get("reference_reached") is True
        and not (ref.get("command_result") or {}).get("logging_errors")
        and type(ref.get("reference_p")) is int
        and abs(ref["reference_p"] - p) <= 3
    )


def require_previous(phase, experiment=EXPERIMENT):
    _, _, previous = STEPS[phase]
    if previous is None:
        # 诊断必须完整结束且最后会话正常；原始±3P不通过本身不是通信故障。
        path = ROOT / "runs/base_residual_20261031/positive_slow/result.json"
    else:
        path = experiment / previous / "result.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    if (
        data.get("error")
        or data.get("logging_errors")
        or not data.get("transport_closed")
        or data.get("final_state", {}).get("status") != "OK"
    ):
        raise ValueError("previous experiment did not end cleanly")
    if previous is not None:
        if not acceptance_ok(data):
            raise ValueError("previous compensation did not meet the unchanged tolerance")
    else:
        final_base_p(data)
        m = data.get("motion", {})
        if (
            m.get("status") not in ("TARGET_REACHED", "OBSERVED_NOT_AT_TARGET")
            or not m.get("motion_write_complete")
            or m.get("logging_errors")
            or m.get("tail_span_p", 999) > 2
        ):
            raise ValueError("last diagnosis was incomplete or unstable")


def reserve_step(phase, plan, experiment=EXPERIMENT):
    reference, command, _ = STEPS[phase]
    if (plan["servo_id"], plan["reference_p"], plan["target_p"], plan["duration_ms"]) != (
        0,
        reference,
        command,
        1500,
    ):
        raise ValueError("reference/direction/command differs from fixed acceptance step")
    with (experiment / (phase + "_attempt.json")).open("x", encoding="utf-8") as stream:
        json.dump({"time": datetime.now().astimezone().isoformat(), "plan": plan}, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--phase", choices=list(STEPS), required=True)
    args = parser.parse_args()
    require_previous(args.phase)
    run_dir = EXPERIMENT / args.phase
    run_dir.mkdir(parents=True, exist_ok=False)
    transport = SerialTransport(args.port)
    result = {"phase": args.phase, "source": "HARDWARE", "error": None, "reference": None}
    with (run_dir / "events.jsonl").open("x", encoding="utf-8") as log:

        def record(event, **values):
            log.write(
                json.dumps(
                    {"time": datetime.now().astimezone().isoformat(), "event": event, **values},
                    ensure_ascii=False,
                )
                + "\n"
            )
            log.flush()

        try:
            with ServoLink(transport, record=record) as link:
                result["identity"] = ArmStateReader(link).read_identity()
                if any(
                    i["replies"]["version"]["value"] != EXPECTED_FIRMWARE
                    or i["replies"]["mode"]["value"] != 1
                    for i in result["identity"]
                ):
                    raise ValueError("firmware/mode differs from verified setup")
                driver = CompensatedBaseController(
                    link,
                    load_limits(ROOT / "configs" / "joint_limits.json"),
                    before_motion=lambda plan: reserve_step(args.phase, plan),
                )
                result["reference"] = driver.move_reference(STEPS[args.phase][0])
                if not link.faulted:
                    result["final_state"] = ArmStateReader(link).read_state()
                    if result["reference"]["reference_reached"]:
                        p = final_base_p(result)
                        result["final_hold_error_p"] = result["reference"]["reference_p"] - p
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            result["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            result["transport_closed"] = transport.closed
            if "link" in locals():
                result["logging_errors"] = link.log_errors
    result["acceptance_passed"] = acceptance_ok(result)
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    ref = result.get("reference") or {}
    print("PHASE:", args.phase, "STATUS:", ref.get("status"), "ERROR:", result["error"])
    print(
        "REFERENCE:",
        ref.get("reference_p"),
        "COMMAND:",
        ref.get("command_p"),
        "FINAL:",
        ref.get("final_p"),
        "REFERENCE_ERROR:",
        ref.get("reference_error_p"),
    )
    print("CLOSED:", result["transport_closed"], "RESULT:", run_dir / "result.json")
    print("ACCEPTANCE_PASSED:", result["acceptance_passed"])
    return 0 if result["acceptance_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
