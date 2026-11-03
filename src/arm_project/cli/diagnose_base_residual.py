"""2026-10-31用户新授权的12P诊断；固定000范围、4条运动上限、每阶段只执行一次。

不删除V1标记。每阶段单独调用、审查结果后才执行下一阶段；不自动追赶或回程。
"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json
import os

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE, MotionController, load_limits
from arm_project.arm_state import ArmStateReader

ROOT = PROJECT_ROOT
EXPERIMENT = ROOT / "runs" / "base_residual_20261031"
STEPS = {
    "negative_fast": (1505, 1500, "observe"),
    "positive_fast": (1555, 1500, "negative_fast"),
    "negative_slow": (1505, 3000, "positive_fast"),
    "positive_slow": (1555, 3000, "negative_slow"),
}


def check_previous(phase, experiment=EXPERIMENT):
    if phase == "observe":
        return
    previous = json.loads(
        (experiment / STEPS[phase][2] / "result.json").read_text(encoding="utf-8")
    )
    if (
        previous.get("error")
        or previous.get("logging_errors")
        or not previous.get("transport_closed")
    ):
        raise ValueError("previous phase did not end cleanly")
    if previous.get("final_state", {}).get("status") != "OK":
        raise ValueError("previous final state was incomplete")
    if previous.get("motion"):
        motion = previous["motion"]
        if motion["status"] not in ("TARGET_REACHED", "OBSERVED_NOT_AT_TARGET"):
            raise ValueError("previous motion failed or was not executed")
        if motion.get("logging_errors") or not motion.get("motion_write_complete"):
            raise ValueError("previous write/logging was incomplete")
        if (
            not motion.get("directed_feedback_change")
            or len(motion.get("feedback", [])) != 11
            or motion.get("tail_span_p", 999) > 2
        ):
            raise ValueError(
                "previous motion did not show a stable change in the intended direction"
            )
    elif phase == "negative_fast":
        samples = previous.get("observation", [])
        if (
            len(samples) != 11
            or max(s["value"] for s in samples) - min(s["value"] for s in samples) > 2
        ):
            raise ValueError("initial observation was missing or unstable")
    else:
        raise ValueError("missing previous motion")


def claim_step(phase, plan, experiment=EXPERIMENT):
    target, duration, _ = STEPS[phase]
    if (plan["servo_id"], plan["target_p"], plan["duration_ms"]) != (0, target, duration):
        raise ValueError("motion differs from the fixed experiment step")
    marker = experiment / (phase + "_attempt.json")
    with marker.open("x", encoding="utf-8") as stream:
        json.dump({"time": datetime.now().astimezone().isoformat(), "plan": plan}, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--phase", choices=["observe", *STEPS], required=True)
    args = parser.parse_args()
    check_previous(args.phase)
    run_dir = EXPERIMENT / args.phase
    run_dir.mkdir(parents=True, exist_ok=False)
    summary = {
        "phase": args.phase,
        "source": "HARDWARE",
        "error": None,
        "motion": None,
        "observation": [],
        "physical_motion_independently_verified": False,
    }
    transport = SerialTransport(args.port)
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
                identities = ArmStateReader(link).read_identity()
                summary["identity"] = identities
                if any(
                    i["replies"]["version"]["value"] != EXPECTED_FIRMWARE
                    or i["replies"]["mode"]["value"] != 1
                    for i in identities
                ):
                    raise ValueError("firmware/mode differs from verified hardware")
                if args.phase == "observe":
                    for index in range(11):
                        sample = link.query(0)
                        summary["observation"].append(sample)
                        if not 1505 <= sample["value"] <= 1555:
                            raise ValueError("observation outside reviewed base window")
                        if index < 10:
                            link.clock.sleep(0.5)
                else:
                    target, duration, _ = STEPS[args.phase]
                    controller = MotionController(
                        link,
                        load_limits(ROOT / "configs" / "joint_limits.json"),
                        before_motion=lambda plan: claim_step(args.phase, plan),
                    )
                    summary["motion"] = controller.move_joint(
                        0, target, duration, feedback_samples=11, feedback_interval_s=0.5
                    )
                    if summary["motion"]["error"]:
                        raise RuntimeError(summary["motion"]["error"])
                summary["final_state"] = ArmStateReader(link).read_state()
                if summary["final_state"]["status"] != "OK":
                    raise RuntimeError("final state incomplete")
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            summary["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            summary["transport_closed"] = transport.closed
            if "link" in locals():
                summary["logging_errors"] = link.log_errors
    (run_dir / "result.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("PHASE:", args.phase, "ERROR:", summary["error"], "CLOSED:", summary["transport_closed"])
    if summary["observation"]:
        print("OBSERVATION:", [s["value"] for s in summary["observation"]])
    if summary["motion"]:
        m = summary["motion"]
        print(
            "MOTION:",
            m["status"],
            "START:",
            m.get("start_p"),
            "TARGET:",
            m["target_p"],
            "FINAL:",
            m.get("final_p"),
            "ERROR_P:",
            m.get("error_p"),
        )
        print("FEEDBACK:", [s["value"] for s in m["feedback"]])
    print("RESULT:", run_dir / "result.json")
    # 诊断采集完成不等于到位验收成功，实际判据保留于motion.status。
    return 1 if summary["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
