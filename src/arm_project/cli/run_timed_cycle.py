"""Validate a reviewed sequence; --port and --execute run it exactly once."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path

from arm_project.arm_link import ServoLink
from arm_project.paths import PROJECT_ROOT
from arm_project.timed_cycle import CycleController, CycleSpec, CycleTransport


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", required=True, type=Path)
    parser.add_argument("--port")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    payload = args.spec.read_text(encoding="utf-8")
    spec = CycleSpec.from_dict(json.loads(payload))
    print("PHASES:", [(p.name, p.duration_ms, p.dwell_ms) for p in spec.phases])
    print(
        "DECLARED_MS:",
        sum(p.duration_ms + p.dwell_ms for p in spec.phases),
        "DEADLINE_MS:",
        spec.deadline_ms,
    )
    if not args.execute:
        print("DRY_RUN: no serial connection")
        return 0
    if not args.port:
        parser.error("explicit --port required for --execute")
    folder = PROJECT_ROOT / "runs/timed_cycles" / spec.review_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "spec.json").write_text(payload, encoding="utf-8")

    def reserve(plan):
        with (folder / "task_attempt.json").open("x", encoding="utf-8") as f:
            json.dump(
                {"claimed_at": datetime.now().astimezone().isoformat(), "plan": plan},
                f,
                ensure_ascii=False,
                indent=2,
            )
            f.flush()
            os.fsync(f.fileno())

    gate = CycleTransport(args.port, spec)
    result = {"source": "HARDWARE", "sequence": None, "error": None}
    with (folder / "events.jsonl").open("x", encoding="utf-8") as f:

        def record(event, **values):
            f.write(
                json.dumps(
                    {"time": datetime.now().astimezone().isoformat(), "event": event, **values},
                    ensure_ascii=False,
                )
                + "\n"
            )
            f.flush()

        try:
            with ServoLink(gate, record=record) as link:
                result["sequence"] = CycleController(
                    link, spec, reserve, cancel_file=folder / "cancel.request"
                ).run()
        except (Exception, KeyboardInterrupt) as error:
            result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            result["transport_closed"] = gate.closed
            result["logging_errors"] = list(link.log_errors) if "link" in locals() else []
    with (folder / "result.json").open("x", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    sequence = result["sequence"] or {}
    print(
        "STATUS:",
        sequence.get("status"),
        "ELAPSED_S:",
        sequence.get("elapsed_s"),
        "ERROR:",
        result["error"] or sequence.get("error"),
    )
    print("RESULT:", folder / "result.json")
    return (
        0
        if sequence.get("response_pass")
        and sequence.get("within_deadline")
        and not result["error"]
        and not result["logging_errors"]
        and result["transport_closed"]
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
