"""Execute a separately reviewed finite joint group; default is offline dry-run."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path

from arm_project.arm_link import ServoLink
from arm_project.coordinated_motion import (
    CoordinatedController,
    CoordinatedSpec,
    CoordinatedTransport,
)
from arm_project.paths import PROJECT_ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--port")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    spec = CoordinatedSpec.from_dict(json.loads(args.spec.read_text(encoding="utf-8")))
    print("PLAN:", [a.command.decode("ascii") for a in spec.axes])
    if not args.execute:
        print("DRY_RUN: no serial connection")
        return 0
    if not args.port:
        parser.error("--execute needs explicit --port")
    folder = PROJECT_ROOT / "runs/coordinated" / spec.review_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "spec.json").write_text(
        json.dumps(asdict(spec), ensure_ascii=False, indent=2), encoding="utf-8"
    )
    transport = CoordinatedTransport(args.port, spec)
    result = {"source": "HARDWARE", "motion": None, "error": None}

    def reserve(plan):
        with (folder / "batch_attempt.json").open("x", encoding="utf-8") as f:
            json.dump(
                {"claimed_at": datetime.now().astimezone().isoformat(), "plan": plan},
                f,
                ensure_ascii=False,
                indent=2,
            )
            f.flush()
            os.fsync(f.fileno())

    with (folder / "events.jsonl").open("x", encoding="utf-8") as log:

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
                result["motion"] = CoordinatedController(
                    link, spec, reserve, cancel_file=folder / "cancel.request"
                ).run()
        except (Exception, KeyboardInterrupt) as error:
            result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            result["transport_closed"] = transport.closed
            result["attempted_ids"] = list(transport.attempted_ids)
            result["logging_errors"] = list(link.log_errors) if "link" in locals() else []
    with (folder / "result.json").open("x", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    motion = result["motion"] or {}
    print(
        "STATUS:",
        motion.get("status"),
        "RESPONSE:",
        motion.get("response_pass"),
        "POSITION:",
        motion.get("position_reached"),
    )
    print("ERROR:", result["error"] or motion.get("error"), "CLOSED:", result["transport_closed"])
    print("RESULT:", folder / "result.json")
    return (
        0
        if not result["error"]
        and not motion.get("error")
        and motion.get("response_pass")
        and not result["logging_errors"]
        and result["transport_closed"]
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
