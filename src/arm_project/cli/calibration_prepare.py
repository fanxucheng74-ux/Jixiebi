"""Offline inventory, empty measurement templates and evidence validation."""

import argparse
import json
from pathlib import Path

from arm_project.calibration_evidence import (
    EvidenceError,
    inventory,
    templates,
    validate_measurements,
)
from arm_project.paths import PROJECT_ROOT


def _save(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=PROJECT_ROOT)
    sub = parser.add_subparsers(dest="action", required=True)
    inv = sub.add_parser(
        "inventory", help="read explicitly named completed unified session directories"
    )
    inv.add_argument("sessions", nargs="+")
    inv.add_argument("--output", type=Path)
    template = sub.add_parser(
        "template", help="create a NEW directory of empty, unusable measurement templates"
    )
    template.add_argument("directory", type=Path)
    validate = sub.add_parser(
        "validate", help="check measurement references and create a candidate fit only"
    )
    validate.add_argument("file")
    validate.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.action == "template":
            args.directory.mkdir(parents=True, exist_ok=False)
            for name, document in templates().items():
                _save(args.directory / name, document)
            result = {
                "status": "EMPTY_TEMPLATES_CREATED",
                "directory": str(args.directory),
                "motion_authorized": False,
                "physical_accuracy_verified": False,
            }
        else:
            result = (
                inventory(args.root, args.sessions)
                if args.action == "inventory"
                else validate_measurements(args.root, args.file)
            )
            if args.output:
                _save(args.output, result)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (EvidenceError, OSError, KeyError, TypeError, ValueError) as error:
        print(
            json.dumps(
                {
                    "status": "INVALID_OR_INCOMPLETE",
                    "error": str(error),
                    "motion_authorized": False,
                    "physical_accuracy_verified": False,
                },
                ensure_ascii=False,
            )
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
