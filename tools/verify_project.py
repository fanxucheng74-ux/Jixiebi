"""Run public offline acceptance; optionally audit explicitly supplied private evidence.

Never opens a serial port/camera or executes a hardware plan. Generated reports
use a new directory; raw experiment files and consumed budgets are read-only.
Private logs are not shipped with the public repository and are not audited by default.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def validate_evidence_integrity():
    checked, errors = 0, []
    baseline_path = ROOT / "artifacts/migration/before.json"
    if not baseline_path.is_file():
        return {
            "passed": False,
            "files_checked": 0,
            "errors": ["missing private baseline: artifacts/migration/before.json"],
        }
    try:
        baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
        entries = baseline["files"]
    except (OSError, ValueError, KeyError, TypeError) as error:
        return {"passed": False, "files_checked": 0, "errors": [f"invalid baseline: {error}"]}
    if not isinstance(entries, list):
        return {"passed": False, "files_checked": 0, "errors": ["baseline files must be a list"]}
    eligible = 0
    for item in entries:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            errors.append("invalid baseline file entry")
            continue
        if not item["path"].startswith(("runs/", "data/")):
            continue
        eligible += 1
        path = (ROOT / item["path"]).resolve()
        # 私有清单也只审计此检出内的资料，不允许相对路径或链接越界读取其他文件。
        if not path.is_relative_to(ROOT.resolve()):
            errors.append("private evidence path escapes repository")
            continue
        expected_digest = item.get("sha256")
        if not isinstance(expected_digest, str) or not re.fullmatch(
            r"[0-9a-fA-F]{64}", expected_digest
        ):
            errors.append("invalid SHA256: " + item["path"])
            continue
        if not path.is_file():
            errors.append("missing: " + item["path"])
            continue
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != expected_digest.lower():
            errors.append("changed: " + item["path"])
        checked += 1
    if eligible == 0:
        errors.append("private baseline contains no runs/data evidence")
    return {"passed": not errors, "files_checked": checked, "errors": errors}


def validate_layout():
    errors = []
    # runs/artifacts是作者本地证据，不是公开检出的必备目录；归档代码仍保留兼容路径。
    required = (
        "src/arm_project",
        "tests",
        "tools",
        "docs",
        "configs",
        "native/servo_protocol",
        "native/arm_driver",
        "native/apps",
        "native/tests",
        "ros2_ws/src/jixiebi_interfaces",
        "ros2_ws/src/jixiebi_hardware",
        "legacy/ros2_ws/src/jixiebi_driver",
        "legacy/ros2_ws/src/jixiebi_description",
        "legacy/ros2_ws/src/jixiebi_vision",
        "legacy/firmware/esp32",
    )
    for directory in required:
        if not (ROOT / directory).is_dir():
            errors.append("missing directory: " + directory)
    for path in ROOT.glob("*.py"):
        errors.append("Python source mixed into project root: " + path.name)
    files = [ROOT / "README.md", *(ROOT / "docs").rglob("*.md"), *(ROOT / "legacy").rglob("*.md")]
    for path in files:
        for link in re.findall(r"\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            if re.match(r"^[a-zA-Z]+:|^#|^/", link):
                continue
            target = link.split("#", 1)[0]
            if target and not (path.parent / target).exists():
                errors.append(str(path.relative_to(ROOT)) + ": broken local link " + link)
    return {"passed": not errors, "errors": errors, "markdown_files_checked": len(files)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--private-evidence",
        action="store_true",
        help="also audit author-supplied private logs; missing inputs fail acceptance",
    )
    args = parser.parse_args()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or ROOT / "artifacts" / ("acceptance_" + timestamp)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "started_at": datetime.now(timezone.utc).isoformat(),
        "project_root": str(ROOT),
        "hardware_io": False,
        "scope": "public_offline_and_private_evidence"
        if args.private_evidence
        else "public_offline",
        "private_evidence": {
            "requested": args.private_evidence,
            "status": "REQUESTED" if args.private_evidence else "NOT_RUN",
            "reason": "explicit opt-in" if args.private_evidence else "not shipped in public clone",
        },
        "checks": [],
    }
    commands = [
        ("lint", ["-m", "ruff", "check", "src", "tests", "tools", "native"]),
        ("format", ["-m", "ruff", "format", "--check", "src", "tests", "tools", "native"]),
        ("dependencies", ["-m", "pip", "check"]),
        ("tests", ["-m", "unittest", "discover", "-s", "tests", "-v"]),
        (
            "driver_dry_run",
            [
                "-m",
                "arm_project.cli.arm_cli",
                "--plan",
                str(ROOT / "data/unified_read_capture.json"),
            ],
        ),
    ]
    if not args.private_evidence:
        print("private_evidence NOT_RUN: public clone does not include private logs", flush=True)
    if args.private_evidence:
        # 私有检查只有明确选用才执行。缺少任一原始目录先报告失败，不生成伪造库存。
        capture_sources = (
            "runs/unified/unified_read_capture_20261031_01",
            "runs/unified/unified_interface_20261031_01",
        )
        missing = [source for source in capture_sources if not (ROOT / source).is_dir()]
        integrity = validate_evidence_integrity()
        report["checks"].append({"name": "evidence_integrity", **integrity})
        if not (ROOT / "artifacts/migration/before.json").is_file():
            report["private_evidence"]["status"] = "MISSING_INPUTS"
            print("evidence_integrity FAIL: private baseline is missing", flush=True)
        if missing:
            report["private_evidence"]["status"] = "MISSING_INPUTS"
            report["checks"].append({"name": "capture_audit", "passed": False, "errors": missing})
            print("capture_audit FAIL: private source directories are missing", flush=True)
        else:
            if report["private_evidence"]["status"] != "MISSING_INPUTS":
                report["private_evidence"]["status"] = "EXECUTED"
            commands.append(
                (
                    "capture_audit",
                    [
                        "-m",
                        "arm_project.cli.calibration_prepare",
                        "inventory",
                        *capture_sources,
                        "--output",
                        str(output / "capture_inventory.json"),
                    ],
                )
            )
    for name, command in commands:
        process = subprocess.run(
            [sys.executable, "-X", "utf8", *command],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            encoding="utf-8",
            errors="replace",
            timeout=180,
        )
        (output / (name + ".log")).write_text(process.stdout, encoding="utf-8")
        check = {"name": name, "passed": process.returncode == 0, "exit_code": process.returncode}
        if name == "tests":
            match = re.search(r"Ran (\d+) tests", process.stdout)
            check["test_count"] = int(match.group(1)) if match else None
        report["checks"].append(check)
        print(name, "PASS" if check["passed"] else "FAIL", flush=True)
    report["checks"].append({"name": "layout_and_links", **validate_layout()})
    report["passed"] = all(item["passed"] for item in report["checks"])
    report["finished_at"] = datetime.now(timezone.utc).isoformat()
    (output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("REPORT:", output / "report.json")
    print("PASS" if report["passed"] else "FAIL")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
