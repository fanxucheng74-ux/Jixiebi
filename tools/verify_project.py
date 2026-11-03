"""Run offline repository acceptance and save an auditable JSON report.

Never opens a serial port/camera or executes a hardware plan. Generated reports
use a new directory; raw experiment files and consumed budgets are read-only.
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
    baseline = json.loads((ROOT / "artifacts/migration/before.json").read_text(encoding="utf-8"))
    checked, errors = 0, []
    for item in baseline["files"]:
        if not item["path"].startswith(("runs/", "data/")):
            continue
        path = ROOT / item["path"]
        if not path.is_file():
            errors.append("missing: " + item["path"])
            continue
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != item["sha256"]:
            errors.append("changed: " + item["path"])
        checked += 1
    return {"passed": not errors, "files_checked": checked, "errors": errors}


def validate_layout():
    errors = []
    required = ("src/arm_project", "tests", "tools", "docs", "configs", "firmware/esp32", "runs")
    for directory in required:
        if not (ROOT / directory).is_dir():
            errors.append("missing directory: " + directory)
    for path in ROOT.glob("*.py"):
        errors.append("Python source mixed into project root: " + path.name)
    files = [ROOT / "README.md", *(ROOT / "docs").rglob("*.md")]
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
    args = parser.parse_args()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = args.output or ROOT / "artifacts" / ("acceptance_" + timestamp)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "started_at": datetime.now(timezone.utc).isoformat(),
        "project_root": str(ROOT),
        "hardware_io": False,
        "checks": [],
    }
    commands = [
        ("lint", ["-m", "ruff", "check", "src", "tests", "tools"]),
        ("format", ["-m", "ruff", "format", "--check", "src", "tests", "tools"]),
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
        (
            "capture_audit",
            [
                "-m",
                "arm_project.cli.calibration_prepare",
                "inventory",
                "runs/unified/unified_read_capture_20261031_01",
                "runs/unified/unified_interface_20261031_01",
                "--output",
                str(output / "capture_inventory.json"),
            ],
        ),
    ]
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
    report["checks"].append({"name": "evidence_integrity", **validate_evidence_integrity()})
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
