"""Evidence exports use synthetic fixtures only; no device access."""

import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.export_motion_evidence import DEFAULT_TRIAL_IDS, export_motion_evidence


def fixture(root, trial_id, *, failed=False):
    folder = root / "runs/timed_cycles" / trial_id
    folder.mkdir(parents=True)
    command = "#003P1050T1500!"
    phase = {
        "name": "lower",
        "duration_ms": 1500,
        "targets": [{"servo_id": 3, "target_p": 1050}],
    }
    spec = {"review_id": trial_id, "phases": [phase], "deadline_ms": 20000}
    sequence = {
        "status": "FAILED_AFTER_ATTEMPT" if failed else "RESPONSE_SEQUENCE_COMPLETED",
        "response_pass": not failed,
        "within_deadline": True,
        "elapsed_s": 1.6,
        "physical_task_verified": False,
        "physical_stop_verified": False,
        "strict_position_verified": False,
        "error": "nominal margin" if failed else None,
        "phases": [
            {
                "name": "lower",
                "response_pass": not failed,
                "commands_sent": [command],
                "starts_p": [1500] * 6,
            }
        ],
    }
    result = {
        "source": "HARDWARE",
        "sequence": sequence,
        "transport_closed": True,
        "logging_errors": [],
    }
    events = [
        {"event": "cycle_phase_started", "name": "lower"},
        {"event": "motion_tx", "command": command},
        *[
            {
                "event": "reply",
                "servo_id": sid,
                "field": "position",
                "value": 1050 + sid,
                "received_at": f"2026-11-01T18:00:0{sid}+08:00",
                "received_monotonic_s": 100.0 + sid / 20,
            }
            for sid in range(6)
        ],
    ]
    for name, payload in (
        ("result.json", result),
        ("spec.json", spec),
        ("task_attempt.json", {"plan": {"commands": [[command]]}}),
    ):
        (folder / name).write_text(json.dumps(payload), encoding="utf-8")
    (folder / "events.jsonl").write_text(
        "".join(json.dumps(e) + "\n" for e in events), encoding="utf-8"
    )
    return folder


class EvidenceExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.first = fixture(self.root, DEFAULT_TRIAL_IDS[0])
        self.second = fixture(self.root, DEFAULT_TRIAL_IDS[1], failed=True)
        self.output = self.root / "export"

    def test_separate_success_and_failure_labels_and_external_observation(self):
        report = export_motion_evidence(self.root, self.output)
        rows = [
            json.loads(x)
            for x in (self.output / "trials.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual([True, False], [r["sequence_completed"] for r in rows])
        self.assertTrue(rows[0]["external_observation"]["physical_cycle_confirmed"])
        self.assertIsNone(rows[1]["external_observation"]["physical_cycle_confirmed"])
        self.assertIsNone(rows[1]["external_observation"]["object_still_held_at_stop"])
        self.assertFalse(rows[1]["external_observation"]["contact_with_table_reported"])
        self.assertTrue(all(r["raw_physical_task_verified"] is False for r in rows))
        self.assertEqual(2, report["trial_count"])
        self.assertFalse(report["hardware_opened"])
        self.assertFalse(report["video"]["trial_binding_verified"])

    def test_preserves_sequential_samples_once_and_raw_hashes(self):
        before = {p: p.read_bytes() for p in self.first.iterdir()}
        report = export_motion_evidence(self.root, self.output)
        with (self.output / "telemetry.csv").open(encoding="utf-8", newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(12, len(rows))
        self.assertEqual([str(x) for x in range(6)], [r["servo_id"] for r in rows[:6]])
        self.assertEqual({"False"}, {r["simultaneous"] for r in rows})
        self.assertEqual({"host_receive"}, {r["timestamp_origin"] for r in rows})
        self.assertEqual({"protocol_P"}, {r["position_unit"] for r in rows})
        self.assertEqual([str(x) for x in range(2, 8)], [r["source_event_index"] for r in rows[:6]])
        self.assertNotEqual(rows[0]["received_at"], rows[1]["received_at"])
        self.assertTrue(all(p.read_bytes() == data for p, data in before.items()))
        for path, digest in report["source_sha256"].items():
            self.assertEqual(digest, hashlib.sha256((self.root / path).read_bytes()).hexdigest())

    def test_refuses_existing_output_without_touching_it(self):
        self.output.mkdir()
        marker = self.output / "marker"
        marker.write_text("keep")
        with self.assertRaises(FileExistsError):
            export_motion_evidence(self.root, self.output)
        self.assertEqual("keep", marker.read_text(encoding="utf-8"))

    def test_bad_schema_fails_before_output_creation(self):
        path = self.second / "result.json"
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["sequence"].pop("phases")
        path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(ValueError, "missing executed phases"):
            export_motion_evidence(self.root, self.output)
        self.assertFalse(self.output.exists())

    def test_motion_not_in_reserved_prefix_rejected(self):
        path = self.second / "events.jsonl"
        text = path.read_text(encoding="utf-8").replace("#003P1050T1500!", "#003P2000T1500!")
        path.write_text(text)
        with self.assertRaisesRegex(ValueError, "prefix mismatch"):
            export_motion_evidence(self.root, self.output)
        self.assertFalse(self.output.exists())

    def test_missing_timestamp_rejected_instead_of_fabricated(self):
        path = self.second / "events.jsonl"
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        rows[-1].pop("received_at")
        path.write_text("".join(json.dumps(row) + "\n" for row in rows))
        with self.assertRaisesRegex(ValueError, "missing receive timestamp"):
            export_motion_evidence(self.root, self.output)
        self.assertFalse(self.output.exists())

    def test_path_traversal_and_overlapping_raw_output_rejected(self):
        with self.assertRaisesRegex(ValueError, "unsafe trial_id"):
            export_motion_evidence(self.root, self.output, ("../evil",))
        with self.assertRaisesRegex(ValueError, "overlaps raw"):
            export_motion_evidence(self.root, self.root / "runs" / "export")

    def test_inconsistent_success_rejected(self):
        path = self.first / "result.json"
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["sequence"]["within_deadline"] = False
        path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(ValueError, "inconsistent completed"):
            export_motion_evidence(self.root, self.output)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
