"""公开验收的私有审计边界：仅用临时合成文件，不依赖作者日志。"""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools import verify_project


class PrivateEvidenceAuditTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="jixiebi-public-audit-")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name).resolve()
        self.assertTrue(self.folder.is_relative_to(Path(tempfile.gettempdir()).resolve()))
        self.root = self.folder / "repository"
        self.root.mkdir()
        root_patch = patch.object(verify_project, "ROOT", self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)

    def write_baseline(self, entries):
        path = self.root / "artifacts/migration/before.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"files": entries}), encoding="utf-8")

    def test_missing_baseline_is_not_a_passed_audit(self):
        result = verify_project.validate_evidence_integrity()
        self.assertFalse(result["passed"])
        self.assertEqual(result["files_checked"], 0)
        self.assertTrue(any("baseline" in error for error in result["errors"]))

    def test_empty_baseline_is_not_a_passed_audit(self):
        self.write_baseline([])
        result = verify_project.validate_evidence_integrity()
        self.assertFalse(result["passed"])
        self.assertEqual(result["files_checked"], 0)
        self.assertTrue(result["errors"])

    def test_escaping_runs_path_is_rejected_before_external_file_read(self):
        # 外部文件仍是本测试的合成资料，位于临时仓库之外、临时目录之内。
        outside = self.folder / "outside.txt"
        content = b"synthetic external evidence must not be read"
        outside.write_bytes(content)
        self.write_baseline(
            [{"path": "runs/../../outside.txt", "sha256": hashlib.sha256(content).hexdigest()}]
        )
        original_open = Path.open

        def guarded_open(path, *args, **kwargs):
            if path.resolve() == outside:
                raise AssertionError("auditor attempted to read outside its repository")
            return original_open(path, *args, **kwargs)

        # 若越界检查被删除，审计器会真正尝试读取外部文件，本测试会立即失败。
        with patch.object(Path, "open", guarded_open):
            result = verify_project.validate_evidence_integrity()
        self.assertFalse(result["passed"])
        self.assertEqual(result["files_checked"], 0)
        self.assertTrue(any("escapes repository" in error for error in result["errors"]))

    def test_matching_synthetic_file_and_hash_pass(self):
        content = b"synthetic private evidence only"
        path = self.root / "data/sample.bin"
        path.parent.mkdir()
        path.write_bytes(content)
        self.write_baseline(
            [{"path": "data/sample.bin", "sha256": hashlib.sha256(content).hexdigest()}]
        )
        result = verify_project.validate_evidence_integrity()
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(result["files_checked"], 1)
        self.assertEqual(result["errors"], [])

    def test_changed_file_fails_the_recorded_hash(self):
        original = b"recorded synthetic content"
        path = self.root / "data/sample.bin"
        path.parent.mkdir()
        path.write_bytes(b"changed synthetic content")
        self.write_baseline(
            [{"path": "data/sample.bin", "sha256": hashlib.sha256(original).hexdigest()}]
        )
        result = verify_project.validate_evidence_integrity()
        self.assertFalse(result["passed"])
        self.assertEqual(result["files_checked"], 1)
        self.assertTrue(any("changed:" in error for error in result["errors"]))


if __name__ == "__main__":
    unittest.main()
