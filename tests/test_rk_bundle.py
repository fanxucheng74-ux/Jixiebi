"""部署包只复制白名单源码；测试不联网、不安装依赖、不访问硬件。"""

import hashlib
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

from tools.build_rk_bundle import ARCHIVE_NAME, FIXED_FILES, TOP_LEVEL, build_bundle


class RkBundleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="jixiebi-rk-bundle-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "project"
        self.root.mkdir()
        self.payloads = {relative: (relative + "\n").encode() for relative in FIXED_FILES}
        self.payloads.update(
            {
                "src/arm_project/__init__.py": b'"""Fixture package."""\n',
                "src/arm_project/cli/read_arm_state.py": b"VALUE = 1\n",
            }
        )
        for relative, payload in self.payloads.items():
            self.write(relative, payload)
        # 故意放入多个危险/无关输入，验收依据是归档实际成员，而不是排除表达式本身。
        for relative in (
            ".env",
            ".venv/private.txt",
            "artifacts/private.json",
            "runs/raw.jsonl",
            "data/models/weights.safetensors",
            "data/private.jpg",
            "configs/wifi.private.json",
            "src/arm_project/__pycache__/secret.pyc",
            "src/arm_project/not_source.txt",
        ):
            self.write(relative, b"DO_NOT_BUNDLE")

    def write(self, relative, payload):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        return path

    def test_archive_matches_whitelist_and_manifest_hashes(self):
        result = build_bundle(self.root / "out", self.root)
        manifest = json.loads(Path(result["manifest"]).read_text(encoding="utf-8"))
        archive_path = Path(result["archive"])
        self.assertEqual(archive_path.name, ARCHIVE_NAME)
        self.assertFalse(manifest["hardware_io"])
        self.assertEqual(manifest["archive_size_bytes"], archive_path.stat().st_size)
        self.assertEqual(
            manifest["archive_sha256"], hashlib.sha256(archive_path.read_bytes()).hexdigest()
        )
        self.assertEqual(manifest["file_count"], len(self.payloads))
        records = {item["path"]: item for item in manifest["files"]}
        self.assertEqual(set(records), set(self.payloads))
        with tarfile.open(archive_path, "r:gz") as archive:
            members = archive.getmembers()
            self.assertEqual(len(members), len(self.payloads))
            self.assertEqual(
                {member.name for member in members},
                {TOP_LEVEL + "/" + relative for relative in self.payloads},
            )
            for member in members:
                self.assertTrue(member.isfile())
                self.assertFalse(member.issym() or member.islnk())
                self.assertNotIn("..", Path(member.name).parts)
                self.assertNotIn("\\", member.name)
                self.assertFalse(member.name.startswith("/"))
                relative = member.name.removeprefix(TOP_LEVEL + "/")
                payload = archive.extractfile(member).read()
                self.assertEqual(payload, self.payloads[relative])
                self.assertEqual(records[relative]["size_bytes"], len(payload))
                self.assertEqual(records[relative]["sha256"], hashlib.sha256(payload).hexdigest())
                self.assertEqual(member.mode, 0o644)
                self.assertEqual(member.uid, 0)
                self.assertEqual(member.uname, "")

    def test_existing_output_is_never_overwritten(self):
        output = self.root / "out"
        result = build_bundle(output, self.root)
        before = Path(result["archive"]).read_bytes()
        with self.assertRaises(FileExistsError):
            build_bundle(output, self.root)
        self.assertEqual(Path(result["archive"]).read_bytes(), before)

    def test_missing_required_source_creates_no_output(self):
        (self.root / ".arm-project-root").unlink()
        output = self.root / "out"
        with self.assertRaises(FileNotFoundError):
            build_bundle(output, self.root)
        self.assertFalse(output.exists())

    def test_relative_output_is_based_on_project_not_shell_directory(self):
        result = build_bundle(Path("out"), self.root)
        self.assertEqual(Path(result["archive"]).parent, self.root.resolve() / "out")

    def test_identical_inputs_produce_identical_archives(self):
        first = build_bundle(self.root / "out-one", self.root)
        second = build_bundle(self.root / "out-two", self.root)
        self.assertEqual(first["archive_sha256"], second["archive_sha256"])
        self.assertEqual(Path(first["archive"]).read_bytes(), Path(second["archive"]).read_bytes())

    def test_symlink_source_is_rejected_before_output_creation(self):
        outside = Path(self.temporary.name) / "outside.py"
        outside.write_text("SECRET = True\n", encoding="utf-8")
        linked = self.root / "src/arm_project/linked.py"
        try:
            linked.symlink_to(outside)
        except OSError as error:
            self.skipTest("platform does not permit fixture symlinks: " + str(error))
        with self.assertRaisesRegex(ValueError, "links"):
            build_bundle(self.root / "out", self.root)
        self.assertFalse((self.root / "out").exists())


if __name__ == "__main__":
    unittest.main()
