"""学习包的白名单、可重复打包和路径边界检查，不依赖本机实验资料。"""

import hashlib
import importlib.util
import json
from pathlib import Path
import tarfile
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "build_learning_bundle.py"
SPEC = importlib.util.spec_from_file_location("learning_bundle_under_test", MODULE_PATH)
bundle = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bundle)


class LearningBundleTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix="jixiebi-learning-bundle-")
        self.addCleanup(self.folder.cleanup)
        self.base = Path(self.folder.name)
        self.root = self.base / "repo"
        self.root.mkdir()
        for relative in bundle.BUNDLE_FILES:
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(f"// public synthetic fixture: {relative}\n", encoding="utf-8")

    def make_symlink(self, source: Path, target: Path, *, is_directory=False):
        try:
            source.symlink_to(target, target_is_directory=is_directory)
        except (NotImplementedError, OSError) as error:
            self.skipTest(f"本机没有创建符号链接的权限: {type(error).__name__}")

    def test_archive_contains_only_whitelisted_source_and_matching_hashes(self):
        # 包外资料即使在仓库中也不得被扫描加入，使用合成标记避免真实秘密进入测试。
        private_log = self.root / "runs" / "private.log"
        private_log.parent.mkdir()
        private_log.write_text("PRIVATE_FIXTURE_NOT_FOR_PACKAGE", encoding="utf-8")
        output = self.root / "artifacts" / "first"
        manifest = bundle.build_bundle(self.root, output)
        archive = output / bundle.ARCHIVE_NAME
        self.assertEqual(len(manifest["files"]), 24)
        self.assertEqual(
            manifest["archive_sha256"], hashlib.sha256(archive.read_bytes()).hexdigest()
        )
        with tarfile.open(archive, "r:gz") as package:
            self.assertEqual(package.getnames(), list(bundle.BUNDLE_FILES))
            for item, expected in zip(package.getmembers(), manifest["files"]):
                self.assertTrue(item.isfile())
                content = package.extractfile(item).read()
                self.assertEqual(content, (self.root / item.name).read_bytes())
                self.assertEqual(expected["sha256"], hashlib.sha256(content).hexdigest())
        self.assertEqual(
            (output / "SHA256SUMS").read_text(encoding="utf-8"),
            f"{manifest['archive_sha256']}  {bundle.ARCHIVE_NAME}\n",
        )
        self.assertEqual(
            json.loads((output / "manifest.json").read_text(encoding="utf-8")), manifest
        )
        self.assertEqual(private_log.read_text(encoding="utf-8"), "PRIVATE_FIXTURE_NOT_FOR_PACKAGE")

    def test_same_source_produces_identical_archive_without_machine_metadata(self):
        first = self.root / "artifacts" / "first"
        second = self.root / "artifacts" / "second"
        bundle.build_bundle(self.root, first)
        bundle.build_bundle(self.root, second)
        self.assertEqual(
            (first / bundle.ARCHIVE_NAME).read_bytes(), (second / bundle.ARCHIVE_NAME).read_bytes()
        )

    def test_missing_source_fails_before_creating_output(self):
        (self.root / bundle.BUNDLE_FILES[-1]).unlink()
        output = self.root / "artifacts" / "missing"
        with self.assertRaises(FileNotFoundError):
            bundle.build_bundle(self.root, output)
        self.assertFalse(output.exists())

    def test_existing_output_is_preserved_without_overwrite(self):
        output = self.root / "artifacts" / "existing"
        bundle.build_bundle(self.root, output)
        original = {path.name: path.read_bytes() for path in output.iterdir()}
        with self.assertRaises(FileExistsError):
            bundle.build_bundle(self.root, output)
        self.assertEqual({path.name: path.read_bytes() for path in output.iterdir()}, original)

    def test_output_outside_artifacts_is_rejected(self):
        for output in (self.base / "outside", self.root / "build" / "wrong", Path("artifacts")):
            with self.subTest(output=str(output)):
                with self.assertRaises(ValueError):
                    bundle.build_bundle(self.root, output)
                self.assertFalse((output if output.is_absolute() else self.root / output).exists())

    def test_source_symlink_to_outside_repository_is_rejected(self):
        outside = self.base / "outside-source.txt"
        outside.write_text("OUTSIDE_FIXTURE", encoding="utf-8")
        source = self.root / bundle.BUNDLE_FILES[0]
        source.unlink()
        self.make_symlink(source, outside)
        output = self.root / "artifacts" / "linked-input"
        with self.assertRaises(ValueError):
            bundle.build_bundle(self.root, output)
        self.assertFalse(output.exists())

    def test_output_parent_symlink_cannot_escape_repository(self):
        outside = self.base / "outside-output"
        outside.mkdir()
        artifacts = self.root / "artifacts"
        artifacts.mkdir()
        self.make_symlink(artifacts / "redirect", outside, is_directory=True)
        output = artifacts / "redirect" / "new-bundle"
        with self.assertRaises(ValueError):
            bundle.build_bundle(self.root, output)
        self.assertEqual(list(outside.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
