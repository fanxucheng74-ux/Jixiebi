"""Integrity and report gates use tiny fixtures; never load ML libraries or CUDA."""

import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.check_local_qwen import REVISION, main, verify_local_model


def fixture(root):
    model = root / "model"
    model.mkdir()
    files = []
    weights = {}
    for index in range(4):
        name = f"model-{index + 1:05d}-of-00004.safetensors"
        weights[f"layer.{index}.weight"] = name
        content = bytes([index + 4]) * 16
        (model / name).write_bytes(content)
        files.append(
            {
                "path": name,
                "size_bytes": len(content),
                "lfs_sha256": hashlib.sha256(content).hexdigest(),
            }
        )
    data = {
        "config.json": {"model_type": "qwen2", "num_hidden_layers": 28},
        "model.safetensors.index.json": {"weight_map": weights},
    }
    for name, value in data.items():
        content = json.dumps(value).encode("utf-8")
        (model / name).write_bytes(content)
        files.append(
            {
                "path": name,
                "size_bytes": len(content),
                "git_blob_oid_sha1": hashlib.sha1(
                    b"blob " + str(len(content)).encode() + b"\0" + content
                ).hexdigest(),
            }
        )
    manifest = root / "manifest.json"
    manifest.write_text(
        json.dumps(
            {"repo_id": "Qwen/Qwen2.5-Coder-7B-Instruct", "revision": REVISION, "files": files}
        ),
        encoding="utf-8",
    )
    return model, manifest


class LocalQwenGateTests(unittest.TestCase):
    def test_lfs_and_git_hashes_verified_and_corruption_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            model, manifest = fixture(Path(directory))
            self.assertTrue(verify_local_model(model, manifest)["all_file_hashes_verified"])
            shard = model / "model-00001-of-00004.safetensors"
            shard.write_bytes(b"x" * shard.stat().st_size)
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                verify_local_model(model, manifest)

    def test_rejects_wrong_revision_and_traversal_before_load(self):
        with tempfile.TemporaryDirectory() as directory:
            model, manifest = fixture(Path(directory))
            original = json.loads(manifest.read_text())
            changed = dict(original, revision="other")
            manifest.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, "revision"):
                verify_local_model(model, manifest)
            original["files"][0]["path"] = "../escape.safetensors"
            manifest.write_text(json.dumps(original))
            with self.assertRaisesRegex(ValueError, "direct model files"):
                verify_local_model(model, manifest)

    def test_corrupt_input_prevents_model_loading_and_failure_is_recorded(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model, manifest = fixture(root)
            (model / "model-00004-of-00004.safetensors").unlink()
            output = root / "result.json"
            with (
                patch("tools.check_local_qwen.run_check") as load,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(
                    main(
                        [
                            "--model-dir",
                            str(model),
                            "--manifest",
                            str(manifest),
                            "--output",
                            str(output),
                        ]
                    ),
                    1,
                )
                load.assert_not_called()
            result = json.loads(output.read_text())
            self.assertEqual(result["status"], "FAIL")
            self.assertFalse(result["downloads_enabled"])
            self.assertFalse(result["training_started"])

    def test_existing_report_prevents_gpu_work_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result.json"
            output.write_bytes(b"keep prior result")
            with patch("tools.check_local_qwen.run_check") as load:
                with self.assertRaises(FileExistsError):
                    main(["--output", str(output)])
                load.assert_not_called()
            self.assertEqual(output.read_bytes(), b"keep prior result")


if __name__ == "__main__":
    unittest.main()
