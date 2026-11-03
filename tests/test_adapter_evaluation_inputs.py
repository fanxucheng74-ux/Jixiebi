"""Checkpoint selection and integrity gates use small files, never an ML runtime."""

import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.adapter_evaluation_inputs import (
    DATASET_FILES,
    POLICY,
    REVISION,
    TARGET_MODULES,
    _verify_safetensors,
    verify_selected_adapter,
)


def put_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tensor_file(path, dtype="F32", start=0):
    header = {
        "base.layer.q_proj.lora_A.weight": {
            "dtype": dtype,
            "shape": [8, 1],
            "data_offsets": [start, start + 32],
        },
        "base.layer.q_proj.lora_B.weight": {
            "dtype": "F32",
            "shape": [1, 8],
            "data_offsets": [32, 64],
        },
    }
    raw = json.dumps(header).encode("utf-8")
    path.write_bytes(len(raw).to_bytes(8, "little") + raw + bytes(64))


def fixture(root):
    run, data = root / "run", root / "data"
    checkpoint = run / "epoch_02"
    checkpoint.mkdir(parents=True)
    data.mkdir()
    for name in DATASET_FILES:
        (data / name).write_bytes(b"frozen fixture\n")
    config = {
        "peft_type": "LORA",
        "task_type": "CAUSAL_LM",
        "r": 8,
        "lora_alpha": 16,
        "lora_dropout": 0.0,
        "inference_mode": True,
        "bias": "none",
        "modules_to_save": None,
        "use_dora": False,
        "use_rslora": False,
        "target_modules": sorted(TARGET_MODULES),
    }
    put_json(checkpoint / "adapter_config.json", config)
    tensor_file(checkpoint / "adapter_model.safetensors")
    hashes = {
        name: digest(checkpoint / name)
        for name in ("adapter_config.json", "adapter_model.safetensors")
    }
    selection = {
        "best_checkpoint": "epoch_02",
        "epoch": 2,
        "validation_loss": 0.7,
        "selected_with_test": False,
    }
    report = {
        "status": "PASS",
        "formal_pilot_training_completed": True,
        "test_examples_evaluated": 0,
        "selection": selection,
        "trainable_parameters": 16,
        "config": {"epochs": 2, "compute_dtype": "bfloat16_autocast"},
        "precision": {
            "policy": POLICY,
            "norms_fp32_verified": True,
            "unrelated_parameter_dtypes_and_trainability_unchanged": True,
            "layers": {
                label: {"dtype_after": "torch.bfloat16", "frozen": True}
                for label in ("input_embeddings", "output_embeddings")
            },
        },
        "model_source": {
            "repo_id": "Qwen/Qwen2.5-Coder-7B-Instruct",
            "revision": REVISION,
            "all_file_hashes_verified": True,
        },
        "dataset_sha256": {name: digest(data / name) for name in DATASET_FILES},
        "completed_epochs": [
            {"epoch": 1, "checkpoint": "epoch_01", "validation": {"token_weighted_loss": 0.9}},
            {
                "epoch": 2,
                "checkpoint": "epoch_02",
                "validation": {"token_weighted_loss": 0.7},
                "adapter_files": hashes,
            },
        ],
        # These intentionally retain WSL metadata paths on a Windows test host.
        "saved_checkpoints": ["/project/Jixiebi/run/epoch_01", "/project/Jixiebi/run/epoch_02"],
    }
    manifest = {
        field: report[field] for field in ("config", "precision", "model_source", "dataset_sha256")
    }
    manifest.update(epoch=2, adapter_parameter_count=16, files_sha256=hashes)
    put_json(run / "report.json", report)
    put_json(run / "selection.json", selection)
    put_json(checkpoint / "checkpoint_complete.json", manifest)
    return run, data, checkpoint, report, selection, manifest


class AdapterEvaluationInputTests(unittest.TestCase):
    def verify_fixture(self, run, data):
        # The production contract stays 20,185,088; fixtures use two 8-element tensors.
        with patch("tools.adapter_evaluation_inputs.EXPECTED_ADAPTER_PARAMETERS", 16):
            return verify_selected_adapter(run, data)

    def test_verified_native_checkpoint_selection_and_all_input_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            run, data, checkpoint, _, _, _ = fixture(Path(directory))
            result = self.verify_fixture(run, data)
            self.assertEqual(result["checkpoint_dir"], str(checkpoint.resolve()))
            self.assertEqual(result["adapter_parameter_count"], 16)
            self.assertEqual(set(result["dataset_sha256"]), DATASET_FILES)
            self.assertEqual(result["selection"]["epoch"], 2)
            self.assertEqual(result["report_sha256"], digest(run / "report.json"))

    def test_incomplete_or_test_selected_training_is_rejected(self):
        mutations = (
            {"status": "FAIL"},
            {"formal_pilot_training_completed": False},
            {"test_examples_evaluated": 1},
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                run, data, _, report, _, _ = fixture(Path(directory))
                report.update(mutation)
                put_json(run / "report.json", report)
                with self.assertRaisesRegex(ValueError, "formal run"):
                    self.verify_fixture(run, data)
        with tempfile.TemporaryDirectory() as directory:
            run, data, _, report, selection, _ = fixture(Path(directory))
            selection["selected_with_test"] = True
            report["selection"] = selection
            put_json(run / "selection.json", selection)
            put_json(run / "report.json", report)
            with self.assertRaisesRegex(ValueError, "formal run"):
                self.verify_fixture(run, data)

    def test_checkpoint_traversal_and_nonminimum_loss_are_rejected(self):
        for name in ("../epoch_02", "/tmp/epoch_02", "C:\\epoch_02"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                run, data, _, report, selection, _ = fixture(Path(directory))
                selection["best_checkpoint"] = name
                report["selection"] = selection
                put_json(run / "selection.json", selection)
                put_json(run / "report.json", report)
                with self.assertRaisesRegex(ValueError, "relative epoch"):
                    self.verify_fixture(run, data)
        with tempfile.TemporaryDirectory() as directory:
            run, data, _, report, _, _ = fixture(Path(directory))
            report["completed_epochs"][0]["validation"]["token_weighted_loss"] = 0.6
            put_json(run / "report.json", report)
            with self.assertRaisesRegex(ValueError, "validation loss"):
                self.verify_fixture(run, data)

    def test_changed_adapter_or_dataset_payload_is_rejected(self):
        for target in ("adapter", "dataset"):
            with self.subTest(target=target), tempfile.TemporaryDirectory() as directory:
                run, data, checkpoint, _, _, _ = fixture(Path(directory))
                changed = (
                    checkpoint / "adapter_model.safetensors"
                    if target == "adapter"
                    else data / "test.messages.jsonl"
                )
                changed.write_bytes(changed.read_bytes() + b"changed")
                with self.assertRaisesRegex(ValueError, "hash mismatch"):
                    self.verify_fixture(run, data)

    def test_manifest_provenance_and_reviewed_precision_are_rejected_on_change(self):
        with tempfile.TemporaryDirectory() as directory:
            run, data, checkpoint, _, _, manifest = fixture(Path(directory))
            manifest["epoch"] = 1
            put_json(checkpoint / "checkpoint_complete.json", manifest)
            with self.assertRaisesRegex(ValueError, "epoch or adapter"):
                self.verify_fixture(run, data)
        with tempfile.TemporaryDirectory() as directory:
            run, data, _, report, _, _ = fixture(Path(directory))
            report["precision"]["norms_fp32_verified"] = False
            put_json(run / "report.json", report)
            with self.assertRaisesRegex(ValueError, "precision"):
                self.verify_fixture(run, data)

    def test_unsupported_adapter_config_is_rejected_even_with_fresh_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            run, data, checkpoint, report, _, manifest = fixture(Path(directory))
            config = json.loads((checkpoint / "adapter_config.json").read_text())
            config["use_dora"] = True
            put_json(checkpoint / "adapter_config.json", config)
            manifest["files_sha256"]["adapter_config.json"] = digest(
                checkpoint / "adapter_config.json"
            )
            report["completed_epochs"][1]["adapter_files"] = manifest["files_sha256"]
            put_json(run / "report.json", report)
            put_json(checkpoint / "checkpoint_complete.json", manifest)
            with self.assertRaisesRegex(ValueError, "configuration"):
                self.verify_fixture(run, data)

    def test_safetensors_invalid_dtype_offsets_count_and_payload_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "weights.safetensors"
            tensor_file(path)
            self.assertEqual(_verify_safetensors(path, 16), 16)
            with self.assertRaisesRegex(ValueError, "parameter count"):
                _verify_safetensors(path, 17)
            tensor_file(path, dtype="F16")
            with self.assertRaisesRegex(ValueError, "FP32"):
                _verify_safetensors(path, 16)
            tensor_file(path, start=4)
            with self.assertRaisesRegex(ValueError, "noncontiguous"):
                _verify_safetensors(path, 16)
            tensor_file(path)
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaisesRegex(ValueError, "payload length"):
                _verify_safetensors(path, 16)

    def test_linked_checkpoint_files_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, data, checkpoint, _, _, _ = fixture(root)
            original = checkpoint / "adapter_model.safetensors"
            # Windows requires an unrelated privilege to create actual symlinks.
            # Simulate the filesystem flag while exercising the real path gate.
            with patch.object(
                Path,
                "is_symlink",
                autospec=True,
                side_effect=lambda path: os.path.samefile(path, original),
            ):
                with self.assertRaisesRegex(ValueError, "linked"):
                    self.verify_fixture(run, data)

    def test_actual_linked_checkpoint_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, data, checkpoint, _, _, _ = fixture(root)
            original = checkpoint / "adapter_model.safetensors"
            outside = root / "outside.safetensors"
            outside.write_bytes(original.read_bytes())
            original.unlink()
            try:
                original.symlink_to(outside)
            except OSError as error:
                self.skipTest(f"creating an actual symlink is unavailable: {error}")
            with self.assertRaisesRegex(ValueError, "linked"):
                self.verify_fixture(run, data)


if __name__ == "__main__":
    unittest.main()
