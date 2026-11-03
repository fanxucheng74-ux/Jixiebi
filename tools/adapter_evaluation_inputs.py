"""Verify a selected local LoRA checkpoint using only files and the standard library."""

import hashlib
import json
import math
from pathlib import Path
import re
import stat


EXPECTED_ADAPTER_PARAMETERS = 20185088
TARGET_MODULES = {"q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"}
DATASET_FILES = {
    "records.jsonl",
    "train.messages.jsonl",
    "validation.messages.jsonl",
    "test.messages.jsonl",
}
REVISION = "c03e6d358207e414f1eca0bb1891e29f1db0e242"
POLICY = "frozen_embeddings_head_bf16_norms_fp32_v1"


def _checked_path(path, directory=False):
    path = Path(path).absolute()
    for item in (path, *path.parents):
        info = item.lstat()
        if item.is_symlink() or getattr(info, "st_file_attributes", 0) & getattr(
            stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400
        ):
            raise ValueError("linked evaluation input is not allowed")
    if directory:
        if not path.is_dir():
            raise ValueError("expected input directory")
    elif not path.is_file():
        raise ValueError("expected regular input file")
    return path.resolve(strict=True)


def _json(path):
    path = _checked_path(path)
    if path.stat().st_size > 2 * 1024 * 1024:
        raise ValueError("metadata exceeds size bound")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("metadata must be a JSON object")
    return value


def _sha256(path):
    path = _checked_path(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _finite_loss(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError("validation loss must be finite and nonnegative")
    return float(value)


def _same_loss(left, right):
    if not math.isclose(_finite_loss(left), _finite_loss(right), rel_tol=0, abs_tol=1e-12):
        raise ValueError("selected validation loss is inconsistent")


def _verify_safetensors(path, expected_count):
    path = _checked_path(path)
    with path.open("rb") as stream:
        prefix = stream.read(8)
        if len(prefix) != 8:
            raise ValueError("missing safetensors header length")
        header_length = int.from_bytes(prefix, "little")
        if not 2 <= header_length <= 1024 * 1024:
            raise ValueError("safetensors header exceeds bound")
        header_bytes = stream.read(header_length)
        if len(header_bytes) != header_length:
            raise ValueError("incomplete safetensors header")
    header = json.loads(header_bytes)
    if not isinstance(header, dict):
        raise ValueError("invalid safetensors header")
    offsets, count, pairs = [], 0, {}
    for name, tensor in header.items():
        if name == "__metadata__":
            continue
        match = re.fullmatch(r"(.+)\.(lora_A|lora_B)\.weight", name)
        if match is None or not isinstance(tensor, dict) or tensor.get("dtype") != "F32":
            raise ValueError("only FP32 LoRA A/B tensors may be evaluated")
        shape, span = tensor.get("shape"), tensor.get("data_offsets")
        if (
            not isinstance(shape, list)
            or len(shape) != 2
            or any(type(size) is not int or size <= 0 for size in shape)
            or not isinstance(span, list)
            or len(span) != 2
            or any(type(offset) is not int or offset < 0 or offset % 4 for offset in span)
        ):
            raise ValueError("invalid adapter tensor shape or offsets")
        if (match[2] == "lora_A" and shape[0] != 8) or (match[2] == "lora_B" and shape[1] != 8):
            raise ValueError("adapter tensor rank differs from reviewed rank8")
        elements = math.prod(shape)
        if span[1] - span[0] != elements * 4:
            raise ValueError("tensor byte span differs from FP32 shape")
        count += elements
        offsets.append(tuple(span))
        pairs.setdefault(match[1], set()).add(match[2])
    if (
        not offsets
        or count != expected_count
        or any(pair != {"lora_A", "lora_B"} for pair in pairs.values())
    ):
        raise ValueError("adapter inventory or parameter count differs")
    cursor = 0
    for start, end in sorted(offsets):
        if start != cursor or end <= start:
            raise ValueError("adapter payload has overlapping or noncontiguous spans")
        cursor = end
    if path.stat().st_size != 8 + header_length + cursor:
        raise ValueError("adapter payload length differs from tensor spans")
    return count


def _verify_adapter_config(config):
    if (
        config.get("peft_type") != "LORA"
        or config.get("task_type") != "CAUSAL_LM"
        or config.get("r") != 8
        or config.get("lora_alpha") != 16
        or config.get("lora_dropout") != 0
        or config.get("inference_mode") is not True
        or config.get("bias") != "none"
        or config.get("modules_to_save") is not None
        or config.get("use_dora") is not False
        or config.get("use_rslora") is not False
        or not isinstance(config.get("target_modules"), list)
        or set(config["target_modules"]) != TARGET_MODULES
    ):
        raise ValueError("adapter configuration differs from reviewed LoRA pilot")


def verify_selected_adapter(run_dir, dataset_dir):
    """Reject incomplete, altered or mismatched checkpoints before importing ML code."""
    run_dir = _checked_path(run_dir, directory=True)
    dataset_dir = _checked_path(dataset_dir, directory=True)
    report_path, selection_path = run_dir / "report.json", run_dir / "selection.json"
    report, selection = _json(report_path), _json(selection_path)
    if (
        report.get("status") != "PASS"
        or report.get("formal_pilot_training_completed") is not True
        or report.get("selection") != selection
        or selection.get("selected_with_test") is not False
        or report.get("test_examples_evaluated") != 0
    ):
        raise ValueError("completed validation-selected formal run required")
    name = selection.get("best_checkpoint")
    if type(name) is not str or re.fullmatch(r"epoch_0[1-5]", name) is None:
        raise ValueError("checkpoint must be one reviewed relative epoch directory")
    epoch = selection.get("epoch")
    if type(epoch) is not int or name != f"epoch_{epoch:02d}":
        raise ValueError("selected epoch and directory disagree")
    epochs = report.get("completed_epochs")
    if not isinstance(epochs, list) or not epochs:
        raise ValueError("formal epoch validation records are missing")
    if any(not isinstance(row, dict) for row in epochs):
        raise ValueError("invalid completed epoch record")
    matching = [
        row for row in epochs if row.get("epoch") == epoch and row.get("checkpoint") == name
    ]
    if len(matching) != 1 or len({row.get("epoch") for row in epochs}) != len(epochs):
        raise ValueError("selected epoch not uniquely completed")
    losses = [_finite_loss(row.get("validation", {}).get("token_weighted_loss")) for row in epochs]
    _same_loss(selection.get("validation_loss"), matching[0]["validation"]["token_weighted_loss"])
    _same_loss(selection["validation_loss"], min(losses))
    precision = report.get("precision", {})
    if (
        precision.get("policy") != POLICY
        or precision.get("norms_fp32_verified") is not True
        or precision.get("unrelated_parameter_dtypes_and_trainability_unchanged") is not True
        or any(
            precision.get("layers", {}).get(layer, {}).get("dtype_after") != "torch.bfloat16"
            or precision.get("layers", {}).get(layer, {}).get("frozen") is not True
            for layer in ("input_embeddings", "output_embeddings")
        )
    ):
        raise ValueError("checkpoint precision is not the reviewed frozen BF16 policy")
    model_source = report.get("model_source", {})
    if (
        model_source.get("repo_id") != "Qwen/Qwen2.5-Coder-7B-Instruct"
        or model_source.get("revision") != REVISION
        or model_source.get("all_file_hashes_verified") is not True
    ):
        raise ValueError("checkpoint base model provenance differs")
    checkpoint = _checked_path(run_dir / name, directory=True)
    manifest_path = checkpoint / "checkpoint_complete.json"
    manifest = _json(manifest_path)
    for field in ("config", "precision", "model_source", "dataset_sha256"):
        if manifest.get(field) != report.get(field):
            raise ValueError("checkpoint provenance differs from report: " + field)
    if manifest.get("epoch") != epoch or (
        manifest.get("adapter_parameter_count") != EXPECTED_ADAPTER_PARAMETERS
        or report.get("trainable_parameters") != EXPECTED_ADAPTER_PARAMETERS
    ):
        raise ValueError("checkpoint epoch or adapter parameter count differs")
    hashes = manifest.get("files_sha256")
    if (
        not isinstance(hashes, dict)
        or not {"adapter_config.json", "adapter_model.safetensors"} <= hashes.keys()
    ):
        raise ValueError("checkpoint lacks adapter integrity hashes")
    if hashes != matching[0].get("adapter_files"):
        raise ValueError("epoch adapter hashes differ from checkpoint manifest")
    for filename, expected in hashes.items():
        if (
            type(filename) is not str
            or Path(filename).name != filename
            or "/" in filename
            or "\\" in filename
            or filename in (".", "..")
            or type(expected) is not str
            or re.fullmatch(r"[a-f0-9]{64}", expected) is None
        ):
            raise ValueError("invalid direct checkpoint filename or digest")
        if _sha256(checkpoint / filename) != expected:
            raise ValueError("checkpoint hash mismatch: " + filename)
    _verify_adapter_config(_json(checkpoint / "adapter_config.json"))
    count = _verify_safetensors(
        checkpoint / "adapter_model.safetensors", EXPECTED_ADAPTER_PARAMETERS
    )
    dataset_hashes = report.get("dataset_sha256")
    if not isinstance(dataset_hashes, dict) or set(dataset_hashes) != DATASET_FILES:
        raise ValueError("frozen v2 dataset hash inventory differs")
    for filename, expected in dataset_hashes.items():
        if _sha256(dataset_dir / filename) != expected:
            raise ValueError("frozen dataset hash mismatch: " + filename)
    return {
        "checkpoint_dir": str(checkpoint),
        "selection": selection,
        "report_sha256": _sha256(report_path),
        "selection_sha256": _sha256(selection_path),
        "checkpoint_manifest_sha256": _sha256(manifest_path),
        "adapter_files_sha256": dict(hashes),
        "adapter_parameter_count": count,
        "precision": precision,
        "model_source": model_source,
        "dataset_sha256": dict(dataset_hashes),
    }
