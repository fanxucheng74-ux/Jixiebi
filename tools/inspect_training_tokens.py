"""Count exact chat-template token lengths; no model weights or training framework."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
from importlib.util import find_spec
import json
import math
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = "Qwen/Qwen2.5-Coder-7B-Instruct"
SPLITS = ("train", "validation", "test")
TOKENIZER_FILES = (
    "tokenizer.json",
    "tokenizer_config.json",
    "vocab.json",
    "merges.txt",
    "special_tokens_map.json",
    "added_tokens.json",
    "config.json",
    "chat_template.jinja",
)


def file_hash(path):
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def load_records(dataset):
    records = [
        json.loads(line)
        for line in (dataset / "records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    if not records or len({r["id"] for r in records}) != len(records):
        raise ValueError("empty dataset or duplicate sample IDs")
    inputs = {"records.jsonl": file_hash(dataset / "records.jsonl")}
    for split in SPLITS:
        filename = split + ".messages.jsonl"
        path = dataset / filename
        messages = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        expected = [{"messages": row["messages"]} for row in records if row["split"] == split]
        if messages != expected or not expected:
            raise ValueError("messages export differs from records: " + filename)
        inputs[filename] = file_hash(path)
    for row in records:
        if row["split"] not in SPLITS:
            raise ValueError("unknown split")
        messages = row["messages"]
        if [m["role"] for m in messages] != ["system", "user", "assistant"]:
            raise ValueError("expected one system/user/assistant exchange")
        if any(not isinstance(m.get("content"), str) or not m["content"] for m in messages):
            raise ValueError("empty or nontext message")
    return records, inputs


def count_records(records, tokenizer):
    rows = []
    for record in records:
        full = tokenizer.apply_chat_template(
            record["messages"], tokenize=True, add_generation_prompt=False, truncation=False
        )
        prefix = tokenizer.apply_chat_template(
            record["messages"][:-1], tokenize=True, add_generation_prompt=True, truncation=False
        )
        if not isinstance(full, list) or not isinstance(prefix, list):
            raise ValueError("plain token ID lists required, without tensor backends")
        if any(type(value) is not int for value in full + prefix):
            raise ValueError("invalid token IDs")
        if not prefix or full[: len(prefix)] != prefix:
            raise ValueError("chat prefix differs for sample: " + record["id"])
        completion = full[len(prefix) :]
        if not completion:
            raise ValueError("empty assistant completion")
        rows.append(
            {
                "id": record["id"],
                "split": record["split"],
                "category": record["category"],
                "prompt_tokens": len(prefix),
                "completion_tokens_including_template_end": len(completion),
                "total_tokens": len(full),
                "exact_token_prefix_checked": True,
                "completion_contains_eos": tokenizer.eos_token_id in completion,
                "source": record["source"],
            }
        )
    return rows


def summarize(rows):
    summary = {}
    for split in SPLITS:
        selected = [row for row in rows if row["split"] == split]
        if not selected:
            raise ValueError("empty split")
        lengths = sorted(row["total_tokens"] for row in selected)
        summary[split] = {
            "count": len(selected),
            "max_prompt_tokens": max(row["prompt_tokens"] for row in selected),
            "max_completion_tokens": max(
                row["completion_tokens_including_template_end"] for row in selected
            ),
            "max_total_tokens": lengths[-1],
            "p50_total_tokens_nearest_rank": lengths[math.ceil(len(lengths) * 0.5) - 1],
            "p95_total_tokens_nearest_rank": lengths[math.ceil(len(lengths) * 0.95) - 1],
            "exceeds_sequence_length": {
                str(limit): sum(length > limit for length in lengths)
                for limit in (2048, 4096, 8192, 16384)
            },
            "category_counts": dict(Counter(row["category"] for row in selected)),
        }
    return summary


def load_tokenizer(model, revision, local_path=None, offline=False):
    if find_spec("jinja2") is None:
        raise ImportError(
            "Chat templates need Jinja2. In the training environment run: "
            'python -m pip install "jinja2>=3.1.0,<4"'
        )
    # Set before Transformers import. Tokenization needs none of these frameworks.
    os.environ["USE_TORCH"] = "0"
    os.environ["USE_TF"] = "0"
    os.environ["USE_FLAX"] = "0"
    from huggingface_hub import snapshot_download
    from transformers import AutoTokenizer

    snapshot = (
        Path(local_path)
        if local_path
        else Path(
            snapshot_download(
                repo_id=model,
                revision=revision,
                allow_patterns=list(TOKENIZER_FILES),
                local_files_only=offline,
            )
        )
    )
    tokenizer = AutoTokenizer.from_pretrained(
        str(snapshot), local_files_only=True, trust_remote_code=False
    )
    if not tokenizer.chat_template:
        raise ValueError("model tokenizer has no chat template")
    return tokenizer, {
        "requested_model": model,
        "requested_revision": revision,
        "snapshot_commit": snapshot.name if not local_path else None,
        "tokenizer_class": type(tokenizer).__name__,
        "library_versions": {
            package: version(package)
            for package in ("transformers", "tokenizers", "huggingface_hub", "Jinja2")
        },
        "eos_token": tokenizer.eos_token,
        "eos_token_id": tokenizer.eos_token_id,
        "pad_token": tokenizer.pad_token,
        "pad_token_id": tokenizer.pad_token_id,
        "chat_template_sha256": hashlib.sha256(
            str(tokenizer.chat_template).encode("utf-8")
        ).hexdigest(),
        "tokenizer_file_sha256": {
            name: file_hash(snapshot / name)
            for name in TOKENIZER_FILES
            if (snapshot / name).is_file()
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset", type=Path, default=ROOT / "data/datasets/jixiebi_code_20261101_v2"
    )
    parser.add_argument("--model", default=MODEL)
    parser.add_argument("--revision", default="main")
    parser.add_argument("--tokenizer-path", type=Path)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    dataset, output = args.dataset.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError("report already exists; choose a new output directory")
    if output == dataset or dataset in output.parents:
        raise ValueError("reports must be outside the frozen dataset")
    records, hashes = load_records(dataset)
    tokenizer, metadata = load_tokenizer(
        args.model, args.revision, args.tokenizer_path, args.offline
    )
    rows = count_records(records, tokenizer)
    summary = summarize(rows)
    if any(file_hash(dataset / name) != expected for name, expected in hashes.items()):
        raise ValueError("dataset changed during token counting")
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "tokenizer": metadata,
        "dataset_sha256": hashes,
        "split_statistics": summary,
        "longest_samples": sorted(rows, key=lambda row: row["total_tokens"], reverse=True)[:5],
        "truncation": False,
        "weights_loaded": False,
        "training_launched": False,
        "hardware_io": False,
        "note": "Counts include chat control tokens; completion includes template endings. "
        "This does not create assistant-only training labels or certify GPU memory fit.",
    }
    output.mkdir(parents=True, exist_ok=False)
    with (output / "lengths.jsonl").open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
    with (output / "report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    for split, data in summary.items():
        print(split.upper(), json.dumps(data, ensure_ascii=False))
    print("REPORT:", output / "report.json")
    print("TOKEN_CHECK_COMPLETE: no weights, no training, no truncation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
