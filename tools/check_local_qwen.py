"""Explicit local NF4 model load and short inference; never downloads or trains."""

import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import time


ROOT = Path(__file__).resolve().parents[1]
REVISION = "c03e6d358207e414f1eca0bb1891e29f1db0e242"


def verify_local_model(model_dir, manifest_path):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest["repo_id"] != "Qwen/Qwen2.5-Coder-7B-Instruct" or manifest["revision"] != REVISION:
        raise ValueError("model source manifest differs from the reviewed revision")
    verified, weights = [], set()
    for item in manifest["files"]:
        name = item["path"]
        if Path(name).name != name or "/" in name or "\\" in name:
            raise ValueError("only reviewed direct model files are allowed")
        path = model_dir / name
        if not path.is_file() or path.stat().st_size != item["size_bytes"]:
            raise ValueError("model file missing or incomplete: " + name)
        lfs_hash = item.get("lfs_sha256")
        hasher = hashlib.sha256() if lfs_hash else hashlib.sha1()
        if not lfs_hash:
            hasher.update(b"blob " + str(item["size_bytes"]).encode() + b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                hasher.update(block)
        if hasher.hexdigest() != (lfs_hash or item["git_blob_oid_sha1"]):
            raise ValueError("model file hash mismatch: " + name)
        verified.append(name)
        if name.endswith(".safetensors"):
            weights.add(name)
    index = json.loads((model_dir / "model.safetensors.index.json").read_text(encoding="utf-8"))
    if len(weights) != 4 or set(index["weight_map"].values()) != weights:
        raise ValueError("weight index differs from verified shards")
    config = json.loads((model_dir / "config.json").read_text(encoding="utf-8"))
    if config.get("model_type") != "qwen2" or config.get("num_hidden_layers") != 28:
        raise ValueError("unexpected Qwen model architecture")
    return {
        "repo_id": manifest["repo_id"],
        "revision": manifest["revision"],
        "verified_files": verified,
        "source_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "all_file_hashes_verified": True,
    }


def run_check(model_dir, provenance):
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    os.environ["USE_TORCH"] = "1"
    os.environ["USE_TF"] = "0"
    os.environ["USE_FLAX"] = "0"
    import bitsandbytes as bnb
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    torch.cuda.set_device(0)
    torch.cuda.reset_peak_memory_stats(0)

    def memory():
        free, total = torch.cuda.mem_get_info(0)
        return {
            "allocated_mib": round(torch.cuda.memory_allocated(0) / 2**20, 2),
            "reserved_mib": round(torch.cuda.memory_reserved(0) / 2**20, 2),
            "peak_allocated_mib": round(torch.cuda.max_memory_allocated(0) / 2**20, 2),
            "global_free_mib": round(free / 2**20, 2),
            "global_total_mib": round(total / 2**20, 2),
        }

    started = time.monotonic()
    before = memory()
    tokenizer = AutoTokenizer.from_pretrained(
        str(model_dir), local_files_only=True, trust_remote_code=False
    )
    print("LOCAL_FILE_HASHES_PASS; loading four shards as NF4 on CUDA", flush=True)
    model = AutoModelForCausalLM.from_pretrained(
        str(model_dir),
        local_files_only=True,
        trust_remote_code=False,
        use_safetensors=True,
        device_map={"": 0},
        dtype=torch.bfloat16,
        attn_implementation="sdpa",
        quantization_config=BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_use_double_quant=True,
            bnb_4bit_compute_dtype=torch.bfloat16,
        ),
    )
    torch.cuda.synchronize()
    after_load = memory()
    load_seconds = time.monotonic() - started
    quantized = [module for module in model.modules() if isinstance(module, bnb.nn.Linear4bit)]
    if not getattr(model, "is_loaded_in_4bit", False) or len(quantized) != 196:
        raise RuntimeError("expected 196 quantized Qwen transformer linear layers")
    if any(
        layer.weight.quant_state is None
        or layer.weight.quant_state.quant_type != "nf4"
        or not layer.weight.quant_state.nested
        for layer in quantized
    ):
        raise RuntimeError("NF4/double-quant state is incomplete")
    if any(p.device.type != "cuda" for p in model.parameters()):
        raise RuntimeError("not all model weights were placed on CUDA")
    # This new, short prompt is only an inference smoke check, not a held-out eval.
    messages = [
        {"role": "system", "content": "你是Python编程助手。"},
        {"role": "user", "content": "请只输出一个Python函数add(a, b)，返回a+b，不要解释。"},
    ]
    inputs = tokenizer.apply_chat_template(
        messages,
        tokenize=True,
        add_generation_prompt=True,
        return_dict=True,
        return_tensors="pt",
    ).to("cuda:0")
    model.eval()
    started = time.monotonic()
    with torch.inference_mode():
        logits = model(**inputs, use_cache=False).logits
        if not torch.isfinite(logits).all().item():
            raise RuntimeError("short forward pass produced nonfinite logits")
        del logits
        generated = model.generate(
            **inputs,
            max_new_tokens=96,
            do_sample=False,
            temperature=None,
            top_p=None,
            top_k=None,
            eos_token_id=model.generation_config.eos_token_id,
            pad_token_id=tokenizer.pad_token_id,
        )
    torch.cuda.synchronize()
    inference_seconds = time.monotonic() - started
    suffix = generated[0, inputs["input_ids"].shape[-1] :]
    answer = tokenizer.decode(suffix, skip_special_tokens=True)
    if suffix.numel() == 0 or not answer.strip():
        raise RuntimeError("short generation produced no answer")
    return {
        "provenance": provenance,
        "versions": {
            name: version(name) for name in ("torch", "transformers", "accelerate", "bitsandbytes")
        },
        "gpu": torch.cuda.get_device_name(0),
        "quantized_linear_layers": len(quantized),
        "all_transformer_linear_layers_nf4_double_quant": True,
        "all_parameters_on_cuda": True,
        "short_forward_logits_finite": True,
        "memory_before": before,
        "memory_after_load": after_load,
        "memory_after_short_inference": memory(),
        "model_parameter_buffer_storage_bytes": model.get_memory_footprint(),
        "load_seconds": round(load_seconds, 3),
        "short_inference_seconds": round(inference_seconds, 3),
        "prompt": messages,
        "generated_text": answer,
        "generated_token_count": suffix.numel(),
        "generated_text_executed": False,
        "code_correctness_verified": False,
        "formal_heldout_baseline": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-dir", type=Path, default=ROOT / "data/models/Qwen2.5-Coder-7B-Instruct"
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=ROOT / "artifacts/model_download_20261101_01/source_manifest.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "STARTED",
        "kind": "LOCAL_QWEN_NF4_LOAD_AND_SHORT_INFERENCE",
        "downloads_enabled": False,
        "training_started": False,
        "training_4096_memory_fit_verified": False,
        "robot_io": False,
    }
    with args.output.open("x", encoding="utf-8") as stream:
        try:
            provenance = verify_local_model(args.model_dir.resolve(), args.manifest)
            report["check"] = run_check(args.model_dir.resolve(), provenance)
            report["status"] = "PASS"
        except (Exception, KeyboardInterrupt) as error:
            report.update(status="FAIL", error=type(error).__name__ + ": " + str(error))
        finally:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    if report["status"] == "PASS":
        print("GENERATED_TEXT:", report["check"]["generated_text"])
        print("GPU_MEMORY:", json.dumps(report["check"]["memory_after_short_inference"]))
    else:
        print("ERROR:", report["error"])
    print("REPORT:", args.output.resolve())
    print("QWEN_4BIT_LOAD_PASS" if report["status"] == "PASS" else "QWEN_4BIT_LOAD_FAIL")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
