"""Local validation baseline followed by exactly one ephemeral QLoRA optimizer step."""

import argparse
from datetime import datetime, timezone
import gc
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import sys
import time

if __package__:
    from .check_local_qwen import verify_local_model
    from .inspect_training_tokens import load_records
    from .training_examples import encode_example, summarize_validation
else:
    from check_local_qwen import verify_local_model
    from inspect_training_tokens import load_records
    from training_examples import encode_example, summarize_validation


ROOT = Path(__file__).resolve().parents[1]
TARGET_MODULES = ["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"]


def write_json(path, data):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def gpu_memory(torch):
    free, total = torch.cuda.mem_get_info(0)
    return {
        "allocated_mib": round(torch.cuda.memory_allocated(0) / 2**20, 2),
        "reserved_mib": round(torch.cuda.memory_reserved(0) / 2**20, 2),
        "peak_allocated_mib": round(torch.cuda.max_memory_allocated(0) / 2**20, 2),
        "global_free_mib": round(free / 2**20, 2),
        "global_total_mib": round(total / 2**20, 2),
    }


def tensor_batch(example, torch):
    return {
        name: torch.tensor([example[name]], dtype=torch.long, device="cuda:0")
        for name in ("input_ids", "attention_mask", "labels")
    }


def run(args, output, report, event):
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    os.environ["USE_TORCH"] = "1"
    os.environ["USE_TF"] = "0"
    os.environ["USE_FLAX"] = "0"
    records, input_hashes = load_records(args.dataset)
    report["dataset_sha256"] = input_hashes
    event("verify_model_files")
    report["model_source"] = verify_local_model(args.model_dir, args.manifest)
    import bitsandbytes as bnb
    from peft import LoraConfig, get_peft_model, prepare_model_for_kbit_training
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    torch.manual_seed(42)
    torch.cuda.set_device(0)
    torch.cuda.reset_peak_memory_stats(0)
    report["versions"] = {
        name: version(name)
        for name in ("torch", "transformers", "peft", "accelerate", "bitsandbytes")
    }
    tokenizer = AutoTokenizer.from_pretrained(
        str(args.model_dir), local_files_only=True, trust_remote_code=False
    )
    examples = [
        encode_example(record, tokenizer, max_length=4096)
        for record in records
        if record["split"] in ("train", "validation")
    ]
    training = [row for row in examples if row["split"] == "train"]
    validation = [row for row in examples if row["split"] == "validation"]
    if len(training) != 38 or len(validation) != 5:
        raise ValueError("this reviewed preflight requires frozen v2 train38/validation5")
    longest = max(training, key=lambda row: (row["sequence_tokens"], row["id"]))
    report["selected_training_example"] = {
        key: longest[key]
        for key in ("id", "split", "sequence_tokens", "prompt_tokens", "completion_tokens")
    }
    report["configured_sequence_cap"] = 4096
    report["test_examples_evaluated"] = 0
    event("load_nf4_model")
    model = AutoModelForCausalLM.from_pretrained(
        str(args.model_dir),
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
    quantized = [module for module in model.modules() if isinstance(module, bnb.nn.Linear4bit)]
    if len(quantized) != 196 or any(
        layer.weight.quant_state is None
        or layer.weight.quant_state.quant_type != "nf4"
        or not layer.weight.quant_state.nested
        for layer in quantized
    ):
        raise RuntimeError("expected all196 transformer linear layers as nested NF4")
    if any(parameter.device.type != "cuda" for parameter in model.parameters()):
        raise RuntimeError("all model weights must be on CUDA")
    model.config.use_cache = False
    report["memory_after_load"] = gpu_memory(torch)
    event("prepare_frozen_kbit_base_for_comparable_baseline")
    model = prepare_model_for_kbit_training(
        model,
        use_gradient_checkpointing=True,
        gradient_checkpointing_kwargs={"use_reentrant": False},
    )
    report["memory_after_kbit_prepare"] = gpu_memory(torch)
    model.eval()
    baseline_rows = []
    event("validation_baseline_before_adapter")
    with (output / "baseline_examples.jsonl").open("x", encoding="utf-8") as stream:
        for example in validation:
            batch = tensor_batch(example, torch)
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                prediction = model(**batch)
                loss = prediction.loss
            if not torch.isfinite(loss).item():
                raise RuntimeError("validation loss is not finite")
            row = {
                "id": example["id"],
                "sequence_tokens": example["sequence_tokens"],
                "supervised_tokens": example["completion_tokens"],
                "loss": float(loss.item()),
            }
            baseline_rows.append(row)
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            print("BASELINE", row["id"], "LOSS", round(row["loss"], 6), flush=True)
            del prediction, loss, batch
    baseline = {
        "kind": "VALIDATION_ASSISTANT_TOKEN_LOSS_BEFORE_ANY_ADAPTER",
        "summary": summarize_validation(baseline_rows),
        "dataset_sha256": input_hashes,
        "model_source": report["model_source"],
        "validation_only": True,
        "official_kbit_preparation_before_baseline": True,
        "lora_injected": False,
        "nonquantized_parameter_precision": "official_prepare_model_for_kbit_training_default",
        "functional_code_accuracy_verified": False,
    }
    write_json(output / "baseline.json", baseline)
    report["baseline_completed"] = True
    report["baseline_summary"] = baseline["summary"]
    event("prepare_lora_training")
    gc.collect()
    torch.cuda.empty_cache()
    model = get_peft_model(
        model,
        LoraConfig(
            r=8,
            lora_alpha=16,
            target_modules=TARGET_MODULES,
            lora_dropout=0.0,
            bias="none",
            task_type="CAUSAL_LM",
        ),
    )
    model.train()
    trainable = [(name, p) for name, p in model.named_parameters() if p.requires_grad]
    count = sum(parameter.numel() for _, parameter in trainable)
    if count != 20185088 or any("lora_" not in name for name, _ in trainable):
        raise RuntimeError("unexpected trainable parameters for reviewed r8 projection adapters")
    if not model.is_gradient_checkpointing or model.config.use_cache:
        raise RuntimeError("gradient checkpointing/cache configuration differs from review")
    report["lora"] = {
        "rank": 8,
        "alpha": 16,
        "target_modules": TARGET_MODULES,
        "trainable_parameters": count,
        "gradient_checkpointing": True,
        "use_reentrant": False,
        "official_kbit_preparation_unmodified": True,
    }
    report["memory_after_prepare"] = gpu_memory(torch)
    optimizer = torch.optim.AdamW([p for _, p in trainable], lr=1e-4, weight_decay=0)
    batch = tensor_batch(longest, torch)
    if int((batch["labels"][:, 1:] != -100).sum().item()) != longest["completion_tokens"]:
        raise RuntimeError("causal supervised token count differs from assistant completion")
    torch.cuda.reset_peak_memory_stats(0)
    event("longest_train_example_forward")
    report["training_step_attempted"] = True
    started = time.monotonic()
    optimizer.zero_grad(set_to_none=True)
    with torch.autocast("cuda", dtype=torch.bfloat16):
        prediction = model(**batch)
        loss = prediction.loss
    if not torch.isfinite(loss).item():
        raise RuntimeError("training loss is not finite")
    report["training_loss_before_update"] = float(loss.item())
    del prediction  # Release the full-length output logits; retain the loss graph.
    event("longest_train_example_backward")
    loss.backward()
    if any(p.grad is None or not torch.isfinite(p.grad).all().item() for _, p in trainable):
        raise RuntimeError("LoRA gradient missing or nonfinite")
    if any(p.grad is not None for p in model.parameters() if not p.requires_grad):
        raise RuntimeError("frozen base parameter unexpectedly has a gradient")
    chosen = next(
        (
            (name, p)
            for name, p in trainable
            if "lora_B" in name and torch.count_nonzero(p.grad).item()
        ),
        None,
    )
    if chosen is None:
        raise RuntimeError("no nonzero LoRA B gradient")
    changed_name, changed_parameter = chosen
    before = changed_parameter.detach().clone()
    norm = torch.nn.utils.clip_grad_norm_(
        [p for _, p in trainable], max_norm=1.0, error_if_nonfinite=True
    )
    report["gradient_norm_before_clip"] = float(norm.item())
    event("single_optimizer_step")
    report["optimizer_step_attempted"] = True
    optimizer.step()
    torch.cuda.synchronize()
    report["completed_optimizer_steps"] = 1
    if torch.equal(before, changed_parameter.detach()):
        raise RuntimeError("selected nonzero-gradient adapter did not change")
    if any(not torch.isfinite(p).all().item() for _, p in trainable):
        raise RuntimeError("adapter contains nonfinite values after update")
    report["lora_B_update_verified"] = changed_name
    report["step_seconds"] = round(time.monotonic() - started, 3)
    report["memory_after_training_step"] = gpu_memory(torch)
    for name, expected in input_hashes.items():
        if hashlib.sha256((args.dataset / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError("frozen dataset changed during preflight")
    report["dataset_unchanged"] = True
    event("preflight_completed_no_adapter_saved")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--dataset", type=Path, default=ROOT / "data/datasets/jixiebi_code_20261101_v2"
    )
    parser.add_argument(
        "--model-dir", type=Path, default=ROOT / "data/models/Qwen2.5-Coder-7B-Instruct"
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=ROOT / "artifacts/model_download_20261101_01/source_manifest.json",
    )
    args = parser.parse_args(argv)
    output = args.output_dir.resolve()
    for protected in (args.dataset.resolve(), args.model_dir.resolve()):
        if output == protected or protected in output.parents:
            raise ValueError("preflight reports must be outside frozen dataset/model directories")
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "kind": "VALIDATION_BASELINE_AND_ONE_LONGEST_TRAIN_EXAMPLE_STEP",
        "status": "STARTED",
        "baseline_completed": False,
        "training_step_attempted": False,
        "optimizer_step_attempted": False,
        "completed_optimizer_steps": 0,
        "adapter_saved": False,
        "formal_training_completed": False,
        "model_capability_improvement_verified": False,
        "robot_io": False,
        "downloads_enabled": False,
        "tool_sha256": {
            name: hashlib.sha256((ROOT / "tools" / name).read_bytes()).hexdigest()
            for name in (
                "run_code_training_preflight.py",
                "training_examples.py",
                "check_local_qwen.py",
            )
        },
    }
    with (output / "events.jsonl").open("x", encoding="utf-8") as log:

        def event(stage):
            report["last_stage"] = stage
            log.write(
                json.dumps({"stage": stage, "time": datetime.now(timezone.utc).isoformat()}) + "\n"
            )
            log.flush()
            print("STAGE:", stage, flush=True)

        try:
            run(args, output, report, event)
            report["status"] = "PASS"
        except (Exception, KeyboardInterrupt) as error:
            report.update(status="FAIL", error=type(error).__name__ + ": " + str(error))
            torch = sys.modules.get("torch")
            if torch is not None:
                try:
                    report["memory_on_failure"] = gpu_memory(torch)
                except Exception:
                    pass
        finally:
            write_json(output / "report.json", report)
    if report["status"] == "FAIL":
        print("ERROR:", report["error"])
    print("REPORT:", output / "report.json")
    print("TRAINING_PREFLIGHT_PASS" if report["status"] == "PASS" else "TRAINING_PREFLIGHT_FAIL")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
