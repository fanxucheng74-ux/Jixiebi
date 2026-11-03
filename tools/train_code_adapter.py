"""Train a finite local QLoRA pilot; save only adapters and select with validation."""

import argparse
from datetime import datetime, timezone
import gc
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

if __package__:
    from .check_local_qwen import verify_local_model
    from .code_training_diagnostics import training_operation
    from .code_training_precision import set_frozen_large_layers_bf16
    from .code_training_schedule import group_supervised_tokens, make_epoch_groups
    from .inspect_training_tokens import load_records
    from .run_code_training_preflight import TARGET_MODULES, gpu_memory, tensor_batch, write_json
    from .training_examples import encode_example, summarize_validation
else:
    from check_local_qwen import verify_local_model
    from code_training_diagnostics import training_operation
    from code_training_precision import set_frozen_large_layers_bf16
    from code_training_schedule import group_supervised_tokens, make_epoch_groups
    from inspect_training_tokens import load_records
    from run_code_training_preflight import TARGET_MODULES, gpu_memory, tensor_batch, write_json
    from training_examples import encode_example, summarize_validation


ROOT = Path(__file__).resolve().parents[1]


def validate_args(args):
    if args.diagnostic_steps not in (0, 1, 2):
        raise ValueError("diagnostic_steps must be 0 (formal), 1 or 2")
    if not 1 <= args.epochs <= 5 or not 1 <= args.accumulation_steps <= 16:
        raise ValueError("epochs1..5 and accumulation_steps1..16 required")
    if not math.isfinite(args.learning_rate) or not 1e-6 <= args.learning_rate <= 1e-3:
        raise ValueError("learning rate outside reviewed pilot limits")
    for protected in (args.dataset.resolve(), args.model_dir.resolve()):
        if args.output_dir.resolve() == protected or protected in args.output_dir.resolve().parents:
            raise ValueError("training output cannot be inside frozen data or base model")


def evaluate(model, examples, torch, output_file, event, label):
    model.eval()
    rows = []
    with output_file.open("x", encoding="utf-8") as stream:
        for example in examples:
            batch = tensor_batch(example, torch)
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                prediction = model(**batch)
                loss = prediction.loss
            if not torch.isfinite(loss).item():
                raise RuntimeError("validation loss is nonfinite")
            row = {
                "id": example["id"],
                "loss": float(loss.item()),
                "supervised_tokens": example["completion_tokens"],
                "sequence_tokens": example["sequence_tokens"],
            }
            rows.append(row)
            stream.write(json.dumps(row, allow_nan=False) + "\n")
            stream.flush()
            del prediction, loss, batch
    summary = summarize_validation(rows)
    event(label, summary=summary)
    return summary


def save_checkpoint(model, trainable, directory, metadata, torch):
    if not hasattr(model, "peft_config"):
        raise RuntimeError("only PEFT adapter checkpoints may be saved")
    if any(not torch.isfinite(p).all().item() for _, p in trainable):
        raise RuntimeError("adapter contains nonfinite values")
    directory.mkdir(exist_ok=False)
    model.save_pretrained(directory, safe_serialization=True, save_embedding_layers=False)
    from safetensors import safe_open

    weights = directory / "adapter_model.safetensors"
    with safe_open(str(weights), framework="pt", device="cpu") as saved:
        names = list(saved.keys())
        if not names or any("lora_A" not in name and "lora_B" not in name for name in names):
            raise RuntimeError("checkpoint unexpectedly contains nonadapter weights")
        count = sum(math.prod(saved.get_slice(name).get_shape()) for name in names)
    if count != sum(p.numel() for _, p in trainable):
        raise RuntimeError("saved adapter parameter count differs from model")
    metadata = dict(metadata, adapter_parameter_count=count)
    metadata["files_sha256"] = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in directory.iterdir()
        if path.is_file()
    }
    write_json(directory / "checkpoint_complete.json", metadata)
    return metadata


def run(args, output, report, event):
    for name, value in {
        "HF_HUB_OFFLINE": "1",
        "TRANSFORMERS_OFFLINE": "1",
        "USE_TORCH": "1",
        "USE_TF": "0",
        "USE_FLAX": "0",
    }.items():
        os.environ[name] = value
    if args.diagnostic_steps:
        # Set before any ML framework import; never change host driver settings.
        os.environ["CUDA_LAUNCH_BLOCKING"] = "1"
    records, hashes = load_records(args.dataset)
    report["dataset_sha256"] = hashes
    event("verify_local_model")
    source = verify_local_model(args.model_dir, args.manifest)
    report["model_source"] = source
    import bitsandbytes as bnb
    from peft import LoraConfig, get_peft_model, prepare_model_for_kbit_training
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    torch.cuda.set_device(0)
    report["cuda_runtime_settings"] = {
        "allocator_backend": torch.cuda.get_allocator_backend(),
        "allocator_environment": {
            name: os.environ.get(name) for name in ("PYTORCH_ALLOC_CONF", "PYTORCH_CUDA_ALLOC_CONF")
        },
        "cuda_launch_blocking": os.environ.get("CUDA_LAUNCH_BLOCKING"),
        "bf16_reduced_precision_reduction": (
            torch.backends.cuda.matmul.allow_bf16_reduced_precision_reduction
        ),
        "torch_cuda_runtime": torch.version.cuda,
    }
    torch.manual_seed(args.seed)
    torch.cuda.reset_peak_memory_stats(0)
    report["versions"] = {
        name: version(name)
        for name in ("torch", "transformers", "peft", "accelerate", "bitsandbytes")
    }
    tokenizer = AutoTokenizer.from_pretrained(
        str(args.model_dir), local_files_only=True, trust_remote_code=False
    )
    encoded = [
        encode_example(row, tokenizer, 4096)
        for row in records
        if row["split"] in ("train", "validation")
    ]
    training = [row for row in encoded if row["split"] == "train"]
    validation = [row for row in encoded if row["split"] == "validation"]
    if len(training) != 38 or len(validation) != 5:
        raise ValueError("this pilot requires frozen v2 train38/validation5")
    by_id = {row["id"]: row for row in training}
    schedule = make_epoch_groups(training, args.epochs, args.accumulation_steps, args.seed)
    full_steps = sum(len(epoch["groups"]) for epoch in schedule)
    report["full_schedule_optimizer_steps"] = full_steps
    report["planned_optimizer_steps"] = args.diagnostic_steps or full_steps
    report["full_schedule_example_exposures"] = len(training) * args.epochs
    report["planned_example_exposures"] = (
        sum(len(ids) for ids in schedule[0]["groups"][: args.diagnostic_steps])
        if args.diagnostic_steps
        else report["full_schedule_example_exposures"]
    )
    write_json(output / "schedule.json", schedule)
    event("load_nf4_base")
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
        raise RuntimeError("unexpected NF4 transformer layer inventory")
    if any(p.device.type != "cuda" for p in model.parameters()):
        raise RuntimeError("all base weights must be on CUDA")
    model.config.use_cache = False
    event("prepare_lower_memory_frozen_base")
    model = prepare_model_for_kbit_training(
        model,
        use_gradient_checkpointing=True,
        gradient_checkpointing_kwargs={"use_reentrant": False},
    )
    precision = set_frozen_large_layers_bf16(model, torch)
    report["precision"] = precision
    gc.collect()
    torch.cuda.empty_cache()
    report["memory_after_precision_setup"] = gpu_memory(torch)
    provenance = {
        "model_source": source,
        "dataset_sha256": hashes,
        "precision": precision,
        "versions": report["versions"],
        "config": report["config"],
    }
    write_json(output / "run_manifest.json", provenance)
    event("measure_new_precision_baseline")
    baseline = evaluate(
        model, validation, torch, output / "baseline_examples.jsonl", event, "baseline_complete"
    )
    report["baseline"] = baseline
    write_json(
        output / "baseline.json",
        dict(summary=baseline, provenance=provenance, validation_only=True),
    )
    torch.manual_seed(args.seed)
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
    trainable = [(name, p) for name, p in model.named_parameters() if p.requires_grad]
    if sum(p.numel() for _, p in trainable) != 20185088 or any(
        "lora_" not in name or p.dtype != torch.float32 for name, p in trainable
    ):
        raise RuntimeError("expected only reviewed FP32 LoRA parameters to be trainable")
    if not model.is_gradient_checkpointing or model.config.use_cache:
        raise RuntimeError("checkpointing/cache state changed")
    gc.collect()
    torch.cuda.empty_cache()
    optimizer = torch.optim.AdamW([p for _, p in trainable], lr=args.learning_rate, weight_decay=0)
    report["trainable_parameters"] = sum(p.numel() for _, p in trainable)
    best = None
    with (output / "train_steps.jsonl").open("x", encoding="utf-8") as step_log:
        for epoch in schedule:
            model.train()
            torch.cuda.reset_peak_memory_stats(0)
            for group_number, ids in enumerate(epoch["groups"], start=1):
                tokens = group_supervised_tokens(ids, by_id)
                with training_operation(
                    torch,
                    event,
                    "zero_grad",
                    bool(args.diagnostic_steps),
                    epoch=epoch["epoch"],
                    group=group_number,
                ):
                    optimizer.zero_grad(set_to_none=True)
                group_loss = 0.0
                started = time.monotonic()
                event("training_group", epoch=epoch["epoch"], group=group_number, ids=ids)
                for sample_id in ids:
                    example = by_id[sample_id]
                    context = {
                        "epoch": epoch["epoch"],
                        "group": group_number,
                        "sample_id": sample_id,
                        "sequence_tokens": example["sequence_tokens"],
                        "completion_tokens": example["completion_tokens"],
                    }
                    with training_operation(
                        torch, event, "batch_alloc", bool(args.diagnostic_steps), **context
                    ):
                        batch = tensor_batch(example, torch)
                        if (
                            int((batch["labels"][:, 1:] != -100).sum().item())
                            != example["completion_tokens"]
                        ):
                            raise RuntimeError("shifted assistant token count differs")
                    with training_operation(
                        torch, event, "forward", bool(args.diagnostic_steps), **context
                    ):
                        with torch.autocast("cuda", dtype=torch.bfloat16):
                            prediction = model(**batch)
                            loss = prediction.loss
                    with training_operation(
                        torch, event, "loss_check", bool(args.diagnostic_steps), **context
                    ):
                        if not torch.isfinite(loss).item():
                            raise RuntimeError("training loss is nonfinite")
                        loss_value = float(loss.item())
                    scale = example["completion_tokens"] / tokens
                    group_loss += loss_value * scale
                    del prediction
                    with training_operation(
                        torch, event, "backward", bool(args.diagnostic_steps), **context
                    ):
                        (loss * scale).backward()
                    del loss, batch
                    report["processed_examples"] += 1
                with training_operation(
                    torch,
                    event,
                    "gradient_check_clip",
                    bool(args.diagnostic_steps),
                    epoch=epoch["epoch"],
                    group=group_number,
                ):
                    if any(p.grad is None for _, p in trainable):
                        raise RuntimeError("adapter gradient missing")
                    if any(p.grad is not None for p in model.parameters() if not p.requires_grad):
                        raise RuntimeError("frozen base unexpectedly has gradients")
                    norm = torch.nn.utils.clip_grad_norm_(
                        [p for _, p in trainable], 1.0, error_if_nonfinite=True
                    )
                    chosen = next(
                        (
                            (n, p)
                            for n, p in trainable
                            if "lora_B" in n and torch.count_nonzero(p.grad).item()
                        ),
                        None,
                    )
                    if chosen is None:
                        raise RuntimeError("all LoRA B gradients are zero")
                    _, checked = chosen
                    before = checked.detach().clone()
                report["optimizer_step_attempts"] += 1
                with training_operation(
                    torch,
                    event,
                    "optimizer_step",
                    bool(args.diagnostic_steps),
                    epoch=epoch["epoch"],
                    group=group_number,
                ):
                    optimizer.step()
                    torch.cuda.synchronize()
                if torch.equal(before, checked.detach()):
                    raise RuntimeError("optimizer did not change nonzero-gradient LoRA B")
                del before
                report["completed_optimizer_steps"] += 1
                row = {
                    "epoch": epoch["epoch"],
                    "group": group_number,
                    "step": report["completed_optimizer_steps"],
                    "example_ids": ids,
                    "supervised_tokens": tokens,
                    "token_weighted_train_loss": group_loss,
                    "gradient_norm_before_clip": float(norm.item()),
                    "seconds": round(time.monotonic() - started, 3),
                    "memory": gpu_memory(torch),
                }
                step_log.write(json.dumps(row, allow_nan=False) + "\n")
                step_log.flush()
                print(
                    "STEP",
                    row["step"],
                    "/",
                    report["planned_optimizer_steps"],
                    "LOSS",
                    round(group_loss, 5),
                    flush=True,
                )
                optimizer.zero_grad(set_to_none=True)
                if args.diagnostic_steps and (
                    report["completed_optimizer_steps"] == args.diagnostic_steps
                ):
                    report["diagnostic_completed"] = True
                    report["adapter_saved"] = False
                    event("finite_diagnostic_complete")
                    return
            checkpoint = output / f"epoch_{epoch['epoch']:02d}"
            event("save_epoch_adapter", epoch=epoch["epoch"])
            saved = save_checkpoint(
                model, trainable, checkpoint, dict(provenance, epoch=epoch["epoch"]), torch
            )
            report["saved_checkpoints"].append(str(checkpoint))
            gc.collect()
            torch.cuda.empty_cache()
            validation_result = evaluate(
                model,
                validation,
                torch,
                output / f"validation_epoch_{epoch['epoch']:02d}.jsonl",
                event,
                "epoch_validation_complete",
            )
            epoch_result = {
                "epoch": epoch["epoch"],
                "checkpoint": checkpoint.name,
                "validation": validation_result,
                "adapter_files": saved["files_sha256"],
            }
            write_json(output / f"epoch_{epoch['epoch']:02d}_result.json", epoch_result)
            report["completed_epochs"].append(epoch_result)
            if (
                best is None
                or validation_result["token_weighted_loss"]
                < best["validation"]["token_weighted_loss"]
            ):
                best = epoch_result
    if report["completed_optimizer_steps"] != report["planned_optimizer_steps"]:
        raise RuntimeError("executed optimizer steps differ from finite schedule")
    for name, expected in hashes.items():
        if hashlib.sha256((args.dataset / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError("frozen dataset changed")
    selection = {
        "best_checkpoint": best["checkpoint"],
        "epoch": best["epoch"],
        "validation_loss": best["validation"]["token_weighted_loss"],
        "same_precision_baseline_loss": baseline["token_weighted_loss"],
        "validation_improved_vs_base": best["validation"]["token_weighted_loss"]
        < baseline["token_weighted_loss"],
        "selected_with_test": False,
        "functional_code_improvement_verified": False,
    }
    write_json(output / "selection.json", selection)
    tokenizer.save_pretrained(output / "tokenizer")
    report.update(selection=selection, dataset_unchanged=True, formal_pilot_training_completed=True)
    event("finite_training_complete")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
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
    parser.add_argument("--epochs", type=int, default=3)
    parser.add_argument("--accumulation-steps", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--diagnostic-steps",
        type=int,
        default=0,
        help="0: formal pilot; 1 or 2: synchronized finite diagnostic, discard adapter",
    )
    args = parser.parse_args(argv)
    validate_args(args)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "STARTED",
        "mode": "diagnostic" if args.diagnostic_steps else "formal",
        "config": {
            "epochs": args.epochs,
            "microbatch": 1,
            "accumulation_steps": args.accumulation_steps,
            "learning_rate": args.learning_rate,
            "seed": args.seed,
            "max_length": 4096,
            "lora_rank": 8,
            "lora_alpha": 16,
            "target_modules": TARGET_MODULES,
            "optimizer": "AdamW",
            "weight_decay": 0.0,
            "gradient_clip_norm": 1.0,
            "compute_dtype": "bfloat16_autocast",
            "adapter_dtype": "float32",
            "gradient_checkpointing": True,
            "use_reentrant": False,
            "loss_accumulation": "assistant_token_weighted_per_group",
            "diagnostic_steps": args.diagnostic_steps,
            "cuda_launch_blocking": bool(args.diagnostic_steps)
            or os.environ.get("CUDA_LAUNCH_BLOCKING") == "1",
        },
        "processed_examples": 0,
        "optimizer_step_attempts": 0,
        "completed_optimizer_steps": 0,
        "saved_checkpoints": [],
        "completed_epochs": [],
        "formal_pilot_training_completed": False,
        "diagnostic_completed": False,
        "test_examples_evaluated": 0,
        "downloads_enabled": False,
        "robot_io": False,
        "exact_optimizer_resume_supported": False,
        "source_sha256": {
            name: hashlib.sha256((ROOT / "tools" / name).read_bytes()).hexdigest()
            for name in (
                "train_code_adapter.py",
                "code_training_diagnostics.py",
                "code_training_precision.py",
                "code_training_schedule.py",
                "training_examples.py",
            )
        },
    }
    with (output / "events.jsonl").open("x", encoding="utf-8") as log:

        def event(stage, **values):
            report["last_stage"] = stage
            report["last_stage_details"] = values
            log.write(
                json.dumps(
                    {"stage": stage, "time": datetime.now(timezone.utc).isoformat(), **values},
                    allow_nan=False,
                )
                + "\n"
            )
            log.flush()
            details = " ".join(str(values[key]) for key in ("phase", "sample_id") if key in values)
            print("STAGE:", stage, details, flush=True)

        try:
            run(args, output, report, event)
            if args.diagnostic_steps:
                if (
                    not report["diagnostic_completed"]
                    or report["completed_optimizer_steps"] != args.diagnostic_steps
                    or report["formal_pilot_training_completed"]
                    or report["saved_checkpoints"]
                ):
                    raise RuntimeError("diagnostic did not complete its exact finite budget")
                report["status"] = "DIAGNOSTIC_PASS"
            elif not report["formal_pilot_training_completed"]:
                raise RuntimeError("training returned without completing the reviewed run")
            else:
                report["status"] = "PASS"
        except (Exception, KeyboardInterrupt) as error:
            report.update(status="FAIL", error=type(error).__name__ + ": " + str(error))
            report["error_time"] = datetime.now(timezone.utc).isoformat()
            report["error_traceback"] = traceback.format_exc()
            with (output / "failure_traceback.txt").open("x", encoding="utf-8") as stream:
                stream.write(report["error_traceback"])
            torch = sys.modules.get("torch")
            if torch is not None:
                try:
                    report["memory_on_failure"] = gpu_memory(torch)
                except Exception as query_error:
                    report["memory_query_error"] = (
                        type(query_error).__name__ + ": " + str(query_error)
                    )
        finally:
            write_json(output / "report.json", report)
    if report["status"] == "PASS":
        print("SELECTION:", json.dumps(report["selection"]))
    elif report["status"] == "FAIL":
        print("ERROR:", report["error"])
    print("REPORT:", output / "report.json")
    marker = {
        "PASS": "FINETUNE_PASS",
        "DIAGNOSTIC_PASS": "DIAGNOSTIC_PASS",
        "FAIL": "FINETUNE_FAIL",
    }
    print(marker[report["status"]])
    return 1 if report["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
