"""Compare frozen base and selected LoRA on test5; never train or execute code."""

import argparse
from datetime import datetime, timezone
import gc
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import sys
import traceback

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from arm_project.paths import PROJECT_ROOT
from tools.adapter_evaluation_inputs import verify_selected_adapter
from tools.check_local_qwen import verify_local_model
from tools.code_training_precision import set_frozen_large_layers_bf16
from tools.inspect_training_tokens import load_records
from tools.run_code_training_preflight import gpu_memory, write_json
from tools.train_code_adapter import evaluate
from tools.training_examples import encode_example

ROOT = PROJECT_ROOT


def run(args, output, report, event):
    for name, value in {
        "HF_HUB_OFFLINE": "1",
        "TRANSFORMERS_OFFLINE": "1",
        "USE_TORCH": "1",
        "USE_TF": "0",
        "USE_FLAX": "0",
    }.items():
        os.environ[name] = value
    event("verify_selected_adapter_and_frozen_dataset")
    provenance = verify_selected_adapter(args.run_dir, args.dataset)
    report["adapter_provenance"] = provenance
    report["selected_checkpoint"] = provenance["checkpoint_dir"]
    records, hashes = load_records(args.dataset)
    if hashes != provenance["dataset_sha256"]:
        raise ValueError("evaluation dataset differs from selected training provenance")
    report["dataset_sha256"] = hashes
    event("verify_frozen_base")
    source = verify_local_model(args.model_dir, args.manifest)
    if source != provenance["model_source"]:
        raise ValueError("base source differs from selected adapter training source")
    report["model_source"] = source
    import bitsandbytes as bnb
    from peft import PeftModel, prepare_model_for_kbit_training
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    torch.cuda.set_device(0)
    torch.manual_seed(42)
    torch.cuda.reset_peak_memory_stats(0)
    report["versions"] = {
        name: version(name)
        for name in ("torch", "transformers", "peft", "accelerate", "bitsandbytes")
    }
    report["cuda_runtime_settings"] = {
        "allocator_backend": torch.cuda.get_allocator_backend(),
        "allocator_environment": {
            name: os.environ.get(name) for name in ("PYTORCH_ALLOC_CONF", "PYTORCH_CUDA_ALLOC_CONF")
        },
        "cuda_launch_blocking": os.environ.get("CUDA_LAUNCH_BLOCKING"),
    }
    tokenizer = AutoTokenizer.from_pretrained(
        str(args.model_dir), local_files_only=True, trust_remote_code=False
    )
    test = [encode_example(row, tokenizer, 4096) for row in records if row["split"] == "test"]
    if len(test) != 5:
        raise ValueError("frozen v2 test partition must contain exactly five examples")
    report["test_ids"] = [row["id"] for row in test]
    report["test_categories"] = {row["id"]: row["category"] for row in test}
    report["max_test_sequence_tokens"] = max(row["sequence_tokens"] for row in test)
    event("load_nf4_base_once")
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
        module.weight.quant_state is None
        or module.weight.quant_state.quant_type != "nf4"
        or not module.weight.quant_state.nested
        for module in quantized
    ):
        raise RuntimeError("unexpected NF4 base layer inventory")
    if any(parameter.device.type != "cuda" for parameter in model.parameters()):
        raise RuntimeError("all base weights must be on CUDA")
    # The preparation utility matches training precision; no optimizer or backward exists here.
    model.config.use_cache = False
    model = prepare_model_for_kbit_training(model, use_gradient_checkpointing=False)
    precision = set_frozen_large_layers_bf16(model, torch)
    if precision != provenance["precision"]:
        raise ValueError("evaluation precision differs from selected training precision")
    report["precision"] = precision
    model.requires_grad_(False)
    gc.collect()
    torch.cuda.empty_cache()
    report["memory_before_base_test"] = gpu_memory(torch)
    event("evaluate_base_test5")
    base = evaluate(
        model, test, torch, output / "base_test_examples.jsonl", event, "base_test_complete"
    )
    report["base_test"] = base
    write_json(output / "base_test.json", base)
    event("load_selected_adapter_for_inference")
    model = PeftModel.from_pretrained(
        model,
        provenance["checkpoint_dir"],
        is_trainable=False,
        autocast_adapter_dtype=True,
        local_files_only=True,
        use_safetensors=True,
    )
    model.requires_grad_(False)
    model.eval()
    lora = [(name, parameter) for name, parameter in model.named_parameters() if "lora_" in name]
    if sum(parameter.numel() for _, parameter in lora) != provenance["adapter_parameter_count"]:
        raise RuntimeError("loaded adapter parameter count differs from verified checkpoint")
    if any(parameter.dtype != torch.float32 for _, parameter in lora):
        raise RuntimeError("loaded adapter precision differs from FP32 training adapter")
    if any(
        parameter.requires_grad or parameter.grad is not None for parameter in model.parameters()
    ):
        raise RuntimeError("evaluation model must have no trainable parameters or gradients")
    if any(parameter.device.type != "cuda" for parameter in model.parameters()):
        raise RuntimeError("adapter loading unexpectedly changed CUDA placement")
    report["loaded_adapter_parameters"] = sum(parameter.numel() for _, parameter in lora)
    report["adapter_loaded"] = True
    gc.collect()
    torch.cuda.empty_cache()
    report["memory_before_adapter_test"] = gpu_memory(torch)
    event("evaluate_adapter_test5")
    adapted = evaluate(
        model, test, torch, output / "adapter_test_examples.jsonl", event, "adapter_test_complete"
    )
    report["adapter_test"] = adapted
    write_json(output / "adapter_test.json", adapted)
    if any(parameter.grad is not None for parameter in model.parameters()):
        raise RuntimeError("evaluation unexpectedly produced gradients")
    for name, expected in hashes.items():
        if hashlib.sha256((args.dataset / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError("frozen dataset changed during evaluation")
    for name, expected in provenance["adapter_files_sha256"].items():
        if (
            hashlib.sha256((Path(provenance["checkpoint_dir"]) / name).read_bytes()).hexdigest()
            != expected
        ):
            raise RuntimeError("adapter files changed during evaluation")
    old_loss, new_loss = base["token_weighted_loss"], adapted["token_weighted_loss"]
    comparison = {
        "base_test_loss": old_loss,
        "adapter_test_loss": new_loss,
        "test_loss_improved": new_loss < old_loss,
        "relative_loss_reduction": (old_loss - new_loss) / old_loss if old_loss else None,
        "metric": "assistant_token_weighted_teacher_forced_cross_entropy",
        "functional_code_improvement_verified": False,
        "checkpoint_selected_using_test": False,
    }
    write_json(output / "comparison.json", comparison)
    report.update(
        comparison=comparison,
        test_examples_evaluated=5,
        dataset_unchanged=True,
        adapter_files_unchanged=True,
        memory_after=gpu_memory(torch),
        evaluation_completed=True,
    )
    event("heldout_loss_evaluation_complete")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--run-dir", type=Path, default=ROOT / "data/models/adapters/jixiebi_code_v2_20261101_02"
    )
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
    if (ROOT / "artifacts").resolve() not in output.parents:
        raise ValueError("evaluation output must be a new directory under project artifacts")
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "STARTED",
        "evaluation_completed": False,
        "training_started": False,
        "test_examples_evaluated": 0,
        "selected_with_test": False,
        "robot_io": False,
        "downloads_enabled": False,
        "adapter_loaded": False,
        "generated_text_executed": False,
        "functional_code_improvement_verified": False,
        "generation_performed": False,
        "metric": "teacher_forced_reference_code_loss_not_generated_code_accuracy",
        "source_sha256": {
            name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in (
                "evaluate_code_adapter.py",
                "adapter_evaluation_inputs.py",
                "train_code_adapter.py",
                "training_examples.py",
                "code_training_precision.py",
            )
        },
    }
    with (output / "events.jsonl").open("x", encoding="utf-8") as log:

        def event(stage, **values):
            report["last_stage"] = stage
            log.write(
                json.dumps(
                    {"stage": stage, "time": datetime.now(timezone.utc).isoformat(), **values}
                )
                + "\n"
            )
            log.flush()
            print("STAGE:", stage, flush=True)

        try:
            run(args, output, report, event)
            if (
                not report["evaluation_completed"]
                or report["test_examples_evaluated"] != 5
                or report["selected_with_test"]
            ):
                raise RuntimeError("evaluation returned without completing fixed test5")
            report["status"] = "PASS"
        except (Exception, KeyboardInterrupt) as error:
            report.update(
                status="FAIL",
                error=type(error).__name__ + ": " + str(error),
                error_traceback=traceback.format_exc(),
            )
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
        print("COMPARISON:", json.dumps(report["comparison"]))
    else:
        print("ERROR:", report["error"])
    print("REPORT:", output / "report.json")
    print("EVALUATION_PASS" if report["status"] == "PASS" else "EVALUATION_FAIL")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
