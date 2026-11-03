"""Finite synthetic LoRA recomputation check; no full model or NF4 base is loaded."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import traceback

ROOT = Path(__file__).resolve().parents[1]
LENGTHS = (3583, 1104, 472, 982)


def load_spec(config_path, failure_path):
    config_raw, failure_raw = config_path.read_bytes(), failure_path.read_bytes()
    config, failure = json.loads(config_raw), json.loads(failure_raw)
    hidden = config.get("hidden_size")
    heads = config.get("num_attention_heads")
    kv_heads = config.get("num_key_value_heads")
    rank = failure.get("config", {}).get("lora_rank")
    alpha = failure.get("config", {}).get("lora_alpha")
    values = (hidden, heads, kv_heads, rank, alpha)
    if any(type(value) is not int or value <= 0 for value in values):
        raise ValueError("positive integer model/adapter dimensions required")
    if config.get("model_type") != "qwen2" or hidden % heads:
        raise ValueError("expected a Qwen2 hidden size divisible by attention heads")
    head_dim = config.get("head_dim", hidden // heads)
    if type(head_dim) is not int or head_dim <= 0:
        raise ValueError("invalid attention head dimension")
    out_features = kv_heads * head_dim
    if hidden > 8192 or out_features > 4096 or rank > 32 or alpha > 128:
        raise ValueError("dimensions exceed finite small-probe limits")
    return {
        "hidden_size": hidden,
        "head_dim": head_dim,
        "v_proj_out_features": out_features,
        "rank": rank,
        "alpha": alpha,
        "scaling": alpha / rank,
        "config_sha256": hashlib.sha256(config_raw).hexdigest(),
        "failure_report_sha256": hashlib.sha256(failure_raw).hexdigest(),
    }


def cuda_memory(torch):
    free, total = torch.cuda.mem_get_info(0)
    return {
        "allocated_mib": round(torch.cuda.memory_allocated(0) / 2**20, 2),
        "reserved_mib": round(torch.cuda.memory_reserved(0) / 2**20, 2),
        "peak_allocated_mib": round(torch.cuda.max_memory_allocated(0) / 2**20, 2),
        "global_free_mib": round(free / 2**20, 2),
        "global_total_mib": round(total / 2**20, 2),
    }


def run_probe(args, report, event):
    os.environ["CUDA_LAUNCH_BLOCKING"] = "1"
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    import torch
    from torch.utils.checkpoint import checkpoint

    spec = load_spec(args.config, args.failure_report)
    report["spec"] = spec
    report["versions"] = {"torch": torch.__version__, "cuda_runtime": torch.version.cuda}
    report["allocator_environment"] = {
        key: os.environ.get(key) for key in ("PYTORCH_ALLOC_CONF", "PYTORCH_CUDA_ALLOC_CONF")
    }
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    torch.cuda.set_device(0)
    torch.manual_seed(42)
    torch.cuda.reset_peak_memory_stats(0)
    report["gpu"] = torch.cuda.get_device_name(0)
    report["allocator_backend"] = torch.cuda.get_allocator_backend()
    report["memory_before"] = cuda_memory(torch)
    a = torch.nn.Linear(
        spec["hidden_size"], spec["rank"], bias=False, device="cuda:0", dtype=torch.float32
    )
    b = torch.nn.Linear(
        spec["rank"], spec["v_proj_out_features"], bias=False, device="cuda:0", dtype=torch.float32
    )
    # Synthetic nonzero B exercises input/A/B gradients; this is not PEFT's zero-B init.
    with torch.no_grad():
        b.weight.normal_(0, 0.01)
    parameters = list(a.parameters()) + list(b.parameters())
    if any(parameter.dtype != torch.float32 for parameter in parameters):
        raise RuntimeError("probe adapter weights must remain FP32")
    report["parameter_shapes"] = {"A": list(a.weight.shape), "B": list(b.weight.shape)}
    report["parameter_strides"] = {"A": list(a.weight.stride()), "B": list(b.weight.stride())}
    report["parameter_dtype"] = "float32"
    optimizer = torch.optim.AdamW(parameters, lr=1e-3, weight_decay=0)
    report["iterations"] = []
    for repeat in range(1, args.repeats + 1):
        optimizer.zero_grad(set_to_none=True)
        before = b.weight.detach().clone()
        for index, length in enumerate(LENGTHS, start=1):
            row = {"repeat": repeat, "index": index, "sequence_tokens": length}
            event("allocate_input", **row)
            x = torch.randn(
                1,
                length,
                spec["hidden_size"],
                device="cuda:0",
                dtype=torch.float32,
                requires_grad=True,
            )
            counts = {"A": 0, "B": 0}

            def projection(value):
                counts["A"] += 1
                intermediate = a(value)
                counts["B"] += 1
                return b(intermediate) * spec["scaling"]

            row.update(
                input_shape=list(x.shape), input_stride=list(x.stride()), input_dtype=str(x.dtype)
            )
            event("forward", **row)
            with torch.autocast("cuda", dtype=torch.bfloat16):
                y = checkpoint(projection, x, use_reentrant=False)
                loss = y.float().square().mean() / len(LENGTHS)
            torch.cuda.synchronize(0)
            if y.dtype != torch.bfloat16:
                raise RuntimeError("synthetic projection did not use BF16 autocast output")
            if not torch.isfinite(loss).item():
                raise RuntimeError("nonfinite synthetic loss")
            row["loss"] = float(loss.item())
            row["output_dtype"] = str(y.dtype)
            event("backward", **row)
            loss.backward()
            torch.cuda.synchronize(0)
            if counts["A"] < 2 or counts["B"] < 2:
                raise RuntimeError("checkpoint recomputation was not entered")
            gradients = [x.grad, *(parameter.grad for parameter in parameters)]
            if any(
                gradient is None or not torch.isfinite(gradient).all().item()
                for gradient in gradients
            ):
                raise RuntimeError("missing or nonfinite synthetic gradient")
            if any(torch.count_nonzero(gradient).item() == 0 for gradient in gradients):
                raise RuntimeError("synthetic input/A/B gradient is zero")
            row.update(
                projection_calls=counts,
                checkpoint_recompute_count=counts["A"] - 1,
                input_and_adapter_gradients_finite_nonzero=True,
                memory=cuda_memory(torch),
            )
            report["iterations"].append(row)
            event("iteration_complete", **row)
            del gradients, y, loss, x
        event("optimizer_step", repeat=repeat)
        norm = torch.nn.utils.clip_grad_norm_(parameters, 1.0, error_if_nonfinite=True)
        if not math.isfinite(float(norm.item())):
            raise RuntimeError("nonfinite synthetic gradient norm")
        optimizer.step()
        torch.cuda.synchronize(0)
        if torch.equal(before, b.weight.detach()):
            raise RuntimeError("synthetic optimizer did not update B")
        del before, norm
        report["completed_optimizer_steps"] += 1
    optimizer.zero_grad(set_to_none=True)
    report["memory_after"] = cuda_memory(torch)
    if len(report["iterations"]) != args.repeats * len(LENGTHS):
        raise RuntimeError("finite probe did not complete all planned iterations")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument(
        "--config", type=Path, default=ROOT / "data/models/Qwen2.5-Coder-7B-Instruct/config.json"
    )
    parser.add_argument(
        "--failure-report",
        type=Path,
        default=ROOT / "data/models/adapters/jixiebi_code_v2_diag_20261101_01/report.json",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 3:
        raise ValueError("repeats must be 1..3")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "status": "STARTED",
        "kind": "SYNTHETIC_LORA_BF16_CHECKPOINT_RECOMPUTE",
        "sequence_lengths": list(LENGTHS),
        "repeats": args.repeats,
        "planned_iterations": args.repeats * len(LENGTHS),
        "completed_optimizer_steps": 0,
        "model_loaded": False,
        "nf4_base_loaded": False,
        "dataset_training_started": False,
        "robot_io": False,
        "adapter_saved": False,
        "full_7b_training_stability_verified": False,
        "synthetic_nonzero_B_initialization": True,
        "loss": "synthetic mean square, equal four-microbatch accumulation",
        "attention_or_bitsandbytes_path_tested": False,
        "actual_failure_input_stride_known": False,
        "probe_input_layout": "synthetic contiguous; original failing strides were not logged",
    }
    with args.output.open("x", encoding="utf-8") as stream:

        def event(stage, **context):
            report["last_stage"] = stage
            report["last_stage_details"] = context
            print(
                "STAGE:",
                stage,
                "repeat",
                context.get("repeat"),
                "length",
                context.get("sequence_tokens"),
                flush=True,
            )

        try:
            run_probe(args, report, event)
            if report["completed_optimizer_steps"] != args.repeats:
                raise RuntimeError("finite optimizer budget incomplete")
            report["status"] = "PASS"
        except (Exception, KeyboardInterrupt) as error:
            report.update(
                status="FAIL",
                error=type(error).__name__ + ": " + str(error),
                error_traceback=traceback.format_exc(),
            )
            import sys

            torch = sys.modules.get("torch")
            if torch is not None:
                try:
                    report["memory_on_failure"] = cuda_memory(torch)
                except Exception as query_error:
                    report["memory_query_error"] = (
                        type(query_error).__name__ + ": " + str(query_error)
                    )
        finally:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    if report["status"] == "FAIL":
        print("ERROR:", report["error"])
    print("REPORT:", args.output.resolve())
    print("LORA_RECOMPUTE_PASS" if report["status"] == "PASS" else "LORA_RECOMPUTE_FAIL")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
