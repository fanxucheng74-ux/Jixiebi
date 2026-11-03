"""Explicit CUDA probe using a tiny random NF4 layer and LoRA, without downloads."""

import argparse
from datetime import datetime, timezone
from importlib.metadata import version
import json
import os
from pathlib import Path


def run_probe():
    # These flags affect this process only. The probe never loads model files.
    os.environ["USE_TORCH"] = "1"
    os.environ["USE_TF"] = "0"
    os.environ["USE_FLAX"] = "0"
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    import bitsandbytes as bnb
    from peft import LoraConfig, get_peft_model
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in this Python environment")
    torch.manual_seed(7)
    torch.cuda.reset_peak_memory_stats()

    class TinyProjection(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.config = {"model_type": "custom", "tie_word_embeddings": False}
            self.proj = bnb.nn.Linear4bit(
                128,
                128,
                bias=False,
                compute_dtype=torch.bfloat16,
                quant_type="nf4",
                compress_statistics=True,
            )
            self.is_loaded_in_4bit = True

        def forward(self, x):
            return self.proj(x)

    network = get_peft_model(
        TinyProjection().to("cuda"),
        LoraConfig(r=4, lora_alpha=8, target_modules=["proj"], lora_dropout=0.0, bias="none"),
    )
    base = network.get_base_model().proj.get_base_layer()
    if not isinstance(base, bnb.nn.Linear4bit) or base.weight.quant_state is None:
        raise RuntimeError("base layer was not quantized to 4-bit")
    if base.weight.quant_state.quant_type != "nf4" or base.weight.requires_grad:
        raise RuntimeError("NF4 base layer must be frozen")
    trainable = [(name, p) for name, p in network.named_parameters() if p.requires_grad]
    if not trainable or any("lora_" not in name for name, _ in trainable):
        raise RuntimeError("expected only LoRA parameters to be trainable")
    base_before = base.weight.detach().clone()
    adapter_before = [p.detach().clone() for _, p in trainable]
    optimizer = torch.optim.AdamW([p for _, p in trainable], lr=1e-3, weight_decay=0)
    network.train()
    x = torch.randn(4, 128, device="cuda", dtype=torch.bfloat16, requires_grad=True)
    loss = network(x).float().square().mean()
    if not torch.isfinite(loss).item():
        raise RuntimeError("loss is not finite")
    loss.backward()
    gradients = [p.grad for _, p in trainable]
    if any(gradient is None or not torch.isfinite(gradient).all().item() for gradient in gradients):
        raise RuntimeError("LoRA gradient is missing or nonfinite")
    # Standard LoRA initializes B to zero, so A can have a zero first gradient.
    if not any(torch.count_nonzero(gradient).item() > 0 for gradient in gradients):
        raise RuntimeError("all adapter gradients are zero")
    if x.grad is None or not torch.isfinite(x.grad).all().item():
        raise RuntimeError("NF4 input gradient is missing or nonfinite")
    optimizer.step()
    torch.cuda.synchronize()
    if not any(
        not torch.equal(before, parameter.detach())
        for before, (name, parameter) in zip(adapter_before, trainable, strict=True)
        if "lora_B" in name
    ):
        raise RuntimeError("optimizer did not update a LoRA B parameter")
    if not torch.equal(base_before, base.weight.detach()):
        raise RuntimeError("frozen quantized base weight changed")
    return {
        "versions": {
            name: version(name)
            for name in ("torch", "transformers", "peft", "accelerate", "bitsandbytes")
        },
        "gpu": torch.cuda.get_device_name(0),
        "cuda_runtime": torch.version.cuda,
        "base_quant_type": base.weight.quant_state.quant_type,
        "base_weight_frozen_and_unchanged": True,
        "trainable_parameters": sum(p.numel() for _, p in trainable),
        "loss": float(loss.detach().item()),
        "lora_finite_nonzero_gradient_and_update": True,
        "nf4_input_gradient_verified": True,
        "peak_cuda_allocated_bytes": torch.cuda.max_memory_allocated(),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "kind": "SYNTHETIC_NF4_LORA_CUDA_PROBE",
        "status": "STARTED",
        "model_downloaded": False,
        "dataset_training_started": False,
        "robot_io": False,
        "full_7b_memory_fit_verified": False,
    }
    # Exclusive create before using CUDA, so a prior report cannot be replaced.
    with args.output.open("x", encoding="utf-8") as stream:
        try:
            report["probe"] = run_probe()
            report["status"] = "PASS"
        except Exception as error:
            report.update(status="FAIL", error=type(error).__name__ + ": " + str(error))
        finally:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("REPORT:", args.output.resolve())
    print("QLORA_STACK_PASS" if report["status"] == "PASS" else "QLORA_STACK_FAIL")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
