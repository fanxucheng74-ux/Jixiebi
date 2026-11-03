"""A narrow BF16 policy for frozen, nonquantized embedding and output modules.

Run after PEFT k-bit preparation. Normalization parameters must already be FP32.
The policy follows TRL's BF16 QLoRA handling of embeddings/lm_head, adding explicit
frozen/quantization guards. It imports no ML packages and never opens a device.

Reference: huggingface/trl v0.24.0, trl/models/utils.py,
peft_module_casting_to_bf16 and prepare_peft_model.
"""

POLICY = "frozen_embeddings_head_bf16_norms_fp32_v1"


def _is_quantized(parameter):
    return (
        any(base.__name__ == "Params4bit" for base in type(parameter).__mro__)
        or getattr(parameter, "quant_state", None) is not None
        or not getattr(getattr(parameter, "dtype", None), "is_floating_point", False)
    )


def _is_norm(name):
    name = name.lower()
    return (
        "layernorm" in name
        or ".norm." in name
        or name.startswith("norm.")
        or name.endswith("norm.weight")
        or name.endswith("norm.bias")
    )


def set_frozen_large_layers_bf16(model, torch):
    """Cast only validated frozen embedding/head modules; never freeze implicitly."""
    modules = {
        "input_embeddings": model.get_input_embeddings(),
        "output_embeddings": model.get_output_embeddings(),
    }
    unique, layer_metadata = {}, {}
    for label, module in modules.items():
        if module is None or getattr(module, "weight", None) is None:
            raise ValueError("missing weighted large module: " + label)
        parameters = list(module.parameters())
        if not parameters or all(parameter is not module.weight for parameter in parameters):
            raise ValueError("large module weight is not a registered parameter: " + label)
        if any(parameter.requires_grad for parameter in parameters):
            raise ValueError("large module must already be frozen: " + label)
        if any(_is_quantized(parameter) for parameter in parameters):
            raise ValueError("large module must be floating and nonquantized: " + label)
        unique[id(module)] = (module, parameters)
        layer_metadata[label] = {"dtype_before": str(module.weight.dtype)}

    target_ids = {id(parameter) for _, parameters in unique.values() for parameter in parameters}
    named = list(model.named_parameters())
    norms = [(name, parameter) for name, parameter in named if _is_norm(name)]
    if not norms or any(parameter.dtype != torch.float32 for _, parameter in norms):
        raise ValueError("normalization parameters must already be FP32")
    if any(id(parameter) in target_ids for _, parameter in norms):
        raise ValueError("large module shares a normalization parameter")
    if any("lora_" in name.lower() and id(parameter) in target_ids for name, parameter in named):
        raise ValueError("large module contains or shares a LoRA parameter")
    untouched = [
        (parameter, parameter.dtype, parameter.requires_grad)
        for _, parameter in named
        if id(parameter) not in target_ids
    ]

    # All model/state checks above precede the first dtype mutation.
    casts = 0
    for module, parameters in unique.values():
        if any(parameter.dtype != torch.bfloat16 for parameter in parameters):
            module.to(dtype=torch.bfloat16)
            casts += 1
        if any(
            parameter.dtype != torch.bfloat16 or parameter.requires_grad
            for parameter in module.parameters()
        ):
            raise RuntimeError("large module did not remain frozen BF16 after casting")
    if any(
        parameter.dtype != dtype or parameter.requires_grad != frozen
        for parameter, dtype, frozen in untouched
    ):
        raise RuntimeError("an unrelated parameter changed during large-layer casting")
    if any(parameter.dtype != torch.float32 for _, parameter in norms):
        raise RuntimeError("normalization precision changed during large-layer casting")
    for label, module in modules.items():
        layer_metadata[label].update(dtype_after=str(module.weight.dtype), frozen=True)
    return {
        "policy": POLICY,
        "layers": layer_metadata,
        "unique_large_module_count": len(unique),
        "modules_cast_count": casts,
        "norm_parameter_count": len(norms),
        "norms_fp32_verified": True,
        "unrelated_parameter_dtypes_and_trainability_unchanged": True,
    }
