"""Precision-policy boundaries use fake modules; no Torch, GPU or downloads."""

import unittest

from tools.code_training_precision import POLICY, set_frozen_large_layers_bf16


class FakeDtype:
    def __init__(self, name, floating=True):
        self.name, self.is_floating_point = name, floating

    def __str__(self):
        return self.name


class FakeTorch:
    float32 = FakeDtype("torch.float32")
    bfloat16 = FakeDtype("torch.bfloat16")
    uint8 = FakeDtype("torch.uint8", floating=False)


class Parameter:
    def __init__(self, dtype=FakeTorch.float32, trainable=False, quant_state=None):
        self.dtype, self.requires_grad, self.quant_state = dtype, trainable, quant_state


class FakeModule:
    def __init__(self, weight=None):
        self.weight = weight if weight is not None else Parameter()
        self.cast_calls = []

    def parameters(self):
        return iter([self.weight])

    def to(self, *, dtype):
        self.cast_calls.append(dtype)
        self.weight.dtype = dtype
        return self


class FakeModel:
    def __init__(self, tied=False):
        self.embedding = FakeModule()
        self.head = self.embedding if tied else FakeModule()
        self.norm = FakeModule()
        self.quant = Parameter(dtype=FakeTorch.uint8, quant_state=object())
        self.adapter = Parameter(trainable=True)

    def get_input_embeddings(self):
        return self.embedding

    def get_output_embeddings(self):
        return self.head

    def named_parameters(self):
        return iter(
            [
                ("model.embed_tokens.weight", self.embedding.weight),
                ("lm_head.weight", self.head.weight),
                ("model.norm.weight", self.norm.weight),
                ("model.layers.0.q_proj.weight", self.quant),
                ("model.layers.0.q_proj.lora_A.default.weight", self.adapter),
            ]
        )


class LargeLayerPrecisionTests(unittest.TestCase):
    def test_casts_only_two_frozen_large_layers_and_reports_actual_dtypes(self):
        model = FakeModel()
        quant_before = (model.quant.dtype, model.quant.quant_state)
        result = set_frozen_large_layers_bf16(model, FakeTorch)
        self.assertEqual(result["policy"], POLICY)
        self.assertEqual(result["modules_cast_count"], 2)
        for label in ("input_embeddings", "output_embeddings"):
            self.assertEqual(result["layers"][label]["dtype_before"], "torch.float32")
            self.assertEqual(result["layers"][label]["dtype_after"], "torch.bfloat16")
            self.assertTrue(result["layers"][label]["frozen"])
        self.assertEqual(model.norm.cast_calls, [])
        self.assertIs(model.norm.weight.dtype, FakeTorch.float32)
        self.assertEqual((model.quant.dtype, model.quant.quant_state), quant_before)
        self.assertIs(model.adapter.dtype, FakeTorch.float32)
        self.assertTrue(model.adapter.requires_grad)

    def test_invalid_output_head_is_rejected_before_embedding_is_changed(self):
        for mode in ("trainable", "quantized", "integer", "missing"):
            with self.subTest(mode=mode):
                model = FakeModel()
                if mode == "trainable":
                    model.head.weight.requires_grad = True
                elif mode == "quantized":
                    model.head.weight.quant_state = object()
                elif mode == "integer":
                    model.head.weight.dtype = FakeTorch.uint8
                else:
                    model.head = None
                with self.assertRaises(ValueError):
                    set_frozen_large_layers_bf16(model, FakeTorch)
                self.assertIs(model.embedding.weight.dtype, FakeTorch.float32)
                self.assertEqual(model.embedding.cast_calls, [])

    def test_params4bit_is_rejected_even_before_quant_state_initialization(self):
        class Params4bit(Parameter):
            pass

        model = FakeModel()
        model.head.weight = Params4bit()
        with self.assertRaisesRegex(ValueError, "nonquantized"):
            set_frozen_large_layers_bf16(model, FakeTorch)
        self.assertEqual(model.embedding.cast_calls, [])

    def test_wrong_norm_precision_and_parameter_alias_rejected_before_cast(self):
        model = FakeModel()
        model.norm.weight.dtype = FakeTorch.bfloat16
        with self.assertRaisesRegex(ValueError, "already be FP32"):
            set_frozen_large_layers_bf16(model, FakeTorch)
        self.assertEqual(model.embedding.cast_calls, [])
        model = FakeModel()
        model.norm.weight = model.embedding.weight
        with self.assertRaisesRegex(ValueError, "shares a normalization"):
            set_frozen_large_layers_bf16(model, FakeTorch)
        self.assertEqual(model.head.cast_calls, [])

    def test_tied_module_cast_once_and_policy_is_idempotent(self):
        model = FakeModel(tied=True)
        result = set_frozen_large_layers_bf16(model, FakeTorch)
        self.assertEqual(result["unique_large_module_count"], 1)
        self.assertEqual(model.embedding.cast_calls, [FakeTorch.bfloat16])
        again = set_frozen_large_layers_bf16(model, FakeTorch)
        self.assertEqual(again["modules_cast_count"], 0)
        self.assertEqual(model.embedding.cast_calls, [FakeTorch.bfloat16])


if __name__ == "__main__":
    unittest.main()
