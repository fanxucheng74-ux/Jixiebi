"""Corpus extraction/provenance/split tests never import or execute extracted code."""

from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from tools.build_code_dataset import (
    assign_splits,
    build_dataset,
    checked_path,
    interface_context,
    source_unit,
    validate_records,
)


class CodeDatasetTests(unittest.TestCase):
    def fixture(self, root):
        (root / "src/arm_project").mkdir(parents=True)
        (root / "tests").mkdir()
        (root / "tests/test_fixture.py").write_text("# offline test reference\n", encoding="utf-8")
        samples = []
        for index in range(4):
            path = f"src/arm_project/m{index}.py"
            (root / path).write_text(
                f"raise RuntimeError('NEVER IMPORT')\nLIMIT={index}\n"
                f"def operation{index}(value):\n    return value + LIMIT\n",
                encoding="utf-8",
            )
            samples.append(
                {
                    "id": f"sample_{index}",
                    "category": "fixture",
                    "instruction": f"实现既有模块内第{index}个带边界参考的离线运算函数。",
                    "source_path": path,
                    "symbol": f"operation{index}",
                    "test_paths": ["tests/test_fixture.py"],
                    "capability_status": "software_tested",
                    "notes": "仅测试生成工具，不执行抽取代码。",
                }
            )
        catalog = root / "catalog.json"
        catalog.write_text(json.dumps({"schema_version": 1, "samples": samples}), encoding="utf-8")
        return catalog

    def test_extracts_full_decorated_unit_and_never_executes(self):
        text = "raise RuntimeError('NO')\n@unknown_decorator\nclass C:\n    x = '中文'\n"
        code, start, end, _, _ = source_unit(text, "C")
        self.assertTrue(code.startswith("@unknown_decorator\nclass C:"))
        self.assertEqual((start, end), (2, 4))
        with self.assertRaises(ValueError):
            source_unit(text, "C.x")

    def test_roundtrip_source_hashes_formats_and_no_execution(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            catalog = self.fixture(root)
            out = root / "dataset"
            report = build_dataset(root, catalog, out)
            self.assertTrue(report["quality"]["passed"])
            self.assertEqual(report["quality"]["count"], 4)
            self.assertFalse(report["training_launched"])
            rows = [
                json.loads(line)
                for line in (out / "records.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            for row in rows:
                path = out / "reference_source" / row["source"]["path"]
                self.assertEqual(path.read_bytes(), (root / row["source"]["path"]).read_bytes())
            for split in ("train", "validation", "test"):
                messages = [
                    json.loads(line)
                    for line in (out / f"{split}.messages.jsonl")
                    .read_text(encoding="utf-8")
                    .splitlines()
                ]
                self.assertTrue(all(set(row) == {"messages"} for row in messages))
            self.assertEqual(
                validate_records(rows)["split_counts"], report["quality"]["split_counts"]
            )

    def test_refuses_overwrite_without_altering_existing_file(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            catalog = self.fixture(root)
            output = root / "existing"
            output.mkdir()
            (output / "keep").write_bytes(b"immutable")
            with self.assertRaises(FileExistsError):
                build_dataset(root, catalog, output)
            self.assertEqual((output / "keep").read_bytes(), b"immutable")

    def test_rejects_escape_missing_tests_and_fake_physical_label(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            catalog = self.fixture(root)
            with self.assertRaises(ValueError):
                checked_path(root, "../outside.py", "src/arm_project")
            original = json.loads(catalog.read_text(encoding="utf-8"))
            for field, value in (("test_paths", []), ("capability_status", "hardware_verified")):
                data = deepcopy(original)
                data["samples"][0][field] = value
                catalog.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    build_dataset(root, catalog, root / "unused")
                self.assertFalse((root / "unused").exists())

    def test_group_split_deterministic_and_isolated(self):
        paths = [f"m{i}.py" for i in range(10)]
        result = assign_splits(paths)
        self.assertEqual(result, assign_splits(list(reversed(paths)) + paths))
        self.assertEqual(set(result.values()), {"train", "validation", "test"})
        with self.assertRaises(ValueError):
            assign_splits(["a.py", "b.py"])
        self.assertEqual(set(assign_splits(["a", "b"] + ["c"] * 18).values()), set(result.values()))

    def test_balances_by_samples_while_keeping_modules_together(self):
        paths = [f"m{i}.py" for i in range(17) for _ in range(1 + i % 5)]
        split = assign_splits(paths)
        for key in ("test", "validation"):
            count = sum(split[p] == key for p in paths)
            self.assertLessEqual(abs(count - round(len(paths) * 0.1)), 1)

    def test_interface_preserves_signature_contract_not_body(self):
        text = (
            '"""Injected driver contract."""\n'
            "class C:\n    def move(self, value: int = 4):\n        return value + 937\n"
        )
        context = interface_context(text)
        self.assertIn("Injected driver contract", context)
        self.assertIn("def move(self, value: int=4):", context)
        self.assertNotIn("937", context)
        compile(context, "<interface>", "exec")

    def test_aliases_of_one_module_cannot_cross_splits(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            catalog = self.fixture(root)
            data = json.loads(catalog.read_text(encoding="utf-8"))
            path = root / "src/arm_project/m0.py"
            path.write_text(
                path.read_text(encoding="utf-8") + "\ndef another():\n    return 'alias'\n"
            )
            other = deepcopy(data["samples"][0])
            other.update(
                id="alias_sample",
                symbol="another",
                source_path="src/arm_project/../arm_project/m0.py",
                instruction="实现同一规范路径下第二个独立的离线参考对象。",
            )
            data["samples"].append(other)
            catalog.write_text(json.dumps(data))
            build_dataset(root, catalog, root / "out")
            rows = [
                json.loads(line)
                for line in (root / "out/records.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            original = next(r for r in rows if r["id"] == "sample_0")
            alias = next(r for r in rows if r["id"] == "alias_sample")
            self.assertEqual(original["split"], alias["split"])
            self.assertEqual(original["source"]["path"], alias["source"]["path"])

    def test_quality_rejects_duplicate_answers_and_cross_split_module(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            catalog = self.fixture(root)
            build_dataset(root, catalog, root / "out")
            rows = [
                json.loads(line)
                for line in (root / "out/records.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            copied = deepcopy(rows)
            copied[1]["code"] = copied[0]["code"]
            with self.assertRaises(ValueError):
                validate_records(copied)
            copied = deepcopy(rows)
            first = next(r for r in copied if r["split"] == "train")
            second = next(r for r in copied if r["split"] == "test")
            second["source"]["group_key"] = first["source"]["group_key"]
            with self.assertRaises(ValueError):
                validate_records(copied)


if __name__ == "__main__":
    unittest.main()
