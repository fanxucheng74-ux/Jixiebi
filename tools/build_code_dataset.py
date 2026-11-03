"""Build a local, source-grounded instruction/code corpus without hardware or APIs."""

import argparse
import ast
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
SPLITS = ("train", "validation", "test")
SYSTEM = (
    "你为Jixiebi机械臂项目编写Python代码。按既有接口提供模块内实现，"
    "遵守参数验证、有限预算、单串口所有者与失败不自动重试的约定。"
    "协议P不等于实测角度或毫米；软件测试不等于实物成功。"
    "不要自行打开硬件、上传数据或宣称尚未验证的能力。"
)
DOCS = [
    "https://developers.openai.com/api/docs/guides/supervised-fine-tuning",
    "https://developers.openai.com/api/docs/guides/fine-tuning-best-practices",
]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def checked_path(root, relative, prefix):
    path = Path(relative)
    resolved = (root / path).resolve()
    if path.is_absolute() or not resolved.is_relative_to((root / prefix).resolve()):
        raise ValueError("path outside allowed source tree: " + str(relative))
    if resolved.suffix != ".py" or not resolved.is_file():
        raise ValueError("existing Python source required: " + str(relative))
    return resolved


def source_unit(text, symbol):
    """Only complete top-level units: never detach a method or execute a module."""
    tree = ast.parse(text)
    matches = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        and node.name == symbol
    ]
    if len(matches) != 1:
        raise ValueError("one top-level function or class required: " + symbol)
    node = matches[0]
    start = min([node.lineno, *(d.lineno for d in node.decorator_list)])
    code = "".join(text.splitlines(keepends=True)[start - 1 : node.end_lineno]).rstrip() + "\n"
    compile(code, "<dataset-unit>", "exec")  # Syntax only; deliberately never exec().
    context = []
    for item in tree.body:
        if isinstance(item, (ast.Import, ast.ImportFrom)):
            context.append(ast.get_source_segment(text, item))
        elif isinstance(item, (ast.Assign, ast.AnnAssign)):
            try:
                ast.literal_eval(item.value)
            except (ValueError, TypeError, SyntaxError):
                continue
            context.append(ast.get_source_segment(text, item))
    siblings = [
        n.name
        for n in tree.body
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and n != node
    ]
    return code, start, node.end_lineno, "\n".join(context), siblings


def interface_context(text):
    """Show signatures and module contracts without any implementation bodies."""
    tree = ast.parse(text)

    def stub(original):
        node = deepcopy(original)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            description = ast.get_docstring(node)
            node.body = ([ast.Expr(value=ast.Constant(description))] if description else []) + [
                ast.Expr(value=ast.Constant(Ellipsis))
            ]
        else:
            node.body = [
                stub(child)
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
                else child
                for child in node.body
                if isinstance(
                    child,
                    (
                        ast.FunctionDef,
                        ast.AsyncFunctionDef,
                        ast.ClassDef,
                        ast.Assign,
                        ast.AnnAssign,
                    ),
                )
                or (
                    isinstance(child, ast.Expr)
                    and isinstance(child.value, ast.Constant)
                    and isinstance(child.value.value, str)
                )
            ] or [ast.Expr(value=ast.Constant(Ellipsis))]
        return node

    interfaces = [
        ast.unparse(ast.fix_missing_locations(stub(node)))
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
    ]
    contract = ast.unparse(
        ast.Expr(value=ast.Constant(ast.get_docstring(tree) or "无额外模块合同。"))
    )
    return contract + "\n\n" + "\n\n".join(interfaces)


def assign_splits(paths):
    """Keep an entire module in one split, independent of catalog row order."""
    weights = Counter(paths)
    groups = sorted(weights, key=lambda p: (digest(p.encode("utf-8")), p))
    if len(groups) < 3:
        raise ValueError("at least three source modules required for isolated splits")
    target = max(1, round(len(paths) * 0.1))

    def choose(available, reserve):
        choices = {0: ()}
        for group in available:
            for count, selected in list(choices.items()):
                total = count + weights[group]
                proposed = (*selected, group)
                if total not in choices or len(proposed) > len(choices[total]):
                    choices[total] = proposed
        candidates = [v for v in choices.values() if v and len(v) <= len(available) - reserve]
        return min(
            candidates,
            key=lambda v: (abs(sum(weights[g] for g in v) - target), -min(len(v), 3), v),
        )

    testing = choose(groups, 2)
    remaining = [g for g in groups if g not in testing]
    validation = choose(remaining, 1)
    return {
        p: "test" if p in testing else "validation" if p in validation else "train" for p in groups
    }


def write_json(path, value):
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def write_jsonl(path, rows):
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")


def validate_records(records):
    ids, prompts, outputs, groups = set(), set(), {}, {}
    for row in records:
        if row["id"] in ids:
            raise ValueError("duplicate sample id")
        ids.add(row["id"])
        prompt = row["instruction"]
        if prompt in prompts:
            raise ValueError("duplicate task instruction")
        prompts.add(prompt)
        code_hash = digest(row["code"].encode("utf-8"))
        if code_hash in outputs:
            raise ValueError("duplicate code answer")
        outputs[code_hash] = row["split"]
        path = row["source"]["group_key"]
        if path in groups and groups[path] != row["split"]:
            raise ValueError("source module leaked across splits")
        groups[path] = row["split"]
        if row["split"] not in SPLITS:
            raise ValueError("unknown split")
        if [m["role"] for m in row["messages"]] != ["system", "user", "assistant"]:
            raise ValueError("invalid messages roles")
        if any(not m["content"].strip() for m in row["messages"]):
            raise ValueError("empty message")
        if row["messages"][-1]["content"] != "```python\n" + row["code"] + "```":
            raise ValueError("answer differs from source extraction")
        compile(row["code"], row["id"], "exec")
    if set(groups.values()) != set(SPLITS):
        raise ValueError("all three splits must be nonempty")
    return {
        "passed": True,
        "count": len(records),
        "split_counts": dict(Counter(r["split"] for r in records)),
        "category_counts": dict(Counter(r["category"] for r in records)),
        "source_groups": len(groups),
        "whole_module_split_isolation": True,
        "exact_answer_duplicates": 0,
        "syntax_checked_not_executed": True,
        "max_answer_characters": max(len(r["code"]) for r in records),
        "semantic_correctness_proven_by_syntax": False,
    }


def build_dataset(root, catalog_path, output):
    root, catalog_path, output = Path(root).resolve(), Path(catalog_path), Path(output)
    if output.exists():
        raise FileExistsError("dataset output already exists; choose a new version")
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    if catalog.get("schema_version") != 1 or not catalog.get("samples"):
        raise ValueError("invalid catalog")
    canonical = {}
    for sample in catalog["samples"]:
        path = checked_path(root, sample["source_path"], "src/arm_project")
        group = path.relative_to(root).as_posix().casefold()
        sample["source_path"] = canonical.setdefault(group, path.relative_to(root).as_posix())
        sample["group_key"] = group
    splits = assign_splits([s["group_key"] for s in catalog["samples"]])
    records, snapshots = [], {}
    for sample in catalog["samples"]:
        if not re.fullmatch(r"[a-z0-9_]+", sample["id"]):
            raise ValueError("invalid sample id")
        if sample["capability_status"] != "software_tested":
            raise ValueError("code corpus status must not infer physical verification")
        path = checked_path(root, sample["source_path"], "src/arm_project")
        content = path.read_bytes()
        text = content.decode("utf-8")
        code, start, end, preamble, siblings = source_unit(text, sample["symbol"])
        if len(code) > 100000 or re.search(r"sk-(?:proj-)?[A-Za-z0-9_-]{24,}", text):
            raise ValueError("oversize answer or potential live credential")
        tests = sample.get("test_paths", [])
        if not tests:
            raise ValueError("test references required")
        test_evidence = []
        for test in tests:
            tp = checked_path(root, test, "tests")
            test_evidence.append({"path": test, "sha256": digest(tp.read_bytes())})
        instruction = sample["instruction"].strip()
        if len(instruction) < 12:
            raise ValueError("specific task instruction required")
        context = (
            f"工程：Jixiebi，Python>=3.12。文件：{sample['source_path']}；"
            f"待实现顶层对象：{sample['symbol']}。\n"
            "这是在既有模块内补全对象的任务，不是独立脚本；保持同模块其余实现不变。\n"
            f"已存在的同模块对象：{', '.join(siblings) or '无'}。\n"
            f"依赖与限制：{sample['notes']}\n"
            "现有导入及字面量常量（上下文，不要求重复输出）：\n```python\n"
            + preamble
            + "\n```\n现有模块合同和接口（省略实现正文）：\n```python\n"
            + interface_context(text)
            + "\n```\n只输出待实现对象的完整实现，保留装饰器、签名及异常检查。"
        )
        records.append(
            {
                "id": sample["id"],
                "category": sample["category"],
                "split": splits[sample["group_key"]],
                "instruction": instruction,
                "context": context,
                "code": code,
                "messages": [
                    {"role": "system", "content": SYSTEM},
                    {"role": "user", "content": instruction + "\n\n" + context},
                    {"role": "assistant", "content": "```python\n" + code + "```"},
                ],
                "source": {
                    "path": sample["source_path"],
                    "group_key": sample["group_key"],
                    "symbol": sample["symbol"],
                    "start_line": start,
                    "end_line": end,
                    "file_sha256": digest(content),
                    "code_sha256": digest(code.encode("utf-8")),
                },
                "verification": {
                    "status": "source_grounded_software_reference",
                    "test_references": test_evidence,
                    "hardware_tested": False,
                    "note": "关联测试不是逐条覆盖证明；训练效果和生成代码仍需另行评估。",
                },
            }
        )
        snapshots[sample["source_path"]] = content
    records.sort(key=lambda r: r["id"])
    quality = validate_records(records)
    output.mkdir(parents=True, exist_ok=False)
    write_jsonl(output / "records.jsonl", records)
    for split in SPLITS:
        rows = [r for r in records if r["split"] == split]
        write_jsonl(output / f"{split}.messages.jsonl", [{"messages": r["messages"]} for r in rows])
        write_jsonl(
            output / f"{split}.alpaca.jsonl",
            [
                {
                    "instruction": SYSTEM + "\n" + r["instruction"],
                    "input": r["context"],
                    "output": r["messages"][-1]["content"],
                }
                for r in rows
            ],
        )
    # Keep local dependencies and test helpers with the corpus. These files are
    # reference material, never extra fine-tuning messages or held-out answers.
    references = dict(snapshots)
    for base in ("src/arm_project", "tests", "tools"):
        for path in (root / base).rglob("*.py"):
            references[path.relative_to(root).as_posix()] = path.read_bytes()
    for path in (root / "tests/fixtures").rglob("*"):
        if path.is_file():
            references[path.relative_to(root).as_posix()] = path.read_bytes()
    for name in ("pyproject.toml", "data/code_dataset_catalog.json"):
        path = root / name
        if path.is_file():
            references[name] = path.read_bytes()
    references["data/code_dataset_catalog.json"] = catalog_path.read_bytes()
    for relative, content in references.items():
        path = output / "reference_source" / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(content)
    write_json(output / "quality_report.json", quality)
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "purpose": "Jixiebi项目内Python对象实现的指令代码监督微调初始语料",
        "quality": quality,
        "source_catalog_sha256": digest(catalog_path.read_bytes()),
        "source_files": {p: digest(b) for p, b in sorted(snapshots.items())},
        "reference_files_count": len(references),
        "files_sha256": {
            str(p.relative_to(output)).replace("\\", "/"): digest(p.read_bytes())
            for p in sorted(output.rglob("*"))
            if p.is_file()
        },
        "official_format_references": DOCS,
        "target_model": None,
        "training_launched": False,
        "hardware_io": False,
        "video_included": False,
        "limitations": [
            "同一源文件整体分组，不能保证导入依赖或相似算法之间完全隔离；评估只代表本小语料。",
            "测试映射与语法检查不证明全部语义正确，也不证明模型或实物控制精度。",
            "代码上下文需配合同一版本仓库；不是每条回答都可单独运行的脚本。",
            "仅训练指定模型的权重/适配器；不能由此宣称当前Codex应用永久学会项目。",
            "尚未选定训练平台、基础模型、上下文上限、tokenizer和许可证要求。",
            "reference_source及test文件仅供离线核验，不要混入训练集；test不用于调参。",
            "不属于视觉检测标注集或机器人模仿学习的同步观测动作数据集。",
        ],
    }
    write_json(output / "manifest.json", manifest)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=ROOT / "data/code_dataset_catalog.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = build_dataset(ROOT, args.catalog, args.output)
    print(json.dumps(result["quality"], ensure_ascii=False))
    print("DATASET:", args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
