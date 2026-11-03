"""Read bounded, reviewed project sources for local inference, without devices or ML."""

import ast
from collections import Counter
import hashlib
import math
from pathlib import Path
import re
import stat


TOPICS = ("servo", "tcp", "vision", "motion", "auto")
ALLOWLIST = (
    "docs/PROJECT_ASSISTANT_SCOPE.md",
    "docs/ARCHITECTURE.md",
    "docs/UNIFIED_DRIVER_GUIDE.md",
    "docs/CONTINUOUS_MOTION_20261101.md",
    "docs/KINEMATICS_NOMINAL.md",
    "docs/VISION_WORKSPACE.md",
    "docs/TASK_API.md",
    "src/arm_project/servo_protocol.py",
    "src/arm_project/arm_link.py",
    "src/arm_project/arm_state.py",
    "src/arm_project/unified_arm.py",
    "src/arm_project/protocol.py",
    "src/arm_project/command_ledger.py",
    "src/arm_project/kinematics.py",
    "src/arm_project/pick_planning.py",
    "src/arm_project/timed_cycle.py",
    "src/arm_project/arm_tasks.py",
    "src/arm_project/task_adapter.py",
    "src/arm_project/vision/colors.py",
    "src/arm_project/vision/workspace.py",
)
SCOPE = ALLOWLIST[0]
MAX_FILE_BYTES = 256 * 1024
MAX_CHUNK_CHARS = 2200
TOPIC_PATHS = {
    "servo": ("servo_protocol.py", "arm_link.py", "arm_state.py", "UNIFIED_DRIVER_GUIDE.md"),
    "tcp": ("/protocol.py", "command_ledger.py", "ARCHITECTURE.md"),
    "vision": ("/vision/", "VISION_WORKSPACE.md"),
    "motion": (
        "kinematics.py",
        "pick_planning.py",
        "timed_cycle.py",
        "arm_tasks.py",
        "task_adapter.py",
        "CONTINUOUS_MOTION_20261101.md",
        "KINEMATICS_NOMINAL.md",
        "TASK_API.md",
    ),
}
TOPIC_WORDS = {
    "servo": ("舵机", "串口", "总线", "prad", "pdst", "zlink", "servo"),
    "tcp": ("tcp", "socket", "wifi", "端口", "客户端", "服务端", "通信协议"),
    "vision": ("视觉", "相机", "opencv", "颜色", "木块", "hsv", "标定"),
    "motion": ("运动学", "坐标", "轨迹", "下沉", "夹取", "ik", "fk", "关节"),
}


def _bounded_int(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")


def _is_link(path):
    info = path.lstat()
    return path.is_symlink() or bool(
        getattr(info, "st_file_attributes", 0)
        & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
    )


def _checked_path(root, relative):
    candidate = root
    for part in Path(relative).parts:
        candidate = candidate / part
        try:
            if _is_link(candidate):
                raise ValueError("linked project source is not allowed: " + relative)
        except FileNotFoundError:
            return None
    resolved = candidate.resolve(strict=True)
    if not resolved.is_relative_to(root):
        raise ValueError("source escapes project root: " + relative)
    if not resolved.is_file():
        raise ValueError("reviewed source is not a regular file: " + relative)
    return resolved


def _terms(text):
    words = re.findall(r"[A-Za-z_][A-Za-z_0-9]*", text.lower())
    for run in re.findall(r"[\u3400-\u9fff]+", text):
        words.extend(run[index : index + 2] for index in range(max(1, len(run) - 1)))
    return Counter(words)


def _chunks(relative, raw):
    lines = raw.decode("utf-8").splitlines()
    digest = hashlib.sha256(raw).hexdigest()
    start = 0
    while start < len(lines):
        end, used = start, 0
        while end < min(len(lines), start + 45):
            length = len(lines[end]) + 1
            if used + length > MAX_CHUNK_CHARS:
                break
            used += length
            end += 1
        if end == start:
            # A single excessive source line is omitted, never silently truncated.
            start += 1
            continue
        yield {
            "path": relative,
            "start_line": start + 1,
            "end_line": end,
            "text": "\n".join(lines[start:end]),
            "sha256": digest,
        }
        if end == len(lines):
            break
        start = max(start + 1, end - 8)


def _intent_symbols(question, topic):
    """Expand protocol intent to existing identifiers, never to protocol answers."""
    symbols = set(re.findall(r"\b[a-z_][a-z_0-9]*\b", question.lower()))
    if topic == "servo":
        if re.search(r"#\d{3}P\d{4}T\d{4}!|运动帧|定位帧|P/T", question, re.IGNORECASE):
            symbols.add("make_move_command")
        if any(word in question for word in ("回包", "回复", "解析")):
            symbols.add("parse_position_reply")
        elif "查询" in question or "prad" in question.lower():
            symbols.add("make_query")
    return symbols


def _focused_chunks(relative, raw, symbols):
    """Keep short, relevant top-level functions intact instead of partial windows."""
    if not relative.endswith(".py"):
        return []
    text = raw.decode("utf-8")
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return []
    lines = text.splitlines()
    result = []
    for node in tree.body:
        if (
            not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            or node.name not in symbols
        ):
            continue
        first = min([node.lineno, *(item.lineno for item in node.decorator_list)])
        body = "\n".join(lines[first - 1 : node.end_lineno])
        if len(body) > MAX_CHUNK_CHARS:
            continue
        result.append(
            {
                "path": relative,
                "start_line": first,
                "end_line": node.end_lineno,
                "text": body,
                "sha256": hashlib.sha256(raw).hexdigest(),
                "symbol": node.name,
            }
        )
    return result


def render_source(source):
    """Render exact whole source lines with their original one-based positions."""
    header = f"[{source['id']}] {source['path']}:L{source['start_line']}-L{source['end_line']}"
    lines = source["text"].split("\n")
    return (
        header
        + "\n"
        + "\n".join(f"{source['start_line'] + index}: {line}" for index, line in enumerate(lines))
    )


def render_sources(sources):
    """Rebuild the visible context after dropping whole evidence excerpts."""
    return "\n\n".join(render_source(source) for source in sources)


def build_context(root, question, topic="servo", max_chars=9000, max_sources=6):
    """Re-read allowlisted files and retrieve line-cited excerpts for one question.

    Lexical ranking is deliberately inspectable. No embeddings, model training,
    source imports, external service, directory traversal, or persistent index.
    """
    if type(question) is not str or not question.strip() or len(question) > 4000:
        raise ValueError("question must contain 1..4000 characters")
    if type(topic) is not str or topic not in TOPICS:
        raise ValueError("unknown topic")
    _bounded_int(max_chars, "max_chars", 256, 24000)
    _bounded_int(max_sources, "max_sources", 1, 12)
    root = Path(root).absolute()
    if _is_link(root):
        raise ValueError("linked project root is not allowed")
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError("project root must be a directory")
    effective_topic = topic
    if topic == "auto":
        matches = {
            name: sum(word in question.lower() for word in words)
            for name, words in TOPIC_WORDS.items()
        }
        effective_topic = max(matches, key=matches.get) if any(matches.values()) else "auto"
    candidates, scanned = [], []
    scope = None
    query_terms = _terms(question)
    intent_symbols = _intent_symbols(question, effective_topic)
    query_terms.update(intent_symbols)
    for relative in ALLOWLIST:
        path = _checked_path(root, relative)
        if path is None:
            continue
        with path.open("rb") as stream:
            raw = stream.read(MAX_FILE_BYTES + 1)
        if len(raw) > MAX_FILE_BYTES:
            raise ValueError("reviewed file exceeds byte limit: " + relative)
        scanned.append(relative)
        chunks = list(_chunks(relative, raw))
        if relative == SCOPE:
            scope = chunks[0] if chunks else None
            continue
        chunks.extend(_focused_chunks(relative, raw, intent_symbols))
        boost = 8 if any(part in relative for part in TOPIC_PATHS.get(effective_topic, ())) else 0
        for chunk in chunks:
            counts = _terms(chunk["text"])
            lexical = sum(1 + math.log1p(counts[term]) for term in query_terms if counts.get(term))
            if lexical or boost:
                focused = 30 if chunk.get("symbol") in intent_symbols else 0
                candidates.append((lexical + boost + focused, chunk))
    candidates.sort(key=lambda item: (-item[0], item[1]["path"], item[1]["start_line"]))
    if not candidates:
        raise ValueError("no relevant reviewed source; choose a topic or reword the question")
    if scope is not None and max_sources < 2:
        raise ValueError("scope plus topical evidence requires at least two sources")
    selected, rendered = [], []

    def add(chunk, mandatory=False):
        if len(selected) == max_sources:
            return False
        same_file = [row for row in selected if row["path"] == chunk["path"]]
        if len(same_file) >= 2 or any(
            row["start_line"] <= chunk["end_line"] and chunk["start_line"] <= row["end_line"]
            for row in same_file
        ):
            return False
        row = dict(chunk, id="S" + str(len(selected) + 1))
        available = max_chars - sum(len(value) for value in rendered) - 2 * len(rendered)
        while (
            not mandatory
            and len(render_source(row)) > available
            and row["end_line"] > row["start_line"]
        ):
            row["text"] = row["text"].rsplit("\n", 1)[0]
            row["end_line"] -= 1
        text = render_source(row)
        if len(text) > available:
            if mandatory:
                raise ValueError("context budget cannot fit mandatory scope")
            return False
        selected.append(row)
        rendered.append(text)
        return True

    if scope is not None:
        add(scope, mandatory=True)
    # Keep matching short protocol functions ahead of broad descriptive prose.
    if effective_topic == "servo":
        protocol = [
            chunk for _, chunk in candidates if chunk["path"].endswith("/servo_protocol.py")
        ]
        focused = [chunk for chunk in protocol if chunk.get("symbol") in intent_symbols]
        for chunk in focused[:2] if focused else protocol[:1]:
            add(chunk)
    for _, chunk in candidates:
        add(chunk)
    if not any(row["path"] != SCOPE for row in selected):
        raise ValueError("context budget cannot fit topical evidence after mandatory scope")
    context = render_sources(selected)
    if len(context) > max_chars:
        raise RuntimeError("context budget invariant failed")
    return {
        "question": question,
        "topic": topic,
        "effective_topic": effective_topic,
        "retrieval_method": "allowlist_lexical_cjk_bigrams_v1",
        "context_text": context,
        "sources": selected,
        "scanned_files": scanned,
    }
