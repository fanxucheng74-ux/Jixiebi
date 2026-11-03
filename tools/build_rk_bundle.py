"""生成仅含RK串口阶段源码的小型部署包，不部署、不安装依赖、不访问硬件。"""

import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile

from arm_project.paths import PROJECT_ROOT


ARCHIVE_NAME = "jixiebi-rk-serial.tar.gz"
TOP_LEVEL = "Jixiebi"
# 固定文件白名单：不递归复制data/configs，避免把实验、照片或后续私密配置带入板端。
FIXED_FILES = (
    ".arm-project-root",
    "README.md",
    "pyproject.toml",
    "configs/joint_limits.json",
    "data/unified_read_capture.json",
    "data/fixtures/position_replay_20261031.json",
)


def _check_path(root: Path, path: Path) -> None:
    """仅接受根目录内普通文件/目录；拒绝符号链接和Windows目录联接。"""
    relative = path.relative_to(root)
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink() or current.is_junction():
            raise ValueError("deployment sources cannot contain links: " + relative.as_posix())
    if not path.resolve(strict=True).is_relative_to(root):
        raise ValueError("deployment source escapes project root: " + relative.as_posix())


def collect_files(project_root: Path) -> list[Path]:
    """固定白名单加主包内Python源码；返回排序后的普通文件路径。"""
    root = project_root.resolve(strict=True)
    files = []
    for relative in FIXED_FILES:
        path = root / relative
        _check_path(root, path)
        if not path.is_file():
            raise ValueError("required deployment file is not a regular file: " + relative)
        files.append(path)

    source_root = root / "src" / "arm_project"
    _check_path(root, source_root)
    if not source_root.is_dir():
        raise ValueError("missing src/arm_project directory")
    for directory, names, filenames in os.walk(source_root, followlinks=False):
        # 在进入下一层前拒绝链接目录；不读取缓存或其他目录里的内容。
        base = Path(directory)
        for name in names:
            _check_path(root, base / name)
        names[:] = [name for name in names if name != "__pycache__"]
        for name in filenames:
            if not name.endswith(".py"):
                continue
            path = base / name
            _check_path(root, path)
            if not path.is_file():
                raise ValueError("Python source is not a regular file: " + str(path))
            files.append(path)
    if not (source_root / "__init__.py").is_file():
        raise ValueError("missing arm_project/__init__.py")
    return sorted(files, key=lambda path: path.relative_to(root).as_posix())


def build_bundle(output_dir: Path, project_root: Path = PROJECT_ROOT) -> dict:
    """独占新输出目录，保存tar.gz及逐文件SHA256清单；失败产物保留供排查。"""
    root = project_root.resolve(strict=True)
    # 一次读入每份源码：清单哈希与归档内容使用同一份字节，不因并行编辑互相矛盾。
    snapshots = [
        (path.relative_to(root).as_posix(), path.read_bytes()) for path in collect_files(root)
    ]
    output = output_dir if output_dir.is_absolute() else root / output_dir
    output.mkdir(parents=True, exist_ok=False)
    archive_path = output / ARCHIVE_NAME
    with archive_path.open("xb") as raw:
        # 不写入本机路径/用户名；固定时间与权限，让相同输入生成相同归档。
        with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
                for relative, payload in snapshots:
                    member = tarfile.TarInfo(TOP_LEVEL + "/" + relative)
                    member.size = len(payload)
                    member.mode = 0o644
                    member.mtime = 0
                    archive.addfile(member, io.BytesIO(payload))
    with archive_path.open("rb") as stream:
        archive_hash = hashlib.file_digest(stream, "sha256").hexdigest()
    manifest = {
        "schema_version": 1,
        "scope": "RK_SERIAL_STAGE_SOURCE_ONLY",
        "hardware_io": False,
        "archive": ARCHIVE_NAME,
        "top_level": TOP_LEVEL,
        "archive_size_bytes": archive_path.stat().st_size,
        "archive_sha256": archive_hash,
        "file_count": len(snapshots),
        "files": [
            {
                "path": relative,
                "size_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
            for relative, payload in snapshots
        ],
    }
    manifest_path = output / "manifest.json"
    with manifest_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return {
        "archive": str(archive_path.resolve()),
        "manifest": str(manifest_path.resolve()),
        "file_count": len(snapshots),
        "archive_sha256": archive_hash,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="新输出目录；已存在则拒绝")
    args = parser.parse_args(argv)
    try:
        result = build_bundle(args.output_dir)
    except (OSError, ValueError) as error:
        parser.exit(1, "%s: %s\n" % (type(error).__name__, error))
    print("BUNDLE:", result["archive"])
    print("MANIFEST:", result["manifest"])
    print("FILES:", result["file_count"])
    print("SHA256:", result["archive_sha256"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
