"""从公开源码生成RK学习包；只读取固定白名单，不访问设备或收集实验日志。"""

import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile
import uuid


ROOT = Path(__file__).resolve().parents[1]
ARCHIVE_NAME = "jixiebi-learning.tar.gz"

# 发布的是可重新构建的源码，而不是某台电脑生成的二进制或历史artifacts包。
# 显式列出24项，新增文件不会因为目录扫描而自动混进部署包。
BUNDLE_FILES = (
    "native/servo_protocol/include/servo_codec.h",
    "native/servo_protocol/include/servo_receiver.h",
    "native/servo_protocol/src/servo_codec.c",
    "native/servo_protocol/src/servo_receiver.c",
    "native/arm_driver/include/position_reader.hpp",
    "native/arm_driver/include/arm_state.hpp",
    "native/arm_driver/src/position_reader.cpp",
    "native/arm_driver/src/arm_state.cpp",
    "native/apps/query_frame_demo.c",
    "native/apps/reply_parse_demo.c",
    "native/apps/receiver_demo.c",
    "native/apps/read_position.cpp",
    "native/apps/read_arm_state.cpp",
    "tools/build_position_reader.sh",
    "tools/build_arm_state.sh",
    "ros2_ws/src/jixiebi_interfaces/CMakeLists.txt",
    "ros2_ws/src/jixiebi_interfaces/package.xml",
    "ros2_ws/src/jixiebi_interfaces/msg/ServoStateRaw.msg",
    "ros2_ws/src/jixiebi_interfaces/msg/ServoStatesRaw.msg",
    "ros2_ws/src/jixiebi_interfaces/srv/GetLessonInfo.srv",
    "ros2_ws/src/jixiebi_interfaces/action/CountSteps.action",
    "ros2_ws/src/jixiebi_hardware/CMakeLists.txt",
    "ros2_ws/src/jixiebi_hardware/package.xml",
    "ros2_ws/src/jixiebi_hardware/src/servo_state_publisher.cpp",
)


def build_bundle(root: Path, output: Path) -> dict:
    """预检全部输入后创建新输出目录；错误时不覆盖已有目录或文件。"""
    root = root.resolve(strict=True)
    destination = output if output.is_absolute() else root / output
    if destination.exists() or destination.is_symlink():
        raise FileExistsError("输出已存在，请使用新的--output目录")

    # 输出必须在本仓库artifacts之内；父目录的符号链接也必须经过resolve核对。
    artifact_root = (root / "artifacts").resolve()
    destination = destination.resolve()
    if not artifact_root.is_relative_to(root):
        raise ValueError("artifacts目录的链接指向仓库外，拒绝输出")
    if destination == artifact_root or not destination.is_relative_to(artifact_root):
        raise ValueError("--output必须是仓库artifacts下的新子目录")

    payloads = []
    for relative in BUNDLE_FILES:
        source = (root / relative).resolve(strict=True)
        if not source.is_relative_to(root):
            raise ValueError(f"输入文件或其父目录链接越出仓库: {relative}")
        if not source.is_file():
            raise ValueError(f"输入不是普通文件: {relative}")
        # 只读取一次，打包内容与清单哈希来自相同字节，避免二次读取发生不一致。
        payloads.append((relative, source.read_bytes()))

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir(exist_ok=False)
    archive = destination / ARCHIVE_NAME
    # 固定gzip/tar时间戳和权限：同一组源码可以生成相同压缩包哈希，不写入本机用户名。
    with archive.open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as package:
                for relative, content in payloads:
                    item = tarfile.TarInfo(relative)
                    item.size = len(content)
                    item.mode = 0o755 if relative.endswith(".sh") else 0o644
                    item.mtime = 0
                    package.addfile(item, io.BytesIO(content))

    archive_sha256 = hashlib.sha256(archive.read_bytes()).hexdigest()
    manifest = {
        "schema": "jixiebi.learning_bundle.v1",
        "archive": ARCHIVE_NAME,
        "archive_sha256": archive_sha256,
        "hardware_io": False,
        "files": [
            {"path": relative, "bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}
            for relative, content in payloads
        ],
    }
    (destination / "SHA256SUMS").write_text(
        f"{archive_sha256}  {ARCHIVE_NAME}\n", encoding="utf-8", newline="\n"
    )
    (destination / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts") / f"rk_learning_{uuid.uuid4().hex[:12]}",
        help="仓库artifacts下的新目录；相对路径以仓库根为基准，已有目录不会覆盖",
    )
    args = parser.parse_args()
    try:
        manifest = build_bundle(ROOT, args.output)
    except (OSError, ValueError) as error:
        parser.exit(1, f"ERROR: {error}\n")
    destination = (args.output if args.output.is_absolute() else ROOT / args.output).resolve()
    print("BUNDLE:", destination / ARCHIVE_NAME)
    print("SHA256:", manifest["archive_sha256"])
    print("FILES:", len(manifest["files"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
