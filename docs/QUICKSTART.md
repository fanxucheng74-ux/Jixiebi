# 不接硬件：安装与离线快速开始

本页命令在项目仓库根目录执行，只解析示例计划，不打开串口或摄像头。项目声明需要 Python 3.12+；公开版目前已用 Windows Python 3.14 验证安装及离线测试，其他平台和版本的完整复现尚待验证。联网安装依赖只下载软件包，不会自动控制机械臂。

## Windows PowerShell

先在文件管理器进入你的项目文件夹，在该目录打开 PowerShell，或使用 `cd` 切换到仓库根目录，然后：

```powershell
# 创建 Python 虚拟环境；仅首次需要。
py -3.14 -m venv .venv

# 安装项目、开发检查工具及测试用的视觉依赖；路径 . 表示当前仓库。
.\.venv\Scripts\python.exe -m pip install -e '.[vision,dev]'

# 只解析示例计划；不指定 --execute 或串口。
.\.venv\Scripts\python.exe -m arm_project.cli.arm_cli --plan data/unified_read_capture.json
```

预期输出包括 `PLAN: 2 steps, 0 exact motion entries` 与 `DRY_RUN: no serial connection or motion`。具体步骤包括 `read_state` 和 `capture_pose`，此时只展示计划，**不读取真实状态也不产生姿态记录**。如果 `py -3.14` 不可用，先安装 Python 3.14 并让 Windows Python Launcher 能找到它。

## Ubuntu / WSL

在 Linux/WSL 终端进入项目仓库根目录：

```bash
# 系统需提供 Python 3.12 和 venv；以下在仓库根目录执行。
python3.12 -m venv .venv
.venv/bin/python -m pip install -e '.[vision,dev]'

# 离线解析，不连接设备、不执行动作。
.venv/bin/python -m arm_project.cli.arm_cli --plan data/unified_read_capture.json
```

在某些 Ubuntu 系统上可能需要安装与 Python 版本匹配的 `venv` 包；已有厂商镜像的开发板应先核对包依赖，避免盲目升级系统。RK 的 ROS Python 环境不一定与本虚拟环境兼容，板端步骤见[硬件与 ROS 2](HARDWARE_AND_ROS2.md)。

## 离线检查

以下命令在仓库根目录运行，使用上面建好的环境；`<python>` 替换为对应平台的 `.venv\Scripts\python.exe` 或 `.venv/bin/python`：

```text
<python> -m ruff check src tests tools
<python> -m ruff format --check src tests tools
<python> -m unittest discover -s tests -v
```

这些是项目源码的常规离线检查。某些本机历史验收还检查私有实验记录、迁移哈希和硬件记录；发布包若不包含这些资料，不能把它们算作公开检出已验证通过。测试发现缺少 fixture 或平台依赖时，请保留完整报错并查阅仓库的 CI 配置，勿将失败改写为通过。

## 常见问题

- `No module named arm_project`：确认位于仓库根目录，并用同一个虚拟环境完成 `pip install -e '.[vision,dev]'`。
- 提示找不到计划文件：确认当前目录为仓库根目录，且 `data/unified_read_capture.json` 已存在。
- `DRY_RUN` 不会让真实机械臂运动，这是预期行为。真实硬件教程和模型证明是后续独立步骤。
