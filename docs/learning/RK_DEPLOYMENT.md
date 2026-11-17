# 从公开源码部署到RK：首次准备与日常运行

本页统一第四至第六课的源码来源。你可以让RK直接克隆GitHub仓库，也可以在电脑生成精简学习包后传入RK；选择一种即可。两种方式都将RK源码放在`$HOME/Jixiebi`，示例用户名、IP和克隆目录必须按自己的环境填写。

本页的构建步骤不读取硬件。末尾只读命令会发送`PRAD`查询；不发送运动目标，也不提供一键抓放。

## 1. 分清三个终端

| 位置 | 负责什么 | 路径示例 |
| --- | --- | --- |
| 电脑WSL/Linux | 编辑源码、编译、运行无硬件测试 | `~/Jixiebi` |
| 电脑Windows PowerShell | 打包当前源码，执行`scp`和`ssh` | 当前克隆目录，由`Get-Location`取得 |
| RK的SSH终端 | 用RK工具链编译，访问RK的USB串口 | `$HOME/Jixiebi` |

电脑x86_64与RK aarch64的可执行程序不能直接互换；传源码，在RK原生编译。若采用WSL打包，输出位于WSL文件系统；Windows可通过`\\wsl.localhost\<发行版名>\...`访问，但更简单的方式是在同一个能访问克隆目录的终端打包和传输。

## 2. 前置环境

- RK安装适配其硬件的Linux系统，以及`gcc`、`g++`、`bash`。第五课本身不需要ROS或Python虚拟环境。
- 第六课需要Ubuntu24.04/ROS 2 Jazzy、`ament_cmake`、`rclcpp`、`rosidl_default_generators`与`colcon`。使用ROS匹配的系统Python生成消息。
- 开发者用WSL22.04/Humble与RK24.04/Jazzy分别验证过本地构建/消息；本课在同一RK上收发。正式PC↔RK联机应统一发行版与接口，不默认Humble/Jazzy跨版本DDS互通。
- ZLink或同类适配器接在RK的USB Host口，舵机按厂商规格独立供电。USB枚举成功不能证明舵机已上电。

若工具缺失，先补对应系统/ROS版本的依赖；厂商镜像存在hold包时保留错误信息，避免不加区分地升级整机或解锁所有软件包。

## 3. 方式A：RK直接克隆完整源码

先在电脑自己的终端用实际账号和地址SSH登录RK。以下代码在**RK的SSH终端**执行：

```bash
# 首次克隆；若同名目录已存在，git会拒绝覆盖，先检查它的来源和本地修改。
git clone https://github.com/fanxucheng74-ux/Jixiebi.git "$HOME/Jixiebi"

# 确认当前源码版本，便于记录验收结果。
cd "$HOME/Jixiebi"
git log -1 --oneline
```

成功后直接前往第5节编译。已有克隆时先运行`git status --short`检查修改，再按自己的分支管理方式更新；不要用强制重置清除自己的学习代码或实验记录。

## 4. 方式B：电脑生成学习包，再传入RK

学习包从当前仓库源码生成，使用固定白名单，只包含本课C/C++源码、两个ROS包和构建脚本；不包含测试、历史Python、实验日志、现场配置或编译结果。需要完整测试时在电脑完整克隆中运行。

### 电脑WSL/Linux：生成包

```bash
# 进入你实际的克隆目录；以下假设位于家目录。
cd "$HOME/Jixiebi"

# 输出目录必须是新的；重复打包时使用另一目录名，保留旧记录。
python3 tools/build_learning_bundle.py --output artifacts/rk-learning
```

成功后输出目录中应有`jixiebi-learning.tar.gz`、`SHA256SUMS`和`manifest.json`。修改源码后重新打包才能部署修改版，旧压缩包不会自动更新。

如果选择在同一个WSL/Linux终端传输，继续执行：

```bash
# 在提示后实际输入自己的账号/地址，不在聊天或源码中保存密码。
read -r -p 'RK的SSH用户名: ' RK_SSH_USER
read -r -p 'RK当前IP或主机名: ' RK_SSH_HOST
if [ -n "$RK_SSH_USER" ] && [ -n "$RK_SSH_HOST" ]; then
  ssh "${RK_SSH_USER}@${RK_SSH_HOST}" 'mkdir -p ~/jixiebi_upload' &&
  scp artifacts/rk-learning/jixiebi-learning.tar.gz artifacts/rk-learning/SHA256SUMS \
    artifacts/rk-learning/manifest.json "${RK_SSH_USER}@${RK_SSH_HOST}:jixiebi_upload/" &&
  ssh "${RK_SSH_USER}@${RK_SSH_HOST}"
else
  echo '用户名和地址不能为空，未进行传输'
fi
```

SSH登录成功后进入下方“RK的SSH终端：校验并首次解包”，不必再执行Windows打包分支。

### 电脑Windows PowerShell：生成并传输

以下从**Windows中可访问的完整克隆根目录**开始。若已在WSL生成包，可改用WSL的`scp`传输，或在Windows指向那份输出；不要在RK终端里输入电脑盘符路径。

```powershell
# 查看当前位置；这里应能看到native、ros2_ws和tools目录。
$repoRoot = (Get-Location).Path

# Python工具只使用标准库；Windows Python Launcher选择本机已安装的Python 3。
py -3 tools/build_learning_bundle.py --output artifacts/rk-learning
if ($LASTEXITCODE -ne 0) { throw '打包失败，请先处理上方错误' }

# 这是输入提示，冒号出现后要实际输入账号与地址；提示文本本身不是变量值。
$rkUser = Read-Host '输入RK的SSH用户名'
$rkHost = Read-Host '输入RK当前IP或主机名'
if ([string]::IsNullOrWhiteSpace($rkUser) -or [string]::IsNullOrWhiteSpace($rkHost)) {
    throw '用户名与地址不能为空'
}

# 远程创建接收目录，完成后返回Windows，不会留在RK的交互shell。
ssh "${rkUser}@${rkHost}" 'mkdir -p ~/jixiebi_upload'
if ($LASTEXITCODE -ne 0) { throw '建立RK接收目录失败' }

# 传递源码包和本次生成的校验/清单，不传电脑编出的二进制。
$bundleDir = Join-Path $repoRoot 'artifacts/rk-learning'
scp (Join-Path $bundleDir 'jixiebi-learning.tar.gz') (Join-Path $bundleDir 'SHA256SUMS') (Join-Path $bundleDir 'manifest.json') "${rkUser}@${rkHost}:jixiebi_upload/"
if ($LASTEXITCODE -ne 0) { throw '传输失败，请先处理错误' }

# 这条才进入RK交互终端，随后所有$HOME路径指RK用户的家目录。
ssh "${rkUser}@${rkHost}"
```

`Read-Host`后直接回车会得到空字符串。你也可以直接赋值给变量，但应填写自己的地址；密码只在自己的终端输入，不放入命令、源码或提交记录。

### RK的SSH终端：校验并首次解包

```bash
# 进入接收目录，按电脑生成的清单核对包内容。
cd "$HOME/jixiebi_upload" &&
sha256sum -c SHA256SUMS
```

预期出现`jixiebi-learning.tar.gz: OK`。只有校验通过后，再执行：

```bash
# 只建立新源码目录；存在同名目录时本条失败，后续解包不执行。
mkdir "$HOME/Jixiebi" &&
tar --keep-old-files -xzf "$HOME/jixiebi_upload/jixiebi-learning.tar.gz" -C "$HOME/Jixiebi"
```

`--keep-old-files`防止覆盖已有同名文件。已经通过方式A克隆或此前完整部署过时，跳过这个步骤；先检查和合并自己的源码修改，不删除目录重来。

## 5. RK编译C++查询程序

在**RK的SSH终端**执行：

```bash
cd "$HOME/Jixiebi" &&
# 本脚本同时生成第四、第五课程序，不打开串口。
bash tools/build_arm_state.sh
```

最后应有`BUILD_OK: build/cpp_lesson05/read_arm_state`。脚本继续把`.c`按C11编译，把C++按C++17编译，并使用头文件里的`extern "C"`链接两种语言。

## 6. RK只读查询：一次只运行一个所有者

先按实物检查连接、供电、权限与串口名称：

```bash
# 这些只枚举设备和用户组，不发送舵机查询。
lsusb
ls -l /dev/ttyUSB* /dev/serial/by-id/ 2>/dev/null
id
```

`/dev/ttyUSB0`是以下示例端口，若现场不同须替换。普通用户通常需属于设备所属组，例如`dialout`；变更用户组后重新登录，不使用`chmod 777`解决权限。

在RK先运行第四课单轴，或第五课六轴，选择一个即可；不要同时运行，也不要与ROS硬件节点同时运行：

```bash
cd "$HOME/Jixiebi" &&
if fuser -s /dev/ttyUSB0; then
  fuser -v /dev/ttyUSB0
  echo '请先在原窗口Ctrl+C退出已知的串口程序，再查询'
else
  # 只查询002一次，发送的是PRAD，不是运动目标。
  ./build/cpp_lesson04/read_position --port /dev/ttyUSB0 --id 2
fi
```

单轴成功应显示`SOURCE=SERIAL_TTY`、`SERVO_ID=2`和`POSITION_P=...`。六轴只需将上面的程序行替换为：

```bash
# 同一个reader顺序查询000～005，各查询一次；失败停止，不自动补查。
./build/cpp_lesson05/read_arm_state --port /dev/ttyUSB0
```

完整成功末尾为`BATCH_STATUS=OK`、`VALID_COUNT=6`；失败/未查询的P显示`--`。P仍为协议单位，不等于弧度或毫米，六个读数也不是同步快照。

`fuser`只是占用快照，程序还会获取协作`flock`；锁不能阻止不遵守锁的外部程序。超时或DTR/RTS配置错误保留输出进行排查，不用`--pty-simulation`绕过真实设备检查。

## 7. RK构建与启动ROS节点

前置条件为ROS 2 Jazzy及构建依赖已经安装。以下在**RK的SSH终端**执行：

```bash
# 在当前终端退出已激活的Python虚拟环境，使用ROS匹配的系统Python。
if type deactivate >/dev/null 2>&1; then deactivate; fi
source /opt/ros/jazzy/setup.bash
cd "$HOME/Jixiebi/ros2_ws"

# 两个包依次构建，限制并行；CMake从同级native链接现有驱动。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select jixiebi_interfaces jixiebi_hardware --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

预期`Summary: 2 packages finished`。不要只复制`ros2_ws`而缺少同级`native`；学习包和完整克隆都保留正确布局。编译失败时保留完整错误，先处理该步。

然后按[第六课](CPP_LESSON06_ROS_PUBLISHER.md#12-rk第一个ssh窗口启动节点)启动唯一发布节点，并在同一RK的第二SSH窗口订阅。每个新窗口都需要加载`/opt/ros/jazzy/setup.bash`和`$HOME/Jixiebi/ros2_ws/install/setup.bash`。订阅端不打开串口，`echo --once`退出也不会停止发布者。

## 8. 日常开发不必重新部署

源码与依赖未变化时，进入已有目录直接执行课程命令。修改源码后先编译；从电脑部署修改版时重新生成学习包，保留已有修改并合并，不反复创建新工程快照。运行ROS节点时使用第六课的`--ros-args -p port:=...`参数，原生CLI使用第四/五课的`--port ...`参数，两种语法不同。

本页没有运动命令。后续有限运动仍需要本机参数、当前状态、完整计划、独占预算和现场验收；旧实物记录不作为新运动许可。
