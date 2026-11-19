"""电脑 WSL 复用第十二课模型，可选厂家标称显示或本机两点标定。"""

from pathlib import Path
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def is_disabled_draft(path):
    # 只识别本课模板里全部禁用的草稿。其他参数布局、有部分端点的文件仍由 C++ 校验。
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or len(document) != 1:
        return False
    group = document.get("joint_state_bridge", document.get("/joint_state_bridge"))
    if not isinstance(group, dict):
        return False
    axes = group.get("ros__parameters")
    return isinstance(axes, dict) and all(
        axis in {"base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"}
        and fields == {"servo_id": -1}
        for axis, fields in axes.items()
    )


def start_nodes(context):
    share = get_package_share_path("jixiebi_joint_state_bridge")
    path = LaunchConfiguration("calibration_file").perform(context)
    mapping = LaunchConfiguration("mapping").perform(context)
    # 保留旧命令：显式传入文件就使用实测；日常只选视图时直接复用厂家比例。
    nominal = mapping == "nominal" or (mapping == "auto" and not path)
    yaw_zero = LaunchConfiguration("yaw_zero_p").perform(context)
    # 标称显示单独使用话题与 TF；普通 /joint_states 继续留给实测标定模式。
    display_remaps = []
    if nominal:
        if path:
            raise ValueError("标称模式只读取厂家模型和底座工作零位，请移除 calibration_file")
        parameters = [{"use_nominal_model": True, "yaw_zero_p": int(yaw_zero or "1500")}]
        display_remaps = [
            (name, "/nominal_estimate" + name)
            for name in ("/tf", "/tf_static", "/robot_description")
        ]
    else:
        if yaw_zero:
            raise ValueError("实测模式从 calibration_file 读取零位，不能另设 yaw_zero_p")
        display_help = (
            "本机看底座请运行：ros2 launch jixiebi_joint_state_bridge lesson14_feedback.launch.py "
            "mapping:=nominal yaw_zero_p:=1511 view:=axis"
        )
        if not path:
            raise ValueError(
                f"实测模式需要 calibration_file 指向已填写的两点标定文件。{display_help}"
            )
        calibration = Path(path).expanduser()
        if not calibration.is_file():
            raise FileNotFoundError(f"找不到标定参数文件：{calibration}")
        if is_disabled_draft(calibration):
            raise ValueError(f"{calibration.name} 尚未启用任何实测关节。{display_help}")
        parameters = [calibration]
    bridge = Node(
        package="jixiebi_joint_state_bridge",
        executable="joint_state_bridge",
        parameters=parameters,
        output="screen",
        on_exit=Shutdown(reason="关节反馈桥已退出"),
    )

    # 从同一份 Xacro 展开四个关节的名称、转轴和标称几何，避免两课模型各自演变。
    lesson = get_package_share_path("jixiebi_learning")
    model = xacro.process_file(str(lesson / "urdf" / "lesson12_arm.urdf.xacro"))
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(model.toxml(), value_type=str)}],
        remappings=display_remaps
        + ([("/joint_states", "/nominal_estimate/joint_states")] if nominal else []),
        output="screen",
    )

    # 底座子坐标系 yaw_link 在 URDF 中没有几何体。单轴视图用 TF 轴显示它的转向。
    cleanup = []
    if LaunchConfiguration("view").perform(context) == "axis":
        config = share / "rviz" / "lesson14_axis.rviz"
    else:
        config = lesson / "rviz" / "lesson12_arm.rviz"
        if nominal and IfCondition(LaunchConfiguration("with_rviz")).evaluate(context):
            # 模型与 RSP 用同一份展开结果。本机从文件加载外形，独立 TF 话题仍决定每个连杆的姿态。
            temporary = TemporaryDirectory(prefix="jixiebi_rviz_")
            folder = Path(temporary.name)
            urdf = folder / "arm.urdf"
            urdf.write_text(model.toxml(), encoding="utf-8")
            settings = yaml.safe_load(config.read_text(encoding="utf-8"))
            for display in settings["Visualization Manager"]["Displays"]:
                if display["Class"] == "rviz_default_plugins/RobotModel":
                    display.update({"Description Source": "File", "Description File": str(urdf)})
            config = folder / "arm.rviz"
            config.write_text(yaml.safe_dump(settings, sort_keys=False), encoding="utf-8")
            cleanup.append(
                RegisterEventHandler(
                    OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: temporary.cleanup())])
                )
            )
    viewer = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(config)],
        remappings=display_remaps,
        condition=IfCondition(LaunchConfiguration("with_rviz")),
    )
    return [*cleanup, bridge, state_publisher, viewer]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping",
                default_value="auto",
                choices=["auto", "measured", "nominal"],
                description="默认有标定文件时用实测，无文件时用标称角。",
            ),
            DeclareLaunchArgument(
                "yaw_zero_p", default_value="", description="标称显示的底座参考 P，默认厂家 1500"
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description="选做实测标定时传入的 ROS 参数 YAML；至少启用一个已标定的关节。",
            ),
            DeclareLaunchArgument(
                "view",
                default_value="model",
                choices=["model", "axis"],
                description="model 显示机械臂外形；axis 显示 TF 坐标轴，可观察底座朝向。",
            ),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            OpaqueFunction(function=start_nodes),
        ]
    )
