"""WSL 共用彩色模型；第十六课从 Topic 播放，第十七课从 Action 播放。"""

from pathlib import Path
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def start_model(context):
    # 展开一次第十二课 Xacro。状态发布器用它解释关节轴，RViz 用它绘制同一根连杆。
    share = get_package_share_path("jixiebi_learning")
    model = xacro.process_file(str(share / "urdf" / "lesson12_arm.urdf.xacro")).toxml()
    temporary = TemporaryDirectory(prefix="jixiebi_lesson16_")
    folder = Path(temporary.name)
    urdf = folder / "arm.urdf"
    urdf.write_text(model, encoding="utf-8")

    # 本机 Humble 有过 TF 正常、窗口只显示网格的情况。沿用第十四课文件加载法和原视角。
    config = yaml.safe_load((share / "rviz" / "lesson12_arm.rviz").read_text(encoding="utf-8"))
    for display in config["Visualization Manager"]["Displays"]:
        if display["Class"] == "rviz_default_plugins/RobotModel":
            display.update({"Description Source": "File", "Description File": str(urdf)})
    mode = LaunchConfiguration("input").perform(context)
    if mode in ("cartesian_line", "speed_profile"):
        # 使用同一份模型，加入工作空间参考点和反馈轨迹。朝 +Y 看可分辨 X 方向的横移。
        config["Visualization Manager"]["Displays"].append({
            "Class": "rviz_default_plugins/MarkerArray",
            "Name": "Lesson22 Path" if mode == "speed_profile" else "Lesson21 Path",
            "Enabled": True,
            "Value": True,
            "Topic": {"Value": "/jixiebi/learning/line_trace", "Reliability Policy": "Reliable",
                      "Durability Policy": "Transient Local", "Depth": 1},
        })
        config["Visualization Manager"]["Views"]["Current"].update({
            "Yaw": 1.3, "Pitch": 0.85, "Distance": 0.68,
            "Focal Point": {"X": 0.0, "Y": 0.10, "Z": 0.32},
        })
    view = folder / "arm.rviz"
    view.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

    # RViz 和状态发布器共用本课的 TF，避免同域另一份模型的同名连杆影响画面。
    # 每一课只启动一个播放器，各用自己的状态与 TF 话题；模型、视角完全相同。
    action_mode = mode in ("action", "checked_action", "cartesian_line", "speed_profile")
    prefix = "/jixiebi/learning/follow" if action_mode else "/jixiebi/learning/playback"
    frames = [(name, prefix + name) for name in ("/tf", "/tf_static")]
    return [
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: temporary.cleanup())])
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="lesson17_model" if action_mode else "lesson16_model",
            parameters=[{"robot_description": ParameterValue(model, value_type=str)}],
            remappings=frames
            + [(name, prefix + name) for name in ("/joint_states", "/robot_description")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(view)],
            remappings=frames,
        ),
        Node(
            package="jixiebi_learning",
            executable="lesson17_follow_server" if action_mode else "lesson16_trajectory_player",
            # 第十八课在接单时核对起点；第十七课保留原来的演示方式。
            parameters=[{"check_start": mode != "action", "sampled_linear": mode == "speed_profile"}]
            if action_mode else [],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input", default_value="topic", choices=["topic", "action", "checked_action", "cartesian_line", "speed_profile"]
            ),
            OpaqueFunction(function=start_model),
        ]
    )
