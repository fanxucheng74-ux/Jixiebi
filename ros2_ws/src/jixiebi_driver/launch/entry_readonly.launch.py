"""启动入口规划与可选只读状态节点；不启动任何实物运动接口。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 默认不打开串口。需要本进程读取真实状态时显式 start_reader:=true 并给出端口。
    arguments = [
        DeclareLaunchArgument("model_path", description="本次名义几何JSON绝对路径"),
        DeclareLaunchArgument("request_path", description="取放任务JSON绝对路径"),
        DeclareLaunchArgument("policy_path", description="入口判定策略JSON绝对路径"),
        DeclareLaunchArgument("start_reader", default_value="false"),
        DeclareLaunchArgument("port", default_value=""),
    ]
    paths = {
        key: ParameterValue(LaunchConfiguration(key), value_type=str)
        for key in ("model_path", "request_path", "policy_path")
    }
    return LaunchDescription(
        arguments
        + [
            Node(
                package="jixiebi_driver",
                executable="transfer_entry_preview",
                parameters=[paths],
                output="screen",
            ),
            Node(
                package="jixiebi_driver",
                executable="raw_state",
                condition=IfCondition(LaunchConfiguration("start_reader")),
                parameters=[
                    {
                        "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
                        "period_s": 1.0,
                    }
                ],
                output="screen",
            ),
        ]
    )
