"""电脑 WSL 上加载第十二课模型；关节角由另一个终端的滑块提供。"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # 从安装的包读取 Xacro，展开属性和宏，把标准 URDF 文本交给状态发布器。
    share = get_package_share_path("jixiebi_learning")
    model = xacro.process_file(str(share / "urdf" / "lesson12_arm.urdf.xacro"))
    description = ParameterValue(model.toxml(), value_type=str)

    # 状态发布器读取 /joint_states，按模型的关节轴和安装位置计算 TF。
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description}],
        output="screen",
    )
    # RViz2 用同一份描述绘制连杆；只想查询坐标时可传 with_rviz:=false。
    viewer = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(share / "rviz" / "lesson12_arm.rviz")],
        condition=IfCondition(LaunchConfiguration("with_rviz")),
    )
    return LaunchDescription(
        [DeclareLaunchArgument("with_rviz", default_value="true"), state_publisher, viewer]
    )
