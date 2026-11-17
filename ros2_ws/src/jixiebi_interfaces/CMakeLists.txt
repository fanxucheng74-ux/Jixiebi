cmake_minimum_required(VERSION 3.8)
project(jixiebi_interfaces)

# 本包只定义消息，不打开设备；生成器会为C++/Python等客户端生成对应类型。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 一个单轴样本和一批六轴样本；stamp使用ROS标准Time类型，不能省略其依赖。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoStateRaw.msg"
  "msg/ServoStatesRaw.msg"
  DEPENDENCIES builtin_interfaces
)

# 让使用消息的其他包获得所需运行时依赖，再由ament安装/登记本包。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
