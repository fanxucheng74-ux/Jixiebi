cmake_minimum_required(VERSION 3.8)
project(jixiebi_description)

find_package(ament_cmake REQUIRED)

# 仅安装标称几何与可视化资料，不安装设备驱动或真实控制配置。
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
