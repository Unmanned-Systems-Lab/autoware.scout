cmake_minimum_required(VERSION 3.8)
project(udp_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 添加YAML-CPP支持
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 生成ROS2自定义消息（原有配置，保留）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajPoint.msg"
  "msg/TrajCmd.msg"
  "msg/RemoteCmd.msg"
  "msg/AutoCmd.msg"
  "msg/PlatformStatus.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

# 定义源文件列表
set(UDP_BRIDGE_SOURCES
  src/udp_bridge_node.cpp
  src/config_manager.cpp
)

# 2. 原有组件式库配置（保留，可选）
add_library(udp_bridge_node_lib SHARED ${UDP_BRIDGE_SOURCES})
target_include_directories(udp_bridge_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(udp_bridge_node_lib 
  rclcpp 
  rclcpp_components 
  std_msgs 
  yaml-cpp
)
# 确保链接yaml-cpp库
target_link_libraries(udp_bridge_node_lib yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(udp_bridge_node_lib "${cpp_typesupport_target}")
rclcpp_components_register_nodes(udp_bridge_node_lib "udp_ros_bridge::UdpRosBridgeNode")

# 3. 新增：生成独立可执行文件（核心！解决"No executable found"）
add_executable(udp_bridge_node src/main.cpp)  # 分离主函数到单独文件
ament_target_dependencies(udp_bridge_node rclcpp std_msgs yaml-cpp)  # 链接依赖
target_link_libraries(udp_bridge_node udp_bridge_node_lib yaml-cpp)  # 链接库

# 4. 安装可执行文件（必须！否则ros2 run找不到）
install(TARGETS
  udp_bridge_node_lib  # 组件式库
  udp_bridge_node      # 独立可执行文件
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可执行文件安装到lib/包名/下（ROS2标准路径）
)

# Install Python scripts
install(PROGRAMS
  test_platform_status_publisher.py
  advanced_platform_status_publisher.py
  configurable_platform_status_publisher.py  # 新增可配置版本
  DESTINATION lib/${PROJECT_NAME}
)

# Install config file
install(FILES
  config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python modules
# Note: We need to install Python modules separately to ensure they are available
file(GLOB PYTHON_FILES "*.py")
install(PROGRAMS
  ${PYTHON_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(udp_bridge_node_lib udp_bridge_node)
ament_export_targets(export_${PROJECT_NAME})
ament_package()