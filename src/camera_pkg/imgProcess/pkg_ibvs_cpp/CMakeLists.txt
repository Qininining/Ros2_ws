cmake_minimum_required(VERSION 3.8) # 保持用户指定的版本

project(pkg_ibvs_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED) # 添加 message_filters 依赖
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio) # 添加OpenCV依赖

find_package(hand_tracking_msgs REQUIRED) # 添加这行

# 确保在 build 目录中生成编译信息，用于 ROS 2 调试
# ament_export_dependencies 可以用于导出依赖，以便其他包能够找到这些依赖。
# 对于内部编译，target_link_libraries 和 ament_target_dependencies 会处理链接。
# 考虑到您使用了 ament_target_dependencies，这里通常不再需要显式调用 ament_export_dependencies，除非有其他特定的包导出需求。
# ament_export_dependencies(rclcpp sensor_msgs cv_bridge OpenCV) # 如果不需要导出，可以注释掉

# 头文件目录
include_directories(
  include/pkg_ibvs_cpp/
  # 如果你的 IBVS.hpp 放在其他子目录，例如 include/pkg_ibvs_cpp/IBVS.hpp，则只需要 include/
  # 如果IBVS.hpp直接在 include/ 目录下，也写 include/
)

# 定义 IBVS 类的源文件
set(IBVS_SOURCES
  src/IBVS.cpp # 添加 IBVS 类的源文件
)

# 添加可执行文件
add_executable(ibvs_feature_node
  src/ibvs_feature_node.cpp # 你的现有 ROS 节点
  ${IBVS_SOURCES}           # 添加 IBVS 类的源文件到可执行文件的编译中
)

target_include_directories(ibvs_feature_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(ibvs_feature_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# 使用 ament_target_dependencies 链接所有依赖
ament_target_dependencies(
  ibvs_feature_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  OpenCV # <--- 修正：将 ${OpenCV_LIBS} 替换为 OpenCV
  hand_tracking_msgs
)

install(TARGETS ibvs_feature_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
