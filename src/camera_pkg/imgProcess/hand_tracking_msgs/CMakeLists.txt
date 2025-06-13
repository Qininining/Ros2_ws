cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(hand_tracking_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages and dependencies
find_package(ament_cmake REQUIRED) # Consolidated, removed duplicate
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成 ROS 2 接口
# ${PROJECT_NAME} 是当前项目的名称 (hand_tracking_msgs)
# "msg/HandLandmarksDepth.msg" 指定了要生成的消息文件
# DEPENDENCIES 参数非常重要，它告诉 rosidl_generate_interfaces
# 你的自定义消息依赖于哪些其他 ROS 消息类型。
# 这会确保生成的 C/C++ 代码能够正确地 #include 这些依赖的头文件。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandLandmarksDepth.msg"
  DEPENDENCIES std_msgs geometry_msgs # <--- 关键修正：明确列出依赖
)

# 导出依赖关系，这样其他依赖此包的 ROS 包就能找到这些运行时依赖
ament_export_dependencies(
  rosidl_default_runtime # 导出 ROS 接口的运行时依赖
  std_msgs             # 导出 std_msgs，以防其他包直接依赖 hand_tracking_msgs
  geometry_msgs        # 导出 geometry_msgs
)

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
