cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(pkg_imgproecss_l515_cpp) # 根据你提供的项目名称进行更新

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) # 查找 OpenCV 库

# ******image_subscriber_node******
add_executable(image_subscriber_node src/image_subscriber_node.cpp)
# 设置头文件搜索路径
target_include_directories(image_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 要求 C99 和 C++17 标准
target_compile_features(image_subscriber_node PUBLIC c_std_99 cxx_std_17)
# 链接 ROS 2 依赖包
ament_target_dependencies(
  image_subscriber_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
)
# 链接 OpenCV 库（解决 undefined reference 错误）
target_link_libraries(image_subscriber_node
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_imgcodecs
)
install(TARGETS image_subscriber_node
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
