cmake_minimum_required(VERSION 3.8)
project(pkg_ur5_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenCV REQUIRED)  # 添加OpenCV依赖

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}      # 添加OpenCV头文件路径
)

add_executable(ur5_joint_controller src/ur5_joint_controller.cpp)
target_include_directories(ur5_joint_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ur5_joint_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ur5_joint_controller
  "rclcpp"
  "std_msgs"
  "trajectory_msgs"
)

install(TARGETS ur5_joint_controller
  DESTINATION lib/${PROJECT_NAME})

add_executable(joint_state_subscriber src/joint_state_subscriber.cpp)
ament_target_dependencies(
  joint_state_subscriber
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

install(TARGETS joint_state_subscriber
  DESTINATION lib/${PROJECT_NAME})

add_executable(velocity_command_publisher src/velocity_command_publisher.cpp)
ament_target_dependencies(velocity_command_publisher rclcpp std_msgs)

add_executable(ur5_kinematics_controller src/ur5_kinematics_controller.cpp src/KinematicsSolver.cpp)
ament_target_dependencies(ur5_kinematics_controller rclcpp std_msgs trajectory_msgs)
target_link_libraries(ur5_kinematics_controller ${OpenCV_LIBS})  # 链接OpenCV库

add_executable(ur5_fk_subscriber_node src/ur5_fk_subscriber.cpp src/KinematicsSolver.cpp)
ament_target_dependencies(ur5_fk_subscriber_node rclcpp sensor_msgs) # 假设KinematicsSolver在pkg_ur5_control_lib中
target_link_libraries(ur5_fk_subscriber_node ${OpenCV_LIBS})  # 链接OpenCV库


add_executable(ur5_velocity_controller src/ur5_velocity_controller.cpp src/KinematicsSolver.cpp)
ament_target_dependencies(ur5_velocity_controller
  rclcpp
  std_msgs
  sensor_msgs
  # OpenCV is linked via target_link_libraries
)
target_link_libraries(ur5_velocity_controller ${OpenCV_LIBS})



install(TARGETS
  velocity_command_publisher
  ur5_kinematics_controller
  ur5_fk_subscriber_node # 添加新的节点
  ur5_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
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
