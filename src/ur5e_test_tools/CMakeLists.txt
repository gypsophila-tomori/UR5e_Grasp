cmake_minimum_required(VERSION 3.8)
project(ur5e_test_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

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

# 末端位姿监测程序
add_executable(end_effector_pose src/end_effector_pose.cpp)
ament_target_dependencies(end_effector_pose
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# 手动控制程序
add_executable(manual_control src/manual_control.cpp)
ament_target_dependencies(manual_control
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

# 安装可执行文件
install(TARGETS
  end_effector_pose
  manual_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()