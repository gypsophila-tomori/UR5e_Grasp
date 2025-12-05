cmake_minimum_required(VERSION 3.8)
project(ur5e_gripper_control)

# use cpp 17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)


#################################################################
#                  ###   核心修改部分  ###
#################################################################

# 1. 将 ur5e_gripper.cpp 编译成一个名为 "ur5e_gripper_lib" 的库
#    这个库包含了 UR5eGripper 类的所有功能
add_library(ur5e_gripper_lib SHARED
  src/ur5e_gripper.cpp
)
#    为这个库链接它所需要的所有依赖
ament_target_dependencies(ur5e_gripper_lib
  rclcpp
  moveit_ros_planning_interface
  tf2
  moveit_core
  moveit_ros_planning
  control_msgs
)
#    指定这个库需要包含的头文件目录
target_include_directories(ur5e_gripper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# 2. 原来的 demo 可执行文件现在只包含 main 函数
add_executable(demo src/demo.cpp)
#    让 demo 链接我们刚刚创建的 ur5e_gripper_lib 库
target_link_libraries(demo ur5e_gripper_lib)
#    为 demo 链接它自己的依赖（通常很少，因为大部分依赖已被库链接）
ament_target_dependencies(demo rclcpp)


# 3. 安装库和头文件，这样其他包才能找到它们 (至关重要!)
install(
  TARGETS ur5e_gripper_lib
  EXPORT export_ur5e_gripper_control # 导出一个目标，供下游包使用
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

#    导出目标，让 ament 能自动处理依赖
ament_export_interfaces(export_ur5e_gripper_control
  HAS_LIBRARY_TARGET
)

#################################################################
#                   ###   修改结束   ###
#################################################################


# 原有的安装指令保持不变
install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
    config
  DESTINATION 
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
