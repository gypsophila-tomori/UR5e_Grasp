cmake_minimum_required(VERSION 3.8)
project(llm_task_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找所有必要的依赖包
# ########## 这是我们需要修改的地方 ##########
# 复制 ur5e_gripper_control 的所有 find_package 指令过来
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # 我们自己需要的
find_package(nlohmann_json REQUIRED) # 我们自己需要的

# 从 ur5e_gripper_control 复制过来的依赖
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 别忘了我们还依赖 ur5e_gripper_control 本身！
find_package(ur5e_gripper_control REQUIRED) 

# ########## 修改结束 ##########


add_executable(task_executor_node src/task_executor_node.cpp)

# 链接依赖库
ament_target_dependencies(task_executor_node
  rclcpp
  std_msgs
  nlohmann_json
  # 链接到我们改造后的库
  ur5e_gripper_control
  # 把 ur5e_gripper_control 的公共依赖也链接上
  moveit_ros_planning_interface
  tf2
  control_msgs
)


install(TARGETS
  task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 注意：我们不再需要 target_include_directories 了，因为
# 我们在 ur5e_gripper_control 中正确地导出了它的头文件目录，
# ament_target_dependencies 会自动处理。

ament_package()
