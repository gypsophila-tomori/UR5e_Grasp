import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 获取我们自己的 boxes.yaml 配置文件的路径
    llm_task_executor_config = os.path.join(
        get_package_share_directory('llm_task_executor'),
        'config',
        'boxes.yaml'
    )

    # 获取 ur5e_gripper_control 包的 target_pose_list.yaml 文件的路径
    ur5e_gripper_control_config = os.path.join(
        get_package_share_directory('ur5e_gripper_control'),
        'config',
        'target_pose_list.yaml'
    )

    return LaunchDescription([
        Node(
            package='llm_task_executor',
            executable='task_executor_node',
            name='llm_task_executor_node',
            # 将两个配置文件都作为参数传入
            # ROS2 会自动合并它们
            parameters=[
                llm_task_executor_config,
                ur5e_gripper_control_config
            ],
            output='screen',
            emulate_tty=True 
        )
    ])
