import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    llm_task_executor_config = os.path.join(
        get_package_share_directory('llm_task_executor'),
        'config',
        'boxes.yaml'
    )

    ur5e_gripper_control_config = os.path.join(
        get_package_share_directory('ur5e_gripper_control'),
        'config',
        'target_pose_list.yaml'
    )

    # ##########  这是我们唯一的修改  ##########
    # 创建一个包含 use_sim_time 参数的字典
    # 这会告诉节点去订阅 /clock 话题来获取时间
    sim_time_parameter = {'use_sim_time': True}
    # ########################################


    return LaunchDescription([
        Node(
            package='llm_task_executor',
            executable='task_executor_node',
            name='llm_task_executor_node',
            parameters=[
                llm_task_executor_config,
                ur5e_gripper_control_config,
                sim_time_parameter  # <-- 将这个参数字典添加到列表中
            ],
            output='screen',
            emulate_tty=True 
        )
    ])
