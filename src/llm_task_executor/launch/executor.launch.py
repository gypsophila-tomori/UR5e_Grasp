import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 获取配置文件的路径
    config = os.path.join(
        get_package_share_directory('llm_task_executor'),
        'config',
        'boxes.yaml'
    )

    return LaunchDescription([
        Node(
            package='llm_task_executor',
            executable='task_executor_node',
            name='llm_task_executor_node',
            parameters=[config],
            output='screen',
            # 使用emulate_tty来确保Python的print和C++的RCLCPP_INFO都能实时刷新
            emulate_tty=True 
        )
    ])
