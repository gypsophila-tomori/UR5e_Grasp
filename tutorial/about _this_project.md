# 项目拆解

# 1.simulation.launch.py
内部函数：``launch`` ``generate_launch_description``  
**launch**：用于真正要启动的内容核心逻辑函数  
**generate_launch_description**：ros2 launch的入口函数  

## (1)generate_launch_description()
```python
def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
```
``declared_argument = []``：生命启动命令行的传入参数
``return LaunchDescription()``：launch文件的核心返回部分，返回LaunchDescription对象，包含所有要执行的动作
``QpaqueFunction(function=launch_setup)``：特殊的动作，将当前上下文传递给function_setup函数然后执行，launch_setup函数的返回值最终添加到启动列表中

## (2)launch_setup()
```python
def launch_setup(context, *args, **kwargs):
    # ......内容省略
    nodes_to_launch = [ ... ]
    return nodes_to_launch
```
### 机器人仿真与控制
```python
dual_ur5e_gripper_control_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare("ur5e_gripper_moveit_config"), "/launch", "/ur5e_gripper_sim_control.launch.py"]
    ),
    launch_arguments={
            "launch_rviz": "true",
    }.items(),
    )
```
用于启动机器人的基本仿真环境。
``FindPackageShare("ur5e_gripper_moveit_config")``：找到功能包，调用``ur5e_gripper_sim_control.launch.py``文件  
**作用**：  
加载模型文件，启动仿真，将模型加载道仿真环境中
，启动仿真世界中的硬件控制，并启动rviz

### 运动规划

```python
    dual_ur5e_gripper_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur5e_gripper_moveit_config"), "/launch", "/ur5e_gripper_moveit.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )
```
**作用**：  
启动moveit2运动规划框架，调用ur5e_gripper_moveit.launch.py，家在机器人关节组信息，启动move_group核心节点，加载其他插件
P.S.注意``launch_arguments={"use_sim_time":"true"}``使用仿真时间而非系统真实时间(/clock话题)，保证ROS系统时间与仿真世界同步

### 深度图像处理
```python
    register_depth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("vision"), "/launch", "/register_depth.launch.py"]
        ),
    )
```
**作用**
处理深度相机的数据，调用的launch文件为：``register_depth.launch.py``，用于定位

### 视觉检测与分割
```python
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("vision"), "/launch", "/seg_and_det.launch.py"]
        ),
    )
```
**作用**  
找到需要抓取的物体，同时对其深度图，将2D检测结果转换为2D空间坐标姿态，发布为ROS2话题

# 2.vision功能包
## (1)seg_and_det.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 节点1: 目标检测
        Node(
            package='vision',
            executable='obj_detect',
            name='obj_detect',
            output='screen'
        ),
        # 节点2: 坐标转换
        Node(
            package='vision',
            executable='det_tf',
            name='det_tf',
            output='screen'
        ),
        # 节点3: 点云处理
        Node(
            package='vision',
            executable='point_cloud_processor',
            name='point_cloud_processor',
            output='screen'
        )
    ])
```
### obj_detect节点
运行 YOLOv8 模型的核心  
任务：  
**输入**: 订阅来自仿真相机发布的彩色图像话题（例如 /camera/color/image_raw）  

**处理**: 将接收到的图像传入 YOLOv8 模型进行推理。  

**输出**: 发布检测结果。检测结果通常包含每个物体的类别、置信度以及2D边界框（Bounding Box） 的像素坐标。这个结果会被发布到一个自定义的 ROS 话题上

### det_tf节点
负责从2D检测结果计算出3D位姿的关键一环
任务：  
