# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    # 1. 声明 shooter_node
    shooter_node = Node(
        package="challenge",
        executable="shooter_node"
    )
    # 2. 封装至 LaunchDescription
    launch_description = LaunchDescription(
        [shooter_node])
    # 3. 返回对象以便 ros2 launch 解析
    return launch_description
