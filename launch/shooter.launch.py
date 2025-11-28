# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    # 1. 声明需要启动的 vision_node
    vision_node = Node(
        package="challenge",
        executable="vision_node"
    )
    # 2. 将节点写入 LaunchDescription
    launch_description = LaunchDescription(
        [vision_node])
    # 3. 返回 LaunchDescription 交由 ROS2 执行
    return launch_description
