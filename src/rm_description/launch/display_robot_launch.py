from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 配置你的包名 + 模型文件名（和你文件夹完全匹配）
    pkg_name = "rm_description"
    urdf_file = "rm_65.urdf.xacro"

    # 2. 获取 URDF 文件路径
    urdf_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", urdf_file]
    )

    # 3. 自动解析 xacro 模型
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_path]
    )

    robot_description = {"robot_description": robot_description_content}

    # 4. 启动：关节状态发布器（可拖动机械臂）
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # 5. 启动：机器人状态发布器
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 6. 启动：RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])