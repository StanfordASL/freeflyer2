from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    language = LaunchConfiguration("language")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot"),
        DeclareLaunchArgument("language", default_value="cpp", choices=["cpp", "py"]),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("ff_sim"),
                "launch",
                "single.launch.py",
            ]),
            launch_arguments={"robot_name": robot_name}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("ff_viz"),
                "launch",
                "ff_viz.launch.py",
            ]),
            launch_arguments={"robot_name": robot_name}.items(),
        ),
        Node(
            package="ff_control",
            executable=["pd_ctrl_", language, "_node"],
            name="pd_ctrl_node",
            namespace=robot_name,
        ),
    ])
