from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot"),
        #IncludeLaunchDescription(
        #    PathJoinSubstitution([
        #        FindPackageShare("ff_viz"),
        #        "launch",
        #        "ff_viz.launch.py",
        #    ]),
        #    launch_arguments={"robot_name": robot_name}.items(),
        #),
        Node(
            package="ff_control",
            executable="pd_ctrl_node",
            name="pd_ctrl_node",
            namespace=robot_name,
        ),
        Node(
            package="ff_estimate",
            executable="mocap_estimator_node",
            name="mocap_estimator_node",
            namespace=robot_name,
            parameters=[{
                "pose_channel": PathJoinSubstitution([
                    "mocap",
                    robot_name,
                    "pose",
                ]),
            }],
        ),
    ])
