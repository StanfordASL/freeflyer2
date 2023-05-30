from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot"),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ff_sim"),
                        "launch",
                        "single.launch.py",
                    ]
                ),
                launch_arguments={"robot_name": robot_name}.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ff_viz"),
                        "launch",
                        "ff_viz.launch.py",
                    ]
                ),
                launch_arguments={"robot_name": robot_name}.items(),
            ),
            Node(
                package="ff_control",
                executable="pd_ctrl_node",
                name="pd_ctrl_node",
                namespace=robot_name,
            ),
            # path planning ############################################################################
            Node(
                package="ff_path_planning",
                executable="simple_plan_planning",
                name="simple_goal_broadcaster",
                namespace=robot_name,
            ),
            #Node(
            #    package="ff_path_planning",
            #    executable="path_planning",
            #    name="path_planner_node",
            #    namespace=robot_name,
            #),
        ]
    )