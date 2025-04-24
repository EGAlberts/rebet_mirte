import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from nav2_common.launch import ReplaceString
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    EmitEvent,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# Replaces this:
# ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False
# autostart:=True map:=/home/$USER/rebet_ws/map_1713963103.yaml
# params_file:=/home/$USER/rebet_ws/frog_nav2_params.yaml


def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Open rviz2 to visualize ROS2 data",
    )

    use_map_arg = DeclareLaunchArgument(
        "use_map",
        default_value="true",
        description="Run using a premade map, no SLAM (true/false)",
    )

    nav_to_pose_tree_arg = DeclareLaunchArgument(
        "nav_to_pose_tree",
        default_value="frog_navigate_to_pose.xml",
        description="Which Nav2 navigate_to_pose BT to use",
    )

    map_file = os.path.join(
        get_package_share_directory("rebet_frog"), "config", "acsos2024_map.yaml"
    )

    nav2_params_file = os.path.join(
        get_package_share_directory("rebet_frog"), "config", "frog_nav2_params.yaml"
    )

    rviz_file = os.path.join(
        get_package_share_directory("rebet_frog"), "config", "frog_default_view.rviz"
    )

    trees_path = os.path.join(
        get_package_share_directory("rebet_frog"),
        "trees",
        "nav2",
    )

    params_file = ReplaceString(
        source_file=nav2_params_file,
        replacements={
            "<tree_path>": (
                trees_path,
                os.path.sep,
                LaunchConfiguration("nav_to_pose_tree"),
            )
        },
    )

    nav2_path = get_package_share_directory("nav2_bringup")
    nav2__bringup_launch_path = os.path.join(nav2_path, "launch", "bringup_launch.py")

    nav2_with_map = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(nav2__bringup_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_file,
            "params_file": params_file,
        }.items(),
        condition=LaunchConfigurationEquals("use_map", "true"),
    )

    slam_tb_path = get_package_share_directory("slam_toolbox")
    slam_tb_launch_path = os.path.join(slam_tb_path, "launch", "online_async_launch.py")

    nav2_nav_launch_path = os.path.join(nav2_path, "launch", "navigation_launch.py")

    nav2_with_slam = GroupAction(
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(nav2_nav_launch_path),
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(slam_tb_launch_path)),
        ],
        condition=LaunchConfigurationEquals("use_map", "false"),
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file, "--ros-args", "--log-level", "WARN"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_map_arg,
            nav_to_pose_tree_arg,
            nav2_with_map,
            nav2_with_slam,
            start_rviz_cmd,
            exit_event_handler,
        ]
    )
