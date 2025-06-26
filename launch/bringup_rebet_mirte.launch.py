from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import EnvironmentVariable
from launch.actions import GroupAction
from launch_ros.actions import SetParameter
from launch.actions import TimerAction
from launch.actions import ExecuteProcess


def generate_launch_description():

    launch_files = os.path.join(get_package_share_directory("rebet_school"), "launch")

    aal = Node(
        package='aal',
        executable='adaptation_layer',
        prefix='xterm -hold -e',
        name='adaptation_layer_node',
    )

    system_reflection = Node(
        package='rebet_school',
        executable='system_reflection.py',
        prefix='xterm -hold -e',
        name='system_reflection_node',)

    arborist = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(launch_files, "arborist_config_launch.py")
        )
    )

    groot = ExecuteProcess(
        cmd=[os.path.expanduser('~/groot.AppImage')],
        name='groot',
        output='screen',
    )

    return LaunchDescription(
        [arborist, aal, groot, system_reflection]
    )