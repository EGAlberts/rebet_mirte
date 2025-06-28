from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    launch_files = os.path.join(get_package_share_directory("rebet_school"), "launch")

    testing_arg = DeclareLaunchArgument(
        "exercise",
        default_value="monitoring",
        description="Run a test sleep tree",
    )

    aal = Node(
        package='aal',
        executable='adaptation_layer',
        prefix='xterm -hold -e',
        name='adaptation_layer_node',
        condition=LaunchConfigurationEquals('exercise', 'execution'),
    )

    system_reflection = Node(
        package='rebet_school',
        executable='system_reflection.py',
        prefix='xterm -hold -e',
        name='system_reflection_node',)

    arborist = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(launch_files, "arborist_config_launch.py")
        ),
        launch_arguments={
            'exercise': LaunchConfiguration('exercise'),
            }.items()
    )

    groot = ExecuteProcess(
        cmd=[os.path.expanduser('~/groot.AppImage')],
        name='groot',
        output='screen',
        condition=LaunchConfigurationEquals('exercise', 'execution'),
    )

    return LaunchDescription(
        [testing_arg, arborist, aal, groot, system_reflection]
    )