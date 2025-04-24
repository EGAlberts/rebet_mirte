from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    # launch_yolo_arg = DeclareLaunchArgument(
    #     "yolo", default_value="true", description="Also launch the yolo node or not"
    # )

    launch_files = os.path.join(get_package_share_directory("rebet_mirte"), "launch")

    # adap_sys = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(launch_files, "adaptation_system_launch.py")
    #     ),
    #     launch_arguments={}.items(),
    # )

    # yolo = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(launch_files, "yolo_self_start_launch.py")
    #     ),
    #     launch_arguments={}.items(),
    #     condition=LaunchConfigurationEquals("yolo", "true"),
    # )

    aal = Node(
        package='aal',
        executable='adaptation_layer',
        prefix=EnvironmentVariable('REBET_TERMINAL_PREFIX'),
        name='adaptation_layer_node',
    )

    arborist = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(launch_files, "arborist_config_launch.py")
        )
    )

    adap_engine = Node(
        package="rebet_java",
        executable="adaptation_engine",
        output="screen",
    )

    context_model = Node(
        package="rebet_mirte",
        executable="context_model.py",
        prefix=EnvironmentVariable('REBET_TERMINAL_PREFIX'),
        output="screen",
    )

    return LaunchDescription(
        [adap_engine, aal, arborist, context_model]
    )
