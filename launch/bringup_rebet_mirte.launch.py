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


def generate_launch_description():
    # launch_yolo_arg = DeclareLaunchArgument(
    #     "yolo", default_value="true", description="Also launch the yolo node or not"
    # )

    launch_files = os.path.join(get_package_share_directory("rebet_mirte"), "launch")
    config_files = os.path.join(get_package_share_directory("rebet_mirte"), "config")

    config_file = os.path.join(
        config_files,
        'adaptation_engine_config.yaml'
    )

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

    typedb = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_typedb"), "launch",
                         "ros_typedb.launch.py")
        ),
        launch_arguments={
            "schema_path": f"[{os.path.join(config_files, 'schema_tactics_resolution.tql')}]",
            "data_path": f"[{os.path.join(config_files, 'insert_measurement.tql')}]",
            "force_database": "True",
            "force_data": "True",
        }.items(),
    )

    x = GroupAction(
                actions = [
                    Node(
                        package="rebet_java",
                        executable="adaptation_engine",
                        output="screen",),
                    SetParameter(name='ros2_path', value='not_empty'),             
                ]
    )

    adap_engine = Node(
        package="rebet_java",
        executable="adaptation_engine",
        output="screen",
        parameters=[config_file]
    )

    delay_adap_engine = TimerAction(
        period=2.0,  # Delay for 5 seconds
        actions=[adap_engine],
    )

    context_model = Node(
        package="rebet_mirte",
        executable="context_model.py",
        prefix=EnvironmentVariable('REBET_TERMINAL_PREFIX'),
        output="screen",
    )

    return LaunchDescription(
        [typedb, delay_adap_engine]#, aal, arborist, context_model]
    )
