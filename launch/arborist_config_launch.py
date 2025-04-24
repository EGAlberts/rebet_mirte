from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('rebet_mirte'),
        'config',
        'mirte_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rebet_mirte',
            executable='mirte_arborist',
            name='mirte_arborist_node',
            prefix=EnvironmentVariable('REBET_TERMINAL_PREFIX'),
            output='screen',
            parameters=[config_file, {}]
        ),
    ])
