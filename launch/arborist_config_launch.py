from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('rebet_school'),
        'config',
        'mirte_config.yaml'
    )

    testing_arg = DeclareLaunchArgument(
        "exercise",
        default_value="monitoring",
        description="Run a test sleep tree",
    )

    mytree_arg = DeclareLaunchArgument(
        "my_tree",
        default_value="",
        description="Your tree",
    )


    return LaunchDescription([
        testing_arg,
        mytree_arg,
        Node(
            package='rebet_school',
            executable='mirte_arborist',
            name='mirte_arborist_node',
            # prefix='xterm -hold -e ',
            output='screen',
            parameters=[config_file, {
                'log_blackboard': True,
                'factory_xml': False,
            }]
        ),
        Node(
            package="rebet_school",
            executable="tree_action_client.py",
            arguments=["Sleep60"],
            parameters=[
                {
                    'autostart' : True
                }
            ],
            condition= LaunchConfigurationEquals('exercise', 'monitoring')
        ),
        Node(
            package="rebet_school",
            executable="tree_action_client.py",
            arguments=["QRSleep60"],
            parameters=[
                {
                    'autostart' : True
                }
            ],
            condition= LaunchConfigurationEquals('exercise', 'analysis')
        ),
        Node(
            package="rebet_school",
            executable="tree_action_client.py",
            arguments=[LaunchConfiguration("my_tree")],
            parameters=[
                {
                    'autostart' : True
                }
            ],
            condition= LaunchConfigurationEquals('exercise', 'analysis')
        ),
    ])
