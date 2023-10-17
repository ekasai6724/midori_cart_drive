import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    joy_config_filepath = launch.substitutions.LaunchConfiguration('joy_config_filepath')
    cart_config_filepath = launch.substitutions.LaunchConfiguration('cart_config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_config', default_value='f710'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('joy_config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(text=os.path.join(get_package_share_directory('teleop_twist_joy'), 'config', '')),
                joy_config,
                launch.substitutions.TextSubstitution(text='.config.yaml')
            ]
        ),
        launch.actions.DeclareLaunchArgument('cart_config_filepath',
            default_value=[
                launch.substitutions.ThisLaunchFileDir(),
                "/cart_config.yaml"
            ]
        ),

        launch_ros.actions.Node(
            package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 0.0,
                'coalesce_interval': 0.001,
            }]
        ),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[joy_config_filepath]
        ),
        launch_ros.actions.Node(
            package="midori_cart_drive", executable="midori_cart_drive", name="midori_cart_drive_node",
            parameters=[cart_config_filepath]
        ),
        launch_ros.actions.Node(
            package="midori_cart_drive", executable="svon_client", name="svon_client_node"
        ),
    ])
