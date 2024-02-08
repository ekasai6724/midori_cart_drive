import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_config_filepath = LaunchConfiguration('joy_config_filepath')
    cart_config_filepath = LaunchConfiguration('cart_config_filepath')
    lidar_pkg_dir = LaunchConfiguration('lidar_pkg_dir',
                                        default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='f710'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_config_filepath',
            default_value=[
                #TextSubstitution(text=os.path.join(get_package_share_directory('teleop_twist_joy'), 'config', '')),
                ThisLaunchFileDir(),
                "/",
                joy_config,
                TextSubstitution(text='.config.yaml')
            ]
        ),
        DeclareLaunchArgument('cart_config_filepath',
            default_value=[
                ThisLaunchFileDir(),
                "/cart_config.yaml"
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/ld08.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),
        Node(
            package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 0.0,
                'coalesce_interval': 0.001,
            }]
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[joy_config_filepath]
        ),
        Node(
            package="midori_cart_drive", executable="midori_cart_drive", name="midori_cart_drive_node",
            parameters=[cart_config_filepath]
        ),
        Node(
            package="midori_cart_drive", executable="svon_client", name="svon_client_node"
        ),
    ])
