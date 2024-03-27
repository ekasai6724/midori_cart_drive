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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_filepath = LaunchConfiguration('params_filepath')
    map_dir = LaunchConfiguration('map',
        default=os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'))    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_filepath', default_value=[ThisLaunchFileDir(), "/navigation_midori_cart.yaml"]),
        DeclareLaunchArgument('map', default_value=map_dir),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bringup_midori_cart.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={'map': map_dir, 'use_sim_time': use_sim_time, 'params_file': params_filepath}.items(),
        ),
        Node(
            package="midori_cart_drive", executable="navigation_client", name="navigation_client_node"
        ),
    ])
