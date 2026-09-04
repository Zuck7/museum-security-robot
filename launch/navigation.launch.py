import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'museum_security_robot'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    nav2_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(share, 'maps', 'museum_map.yaml')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    rviz_cfg = os.path.join(share, 'config', 'museum.rviz')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Occupancy grid .yaml. Swap in your slam_toolbox map '
                        'for the graded demo.'),
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('rviz', default_value='true'),
        nav2, rviz,
    ])
