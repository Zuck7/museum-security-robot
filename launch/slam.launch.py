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
    slam_params = os.path.join(share, 'config', 'slam_params.yaml')
    rviz_cfg = os.path.join(share, 'config', 'museum.rviz')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'render_engine': LaunchConfiguration('render_engine')}.items(),
    )

    # async_slam_toolbox_node processes scans in a background thread, so a slow
    # loop-closure optimisation never blocks the incoming scan queue.  On a VM
    # that is the difference between a clean map and a smeared one.
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        sim, slam, rviz,
    ])
