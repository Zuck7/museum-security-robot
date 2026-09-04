#!/usr/bin/env python3
"""One-shot demo: simulation + Nav2 + RViz.

This is what to run during the live demonstration (rubric item 9).  Start the
LLM console separately in a second terminal:

  Terminal 1:  ros2 launch museum_security_robot museum_demo.launch.py
  Terminal 2:  ros2 launch museum_security_robot llm_navigator.launch.py

Nav2 is delayed so the simulator has time to publish /clock and the first
/scan.  Without the delay AMCL activates against an empty TF buffer and spends
its first few seconds logging extrapolation errors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = 'museum_security_robot'


def generate_launch_description():
    share = get_package_share_directory(PKG)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'render_engine': LaunchConfiguration('render_engine')}.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'navigation.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'rviz': LaunchConfiguration('rviz')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(share, 'maps', 'museum_map.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        DeclareLaunchArgument(
            'nav_delay', default_value='8.0',
            description='seconds to wait for Gazebo before starting Nav2'),
        sim,
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[nav]),
    ])
