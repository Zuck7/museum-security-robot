#!/usr/bin/env python3
"""The LLM command console on its own.

Kept separate from the Nav2 launch on purpose: the console reads from stdin,
and stdin does not behave well when a launch file is also streaming the logs
of fifteen other nodes into the same terminal.  Run Nav2 in one terminal and
this in another.

  export GEMINI_API_KEY='...'
  ros2 launch museum_security_robot llm_navigator.launch.py
  ros2 launch museum_security_robot llm_navigator.launch.py mode:=--menu
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'museum_security_robot'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='',
            description="'' for AI mode, '--menu' for the manual menu, "
                        "'--patrol' for a scripted full circuit"),
        Node(
            package=PKG, executable='museum_ai_navigator',
            name='museum_ai_navigator', output='screen',
            emulate_tty=True,          # keeps input() usable under ros2 launch
            arguments=[LaunchConfiguration('mode')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
