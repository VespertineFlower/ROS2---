"""Reactive obstacle avoidance demo (no wall following).

    ros2 launch maze_bringup reactive_avoid.launch.py
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import maze_common  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()
    for action in maze_common.sim_arguments():
        ld.add_action(action)

    for name, default, desc in (
            ('front_stop', '0.45', 'distance at which the robot turns [m]'),
            ('speed', '0.16', 'cruise speed [m/s]')):
        ld.add_action(DeclareLaunchArgument(name, default_value=default,
                                            description=desc))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    ld.add_action(Node(
        package='maze_control',
        executable='reactive_avoid',
        name='reactive_avoid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'front_stop': ParameterValue(LaunchConfiguration('front_stop'), value_type=float),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
        }],
    ))
    ld.add_action(Node(
        package='maze_control',
        executable='scan_processor',
        name='scan_processor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ))
    return ld
