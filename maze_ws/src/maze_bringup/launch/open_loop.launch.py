"""Open-loop demo: straight line / spin in place / square.

    ros2 launch maze_bringup open_loop.launch.py mode:=straight distance:=1.0
    ros2 launch maze_bringup open_loop.launch.py mode:=turn_left angle:=1.5708
    ros2 launch maze_bringup open_loop.launch.py mode:=square
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
    for action in maze_common.sim_arguments(maze_common.EMPTY_WORLD):
        ld.add_action(action)

    for name, default, desc in (
            ('mode', 'straight', 'straight|turn_left|turn_right|square'),
            ('distance', '1.0', 'distance for mode:=straight [m]'),
            ('angle', '1.5708', 'angle for turns [rad]'),
            ('square_side', '0.7', 'side of the square [m]'),
            ('linear_speed', '0.15', 'linear speed [m/s]'),
            ('angular_speed', '0.8', 'angular speed [rad/s]')):
        ld.add_action(DeclareLaunchArgument(name, default_value=default,
                                            description=desc))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    for action in maze_common.odom_trail():   # RViz 里的绿色轨迹
        ld.add_action(action)

    ld.add_action(Node(
        package='maze_control',
        executable='open_loop_control',
        name='open_loop_control',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': LaunchConfiguration('mode'),
            'distance': ParameterValue(LaunchConfiguration('distance'), value_type=float),
            'angle': ParameterValue(LaunchConfiguration('angle'), value_type=float),
            'square_side': ParameterValue(LaunchConfiguration('square_side'), value_type=float),
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'angular_speed': ParameterValue(LaunchConfiguration('angular_speed'), value_type=float),
        }],
    ))
    return ld
