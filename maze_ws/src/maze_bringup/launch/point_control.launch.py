"""Closed-loop P controller demo: drive to one (x, y) point.

    # pick a goal automatically: the far end of the longest free straight run
    # from the spawn, read out of the world file (works on any maze seed)
    ros2 launch maze_bringup point_control.launch.py

    # or drive to an explicit point
    ros2 launch maze_bringup point_control.launch.py goal_x:=0.4 goal_y:=1.2

The controller is a pure P law with heading priority and no obstacle
avoidance, so the goal must be reachable in a straight line; a blocked goal is
reported as a warning when the node starts.
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

    ld.add_action(DeclareLaunchArgument(
        'plan_world', default_value=maze_common.DEFAULT_WORLD,
        description='world file the auto goal is planned on '
                    '(Gazebo loads world, which defaults to empty.world)'))
    for name, default, desc in (
            ('goal_x', 'nan',
             'goal x [m]; leave unset to pick the longest free run from the world'),
            ('goal_y', 'nan', 'goal y [m]; see goal_x'),
            ('max_linear', '0.20', 'maximum linear speed [m/s]'),
            ('tolerance', '0.05', 'goal tolerance [m]')):
        ld.add_action(DeclareLaunchArgument(name, default_value=default,
                                            description=desc))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    for action in maze_common.odom_trail():   # RViz 里的绿色轨迹
        ld.add_action(action)

    ld.add_action(Node(
        package='maze_control',
        executable='point_control',
        name='point_control',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'world': LaunchConfiguration('plan_world'),
            'start_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'start_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'max_linear': ParameterValue(LaunchConfiguration('max_linear'), value_type=float),
            'tolerance': ParameterValue(LaunchConfiguration('tolerance'), value_type=float),
        }],
    ))
    return ld
