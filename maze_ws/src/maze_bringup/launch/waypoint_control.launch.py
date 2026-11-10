"""Waypoint sequence demo: follow the maze solution point by point.

    # plan the waypoint list from the world file Gazebo is loading
    ros2 launch maze_bringup waypoint_control.launch.py

    # or give the points yourself
    ros2 launch maze_bringup waypoint_control.launch.py \
        waypoints:="0.4,0.4;1.2,0.4;1.2,1.2"

With ``waypoints`` empty (the default) the list is planned from the walls in
the ``.world`` file, so the same command works on any maze seed instead of
only on the one the list was written for.
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

DEFAULT_WAYPOINTS = ''   # empty -> plan from the world file


def generate_launch_description():
    ld = LaunchDescription()
    for action in maze_common.sim_arguments(maze_common.EMPTY_WORLD):
        ld.add_action(action)

    ld.add_action(DeclareLaunchArgument(
        'plan_world', default_value=maze_common.DEFAULT_WORLD,
        description='world file the waypoint list is planned on '
                    '(Gazebo loads world, which defaults to empty.world)'))
    for name, default, desc in (
            ('waypoints', DEFAULT_WAYPOINTS,
             '"x,y;x,y;..." list of goals; empty = plan from the world file'),
            ('goal_x', '3.60', 'exit x used for the auto waypoint list'),
            ('goal_y', '4.15', 'exit y used for the auto waypoint list'),
            ('max_linear', '0.20', 'maximum linear speed [m/s]'),
            ('pause', '0.4', 'pause at every waypoint [s]')):
        ld.add_action(DeclareLaunchArgument(name, default_value=default,
                                            description=desc))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    for action in maze_common.odom_trail():   # RViz 里的绿色轨迹
        ld.add_action(action)

    ld.add_action(Node(
        package='maze_control',
        executable='waypoint_control',
        name='waypoint_control',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'waypoints': LaunchConfiguration('waypoints'),
            'world': LaunchConfiguration('plan_world'),
            'start_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'start_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'max_linear': ParameterValue(LaunchConfiguration('max_linear'), value_type=float),
            'pause': ParameterValue(LaunchConfiguration('pause'), value_type=float),
        }],
    ))
    return ld
