"""Pure pursuit trajectory tracking demo.

    # plan the route from the world file and follow it (default);
    # no seed-specific arguments, the walls are read from the world
    ros2 launch maze_bringup pure_pursuit.launch.py

    # follow a custom polyline
    ros2 launch maze_bringup pure_pursuit.launch.py \
        path:="0.4,0.4;2.0,0.4;2.0,1.2;0.4,1.2;0.4,0.4"

The reference path is rounded at the corners (``corner_radius``) because a
pure pursuit controller cuts corners and would otherwise clip the walls.
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

    ld.add_action(DeclareLaunchArgument(
        'path', default_value='',
        description='reference polyline "x,y;x,y;..."; empty = plan it from '
                    'the world file (works on any maze seed)'))
    ld.add_action(DeclareLaunchArgument(
        'goal_x', default_value='3.60', description='exit x used for the auto route'))
    ld.add_action(DeclareLaunchArgument(
        'goal_y', default_value='4.15', description='exit y used for the auto route'))
    ld.add_action(DeclareLaunchArgument(
        'lookahead', default_value='0.32',
        description='pure pursuit lookahead distance [m]'))
    ld.add_action(DeclareLaunchArgument(
        'max_linear', default_value='0.20', description='max speed [m/s]'))
    ld.add_action(DeclareLaunchArgument(
        'max_angular', default_value='1.4', description='max yaw rate [rad/s]'))
    ld.add_action(DeclareLaunchArgument(
        'corner_radius', default_value='0.18',
        description='path corner rounding radius [m]'))
    ld.add_action(DeclareLaunchArgument(
        'timeout', default_value='300.0', description='mission timeout [s]'))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    ld.add_action(Node(
        package='maze_control',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'path': LaunchConfiguration('path'),
            'world': LaunchConfiguration('world'),
            'start_x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'start_y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'),
                                     value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'),
                                     value_type=float),
            'lookahead': ParameterValue(LaunchConfiguration('lookahead'),
                                        value_type=float),
            'max_linear': ParameterValue(LaunchConfiguration('max_linear'),
                                         value_type=float),
            'max_angular': ParameterValue(LaunchConfiguration('max_angular'),
                                          value_type=float),
            'corner_radius': ParameterValue(LaunchConfiguration('corner_radius'),
                                            value_type=float),
            'timeout': ParameterValue(LaunchConfiguration('timeout'),
                                      value_type=float),
        }],
    ))
    ld.add_action(Node(
        package='maze_control',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ))
    return ld
