"""Two-pass maze run: explore first, then race the planned route.

    # pass 1 -- drive with the reactive wall follower and record a map
    ros2 launch maze_bringup race.launch.py mode:=explore

    # pass 2 -- plan on that map and drive the fastest route
    ros2 launch maze_bringup race.launch.py mode:=race

The map is written to ``~/maze_map.npz`` by default.  Both passes spawn the
robot at the same entrance pose, so the odom frames of the two runs match.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import maze_common  # noqa: E402


def _mode_is(value):
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration('mode'), "' == '", value, "'"]))


def generate_launch_description():
    ld = LaunchDescription()
    for action in maze_common.sim_arguments():
        ld.add_action(action)

    ld.add_action(DeclareLaunchArgument(
        'mode', default_value='explore',
        description='explore (build the map) or race (drive the plan)'))
    ld.add_action(DeclareLaunchArgument(
        'map_file', default_value=os.path.expanduser('~/maze_map.npz'),
        description='map file shared by the two passes'))
    ld.add_action(DeclareLaunchArgument(
        'goal_x', default_value='3.60', description='exit goal x [m]'))
    ld.add_action(DeclareLaunchArgument(
        'goal_y', default_value='4.15', description='exit goal y [m]'))
    ld.add_action(DeclareLaunchArgument(
        'speed', default_value='0.22',
        description='explore pass cruise speed [m/s]'))
    ld.add_action(DeclareLaunchArgument(
        'speed_scale', default_value='1.0',
        description='scale applied to the planned speed profile'))
    ld.add_action(DeclareLaunchArgument(
        'max_linear', default_value='0.40',
        description='speed cap while racing the plan [m/s]'))
    ld.add_action(DeclareLaunchArgument(
        'lookahead', default_value='0.30',
        description='pure pursuit lookahead while racing [m]'))
    ld.add_action(DeclareLaunchArgument(
        'controller', default_value='pid',
        description='tracker used in the race pass: pid or pure_pursuit'))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    # ---- pass 1: reactive exploration + mapping ------------------------
    follower = Node(
        package='maze_control',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
        }],
        condition=_mode_is('explore'),
    )
    builder = Node(
        package='maze_control',
        executable='map_builder',
        name='map_builder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_file': LaunchConfiguration('map_file'),
        }],
        condition=_mode_is('explore'),
    )

    # ---- pass 2: plan on the map and race -----------------------------
    racer = Node(
        package='maze_control',
        executable='path_racer',
        name='path_racer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_file': LaunchConfiguration('map_file'),
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'speed_scale': ParameterValue(
                LaunchConfiguration('speed_scale'), value_type=float),
            'max_linear': ParameterValue(
                LaunchConfiguration('max_linear'), value_type=float),
            'lookahead': ParameterValue(
                LaunchConfiguration('lookahead'), value_type=float),
            'controller': LaunchConfiguration('controller'),
        }],
        condition=_mode_is('race'),
    )

    recorder = Node(
        package='maze_control',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    for node in (follower, builder, racer, recorder):
        ld.add_action(node)
    return ld
