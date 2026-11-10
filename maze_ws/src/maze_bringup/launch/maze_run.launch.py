"""One-shot demo: simulation + laser processing + left-wall maze solving.

    ros2 launch maze_bringup maze_run.launch.py
    ros2 launch maze_bringup maze_run.launch.py gui:=false rviz:=false

The robot is spawned in the entrance cell facing east; the wall follower
drives until it leaves through the north exit gap (y > 4.10 m).
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        'mission', default_value='true',
        description='start scan_processor + wall_follower + path recorder'))
    ld.add_action(DeclareLaunchArgument(
        'speed', default_value='0.22', description='cruise speed [m/s]'))
    ld.add_action(DeclareLaunchArgument(
        'speed_clear_distance', default_value='1.00',
        description='front clearance for full cruise speed [m]'))
    ld.add_action(DeclareLaunchArgument(
        'speed_clear_min', default_value='0.70',
        description='speed factor when the wall ahead is at front_stop'))
    ld.add_action(DeclareLaunchArgument(
        'target_left', default_value='0.34',
        description='distance kept to the left wall [m]'))
    ld.add_action(DeclareLaunchArgument(
        'corner_enabled', default_value='1.0',
        description='1 = cut corners on an arc, 0 = pivot turns only'))
    ld.add_action(DeclareLaunchArgument(
        'corner_speed', default_value='0.25',
        description='speed while tracking the doorway corner path [m/s]'))
    ld.add_action(DeclareLaunchArgument(
        'corner_diagonal', default_value='1.0',
        description='1 = cut junctions on the chord, 0 = rounded L path'))
    ld.add_action(DeclareLaunchArgument(
        'corner_chain_enabled', default_value='1.0',
        description='1 = merge two opposite turns (Z bend) into one S, 0 = keep them apart'))

    for action in maze_common.gazebo_world():
        ld.add_action(action)

    scan = Node(
        package='maze_control',
        executable='scan_processor',
        name='scan_processor',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('mission')),
    )
    path = Node(
        package='maze_control',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('mission')),
    )
    follower = Node(
        package='maze_control',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'speed_clear_distance': ParameterValue(
                LaunchConfiguration('speed_clear_distance'), value_type=float),
            'speed_clear_min': ParameterValue(
                LaunchConfiguration('speed_clear_min'), value_type=float),
            'target_left': ParameterValue(LaunchConfiguration('target_left'), value_type=float),
            'corner_enabled': ParameterValue(
                LaunchConfiguration('corner_enabled'), value_type=float),
            'corner_speed': ParameterValue(
                LaunchConfiguration('corner_speed'), value_type=float),
            'corner_diagonal': ParameterValue(
                LaunchConfiguration('corner_diagonal'), value_type=float),
            'corner_chain_enabled': ParameterValue(
                LaunchConfiguration('corner_chain_enabled'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    ld.add_action(scan)
    ld.add_action(path)
    ld.add_action(follower)
    return ld
