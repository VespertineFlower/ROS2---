"""Load the maze world, the robot URDF and (optionally) RViz2.

    ros2 launch maze_bringup maze_world.launch.py
    ros2 launch maze_bringup maze_world.launch.py gui:=false rviz:=false
"""

import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import maze_common  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()
    for action in maze_common.sim_arguments():
        ld.add_action(action)
    for action in maze_common.gazebo_world():
        ld.add_action(action)
    return ld
