"""Helpers shared by every launch file of the project.

The robot is the **stock TurtleBot3** shipped with ``turtlebot3_gazebo``: we
only contribute the maze world and spawn the ready-made model inside it.  That
keeps the physics (differential drive plug-in, LDS-01 laser, IMU), the meshes
and the TF tree exactly as ROBOTIS ships them, so the simulation numbers stay
comparable with the official TurtleBot3 examples.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESCRIPTION_PKG = 'maze_description'
DEFAULT_WORLD = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), 'worlds', 'maze.world')
# flat ground without walls: the demos that need no map load this one
EMPTY_WORLD = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), 'worlds', 'empty.world')
RVIZ_PATH = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), 'rviz', 'maze.rviz')

# --- stock TurtleBot3 model -------------------------------------------------
# burger is the smallest of the three (0.14 m footprint, 0.16 m wheel track),
# and the one that fits the 0.70 m corridors comfortably; waffle also works.
TB3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
TB3_GAZEBO = get_package_share_directory('turtlebot3_gazebo')
TB3_MODELS = os.path.join(TB3_GAZEBO, 'models')
TB3_SDF = os.path.join(TB3_MODELS, 'turtlebot3_' + TB3_MODEL, 'model.sdf')
TB3_URDF = os.path.join(TB3_GAZEBO, 'urdf', 'turtlebot3_' + TB3_MODEL + '.urdf')


def robot_description():
    """The stock URDF, used by robot_state_publisher for the TF tree."""
    with open(TB3_URDF, 'r', encoding='utf-8') as handle:
        return handle.read()


def turtlebot3_environment():
    """Gazebo needs GAZEBO_MODEL_PATH to resolve ``model://turtlebot3_*``."""
    current = os.environ.get('GAZEBO_MODEL_PATH', '')
    parts = [TB3_MODELS] + [p for p in current.split(os.pathsep) if p]
    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(parts)),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', TB3_MODEL),
    ]


def odom_trail():
    """Publish ``/actual_path`` from /odom so RViz draws the driven trail.

    Used by every launch that moves the robot: without it the green Path
    display in RViz stays empty even though the robot is driving.
    """
    return [Node(
        package='maze_control',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )]


def sim_arguments(world_default=None):
    """Launch arguments understood by every simulation launch file.

    ``world_default`` lets a launch file that needs no walls default to
    ``empty.world`` instead of the maze.
    """
    return [
        DeclareLaunchArgument('gui', default_value='true',
                              description='start the Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='start RViz2'),
        DeclareLaunchArgument('world',
                              default_value=world_default or DEFAULT_WORLD,
                              description='world file to load'),
        DeclareLaunchArgument('x', default_value='0.40',
                              description='robot spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.40',
                              description='robot spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='robot spawn yaw [rad]'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='verbose gzserver output'),
    ]


def gazebo_world():
    """gzserver + optional gzclient + robot_state_publisher + spawn + RViz."""
    gz_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_launch, 'gzserver.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_launch, 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description(), value_type=str),
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_' + TB3_MODEL,
            '-file', TB3_SDF,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', RVIZ_PATH],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return (turtlebot3_environment()
            + [gzserver, gzclient, robot_state_publisher, spawn_entity, rviz])
