from setuptools import find_packages, setup

package_name = 'maze_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maze team',
    maintainer_email='student@example.com',
    description='Control nodes for the maze differential-drive robot '
                '(open loop, P point control, waypoints, scan sectors, '
                'reactive avoidance, left-wall following, pure pursuit).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_control = maze_control.open_loop_control:main',
            'point_control = maze_control.point_control:main',
            'waypoint_control = maze_control.waypoint_control:main',
            'scan_processor = maze_control.scan_processor:main',
            'reactive_avoid = maze_control.reactive_avoid:main',
            'wall_follower = maze_control.wall_follower:main',
            'odom_path_publisher = maze_control.odom_path_publisher:main',
            'pure_pursuit = maze_control.pure_pursuit:main',
            'map_builder = maze_control.map_builder:main',
            'path_racer = maze_control.path_racer:main',
        ],
    },
)
