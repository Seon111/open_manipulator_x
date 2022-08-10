import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_open_manipulator_x_gazebo = get_package_share_directory(
        'open_manipulator_x_gazebo')
    pkg_open_manipulator_x_description = get_package_share_directory(
        'open_manipulator_x_description')

    # Sart World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_open_manipulator_x_gazebo, 'launch',
                         'gazebo_world.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_open_manipulator_x_description, 'launch',
                         'multi_spawn_robot.launch.py'),
        )
    )

    return LaunchDescription([
        start_world,
        spawn_robot_world
    ])
