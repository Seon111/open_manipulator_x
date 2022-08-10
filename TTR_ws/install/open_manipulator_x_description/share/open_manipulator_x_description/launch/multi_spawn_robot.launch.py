#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def gen_robot_list(number_of_robots):

    robots = []
    y_pos = [0.975, -0.975]

    for i in range(number_of_robots):
        robot_name = "open_manipulator_x"+str(i)

        robots.append({'name': robot_name, 'x_pose': 0.0,
                      'y_pose': y_pos[i], 'z_pose': 0.665})

    return robots


def generate_launch_description():

    urdf = os.path.join(get_package_share_directory(
        'open_manipulator_x_description'), 'urdf/', 'open_manipulator_x_robot.urdf')
    pkg_open_manipulator_x_description = get_package_share_directory(
        'open_manipulator_x_description')
    assert os.path.exists(
        urdf), "The open_manipulator_x.urdf doesnt exist in "+str(urdf)

    urdf2 = os.path.join(get_package_share_directory(
        'open_manipulator_x_description'), 'urdf/', 'open_manipulator_x_robot2.urdf')
    pkg_open_manipulator_x_description = get_package_share_directory(
        'open_manipulator_x_description')
    assert os.path.exists(
        urdf2), "The open_manipulator_x.urdf doesnt exist in "+str(urdf2)

    urdfs = [urdf, urdf2]

    # Names and poses of the robots
    robots = gen_robot_list(2)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for i, robot in enumerate(robots):
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_open_manipulator_x_description, 'launch',
                                                           'spawn_robot.launch.py')),
                launch_arguments={
                    'robot_urdf': urdfs[i],
                    'x': TextSubstitution(text=str(robot['x_pose'])),
                    'y': TextSubstitution(text=str(robot['y_pose'])),
                    'z': TextSubstitution(text=str(robot['z_pose'])),
                    'robot_name': robot['name'],
                    'robot_namespace': robot['name']
                }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld
