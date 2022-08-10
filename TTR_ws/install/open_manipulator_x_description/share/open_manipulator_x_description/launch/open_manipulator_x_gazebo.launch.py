import os
import this

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi


def generate_launch_description():

    this_pkg_path = os.path.join(
        get_package_share_directory('open_manipulator_x_description'))

    gazebo_model_path = os.path.join(this_pkg_path, 'GazeboFiles', 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"),
          "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    world_path = os.path.join(this_pkg_path, 'worlds', 'table_tennis.world')

    xacro_file_path = os.path.join(
        this_pkg_path, 'urdf', 'open_manipulator_x_robot.urdf.xacro')

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    rviz_config = os.path.join(
        this_pkg_path, 'rviz', 'open_manipulator_x.rviz')

    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'open_manipulator_x',
                   '-x', '0.0',
                   '-y', '0.975',
                   '-z', '0.665'],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'start', 'joint_trajectory_controller'],
        output='screen'
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher,
        spawn_entity,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        # spawn_controller,
        # rviz2,
        load_joint_state_controller,
        load_joint_trajectory_controller
    ])
