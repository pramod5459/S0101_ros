from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # -----------------------------
    # Package paths
    # -----------------------------
    description_pkg = get_package_share_directory(
        'so101_description'
    )

    ros_gz_sim_pkg = get_package_share_directory(
        'ros_gz_sim'
    )

    # -----------------------------
    # SO-101 Xacro
    # -----------------------------
    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'so101.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # -----------------------------
    # Start Gazebo
    # -----------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # -----------------------------
    # Spawn SO-101 into Gazebo
    # -----------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'so101'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot
    ])