from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_path = get_package_share_directory('so101_description')

    # Main Xacro file
    xacro_file = os.path.join(
        package_path,
        'urdf',
        'so101.urdf.xacro'
    )

    # Saved RViz configuration
    rviz_config = os.path.join(
        package_path,
        'rviz',
        'config.rviz'
    )

    # Xacro -> URDF
    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        output='screen'
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
