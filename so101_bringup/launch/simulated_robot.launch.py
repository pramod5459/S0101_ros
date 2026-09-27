import os

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    SetEnvironmentVariable,
)

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==========================================================
    # SO101 DESCRIPTION PATH
    # ==========================================================

    description_share = get_package_share_directory(
        "so101_description"
    )

    # Example:
    # /home/pramod/so101_ws/install/so101_description/share/so101_description

    description_share_parent = os.path.dirname(
        description_share
    )

    # Example:
    # /home/pramod/so101_ws/install/so101_description/share

    models_path = os.path.join(
        description_share,
        "models"
    )


    # ==========================================================
    # WORLD FILE
    # ==========================================================

    world_file = PathJoinSubstitution([
        FindPackageShare("so101_description"),
        "worlds",
        "pick_place_world.sdf"
    ])


    # ==========================================================
    # GAZEBO RESOURCE PATH
    # ==========================================================

    gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",

        value=[

            # Required for:
            # model://so101_description/meshes/...
            description_share_parent,

            ":",

            # Required for:
            # model://work_platform
            # model://red_block
            models_path,

            ":",

            EnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                default_value=""
            )
        ]
    )


    # ==========================================================
    # RED BLOCK FILE
    # ==========================================================

    red_block_file = PathJoinSubstitution([
        FindPackageShare("so101_description"),
        "models",
        "red_block",
        "model.sdf"
    ])


    # ==========================================================
    # ROBOT DESCRIPTION
    # ==========================================================

    xacro_file = PathJoinSubstitution([
        FindPackageShare("so101_description"),
        "urdf",
        "so101.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file
        ]),
        value_type=str
    )


    # ==========================================================
    # ROBOT STATE PUBLISHER
    # ==========================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",

        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )


    # ==========================================================
    # GAZEBO
    # ==========================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),

        launch_arguments={
            "gz_args": [
                "-r ",
                world_file
            ]
        }.items()
    )


    # ==========================================================
    # SPAWN SO-101
    # ==========================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",

        arguments=[
            "-name", "so101",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0"
        ],

        output="screen"
    )


    # ==========================================================
    # SPAWN RED BLOCK
    # ==========================================================

    spawn_red_block = Node(
        package="ros_gz_sim",
        executable="create",

        arguments=[
            "-world", "default",
            "-file", red_block_file,
            "-name", "red_block",
            "-x", "0.25",
            "-y", "-0.10",
            "-z", "0.07"
        ],

        output="screen"
    )


    # ==========================================================
    # CONTROLLER LAUNCH
    # ==========================================================

    controller_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            PathJoinSubstitution([
                FindPackageShare("so101_controller"),
                "launch",
                "controller.launch.py"
            ])
        )
    )


    # ==========================================================
    # AFTER ROBOT SPAWNS -> START CONTROLLERS
    # ==========================================================

    start_controllers = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_robot,

            on_exit=[

                TimerAction(
                    period=2.0,

                    actions=[
                        controller_launch
                    ]
                )

            ]
        )
    )


    # ==========================================================
    # LAUNCH EVERYTHING
    # ==========================================================

    return LaunchDescription([

        # Must be set BEFORE Gazebo starts
        gazebo_resource_path,

        gazebo,

        robot_state_publisher,

        # Spawn SO-101 after Gazebo starts
        TimerAction(
            period=3.0,
            actions=[
                spawn_robot
            ]
        ),

        # Spawn red block
        TimerAction(
            period=4.0,
            actions=[
                spawn_red_block
            ]
        ),

        start_controllers,

    ])