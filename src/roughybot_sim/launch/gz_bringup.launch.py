import os
from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable

def generate_launch_description():

    """
    Launches:
        - gz sim

    Spawns robot from robot_description topic.
    """

    pkg_dir = get_package_share_directory("roughybot_sim")

    # <--- LAUNCH CONFIGURATIONS --->
    gz_args_lc = LaunchConfiguration("gz_args")

    # <--- LAUNCH DECLARATIONS --->
    gz_args_decl = DeclareLaunchArgument(
        "gz_args", 
        default_value = "-r " + os.path.join(pkg_dir, "world", "default_world_room.sdf"), 
        description = "Gz sim args."
    )

    # <--- LAUNCH FILES --->
    gz_sim_launch = IncludeLaunchDescription(
        [
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", 
                "gz_sim.launch.py"
            ), 
        ],
        launch_arguments = {
            "gz_args" : gz_args_lc, 
            "on_exit_shutdown" : "true"
        }.items()
    )
    # <--- NODES --->

    robot_spawner_node = Node(
        package = "ros_gz_sim", 
        executable = "create", 
        arguments = ["-topic", "robot_description", "-z", "0.01"]
    )

    delayed_robot_spawn = TimerAction(
        period = 3.0, 
        actions = [robot_spawner_node]
    )

    return LaunchDescription(
        [
            gz_args_decl, 
            delayed_robot_spawn, 

            gz_sim_launch
        ]
    )
