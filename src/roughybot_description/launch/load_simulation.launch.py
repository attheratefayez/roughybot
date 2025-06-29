import os 

from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    gz_sim_env_var_update = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(
            get_package_prefix("roughybot_description"),
            "share"
        )
    )

    robot_spawner_node = Node(
        package = "ros_gz_sim", 
        executable = "create", 
        output = "screen",
        arguments = "-topic /robot_description -z 0.01".split()
    )

    robot_spawner_delay = TimerAction(
        period = 3.0, 
        actions= [robot_spawner_node]
    )


    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    gz_sim_launch = IncludeLaunchDescription(
        [
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", 
                "gz_sim.launch.py"
            )
        ],
        launch_arguments = {
            "gz_args" : "-r empty.sdf",
            "on_exit_shutdown" : "true"
        }.items()
    )

    return LaunchDescription([
        gz_sim_env_var_update,
        bridge_node,
        gz_sim_launch,
        robot_spawner_delay, 
    ])
