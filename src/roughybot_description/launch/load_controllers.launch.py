import os 

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory("roughybot_description")

    controller_list = ["joint_state_broadcaster", "diff_drive_base_controller"]

    controller_spawner_node = Node(
        package = "controller_manager",
        executable = "spawner",
        output = "screen",
        arguments = controller_list + ["--param-file", os.path.join(pkg_dir, "config", "controller_config.yaml")],
        remappings=[("joint_state_broadcaster/joint_states", "/joint_states")]
    )

    delayed_controller_spawner = TimerAction(
        period = 2.0, 
        actions = [
            controller_spawner_node
        ]
    )



    return LaunchDescription([
        delayed_controller_spawner
    ])




