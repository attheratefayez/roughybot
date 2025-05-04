import os
from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable

def generate_launch_description():

    """
    Launches:

        - robot_state_publisher
        - rviz2

    Accepted Parameters: 
        - use_sim_time : whether to use simulation time or not. 
        - robot_description : xacro processed urdf file. 
        - start_rviz : (true | false)
    """

    pkg_dir = get_package_share_directory("roughybot_sim")

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", 
        os.path.join(pkg_dir, "robot_description", "urdf", "robot_core.urdf.xacro")
    ])

    # <--- LAUNCH CONFIGURATIONS --->
    robot_description_lc = LaunchConfiguration("robot_description")
    use_sim_time_lc = LaunchConfiguration("use_sim_time")
    start_rviz_lc = LaunchConfiguration("start_rviz")

    # <--- LAUNCH DECLARATIONS --->
    robot_description_decl = DeclareLaunchArgument(
        "robot_description",
        default_value = robot_description,
        description = "Robot URDF file path."
    )

    use_sim_time_decl = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "True",
        description = "Whether or not to use simulation time."
    )

    start_rviz_decl = DeclareLaunchArgument(
        "start_rviz", 
        default_value = "True",
        description = "Start Rviz node or not."
    )

    # <--- NODES --->
    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            "robot_description" : robot_description_lc,
            "use_sim_time" : use_sim_time_lc
        }]
    )

    rviz_node = Node(
        condition = IfCondition(start_rviz_lc),
        package = "rviz2", 
        executable = "rviz2", 
        output = "screen", 
        arguments = ["-d", os.path.join(pkg_dir, "config", "rviz_default.rviz")], 
        parameters = [{
            "use_sim_time" : use_sim_time_lc,
        }]
    )

    return LaunchDescription([
        robot_description_decl,
        use_sim_time_decl,
        start_rviz_decl, 
        
        robot_state_publisher_node,
        rviz_node
    ])
