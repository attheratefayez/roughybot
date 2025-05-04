import os 

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, AndSubstitution
from ament_index_python import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory("roughybot_description")

    # launch configurations
    add_camera_lc = LaunchConfiguration(
        "add_camera",
        default="true"
    )

    add_lidar_lc = LaunchConfiguration(
        "add_lidar",
        default="true"
    )

    use_sim_time_lc = LaunchConfiguration(
        "use_sim_time",
        default = "true"
    )

    launch_rviz_lc = LaunchConfiguration(
        "launch_rviz",
        default = "true"
    )

    launch_joint_state_publisher_gui_lc = LaunchConfiguration(
        "launch_joint_state_publisher_gui",
        default = "true"
    )


    # launch declarationS
    add_camera_decl = DeclareLaunchArgument(
        "add_camera",
        default_value="true",
        choices=["true", "false"]
    )

    add_lidar_decl = DeclareLaunchArgument(
        "add_lidar",
        default_value="true",
        choices=["true", "false"]
    )

    use_sim_time_decl = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"]
    )

    launch_rviz_decl = DeclareLaunchArgument(
        "launch_rviz",
        default_value = "true", 
        choices = ["true", "false"]
    )

    launch_joint_state_publisher_gui_decl = DeclareLaunchArgument(
        "launch_joint_state_publisher_gui",
        default_value = "true", 
        choices = ["true", "false"]
    )

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", os.path.join(pkg_dir, "urdf", "robot_core.urdf.xacro"), 
        " ", "add_camera:=", add_camera_lc,
        " ", "add_lidar:=", add_lidar_lc
    ])



    # nodes 
    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            "robot_description" : robot_description,
            "use_sim_time" : use_sim_time_lc
        }]
    )

    rviz_node = Node(
        condition = IfCondition(launch_rviz_lc),
        package = "rviz2",
        executable = "rviz2",
        output = "screen",
        arguments = ["--display-config", os.path.join(pkg_dir, "config", "view_robot.rviz")]
    )

    joint_state_publisher_gui_node = Node(
        condition = IfCondition(AndSubstitution(
            launch_rviz_lc, launch_joint_state_publisher_gui_lc
        )),
        package = "joint_state_publisher_gui", 
        executable = "joint_state_publisher_gui",
        output = "screen"
    )


    return LaunchDescription([
        add_camera_decl, 
        add_lidar_decl, 
        use_sim_time_decl, 
        launch_rviz_decl,
        launch_joint_state_publisher_gui_decl,

        robot_state_publisher_node,
        rviz_node, 
        joint_state_publisher_gui_node
    ])
