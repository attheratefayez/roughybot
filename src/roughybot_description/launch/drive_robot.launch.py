# from launch import LaunchDescription
# from launch_ros.actions import Node
#
# def generate_launch_description():
#
#     teleop_twist_keyboard_node = Node(
#         package="teleop_twist_keyboard",
#         executable="teleop_twist_keyboard",
#         output="screen",
#     )
#
#     return LaunchDescription(
#         [
#             teleop_twist_keyboard_node
#         ]
#     )

#
# USE THIS COMMAND TO DRIVE THE ROBOT
#
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --param stamped:="true" --remap /cmd_vel:=/diff_drive_base_controller/cmd_vel

