import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    arthrobot_controller_path = os.path.join(get_package_share_directory("arthrobot_controller"), "launch", "controller.launch.py")
    arthrobot_moveit_path = os.path.join(get_package_share_directory("arthrobot_moveit"), "launch", "moveit.launch.py")

    control = Node(package="arthrobot_client", executable="arthrobot_control_joystick")
    controller = IncludeLaunchDescription(arthrobot_controller_path)
    moveit = IncludeLaunchDescription(arthrobot_moveit_path)
    

    return LaunchDescription([
        controller,
        moveit,
        control
    ])