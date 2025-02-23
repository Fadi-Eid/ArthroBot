import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("arthrobot_description"),
                    "urdf",
                    "arthrobot.urdf.xacro",
                )
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("arthrobot_controller"),
                "config",
                "arthrobot_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arthrobot_controller", "--controller-manager", "/controller_manager"],
    )

    arm_servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arthrobot_servo_controller", "--controller-manager", "/controller_manager", "--stopped"],
    )

    motors_interface_node = Node(
        package="arthrobot_controller",
        executable="motors_interface.py",
    )

    
    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            arm_servo_controller_spawner,
            motors_interface_node,
        ]
    )
