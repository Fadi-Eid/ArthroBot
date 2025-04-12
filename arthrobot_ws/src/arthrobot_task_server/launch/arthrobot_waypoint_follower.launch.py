import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arthrobot", package_name="arthrobot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("arthrobot_description"),
            "urdf",
            "arthrobot.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/arthrobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines("ompl", ["ompl"])
        .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    arthrobot_waypoint_follower = Node(
        name="arthrobot_waypoint_follower",
        package="arthrobot_task_server",
        executable="arthrobot_waypoint_follower",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([arthrobot_waypoint_follower])