#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arthrobot_waypoint_follower");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("arthrobot_waypoint_follower", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    static const std::string PLANNING_GROUP = "arthrobot_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    // const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Planning to the initial pose
    std::vector<double> joint_angles(5);
    joint_angles[0] = 0.0;
    joint_angles[1] = -0.767945;
    joint_angles[2] = -0.994838;
    joint_angles[3] = 0.0;
    joint_angles[4] = 0.279253;
    move_group.setJointValueTarget(joint_angles);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(plan);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Cartesian Paths
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    for (int i = 0; i < 50; ++i)
    {
        geometry_msgs::msg::Pose target_pose = start_pose;

        target_pose.position.z -= 0.05;
        waypoints.push_back(target_pose); 

        target_pose.position.y -= 0.05;
        waypoints.push_back(target_pose); 

        target_pose.position.z += 0.05;
        waypoints.push_back(target_pose);

        target_pose.position.y += 0.05;
        waypoints.push_back(target_pose);
    }

    // We want the Cartesian path to be interpolated at a resolution of 1 cm,
    // which is why we will specify 0.01 as the max step in Cartesian translation.
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Cartesian path (%.2f%% achieved)", fraction * 100.0);
    RCLCPP_INFO(LOGGER, "Executing Cartesian trajectory now...");
    move_group.execute(trajectory);

    rclcpp::shutdown();
    return 0;
}