#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <arthrobot_interfaces/action/arthrobot_task.hpp>
#include <memory>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std::placeholders;
using namespace std::chrono_literals;

struct Quaternion
{
    double w, x, y, z;
};

// This is not in game format, it is in mathematical format.
Quaternion ToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

namespace arthrobot_task_server_node
{
  class ArthrobotTaskServerNode : public rclcpp::Node
  {
  private:
    rclcpp_action::Server<arthrobot_interfaces::action::ArthrobotTask>::SharedPtr arthrobot_task_server;

  public:
    explicit ArthrobotTaskServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("arthrobot_task_server_node", options)
    {
      RCLCPP_INFO(get_logger(), "Starting Arthrobot Task Server");
      arthrobot_task_server = rclcpp_action::create_server<arthrobot_interfaces::action::ArthrobotTask>(
          this, "arthrobot_task_server", std::bind(&ArthrobotTaskServerNode::GoalCallback, this, _1, _2),
          std::bind(&ArthrobotTaskServerNode::cancelCallback, this, _1),
          std::bind(&ArthrobotTaskServerNode::acceptedCallback, this, _1));
    }

    /* The callback functions */
  private:
    rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arthrobot_interfaces::action::ArthrobotTask::Goal> goal)
    {
      (void)uuid; // to suppress unused variable warning
      if (goal->task_number == 1)
      {
        RCLCPP_INFO(get_logger(), "Received <Gripper Open> task");
      }
      else if (goal->task_number == 2)
      {
        RCLCPP_INFO(get_logger(), "Received t<Gripper Close> task");
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arthrobot_interfaces::action::ArthrobotTask>> goal_handle)
    {
      (void)goal_handle; // to suppress unused variable warning
      int task_number = goal_handle->get_goal()->task_number;

      if (task_number == 1)
      {
        RCLCPP_INFO(get_logger(), "Received request to cancel <Gripper Open> task");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_ee");
        gripper_move_group.stop();
      }
      else if (task_number == 2)
      {
        RCLCPP_INFO(get_logger(), "Received request to cancel <Gripper Close> task");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_ee");
        gripper_move_group.stop();
      }

      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arthrobot_interfaces::action::ArthrobotTask>> goal_handle)
    {
      std::thread{std::bind(&ArthrobotTaskServerNode::taskExecute, this, _1), goal_handle}.detach();
    }

    void taskExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arthrobot_interfaces::action::ArthrobotTask>> goal_handle)
    {
      int task_number = goal_handle->get_goal()->task_number;

      if (task_number == 0)
      {
        RCLCPP_INFO(get_logger(), "Executing <Servoing Ready Pose> task");
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();

        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        move_group_interface.setMaxVelocityScalingFactor(1);
        move_group_interface.setMaxAccelerationScalingFactor(1);
        const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arthrobot_arm");
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
        std::vector<double> joint_angles;
        current_state->copyJointGroupPositions(joint_model_group, joint_angles);

        // specify joint angles destination in radians
        joint_angles[0] = 0.0;
        joint_angles[1] = 0.0;
        joint_angles[2] = 0.0;
        joint_angles[3] = 0.0;
        joint_angles[4] = 0.0;

        move_group_interface.setJointValueTarget(joint_angles);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_interface.move(); // execute the planned trajectory
          RCLCPP_INFO(get_logger(), "<Servoing Ready Pose> task executed successfully");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "<Servoing Ready Pose> task execution failed (no plan found)");
          return;
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "task succeeded");
      }
      else if (task_number == 1 || task_number == 2)
      {

        std::string gripper_task_name;
        if (task_number == 1)
          gripper_task_name = "Gripper Open";
        else
          gripper_task_name = "Gripper Close";

        RCLCPP_INFO(get_logger(), "Executing <%s> task", gripper_task_name.c_str());

        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_ee");
        const moveit::core::JointModelGroup *joint_model_group;
        joint_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("arthrobot_ee");
        moveit::core::RobotStatePtr current_state = gripper_move_group.getCurrentState(10);
        std::vector<double> joint_angles;
        current_state->copyJointGroupPositions(joint_model_group, joint_angles);

        if (task_number == 1)
          joint_angles[0] = 1.57;
        else
          joint_angles[0] = 0;

        gripper_move_group.setJointValueTarget(joint_angles);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          gripper_move_group.move();
          RCLCPP_INFO(get_logger(), "<%s> task executed successfully", gripper_task_name.c_str());
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "<%s> task execution failed", gripper_task_name.c_str());
          return;
        }

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "<%s> task finished", gripper_task_name.c_str());
      }
      else if (task_number == 3) // servoing ready position (out of singularity)
      {
        RCLCPP_INFO(get_logger(), "Executing <Servoing Ready Pose> task");
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();

        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        move_group_interface.setMaxVelocityScalingFactor(1);
        move_group_interface.setMaxAccelerationScalingFactor(1);
        const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arthrobot_arm");
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
        std::vector<double> joint_angles;
        current_state->copyJointGroupPositions(joint_model_group, joint_angles);
        const Eigen::Isometry3d& ee_link = current_state->getGlobalLinkTransform("gripper_support");
        RCLCPP_INFO_STREAM(get_logger(), "Translation: " << ee_link.translation());
        // Extract rotation matrix
        Eigen::Matrix3d rotation_matrix = ee_link.rotation();

        // Convert to Roll, Pitch, Yaw (RPY)
        Eigen::Vector3d rpy = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order (yaw, pitch, roll)

        RCLCPP_INFO_STREAM(get_logger(), "Roll: " << rpy[2] << ", Pitch: " << rpy[1] << ", Yaw: " << rpy[0]);

        // specify joint angles destination in radians
        joint_angles[0] = 0.0;
        joint_angles[1] = -0.628;
        joint_angles[2] = -1.0472;
        joint_angles[3] = 0.0;
        joint_angles[4] = 0.17453;

        move_group_interface.setJointValueTarget(joint_angles);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_interface.move(); // execute the planned trajectory
          RCLCPP_INFO(get_logger(), "<Servoing Ready Pose> task executed successfully");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "<Servoing Ready Pose> task execution failed (no plan found)");
          return;
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "task succeeded");


      }
      else if(task_number == 4) {
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();

        geometry_msgs::msg::Pose ik_pose;
        ik_pose.position.x = 0.00172086;
        ik_pose.position.y = 0.191754;
        ik_pose.position.z = 0.0619862;
        Quaternion q = ToQuaternion(-1.60608, 0.00162773, 0.00436216);
        ik_pose.orientation.x = q.x;
        ik_pose.orientation.y = q.y;
        ik_pose.orientation.z = q.z;
        ik_pose.orientation.w = q.w;

        // MoveIt 2 Interface
        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        move_group_interface.setMaxVelocityScalingFactor(0.6);
        move_group_interface.setMaxAccelerationScalingFactor(0.6);
        move_group_interface.setPoseTarget(ik_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            move_group_interface.move();
        }
        else
        {
            return;
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
      }
      else if(task_number == 5) {
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();

        geometry_msgs::msg::Pose ik_pose;
        ik_pose.position.x = -0.0154504;
        ik_pose.position.y = -0.0914863;
        ik_pose.position.z = 0.380033;
        Quaternion q = ToQuaternion(1.56191, 0.0124376, 0.0779226);
        ik_pose.orientation.x = q.x;
        ik_pose.orientation.y = q.y;
        ik_pose.orientation.z = q.z;
        ik_pose.orientation.w = q.w;

        // MoveIt 2 Interface
        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        move_group_interface.setMaxVelocityScalingFactor(0.6);
        move_group_interface.setMaxAccelerationScalingFactor(0.6);
        move_group_interface.setPoseTarget(ik_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            move_group_interface.move();
        }
        else
        {
            return;
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
      }
    }
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(arthrobot_task_server_node::ArthrobotTaskServerNode)