#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <arthrobot_interfaces/action/arthrobot_task.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std::placeholders;
using namespace std::chrono_literals;

// unit quaternion data-type
struct Quaternion
{
  double w, x, y, z;
};

// This is not in game format, it is in mathematical format. (check wikipedia)
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

      if (task_number == 0 || task_number == 3)
      {
        std::string pose_name;
        if (task_number == 0)
          pose_name = "Servo Ready Pose";
        else
          pose_name = "Default Pose";
        RCLCPP_INFO(get_logger(), "Executing <%s> task", pose_name.c_str());
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();

        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        move_group_interface.setMaxVelocityScalingFactor(1);
        move_group_interface.setMaxAccelerationScalingFactor(1);
        const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arthrobot_arm");
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
        std::vector<double> joint_angles;

        /* Help to get positions that works */
        current_state->copyJointGroupPositions(joint_model_group, joint_angles);
        const Eigen::Isometry3d &ee_link = current_state->getGlobalLinkTransform("gripper_support");
        RCLCPP_INFO_STREAM(get_logger(), "current translation: " << ee_link.translation());
        // rotation matrix
        Eigen::Matrix3d rotation_matrix = ee_link.rotation();
        // Convert to euler
        Eigen::Vector3d rpy = rotation_matrix.eulerAngles(2, 1, 0); // order id yaw - pitch- roll
        RCLCPP_INFO_STREAM(get_logger(), " Current rotation: Roll: " << rpy[2] << ", Pitch: " << rpy[1] << ", Yaw: " << rpy[0]);

        // specify joint angles destination in radians
        if (task_number == 0)
        { // default pose (straight up)
          joint_angles[0] = 0.0;
          joint_angles[1] = 0.0;
          joint_angles[2] = 0.0;
          joint_angles[3] = 0.0;
          joint_angles[4] = 0.0;
        }
        else
        { // servoing ready pose
          joint_angles[0] = 0.0;
          joint_angles[1] = -0.628;
          joint_angles[2] = -1.0472;
          joint_angles[3] = 0.0;
          joint_angles[4] = 0.17453;
        }

        move_group_interface.setJointValueTarget(joint_angles);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_interface.move(); // execute the planned trajectory
          RCLCPP_INFO(get_logger(), "<%s> task executed successfully", pose_name.c_str());
        }
        else
        {
          RCLCPP_INFO(get_logger(), "<%s> task execution failed (no plan found)", pose_name.c_str());
          return;
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "<%s> task succeeded", pose_name.c_str());
      }
      // OPEN / CLOSE gripper
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
      // IK poses
      else if (task_number == 4 || task_number == 5)
      {
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();

        geometry_msgs::msg::Pose ik_pose;

        if (task_number == 4)
        {
          ik_pose.position.x = -0.22;
          ik_pose.position.y = 0.04;
          ik_pose.position.z = 0.06;
          Quaternion q = ToQuaternion(-3.14159 / 2, 0.0, 3.14159 / 2);
          ik_pose.orientation.x = q.x;
          ik_pose.orientation.y = q.y;
          ik_pose.orientation.z = q.z;
          ik_pose.orientation.w = q.w;
        }
        else
        {
          ik_pose.position.x = 0.22;
          ik_pose.position.y = 0.04;
          ik_pose.position.z = 0.06;
          Quaternion q = ToQuaternion(-3.14159 / 2, 0.0, -3.14159 / 2);
          ik_pose.orientation.x = q.x;
          ik_pose.orientation.y = q.y;
          ik_pose.orientation.z = q.z;
          ik_pose.orientation.w = q.w;
        }

        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        move_group_interface.setMaxVelocityScalingFactor(0.4);
        move_group_interface.setMaxAccelerationScalingFactor(0.4);
        move_group_interface.setApproximateJointValueTarget(ik_pose); // approximation required because of 5-DOF
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          move_group_interface.move();
          result->success = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal succeeded");
        }
        else
        {
          result->success = false;
          goal_handle->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal failed");
        }
      }

      // Add / remove collision object
      else if (task_number == 6 || task_number == 7)
      {
        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();
        auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_arm");
        auto const collision_object = [frame_id =
                                           move_group_interface.getPlanningFrame()]
        {
          moveit_msgs::msg::CollisionObject collision_object;
          collision_object.header.frame_id = frame_id;
          collision_object.id = "box1";
          shape_msgs::msg::SolidPrimitive primitive;

          // Define the size of the box in meters
          primitive.type = primitive.BOX;
          primitive.dimensions.resize(3);
          primitive.dimensions[primitive.BOX_X] = 0.05;
          primitive.dimensions[primitive.BOX_Y] = 0.13;
          primitive.dimensions[primitive.BOX_Z] = 0.25;

          // Define the pose of the box (relative to the frame_id)
          geometry_msgs::msg::Pose box_pose;
          box_pose.orientation.w = 1.0;
          box_pose.position.x = 0.0;
          box_pose.position.y = 0.2;
          box_pose.position.z = 0.125;

          collision_object.primitives.push_back(primitive);
          collision_object.primitive_poses.push_back(box_pose);
          collision_object.operation = collision_object.ADD;

          return collision_object;
        }();
        // Add the collision object to the scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        if (task_number == 6)
          planning_scene_interface.applyCollisionObject(collision_object);
        else
        {
          planning_scene_interface.removeCollisionObjects({"box1"});
          rclcpp::sleep_for(std::chrono::milliseconds(500)); // Give time for update
          auto objects = planning_scene_interface.getKnownObjectNames();
          RCLCPP_INFO(get_logger(), "Objects remaining: %ld", objects.size());
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Collision object added");
      }
    }
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(arthrobot_task_server_node::ArthrobotTaskServerNode)