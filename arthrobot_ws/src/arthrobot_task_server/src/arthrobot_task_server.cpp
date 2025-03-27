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

namespace arthrobot_task_server_node
{
class ArthrobotTaskServerNode : public rclcpp::Node
{
  private:
    rclcpp_action::Server<arthrobot_interfaces::action::ArthrobotTask>::SharedPtr arthrobot_task_server;

  public:
    explicit ArthrobotTaskServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : 
                                                    Node("arthrobot_task_server_node", options)
    {
      RCLCPP_INFO(get_logger(), "Starting Arthrobot Task Server");
      arthrobot_task_server = rclcpp_action::create_server<arthrobot_interfaces::action::ArthrobotTask>(
        this, "arthrobot_task_server", std::bind(&ArthrobotTaskServerNode::GoalCallback, this, _1, _2),
        std::bind(&ArthrobotTaskServerNode::cancelCallback, this, _1),
        std::bind(&ArthrobotTaskServerNode::acceptedCallback, this, _1));
    }

  
    /* The callback functions */
  private:
    rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const arthrobot_interfaces::action::ArthrobotTask::Goal> goal)
    {
      (void)uuid; // to suppress unused variable warning
      if(goal->task_number == 1) {
        RCLCPP_INFO(get_logger(), "Received <Gripper Open> task");
      }
      else if(goal->task_number == 2) {
        RCLCPP_INFO(get_logger(), "Received t<Gripper Close> task");
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arthrobot_interfaces::action::ArthrobotTask>> goal_handle)
    {
      (void)goal_handle; // to suppress unused variable warning
      int task_number = goal_handle->get_goal()->task_number;
      
      if(task_number == 1){
        RCLCPP_INFO(get_logger(), "Received request to cancel <Gripper Open> task");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_ee");
        gripper_move_group.stop();
      }
      else if(task_number == 2){
        RCLCPP_INFO(get_logger(), "Received request to cancel <Gripper Close> task");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_ee");
        gripper_move_group.stop();
      }
      
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arthrobot_interfaces::action::ArthrobotTask>> goal_handle)
    {
      std::thread{ std::bind(&ArthrobotTaskServerNode::taskExecute, this, _1), goal_handle }.detach();
    }

    void taskExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arthrobot_interfaces::action::ArthrobotTask>> goal_handle)
    {
      int task_number = goal_handle->get_goal()->task_number;

      if(task_number == 1 || task_number == 2){

        std::string gripper_task_name;
        if(task_number ==1) gripper_task_name = "Gripper Open";
        else gripper_task_name = "Gripper Close";

        RCLCPP_INFO(get_logger(), "Executing <%s> task", gripper_task_name.c_str());

        auto result = std::make_shared<arthrobot_interfaces::action::ArthrobotTask::Result>();
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arthrobot_ee");
        const moveit::core::JointModelGroup* joint_model_group;
        joint_model_group = gripper_move_group.getCurrentState()->getJointModelGroup("arthrobot_ee");
        moveit::core::RobotStatePtr current_state = gripper_move_group.getCurrentState(10);
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        if(task_number == 1)
            joint_group_positions[0] = 1.57;
        else
            joint_group_positions[0] = 0;

        gripper_move_group.setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
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

    }
  
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(arthrobot_task_server_node::ArthrobotTaskServerNode)