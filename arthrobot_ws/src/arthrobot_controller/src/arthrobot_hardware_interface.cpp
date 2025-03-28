#include "arthrobot_controller/arthrobot_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "arthrobot_interfaces/msg/arthrobot_position_command.hpp"

#include <string>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace arthrobot_controller
{
    double waist_position_feedback {0};
    double shoulder_position_feedback {0};
    double forearm_position_feedback {0};
    double wrist_position_feedback {0};
    double palm_position_feedback {0};
    double gripper_position_feedback {0};

    void data_callback(const arthrobot_interfaces::msg::ArthrobotPositionCommand::SharedPtr msg) 
    {
        waist_position_feedback = msg->waist_pos;
        shoulder_position_feedback = msg->shoulder_pos;
        forearm_position_feedback = msg->forearm_pos;
        wrist_position_feedback = msg->wrist_pos;
        palm_position_feedback = msg->palm_pos;
        gripper_position_feedback = msg->gripper_pos;
    }


    auto node = rclcpp::Node::make_shared("hardware_interface_node");
    auto data_publisher = node->create_publisher<arthrobot_interfaces::msg::ArthrobotPositionCommand>("arthrobot_hardware_commands", 10);
    auto data_subscriber = node->create_subscription<arthrobot_interfaces::msg::ArthrobotPositionCommand>("arthrobot_hardware_feedback", 10, data_callback);

    ArthrobotHardwareInterface::ArthrobotHardwareInterface()
    {

    }   

    ArthrobotHardwareInterface::~ArthrobotHardwareInterface() {
        
    }

    CallbackReturn ArthrobotHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if(result != CallbackReturn::SUCCESS) {
            return result;
        }

        /** Allocate memory for the command and state vectors **/
        prev_position_commands_.reserve(info_.joints.size());
        position_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArthrobotHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(std::size_t i {}; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArthrobotHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(std::size_t i {}; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn ArthrobotHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_sate) {
        (void)previous_sate;    // supress unused parameter compiler warning
        RCLCPP_INFO(rclcpp::get_logger("ArthrobotHardwareInterface"), \
                    "Starting arthrobot Hardware");

        position_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        RCLCPP_INFO(rclcpp::get_logger("ArthrobotHardwareInterface"), \
                    "arthrobot Hardware Activated. Ready to take commands");
        return CallbackReturn::SUCCESS;
    }
    
    
    
    CallbackReturn ArthrobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_sate) {
        (void)previous_sate;    // supress unused parameter compiler warning
        RCLCPP_INFO(rclcpp::get_logger("ArthrobotHardwareInterface"), \
                    "Deactivating arthrobot Hardware");

        RCLCPP_INFO(rclcpp::get_logger("ArthrobotHardwareInterface"), \
                "arthrobot Hardware Deactivated.");
        return CallbackReturn::SUCCESS;
    }


    // Reading the current position and velocity of the joints
    hardware_interface::return_type  ArthrobotHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time;
        (void)period;
        rclcpp::spin_some(node);
        position_states_[0] = waist_position_feedback;
        position_states_[1] = shoulder_position_feedback;
        position_states_[2] = forearm_position_feedback;
        position_states_[3] = wrist_position_feedback;
        position_states_[4] = palm_position_feedback;
        position_states_[5] = gripper_position_feedback;

        return hardware_interface::return_type::OK;
    }
            
    // Writing the position command to the joints
    hardware_interface::return_type ArthrobotHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time;
        (void)period;
        if(position_commands_ == prev_position_commands_) {
            return hardware_interface::return_type::OK;
        }

        arthrobot_interfaces::msg::ArthrobotPositionCommand msg;
            
        msg.waist_pos = position_commands_.at(0);
        msg.shoulder_pos = position_commands_.at(1);
        msg.forearm_pos = position_commands_.at(2);
        msg.wrist_pos = position_commands_.at(3);
        msg.palm_pos = position_commands_.at(4);
        msg.gripper_pos = position_commands_.at(5);

        data_publisher->publish(msg);
        
        prev_position_commands_ = position_commands_;

        return hardware_interface::return_type::OK;
    }

} // namespace arthrobot_controller

PLUGINLIB_EXPORT_CLASS(arthrobot_controller::ArthrobotHardwareInterface, hardware_interface::SystemInterface)
