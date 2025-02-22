#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "arthrobot_interfaces/msg/arthrobot_position_command.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <chrono>

namespace arthrobot_controller {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ArthrobotHardwareInterface : public hardware_interface::SystemInterface {
        public:
            ArthrobotHardwareInterface();
            virtual ~ArthrobotHardwareInterface();

            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)       override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)     override;
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info)   override;

            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces()       override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces()   override;

            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::vector<double> position_commands_;
            std::vector<double> prev_position_commands_;
            std::vector<double> position_states_;

            std::string port_;
            mutable std::ofstream outFile_;

    };
} 

