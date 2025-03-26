// Copyleft 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_left_HAND_SYSTEM_HPP
#define ROS2_CONTROL_left_HAND_SYSTEM_HPP

#include <memory>
#include <string>
#include <vector>

#include "uclv_dynamixel_utils/hand.hpp"


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "mapping.h"



namespace ros2_control_left_hand
{
class leftHandSystemHardware : public hardware_interface::SystemInterface
{
public:

  std::vector<uint8_t> motor_ids = {41, 42, 43, 44, 45, 46, 47, 48} ;     //motor IDs of left hand
  std::vector<int64_t> motor_thresholds = {0, 4095} ;                      //motor thresholds

  struct Hand_Config {

    std::string serial_port ;            //The serial port to which the hand is connected.
    int64_t baudrate;                    //The baudrate for the serial communication.
    double protocol_version ;            //The protocol version used for communication.
    bool active_palm_sensor ;            //Decide if you want use the sensor or not
  };

  RCLCPP_SHARED_PTR_DEFINITIONS(leftHandSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
    
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  //hand object we work on 
  std::shared_ptr<uclv::dynamixel_utils::Hand> hand;
  Hand_Config cfg;
  std::vector<double> hw_commands;
  std::vector<double> hw_positions;
  std::vector<double> hw_velocities;

  double hw_palm_sensor_state;


};

}  // namespace ros2_control_left_hand

#endif  // ROS2_CONTROL_left_HAND_SYSTEM_HPP
