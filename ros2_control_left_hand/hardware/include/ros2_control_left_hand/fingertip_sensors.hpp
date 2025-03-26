/** .hpp of hardware interface plugin **/

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

#ifndef ROS2_CONTROL_FINGERTIP_SENSORS_HPP
#define ROS2_CONTROL_FINGERTIP_SENSORS_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "uclv_dynamixel_utils/colors.hpp"
#include "hand_sensors_interfaces/msg/fts3_sensors.hpp"
#include "serial/serial.h"



namespace ros2_control_fingertip_sensors
{
class FingertipSensorsHardware : public hardware_interface::SensorInterface
{
public:

  struct Fingertips_Sensors_Config {

    std::string serial_port ;            //The serial port where the sensors are connected.
    int64_t baudrate;                    //The baudrate for the serial communication.
    uint32_t serial_timeout = 1000;     // Timeout for serial communication
  };

  RCLCPP_SHARED_PTR_DEFINITIONS(FingertipSensorsHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
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

    // Serial communication objects
    std::shared_ptr<serial::Serial> sensor_read;
    std::string line;

    //  timestamp offset
    bool timestamp;       //lo metterei come parametro da urdf
    int timestamp_offset;

    //sensor object we work on 
    Fingertips_Sensors_Config cfg;
    hand_sensors_interfaces::msg::FTS3Sensors sensors_reading;
    //std::vector<geometry_msgs::msg::Vector3> hw_sensors_state;
    std::vector<std::vector<double>> hw_sensors_state;
};

}  // namespace ros2_control_fingertip_sensors

#endif  // ROS2_CONTROL_FINGERTIP_SENSORS_HPP
