/***  plugin dell'interfaccia hardware dei sensori ***/// Copyleft 2021 ros2_control Development Team
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

#include "ros2_control_left_hand/fingertip_sensors.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <string>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace ros2_control_fingertip_sensors
{
hardware_interface::CallbackReturn FingertipSensorsHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("controller_manager.resource_manager.hardware_component.sensor.FingertipSensors"));
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

    //parameters from hardware_interface
    cfg.serial_port = info_.hardware_parameters["serial_port"];
    cfg.baudrate  = std::stoi(info_.hardware_parameters["baudrate"].c_str());                   

    hw_sensors_state.resize(info_.sensors.size(), std::vector<double>(3, std::numeric_limits<double>::quiet_NaN()));

    sensors_reading.forces.resize(info_.sensors.size());
    sensors_reading.motor_ids.resize(info_.sensors.size());

    timestamp = false ; 
    timestamp_offset = 0;

    //Set the serial port to low latency mode
    std::cout << "Setting low latency for " << WARN_COLOR << cfg.serial_port << CRESET << std::endl;
    std::string command = "setserial " + cfg.serial_port + " low_latency";
    int result = system(command.c_str());
    std::cout << "Set low latency for " << WARN_COLOR << cfg.serial_port << CRESET
                << " result: " << SUCCESS_COLOR << result << CRESET << std::endl;


    sensor_read = std::make_shared<serial::Serial>(cfg.serial_port, cfg.baudrate, serial::Timeout::simpleTimeout(cfg.serial_timeout));
    
    // Initialize sensor communication by flushing and sending necessary commands
    sensor_read->flush();
    const std::vector<std::string> commands = {
    "pausedata\r\n",
    "calibrate\r\n",
    "enabletime\r\n",
    "resume\r\n"
    };

    for (const auto& cmd : commands) {
        sensor_read->flush();
        sensor_read->write(cmd);
        sensor_read->waitByteTimes(100 * cmd.size());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> FingertipSensorsHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // export sensor state interface
    for (size_t i = 0; i < info_.sensors.size(); i++){
        for( size_t j = 0; j < info_.sensors[0].state_interfaces.size(); j++){
          state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.sensors[i].name, 
          info_.sensors[i].state_interfaces[j].name, 
          &hw_sensors_state[i][j]));     
      }
    }

  return state_interfaces;
}

hardware_interface::CallbackReturn FingertipSensorsHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  // BEGIN
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  
  // reset values always when configuring hardware
    for (size_t i = 0; i < info_.sensors.size(); i++)
    {
      sensors_reading.forces[i].x = 0.0;
      sensors_reading.forces[i].y = 0.0;
      sensors_reading.forces[i].z = 0.0;
    }

    sensors_reading.motor_ids[0] = 45 ; // thumb motor id
    sensors_reading.motor_ids[1] = 46 ; // index motor id 
    sensors_reading.motor_ids[2] = 47 ; // middle motor id
    sensors_reading.motor_ids[3] = 48 ; // ring motor id
    sensors_reading.motor_ids[4] = 48 ; // small motor id

  
    //Sensors Calibration
    try{

        // Perform sensor calibration
        RCLCPP_INFO(get_logger(), "Calibrating sensors...");
        sensor_read->flush();            // Clear communication buffers
        // Send the calibration command
        const std::string &cmd = "calibrate\r\n";
        sensor_read->write(cmd);
        sensor_read->waitByteTimes(100 * cmd.size());
        sensor_read->flush();
        RCLCPP_INFO(get_logger(), "Calibration completed.");
        
        line = sensor_read->readline();
        if(line.empty()){
          const std::string &cmd = "resume\r\n";
          sensor_read->write(cmd);
          sensor_read->waitByteTimes(100 * cmd.size());
          sensor_read->flush();
         RCLCPP_INFO(get_logger(), "Resume completed.");
        }
        
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Calibration failed: %s", e.what());
    }

    // END

  RCLCPP_INFO(get_logger() , "Successfully sensors activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FingertipSensorsHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    sensor_read->flush();
    const std::string cmd = "pausedata\r\n";
    sensor_read->write(cmd);
    sensor_read->waitByteTimes(100 * cmd.size());
    sensor_read->flush();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FingertipSensorsHardware::read(
  const rclcpp::Time &  /*time*/, const rclcpp::Duration & period)
{
   if (sensor_read->available() >= 40)
   {
      
    line = sensor_read->readline();

    if (line.empty())
    {
        RCLCPP_WARN(get_logger(), "No data received from the sensor.");
        return hardware_interface::return_type::OK;

    }
      
    // Tokenize the received line using commas as delimiters
    std::string token;
    std::vector<std::string> tokens;
    char delimiter = ',';
    std::stringstream ss(line);

    while (getline(ss, token, delimiter))
    {
      tokens.push_back(token);
    }
    tokens.pop_back(); // Remove any empty token at the end
    
    sensor_read->flush();

    // Check if there is a timestamp in the data
    if (tokens.size() == 18)
    {
      timestamp = true;
      timestamp_offset = 2;
  
      // Parse and assign sensor forces
      if (tokens[0] == "@")
      {
        for (size_t i = 1; i < 6; i++)
        {   
          auto fx = tokens[timestamp_offset - 2 + 3 * i];
          auto fy = tokens[timestamp_offset - 1 + 3 * i];
          auto fz = tokens[timestamp_offset + 0 + 3 * i];
          try{
            hw_sensors_state[i-1][0] = std::stod(fx);
            hw_sensors_state[i-1][1] = std::stod(fy);
            hw_sensors_state[i-1][2] = std::stod(fz); 
          }
          catch (const std::invalid_argument& e) {
            std::cerr << e.what() << '\n';
          }
        }
      }  
    } 
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_fingertip_sensors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_fingertip_sensors::FingertipSensorsHardware, hardware_interface::SensorInterface)
