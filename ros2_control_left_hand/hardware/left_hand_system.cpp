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

#include "ros2_control_left_hand/left_hand_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_left_hand
{
hardware_interface::CallbackReturn leftHandSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.leftHand"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
  
  //parameters from hardware_interface
  cfg.serial_port = info_.hardware_parameters["serial_port"];
  cfg.baudrate  = std::stoi(info_.hardware_parameters["baudrate"].c_str());                   
  cfg.protocol_version = std::stod(info_.hardware_parameters["protocol_version"].c_str()); 
  if( info_.hardware_parameters["active_palm_sensor"] == "False"){
    cfg.active_palm_sensor = false;
  } else cfg.active_palm_sensor = true;
  
  hand->setSerialPortLowLatency(cfg.serial_port);
  hand = std::make_shared<Hand>(cfg.serial_port, cfg.baudrate, cfg.protocol_version);
  if(!hand->initialize()){
    RCLCPP_ERROR(
        get_logger(), "Hand not initialized");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Hand system has exactly two states and one command interface on each joint
    // command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_INFO(
        get_logger(), "Joint '%s' hasn't command interface",
        joint.name.c_str());
      //return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    //  state interfaces
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  
  }

  hw_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn leftHandSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
 
  // reset values always when configuring hardware - homing position
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    hw_positions[i] = 0.0 ;
    if( i < 3){ 
      hw_commands[i] = 2047.0; 
    } else {
      hw_commands[i] = 0.0 ;
    }
    
  }


  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> leftHandSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    // association between hardware interface and local variables
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities[i]));
  }
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &hw_palm_sensor_state));
    

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> leftHandSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
     command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn leftHandSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  
  for(const auto &id : motor_ids){

    if ( id > 40 && id < 44) {
      hand->addWristMotor(id);
    } else {
      hand->addFingerMotor(id);
    }
  }

  hand->createPalmSensor();
  
  hw_commands[3] = 4000.0 ; //max thumb adduction for grasping
  // END

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn leftHandSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  for(const auto &id : motor_ids){
    if ( id > 40 && id < 44) {
      hand->removeWristMotor(id);

    } else {
      hand->removeFingerMotor(id);
    }
  }
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type leftHandSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  while (rclcpp::ok())
  {
    std::vector<uint32_t> motor_positions_uint32;
    std::vector<double> hw_pos = hw_positions;
    double deltaSeconds = period.seconds();
    try
    {
      
      motor_positions_uint32 = hand->readMotorsPositions(motor_ids);
      // Usa std::transform per convertire da uint32_t a double
      std::transform(motor_positions_uint32.begin(), motor_positions_uint32.end(), hw_positions.begin(),
                [](uint32_t val) { return static_cast<double>(val); });

      if( cfg.active_palm_sensor ){
        //measuring distance from palm sensor
        hw_palm_sensor_state = static_cast<double>(hand->readPalmSensorState());
      }


      // compute velocities
      for(size_t i = 0 ; i< hw_positions.size(); i++){
        hw_velocities[i] = (hw_positions[i] - hw_pos[i] ) / deltaSeconds;
      }
    }
    catch(...)
    {

      RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *this->get_clock(), 1000, "ERROR");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hand->~Hand();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_left_hand ::leftHandSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  while (rclcpp::ok())
  {
    for( size_t i = 0; i< motor_ids.size(); i++){

      /** MAPPING CONTROLLERS VALUES (ANGLES) INTO MOTORS VALUES **/
      //hw_commands[i]= mapping( hw_commands[i], 0.0 , 2 * M_PI , 0.0, 4095);

      if(hw_commands[i] < motor_thresholds[0] || hw_commands[i] > motor_thresholds[1]){

        RCLCPP_ERROR(get_logger(), 
        "ERROR: position %f for motor ID %d out of range [%1ld, %1ld]", hw_commands[i], motor_ids[i], motor_thresholds[0], motor_thresholds[1]);
      }
    }

    std::vector<float> commands_float ;
    commands_float.resize(hw_commands.size());
    // Use std::transform to convert double to float
      std::transform(hw_commands.begin(), hw_commands.end(), commands_float.begin(),
                [](double val) { return static_cast<float>(val); });
    

    try{
      hand->moveMotors(motor_ids, commands_float);
    } catch (...) {
      RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *this->get_clock(), 1000, "ERROR");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }
  
  for (const auto &motor : hand->getWristMotors())
  {
    motor->disableTorque();
  }

  for (const auto &motor : hand->getFingerMotors())
  {
    motor->disableTorque();
  }
  
  hand->~Hand();
  

}

}  // namespace ros2_control_left_hand

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_left_hand::leftHandSystemHardware, hardware_interface::SystemInterface)
