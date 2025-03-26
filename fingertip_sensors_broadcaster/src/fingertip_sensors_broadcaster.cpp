// Copyleft (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

/*
 * Authors: Subhas Das, Denis Stogl - modified by Giuliana Bianchini
 */

#include "fingertip_sensors_broadcaster/fingertip_sensors_broadcaster.hpp"


#include <memory>
#include <string>

namespace fingertip_sensors_broadcaster
{
  FingertipSensorsBroadcaster::FingertipSensorsBroadcaster()
  : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn FingertipSensorsBroadcaster::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn FingertipSensorsBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();
    
    if (params_.sensor_names.size() != 0){

      for (const auto &sensor_name : params_.sensor_names)
      {
        force_sensors_.push_back(std::make_unique<semantic_components::ForceSensor>(
          semantic_components::ForceSensor(sensor_name)));
      }
    } else {
      RCLCPP_ERROR( get_node()->get_logger(), "There isn't any sensor specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    try{

      // register ft sensor data publisher
      sensor_state_publisher_ = get_node()->create_publisher<ForceSensors_msg_template>(
        "~/fingertip_force_sensors", rclcpp::SystemDefaultsQoS());
      realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
    }
    catch (const std::exception & e){
      fprintf(
        stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
        e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    realtime_publisher_->lock();
      if(!params_.frame_id.empty()){
        realtime_publisher_->msg_.header.frame_id = params_.frame_id;
      }
    realtime_publisher_->unlock();

    RCLCPP_DEBUG(get_node()->get_logger(), "Fingertip Sensors Broadcaster successfully configured");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  FingertipSensorsBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  FingertipSensorsBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &sensor : force_sensors_){ 
      for (size_t i = 0; i < sensor->get_state_interface_names().size(); i++){
        state_interfaces_config.names.push_back(sensor->get_state_interface_names()[i]);
      }
    }
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn FingertipSensorsBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (const auto &sensor : force_sensors_){
      sensor->assign_loaned_state_interfaces(state_interfaces_);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn FingertipSensorsBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (const auto &sensor : force_sensors_){
      sensor->release_interfaces();
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

controller_interface::return_type FingertipSensorsBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{ 
  
  if (realtime_publisher_ && realtime_publisher_->trylock()){
    
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.forces.resize(params_.sensor_names.size());
    realtime_publisher_->msg_.motor_ids.resize(params_.sensor_names.size());
    realtime_publisher_->msg_.motor_names.resize(params_.sensor_names.size());
    realtime_publisher_->msg_.norm_forces.resize(params_.sensor_names.size());

    for (size_t i = 0; i < force_sensors_.size() ; i++){
      force_sensors_[i]->get_values_as_message(realtime_publisher_->msg_.forces[i]);
      realtime_publisher_->msg_.norm_forces[i] = force_sensors_[i]->get_norm();
    }
    
    int i = 0;
    for (const auto &motor : motor_ids)
    {
      auto motor_name = motor.first;
      auto motor_id = motor.second;
      realtime_publisher_->msg_.motor_names[i] = motor_name ;
      realtime_publisher_->msg_.motor_ids[i] = motor_id ;
      i++;
      if( motor_name == "ring_small_flexion_motor"){
        realtime_publisher_->msg_.motor_names[i] = motor_name ;
        realtime_publisher_->msg_.motor_ids[i] = motor_id ;
        i = 0;
      }
    }

    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace fingertip_sensors_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fingertip_sensors_broadcaster::FingertipSensorsBroadcaster,
  controller_interface::ControllerInterface)
