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

#include "palm_sensor_broadcaster/palm_sensor_broadcaster.hpp"


#include <memory>
#include <string>

namespace palm_sensor_broadcaster
{
  PalmSensorBroadcaster::PalmSensorBroadcaster()
  : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn PalmSensorBroadcaster::on_init()
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

  controller_interface::CallbackReturn PalmSensorBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();

    if(params_.sensor_name.empty()){
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Sensor name has to be specified for PalmSensorBroadcaster");
      return controller_interface::CallbackReturn::ERROR;
    }

    tof_sensor_ = std::make_unique<semantic_components::TOFSensor>(
          semantic_components::TOFSensor(params_.sensor_name));

    try
    {
      // register tof sensor data publisher
      sensor_state_publisher_ = get_node()->create_publisher<TOFSensor_msg_template>(
        "~/distance_measured", rclcpp::SystemDefaultsQoS());
      realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
    }
    catch (const std::exception & e)
    {
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

    RCLCPP_DEBUG(get_node()->get_logger(), "PalmSensorBroadcaster successfully configured");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  PalmSensorBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  PalmSensorBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.push_back(tof_sensor_->get_state_interface_names()[0]);
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn PalmSensorBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
   
    tof_sensor_->assign_loaned_state_interfaces(state_interfaces_);
    
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn PalmSensorBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
     
    tof_sensor_->release_interfaces();
    
    return controller_interface::CallbackReturn::SUCCESS;
  }

controller_interface::return_type PalmSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{ 
  
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    
    realtime_publisher_->msg_.header.stamp = time;

    tof_sensor_->get_values_as_message(realtime_publisher_->msg_);
    
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace palm_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  palm_sensor_broadcaster::PalmSensorBroadcaster,
  controller_interface::ControllerInterface)
