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
 * Authors: Subhas Das, Denis Stogl
 */

#ifndef PALM_SENSOR_BROADCASTER__PALM_SENSOR_BROADCASTER_HPP_
#define PALM_SENSOR_BROADCASTER__PALM_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "palm_sensor_broadcaster/visibility_control.h"
#include "palm_sensor_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "palm_sensor_broadcaster/tof_sensor.hpp"
#include "hand_sensors_interfaces/msg/palm_sensor.hpp"

namespace palm_sensor_broadcaster
{
class PalmSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  PALM_SENSOR_BROADCASTER_PUBLIC
  PalmSensorBroadcaster();

  PALM_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PALM_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PALM_SENSOR_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  PALM_SENSOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PALM_SENSOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PALM_SENSOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PALM_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<semantic_components::TOFSensor> tof_sensor_;      

  using TOFSensor_msg_template = hand_sensors_interfaces::msg::PalmSensor;
  using StatePublisher = realtime_tools::RealtimePublisher<TOFSensor_msg_template>;

  rclcpp::Publisher<TOFSensor_msg_template>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;

};

}  // namespace palm_sensor_broadcaster

#endif  // PALM_SENSOR_BROADCASTER__PALM_SENSOR_BROADCASTER_HPP_
