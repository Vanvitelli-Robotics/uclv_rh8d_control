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

#ifndef FINGERTIP_SENSORS_BROADCASTER__FINGERTIP_SENSORS_BROADCASTER_HPP_
#define FINGERTIP_SENSORS_BROADCASTER__FINGERTIP_SENSORS_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "fingertip_sensors_broadcaster/visibility_control.h"
#include "fingertip_sensors_broadcaster/fingertip_sensors_broadcaster_parameters.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "fingertip_sensors_broadcaster/force_sensor.hpp"
#include "hand_sensors_interfaces/msg/fts3_sensors.hpp"

namespace fingertip_sensors_broadcaster
{
class FingertipSensorsBroadcaster : public controller_interface::ControllerInterface
{
public:
  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  FingertipSensorsBroadcaster();

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FINGERTIP_SENSORS_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::unique_ptr<semantic_components::ForceSensor>> force_sensors_;      //there are 5 force sensors, one for each finger

  using ForceSensors_msg_template = hand_sensors_interfaces::msg::FTS3Sensors;
  using StatePublisher = realtime_tools::RealtimePublisher<ForceSensors_msg_template>;

  rclcpp::Publisher<ForceSensors_msg_template>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;

  //motor ids for left hand
  std::vector<std::pair<std::string, uint16_t>> motor_ids = {
    {"thumb_flexion_motor" , 45},
    {"index_flexion_motor" , 46},
    {"middle_flexion_motor", 47},
    {"ring_small_flexion_motor" , 48}
  };

};

}  // namespace fingertip_sensors_broadcaster

#endif  // FINGERTIP_SENSORS_BROADCASTER__FINGERTIP_SENSORS_BROADCASTER_HPP_
