// Copyleft (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
//
// Authors: Daniel Azanov, Dr. Denis
//


#ifndef INTEGRATOR__INTEGRATOR_HPP_
#define INTEGRATOR__INTEGRATOR_HPP_

#include <memory>
#include <string>
#include <vector>


#include "controller_interface/chainable_controller_interface.hpp"
#include "integrator/visibility_control.h"
#include "integrator/integrator_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "hand_sensors_interfaces/msg/single_command.hpp"
#include "hand_sensors_interfaces/msg/single_output.hpp"

namespace integrator
{

class Integrator : public controller_interface::ChainableControllerInterface
{
public:
  INTEGRATOR__VISIBILITY_PUBLIC
  Integrator();

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  INTEGRATOR__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using IntegratorInputMsg = hand_sensors_interfaces::msg::SingleCommand;
  using IntegratorStateMsg = hand_sensors_interfaces::msg::SingleOutput;

protected:
  controller_interface::return_type update_reference_from_subscribers() override;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  size_t num_input_; 
  std::string input_name_ ; 
  double command = std::numeric_limits<double>::quiet_NaN();
  double input_integrator = std::numeric_limits<double>::quiet_NaN();
  
  // Command subscribers and Integrator State publisher
  rclcpp::Subscription<IntegratorInputMsg>::SharedPtr input_sub_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<IntegratorInputMsg>> input_;
 
  using IntegratorStatePublisher = realtime_tools::RealtimePublisher<IntegratorStateMsg>;
  rclcpp::Publisher<IntegratorStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<IntegratorStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  // internal methods
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();

private:
  // callback for topic interface
  INTEGRATOR__VISIBILITY_LOCAL
  void input_sub_callback(const std::shared_ptr<IntegratorInputMsg> msg);
};

}  // namespace integrator

#endif  // INTEGRATOR__INTEGRATOR_HPP_
