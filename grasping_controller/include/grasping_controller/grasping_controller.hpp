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

#ifndef GRASPING_CONTROLLER__GRASPING_CONTROLLER_HPP_
#define GRASPING_CONTROLLER__GRASPING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "grasping_controller/visibility_control.h"
#include "grasping_controller/grasping_controller_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "fingertip_sensors_broadcaster/force_sensor.hpp"
#include "palm_sensor_broadcaster/tof_sensor.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "hand_sensors_interfaces/msg/single_command.hpp"

namespace grasping_controller
{
  enum class GraspingState : int
  {
    MEASURING_DISTANCE = 0, // state wherein the distance  is measured from palm_sensor.
    CLOSING = 1,            // state wherein the integrator is used with constant velocity to closing the hand
    FORCE_CONTROL = 2,      // state wherein the force control starts
    SLIPPING_AVOIDANCE = 3  // state wherein the slipping avoidance control starts
  };

  class GraspingController : public controller_interface::ControllerInterface
  {
  public:
    GRASPING_CONTROLLER_PUBLIC
    GraspingController();

    GRASPING_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    GRASPING_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    GRASPING_CONTROLLER_PUBLIC controller_interface::CallbackReturn on_init() override;

    GRASPING_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    GRASPING_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    GRASPING_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    GRASPING_CONTROLLER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period) override;

    using SingleCommandMsg = hand_sensors_interfaces::msg::SingleCommand;

  protected:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::vector<std::shared_ptr<semantic_components::ForceSensor>> force_sensors_;
    std::unique_ptr<semantic_components::TOFSensor> tof_sensor_;

    using SingleCommandPublisher = realtime_tools::RealtimePublisher<SingleCommandMsg>;
    std::vector<rclcpp::Publisher<SingleCommandMsg>::SharedPtr> i_publishers_;
    std::vector<std::unique_ptr<SingleCommandPublisher>> integrator_publishers_;

    // error publisher
    std::vector<rclcpp::Publisher<SingleCommandMsg>::SharedPtr> e_publishers_;
    std::vector<std::unique_ptr<SingleCommandPublisher>> error_publishers_;

    // controller manager service client - proportional activation
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr cm_srv_client_;
    // controller manager service request
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> srv_request;

    // map between sensor's name and their desired force values
    std::map<std::string, std::pair<int, double>> f_des_sensors_map_;

    // subscriber to desired force topic
    std::vector<rclcpp::Subscription<SingleCommandMsg>::SharedPtr> f_des_subs_;
    // std::vector<realtime_tools::RealtimeBuffer<std::shared_ptr<SingleCommandMsg>>> f_des_buffers;

    double distance_threshold;
    double force_threshold;

    std::vector<std::string> command_interface_types_;

    GraspingState current_state = GraspingState::MEASURING_DISTANCE;
    GraspingState last_state;

    std::vector<std::string> fingers_in_force;
    std::vector<std::string> controllers_activated;
    double command_flexion;
    double command_adduction;
    bool print = true;
    rclcpp::Time time_fingers_ok_;
    rclcpp::Time time_request;


    

    // internal methods
    void printState(GraspingState &last_state, GraspingState &current_state);
    void OpenCloseHand(bool mode, const rclcpp::Time &time);
    void ActiveControllers(std::vector<std::string> &sensor_names, const std::string controller_name);
    //overload
    void ActiveControllers(const std::string &sensor_names, const std::string controller_name);
    void DeactiveControllers(std::vector<std::string> &sensor_names, const std::string controller_name);

  private:
    // callback for desired force values
    void f_des_sub_callback(const std::shared_ptr<SingleCommandMsg> msg);
    // function for compute the force error
    double compute_error(std::shared_ptr<semantic_components::ForceSensor> force_sensor);
  };

} // namespace grasping_controller

#endif // GRASPING_CONTROLLER__GRASPING_CONTROLLER_HPP_
