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

#ifndef SLIPPING_AVOIDANCE__SLIPPING_AVOIDANCE_HPP_
#define SLIPPING_AVOIDANCE__SLIPPING_AVOIDANCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "slipping_avoidance/visibility_control.h"
#include "slipping_avoidance/slipping_avoidance_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "fingertip_sensors_broadcaster/force_sensor.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hand_sensors_interfaces/msg/slipping_avoidance_state.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"


namespace slipping_avoidance
{

class SlippingAvoidance : public controller_interface::ControllerInterface
{
public:
  SLIPPING_AVOIDANCE_PUBLIC
  SlippingAvoidance();

  SLIPPING_AVOIDANCE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SLIPPING_AVOIDANCE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SLIPPING_AVOIDANCE_PUBLIC controller_interface::CallbackReturn on_init() override;

  SLIPPING_AVOIDANCE_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SLIPPING_AVOIDANCE_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SLIPPING_AVOIDANCE_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SLIPPING_AVOIDANCE_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;


  using NormalDirectionMsg = geometry_msgs::msg::Vector3Stamped; 
  using ErrorMsg = hand_sensors_interfaces::msg::SlippingAvoidanceState;
  using PoseMsg = geometry_msgs::msg::PoseStamped;

protected:

  //internal methods
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();
  Eigen::Vector3d ComputeNormalDirection();
  Eigen::Matrix3d ComputeArucoRotationMatrix( geometry_msgs::msg::PoseStamped &pose);
  std::string getPrefix ( const std::string &str);
  

  // normal direction publisher
  using NormalDirectionPublisher = realtime_tools::RealtimePublisher<NormalDirectionMsg>;
  rclcpp::Publisher<NormalDirectionMsg>::SharedPtr n_publisher_;
  std::unique_ptr<NormalDirectionPublisher> normal_publisher_;

  // error publisher
  using ErrorPublisher = realtime_tools::RealtimePublisher<ErrorMsg>;
  rclcpp::Publisher<ErrorMsg>::SharedPtr e_publisher_;
  std::unique_ptr<ErrorPublisher> error_publisher_;

  //subscriber to finger pose
  rclcpp::Subscription<PoseMsg>::SharedPtr aruco_pose_sub;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::string state_interface_name_;
  std::vector<std::string> next_controller_names_;
  bool is_chained_mode_ = false;
  double friction_coefficient; 
  double min_desired_normal_force;
  bool use_aruco ;
  std::string camera_name;

  std::shared_ptr<PoseMsg> aruco_pose; 
  bool active = false;
  bool first = true;
  std::shared_ptr<semantic_components::ForceSensor> force_sensor_;  
  Eigen::Matrix3d camera_R_aruco; 
  double desired_normal_force;
  Eigen::Vector3d normal_direction_camera;
  Eigen::Vector3d normal_direction_aruco;
  double command = std::numeric_limits<double>::quiet_NaN();


private:

  // callback for finger pose 
  void aruco_pose_callback(const std::shared_ptr<PoseMsg> msg);
    
};

}  // namespace slipping_avoidance

#endif  // SLIPPING_AVOIDANCE__SLIPPING_AVOIDANCE_HPP_
