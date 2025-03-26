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

#include "slipping_avoidance/slipping_avoidance.hpp"

#include <memory>
#include <string>

namespace // utility
{

  using PoseMsg = slipping_avoidance::SlippingAvoidance::PoseMsg;

} // namespace

namespace slipping_avoidance
{
  SlippingAvoidance::SlippingAvoidance()
      : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn SlippingAvoidance::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during slipping_avoidance init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  void SlippingAvoidance::update_parameters()
  {
    if (!param_listener_->is_old(params_))
    {
      return;
    }
    params_ = param_listener_->get_params();
  }

  controller_interface::CallbackReturn SlippingAvoidance::configure_parameters()
  {
    update_parameters();

    state_interface_name_ = params_.state_interface_name;

    if (params_.friction_coefficient < 0.0)
    {

      RCLCPP_ERROR(get_node()->get_logger(),
                   "Friction coefficient is negative. The coefficient has to be positive.");

      return CallbackReturn::ERROR;
    }
    friction_coefficient = params_.friction_coefficient;
    min_desired_normal_force = params_.min_desired_normal_force;

    if (params_.use_aruco)
    {
      if (params_.camera_name.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "camera_name is empty. In use_auruco mode the param 'camera_name' has to be set.");
        return CallbackReturn::ERROR;
      }
      use_aruco = true;
      camera_name = params_.camera_name;
    }
    use_aruco = false;

    if (params_.is_chained_mode)
    {
      is_chained_mode_ = true;

      if (params_.next_controller_names.size() == 0)
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Next_controller_name is empty. In chained mode the param 'next_controller_name' has to be set.");
        return CallbackReturn::ERROR;
      }

      // save the next controller names
      for (const auto &next_controller : params_.next_controller_names)
      {
        next_controller_names_.push_back(next_controller);
      }

      if (params_.command_interface_name.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Command interface name is empty. In chained mode the param has to be set.");
        return CallbackReturn::ERROR;
      }

      if (params_.command_interface_type.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Command interface type is empty. In chained mode the param has to be set.");
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::string SlippingAvoidance::getPrefix(const std::string &str)
  {

    size_t pos = str.find('_'); // find the first '_'
    if (pos != std::string::npos)
    {
      return str.substr(0, pos);
    }
    return str;
  }

  void SlippingAvoidance::aruco_pose_callback(const std::shared_ptr<PoseMsg> msg)
  {
    aruco_pose = msg;
    return;
  }

  controller_interface::CallbackReturn SlippingAvoidance::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    auto ret = configure_parameters();
    if (ret != CallbackReturn::SUCCESS)
    {
      return ret;
    }

    force_sensor_ = std::make_shared<semantic_components::ForceSensor>(
        semantic_components::ForceSensor(state_interface_name_));

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    aruco_pose_sub = get_node()->create_subscription<PoseMsg>(
        camera_name + "/resin_block/" + getPrefix(state_interface_name_) + "/pose", subscribers_qos,
        std::bind(&SlippingAvoidance::aruco_pose_callback, this, std::placeholders::_1));

    normal_direction_camera = Eigen::Vector3d::Zero();
    normal_direction_aruco = Eigen::Vector3d::Zero();

    try
    {
      // normal direction publisher
      n_publisher_ = get_node()->create_publisher<NormalDirectionMsg>(
          "~/normal_direction", rclcpp::SystemDefaultsQoS());
      normal_publisher_ = std::make_unique<NormalDirectionPublisher>(n_publisher_);

      // error publisher
      e_publisher_ = get_node()->create_publisher<ErrorMsg>(
          "~/output", rclcpp::SystemDefaultsQoS());
      error_publisher_ = std::make_unique<ErrorPublisher>(e_publisher_);
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), " SlippingAvoidance successfully configured ");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  SlippingAvoidance::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    std::string command_interface_name;
    if (is_chained_mode_)
    {
      for (const auto &controller_name : next_controller_names_)
      {
        command_interface_name = command_interface_name + controller_name + "/";
      }
      command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      command_interfaces_config.names.push_back(
          command_interface_name + params_.command_interface_name + "/" + params_.command_interface_type);
    }
    else
    {
      command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    }

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  SlippingAvoidance::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (size_t i = 0; i < force_sensor_->get_state_interface_names().size(); i++)
    {
      state_interfaces_config.names.push_back(force_sensor_->get_state_interface_names()[i]);
    }

    return state_interfaces_config;
  }

  Eigen::Vector3d SlippingAvoidance::ComputeNormalDirection()
  {
    std::array<double, 3> measured_force = force_sensor_->get_forces();
    double norm_force = force_sensor_->get_norm();

    if (norm_force == 0)
      return Eigen::Vector3d::Zero();

    return Eigen::Vector3d(measured_force[0] / norm_force,
                           measured_force[1] / norm_force,
                           measured_force[2] / norm_force);
  }

  Eigen::Matrix3d SlippingAvoidance::ComputeArucoRotationMatrix(geometry_msgs::msg::PoseStamped &pose)
  {
    // convert geometry_msgs::msg::Quaternion into Eigen::Quaternion
    Eigen::Quaterniond q(pose.pose.orientation.w,
                         pose.pose.orientation.x,
                         pose.pose.orientation.y,
                         pose.pose.orientation.z);

    // return q.toRotationMatrix();
    return Eigen::Matrix3d::Identity();
  }

  controller_interface::CallbackReturn SlippingAvoidance::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    force_sensor_->assign_loaned_state_interfaces(state_interfaces_);

    aruco_pose = nullptr;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn SlippingAvoidance::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    state_interface_name_.clear();
    aruco_pose = nullptr;
    normal_direction_camera = Eigen::Vector3d::Zero();
    normal_direction_aruco = Eigen::Vector3d::Zero();
    force_sensor_->release_interfaces();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type SlippingAvoidance::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {

    std::array<double, 3> force = force_sensor_->get_forces();
    Eigen::Vector3d measured_force(force[0], force[1], force[2]);
    double measured_normal_force;
    Eigen::Vector3d tangential_force;

    if (use_aruco)
    {
      if (aruco_pose == nullptr)
      {
        return controller_interface::return_type::OK;
      }
      else
        active = true;

      if (active && first)
      {
        camera_R_aruco = ComputeArucoRotationMatrix(*aruco_pose);
        // calculate the normal_direction in camera_frame
        normal_direction_camera = camera_R_aruco * ComputeNormalDirection();
        first = false;
        return controller_interface::return_type::OK;
      }

      // calculate normal direction in aruco frame - from camera to aruco
      camera_R_aruco = ComputeArucoRotationMatrix(*aruco_pose);
      normal_direction_aruco = camera_R_aruco.transpose() * normal_direction_camera;

      measured_normal_force = normal_direction_aruco.transpose().dot(measured_force);
      tangential_force = measured_force - (measured_normal_force * normal_direction_aruco);
      desired_normal_force = (tangential_force.norm() / friction_coefficient) + min_desired_normal_force;
    }
    else
    {
      active = true;
      if (active && first)
      {
        // calculate the normal_direction in camera_frame
        normal_direction_camera = ComputeNormalDirection();
        first = false;
        return controller_interface::return_type::OK;
      }
      measured_normal_force = normal_direction_camera.transpose().dot(measured_force);
      tangential_force = measured_force - (measured_normal_force * normal_direction_camera);
      desired_normal_force = (tangential_force.norm() / friction_coefficient) + min_desired_normal_force;
    }

    // compute the next controller input - this is the error between desired normal force and measured normal force
    command = desired_normal_force - measured_normal_force;

    // write calculated value
    for (size_t i = 0; i < command_interfaces_.size(); i++)
    {
      command_interfaces_[i].set_value(command);
    }

    if (normal_publisher_ && normal_publisher_->trylock())
    {
      normal_publisher_->msg_.header.stamp = time;
      normal_publisher_->msg_.vector.x = normal_direction_camera.x();
      normal_publisher_->msg_.vector.y = normal_direction_camera.y();
      normal_publisher_->msg_.vector.z = normal_direction_camera.z();
      normal_publisher_->unlockAndPublish();
    }

    if (error_publisher_ && error_publisher_->trylock())
    {
      error_publisher_->msg_.header.stamp = time;
      error_publisher_->msg_.desired_force = desired_normal_force;
      error_publisher_->msg_.measured_force = measured_normal_force;
      error_publisher_->msg_.tangential_force = tangential_force.norm();
      error_publisher_->msg_.error = command;
      error_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace slipping_avoidance

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    slipping_avoidance::SlippingAvoidance,
    controller_interface::ControllerInterface)
