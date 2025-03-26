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

#include "grasping_controller/grasping_controller.hpp"

#include <memory>
#include <string>

namespace // utility
{

  using SingleCommandMsg = grasping_controller::GraspingController::SingleCommandMsg;

} // namespace

namespace grasping_controller
{
  GraspingController::GraspingController()
      : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn GraspingController::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during grasping_controller init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GraspingController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();
    int index = 0;
    srv_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    time_fingers_ok_ = rclcpp::Time(0);
    time_request = rclcpp::Time(0);

    if (params_.sensor_names.size() != 0)
    {
      for (const auto &sensor_name : params_.sensor_names)
      {
        if (sensor_name == "palm_sensor")
        {
          tof_sensor_ = std::make_unique<semantic_components::TOFSensor>(semantic_components::TOFSensor(sensor_name));
        }
        else
        {
          force_sensors_.push_back(std::make_shared<semantic_components::ForceSensor>(
              semantic_components::ForceSensor(sensor_name)));

          // the second element of the pair is the initialization of desired force value
          f_des_sensors_map_.insert({sensor_name, std::make_pair(index, params_.desired_force)});
          index++;
        }
      }
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "There isn't any sensor specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.finger_motors.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Finger motors have to be set");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.distance_threshold != 0.0)
    {
      distance_threshold = params_.distance_threshold;
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Distance threshold has to be set");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.norm_force_threshold != 0.0)
    {
      force_threshold = params_.norm_force_threshold;
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Force threshold has to be set");
      return controller_interface::CallbackReturn::ERROR;
    }
    // create the integrator publishers

    try
    {
      for (const auto &finger_motor : params_.finger_motors)
      {
        i_publishers_.push_back(get_node()->create_publisher<SingleCommandMsg>(
            finger_motor + "/integrator_input", rclcpp::SystemDefaultsQoS()));
        integrator_publishers_.push_back(std::make_unique<SingleCommandPublisher>(i_publishers_.back()));
        // Reserve memory in integrator publisher
        integrator_publishers_.back()->lock();
        integrator_publishers_.back()->msg_.dof_name = finger_motor;
        integrator_publishers_.back()->msg_.value = std::numeric_limits<double>::quiet_NaN();
        integrator_publishers_.back()->unlock();
      }
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    try
    {
      for (const auto &force_sensor : force_sensors_)
      {
        e_publishers_.push_back(get_node()->create_publisher<SingleCommandMsg>(
            force_sensor->get_name() + "/error", rclcpp::SystemDefaultsQoS()));
        error_publishers_.push_back(std::make_unique<SingleCommandPublisher>(e_publishers_.back()));
        // Reserve memory in integrator publisher
        error_publishers_.back()->lock();
        error_publishers_.back()->msg_.dof_name = force_sensor->get_name();
        error_publishers_.back()->msg_.value = std::numeric_limits<double>::quiet_NaN();
        error_publishers_.back()->unlock();
      }
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    // // create the client to active proportional controller for chained_mode
    // cm_srv_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
    // FOR DEMO
    cm_srv_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>("/rh8dl/controller_manager/switch_controller");

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Input Subscriber
    for (const auto &force_sensor : force_sensors_)
    {
      f_des_subs_.push_back(get_node()->create_subscription<SingleCommandMsg>(
          force_sensor->get_name() + "/f_des", subscribers_qos,
          std::bind(&GraspingController::f_des_sub_callback, this, std::placeholders::_1)));
    }

    RCLCPP_INFO(get_node()->get_logger(), "Graspign controller successfully configured");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void GraspingController::f_des_sub_callback(const std::shared_ptr<SingleCommandMsg> msg)
  {
    if (msg->dof_name.empty())
    {
      RCLCPP_WARN(get_node()->get_logger(), " Dof_name is empty, the message will be ignored");
      return;
    }
    auto found_key = f_des_sensors_map_.find(msg->dof_name);
    if (found_key != f_des_sensors_map_.end())
    {
      f_des_sensors_map_[msg->dof_name].second = msg->value;
    }
    return;
  }

  double GraspingController::compute_error(std::shared_ptr<semantic_components::ForceSensor> force_sensor)
  {
    double force_norm = force_sensor->get_norm();
    double desired_force = f_des_sensors_map_[force_sensor->get_name()].second;

    // std::string other_finger;
    // bool compute_average = (force_sensor->get_name() == "ring_force_sensor" ||
    //                         force_sensor->get_name() == "small_force_sensor");
    // if (compute_average)
    // {
    //   other_finger = (force_sensor->get_name() == "ring_force_sensor")
    //                      ? "small_force_sensor"
    //                      : "ring_force_sensor";
    //   auto it = std::find_if(force_sensors_.begin(), force_sensors_.end(),
    //                          [&other_finger](const std::shared_ptr<semantic_components::ForceSensor> &sensor)
    //                          {
    //                            return sensor->get_name() == other_finger;
    //                          });
    //   if (it != force_sensors_.end())
    //   {
    //     force_norm = force_sensor->get_norm() + (*it)->get_norm();
    //     force_norm = force_norm / 2;
    //   }
    //   else
    //   {
    //     force_norm = force_sensor->get_norm();
    //     std::cout << other_finger + " was not found!" << std::endl;
    //   }
    // }
    // else
    // {
    //   force_norm = force_sensor->get_norm();
    // }
    return desired_force - force_norm;
  }

  controller_interface::InterfaceConfiguration
  GraspingController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  GraspingController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &sensor : force_sensors_)
    {
      for (size_t i = 0; i < sensor->get_state_interface_names().size(); i++)
      {
        state_interfaces_config.names.push_back(sensor->get_state_interface_names()[i]);
      }
    }

    state_interfaces_config.names.push_back(tof_sensor_->get_state_interface_names()[0]);

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn GraspingController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (const auto &sensor : force_sensors_)
    {
      sensor->assign_loaned_state_interfaces(state_interfaces_);
    }

    tof_sensor_->assign_loaned_state_interfaces(state_interfaces_);

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    if (
        !controller_interface::get_ordered_interfaces(
            command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
        command_interface_types_.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
          command_interface_types_.size(), ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn GraspingController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (const auto &sensor : force_sensors_)
    {
      sensor->release_interfaces();
    }

    tof_sensor_->release_interfaces();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  void GraspingController::printState(GraspingState &last_state, GraspingState &current_state)
  {
    if (current_state != last_state)
    {
      switch (current_state)
      {
      case GraspingState::MEASURING_DISTANCE:
        std::cout << "stato corrente: MEASURING_DISTANCE " << std::endl;
        break;

      case GraspingState::FORCE_CONTROL:
        std::cout << "stato corrente: FORCE CONTROL " << std::endl;
        break;

      case GraspingState::SLIPPING_AVOIDANCE:
        std::cout << "stato corrente: SLIPPING AVOIDANCE " << std::endl;
        break;

      default:
        break;
      }
      last_state = current_state;
    }
  }

  void GraspingController::OpenCloseHand(bool mode, const rclcpp::Time &time)
  {
    // mode = 1 -> open
    // mode = 0 -> close
    if (mode)
    {
      for (const auto &integrator_publisher : integrator_publishers_)
      {
        if (integrator_publisher && integrator_publisher->trylock())
        {
          integrator_publisher->msg_.header.stamp = time;
          integrator_publisher->msg_.value = -params_.constant_velocity; // set negative velocity to open the hand
          integrator_publisher->unlockAndPublish();
        }
      }
    }
    else
    {
      for (const auto &integrator_publisher : integrator_publishers_)
      {
        if (integrator_publisher && integrator_publisher->trylock())
        {
          integrator_publisher->msg_.header.stamp = time;
          integrator_publisher->msg_.value = params_.constant_velocity;
          integrator_publisher->unlockAndPublish();
        }
      }
    }
  }

  void GraspingController::ActiveControllers(std::vector<std::string> &sensor_names, const std::string controller_name)
  {
    std::string controller;
    srv_request->activate_controllers.clear();

    for (const auto &sensor_name : sensor_names)
    {
      controller = sensor_name + "_" + controller_name;
      auto it = std::find(controllers_activated.begin(), controllers_activated.end(), controller);
      // if the controllers is not active, add it into the vector and activate it
      if (it == controllers_activated.end())
      {
        controllers_activated.push_back(controller);
        srv_request->activate_controllers.push_back(controller);
        std::cout << controller << " is activated" << std::endl;
      }
      else
        std::cout << controller << " is already active" << std::endl;
    }
    srv_request->deactivate_controllers.clear();
    cm_srv_client_->async_send_request(srv_request);
  }

  // overload method
  void GraspingController::ActiveControllers(const std::string &sensor_name, const std::string controller_name)
  {
    std::string controller = sensor_name + "_" + controller_name;
    srv_request->activate_controllers.clear();

    auto it = std::find(controllers_activated.begin(), controllers_activated.end(), controller);
    // if the controllers is not active, add it into the vector and activate it
    if (it == controllers_activated.end())
    {
      controllers_activated.push_back(controller);
      srv_request->activate_controllers.push_back(controller);
      srv_request->deactivate_controllers.clear();
      cm_srv_client_->async_send_request(srv_request);
      std::cout << controller << " is activated" << std::endl;
    }
    else
      std::cout << controller << " is already active" << std::endl;
  }

  void GraspingController::DeactiveControllers(std::vector<std::string> &sensor_names, const std::string controller_name)
  {
    std::string controller;
    srv_request->deactivate_controllers.clear();

    for (const auto &sensor_name : sensor_names)
    {
      controller = sensor_name + "_" + controller_name;
      auto it = std::find(controllers_activated.begin(), controllers_activated.end(), controller);
      // if the controller is active, deactivate it and remove it from the vector
      if (it != controllers_activated.end())
      {
        srv_request->deactivate_controllers.push_back(controller);
        controllers_activated.erase(it);
        std::cout << controller << " is deactivated" << std::endl;
      }
      else
        std::cout << controller << " is already deactivated" << std::endl;
    }
    srv_request->activate_controllers.clear();
    cm_srv_client_->async_send_request(srv_request);
  }

  controller_interface::return_type GraspingController::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {

    switch (current_state)
    {
    case GraspingState::MEASURING_DISTANCE:

      printState(last_state, current_state);

      if (tof_sensor_->get_distance() <= distance_threshold)
      {
        current_state = GraspingState::CLOSING;
        break;
      }

      break;

    case GraspingState::CLOSING:

      printState(last_state, current_state);

      OpenCloseHand(false, time);

      for (const auto &force_sensor : force_sensors_)
      {
        if ((force_sensor->get_norm()) >= force_threshold)
        {
          if (std::find(fingers_in_force.begin(), fingers_in_force.end(), force_sensor->get_name()) == fingers_in_force.end())
          {
            fingers_in_force.push_back(force_sensor->get_name());
            std::cout << "fingers_in_force size = " << fingers_in_force.size() << std::endl;
            ActiveControllers(force_sensor->get_name(), "proportional");
            /* fare attenzione se si vuole aggiungere small_force_sensor - usare prefix_proportional */
          }
        }

        // all proprortionals are activated
        if (fingers_in_force.size() == force_sensors_.size())
        {
          current_state = GraspingState::FORCE_CONTROL;
          break;
        }
      }

      // if object is removed before the force control
      if (tof_sensor_->get_distance() > distance_threshold)
      {
        if (!fingers_in_force.empty())
        {
          DeactiveControllers(fingers_in_force, "proportional");
        }

        if (time_request.seconds() == 0 && controllers_activated.size() == 0)
        {
          time_request = time;
        }

        if ((time - time_request) > rclcpp::Duration::from_seconds(2.0))
        {
          OpenCloseHand(true, time);
          fingers_in_force.clear();
          time_request = rclcpp::Time(0);
          current_state = GraspingState::MEASURING_DISTANCE;
          break;
        }
      }
      break;

    case GraspingState::FORCE_CONTROL:
    {
      printState(last_state, current_state);

      for (const auto &finger_in_force : fingers_in_force)
      {
        int index = f_des_sensors_map_[finger_in_force].first;
        error_publishers_[index]->lock();
        error_publishers_[index]->msg_.dof_name = finger_in_force;
        error_publishers_[index]->msg_.value = compute_error(force_sensors_[index]);
        error_publishers_[index]->unlockAndPublish();
      }

      // all proportionals converge
      bool converged = true;
      for (const auto &sensor : force_sensors_)
      {
        if (fabs(compute_error(sensor)) > 0.01)
        {
          converged = false;
          if (time_fingers_ok_.seconds() != 0)
          {
            std::cout << "************* NOT CONVERGED ***************" << std::endl;
          }
          time_fingers_ok_ = rclcpp::Time(0);
          break;
        }
      }

      if (converged)
      {
        if (time_fingers_ok_.seconds() == 0)
        {
          std::cout << "################ CONVERGED ######################" << std::endl;
          time_fingers_ok_ = time;
        }

        // after 5s from convergence
        if ((time - time_fingers_ok_) > rclcpp::Duration::from_seconds(3.0))
        {
          // active all slipping avoidance controllers
          ActiveControllers(fingers_in_force, "sa");
          time_fingers_ok_ = rclcpp::Time(0);
          current_state = GraspingState::SLIPPING_AVOIDANCE;
          break;
        }
      }

      // object removed
      if (tof_sensor_->get_distance() > distance_threshold)
      {
        if (!(controllers_activated.empty()))
          DeactiveControllers(fingers_in_force, "proportional");

        if (time_request.seconds() == 0 && controllers_activated.size() == 0)
          time_request = time;

        if ((time - time_request) > rclcpp::Duration::from_seconds(2.0))
        {
          OpenCloseHand(true, time);
          fingers_in_force.clear();
          time_request = rclcpp::Time(0);
          current_state = GraspingState::MEASURING_DISTANCE;
          break;
        }
      }
      break;
    }

    case GraspingState::SLIPPING_AVOIDANCE:
    {
      printState(last_state, current_state);

      // if object is removed during the slipping avoidance control
      if (tof_sensor_->get_distance() > distance_threshold)
      {

        // deactivate all slipping avoidance controllers
        if (!(controllers_activated.empty()))
        {
          DeactiveControllers(fingers_in_force, "sa");

          // cause of the control chain,  is possible to deactive the proportional controllers only after slipping avoidance ones
          DeactiveControllers(fingers_in_force, "proportional");
        }

        if (time_request.seconds() == 0 && controllers_activated.size() == 0)
          time_request = time;

        // opening
        if ((time - time_request) > rclcpp::Duration::from_seconds(2.0))
        {
          OpenCloseHand(true, time);
          fingers_in_force.clear();
          time_request = rclcpp::Time(0);
          current_state = GraspingState::MEASURING_DISTANCE;
          break;
        }
      }
      break;
    }

    default:
      break;

    } // end switch_case

    return controller_interface::return_type::OK;
  }

} // namespace grasping_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    grasping_controller::GraspingController,
    controller_interface::ControllerInterface)
