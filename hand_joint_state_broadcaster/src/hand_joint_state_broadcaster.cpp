// Copyleft 2021 ros2_control development team
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

#include "hand_joint_state_broadcaster/hand_joint_state_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

#include "mapping.h"
#include "math.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace hand_joint_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

HandJointStateBroadcaster::HandJointStateBroadcaster() {}

controller_interface::CallbackReturn HandJointStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HandJointStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration HandJointStateBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces())
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints)
    {
      for (const auto & interface : params_.interfaces)
      {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn HandJointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. "
      "All available state interfaces will be published");
    params_.joints.clear();
    params_.interfaces.clear();
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing state interfaces defined in 'joints' and 'interfaces' parameters.");
  }

  auto get_map_interface_parameter =
    [&](std::string const & interface, std::string const & interface_to_map)
  {
    if (
      std::find(params_.interfaces.begin(), params_.interfaces.end(), interface) !=
      params_.interfaces.end())
    {
      map_interface_to_joint_state_[interface] = interface;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Mapping from '%s' to interface '%s' will not be done, because '%s' is defined "
        "in 'interface' parameter.",
        interface_to_map.c_str(), interface.c_str(), interface.c_str());
    }
    else
    {
      map_interface_to_joint_state_[interface_to_map] = interface;
    }
  };

  map_interface_to_joint_state_ = {};
  get_map_interface_parameter(HW_IF_POSITION, params_.map_interface_to_joint_state.position);
  get_map_interface_parameter(HW_IF_VELOCITY, params_.map_interface_to_joint_state.velocity);
  get_map_interface_parameter(HW_IF_EFFORT, params_.map_interface_to_joint_state.effort);

  try
  {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "/";

    joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      topic_name_prefix + "joint_states", rclcpp::SystemDefaultsQoS());

    realtime_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        joint_state_publisher_);

    dynamic_joint_state_publisher_ =
      get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
        topic_name_prefix + "dynamic_joint_states", rclcpp::SystemDefaultsQoS());

    realtime_dynamic_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(
        dynamic_joint_state_publisher_);
  }
  catch (const std::exception & e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HandJointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  init_joint_state_msg();
  init_dynamic_joint_state_msg();

  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.joints.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exists. "
      "Check ControllerManager output for more detailed information.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HandJointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  name_if_value_mapping_.clear();

  return CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  bool found_key = false;
  for (const auto & key_item : map)
  {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool HandJointStateBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty())
  {
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0)
    {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    // add interface name
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }


  /******* COMMENTO PERCHE' NON VOGLIO PUBBLICARE SU JOINT_STATES ANCHE I MOTORI ******/
  // filter state interfaces that have at least one of the joint_states fields,
  // the rest will be ignored for this message
  /** 
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}))
    {
      joint_names_.push_back(name_ifv.first);
    }
  }*/


  // Add extra joints from parameters, each joint will be added to joint_names_ and
  // name_if_value_mapping_ if it is not already there
  rclcpp::Parameter extra_joints;
  if (get_node()->get_parameter("hand_joints", extra_joints))                      //hand_joints in file yaml has the joints name that will be published on joint_states
  {
    const std::vector<std::string> & extra_joints_names = extra_joints.as_string_array();
    for (const auto & extra_joint_name : extra_joints_names)
    {
      if (hand_joints_map.count(extra_joint_name) == 0)
      {
        hand_joints_map[extra_joint_name] = {
          {HW_IF_POSITION, 0.0}, {HW_IF_VELOCITY, 0.0}, {HW_IF_EFFORT, 0.0}};
        joint_names_.push_back(extra_joint_name);
      }
    }
  }

  return true;
}

void HandJointStateBroadcaster::init_joint_state_msg()
{
  const size_t num_joints = joint_names_.size();

  /// @note joint_state_msg publishes position, velocity and effort for all joints,
  /// with at least one of these interfaces, the rest are omitted from this message

  // default initialization for joint state message
  auto & joint_state_msg = realtime_joint_state_publisher_->msg_;
  joint_state_msg.name = joint_names_;
  joint_state_msg.position.resize(num_joints, kUninitializedValue);
  joint_state_msg.velocity.resize(num_joints, kUninitializedValue);
  joint_state_msg.effort.resize(num_joints, kUninitializedValue);
}

void HandJointStateBroadcaster::init_dynamic_joint_state_msg()
{
  auto & dynamic_joint_state_msg = realtime_dynamic_joint_state_publisher_->msg_;
  dynamic_joint_state_msg.joint_names.clear();
  dynamic_joint_state_msg.interface_values.clear();
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & name = name_ifv.first;
    const auto & interfaces_and_values = name_ifv.second;
    dynamic_joint_state_msg.joint_names.push_back(name);
    control_msgs::msg::InterfaceValue if_value;
    for (const auto & interface_and_value : interfaces_and_values)
    {
      if_value.interface_names.emplace_back(interface_and_value.first);
      if_value.values.emplace_back(kUninitializedValue);
    }
    dynamic_joint_state_msg.interface_values.emplace_back(if_value);
  }
}

bool HandJointStateBroadcaster::use_all_available_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend())
  {
    return interface_and_value->second;
  }
  else
  {
    return kUninitializedValue;
  }
}

controller_interface::return_type HandJointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  /** After reading motors states interfaces, update the hand joints value w\ linear mapping */
  //wrist joints and thumb adduction
  for (const auto &wrist_motor : motors_wrist_joints)
  {
      const auto &motor_name = wrist_motor.first;
      const auto &joints = wrist_motor.second;
      for ( const auto &joint : joints)
      {
        const auto &joint_name = joint.first;
        const auto &wrist_joint = joint.second; 
       
        if(joint_name == "wrist_rotation_joint"){
          hand_joints_map[wrist_joint][HW_IF_POSITION]= 
            mapping(name_if_value_mapping_[motor_name][HW_IF_POSITION], 0.0, 4095, -M_PI_2, M_PI_2);
        } else if (joint_name == "wrist_adduction_joint" || joint_name == "wrist_flexion_joint"){
          hand_joints_map[wrist_joint][HW_IF_POSITION]= 
            mapping(name_if_value_mapping_[motor_name][HW_IF_POSITION], 0.0, 4095, -M_PI_4, M_PI_4);
        } else {
          hand_joints_map[wrist_joint][HW_IF_POSITION] = 
            mapping(name_if_value_mapping_[motor_name][HW_IF_POSITION], 0.0, 4095, 0.0, M_PI_2);
        }
      }
  }

  //fingers joints
  for (const auto &finger_motor : motors_fingers_joints)
  {
      const auto &motor_name = finger_motor.first;
      const auto &joints = finger_motor.second;
      for ( const auto &joint : joints)
      {
        const auto &joint_name = joint.first;
        const auto &finger_joint = joint.second; 
       
        if(joint_name == "mcp_joint" || joint_name == "mpc_joint_small"  ||  joint_name == "pip_joint_thumb"){
          hand_joints_map[finger_joint][HW_IF_POSITION] =  mapping(name_if_value_mapping_[motor_name][HW_IF_POSITION], 0.0, 2000, 0.0, M_PI_2);
        } else if (joint_name == "pip_joint" || joint_name == "pip_joint_small" || joint_name == "mpc_joint_thumb"){
          hand_joints_map[finger_joint][HW_IF_POSITION] =  mapping(name_if_value_mapping_[motor_name][HW_IF_POSITION], 2000, 3800, 0.0, M_PI_2);
        } else {
          hand_joints_map[finger_joint][HW_IF_POSITION] =  mapping(name_if_value_mapping_[motor_name][HW_IF_POSITION], 3800, 4095, 0.0, M_PI_4);
        }
      }
  }
  

  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  


  if (realtime_joint_state_publisher_ && realtime_joint_state_publisher_->trylock())
  {
    auto & joint_state_msg = realtime_joint_state_publisher_->msg_;

    joint_state_msg.header.stamp = time;

    // update joint state message and dynamic joint state message
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_state_msg.position[i] = 
       get_value(hand_joints_map, joint_names_[i], HW_IF_POSITION);  
      joint_state_msg.velocity[i] = 
       get_value(hand_joints_map, joint_names_[i], HW_IF_VELOCITY);    
      joint_state_msg.effort[i] = get_value(hand_joints_map, joint_names_[i], HW_IF_EFFORT);
    }
    realtime_joint_state_publisher_->unlockAndPublish();
  }

  if (realtime_dynamic_joint_state_publisher_ && realtime_dynamic_joint_state_publisher_->trylock())
  {
    auto & dynamic_joint_state_msg = realtime_dynamic_joint_state_publisher_->msg_;
    dynamic_joint_state_msg.header.stamp = time;
    for (size_t joint_index = 0; joint_index < dynamic_joint_state_msg.joint_names.size();
         ++joint_index)
    {
      const auto & name = dynamic_joint_state_msg.joint_names[joint_index];
      for (size_t interface_index = 0;
           interface_index <
           dynamic_joint_state_msg.interface_values[joint_index].interface_names.size();
           ++interface_index)
      {
        const auto & interface_name =
          dynamic_joint_state_msg.interface_values[joint_index].interface_names[interface_index];
        dynamic_joint_state_msg.interface_values[joint_index].values[interface_index] =
          name_if_value_mapping_[name][interface_name];
      }
    }
    realtime_dynamic_joint_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace hand_joint_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hand_joint_state_broadcaster::HandJointStateBroadcaster, controller_interface::ControllerInterface)
