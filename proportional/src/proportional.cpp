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

#include "proportional/proportional.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace   // utility
{  

  using ProportionalInputMsg = proportional::Proportional::ProportionalInputMsg;

  // called from RT control loop
  void reset_input_msg(
    const std::shared_ptr<ProportionalInputMsg> & msg, const std::string & input_name)
  {
    msg->dof_name = input_name;
    msg->value =  std::numeric_limits<double>::quiet_NaN();
  }

}  // namespace

namespace proportional
{
Proportional::Proportional() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn Proportional::on_init()
{

  try
  {
    param_listener_ = std::make_shared<proportional::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during proportional's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void Proportional::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn Proportional::configure_parameters()
{
  update_parameters();
  
  command_interface_name_ = params_.command_interface_name;
  state_interface_name_ = params_.state_interface_name;

  if(params_.p_gain < 0.0 ){
     RCLCPP_ERROR( get_node()->get_logger(),
      "p_gain is negative. The gain has to be positive.");
      return CallbackReturn::ERROR;
  } 

  //it is used one input - one output proportional .
  num_input_ = 1 ;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Proportional::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interface_name_.clear();
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Proportional::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command = 0.0 ;
  auto ret = configure_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Input Subscriber
  input_sub_ = get_node()->create_subscription<ProportionalInputMsg>(
    state_interface_name_ + "/error", subscribers_qos,
    std::bind(&Proportional::input_sub_callback, this, std::placeholders::_1));

  std::shared_ptr<ProportionalInputMsg> input_msg = std::make_shared<ProportionalInputMsg>();
  reset_input_msg(input_msg, state_interface_name_);
  input_.writeFromNonRT(input_msg);

  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ProportionalStateMsg>(
       "~/output", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ProportionalStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher
  state_publisher_->lock();
  for (size_t i = 0; i < num_input_; ++i)
  {
    state_publisher_->msg_.dof_name = state_interface_name_;
  }
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), " Proportional successfully configured ");
  return controller_interface::CallbackReturn::SUCCESS;
}

void Proportional::input_sub_callback(const std::shared_ptr<ProportionalInputMsg> msg)
{
  
  if(  !std::isnan(msg->value)){
    input_.writeFromNonRT(msg); 
  } 

}

controller_interface::InterfaceConfiguration Proportional::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  
    if(params_.is_chained_mod){
      command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      command_interfaces_config.names.reserve(num_input_);
      command_interfaces_config.names.push_back(params_.next_controller_name + "/" + params_.command_interface_name + "/" + params_.command_interface_type);
    } else{
      command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    }
    
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration Proportional::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> Proportional::on_export_reference_interfaces()
{
  reference_interfaces_.resize( num_input_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  size_t index = 0;
  for (size_t i = 0; i< num_input_; i++)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(),
      params_.next_controller_name + "/" + params_.command_interface_name + "/" + params_.command_interface_type , &reference_interfaces_[index]));
    ++index;
  }

  return reference_interfaces;
}

bool Proportional::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn Proportional::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  reset_input_msg(*(input_.readFromRT()), state_interface_name_);
  reference_interfaces_.assign(
    reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Proportional::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interface_name_.clear();
  state_interface_name_.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Proportional::update_reference_from_subscribers()
{ 
  // check for any parameter updates
  update_parameters();

  //reading input from topic
  auto current_input = input_.readFromRT();

  for (size_t i = 0; i < num_input_; ++i)
  {
    if (!std::isnan((*current_input)->value))
    {
      reference_interfaces_[i] = (*current_input)->value;
      (*current_input)->value = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type Proportional::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  update_parameters();

  for (size_t i = 0; i < num_input_ ; ++i)
  {
    proportional_input_ = reference_interfaces_[i];
    
    //if p_gain is set negative during the execution
    if(params_.p_gain < 0.0) {
      RCLCPP_ERROR( get_node()->get_logger(),
      "p_gain is set negative. The gain has to be positive.");
      return controller_interface::return_type::ERROR ;
    }

    // compute the proportional output
    if(!std::isnan(proportional_input_)) command = params_.p_gain * proportional_input_ ;

    // write calculated value 
    command_interfaces_[i].set_value(command);
  }
  
  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    for (size_t i = 0; i < num_input_ ; ++i)
    {
      state_publisher_->msg_.dof_name = state_interface_name_;
      state_publisher_->msg_.time_step = period.seconds();
      state_publisher_->msg_.output = command_interfaces_[i].get_value();
    }
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace proportional

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  proportional::Proportional, controller_interface::ChainableControllerInterface)
