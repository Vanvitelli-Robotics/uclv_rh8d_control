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

#include "integrator/integrator.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"

namespace   // utility
{  

  using IntegratorInputMsg = integrator::Integrator::IntegratorInputMsg;

  // called from RT control loop
  void reset_input_msg(
    const std::shared_ptr<IntegratorInputMsg> & msg, const std::string & input_name)
  {
    msg->dof_name = input_name;
    msg->value =  std::numeric_limits<double>::quiet_NaN();
  }

}  // namespace

namespace integrator
{
Integrator::Integrator() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn Integrator::on_init()
{

  try
  {
    param_listener_ = std::make_shared<integrator::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during integrator's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void Integrator::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn Integrator::configure_parameters()
{
  update_parameters();
  
  input_name_ = params_.command_interface_name;
  
  if (params_.command_interface_name.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Input name is not specified. It has be set in config file");
    return CallbackReturn::FAILURE;
  }

  //it is used one input - one output integrator .
  num_input_ = 1 ;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Integrator::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  input_name_.clear();
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Integrator::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command = 0.0;
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
  input_sub_ = get_node()->create_subscription<IntegratorInputMsg>(
    input_name_ + "/integrator_input", subscribers_qos,
    std::bind(&Integrator::input_sub_callback, this, std::placeholders::_1));

  std::shared_ptr<IntegratorInputMsg> input_msg = std::make_shared<IntegratorInputMsg>();
  reset_input_msg(input_msg, input_name_);
  input_.writeFromNonRT(input_msg);


  try
  {

    // State publisher
    s_publisher_ = get_node()->create_publisher<IntegratorStateMsg>(
      "~/integrator_output", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<IntegratorStatePublisher>(s_publisher_);
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
    state_publisher_->msg_.dof_name = input_name_;
  }
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), " Integrator successfully configured ");
  return controller_interface::CallbackReturn::SUCCESS;
}

void Integrator::input_sub_callback(const std::shared_ptr<IntegratorInputMsg> msg)
{
  if (msg->dof_name.empty() && !std::isnan(msg->value))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Reference massage does not have Input Name defined. "
      "Assuming that value as defined into config file");
    auto ref_msg = msg;
    ref_msg->dof_name = input_name_;
    input_.writeFromNonRT(ref_msg);
  }
  else if( msg->dof_name == input_name_ && !std::isnan(msg->value)){
    
    input_.writeFromNonRT(msg); 
    
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Msg Dof_name is not matching the expected input name.");
  }
}

controller_interface::InterfaceConfiguration Integrator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(num_input_);
  command_interfaces_config.names.push_back(input_name_ + "/" + params_.command_interface_type);
  
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration Integrator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(num_input_ * params_.input_state_interfaces.size());
  for (const auto & interface : params_.input_state_interfaces)
  {
    state_interfaces_config.names.push_back(input_name_ + "/" + interface);
    
  }
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> Integrator::on_export_reference_interfaces()
{
  reference_interfaces_.resize(
    num_input_ * params_.input_state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  size_t index = 0;
  for (const auto & interface : params_.input_state_interfaces)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), input_name_ + "/" + interface, &reference_interfaces_[index]));
    ++index;
  }

  return reference_interfaces;
}

bool Integrator::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn Integrator::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  reset_input_msg(*(input_.readFromRT()), input_name_);
  reference_interfaces_.assign(
    reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Integrator::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
   /* TO DO */
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Integrator::update_reference_from_subscribers()
{
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

controller_interface::return_type Integrator::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  update_parameters();

  for (size_t i = 0; i < num_input_ ; ++i)
  {
    input_integrator = reference_interfaces_[i];
    
    // compute the integrator output
    if(!std::isnan(input_integrator)) command += period.seconds() * input_integrator ;
    
    if( params_.antiwindup ){
      command = std::clamp(command, params_.i_min, params_.i_max);
    }
    // write calculated value
    command_interfaces_[i].set_value(command);
  }
  
    if (state_publisher_ && state_publisher_->trylock())
    {
      state_publisher_->msg_.header.stamp = time;
      for (size_t i = 0; i < num_input_ ; ++i)
      {
        state_publisher_->msg_.dof_name = input_name_;
        state_publisher_->msg_.time_step = period.seconds();
        state_publisher_->msg_.output = command_interfaces_[i].get_value();
      }
      state_publisher_->unlockAndPublish();
    }

  return controller_interface::return_type::OK;
}

}  // namespace integrator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  integrator::Integrator, controller_interface::ChainableControllerInterface)
