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

#ifndef SEMANTIC_COMPONENTS__FORCE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__FORCE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"
#include "math.h"

namespace semantic_components
{
class ForceSensor : public SemanticComponentInterface<geometry_msgs::msg::Vector3>
{
public:
  /// Constructor for 3D Force Sensor 
  explicit ForceSensor(const std::string & name) : SemanticComponentInterface(name, 3)
  {
    // If 6D FTS use standard names
    interface_names_.emplace_back(name_ + "/" + "force.x");
    interface_names_.emplace_back(name_ + "/" + "force.y");
    interface_names_.emplace_back(name_ + "/" + "force.z");

    // Set all interfaces existing
    std::fill(existing_axes_.begin(), existing_axes_.end(), true);

    // Set default force and torque values to NaN
    std::fill(forces_.begin(), forces_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  /// Constructor for 3D FTS with custom interface names.
  /**
   * Constructor for 3D FTS with custom interface names or FTS with less then six measurement axes,
   * e.g., 1D and 2D force load cells.
   * For non existing axes interface is empty string, i.e., ("");
   *
   * The name should be in the following order:
   *   force X, force Y, force Z.
   */
  ForceSensor(
    const std::string & interface_force_x, const std::string & interface_force_y,
    const std::string & interface_force_z)
  : SemanticComponentInterface("", 3)
  {
    auto check_and_add_interface = [this](const std::string & interface_name, const int index)
    {
      if (!interface_name.empty())
      {
        interface_names_.emplace_back(interface_name);
        existing_axes_[index] = true;
      }
      else
      {
        existing_axes_[index] = false;
      }
    };

    check_and_add_interface(interface_force_x, 0);
    check_and_add_interface(interface_force_y, 1);
    check_and_add_interface(interface_force_z, 2);

    // Set default force values to NaN
    std::fill(forces_.begin(), forces_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  virtual ~ForceSensor() = default;

  /// Return forces.
  /**
   * Return forces of a FTS. 
   * The reading is in mN so the values are converted into N
   *
   * \return array of size 3 with force values in N.
   */
  std::array<double, 3> get_forces()
  {
    
      for (size_t i = 0; i < 3; ++i)
    {
      if (existing_axes_[i])
      {
        // reading sensors state and convert values from mN to N
        forces_[i] = ( state_interfaces_[i].get().get_value() ) / 1000.0;
        
      }
    }

    return forces_;
  }

  /// Return FTS3Sensors message with forces and ids.
  /**
   * Constructs and return a message from the current values.
   * The method assumes that the interface names on the construction are in the following order:
   *   force X, force Y, force Z.
   *
   * \return FTS3Sensors message from values;
   */
  bool get_values_as_message(geometry_msgs::msg::Vector3 & message)
  {
    // call get_forces() to update with the latest values
    get_forces();

    // update the message values
    message.x = forces_[0];
    message.y = forces_[1];
    message.z = forces_[2];


    return true;
  }

  double get_norm(){

    get_forces();
    double sum = 0.0;
    for (const auto &force : forces_)
    {
      sum += pow(force,2);
    }
    return std::sqrt(sum);
  }

  std::string get_name(){
    return name_;
  }

protected:
  /// Vector with existing axes for sensors with less then 3D axes.
  // Order is: force X, force Y, force Z.
  std::array<bool, 3> existing_axes_;
  std::array<double, 3> forces_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__FORCE_SENSOR_HPP_
