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

#ifndef SEMANTIC_COMPONENTS__TOF_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__TOF_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

#include "hand_sensors_interfaces/msg/palm_sensor.hpp"

namespace semantic_components
{
class TOFSensor : public SemanticComponentInterface<hand_sensors_interfaces::msg::PalmSensor>
{
public:
  /// Constructor for 3D Force Sensor 
  explicit TOFSensor(const std::string & name) : SemanticComponentInterface(name, 1)
  {
    
    interface_names_.emplace_back(name_ + "/" + "distance");

  }

  /* costruttore con interfacce custom - to do */

  virtual ~TOFSensor() = default;

  /// Return distance.
  /**
   * Return distance measured from palm sensor.
   *
   * \return double with distance value.
   */
  double get_distance()
  {

    return distance = state_interfaces_[0].get().get_value();
  }

  /// Return PalmSensor message with forces and ids.
  /**
   * 
   *
   * \return PalmSensor message from values;
   */
  bool get_values_as_message(hand_sensors_interfaces::msg::PalmSensor & message)
  {
    // call get_distance() to update with the latest values
    get_distance();

    // update the message values
    message.distance = distance;


    return true;
  }

protected:
  /// Distance measured 
  double distance; 
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__TOF_SENSOR_HPP_
