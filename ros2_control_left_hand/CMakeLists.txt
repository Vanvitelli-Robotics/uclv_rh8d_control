cmake_minimum_required(VERSION 3.16)
project(ros2_control_left_hand LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  uclv_dynamixel_utils
  dynamixel_sdk
  hand_sensors_interfaces
  serial
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ros2_control_left_hand
  SHARED
  hardware/left_hand_system.cpp
  hardware/fingertip_sensors.cpp
)
target_compile_features(ros2_control_left_hand PUBLIC cxx_std_17)
target_include_directories(ros2_control_left_hand PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_left_hand>
)
ament_target_dependencies(
  ros2_control_left_hand PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
#target_link_libraries(ros2_control_left_hand PUBLIC dynamixel_sdk)
#target_link_libraries(ros2_control_left_hand PUBLIC uclv_dynamixel_utils)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_left_hand.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_left_hand
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/ros2_control_left_hand
)
install(
  DIRECTORY description
  DESTINATION share/ros2_control_left_hand
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_left_hand
)
install(TARGETS ros2_control_left_hand
  EXPORT export_ros2_control_left_hand
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_ros2_control_left_hand HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
