cmake_minimum_required(VERSION 3.16)
project(grasping_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  hand_sensors_interfaces
  palm_sensor_broadcaster
  fingertip_sensors_broadcaster
  control_msgs
  controller_manager_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(grasping_controller_parameters
  src/grasping_controller_parameters.yaml
)

add_library(grasping_controller SHARED
  src/grasping_controller.cpp
)
target_compile_features(grasping_controller PUBLIC cxx_std_17)
target_include_directories(grasping_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/grasping_controller>
)
target_link_libraries(grasping_controller PUBLIC
  grasping_controller_parameters
)
ament_target_dependencies(grasping_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(grasping_controller PRIVATE "GRASPING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface grasping_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)

endif()

install(
  DIRECTORY include/
  DESTINATION include/grasping_controller
)
install(
  TARGETS
    grasping_controller
    grasping_controller_parameters
  EXPORT export_grasping_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_grasping_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
