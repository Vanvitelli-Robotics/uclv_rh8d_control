cmake_minimum_required(VERSION 3.16)
project(fingertip_sensors_broadcaster LANGUAGES CXX)

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
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(fingertip_sensors_broadcaster_parameters
  src/fingertip_sensors_broadcaster_parameters.yaml
)

add_library(fingertip_sensors_broadcaster SHARED
  src/fingertip_sensors_broadcaster.cpp
)
target_compile_features(fingertip_sensors_broadcaster PUBLIC cxx_std_17)
target_include_directories(fingertip_sensors_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/fingertip_sensors_broadcaster>
)
target_link_libraries(fingertip_sensors_broadcaster PUBLIC
  fingertip_sensors_broadcaster_parameters
)
ament_target_dependencies(fingertip_sensors_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fingertip_sensors_broadcaster PRIVATE "FINGERTIP_SENSORS_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface fingertip_sensors_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)

endif()

install(
  DIRECTORY include/
  DESTINATION include/fingertip_sensors_broadcaster
)
install(
  TARGETS
    fingertip_sensors_broadcaster
    fingertip_sensors_broadcaster_parameters
  EXPORT export_fingertip_sensors_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_fingertip_sensors_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
