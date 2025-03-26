cmake_minimum_required(VERSION 3.16)
project(palm_sensor_broadcaster LANGUAGES CXX)

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

generate_parameter_library(palm_sensor_broadcaster_parameters
  src/palm_sensor_broadcaster_parameters.yaml
)

add_library(palm_sensor_broadcaster SHARED
  src/palm_sensor_broadcaster.cpp
)
target_compile_features(palm_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(palm_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/palm_sensor_broadcaster>
)
target_link_libraries(palm_sensor_broadcaster PUBLIC
  palm_sensor_broadcaster_parameters
)
ament_target_dependencies(palm_sensor_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(palm_sensor_broadcaster PRIVATE "PALM_SENSOR_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface palm_sensor_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)

endif()

install(
  DIRECTORY include/
  DESTINATION include/palm_sensor_broadcaster
)
install(
  TARGETS
    palm_sensor_broadcaster
    palm_sensor_broadcaster_parameters
  EXPORT export_palm_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_palm_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
