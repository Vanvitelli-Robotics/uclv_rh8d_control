cmake_minimum_required(VERSION 3.16)
project(slipping_avoidance LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools 
  hand_sensors_interfaces
  geometry_msgs
  fingertip_sensors_broadcaster
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
#Eigen
find_package(Eigen3 REQUIRED NO_MODULE)


generate_parameter_library(slipping_avoidance_parameters
  src/slipping_avoidance_parameters.yaml
)

add_library(slipping_avoidance SHARED
  src/slipping_avoidance.cpp
)
target_compile_features(slipping_avoidance PUBLIC cxx_std_17)
target_include_directories(slipping_avoidance PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/slipping_avoidance>
)
target_link_libraries(slipping_avoidance PUBLIC
  slipping_avoidance_parameters
  Eigen3::Eigen
)
ament_target_dependencies(slipping_avoidance PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(slipping_avoidance PRIVATE "FINGERTIP_SENSORS_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface slipping_avoidance.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)

endif()

install(
  DIRECTORY include/
  DESTINATION include/slipping_avoidance
)
install(
  TARGETS
    slipping_avoidance
    slipping_avoidance_parameters
  EXPORT export_slipping_avoidance
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_slipping_avoidance HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
