cmake_minimum_required(VERSION 3.16)
project(hand_joint_state_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(hand_joint_state_broadcaster_parameters
  src/hand_joint_state_broadcaster_parameters.yaml
)

add_library(hand_joint_state_broadcaster SHARED
  src/hand_joint_state_broadcaster.cpp
)
target_compile_features(hand_joint_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(hand_joint_state_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hand_joint_state_broadcaster>
)
target_link_libraries(hand_joint_state_broadcaster PUBLIC
  hand_joint_state_broadcaster_parameters
)
ament_target_dependencies(hand_joint_state_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hand_joint_state_broadcaster PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface joint_state_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  #ament_add_gmock(test_load_hand_joint_state_broadcaster
  #  test/test_load_hand_joint_state_broadcaster.cpp
  #)
  #target_link_libraries(test_load_hand_joint_state_broadcaster
  #  hand_joint_state_broadcaster
  #)
  #ament_target_dependencies(test_load_hand_joint_state_broadcaster
  #  controller_manager
  #  hardware_interface
  #  ros2_control_test_assets
  #)

  #ament_add_gmock(test_joint_state_broadcaster
  #  test/test_joint_state_broadcaster.cpp
  #)
  #target_link_libraries(test_joint_state_broadcaster
  #  hand_joint_state_broadcaster
  #)
  #ament_target_dependencies(test_joint_state_broadcaster
  #  hardware_interface
  #)
endif()

install(
  DIRECTORY include/
  DESTINATION include/hand_joint_state_broadcaster
)
install(
  TARGETS
    hand_joint_state_broadcaster
    hand_joint_state_broadcaster_parameters
  EXPORT export_hand_joint_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_hand_joint_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
