# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_supervision)

find_package(ament_cmake REQUIRED)
find_package(araco_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(command_policy src/command_policy.cpp)
target_compile_features(command_policy PUBLIC cxx_std_17)
target_include_directories(command_policy PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(source_arbiter src/source_arbiter.cpp)
target_compile_features(source_arbiter PUBLIC cxx_std_17)
target_include_directories(source_arbiter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(safety_machine src/safety_machine.cpp)
target_compile_features(safety_machine PUBLIC cxx_std_17)
target_include_directories(safety_machine PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

generate_parameter_library(
  command_arbiter_parameters
  parameters/command_arbiter.yaml
)

add_executable(command_arbiter_node src/command_arbiter_node.cpp)
target_link_libraries(
  command_arbiter_node command_arbiter_parameters command_policy source_arbiter)
ament_target_dependencies(
  command_arbiter_node
  araco_interfaces
  rclcpp
  rclcpp_lifecycle
)

add_executable(safety_supervisor_node src/safety_supervisor_node.cpp)
target_link_libraries(
  safety_supervisor_node safety_supervisor_parameters command_policy safety_machine)
ament_target_dependencies(
  safety_supervisor_node
  araco_interfaces
  control_msgs
  controller_manager_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rosgraph_msgs
  sensor_msgs
)

install(
  TARGETS command_arbiter_node safety_supervisor_node command_policy source_arbiter safety_machine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION "lib/${PROJECT_NAME}"
)
generate_parameter_library(
  safety_supervisor_parameters
  parameters/safety_supervisor.yaml
)

install(FILES LICENSE DESTINATION "share/${PROJECT_NAME}")

set(package_resource_directories
  config
  schema
  parameters
)

foreach(resource_directory IN LISTS package_resource_directories)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_directory}")
    install(
      DIRECTORY "${resource_directory}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/ DESTINATION include)
endif()

ament_export_dependencies(
  araco_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # SPDX and copyright policy is enforced by package metadata tests.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_package_metadata
    test/test_package_metadata.py
  )
  ament_add_gtest(test_command_policy test/test_command_policy.cpp)
  target_link_libraries(test_command_policy command_policy)
  ament_add_gtest(test_source_arbiter test/test_source_arbiter.cpp)
  target_link_libraries(test_source_arbiter source_arbiter)
  ament_add_gtest(test_safety_machine test/test_safety_machine.cpp)
  target_link_libraries(test_safety_machine safety_machine)
endif()

ament_package()
