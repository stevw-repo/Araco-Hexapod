# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_locomotion)

find_package(ament_cmake REQUIRED)
find_package(araco_description REQUIRED)
find_package(araco_interfaces REQUIRED)
find_package(araco_kinematics REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)

generate_parameter_library(
  locomotion_parameters
  parameters/locomotion.yaml
)

add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(locomotion_node locomotion_parameters)
ament_target_dependencies(
  locomotion_node
  araco_interfaces
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
)

install(TARGETS locomotion_node DESTINATION "lib/${PROJECT_NAME}")

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
  araco_description
  araco_interfaces
  araco_kinematics
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # SPDX and copyright policy is enforced by package metadata tests.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_package_metadata
    test/test_package_metadata.py
  )
endif()

ament_package()
