# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_locomotion)

find_package(ament_cmake REQUIRED)
find_package(araco_description REQUIRED)
find_package(araco_interfaces REQUIRED)
find_package(araco_kinematics REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)

generate_parameter_library(
  locomotion_parameters
  parameters/locomotion.yaml
)

add_library(standing_target src/standing_target.cpp)
target_compile_features(standing_target PUBLIC cxx_std_17)
target_include_directories(standing_target PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(standing_target araco_kinematics::araco_kinematics)

add_library(tripod_gait src/tripod_gait.cpp)
target_compile_features(tripod_gait PUBLIC cxx_std_17)
target_include_directories(tripod_gait PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(safe_command_guard src/safe_command_guard.cpp)
target_compile_features(safe_command_guard PUBLIC cxx_std_17)
target_include_directories(safe_command_guard PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(tripod_gait araco_kinematics::araco_kinematics)

add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(
  locomotion_node locomotion_parameters safe_command_guard standing_target tripod_gait)
ament_target_dependencies(
  locomotion_node
  araco_interfaces
  geometry_msgs
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
)

install(
  TARGETS locomotion_node safe_command_guard standing_target tripod_gait
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION "lib/${PROJECT_NAME}"
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
  araco_description
  araco_interfaces
  araco_kinematics
  geometry_msgs
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
  ament_add_gtest(test_standing_target test/test_standing_target.cpp)
  target_link_libraries(test_standing_target standing_target tripod_gait)
  ament_add_gtest(test_tripod_gait test/test_tripod_gait.cpp)
  target_link_libraries(test_tripod_gait tripod_gait)
  ament_add_gtest(test_safe_command_guard test/test_safe_command_guard.cpp)
  target_link_libraries(test_safe_command_guard safe_command_guard)
endif()

ament_package()
