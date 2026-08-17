# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

install(FILES LICENSE DESTINATION "share/${PROJECT_NAME}")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionIntent.msg"
  "msg/CommandCandidate.msg"
  "msg/SelectedCommand.msg"
  "msg/ArbitrationStatus.msg"
  "msg/SafeCommand.msg"
  "msg/JointStateProvenance.msg"
  "msg/LocomotionStatus.msg"
  "msg/SafetyStatus.msg"
  "action/SafetyTransition.action"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # SPDX and copyright policy is enforced by package metadata tests.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_generated_interfaces
    test/test_generated_interfaces.py
  )
  ament_add_pytest_test(
    test_package_metadata
    test/test_package_metadata.py
  )
endif()

ament_package()
