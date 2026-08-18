# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_perception)

find_package(ament_cmake REQUIRED)

install(FILES LICENSE DESTINATION "share/${PROJECT_NAME}")

foreach(resource_directory IN ITEMS config schema rviz)
  install(
    DIRECTORY "${resource_directory}"
    DESTINATION "share/${PROJECT_NAME}"
  )
endforeach()

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
  ament_add_pytest_test(
    test_sensor_contract
    test/test_sensor_contract.py
  )
endif()

ament_package()
