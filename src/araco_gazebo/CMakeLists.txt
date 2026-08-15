# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(araco_description REQUIRED)

ament_python_install_package(araco_gazebo)

install(FILES LICENSE DESTINATION "share/${PROJECT_NAME}")

install(
  PROGRAMS scripts/contact_filter_node
  DESTINATION "lib/${PROJECT_NAME}"
)

set(package_resource_directories
  config
  schema
  worlds
  launch
)

foreach(resource_directory IN LISTS package_resource_directories)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_directory}")
    install(
      DIRECTORY "${resource_directory}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

ament_export_dependencies(
  araco_description
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
  ament_add_pytest_test(
    test_world_rendering
    test/test_world_rendering.py
  )
endif()

ament_package()
