# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_system_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(araco_system_tests)

install(FILES LICENSE DESTINATION "share/${PROJECT_NAME}")

install(
  PROGRAMS
    scripts/araco_gate0_evidence
    scripts/araco_gate1_evidence
    scripts/araco_gate1_score
  DESTINATION "lib/${PROJECT_NAME}"
)

set(package_resource_directories
  config
  schema
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
    test_project_dependency_graph
    test/test_project_dependency_graph.py
  )
  ament_add_pytest_test(
    test_gate0_bundle
    test/test_gate0_bundle.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_gate1_scoring
    test/test_gate1_scoring.py
  )
  ament_add_pytest_test(
    test_gate1_preflight
    test/test_gate1_preflight.py
    TIMEOUT 60
  )
endif()

ament_package()
