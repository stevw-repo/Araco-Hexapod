# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(araco_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)

ament_python_install_package(araco_teleop)
generate_parameter_module(
  teleop_adapter_parameters
  parameters/teleop_adapter.yaml
)
generate_parameter_module(
  joystick_adapter_parameters
  parameters/joystick_adapter.yaml
)

install(FILES LICENSE DESTINATION "share/${PROJECT_NAME}")

install(
  PROGRAMS
    scripts/keyboard_teleop_ui
    scripts/joystick_adapter_node
    scripts/teleop_adapter_node
  DESTINATION "lib/${PROJECT_NAME}"
)

set(package_resource_directories
  config
  schema
  parameters
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
  araco_interfaces
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
    test_keyboard
    test/test_keyboard.py
  )
  ament_add_pytest_test(
    test_joystick
    test/test_joystick.py
  )
endif()

ament_package()
