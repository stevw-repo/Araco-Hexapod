# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(araco_kinematics)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} src/leg_kinematics.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  ament_add_gtest(test_leg_kinematics test/test_leg_kinematics.cpp)
  target_link_libraries(test_leg_kinematics ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
