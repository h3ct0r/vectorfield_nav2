# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(vectorfield_nav2_controller)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vectorfield_core REQUIRED)
find_package(vectorfield_msgs REQUIRED)
find_package(vectorfield_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
# Lyrical and later only (REP 149 condition in package.xml).
find_package(nav2_ros_common QUIET)

if(NOT COMMAND vf_link_ament_deps OR NOT COMMAND vf_set_warnings)
  message(FATAL_ERROR
    "find_package(vectorfield_ros) did not define vf_link_ament_deps() and vf_set_warnings()")
endif()

# Every package whose targets appear in the exported link interface.
set(VF_CONTROLLER_PUBLIC_DEPS
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

# Legacy path-handler parameters (Gen A/B only at run time; the header is
# generated on every distro). Linked privately: no public header includes it.
generate_parameter_library(vectorfield_legacy_path_params
  params/vectorfield_legacy_path_params.yaml)

add_library(${PROJECT_NAME} SHARED
  src/legacy_path_handler.cpp
  src/plan_processing.cpp
  src/vectorfield_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    vectorfield_core::vectorfield_core
    vectorfield_ros::vectorfield_ros
  PRIVATE
    vectorfield_legacy_path_params
)
vf_link_ament_deps(${PROJECT_NAME} PUBLIC ${VF_CONTROLLER_PUBLIC_DEPS})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${PROJECT_NAME})

# The description names the library above (plan section 5.3).
pluginlib_export_plugin_description_file(nav2_core vectorfield_nav2_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES params/vectorfield_legacy_path_params.yaml
  DESTINATION share/${PROJECT_NAME}/params
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_pep257
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  # Loaded at run time through pluginlib by the server harness: the goal
  # checkers and, on Gen C, FeasiblePathHandler; the rotation shim.
  find_package(nav2_controller REQUIRED)

  # Self-containment of the public headers.
  file(GLOB VF_CONTROLLER_PUBLIC_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vectorfield_nav2_controller/*.hpp")
  set(_vf_header_check_sources)
  foreach(_vf_header IN LISTS VF_CONTROLLER_PUBLIC_HEADERS)
    get_filename_component(_vf_header_name "${_vf_header}" NAME_WE)
    set(_vf_source "${CMAKE_CURRENT_BINARY_DIR}/header_check/${_vf_header_name}.cpp")
    file(CONFIGURE OUTPUT "${_vf_source}" CONTENT
      "#include \"vectorfield_nav2_controller/${_vf_header_name}.hpp\"\n")
    list(APPEND _vf_header_check_sources "${_vf_source}")
  endforeach()
  add_library(vf_controller_header_check OBJECT ${_vf_header_check_sources})
  target_link_libraries(vf_controller_header_check PRIVATE ${PROJECT_NAME})
  vf_set_warnings(vf_controller_header_check)

  # Test support (plan section 9, layer 2): the per-distro server harness and
  # the plan generators. Not installed.
  add_library(vf_controller_test_support STATIC
    test/support/plan_generators.cpp
    test/support/server_harness.cpp
  )
  target_include_directories(vf_controller_test_support PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/test/support")
  target_link_libraries(vf_controller_test_support PUBLIC ${PROJECT_NAME})
  vf_link_ament_deps(vf_controller_test_support PUBLIC rcl_interfaces)
  set_target_properties(vf_controller_test_support PROPERTIES
    CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON)
  vf_set_warnings(vf_controller_test_support)

  # Every test runs isolated (unique ROS_DOMAIN_ID per test process). Tests
  # that load plugins see this package's installed plugin description:
  # colcon installs a package before testing it. The timeout covers the CI coverage build
  # (Debug, --coverage), where the NavFn tail grid runs about 30x slower than in RelWithDebInfo.
  function(vf_controller_add_gtest name)
    ament_add_ros_isolated_gtest(${name} test/${name}.cpp
      TIMEOUT 600
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
    if(TARGET ${name})
      target_link_libraries(${name} vf_controller_test_support)
      vf_set_warnings(${name})
    endif()
  endfunction()

  vf_controller_add_gtest(test_plugin_discovery)
  # Pure unit test of the raw-plan intake (plan section 5.1).
  vf_controller_add_gtest(test_plan_processing)
  # Layer 2 through the server harness (plan section 9).
  vf_controller_add_gtest(test_controller_lifecycle)
  vf_controller_add_gtest(test_controller_approach)
  vf_controller_add_gtest(test_controller_obstacles_cancel)
  vf_controller_add_gtest(test_controller_replans)
  vf_controller_add_gtest(test_controller_navfn_tails)
  vf_controller_add_gtest(test_controller_footprint)
  vf_controller_add_gtest(test_controller_goal_checkers)
  # Its own process: Jazzy's shim library links PositionGoalChecker's.
  vf_controller_add_gtest(test_controller_rotation_shim)
  # LegacyPathHandler against FeasiblePathHandler (plan section 5.1); its
  # body is compiled on Gen C only.
  vf_controller_add_gtest(test_path_handler_parity)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(vectorfield_core vectorfield_ros ${VF_CONTROLLER_PUBLIC_DEPS})
ament_package()
