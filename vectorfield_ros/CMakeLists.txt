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
project(vectorfield_ros)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vectorfield_core REQUIRED)
find_package(vectorfield_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Lyrical and later only (REP 149 condition in package.xml).
find_package(nav2_ros_common QUIET)

# vf_link_ament_deps() and vf_set_warnings() are used by this package and
# exported to dependents through CONFIG_EXTRAS below.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/vf_link_ament_deps.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/vf_compile_options.cmake")

# Every package whose targets appear in the exported link interface of
# vectorfield_ros. Dependents find them through ament_export_dependencies().
set(VF_ROS_PUBLIC_DEPS
  builtin_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  nav_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vectorfield_msgs
  visualization_msgs
)
if(nav2_ros_common_FOUND)
  list(APPEND VF_ROS_PUBLIC_DEPS nav2_ros_common)
endif()

# Shared parameters: an INTERFACE target with the generated
# header vectorfield_ros/vectorfield_common_params.hpp, exported below.
generate_parameter_library(vectorfield_common_params
  params/vectorfield_common_params.yaml)

add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/core_params.cpp
  src/core_params_table.cpp
  src/field_state.cpp
  src/field_visualizer.cpp
  src/marker_builder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  vectorfield_common_params
  vectorfield_core::vectorfield_core
)
vf_link_ament_deps(${PROJECT_NAME} PUBLIC ${VF_ROS_PUBLIC_DEPS})
# compat.hpp probes nav2_ros_common with __has_include; the CMake result is
# exported too, and the API-generation test asserts that both agree
# (nav2_api_matrix.md section 7.4).
if(nav2_ros_common_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC VF_CMAKE_HAS_ROS_COMMON=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC VF_CMAKE_HAS_ROS_COMMON=0)
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} vectorfield_common_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES cmake/vf_compile_options.cmake cmake/vf_link_ament_deps.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  FILES params/vectorfield_common_params.yaml
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

  # Self-containment of the public headers: one translation unit per header,
  # each including only that header, compiled with our warnings.
  file(GLOB VF_ROS_PUBLIC_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vectorfield_ros/*.hpp")
  set(_vf_header_check_sources)
  foreach(_vf_header IN LISTS VF_ROS_PUBLIC_HEADERS)
    get_filename_component(_vf_header_name "${_vf_header}" NAME_WE)
    set(_vf_source "${CMAKE_CURRENT_BINARY_DIR}/header_check/${_vf_header_name}.cpp")
    file(CONFIGURE OUTPUT "${_vf_source}"
      CONTENT "#include \"vectorfield_ros/${_vf_header_name}.hpp\"\n")
    list(APPEND _vf_header_check_sources "${_vf_source}")
  endforeach()
  add_library(vf_ros_header_check OBJECT ${_vf_header_check_sources})
  target_link_libraries(vf_ros_header_check PRIVATE ${PROJECT_NAME})
  vf_set_warnings(vf_ros_header_check)

  # The API generation expected for the distro this package is configured
  # for (1 = A, 2 = B, 3 = C; 0 = unknown distro, checked for consistency
  # only), and the expected default of enable_stamped_cmd_vel (-1 unknown).
  set(_vf_distro "$ENV{ROS_DISTRO}")
  set(VF_TEST_EXPECTED_GENERATION 0)
  set(VF_TEST_EXPECTED_STAMPED -1)
  if(_vf_distro STREQUAL "humble")
    set(VF_TEST_EXPECTED_GENERATION 1)
    set(VF_TEST_EXPECTED_STAMPED 0)
  elseif(_vf_distro STREQUAL "jazzy")
    set(VF_TEST_EXPECTED_GENERATION 2)
    set(VF_TEST_EXPECTED_STAMPED 0)
  elseif(_vf_distro STREQUAL "kilted")
    set(VF_TEST_EXPECTED_GENERATION 2)
    set(VF_TEST_EXPECTED_STAMPED 1)
  elseif(_vf_distro MATCHES "^(lyrical|rolling)$")
    set(VF_TEST_EXPECTED_GENERATION 3)
    set(VF_TEST_EXPECTED_STAMPED 1)
  endif()

  # Every test that creates nodes runs isolated (unique ROS_DOMAIN_ID per
  # test process). ament_add_gtest links gtest with the plain
  # target_link_libraries() signature, so the calls below use it too.
  function(vf_ros_add_gtest name)
    ament_add_ros_isolated_gtest(${name} test/${name}.cpp TIMEOUT 120)
    if(TARGET ${name})
      target_link_libraries(${name} ${PROJECT_NAME})
      vf_set_warnings(${name})
    endif()
  endfunction()

  vf_ros_add_gtest(test_api_generation)
  if(TARGET test_api_generation)
    target_compile_definitions(test_api_generation PRIVATE
      "VF_TEST_EXPECTED_GENERATION=${VF_TEST_EXPECTED_GENERATION}"
      "VF_TEST_EXPECTED_STAMPED=${VF_TEST_EXPECTED_STAMPED}"
      "VF_TEST_ROS_DISTRO=\"${_vf_distro}\"")
  endif()
  vf_ros_add_gtest(test_conversions)
  vf_ros_add_gtest(test_core_params)
  vf_ros_add_gtest(test_field_state_constants)
  vf_ros_add_gtest(test_field_visualizer)
  vf_ros_add_gtest(test_marker_builder)
  vf_ros_add_gtest(test_param_table)
  vf_ros_add_gtest(test_single_goal_action_server)
  vf_ros_add_gtest(test_topics_legend)
endif()

# The MarkerBuilder benchmark (markers < 0.2 ms; the 12 x 12
# field grid reported separately). Non-gating; built only when requested and
# google benchmark is installed.
option(VF_BUILD_BENCHMARKS "Build the google-benchmark targets (non-gating)" OFF)
if(VF_BUILD_BENCHMARKS)
  find_package(benchmark QUIET)
  if(benchmark_FOUND)
    add_executable(bench_marker_builder bench/bench_marker_builder.cpp)
    target_include_directories(bench_marker_builder PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/test")
    target_link_libraries(bench_marker_builder PRIVATE
      ${PROJECT_NAME} benchmark::benchmark_main)
    vf_set_warnings(bench_marker_builder)
  else()
    message(STATUS "google benchmark not found: bench_marker_builder is not built")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(vectorfield_core ${VF_ROS_PUBLIC_DEPS})
ament_package(CONFIG_EXTRAS cmake/vectorfield_ros-extras.cmake)
