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
project(vectorfield_curve_follower)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vectorfield_core REQUIRED)
find_package(vectorfield_msgs REQUIRED)
find_package(vectorfield_ros REQUIRED)
# Lyrical and later only (REP 149 condition in package.xml).
find_package(nav2_ros_common QUIET)

if(NOT COMMAND vf_link_ament_deps OR NOT COMMAND vf_set_warnings)
  message(FATAL_ERROR
    "find_package(vectorfield_ros) did not define vf_link_ament_deps() and vf_set_warnings()")
endif()

# Every package whose targets appear in the exported link interface.
set(VF_FOLLOWER_PUBLIC_DEPS
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

# Follower parameters: an INTERFACE target with the generated
# header vectorfield_curve_follower/vectorfield_follower_params.hpp, included
# by the public follower_params.hpp and therefore exported.
generate_parameter_library(vectorfield_follower_params
  params/vectorfield_follower_params.yaml)

add_library(${PROJECT_NAME} SHARED
  src/curve_follower.cpp
  src/follower_params.cpp
  src/frame_twist_estimator.cpp
  src/goal_setup.cpp
  src/goal_supervisor.cpp
  src/lap_counter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  vectorfield_core::vectorfield_core
  vectorfield_ros::vectorfield_ros
  vectorfield_follower_params
)
vf_link_ament_deps(${PROJECT_NAME} PUBLIC ${VF_FOLLOWER_PUBLIC_DEPS})
vf_link_ament_deps(${PROJECT_NAME} PRIVATE diagnostic_updater rclcpp_components)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${PROJECT_NAME})

# The component and its stand-alone executable (lib/vectorfield_curve_follower/curve_follower).
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "vectorfield_curve_follower::CurveFollower"
  EXECUTABLE curve_follower)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} vectorfield_follower_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES params/vectorfield_follower_params.yaml
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

  # Self-containment of the public headers.
  file(GLOB VF_FOLLOWER_PUBLIC_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vectorfield_curve_follower/*.hpp")
  set(_vf_header_check_sources)
  foreach(_vf_header IN LISTS VF_FOLLOWER_PUBLIC_HEADERS)
    get_filename_component(_vf_header_name "${_vf_header}" NAME_WE)
    set(_vf_source "${CMAKE_CURRENT_BINARY_DIR}/header_check/${_vf_header_name}.cpp")
    file(CONFIGURE OUTPUT "${_vf_source}" CONTENT
      "#include \"vectorfield_curve_follower/${_vf_header_name}.hpp\"\n")
    list(APPEND _vf_header_check_sources "${_vf_source}")
  endforeach()
  add_library(vf_follower_header_check OBJECT ${_vf_header_check_sources})
  target_link_libraries(vf_follower_header_check PRIVATE ${PROJECT_NAME})
  vf_set_warnings(vf_follower_header_check)

  # Pure helpers: no ROS graph.
  function(vf_follower_add_unit_gtest name)
    ament_add_gtest(${name} test/${name}.cpp)
    if(TARGET ${name})
      target_link_libraries(${name} ${PROJECT_NAME})
      vf_set_warnings(${name})
    endif()
  endfunction()
  vf_follower_add_unit_gtest(test_frame_twist_estimator)
  vf_follower_add_unit_gtest(test_goal_supervisor)
  vf_follower_add_unit_gtest(test_lap_counter)

  # The node in process (unique ROS_DOMAIN_ID per test process). The timeout
  # leaves room for the CI coverage build (Debug, --coverage).
  ament_add_ros_isolated_gtest(test_follower_smoke test/test_follower_smoke.cpp TIMEOUT 300)
  if(TARGET test_follower_smoke)
    # ament_add_gtest uses the plain target_link_libraries signature.
    target_link_libraries(test_follower_smoke ${PROJECT_NAME} ${rclcpp_action_TARGETS})
    vf_set_warnings(test_follower_smoke)
  endif()

  # Layer 2b: the node in process on simulated time, driven in
  # lock step by test/follower_test_harness.hpp, one executable per theme.
  find_package(rosgraph_msgs REQUIRED)
  function(vf_follower_add_node_gtest name)
    ament_add_ros_isolated_gtest(${name} test/${name}.cpp TIMEOUT 600)
    if(TARGET ${name})
      target_link_libraries(${name}
        ${PROJECT_NAME} ${rclcpp_action_TARGETS} ${rosgraph_msgs_TARGETS})
      vf_set_warnings(${name})
    endif()
  endfunction()
  vf_follower_add_node_gtest(test_follower_params)
  vf_follower_add_node_gtest(test_follower_goal_checks)
  vf_follower_add_node_gtest(test_follower_supervision)
  vf_follower_add_node_gtest(test_follower_tracking)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(vectorfield_core vectorfield_ros ${VF_FOLLOWER_PUBLIC_DEPS})
ament_package()
