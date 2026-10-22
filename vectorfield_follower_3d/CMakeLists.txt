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
project(vectorfield_follower_3d)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vectorfield_core REQUIRED)
find_package(vectorfield_curve_follower REQUIRED)
find_package(vectorfield_msgs REQUIRED)
find_package(vectorfield_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_ros_common QUIET)

if(NOT COMMAND vf_link_ament_deps OR NOT COMMAND vf_set_warnings)
  message(FATAL_ERROR
    "find_package(vectorfield_ros) did not define vf_link_ament_deps() and vf_set_warnings()")
endif()

set(VF_FOLLOWER_3D_PUBLIC_DEPS
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_ros
  vectorfield_msgs
  visualization_msgs
)

generate_parameter_library(vectorfield_follower_3d_params
  params/vectorfield_follower_3d_params.yaml)

add_library(${PROJECT_NAME} SHARED
  src/follower_3d.cpp
  src/goal_setup.cpp
  src/markers.cpp
  src/params.cpp
  src/voxelizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  vectorfield_core::vectorfield_core
  vectorfield_ros::vectorfield_ros
  vectorfield_curve_follower::vectorfield_curve_follower
  vectorfield_follower_3d_params
)
vf_link_ament_deps(${PROJECT_NAME} PUBLIC ${VF_FOLLOWER_3D_PUBLIC_DEPS})
vf_link_ament_deps(${PROJECT_NAME} PRIVATE rclcpp_components std_msgs)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "vectorfield_follower_3d::Follower3D"
  EXECUTABLE follower_3d)

# The kinematic point simulator of the tests and the demo.
add_executable(point_sim_3d src/point_sim_3d.cpp)
target_link_libraries(point_sim_3d PRIVATE vectorfield_core::vectorfield_core)
target_include_directories(point_sim_3d PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
vf_link_ament_deps(point_sim_3d PRIVATE
  geometry_msgs nav_msgs rclcpp rosgraph_msgs sensor_msgs tf2_msgs tf2_ros)
set_target_properties(point_sim_3d PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(point_sim_3d)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} vectorfield_follower_3d_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS point_sim_3d DESTINATION lib/${PROJECT_NAME})
install(
  FILES params/vectorfield_follower_3d_params.yaml
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
  file(GLOB VF_FOLLOWER_3D_PUBLIC_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vectorfield_follower_3d/*.hpp")
  set(_vf_header_check_sources)
  foreach(_vf_header IN LISTS VF_FOLLOWER_3D_PUBLIC_HEADERS)
    get_filename_component(_vf_header_name "${_vf_header}" NAME_WE)
    set(_vf_source "${CMAKE_CURRENT_BINARY_DIR}/header_check/${_vf_header_name}.cpp")
    file(CONFIGURE OUTPUT "${_vf_source}" CONTENT
      "#include \"vectorfield_follower_3d/${_vf_header_name}.hpp\"\n")
    list(APPEND _vf_header_check_sources "${_vf_source}")
  endforeach()
  add_library(vf_follower_3d_header_check OBJECT ${_vf_header_check_sources})
  target_link_libraries(vf_follower_3d_header_check PRIVATE ${PROJECT_NAME})
  vf_set_warnings(vf_follower_3d_header_check)

  # Pure helpers: no ROS graph.
  ament_add_gtest(test_follower_3d_helpers test/test_follower_3d_helpers.cpp)
  if(TARGET test_follower_3d_helpers)
    target_link_libraries(test_follower_3d_helpers ${PROJECT_NAME})
    vf_set_warnings(test_follower_3d_helpers)
  endif()

  # The node in process on simulated time, driven in lock step (test/follower_3d_harness.hpp).
  # The timeout leaves room for the CI coverage build (Debug, --coverage).
  find_package(action_msgs REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_ros_isolated_gtest(test_follower_3d_node test/test_follower_3d_node.cpp TIMEOUT 600)
  if(TARGET test_follower_3d_node)
    target_link_libraries(test_follower_3d_node
      ${PROJECT_NAME} ${rclcpp_action_TARGETS} ${rosgraph_msgs_TARGETS} ${tf2_msgs_TARGETS}
      ${action_msgs_TARGETS} ${lifecycle_msgs_TARGETS})
    vf_set_warnings(test_follower_3d_node)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  vectorfield_core vectorfield_ros vectorfield_curve_follower ${VF_FOLLOWER_3D_PUBLIC_DEPS})
ament_package()
