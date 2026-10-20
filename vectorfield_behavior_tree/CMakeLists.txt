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
project(vectorfield_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vectorfield_msgs REQUIRED)
# Build time only, for vf_link_ament_deps() and vf_set_warnings().
find_package(vectorfield_ros REQUIRED)

if(NOT COMMAND vf_link_ament_deps OR NOT COMMAND vf_set_warnings)
  message(FATAL_ERROR
    "find_package(vectorfield_ros) did not define vf_link_ament_deps() and vf_set_warnings()")
endif()

# The BT.CPP that nav2_behavior_tree uses, already found by its package configuration: v3 on
# Humble, v4 from Jazzy on. Both can be installed side by side, so bt_compat.hpp gets the
# version from here rather than from __has_include.
if(behaviortree_cpp_v3_FOUND)
  set(VF_BTCPP_PACKAGE behaviortree_cpp_v3)
  set(VF_BTCPP_VERSION 3)
elseif(behaviortree_cpp_FOUND)
  set(VF_BTCPP_PACKAGE behaviortree_cpp)
  set(VF_BTCPP_VERSION 4)
else()
  message(FATAL_ERROR "nav2_behavior_tree did not bring in behaviortree_cpp or behaviortree_cpp_v3")
endif()
message(STATUS "vectorfield_behavior_tree: BT.CPP v${VF_BTCPP_VERSION} (${VF_BTCPP_PACKAGE})")

set(VF_BT_PUBLIC_DEPS
  ${VF_BTCPP_PACKAGE}
  builtin_interfaces
  rclcpp
  rclcpp_action
  vectorfield_msgs
)

# The FollowCurve node, loaded by name (bt_navigator's plugin_lib_names, bt_runner).
set(VF_BT_PLUGIN vectorfield_follow_curve_action_bt_node)
add_library(${VF_BT_PLUGIN} SHARED src/follow_curve_action.cpp)
target_include_directories(${VF_BT_PLUGIN} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(${VF_BT_PLUGIN} PUBLIC cxx_std_17)
target_compile_definitions(${VF_BT_PLUGIN}
  PUBLIC VF_BTCPP_VERSION=${VF_BTCPP_VERSION}
  # BT_REGISTER_NODES exports BT_RegisterNodesFromPlugin only with this definition.
  PRIVATE BT_PLUGIN_EXPORT
)
# Lyrical exports a target per Nav2 BT plugin; only the core library is needed.
if(TARGET nav2_behavior_tree::nav2_behavior_tree)
  target_link_libraries(${VF_BT_PLUGIN} PUBLIC nav2_behavior_tree::nav2_behavior_tree)
else()
  vf_link_ament_deps(${VF_BT_PLUGIN} PUBLIC nav2_behavior_tree)
endif()
vf_link_ament_deps(${VF_BT_PLUGIN} PUBLIC ${VF_BT_PUBLIC_DEPS})
set_target_properties(${VF_BT_PLUGIN} PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${VF_BT_PLUGIN})

# The standalone executor (lib/vectorfield_behavior_tree/bt_runner). It loads the plugins at
# run time; the link to the plugin library only brings in its headers and dependencies.
add_executable(bt_runner src/bt_runner.cpp)
target_link_libraries(bt_runner PRIVATE ${VF_BT_PLUGIN})
vf_link_ament_deps(bt_runner PRIVATE geometry_msgs)
set_target_properties(bt_runner PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(bt_runner)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${VF_BT_PLUGIN}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS bt_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  file(GLOB VF_BT_PUBLIC_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vectorfield_behavior_tree/*.hpp")
  set(_vf_header_check_sources)
  foreach(_vf_header IN LISTS VF_BT_PUBLIC_HEADERS)
    get_filename_component(_vf_header_name "${_vf_header}" NAME_WE)
    set(_vf_source "${CMAKE_CURRENT_BINARY_DIR}/header_check/${_vf_header_name}.cpp")
    file(CONFIGURE OUTPUT "${_vf_source}" CONTENT
      "#include \"vectorfield_behavior_tree/${_vf_header_name}.hpp\"\n")
    list(APPEND _vf_header_check_sources "${_vf_source}")
  endforeach()
  add_library(vf_bt_header_check OBJECT ${_vf_header_check_sources})
  target_link_libraries(vf_bt_header_check PRIVATE ${VF_BT_PLUGIN})
  vf_set_warnings(vf_bt_header_check)

  # The node in a tree against a fake follow_curve server (unique ROS_DOMAIN_ID per process).
  # The plugin is loaded by path, through BT_REGISTER_NODES, as bt_navigator loads it.
  ament_add_ros_isolated_gtest(test_follow_curve_action test/test_follow_curve_action.cpp
    TIMEOUT 120)
  if(TARGET test_follow_curve_action)
    # ament_add_gtest uses the plain target_link_libraries signature.
    target_link_libraries(test_follow_curve_action ${VF_BT_PLUGIN})
    target_compile_definitions(test_follow_curve_action PRIVATE
      "VF_TEST_PLUGIN=\"$<TARGET_FILE:${VF_BT_PLUGIN}>\"")
    vf_set_warnings(test_follow_curve_action)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nav2_behavior_tree ${VF_BT_PUBLIC_DEPS})
ament_package()
