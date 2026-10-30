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
project(vectorfield_sim)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vectorfield_core REQUIRED)
find_package(vectorfield_ros REQUIRED)

if(NOT COMMAND vf_link_ament_deps OR NOT COMMAND vf_set_warnings)
  message(FATAL_ERROR
    "find_package(vectorfield_ros) did not define vf_link_ament_deps() and vf_set_warnings()")
endif()

# The ROS-free model (plant, ray caster, obstacle parsing): links vectorfield_core only.
add_library(${PROJECT_NAME} SHARED
  src/obstacles.cpp
  src/plant.cpp
  src/ray_caster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC vectorfield_core::vectorfield_core)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${PROJECT_NAME})

# The node, as an rclcpp component and the kinematic_sim executable.
set(VF_SIM_NODE_PUBLIC_DEPS
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)
add_library(${PROJECT_NAME}_node SHARED src/kinematic_sim_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}
  vectorfield_ros::vectorfield_ros
)
vf_link_ament_deps(${PROJECT_NAME}_node PUBLIC ${VF_SIM_NODE_PUBLIC_DEPS})
vf_link_ament_deps(${PROJECT_NAME}_node PRIVATE rclcpp_components)
set_target_properties(${PROJECT_NAME}_node PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(${PROJECT_NAME}_node)
rclcpp_components_register_nodes(${PROJECT_NAME}_node "vectorfield_sim::KinematicSimNode")

add_executable(kinematic_sim src/kinematic_sim_main.cpp)
target_link_libraries(kinematic_sim PRIVATE ${PROJECT_NAME}_node)
set_target_properties(kinematic_sim PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(kinematic_sim)

# shutdown_last runs a simulator that is not ours so that it stops after its /clock readers.
add_executable(shutdown_last src/shutdown_last.cpp)
target_link_libraries(shutdown_last PRIVATE vectorfield_ros::vectorfield_ros)
vf_link_ament_deps(shutdown_last PRIVATE rclcpp)
set_target_properties(shutdown_last PROPERTIES CXX_EXTENSIONS OFF)
vf_set_warnings(shutdown_last)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS kinematic_sim shutdown_last
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
  find_package(tf2_msgs REQUIRED)

  # Self-containment of the public headers.
  file(GLOB VF_SIM_PUBLIC_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/vectorfield_sim/*.hpp")
  set(_vf_header_check_sources)
  foreach(_vf_header IN LISTS VF_SIM_PUBLIC_HEADERS)
    get_filename_component(_vf_header_name "${_vf_header}" NAME_WE)
    set(_vf_source "${CMAKE_CURRENT_BINARY_DIR}/header_check/${_vf_header_name}.cpp")
    file(CONFIGURE OUTPUT "${_vf_source}"
      CONTENT "#include \"vectorfield_sim/${_vf_header_name}.hpp\"\n")
    list(APPEND _vf_header_check_sources "${_vf_source}")
  endforeach()
  add_library(vf_sim_header_check OBJECT ${_vf_header_check_sources})
  target_link_libraries(vf_sim_header_check PRIVATE ${PROJECT_NAME}_node)
  vf_set_warnings(vf_sim_header_check)

  # Every test runs isolated. The timeout leaves room for the CI coverage build.
  function(vf_sim_add_gtest name)
    ament_add_ros_isolated_gtest(${name} test/${name}.cpp TIMEOUT 300)
    if(TARGET ${name})
      target_link_libraries(${name} ${ARGN})
      vf_set_warnings(${name})
    endif()
  endfunction()

  # The ROS-free model.
  vf_sim_add_gtest(test_obstacles ${PROJECT_NAME})
  vf_sim_add_gtest(test_plant ${PROJECT_NAME})
  vf_sim_add_gtest(test_ray_caster ${PROJECT_NAME})
  # The node, in process (motion and time; frames, laser and obstacles).
  vf_sim_add_gtest(test_sim_node_motion ${PROJECT_NAME}_node)
  vf_sim_add_gtest(test_sim_node_world ${PROJECT_NAME}_node)
  foreach(_vf_test test_sim_node_motion test_sim_node_world)
    if(TARGET ${_vf_test})
      target_include_directories(${_vf_test} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test")
      target_link_libraries(${_vf_test} ${tf2_msgs_TARGETS})
    endif()
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(vectorfield_core vectorfield_ros ${VF_SIM_NODE_PUBLIC_DEPS})
ament_package()
