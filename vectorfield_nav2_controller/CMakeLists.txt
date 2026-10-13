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

if(NOT COMMAND vf_link_ament_deps)
  message(FATAL_ERROR "find_package(vectorfield_ros) did not define vf_link_ament_deps()")
endif()

# M0 resolves the dependencies only. The VectorFieldController library, its
# parameters and the pluginlib description file are planned for M2; the
# description is exported only together with the library it names.

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
  find_package(nav2_controller REQUIRED)
endif()

ament_package()
