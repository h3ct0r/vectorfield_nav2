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
project(vectorfield_system_tests)

find_package(ament_cmake REQUIRED)

# Nightly-only tests (Gazebo TB3, labelled `gazebo`; noise/latency/slip
# injection, labelled `nightly`) are registered only when this is ON, so
# per-PR jobs, devcontainers and buildfarm devel jobs never see them.
option(VF_NIGHTLY_TESTS "Gazebo and noise/latency/slip tests" OFF)

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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(vectorfield_core REQUIRED)
  find_package(vectorfield_ros REQUIRED)

  # Restrict discovery to this host (plan section 9); never set ROS_DOMAIN_ID, which
  # run_test_isolated.py picks per test.
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(VF_DISCOVERY_ENV "ROS_LOCALHOST_ONLY=1")
  else()
    set(VF_DISCOVERY_ENV "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()

  # Shipped-configuration validation: validateFromNode() on every installed
  # controller overlay, and the merged overlays.
  # gtest links with the plain signature, so the dependencies come through an
  # interface library.
  add_library(vf_system_tests_deps INTERFACE)
  target_link_libraries(vf_system_tests_deps INTERFACE
    vectorfield_ros::vectorfield_ros)
  vf_link_ament_deps(vf_system_tests_deps INTERFACE
    ament_index_cpp nav2_costmap_2d rclcpp)
  ament_add_ros_isolated_gtest(test_shipped_configs
    test/test_shipped_configs.cpp
    TIMEOUT 120
    ENV ${VF_DISCOVERY_ENV}
  )
  if(TARGET test_shipped_configs)
    target_link_libraries(test_shipped_configs vf_system_tests_deps)
    vf_set_warnings(test_shipped_configs)
  endif()
  ament_add_ros_isolated_pytest_test(test_merged_configs
    test/test_merged_configs.py
    TIMEOUT 120
    ENV ${VF_DISCOVERY_ENV}
  )

  # Per-PR launch tests (plan section 9, layer 4): the default `launch_test`
  # label, isolated, serial and without retries. The motion model reaches the
  # test through VF_TEST_MOTION_MODEL; the shared helpers are imported from the
  # source tree. TIMEOUT (default 300 s) overrides the CTest timeout.
  # The CI coverage build (Debug, --coverage) has taken 270 s just to load Nav2's container:
  # scale the CTest timeouts and, through VF_TEST_TIME_SCALE, the harness's start-up wait.
  set(VF_TEST_TIME_SCALE 1)
  if(CMAKE_CXX_FLAGS MATCHES "--coverage")
    set(VF_TEST_TIME_SCALE 3)
  endif()
  function(vf_add_launch_test file model)
    cmake_parse_arguments(_vf "" "TIMEOUT" "" ${ARGN})
    if(NOT _vf_TIMEOUT)
      set(_vf_TIMEOUT 300)
    endif()
    math(EXPR _vf_TIMEOUT "${_vf_TIMEOUT} * ${VF_TEST_TIME_SCALE}")
    get_filename_component(_vf_name "${file}" NAME_WE)
    set(_vf_target "${_vf_name}_${model}")
    add_launch_test("${file}"
      TARGET "${_vf_target}"
      RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
      TIMEOUT ${_vf_TIMEOUT}
      ENV "VF_TEST_MOTION_MODEL=${model}" "VF_TEST_TIME_SCALE=${VF_TEST_TIME_SCALE}"
        ${VF_DISCOVERY_ENV}
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
    )
    set_tests_properties("${_vf_target}" PROPERTIES RUN_SERIAL TRUE)
  endfunction()

  foreach(_vf_model differential skid_steer omnidirectional ackermann)
    vf_add_launch_test(test/test_open_kinematic.py ${_vf_model})
    # Nav2's loopback simulator exists from Jazzy on. The coverage build (CI jazzy amd64) skips
    # it: under load, Jazzy's loopback simulator can fetch an empty map from a not yet active
    # map_server and crash at its first scan (README, Known issues); the kinematic runs cover the
    # same controller paths.
    if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble" AND NOT CMAKE_CXX_FLAGS MATCHES "--coverage")
      vf_add_launch_test(test/test_open_loopback.py ${_vf_model})
    endif()
  endforeach()
  # Not Ackermann: the corridor test turns around in a 2 m corridor, which a car
  # cannot; its contour and blocked cases run in test_open_kinematic.
  foreach(_vf_model differential skid_steer omnidirectional)
    vf_add_launch_test(test/test_corridor_kinematic.py ${_vf_model})
  endforeach()
  # The CBF obstacle mode (M7): not Ackermann, which rejects it.
  foreach(_vf_model differential omnidirectional)
    vf_add_launch_test(test/test_cbf_kinematic.py ${_vf_model})
  endforeach()
  # Reversing (M7): Smac Hybrid-A* with Reeds-Shepp and allow_reversing.
  foreach(_vf_model differential ackermann)
    vf_add_launch_test(test/test_reversing_kinematic.py ${_vf_model})
  endforeach()

  # The curve follower (M5) on curve_follower_demo, differential only: laps of closed curves,
  # moving frames, open curves and the Nav2 interlock. Minutes of simulated driving each.
  foreach(_vf_test test_curve_laps test_curve_moving test_curve_open)
    vf_add_launch_test(test/${_vf_test}.py differential TIMEOUT 600)
  endforeach()
  # The FollowCurve behaviour-tree node (M7) in bt_runner and in bt_navigator, through the real
  # follower, differential only.
  vf_add_launch_test(test/test_follow_curve_bt.py differential TIMEOUT 900)

  if(VF_NIGHTLY_TESTS)
    message(STATUS "VF_NIGHTLY_TESTS is ON, but no nightly tests exist yet")
  endif()
endif()

# The controller comparison (docs/controller_comparison.md): installed with the
# helpers it imports, never registered as a test.
install(PROGRAMS benchmarks/compare_controllers.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY vectorfield_system_tests
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py")

ament_package()
