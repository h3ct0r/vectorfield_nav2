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
project(vectorfield_mujoco)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Launch tests of every ground example (label `mujoco`), registered only when this is ON, as in
# vectorfield_system_tests; the per-PR smoke test is always registered.
option(VF_NIGHTLY_TESTS "MuJoCo example launch tests" OFF)

# The simulator core (registry, menagerie, scene, adapters, flight, sensors, overlay, recorder,
# gl and simulation: plain Python on mujoco and numpy), the mujoco_sim node (sim_node) and the
# examples' launch helpers (examples).
ament_python_install_package(${PROJECT_NAME})

# `ros2 run vectorfield_mujoco fetch_menagerie`: download the Menagerie robots ahead of time;
# `mujoco_sim`: the simulator node; `run_scenario`: drive an example's scenario.
foreach(_vf_program IN ITEMS fetch_menagerie mujoco_sim run_scenario)
  install(
    PROGRAMS scripts/${_vf_program}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${_vf_program}
  )
endforeach()

# The robot models (with their README.md licence notes), the launch files and the configs.
# tools/ holds dev-time converters and is not installed.
foreach(_vf_dir IN ITEMS models launch config)
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${_vf_dir}")
    install(DIRECTORY ${_vf_dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # colcon test puts only the dependencies' install spaces on the paths; the tests import the
  # package this package has just installed and read its installed models.
  foreach(_vf_test IN ITEMS
      test_adapters test_flight test_scene test_sensors test_overlay test_models
      test_menagerie test_menagerie_robots test_recorder test_simulation test_sim_node
      test_examples)
    ament_add_pytest_test(${_vf_test} test/${_vf_test}.py
      APPEND_ENV
        PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
        AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
      TIMEOUT 300
    )
  endforeach()

  # Launch tests (label `mujoco`), as vectorfield_system_tests registers its own: isolated (one
  # ROS_DOMAIN_ID each), serial, discovery restricted to this host, TIMEOUT scaled in coverage
  # builds, and on Fast DDS 3 (Lyrical and later) the profile that keeps builtin traffic off the
  # intra-process path (test/fastdds_no_builtin_intraprocess.xml).
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(VF_DISCOVERY_ENV "ROS_LOCALHOST_ONLY=1")
  else()
    set(VF_DISCOVERY_ENV "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()
  set(VF_TEST_TIME_SCALE 1)
  if(CMAKE_CXX_FLAGS MATCHES "--coverage")
    set(VF_TEST_TIME_SCALE 3)
  endif()
  set(VF_FASTDDS_ENV "")
  if(NOT "$ENV{ROS_DISTRO}" MATCHES "^(humble|jazzy)$")
    set(VF_FASTDDS_ENV
      "FASTDDS_DEFAULT_PROFILES_FILE=${CMAKE_CURRENT_SOURCE_DIR}/test/fastdds_no_builtin_intraprocess.xml")
  endif()
  # vf_mujoco_launch_test(<name> <example> [SCENARIO <name or path>] [TIMEOUT <s>]): the example
  # launch, headless, and run_scenario on SCENARIO (default: the example's own scenario).
  function(vf_mujoco_launch_test name example)
    cmake_parse_arguments(_vf "" "SCENARIO;TIMEOUT" "" ${ARGN})
    if(NOT _vf_SCENARIO)
      set(_vf_SCENARIO "${example}")
    endif()
    if(NOT _vf_TIMEOUT)
      set(_vf_TIMEOUT 600)
    endif()
    math(EXPR _vf_TIMEOUT "${_vf_TIMEOUT} * ${VF_TEST_TIME_SCALE}")
    add_launch_test(test/test_example_launch.py
      TARGET "${name}"
      RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
      TIMEOUT ${_vf_TIMEOUT}
      ENV "VF_MUJOCO_EXAMPLE=${example}" "VF_MUJOCO_SCENARIO=${_vf_SCENARIO}"
        "VF_MUJOCO_TIMEOUT=${_vf_TIMEOUT}" "VF_TEST_TIME_SCALE=${VF_TEST_TIME_SCALE}"
        ${VF_DISCOVERY_ENV} ${VF_FASTDDS_ENV}
      APPEND_ENV
        PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
        AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
    )
    set_tests_properties("${name}" PROPERTIES RUN_SERIAL TRUE LABELS "launch_test;mujoco")
  endfunction()

  # Per PR: the vendored Scout Mini (no download) on a short Nav2 route in the skid_steer world.
  vf_mujoco_launch_test(test_mujoco_smoke skid_steer
    SCENARIO "${CMAKE_CURRENT_SOURCE_DIR}/test/config/smoke.yaml" TIMEOUT 300)
  if(VF_NIGHTLY_TESTS)
    # Every ground example's own scenario; the Menagerie robots need the checkout (fetched on
    # first use, or VF_MENAGERIE_DIR).
    foreach(_vf_example differential skid_steer omnidirectional ackermann curve_following)
      vf_mujoco_launch_test(test_mujoco_${_vf_example} ${_vf_example} TIMEOUT 900)
    endforeach()
  endif()

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_pep257
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
