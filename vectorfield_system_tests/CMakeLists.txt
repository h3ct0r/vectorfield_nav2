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
  find_package(launch_testing_ament_cmake REQUIRED)

  # Per-PR launch tests and shipped-configuration validation are planned for
  # M3, the curve follower tests for M5. No tests are registered yet.
  if(VF_NIGHTLY_TESTS)
    message(STATUS "VF_NIGHTLY_TESTS is ON, but no nightly tests exist yet")
  endif()
endif()

ament_package()
