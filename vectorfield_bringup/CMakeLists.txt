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
project(vectorfield_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python helpers for the launch files: vectorfield_bringup.params.merge() and the shared demo
# pieces in vectorfield_bringup.demo; the field-test kit's field_metrics, slip, bag and
# field_report.
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME initial_pose_publisher
)
# The curve follower's example clients keep their names: `ros2 run vectorfield_bringup send_curve.py`.
install(
  PROGRAMS scripts/send_curve.py scripts/send_curve3d.py scripts/moving_frame_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
# The field-test kit (docs/tutorials/field_testing.md): `ros2 run vectorfield_bringup
# record_field_test.py`, `field_test_report.py <bag>` and `identify_skid_slip.py`.
install(
  PROGRAMS scripts/record_field_test.py scripts/field_test_report.py scripts/identify_skid_slip.py
  DESTINATION lib/${PROJECT_NAME}
)
# The tutorials' minimal clients: `ros2 run vectorfield_bringup follow_curve_client.py`.
install(
  PROGRAMS examples/follow_curve_client.py examples/navigate_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

# Humble's component_container_isolated never exits after a shutdown signal (rclcpp_components
# 16); the demos start this replacement there (src/isolated_container.cpp). Probed by version:
# a package build does not always set ROS_DISTRO.
find_package(rclcpp_components QUIET)
if(rclcpp_components_FOUND AND rclcpp_components_VERSION VERSION_LESS 17)
  find_package(rclcpp REQUIRED)
  add_executable(isolated_container src/isolated_container.cpp)
  target_link_libraries(isolated_container PRIVATE
    rclcpp::rclcpp rclcpp_components::component_manager)
  target_compile_features(isolated_container PRIVATE cxx_std_17)
  install(TARGETS isolated_container DESTINATION lib/${PROJECT_NAME})
endif()

# Install the resource directories that exist; their README.md files are not installed.
foreach(_vf_dir IN ITEMS launch config behavior_trees maps rviz foxglove plotjuggler)
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${_vf_dir}")
    install(
      DIRECTORY ${_vf_dir}
      DESTINATION share/${PROJECT_NAME}
      PATTERN "README.md" EXCLUDE
    )
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # colcon test puts only the dependencies' install spaces on PYTHONPATH; the tests import the
  # package this package has just installed and read the installed overlays.
  ament_add_pytest_test(test_params test/test_params.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
  )
  ament_add_pytest_test(test_overlays test/test_overlays.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
  )
  # The field-test metrics (field_metrics, slip, field_report) on synthetic data.
  ament_add_pytest_test(test_field_test test/test_field_test.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
  )

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
