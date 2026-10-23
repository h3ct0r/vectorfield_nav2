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

# The simulator core: registry, menagerie, scene, adapters, flight, sensors, overlay, recorder
# and gl, plain Python on mujoco and numpy.
ament_python_install_package(${PROJECT_NAME})

# `ros2 run vectorfield_mujoco fetch_menagerie`: download the Menagerie robots ahead of time.
install(
  PROGRAMS scripts/fetch_menagerie.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fetch_menagerie
)

# The robot models (with their README.md licence notes), and the launch files and configs
# once they exist. tools/ holds dev-time converters and is not installed.
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
      test_menagerie test_menagerie_robots test_recorder)
    ament_add_pytest_test(${_vf_test} test/${_vf_test}.py
      APPEND_ENV
        PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
        AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
      TIMEOUT 300
    )
  endforeach()

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
