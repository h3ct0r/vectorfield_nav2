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

# Two build paths:
#   * SKBUILD (pip install ./vectorfield_py, through scikit-build-core): compiles the sibling
#     ../vectorfield_core as a static PIC library, so the wheel is self-contained. Only in-tree
#     builds from a git checkout are supported (no sdist, no PyPI in 0.x).
#   * otherwise colcon / ament_cmake: links the installed vectorfield_core and installs the
#     packages and the extension into ${PYTHON_INSTALL_DIR}.

cmake_minimum_required(VERSION 3.22)
project(vectorfield_py LANGUAGES CXX)

option(VF_WERROR "Add -Werror to the vectorfield_py targets (never to dependencies)" OFF)

if(SKBUILD)
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../vectorfield_core"
    "${CMAKE_BINARY_DIR}/_core"
    EXCLUDE_FROM_ALL
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(vectorfield_core REQUIRED)
endif()

# Find Python with its development module before pybind11, so pybind11 uses FindPython mode on
# every supported version (2.9 to 3.x). Under ament, ament_cmake_python has already found
# Python3, but only the interpreter. scikit-build-core passes Python3_EXECUTABLE.
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_vectorfield MODULE src/bindings.cpp)
target_link_libraries(_vectorfield PRIVATE vectorfield_core::vectorfield_core)
set_target_properties(_vectorfield PROPERTIES CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Same set as vectorfield_core; pybind11 headers come in as SYSTEM includes.
  target_compile_options(_vectorfield PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual -Wformat=2)
  if(VF_WERROR)
    target_compile_options(_vectorfield PRIVATE -Werror)
  endif()
endif()

if(SKBUILD)
  # pyproject.toml packs python/vectorfield and python/vectorfield_tools (wheel.packages); the
  # compiled module joins the vectorfield package.
  install(TARGETS _vectorfield LIBRARY DESTINATION vectorfield)
  # M4: the CLIs are [project.scripts] entry points in pyproject.toml.
else()
  ament_python_install_package(vectorfield PACKAGE_DIR python/vectorfield)
  ament_python_install_package(vectorfield_tools PACKAGE_DIR python/vectorfield_tools)
  install(TARGETS _vectorfield LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/vectorfield")
  # M4: install(PROGRAMS scripts/vf-plot scripts/vf-sim scripts/vf-tune scripts/vf-curve
  #     DESTINATION lib/${PROJECT_NAME}), run as `ros2 run vectorfield_py vf-sim`.

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    # colcon test puts only the dependencies' install spaces on PYTHONPATH; the tests import the
    # packages this package has just installed.
    ament_add_pytest_test(test_import test/test_import.py
      APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
    )

    find_package(ament_lint_auto QUIET)
    if(ament_lint_auto_FOUND)
      # Python lint and C++ format run in pre-commit (ruff, clang-format); cppcheck is replaced
      # by clang-tidy (plan section 9).
      list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_uncrustify
        ament_cmake_cpplint
        ament_cmake_flake8
        ament_cmake_pep257
        ament_cmake_cppcheck
      )
      ament_lint_auto_find_test_dependencies()
    endif()
  endif()

  ament_package()
endif()
