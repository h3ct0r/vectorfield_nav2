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

# vectorfield_core builds three ways from this one file:
#   * colcon / ament_cmake: top level and ament_cmake found. Installs and exports
#     vectorfield_core::vectorfield_core.
#   * plain CMake or CMakePresets.json: top level, no ament. Builds and tests only; no install
#     rules.
#   * add_subdirectory() from vectorfield_py's wheel build (SKBUILD): static and PIC, with no
#     tests, no install rules and no ament calls, so nothing but the extension enters the wheel.

cmake_minimum_required(VERSION 3.22)

# package.xml is the single source of the version. Parse it before project() so the project
# version, the generated version.hpp and the ament metadata always agree.
file(READ "${CMAKE_CURRENT_LIST_DIR}/package.xml" _vf_package_xml)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _vf_version_tag
  "${_vf_package_xml}")
if(NOT _vf_version_tag)
  message(FATAL_ERROR "vectorfield_core: package.xml has no MAJOR.MINOR.PATCH version tag")
endif()
set(VF_PKG_VERSION "${CMAKE_MATCH_1}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/package.xml")

project(vectorfield_core VERSION ${VF_PKG_VERSION} LANGUAGES CXX)

if(PROJECT_IS_TOP_LEVEL AND NOT SKBUILD)
  # Present under colcon, absent in a plain CMake or preset build.
  find_package(ament_cmake QUIET)
endif()

if(NOT SKBUILD)
  # Plain ament_cmake leaves BUILD_SHARED_LIBS unset. Default to shared so the controller plugin
  # and the follower component can link the core; the wheel build forces a static core.
  option(BUILD_SHARED_LIBS "Build vectorfield_core as a shared library" ON)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(VF_WERROR "Add -Werror to the vectorfield_core targets (never to dependencies)" OFF)
option(VF_BUILD_BENCHMARKS "Build the google-benchmark targets (non-gating)" OFF)
option(VF_BUILD_FUZZERS "Build the libFuzzer targets (LLVM clang only)" OFF)

if(PROJECT_IS_TOP_LEVEL)
  include(CTest)

  # Preset build trees live inside the package (build/<preset>). Mark them so the ament linters
  # and colcon skip them when this checkout is later built as a colcon workspace.
  cmake_path(IS_PREFIX PROJECT_SOURCE_DIR "${PROJECT_BINARY_DIR}" NORMALIZE _vf_nested_build)
  if(_vf_nested_build AND NOT PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
    file(TOUCH "${PROJECT_BINARY_DIR}/AMENT_IGNORE" "${PROJECT_BINARY_DIR}/COLCON_IGNORE")
  endif()
endif()

# Warning flags and -Werror apply to our own targets only. Vendored muParser is compiled with -w;
# Eigen, GTest and benchmark headers arrive through imported targets, hence as SYSTEM includes.
function(vf_core_set_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual -Wformat=2)
    if(VF_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
endfunction()

# No version argument: Eigen 5's config-version file rejects a "3.4" request. src/version.cpp
# enforces the 3.4.0 floor with EIGEN_VERSION_AT_LEAST instead.
find_package(Eigen3 REQUIRED NO_MODULE)

set(VF_CORE_GENERATED_INCLUDE_DIR "${PROJECT_BINARY_DIR}/include")
configure_file(
  cmake/version.hpp.in
  "${VF_CORE_GENERATED_INCLUDE_DIR}/vectorfield_core/version.hpp"
  @ONLY
)

add_library(vectorfield_core
  src/version.cpp
)
add_library(vectorfield_core::vectorfield_core ALIAS vectorfield_core)
target_include_directories(vectorfield_core PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${VF_CORE_GENERATED_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(vectorfield_core PUBLIC Eigen3::Eigen)
target_compile_features(vectorfield_core PUBLIC cxx_std_17)
vf_core_set_warnings(vectorfield_core)

# Vendored muParser (plan section 3.2) is an OBJECT library folded into vectorfield_core through
# $<TARGET_OBJECTS>. That adds no link edge, so there is nothing extra to export, and hidden
# visibility keeps mu:: symbols from clashing with a system libmuparser in the same process.
# M1 vendors third_party/muparser and lists its six src/*.cpp files here.
set(VF_MUPARSER_SOURCES)
if(VF_MUPARSER_SOURCES)
  set(_vf_muparser_include "${PROJECT_SOURCE_DIR}/third_party/muparser/include")
  add_library(vf_muparser OBJECT ${VF_MUPARSER_SOURCES})
  target_include_directories(vf_muparser SYSTEM PRIVATE "${_vf_muparser_include}")
  target_compile_definitions(vf_muparser PRIVATE MUPARSER_STATIC)
  target_compile_features(vf_muparser PRIVATE cxx_std_17)
  target_compile_options(vf_muparser PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-w>)
  set_target_properties(vf_muparser PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
  )
  target_sources(vectorfield_core PRIVATE $<TARGET_OBJECTS:vf_muparser>)
  target_include_directories(vectorfield_core SYSTEM PRIVATE "${_vf_muparser_include}")
  target_compile_definitions(vectorfield_core PRIVATE MUPARSER_STATIC)
endif()

if(PROJECT_IS_TOP_LEVEL AND BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  # One gtest executable per call. Every TEST becomes a ctest entry with a JUnit file under
  # test_results/, where `colcon test-result` finds it.
  function(vf_core_add_gtest name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE vectorfield_core::vectorfield_core GTest::gtest_main)
    vf_core_set_warnings(${name})
    gtest_discover_tests(${name}
      XML_OUTPUT_DIR "${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}"
    )
  endfunction()

  vf_core_add_gtest(test_version test/test_version.cpp)
  target_compile_definitions(test_version PRIVATE
    "VF_TEST_PACKAGE_XML=\"${PROJECT_SOURCE_DIR}/package.xml\"")
endif()

if(PROJECT_IS_TOP_LEVEL AND VF_BUILD_BENCHMARKS)
  find_package(benchmark REQUIRED)

  function(vf_core_add_benchmark name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE
      vectorfield_core::vectorfield_core benchmark::benchmark_main)
    vf_core_set_warnings(${name})
  endfunction()

  # The core benchmarks of plan section 3.5 (field evaluation, EDT, Tracker::step, episode
  # start) arrive in M1. The MarkerBuilder benchmark belongs to vectorfield_ros (M2).
endif()

if(PROJECT_IS_TOP_LEVEL AND VF_BUILD_FUZZERS)
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR
      "VF_BUILD_FUZZERS needs LLVM clang; Apple clang and GCC ship no libFuzzer runtime")
  endif()
  # Instrument the library for coverage feedback. Combine with the asan preset to also get
  # AddressSanitizer and UndefinedBehaviorSanitizer inside the library.
  target_compile_options(vectorfield_core PRIVATE -fsanitize=fuzzer-no-link)
  if(TARGET vf_muparser)
    target_compile_options(vf_muparser PRIVATE -fsanitize=fuzzer-no-link)
  endif()

  function(vf_core_add_fuzzer name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE vectorfield_core::vectorfield_core)
    target_compile_options(${name} PRIVATE -fsanitize=fuzzer)
    target_link_options(${name} PRIVATE -fsanitize=fuzzer)
    vf_core_set_warnings(${name})
  endfunction()

  # The ExpressionCurve libFuzzer target and its seed corpus arrive in M1.
endif()

if(PROJECT_IS_TOP_LEVEL AND ament_cmake_FOUND AND NOT SKBUILD)
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
  install(
    FILES "${VF_CORE_GENERATED_INCLUDE_DIR}/vectorfield_core/version.hpp"
    DESTINATION include/${PROJECT_NAME}/vectorfield_core
  )
  install(
    TARGETS vectorfield_core
    EXPORT vectorfield_coreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(vectorfield_coreTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)

  if(BUILD_TESTING)
    find_package(ament_lint_auto QUIET)
    if(ament_lint_auto_FOUND)
      # C++ format and Python lint run in pre-commit (clang-format, ruff); cppcheck is replaced
      # by clang-tidy (plan section 9).
      list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_uncrustify
        ament_cmake_cpplint
        ament_cmake_flake8
        ament_cmake_pep257
        ament_cmake_cppcheck
      )
      file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE "${PROJECT_SOURCE_DIR}/third_party/*")
      ament_lint_auto_find_test_dependencies()
    endif()
  endif()

  ament_package()
endif()
