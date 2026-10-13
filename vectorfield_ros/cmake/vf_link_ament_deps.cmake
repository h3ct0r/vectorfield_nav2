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

#
# Link a target against ament packages, replacing ament_target_dependencies().
#
# ament_target_dependencies() is deprecated in Kilted and removed in Lyrical,
# and on Humble and Jazzy some packages (e.g. nav2_core) export include
# directories but no CMake target. For each package this function links the
# imported targets listed in <pkg>_TARGETS. If that list holds no actual
# target, it falls back to the classic ament variables: <pkg>_INCLUDE_DIRS
# (as SYSTEM includes), <pkg>_LIBRARIES and <pkg>_DEFINITIONS.
#
# :param target: an existing target
# :type target: string
# :param scope: PUBLIC, PRIVATE or INTERFACE
# :type scope: string
# :param ARGN: one or more package names, each already found with
#   find_package()
# :type ARGN: list of strings
#
# Example: vf_link_ament_deps(my_lib PUBLIC rclcpp nav2_core nav2_util)
#
# @public
#
include_guard(GLOBAL)

function(vf_link_ament_deps target scope)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR "vf_link_ament_deps(): '${target}' is not a target")
  endif()
  if(NOT scope MATCHES "^(PUBLIC|PRIVATE|INTERFACE)$")
    message(FATAL_ERROR
      "vf_link_ament_deps(): scope must be PUBLIC, PRIVATE or INTERFACE, not '${scope}'")
  endif()
  if(ARGC LESS 3)
    message(FATAL_ERROR "vf_link_ament_deps(): no packages given for '${target}'")
  endif()

  foreach(_vf_pkg IN LISTS ARGN)
    if(NOT ${_vf_pkg}_FOUND)
      message(FATAL_ERROR
        "vf_link_ament_deps(): package '${_vf_pkg}' was not found; "
        "call find_package(${_vf_pkg}) first")
    endif()

    # Like ament itself, only use entries of <pkg>_TARGETS that are targets.
    set(_vf_targets "")
    foreach(_vf_tgt IN LISTS ${_vf_pkg}_TARGETS)
      if(TARGET "${_vf_tgt}")
        list(APPEND _vf_targets "${_vf_tgt}")
      endif()
    endforeach()

    if(_vf_targets)
      target_link_libraries("${target}" ${scope} ${_vf_targets})
    else()
      if(${_vf_pkg}_INCLUDE_DIRS)
        target_include_directories("${target}" SYSTEM ${scope} ${${_vf_pkg}_INCLUDE_DIRS})
      endif()
      if(${_vf_pkg}_LIBRARIES)
        target_link_libraries("${target}" ${scope} ${${_vf_pkg}_LIBRARIES})
      endif()
      if(${_vf_pkg}_DEFINITIONS)
        target_compile_definitions("${target}" ${scope} ${${_vf_pkg}_DEFINITIONS})
      endif()
    endif()
  endforeach()
endfunction()
