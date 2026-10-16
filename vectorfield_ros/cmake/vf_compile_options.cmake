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
# Warning flags of the vectorfield ROS packages, identical to vectorfield_core's.
#
# Applies -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
# -Woverloaded-virtual -Wformat=2 and -ffp-contract=off (results must not
# depend on FMA contraction, see vectorfield_core/CMakeLists.txt) to one of
# our own targets, plus -Werror when the VF_WERROR option is ON. Dependencies
# arrive as imported targets or SYSTEM include directories, so their headers
# never trigger these warnings. Never call it on generated or vendored code.
#
# :param target: an existing target of this project
# :type target: string
#
# Example: vf_set_warnings(my_plugin)
#
# @public
#
include_guard(GLOBAL)

option(VF_WERROR
  "Add -Werror to the vectorfield targets (never to dependencies)" OFF)

function(vf_set_warnings target)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR "vf_set_warnings(): '${target}' is not a target")
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options("${target}" PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
      -Woverloaded-virtual -Wformat=2 -ffp-contract=off)
    if(VF_WERROR)
      target_compile_options("${target}" PRIVATE -Werror)
    endif()
  endif()
endfunction()
