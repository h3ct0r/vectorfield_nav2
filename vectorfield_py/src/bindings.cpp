// Copyright 2026 Héctor Azpúrua
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Compiled extension module `vectorfield._vectorfield`.
//
// Only the pybind11 2.9 API subset is used (jammy ships 2.9.1; Homebrew and resolute ship 3.x).
// M0 exposes the version; the bindings listed in plan section 8 arrive in M4.

#include <pybind11/pybind11.h>

#include "vectorfield_core/vectorfield_core.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_vectorfield, m)
{
  m.doc() = "Compiled bindings of vectorfield_core. Import the vectorfield package instead.";
  m.attr("__version__") = py::str(vectorfield_core::version());
}
