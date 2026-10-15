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

// libFuzzer target for ExpressionCurve (plan §3.2, §9; upstream audit U01): untrusted expression
// strings reach muParser through makeCurve, which must reject them with CurveError or build a
// curve that evaluates without crashing. Any other exception, crash or sanitizer report is a bug.
//
// Input layout: x_expression, a newline, y_expression (no newline: y = "sin(s)"). An even input
// size builds a closed curve on [0, 2π], an odd size an open one on [0, 1]. Parameters `a` = 1.5
// and `b` = −0.5 are always declared, so assignments to constants are reachable.
//
// Build with LLVM clang: cmake -DVF_BUILD_FUZZERS=ON (combine with the asan preset), then run
//   ./fuzz_expression_curve -max_total_time=60 <copy of test/fuzz/corpus>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "vectorfield_core/config.hpp"
#include "vectorfield_core/curve.hpp"
#include "vectorfield_core/curve_spec.hpp"
#include "vectorfield_core/geometry.hpp"
#include "vectorfield_core/types.hpp"

namespace vf = vectorfield_core;

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  const std::string input(reinterpret_cast<const char *>(data), size);
  const std::size_t split = input.find('\n');

  vf::CurveSpec spec;
  spec.type = vf::CurveSpec::Type::kExpression;
  spec.x_expression = input.substr(0, split);
  spec.y_expression = split == std::string::npos ? std::string("sin(s)") : input.substr(split + 1);
  spec.closed = size % 2 == 0;
  spec.s_min = 0.0;
  spec.s_max = spec.closed ? vf::kTwoPi : 1.0;
  spec.parameter_names = {"a", "b"};
  spec.parameter_values = {1.5, -0.5};

  vf::CurveOptions options;
  options.global_search_samples = 64;  // keeps each execution fast

  try {
    const std::shared_ptr<const vf::Curve> curve = vf::makeCurve(spec, options);
    vf::EvalContext context;
    context.t = 0.25;
    vf::CurveHint hint;
    const vf::Vec2 queries[] = {
      vf::Vec2(0.0, 0.0), vf::Vec2(2.0, 1.0), vf::Vec2(-3.0, 0.5), vf::Vec2(1e6, -1e6)};
    for (const vf::Vec2 & x : queries) {
      const vf::ClosestPoint cp = curve->closestPoint(x, context, hint);
      hint = vf::makeHint(cp);
      static_cast<void>(curve->velocity(cp, context.t));
    }
    const double length = curve->length(context.t);
    static_cast<void>(curve->pointAt(0.3 * length, context.t));
    static_cast<void>(curve->tangentAt(0.7 * length, context.t));
  } catch (const vf::CurveError &) {
    // Rejected (INVALID_CURVE or UNSUPPORTED): the expected outcome for most inputs.
  }
  return 0;
}
