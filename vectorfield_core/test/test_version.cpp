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

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include "vectorfield_core/vectorfield_core.hpp"

#ifndef VF_TEST_PACKAGE_XML
#error "VF_TEST_PACKAGE_XML must name the package.xml of vectorfield_core"
#endif

namespace
{

// Parses <version> from package.xml at run time, independently of the CMake regex that
// generated version.hpp, so a broken parser or a stale build tree fails here.
std::string packageXmlVersion()
{
  std::ifstream file(VF_TEST_PACKAGE_XML);
  if (!file) {
    return {};
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();

  static const std::regex kVersionTag(R"(<version>\s*([^<\s]+)\s*</version>)");
  std::smatch match;
  if (!std::regex_search(text, match, kVersionTag)) {
    return {};
  }
  return match[1].str();
}

}  // namespace

TEST(Version, MatchesPackageXml)
{
  const std::string expected = packageXmlVersion();
  ASSERT_FALSE(expected.empty()) << "cannot read <version> from " << VF_TEST_PACKAGE_XML;
  EXPECT_EQ(expected, vectorfield_core::version());
}

TEST(Version, MatchesGeneratedHeader)
{
  EXPECT_STREQ(VF_CORE_VERSION_STR, vectorfield_core::version());

  const std::string from_components = std::to_string(VF_CORE_VERSION_MAJOR) + "." +
                                      std::to_string(VF_CORE_VERSION_MINOR) + "." +
                                      std::to_string(VF_CORE_VERSION_PATCH);
  EXPECT_EQ(from_components, vectorfield_core::version());
}

TEST(Version, AtLeastComparesLexicographically)
{
  using vectorfield_core::versionAtLeast;
  constexpr int kMajor = VF_CORE_VERSION_MAJOR;
  constexpr int kMinor = VF_CORE_VERSION_MINOR;
  constexpr int kPatch = VF_CORE_VERSION_PATCH;

  static_assert(versionAtLeast(0, 0, 0), "every version is at least 0.0.0");
  static_assert(versionAtLeast(kMajor, kMinor, kPatch), "a version is at least itself");
  static_assert(!versionAtLeast(kMajor, kMinor, kPatch + 1), "patch is compared");
  static_assert(!versionAtLeast(kMajor, kMinor + 1, 0), "minor outranks patch");
  static_assert(!versionAtLeast(kMajor + 1, 0, 0), "major outranks minor");
  static_assert(versionAtLeast(kMajor - 1, kMinor + 1, kPatch + 1), "a lower major always passes");

  EXPECT_TRUE(versionAtLeast(kMajor, kMinor, kPatch));
  EXPECT_FALSE(versionAtLeast(kMajor, kMinor, kPatch + 1));
}
