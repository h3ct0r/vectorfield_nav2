# third_party

Vendored upstream code used by `vectorfield_core`. Nothing is vendored yet.

## muParser (planned for M1)

M1 vendors [muParser](https://github.com/beltoforion/muparser) (BSD-2-Clause) into
`third_party/muparser/` for `ExpressionCurve`, which evaluates curve expressions r(s, t).

- **What is copied:** `LICENSE`, `CHANGELOG`, `include/*.h` and the six library sources in
  `src/*.cpp`, plus a `VERSION` file that records the pinned upstream commit (one that includes
  the OSS-Fuzz fixes). Upstream build files, samples and tests are not copied.
- **Never edited.** An update replaces the files and the pinned commit together.
- **How it is built:** `CMakeLists.txt` lists the sources in `VF_MUPARSER_SOURCES`. They compile
  as the OBJECT library `vf_muparser` (PIC, hidden visibility, `MUPARSER_STATIC`, `-w`, SYSTEM
  include) and are folded into `vectorfield_core` through `$<TARGET_OBJECTS>`. muParser headers
  never appear in public headers.
- **Excluded from checks:** pre-commit, the ament linters (`AMENT_LINT_AUTO_FILE_EXCLUDE`),
  clang-tidy, warning flags and coverage all skip this directory.
- **Credited** in the repository `NOTICE` file.
