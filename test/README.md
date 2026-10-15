# shulib host tests

Off-robot unit tests for the **pure C++20 core** (`math/`, `kinematics/`, `control/`, … — the layers
that contain **zero `pros/` includes**). Running the logic on a laptop, in seconds, is what lets us
catch bugs before they ever reach the V5. This build is **completely separate** from the PROS ARM
Makefile at the repo root.

## Build & run

```sh
cmake -S test -B build/test        # configure (once, or after adding files)
cmake --build build/test           # compile
./build/test/shulib_tests          # run all tests
# or:
ctest --test-dir build/test --output-on-failure
```

`build/` is gitignored, so this never pollutes the repo. A non-zero exit code means a test failed.

**The compile-refusal harness** (since the mechanism core's first part): the mechanism core refuses a
robot file's mistakes AT BUILD, each by name. Every named refusal a robot file can reach has a
translation unit in [`compile_refusals/`](compile_refusals/) that must FAIL to compile, naming its
reason. Two names have none, on purpose: `a_smart_port_is_used_twice` and
`a_three_wire_port_is_used_twice`, the "used twice" fallbacks no list can reach (each said so beside it
in `mechanism/wiring.hpp`). And every PLACE in the headers that calls a refusal is reached by a TU, not
only every name (checked by deleting each call in turn and seeing a TU go red: 148 of the 150 direct
calls — the other two are those fallbacks — and all 11 calls made through a dispatcher, such as
`refuse(specRefusal(...))` or the route-id recorder). Where two layers could both refuse with the same
name — a `wire(...)` check and `mechanisms()`' own re-check of each entry — the TU also pins the line
GCC must cite (`// expect-at:`). They are not part of `shulib_tests` — `ctest` builds each one on its
own, one at a time even under `ctest -j` (they share a lock on the build tree), which takes two to three
minutes for all of them (127 s and 153 s, measured 2026-10-15 at 195 TUs), and
`ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure` runs just those.
The TU's own `// expect:` line states its reason; [`compile_refusals/run_refusal.py`](compile_refusals/run_refusal.py)
is the judge (the reason must be in a diagnostic's message, never only in a path), and four
`harness_*` TUs prove the judge cannot be fooled (a TU that compiles, the wrong reason, the reason only
in a file name, the wrong line). A TU dropped into that directory is a test at the next configure, and
CI fails when the number of harness tests is not the number of TUs (a detached harness would otherwise
report "No tests were found" and pass). The suite and the harness compile with the same flags
(`shulib_test_flags`, in `CMakeLists.txt`). The spec's per-field CONDITIONS (both sides of a range,
each field of a pair) are pinned at run time, one property per case, by `mechanism_spec_test.cpp`.

**The build needs `python3`.** Before compiling anything it runs the documentation gates
(`tools/api_doc_tool.py`): a public entity anywhere under `include/shulib/` with no
documentation comment fails the build *naming that entity, its file and its line*; the generated reference in
`docs/api/` must match a fresh generation; every ```` ```cpp ```` line in the public
documentation must appear verbatim in a compiled example test (or in a team file a chapter quotes
— robot two's driver file, `src/team/robot_two_driver.hpp`, each listing of which must also be an
unbroken run of the file); and no public document may link into the development-process notes.
The library itself is header-only C++ and depends on none of
this — only the test build does, and deliberately: a gate you can forget to run is not a gate.

## Conventions

- **One test file per unit:** every `*_test.cpp` in this directory is auto-discovered and compiled in.
- **`test_main.cpp`** provides the test runner's `main()` exactly once — leave it empty of tests.
- Include the unit under test from `include/shulib/...` and assert against it.

## Strict by design

The build uses `-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion
-Wdouble-promotion`. **Warnings are errors** — a narrowing conversion or an implicit double→float is a
bug we want the compiler to catch, not us. doctest is included as a *system* header so the framework's
internals never trip these flags; only our code is held to them.

## How we write tests

Tests exist to **find bugs, not confirm the obvious** (see the
[Testing discipline](../docs/roadmap.md) in the roadmap). Each test targets a specific way the
logic could be wrong (edge case, boundary, sign flip, wrap-around, NaN); we prefer **invariants/
properties** swept across the input space over hand-picked points; and for load-bearing code we run a
**mutation check** — deliberately break the implementation and confirm a test goes red. A suite that
stays green while the code is wrong is theater.

Trivial checks (`1+1==2`) are only ever *harness self-checks* — proof the runner itself works (and can
fail) — and are retired once real tests exist.

## Vendored dependency

[`vendor/doctest.h`](vendor/doctest.h) — **doctest v2.4.11**, a single-header test framework.
Source: <https://github.com/doctest/doctest> (pinned to release tag `v2.4.11`). License: **MIT**
(included at the top of the header). Chosen for zero install, fast compile, and clean support for the
strict/edge-case style above. Swapping it later is a single-file change.
