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

**The build needs `python3`.** Before compiling anything it runs the documentation gates
(`tools/api_doc_tool.py`): a public entity anywhere under `include/shulib/` with no
documentation comment fails the build *naming that entity, its file and its line*; the generated reference in
`docs/api/` must match a fresh generation; every ```` ```cpp ```` line in the public
documentation must appear verbatim in a compiled example test; and no public document may link
into the development-process notes. The library itself is header-only C++ and depends on none of
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
