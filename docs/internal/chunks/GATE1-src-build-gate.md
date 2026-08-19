# GATE1 — put `src/` under a gate

> **A reactive chunk, out of build order**, in the DOCS1 / DOCS2 / DEFECTS1 family: it exists
> because R3b piece 3 had to edit `src/main.cpp` and nothing in CI could tell whether the edit
> compiled. Small, self-contained, and it closes a blind spot that has already cost this project
> a shipped defect.

---

## 1. Why this chunk is here

`.github/workflows/ci.yml` cross-compiles **every header** under `include/shulib/` for the V5 and
holds that line hard. It never compiles **`src/`** — not `main.cpp`, not `bench_r3a.cpp`. The host
build does not either: `test/CMakeLists.txt` globs `test/*_test.cpp`, and `src/` is not in it.

So the only thing standing between a broken `src/` and the robot is somebody running `make` by
hand. The `Makefile`'s own comment records what that cost:

> *"src/ compiled with NO warning flags until 2026-08-18 … `src/main.cpp` and `src/bench_r3a.cpp`
> got nothing, **and a data abort shipped to the robot as a result.**"*

That fix enabled `-Wall -Wextra -Wformat=2` in the **local** `Makefile`. It did not put `src/` in
CI, so the warnings only exist for whoever types `make`. **This chunk puts the build itself in CI.**

**MEASURED 2026-08-19, and it changes what is possible here:** `make` now **succeeds end to end** —
compile *and* link — exit 0, producing `bin/hot.package.bin`, at `arm-none-eabi-g++ 13.2.1`.

> ⚠ **This retires a standing belief.** The on-robot build was recorded as blocked by a
> soft-float `firmware/`+`liblvgl.a` ABI mismatch and a `gnu++26` default. Both are gone: the
> `Makefile` pins `CXX_STANDARD:=gnu++20` (and `C_STANDARD:=gnu2x`) ahead of `common.mk`'s `?=`,
> and the link resolves. **The gate can therefore be the real build, not a compile-only proxy.**
> Do not re-derive the old blocker from memory; re-measure if you doubt it.

---

## 2. What already exists to build on

| Thing | Where | Why it matters here |
|---|---|---|
| `arm-compile-gate` job | `.github/workflows/ci.yml` | **already installs `gcc-arm-none-eabi`** — the toolchain this needs, no new install step |
| Generated header list | same job | the precedent: *"a gate you must remember to update when adding a file is a gate that rots"* — enumerate nothing |
| `WARNFLAGS` + its rationale | `Makefile:17-30` | `-Wall -Wextra -Wformat=2`, deliberately **without** `-Werror`, with the reason written out |
| Vendored toolchain inputs | `firmware/*.a`, `include/pros/`, `include/liblvgl/` | all in-tree, so CI needs nothing beyond the compiler |
| `tools/*.py` + `self-test` | `api_doc_tool.py`, `doc_staleness_audit.py` | the convention: a gate ships a self-test that **proves its detectors can fire** |
| `briefing_status.py` binary handling | `tools/briefing_status.py:112-124, 482-506` | suite counts are `—` and the suite line is **excluded from comparison** when no binary exists — which is why CI's gates-before-build ordering is safe. Do not "fix" it. |

---

## 3. Scope

**In:**
1. **Fix the variant-selection defect** (§4.1) — it is a prerequisite: the gate must build both
   variants, and today there is no correct way to ask for one of them.
2. **A gate that builds `src/` for the V5, both variants, in CI**, with a warning policy.
3. **A self-test that proves the gate fires.**

**Out:**
- Uploading to or running on a brain. This gate proves the package **builds**, never that it works.
  Chunks R3a/R3b own the robot.
- Changing the library-header ARM gate. It works; leave it.
- Adding `src/` to the **host** build. It is PROS-coupled by design; that is what the adapters and
  the PROS-free guard exist to manage.

**Explicitly rejected:**
- **`-Werror` on `src/`.** The three `-Wunused-function` warnings in the bench build are *correct,
  informative output* — the `Makefile` says so in writing: they are how you can see that the
  invented X-drive wiring really is dead code. `-Werror` would force deleting or `#ifdef`-ing that
  wiring to satisfy a compiler, which is worse code for a gate's convenience.
- **Suppressing the vendor warning globally** (`-Wno-deprecated-enum-enum-conversion`). It comes
  from `include/liblvgl/core/lv_obj_style.h`, which is vendored third-party code. Silence it by
  **path**, never by category — killing the category would also silence it in *our* code.

---

## 4. The design

### 4.1 THE PREREQUISITE DEFECT — there is no correct way to build the X-drive variant

`src/main.cpp:64` documents:

```
// Build the X-drive path with:  make CXXFLAGS_EXTRA=-DSHULIB_ROBOT_XDRIVE_INVENTED
```

`common.mk:279` consumes **`EXTRA_CXXFLAGS`**. The documented name is transposed, so the flag sets
an unused make variable and is a **silent no-op**.

**Both halves were measured, not reasoned** (detector: the bench variant emits 3
`-Wunused-function` from `src/main.cpp`, the X-drive variant emits 0):

| Invocation | Result |
|---|---|
| `make CXXFLAGS_EXTRA=-DSHULIB_ROBOT_XDRIVE_INVENTED` (as documented) | **3 warnings — it built the BENCH variant.** The flag did nothing, and nothing said so. |
| `make EXTRA_CXXFLAGS=-DSHULIB_ROBOT_XDRIVE_INVENTED` (correct name) | 0 warnings — X-drive really built — **but the build hash is GONE** (`strings` finds no `v0.1.1-271`) |

The second failure is make semantics, not a typo: a **command-line variable overrides file
assignments**, so `EXTRA_CXXFLAGS=…` wipes the `Makefile`'s `EXTRA_CXXFLAGS+=-DSHULIB_BUILD_HASH=…`.
The `Makefile` calls that hash load-bearing, and `build_info.hpp` has a LOUD-missing path — so the
result is honest at runtime rather than silently wrong, but **you still cannot build the X-drive
variant with an identity today.**

**RULING: add a named variant switch to the `Makefile`** — a dedicated variable (e.g. `ROBOT ?=
bench`, with `ROBOT=xdrive` appending the define) so the selection is a *make variable of its own*
and never collides with the flags carrying the hash. Then **fix `main.cpp:64` to the working
invocation.** Verify by measurement, with the warning-count detector above and by confirming the
hash survives (`strings bin/*.elf`). *Naming is yours; record the alternative you rejected.*

### 4.2 What the gate builds

**Both variants, every file in `src/`, for the V5.** Prefer driving the **real `make`** over
re-implementing its flags: a hand-rolled command line is a second source of truth that will drift
from `common.mk`, and drift in a gate is how a gate stops meaning anything. If you re-implement
instead, justify it in writing.

**FORCE A REBUILD.** `make` is incremental, and a gate that no-ops on stale objects reports green
without compiling anything — the same stale-artifact class of bug that nearly faked a mutation
result in R3b. Make the gate's freshness structural (clean output dir, or a `touch`), and make it
**observable**: print how many TUs were compiled, so a silent no-op is visible in the CI log.

### 4.3 The warning policy — the one real design decision

Fail on **any error**, in either variant. For warnings, the rule must distinguish three things the
compiler does not:

| Class | Example measured today | Policy |
|---|---|---|
| **Ours, expected** | 3 × `-Wunused-function` from `src/main.cpp` (bench variant) | **allowed**, by CATEGORY, documented |
| **Ours, new** | anything else originating in `src/` | **FAIL** |
| **Vendor** | 1 × `-Wdeprecated-enum-enum-conversion` from `include/liblvgl/core/lv_obj_style.h` | **ignored**, by PATH |

Allow by **category**, never by file+line: line numbers move with every edit, and a gate that has
to be re-baselined after every commit gets switched off. `-Wunused-function` is the only category
with a written justification today; anything else arriving in `src/` should fail and be considered.

**STATE THE RESIDUAL HOLE IN THE HEADER OF WHATEVER YOU WRITE:** a genuinely new dead function in
`src/` will be allowed by this policy. That is the accepted cost of not re-baselining line numbers.
Naming a hole is what makes it a known limit instead of a surprise — *"a gate's exclusion list is
where its holes live"* (D3).

### 4.4 Shape and placement

A `tools/` Python tool with `check` and `self-test` subcommands matches the house convention and is
what a warning-policy parser wants. **Add it as a step to the existing `arm-compile-gate` job** —
the toolchain is already installed there — and update that job's header comment, which currently
describes a headers-only scope. Keep the job NAME stable.

**It must be one command locally, identical to CI.** A gate that is awkward to run locally gets
discovered in CI, which is the expensive place.

**If the toolchain is missing, FAIL LOUDLY.** Never skip-and-pass: a gate that silently no-ops when
its compiler is absent is worse than no gate, because the green tick lies.

---

## 5. Test requirements

The self-test IS this chunk's test suite, and it must **prove the detectors can fire** — the
standard `doc_staleness_audit.py self-test` already sets. Plant each fault, run the gate against
the planted tree, confirm the verdict, restore. Every case names the bug it catches:

| # | Planted fault | Gate must |
|---|---|---|
| 1 | a syntax/type error in a `src/` file | **FAIL**, naming the file — the data-abort class, the whole reason this chunk exists |
| 2 | a **new** disallowed warning in `src/` (e.g. an unused *variable*, or a sign-compare) | **FAIL** — proves the policy is not "ignore all warnings" |
| 3 | the allowlisted `-Wunused-function` | **PASS** — proves the policy is not "ban all warnings", which would force worse code |
| 4 | a vendor-path warning | **PASS** — proves the path exclusion works |
| 5 | nothing planted (the real tree) | **PASS** |

**Required mutation checks** (break it, RUN it, observe red, restore — a mutation not run does not
count):

| # | Mutation | Must go RED |
|---|---|---|
| 1 | delete the vendor-path exclusion | self-test case 4 |
| 2 | widen the allowlist to "all warnings" | self-test case 2 |
| 3 | narrow the allowlist to "no warnings" | self-test case 3 |
| 4 | make the gate skip-and-pass when the compiler is absent | a missing-toolchain case must FAIL |
| 5 | break the §4.1 variant switch so `ROBOT=xdrive` silently builds bench | the variant assertion (warning-count detector) |

---

## 6. Definition of Done

- [ ] `make` builds the **bench** variant and the **X-drive** variant, each with a working,
      documented invocation that **preserves the build hash**; `main.cpp:64` is true again
- [ ] A gate builds every `src/` TU for the V5 in **both** variants, fails on any error, and
      applies the §4.3 warning policy
- [ ] The gate is one command locally and the SAME command in CI
- [ ] It runs in CI on every push, and a missing toolchain FAILS rather than skips
- [ ] A self-test covers all five §5 cases; all five mutations observed RED then restored
- [ ] The gate PASSES on the current tree, and the expected-warning set is recorded with its
      justification
- [ ] Existing CI is untouched and still green: host suite, PROS-free guard, sim-layering guard,
      the 151-header ARM gate, and all eight documentation gates

---

## 7. Documentation contract

- A live **`GATE1-PROGRESS.md`, created FIRST and appended as the work happens.**
- If any **public** doc changes, the eight gates apply (`api_doc_tool.py` ×5, `briefing_status.py
  check`, `doc_staleness_audit.py` ×2). Adding a `tools/` script and a CI step should touch none of
  the generated API surface — **if `check-coverage` or `check-fresh` fires, something unintended
  became public.**
- `python3 tools/briefing_status.py generate` if repo-derived counts move; never hand-edit inside
  the generated markers.
- **`build-order.md`** — record GATE1 in the position narrative, the DOCS1/DOCS2/DEFECTS1 family.
- **`docs/internal/RESUMING.md`** — its verification list is what a new session runs. **A new gate
  that RESUMING does not mention will not be run by the next session.** Add it.
- **The ABI correction (§1) must be written down** where the old belief would otherwise be
  re-derived — the on-robot build links today, at `gcc 13.2.1`, with `CXX_STANDARD:=gnu++20`.
- Record the §4.1 defect as **found and fixed**, with the measurement that proved it. It is a good
  example of the class: a documented instruction that silently does nothing.
- **Do NOT retro-edit** `chunks/*-PROGRESS.md` or `*-COMPLETED.md`. Append.

---

## 8. Landmines

1. **`make` is incremental.** A gate that compiles nothing passes. Force freshness and print the TU
   count (§4.2).
2. **Command-line make variables override file assignments.** That is exactly how §4.1's second
   failure drops the build hash. Do not pass `EXTRA_CXXFLAGS=` on the command line.
3. **`make` writes `bin/`** (gitignored) and runs `git describe --dirty`. Do not let the gate's own
   artifacts dirty the tree — a dirty tree changes the hash, and the assertion count with it
   (R3a-PROGRESS §12.2, which bit this project twice).
4. **Silence the vendor warning by PATH, not by category** (§3).
5. **A skip is not a pass** (§4.4).
6. **CI checkout is shallow and has no tags**, so `git describe --always` yields a bare short hash
   there and a `v0.1.1-…` string locally. The ARM build does not assert on it — but do not build
   anything that assumes the two match.
7. Run everything **from the repo root**.

---

## 9. Verification baseline — measured 2026-08-19 on a clean tree at `f034b46`

| Check | State |
|---|---|
| `make` (bench variant) | **exit 0**, links, produces `bin/hot.package.bin` |
| `src/` warnings, bench variant | `main.cpp` **3 × `-Wunused-function`**; `bench_r3a.cpp` **0** |
| `src/` warnings, X-drive variant | **0** from `src/` |
| Vendor warnings, both variants | **1** × `-Wdeprecated-enum-enum-conversion`, `include/liblvgl/core/lv_obj_style.h` |
| 8 documentation gates | **8 × PASS** |
| PROS-free + sim-layering guards | **PASS** |
| ARM header gate | **CLEAN, 151 headers** |
| Host suite | **1157 cases / 1,538,101 assertions**, 0 failed, 3 skipped (CLEAN-tree count) |

*Created 2026-08-19, after R3b piece 3 (`f034b46`) had to edit `src/main.cpp` with no gate to catch it.*
