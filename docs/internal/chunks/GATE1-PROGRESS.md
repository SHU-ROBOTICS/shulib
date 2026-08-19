# GATE1 — live progress log

> Appended as the work happens. Created FIRST, before any code, per the documentation contract.
> Brief: [`GATE1-src-build-gate.md`](GATE1-src-build-gate.md). Tree at start: CLEAN at `82caa30`.

---

## 1. Session start — required reading done, plan formed (2026-08-19)

Read, in the brief's order: the brief itself, `RESUMING.md`, `.github/workflows/ci.yml`
(both jobs; `arm-compile-gate` installs `gcc-arm-none-eabi` already), `Makefile` (WARNFLAGS
rationale, `SHULIB_GIT_HASH` injection via `EXTRA_CXXFLAGS+=`, the `CXX_STANDARD:=gnu++20` pin),
`common.mk` (the cxx rule at :279 consumes `EXTRA_CXXFLAGS`; `test_output_2` prints the full
command line at `V=1` and prints compiler output under a `[WARNINGS]`/`[ERRORS]` marker;
`clean::` removes `bin/` and `.d/`), `src/main.cpp:49-66` (the variant `#ifdef` and the wrong
instruction at :64), and `tools/doc_staleness_audit.py` end to end (the self-test convention:
plant, run, confirm, restore, count the checks).

Local toolchain measured: `arm-none-eabi-g++ 13.2.1`, GNU Make 4.3, Python 3.12.3 — same
compiler version the brief measured.

**Plan, in order:**
1. Baseline measurements of my own (re-measure the brief's §9 facts I depend on: `make` links,
   the 3/0 warning split, the vendor warning, the §4.1 transposition, the hash drop).
2. Fix §4.1 in the `Makefile` (named variant switch) + fix `src/main.cpp:64`. Verify by
   measurement (warning-count detector + `strings` for the hash, both variants).
3. `tools/src_build_gate.py` with `check` and `self-test` subcommands (house convention).
4. CI: new steps in the existing `arm-compile-gate` job (name kept stable), header comment
   updated from headers-only scope.
5. All five self-test cases; all five mutations run RED and restored.
6. Full verification battery + documentation contract (RESUMING, build-order, ABI correction).

## 2. Design decisions taken up front (alternatives recorded as they are decided)

### 2.1 Variant switch naming — `ROBOT ?= bench` / `ROBOT=xdrive`
Chosen: the brief's suggested shape, because the variable names the thing actually being
selected — WHICH PHYSICAL ROBOT the binary boots on — and because an enumerated variable can be
**validated**: any value outside {bench, xdrive} is a `$(error)`, so a typo (`ROBOT=xdrvie`)
fails loudly instead of silently building the default. That last property is the whole lesson
of §4.1: the defect class is "an instruction that silently does nothing".

**Rejected alternative:** keep the documented `CXXFLAGS_EXTRA` name and forward it
(`EXTRA_CXXFLAGS += $(CXXFLAGS_EXTRA)`). Rejected because (a) a free-form flags variable cannot
be validated — any misspelling of the -D define is again a silent no-op; (b) it keeps a second
flags-carrying variable alive next to `EXTRA_CXXFLAGS`, inviting the exact command-line-override
collision that drops the build hash; (c) the caller's intent ("build the X-drive robot") is
better stated as a named switch than as a raw preprocessor define.

### 2.2 Gate tool shape — `tools/src_build_gate.py check | self-test`
Matches `api_doc_tool.py` / `doc_staleness_audit.py`: a Python tool whose self-test proves the
detectors fire. One command locally, the identical command in CI.

### 2.3 The gate drives the REAL `make` (not a re-implemented compile line)
Per the brief §4.2 and constraint 9. The tool shells out to `make ROBOT=<variant> V=1`:
`V=1` makes `common.mk`'s `test_output_2` print every full compiler command line, which gives
the gate structural evidence (the exact `-D` flags each TU was compiled with) without
re-implementing any flag. No second source of truth.

### 2.4 Freshness: delete `bin/` + `.d/` before each gated build; count TUs
`make` is incremental (landmine 1). The gate removes `bin/` and `.d/` (both gitignored — the
same set `make clean` removes) before each build, then counts `Compiled src/...` lines in the
output and compares against the TU list it globs from `src/` itself (generated, never
enumerated — the A4 precedent). Mismatch = FAIL, and the count is printed so a silent no-op
would be visible in the CI log.

### 2.5 Warning policy (§4.3), encoded exactly
- any `error:` → FAIL, naming the file.
- warning from a vendored path (`include/liblvgl/`, `include/pros/`) → ignored, BY PATH.
- warning from our code with category `-Wunused-function` → allowed, BY CATEGORY.
- any other warning → FAIL.
- RESIDUAL HOLE, stated in the tool header: a genuinely NEW dead function in `src/` passes.
  Accepted cost of never re-baselining line numbers.

### 2.6 The variant assertion — two detectors, structural + behavioral
1. **Structural** (from the `V=1` command lines): every `src/` C++ compile in the `xdrive`
   build must carry `-DSHULIB_ROBOT_XDRIVE_INVENTED`; the `bench` build must carry none; BOTH
   builds must carry `-DSHULIB_BUILD_HASH=` (when `git describe` yields anything) — this is
   §4.1's second failure (the dropped hash) encoded as a permanent check.
2. **Behavioral** (the brief's warning-count detector): the bench build emits ≥1
   `-Wunused-function` attributed to `src/main.cpp` (the invented X-drive wiring is dead code
   there, and the Makefile documents those warnings as the evidence of that); the xdrive build
   emits 0. This catches the case the structural check cannot: the define passed but the
   `#ifdef` in `main.cpp` renamed/broken.
   **Known coupling, stated here and in the tool:** if a future chunk deletes the invented
   X-drive wiring or makes both variants warning-free, this behavioral assertion is the line
   to update — it is a deliberate restatement of the Makefile's "the warnings are how you see
   the wiring is dead" rationale, so it should change only when that rationale does.
3. Additionally the hash string (computed by the tool with the same
   `git describe --always --dirty --abbrev=7` the Makefile uses, immediately before each build)
   must appear in the bytes of `bin/hot.package.elf` in BOTH variants — the end-to-end version
   of the structural check, and the exact measurement the brief used.

### 2.7 Missing toolchain = FAIL (constraint 5)
`shutil.which("arm-none-eabi-g++")` (resolution overridable for the self-test via an explicit
PATH argument, so the missing-toolchain verdict is testable without uninstalling a compiler).
Absent → print why and exit 1. Never skip.

---
## 3. Baseline re-measured myself (2026-08-19, tree at 82caa30 + this log)

| Measurement | Result | Matches brief §9? |
|---|---|---|
| `rm -rf bin .d && make` | exit 0, links, `bin/hot.package.bin` (22,176 B), **~2.1 s clean build** | yes |
| bench warnings | `src/main.cpp` 3 × `-Wunused-function` (`shaped`, `portMapString`, `robot`); `bench_r3a.cpp` 0 | yes |
| vendor warning | 1 × `-Wdeprecated-enum-enum-conversion` at `./include/liblvgl/core/lv_obj_style.h:94` — note the `./` prefix and ANSI color codes in the raw output; the parser must strip/normalize both | yes |
| `make CXXFLAGS_EXTRA=-DSHULIB_ROBOT_XDRIVE_INVENTED` | **3 warnings — bench built; documented flag is a silent no-op** | yes (§4.1 half 1) |
| `make EXTRA_CXXFLAGS=-DSHULIB_ROBOT_XDRIVE_INVENTED` | 0 warnings — X-drive built — but `strings bin/hot.package.elf` finds **no** `v0.1.1` hash | yes (§4.1 half 2) |
| `git describe --always --dirty --abbrev=7` with ONLY an untracked file added | `v0.1.1-272-g82caa30` — **no `-dirty`**: untracked files do not dirty `git describe` | new fact, relevant to the gate's hash check |
| `firmware/*.mk` | none exist; only `Makefile` + `common.mk` touch `EXTRA_CXXFLAGS` | — |

The ~2 s clean build settles a design worry: the self-test can afford one real build per
planted-fault case (5+ builds ≈ seconds, not minutes).

## 4. §4.1 fix — the `ROBOT` variant switch (Makefile) + `main.cpp:64`

Implemented as decided in §2.1, with one hardening beyond the brief's minimum, recorded here
because it changes a measured behavior:

**`override` on the two `EXTRA_CXXFLAGS +=` lines** (the hash append and the variant append).
GNU make semantics: a command-line `EXTRA_CXXFLAGS=…` silently discards plain `+=` file appends
— that is exactly §4.1's second failure. With `override … +=`, the append lands ON TOP of any
command-line value instead of being discarded. Consequence: `make EXTRA_CXXFLAGS=-DFOO` now
KEEPS the build hash (the old behavior shipped a hash-less binary), and
`make ROBOT=xdrive EXTRA_CXXFLAGS=-DFOO` keeps both the hash and the variant define. The hash
is documented in the Makefile as load-bearing and `--dirty`-bearing; there is no legitimate
invocation that wants it dropped. Rejected alternative: leave the landmine and rely on nobody
passing `EXTRA_CXXFLAGS=` — rejected because the gate cannot police human invocations, and the
class is now structurally dead instead of documented-around. (To be verified by measurement
below.)

**§4.1 fix VERIFIED BY MEASUREMENT (all runs from a fresh `rm -rf bin .d`):**

| Invocation | unused-function warnings | hash in `strings bin/hot.package.elf` | verdict |
|---|---|---|---|
| `make` | 3 | `v0.1.1-272-g82caa30-dirty` | bench, hash intact |
| `make ROBOT=xdrive` | 0 | `v0.1.1-272-g82caa30-dirty` | **X-drive, hash intact — the §4.1 defect is closed** |
| `make ROBOT=xdrvie` (typo) | — | — | `Makefile:66: *** unknown ROBOT 'xdrvie' ...` **exit 2, loud** |
| `make EXTRA_CXXFLAGS=-DGATE1_PROBE` | — | present (count 1) | **hash no longer droppable** (`override` hardening) |
| `make ROBOT=xdrive EXTRA_CXXFLAGS=-DGATE1_PROBE` | 0 | present | both survive together |
| `make ROBOT=xdrive V=1` | — | — | compile lines show `-DSHULIB_BUILD_HASH` ×2 and `-DSHULIB_ROBOT_XDRIVE_INVENTED` ×2 (both C++ TUs) — the gate's structural check has what it needs |

`src/main.cpp:64` corrected to `make ROBOT=xdrive`, with the old transposed instruction
recorded inline as found-and-fixed (the defect class: a documented instruction that silently
does nothing).

Also verified for the self-test design: `src/bench_r3a.cpp` is UNGUARDED (compiles in both
variants, ends with a namespace close) — end-of-file fault plants are safe there.

## 5. The gate tool — `tools/src_build_gate.py`

Written as designed in §2.2–§2.7. 449 lines. Shape: `check` (no-arg default) and `self-test`.
Key implementation facts, measured while building it:

- **ANSI + path normalization**: make's output carries `\x1b[...m`/`\x1b[K` sequences and
  header paths print as `./include/...`; the parser strips/normalizes both.
- **Vendor count is 2, not the brief's 1** — same single site (`lv_obj_style.h:94`,
  `-Wdeprecated-enum-enum-conversion`) firing once per TU: both `src/main.cpp` and
  `src/bench_r3a.cpp` include it via `main.h`. The brief counted unique sites; the parser
  counts occurrences. Same substance, recorded so the "2 vendor-ignored" in gate output does
  not read as a new warning.
- **Uncategorized warnings fail closed**: a warning in our code with no `[-W...]` tag is
  disallowed, not shrugged off.
- **`git describe` yielding nothing FAILS the hash assertion** rather than skipping it
  (decision recorded in the tool header): every supported environment (local checkout, CI
  checkout) has git; a hash-less binary is legal at RUNTIME per build_info.hpp's loud-missing
  contract, but a gate that cannot verify identity must say so in red. Rejected alternative:
  loud-note-and-pass — rejected because "a skip is not a pass" is this chunk's constraint 5
  and the exception it would serve does not exist in practice.
- **First `check` run: PASS.** bench: 2/2 TUs, 3 allowed `-Wunused-function`, 2
  vendor-ignored, 0 disallowed, hash asserted. xdrive: 2/2 TUs, 0 allowed, 2 vendor-ignored,
  0 disallowed, hash asserted. Exit 0.

## 6. Self-test — all five brief §5 cases, run against REAL builds

`python3 tools/src_build_gate.py self-test` → **OK, 10 detector cases, ~11 s** (the ~2 s
clean build is what makes one real build per case affordable). Each case plants, builds,
confirms, restores byte-exact (asserted). The five brief cases plus a sixth:

| # | Planted | Verdict observed | Extra assertion (so a case that never fires proves nothing) |
|---|---|---|---|
| 1 | `this line is not C++` at end of `src/bench_r3a.cpp` | FAIL | failure text names `bench_r3a.cpp` |
| 2 | file-scope `static int ... = 42;` (unused variable) | FAIL | failure names `-Wunused-variable` AND the file |
| 3 | anonymous-namespace dead function | PASS | the planted `-Wunused-function` actually FIRED and was attributed to `src/bench_r3a.cpp` in the allowed list |
| 4 | `static int ... = 7;` at end of `include/liblvgl/core/lv_obj_style.h` — a DISALLOWED category (`-Wunused-variable`) from a vendor path | PASS | the planted warning actually FIRED and landed in the vendor-ignored list |
| 5 | nothing (real tree) | PASS via the full two-variant `check` | — |
| 6 | toolchain absent (PATH overridden to an empty dir via the tool's `path_env` seam) | FAIL before any `make` runs | — |

Case 4's plant is deliberately a category the policy FORBIDS — that is what proves the
exclusion is by PATH; an allowed-category plant would pass for the wrong reason.

## 7. Mutation campaign — all five run, RED observed, restored

Method: snapshot originals to the session scratchpad, apply one mutation, RUN the gate,
record the observed red, restore, confirm green before the next. `tools/src_build_gate.py`
restoration verified byte-exact by `diff`; `Makefile` restored from snapshot; zero
`MUTATION` markers left in the tree (grep-verified).

| # | Mutation (as applied) | Observed RED |
|---|---|---|
| 1 | `VENDOR_PREFIXES = ()` | self-test exit 1. **Case 4 red twice**: "a vendor-path warning FAILED the gate" (the real `-Wdeprecated-enum-enum-conversion` became disallowed) AND "the planted vendor warning never fired" (it was classified disallowed, not vendor). Collateral red: cases 3 and 5 (the ever-present vendor warning now fails every build) — expected, noted as such. |
| 2 | `elif cat in ALLOWED_CATEGORIES:` → `elif True:` (allow ALL categories) | self-test exit 1. **Case 2 red**: "a disallowed warning (-Wunused-variable) in src/ passed". |
| 3 | `ALLOWED_CATEGORIES = set()` (ban ALL warnings) | self-test exit 1. **Case 3 red**: the planted AND the three documented `-Wunused-function` all became disallowed — the observed failure list names `shaped`/`portMapString`/`robot` at `src/main.cpp:275/264/257`, which is precisely the worse-code-for-a-gate's-convenience outcome the no-Werror ruling exists to prevent. Collateral red: cases 4, 5. |
| 4 | missing-toolchain branch `return 1` → `return 0` (skip-and-pass) | self-test exit 1. **Case 6 red**: "a missing toolchain did not FAIL the gate". |
| 5a | Makefile variant append DELETED (`ROBOT=xdrive` silently builds bench) | `check` exit 1. **Structural detector red**: "[xdrive] ROBOT=xdrive but src/main.cpp compiled without -DSHULIB_ROBOT_XDRIVE_INVENTED — the variant switch is a silent no-op again (§4.1)" (both TUs named). |
| 5b | Makefile appends look-alike `-DSHULIB_ROBOT_XDRIVE_INVENTED_BROKEN` — defeats the structural substring check; behavior still bench | `check` exit 1. **The brief's warning-count detector red, on its own**: "[variant] xdrive build emitted 3 -Wunused-function from src/main.cpp — the X-drive wiring should be LIVE in this variant; ROBOT=xdrive looks like a silent no-op (§4.1)". This is why the gate carries BOTH detectors: 5b is invisible to the structural check. |

Mutation 5 was run in two forms because the two detectors have different blind spots; the
brief's required red (warning-count) is 5b, observed verbatim above.

**Post-restoration: self-test OK (10 cases, exit 0), check PASS (exit 0).** Only intended
edits remain in the tree (`Makefile`, `src/main.cpp`, plus the two new files).

*(Session interruption note: the executing session was killed by an API connection error
between running the mutations and logging them; the coordinator re-verified check/self-test
green on the left state before this section was written. Sections 7 onward are written in
the resumed session from the recorded observations above — every quoted red is a verbatim
observed output, not a reconstruction.)*

## 8. CI wiring — `arm-compile-gate` widened, name kept stable

`.github/workflows/ci.yml`:
- **Two new steps** at the end of `arm-compile-gate`: `python3 tools/src_build_gate.py
  self-test` then `... check` — the identical commands run locally (one-command rule §4.4).
  Self-test runs FIRST, matching the doc-gates convention (prove the detectors, then gate).
- **Job header comment rewritten** — it described a headers-only scope and claimed the link
  "still needs PROS's bundled toolchain", which GATE1 measured false. The comment now states
  the two-layer scope honestly (header COMPILE gate + real src/ BUILD gate, link included)
  and notes the job name predates GATE1 and is kept stable.
- **File-level header comment corrected the same stale claim** — it said "the on-robot ARM
  LINK/run is intentionally NOT in CI". The LINK is in CI now; only the on-robot RUN remains
  out (R3a/R3b/R3c). This is one of the two places the retired ABI belief lived.
- **host-tests job untouched** (verified by diff: no `-`/`+` lines anywhere in its body).
- **YAML validated with `yaml.safe_load`** — parses. HONEST LIMIT: **no GitHub Actions
  runner exists here, so the CI job itself was never executed by a runner.** The commands it
  runs are byte-identical to the locally-verified ones, and the job's one environmental
  difference (shallow, tagless checkout → bare short hash from `git describe`) is handled:
  the tool computes the hash fresh in the same environment as make does, asserting equality
  with itself, never with the local `v0.1.1-…` form (landmine 6).

## 9. Documentation contract — where each required record went

| Requirement | Where |
|---|---|
| RESUMING verification list | `docs/internal/RESUMING.md` §3: the two gate commands added after the ARM header gate, with a note that the gate cleans/rebuilds only gitignored dirs |
| ABI correction, where the old belief would be re-derived | THREE places: `RESUMING.md` (the block a new session reads first — a boxed "standing belief RETIRED, do not re-derive"), `ci.yml`'s two corrected comments (which carried the stale claim verbatim), and `build-order.md`'s GATE1 paragraph. The `Makefile` already carried the gnu++20-pin rationale |
| §4.1 found-and-fixed, durable (survives the internal-docs drop at merge) | `Makefile` ROBOT block comment (the defect, the measurement date, the override rationale) + `src/main.cpp:64-69` (old instruction recorded inline as found-and-fixed). Narrative copies: `build-order.md` GATE1 paragraph + deviations row; full measurement tables in THIS log §3–4 |
| build-order.md | GATE1 paragraph in the position narrative (after the R3b in-flight block); DOCS/DEFECTS family row now lists GATE1; total 46 → **47 chunks** with the history line extended; a deviations-table row |
| briefing regenerate | Repo-derived counts moved (total 47; GATE1 flagged as in-flight). Ran `generate` BEFORE the host reconfigure, while the on-disk test binary still carried the clean-tree hash — so the committed suite line keeps the CLEAN count **1,538,101** (per R3a §12.2: the committed number must be what a fresh clone reproduces). Generate diff = exactly two derived lines (total, interrupted list) |
| check-coverage / check-fresh untouched by this chunk | both PASS — nothing unintended became public (1645 entities / 118 headers, unchanged by this chunk; the counts predate GATE1) |

## 10. Final verification battery (dirty tree, all from repo root)

| Check | Result |
|---|---|
| host suite (fresh configure + build) | **1157 cases / 1,538,107 assertions, 0 failed, 3 skipped** — 107 is the EXPECTED dirty-tree number (`-dirty` = 6 chars, asserted through; R3a §12.2); clean-tree value stays 1,538,101 and that is what every committed doc carries |
| api_doc_tool self-test / coverage / fresh / examples / removability | 5 × exit 0 |
| doc_staleness_audit self-test / audit | 2 × exit 0 |
| briefing_status check | **exit 1, on the suite line ONLY** (committed clean 1,538,101 vs dirty-binary 1,538,107 — verified the diff contains nothing else; total-47 and interrupted-GATE1 lines are current). This is §12.2's documented dirty-tree state, NOT a regression: it self-heals on a clean tree, and in CI the suite line is exempt (gates run before the build, no binary exists). Writing 107 into the doc to silence it would be writing a number a fresh clone cannot reproduce |
| PROS-free guard | prints nothing (grep exit 1) |
| sim-layering guard | prints nothing (grep exit 1) |
| ARM header gate | **151 headers**, exit 0, `-Werror` clean |
| `src_build_gate.py check` | PASS exit 0 (bench 2/2 TUs, 3 allowed, 2 vendor-ignored, 0 disallowed, hash asserted; xdrive 2/2, 0 allowed, 2 vendor-ignored, 0 disallowed, hash asserted) |
| `src_build_gate.py self-test` | OK, 10 detector cases, exit 0 |
| `yaml.safe_load(ci.yml)` | parses |
| `git status` | exactly the 8 intended paths (6 modified + 2 new), nothing staged, nothing committed |

## 11. What is NOT done / could not be verified — stated plainly

- **The CI job was never executed by a GitHub Actions runner.** No runner exists in this
  environment. Verified instead: the YAML parses, the job name is stable, the steps run the
  byte-identical commands verified locally, and the one CI-environment difference (tagless
  shallow checkout → bare-hash `git describe`) is designed for rather than assumed away.
  The first real push is the remaining proof.
- **`briefing_status.py check` reads exit 1 at handoff** — suite line only, dirty-tree
  artifact per §12.2 (see §10). Expected to read green after commit on a clean tree without
  any further action; if it does not, that is a real finding, not this chunk's residue.
- **The behavioral variant detector is coupled to the invented X-drive wiring staying dead
  code in the bench build** (≥1 `-Wunused-function` from `src/main.cpp`). Deliberate,
  documented in the tool header and §2.6 — it restates the Makefile's own rationale — but a
  future chunk that deletes or livens that wiring must update the differential alongside it.
- **The residual policy hole stands, by design:** a genuinely NEW dead function in `src/`
  passes the gate (`-Wunused-function` is category-allowed). Stated in the tool header.
- **The gate proves the package BUILDS, never that it works.** Upload/run on a brain is
  R3a/R3b/R3c scope, untouched.
- **`bench_r3a.cpp`'s 0-warning state is asserted only via "0 disallowed"** — no positive
  per-file expectation list exists, by design (file-level expectations are the re-baseline
  trap the category policy avoids).
- Nothing committed, nothing staged, nothing pushed. Roadmap "you are here" not moved —
  that is the close-of-chunk step and the reviewer's call.

*Log complete. Handing to independent verification.*
