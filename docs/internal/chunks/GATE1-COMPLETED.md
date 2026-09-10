# GATE1 — COMPLETED (work landed 2026-08-19 as `7d3e7ce`; this record written 2026-09-10)

> The `src/` build gate — reactive, out of order, in the DOCS1/DOCS2/DEFECTS1 family.
> Written FROM [`GATE1-PROGRESS.md`](GATE1-PROGRESS.md) (the live log) and from a re-verification
> on 2026-09-10, not instead of either. Brief: [`GATE1-src-build-gate.md`](GATE1-src-build-gate.md)
> (committed `82caa30`).
>
> **This record is three weeks late.** The work was verified and committed on 2026-08-19, but the
> close-out — this file and the `roadmap.md` pointer — was left as "the reviewer's call" in the
> log's §11 and never made. A chunk without its completion record and pointer move is not closed;
> that rule is now written into `RESUMING.md`'s pace rules, and this is the case that earned it.

## The one-sentence honest status

`src/` is under a real gate — CI drives the actual `make`, compile AND link, for every robot
variant, and fails on what the warning policy says it fails on — and along the way two standing
records were corrected by measurement: the on-robot link was never blocked, and the documented
variant flag had been a silent no-op; **what remains unproven is the CI job itself on a GitHub
runner, because nothing has been pushed since 2026-08-17.**

## Built, file by file

- **`tools/src_build_gate.py`** (449 lines) — `check` (default) and `self-test`. `check` deletes
  `bin/` and `.d/` (both gitignored — a gate over an incremental `make` must force freshness),
  runs `make ROBOT=<variant> V=1` per variant, counts the compiled `src/` TUs against a globbed
  list (generated, never enumerated), parses the compiler output with ANSI and `./` normalization,
  and applies the policy: any `error:` fails naming the file; a warning from a vendored path
  (`include/liblvgl/`, `include/pros/`) is ignored BY PATH; `-Wunused-function` in our code is
  allowed BY CATEGORY (residual hole stated in the header: a genuinely new dead function passes);
  an uncategorized warning fails closed; anything else fails. Two variant detectors, because each
  has a blind spot the other covers: STRUCTURAL (every `src/` C++ compile in the `xdrive` build
  carries `-DSHULIB_ROBOT_XDRIVE_INVENTED`, the `bench` build none, BOTH carry
  `-DSHULIB_BUILD_HASH=`) and BEHAVIORAL (the bench build emits ≥ 1 `-Wunused-function` from
  `src/main.cpp` because the invented X-drive wiring is dead code there; the xdrive build emits 0 —
  a deliberate coupling to the Makefile's own "the warnings are how you see the wiring is dead"
  rationale, stated in the tool). The build hash must appear in the bytes of
  `bin/hot.package.elf` in both variants. A missing toolchain FAILS; it never skips.
- **`Makefile`** — the validated `ROBOT ?= bench` / `ROBOT=xdrive` switch; any other value is a
  loud `$(error)`. Both `EXTRA_CXXFLAGS` appends (the hash and the variant define) carry `override`,
  so a command-line `EXTRA_CXXFLAGS=…` can no longer silently discard them.
- **`src/main.cpp:64-69`** — the build instruction corrected to `make ROBOT=xdrive`, the old
  transposed instruction recorded inline as found-and-fixed.
- **`.github/workflows/ci.yml`** — two steps appended to `arm-compile-gate` (`self-test`, then
  `check`, the identical local commands); the job and file header comments corrected: the link IS
  in CI now, only the on-robot RUN is not.
- **`docs/internal/RESUMING.md`** — the two gate commands added to the verification list, and the
  boxed "standing belief RETIRED" note so no session re-derives the old blocker.
- **`docs/internal/build-order.md`** — the GATE1 paragraph in the position narrative, the
  DOCS/DEFECTS family row, the chunk total (46 → 47), a deviations-table row.

## Two records corrected by measurement

1. **The on-robot build was never blocked.** The tree carried, for weeks, the belief that
   `firmware/` + `liblvgl.a` were soft-float and the link could not succeed at apt's toolchain.
   Measured: `rm -rf bin .d && make` → exit 0, `bin/hot.package.bin`, ~2 s clean build at
   `arm-none-eabi-g++ 13.2.1` under the `Makefile`'s `CXX_STANDARD:=gnu++20` pin. The gate is
   therefore the real build, not a compile-only proxy.
2. **The documented variant flag was a silent no-op.** `make CXXFLAGS_EXTRA=-D…` built the BENCH
   variant (3 tell-tale warnings); the one working spelling (`EXTRA_CXXFLAGS=`) silently dropped
   the build hash, because a command-line make variable discards a plain `+=`. Both halves fixed
   as above; both verified by warning counts and by `strings` on the ELF.

## Definition of Done — with evidence

| Item | Evidence |
|---|---|
| `make` builds bench AND X-drive with a documented invocation that preserves the hash; `main.cpp:64` true | PROGRESS §4 table: `make` → 3 warnings, hash present; `make ROBOT=xdrive` → 0 warnings, hash present; `ROBOT=xdrvie` → `$(error)` exit 2 |
| A gate builds every `src/` TU for both variants, fails on error, applies the §4.3 policy | PROGRESS §5: first `check` PASS — bench 2/2 TUs, 3 allowed, 2 vendor-ignored, 0 disallowed; xdrive 2/2, 0/2/0 |
| One command locally, the same in CI | `ci.yml` steps run `python3 tools/src_build_gate.py self-test` then `check`, byte-identical to the local commands |
| Runs in CI on every push; a missing toolchain FAILS | steps present in `arm-compile-gate`; mutation 4 (skip-and-pass) observed red. **The runner itself: not yet exercised — see below** |
| Self-test covers the five brief cases; five mutations red then restored | PROGRESS §6 (10 detector cases, ~11 s, each plant confirmed to FIRE) and §7 (5 mutations incl. 5a/5b, every red quoted verbatim, byte-exact restores) |
| Gate PASSES on the tree; expected warnings recorded with justification | §5 and §10; the 3 `-Wunused-function` sites named (`shaped`, `portMapString`, `robot`) and the 1 vendored site (`lv_obj_style.h:94`, counted twice — once per TU) |
| Existing CI untouched and green | host-tests job: zero diff lines; battery §10: suite 1157 / 1,538,107 dirty (= 1,538,101 clean + the 6-byte `-dirty`), 5 doc gates, 2 audits, both guards, 151-header ARM gate |

**Re-verified 2026-09-10 on the clean tree at `7d3e7ce`** (by the coordinator, before writing this):
suite **1157 / 1,538,101 / 0 failed / 3 skipped**; `src_build_gate.py check` **PASS**, both
variants link, hash `v0.1.1-273-g7d3e7ce` asserted in both ELFs; `arm-none-eabi-g++ 13.2.1`.

## What is NOT done, stated plainly

- **The CI job has never executed on a GitHub Actions runner.** No runner exists locally, and
  `shulib-v2` has not been pushed since 2026-08-17 (33 local commits at the time of writing). The
  YAML parses and the commands are the verified local ones; the one environmental difference
  (shallow, tagless checkout → bare short hash) is designed for. The first push is the proof.
- **The behavioral detector is coupled to the invented X-drive wiring staying dead code in the
  bench build.** A chunk that livens or deletes that wiring must update the detector with it — and
  R3b session 2 (in flight) adds a third variant, `ROBOT=tank`, which must define its own expectation.
- **The residual policy hole stands by design:** a new dead function in `src/` passes.
- **The gate proves the package BUILDS, never that it works.** Upload and run remain R3a/R3b/R3c.

## Successor

R3b session 2 extends the switch and the gate to `ROBOT=tank` (the season's chassis) — see
[`R3b-session2-tank-chassis.md`](R3b-session2-tank-chassis.md) §3.4.
