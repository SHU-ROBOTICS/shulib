# G2 — COMPLETED (work landed 2026-09-18 as `cf7a943`; this record written 2026-09-24)

> `PathRunner` + the command-id registry + the manifest → **F8 frozen**. Written FROM
> [`G2-PROGRESS.md`](G2-PROGRESS.md) (the live log, including its interruption and resumption),
> the `cf7a943` commit, the coordinator's verification recorded in
> [`SESSION-2026-09-18-autonomous.md`](SESSION-2026-09-18-autonomous.md) (21:30 entry), and a
> re-run of every gate on 2026-09-24 — not instead of any of them. Brief:
> [`G2-path-runner-and-registry.md`](G2-path-runner-and-registry.md) (committed `e9ba69f`).
>
> **This record is six days late**, found by the 2026-09-24 handoff audit (finding pointers#14 /
> critic#2): G2 landed, verified and committed with its pointer moved, but without this file — so
> `RESUMING.md`'s rule 5 ("a chunk without its `-COMPLETED.md` … is not closed") read it as open,
> and the generated briefing listed it among the interrupted chunks. The GATE1 record carries the
> same lesson; this is the second time it has been earned.

## The one-sentence honest status

A routine can now be **data** — a `config::Route` runs end to end through the frozen facade, its
markers call handlers a coder registered once by id, and the id vocabulary is exported for a path
editor — and the data half of that contract (F8) is locked; **a route has executed on the A2 host
test plant only, and nothing of Phase G has run on a robot.**

## Built, file by file

- **`include/shulib/runner/command_registry.hpp`** — `CommandRegistry`: 32 fixed slots, no heap, no
  `std::function`; `on(id, handler)` takes a non-owning callable and adapts its return value the
  way `Routine::then()` does; ids are `[a-z0-9_]`, ≤ 23 bytes; a duplicate id, a built-in id, a bad
  id or a full registry is a precondition violation.
- **`include/shulib/runner/path_runner.hpp`** — `PathRunner::run`: `Start`/`MoveToPose`/`DriveTo`/
  `TurnTo`/`StrafeTo`/`Hold` straight onto the facade's verbs (never through `Routine`, D1's
  ruling); `bearingTo` is `Routine::bearingTo`'s expression and pinned bit-identical to it; markers
  fire ONLY on `ExitReason::Settled`; the first failure stops the run, `cancel()`s the drive to its
  safe state and counts the skipped remainder; an unknown id is one Warn line and a count, and the
  route continues.
- **`include/shulib/runner/manifest.hpp`** — `writeManifest`: line-JSON through `ICharSink` — a
  schema-version line, the four library-owned built-ins (`wait_ms`, `brake`, `hold_ms`, `log`) in a
  fixed order, then the registered ids; each line carries `minArgs`/`maxArgs`/`doc` (ruling
  R-MANIFEST-FIELDS: `log` takes 0–2 args, so one `args` number cannot describe it). Byte-exact golden.
- **`include/shulib/config/route.hpp`** — `kMaxMarkerIdBytes` named, the built-in table and
  `findBuiltinCommand`, and `Route::valid()`'s G2 rules; **one rule RELAXED** (ruling R6-SPLIT:
  `MoveToPose` + `HeadingMode::Free` is legal and means "keep the heading the estimate has";
  `TurnTo` + `Free` is still refused). Relaxing a validity rule is the additive direction — every
  route valid under 2.1 is valid under 2.2.
- **`test/f8_signature_pin_test.cpp`** — F8 pinned in F10's mould: a changed frozen signature fails
  the BUILD naming F8, in both directions of `noexcept`.
- **`include/shulib/version.hpp`** — API 2.1 → **2.2** (additive).
- **Tests:** `command_registry_test.cpp` (tests 1–2), `path_runner_test.cpp` (3–10, 10b),
  `manifest_test.cpp` (11, 11b, 11c), `route_test.cpp` 2j (test 13) and 2h rewritten by R6-SPLIT.

## Frozen (F8), and what is deliberately not

Frozen: the in-memory `Route` vocabulary and enum values, the registry's members, `run()`'s
signature, `RunReport`'s fields, the built-in id strings, and the file schema's field names in
[`docs/vexbuilder-integration.md`](../../vexbuilder-integration.md) §4.1–4.3 (reconciled to the
code: `"locked"`→`"fixed"`, `"tangent"`→`drive`, `args` a positional int array, `"arc"` dropped as
a NAMED gap, `NONE` answered). The Freeze Register row in [`roadmap.md`](../../roadmap.md) carries
the frozen list, the additive list and the not-frozen list.

## Evidence

- **Interrupted and resumed.** The first executor was stopped at the reboot (19:39, 2026-09-18) with
  code complete but never built; a second executor re-derived everything from the files and found
  three defects on the first full build — `version.hpp` unbumped (the F8 pin's own case red),
  `Route::valid()` refusing the combination ruling R6 defines, and a tank test asserting what the
  library deliberately does not do (a tank route must carry its turns; the test now pins that both
  ways). All three fixed and logged (`G2-PROGRESS.md` R2–R3).
- **Mutations:** 14 run against the final code, 14 red, 0 survived (`G2-PROGRESS.md` R4, observed
  messages quoted). The coordinator re-ran two by hand on fresh binaries: `atan2` arguments swapped
  ⇒ the bit-identity REQUIRE and the reverse-heading checks red; markers moved ahead of the Settled
  gate ⇒ "fired == 0" red. One process finding kept: an `inline constexpr` table leaves stale
  objects unless EVERY including test file is touched before a fast relink.
- **Suite at landing:** 1342 / 1,632,353 → 1358 cases / 1,635,290 assertions, green; exactly one
  pre-existing case moved (+9, the CI-guard mirror: three per new header). **Correction
  (2026-09-24):** 1,635,290 included six assertions from a stale `-dirty` build hash in the test
  build; the honest count at the same code was 1,635,284, and 1,635,266 since the build-identity
  fix (`05ad50a`, which made the count independent of the hash). No test changed meaning.
- **Re-verified 2026-09-24 at `42eea85`** (no source file has changed since `cf7a943`; only
  documents): host build + doc gates; suite 1358 / 1,635,266 green; `src` build gate self-test +
  check (six builds, beacons); ARM gate; both CI greps; api freshness; briefing; C7 removability.
  All pass.

## Corrections to the live log, made here rather than in it

- `G2-PROGRESS.md` R7 says the ARM gate compiled "ALL 145 headers". The ARM gate compiles every
  `include/shulib/**/*.hpp` — **181** at landing; 145 is the doc-coverage gate's count of headers
  with public entities. (Audit finding critic#13.)
- **The changelog was not written when the change landed.** `docs/changelog.md` promises "one
  section per API version … written when the change landed"; API 2.2 had no section until the
  2026-09-24 audit (finding critic#4). It is added in the same commit as this record, marked as
  written late, and covers everything user-visible since 2.1 — not only G2.

## Documentation that moved with the chunk (cf7a943 and 225d0ac)

`docs/vexbuilder-integration.md` §4.1–4.3 (the frozen contract), `docs/roadmap.md` (F8 row, M5
bullets, the pointer), `docs/legacy-command-vocabulary.md` (three sentences that named G2 as owner
of a decision G2 closed), `docs/internal/build-order.md`, `docs/api/` + `mkdocs.yml`, the briefing;
and at `225d0ac` the master plan's `runner.on(...)` → `registry.on(...)` (the team lead: "if the
code is fine, then we change the docs").

## Honest partials — what G2 did not do

- **No hardware.** A route has run on the host test plant only.
- **No guide chapter.** The "your first data-driven routine" chapter waits for G3, because today the
  only way to get a `Route` is to type it in C++ — the audience the feature exists for cannot follow
  it yet. M5's DoD is not ticked.
- **The `.vexbot` reader is G3's.**
- **A tank cannot `DriveTo` a point far off its heading without an authored turn** — that is the
  facade's deliberate honesty (the library never sequences a turn you did not ask for), and a path
  editor must emit the turns.
