# Chunk A4 — COMPLETED (2026-08-06)

> Completion record for [`A4-assumptions-register.md`](A4-assumptions-register.md) — the Hardware
> Assumptions Register + the ARM compile gate, **the chunk that closes Phase A**. Everything below
> is as actually observed — commands run, outputs captured, the gate mutation executed and watched
> fail (the live sequence is in [`A4-PROGRESS.md`](A4-PROGRESS.md)). Changes are in the working
> tree, uncommitted, pending review.
>
> A4 is deliberately smaller than A2/A3 — a documentation + CI chunk that adds **no test surface**
> (the brief and build-order both say so; the suite count is unchanged and that is correct, not a
> shortfall). What it adds is the artifact that converts "we built a lot without a robot" from a
> risk into a plan.

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| **The Hardware Assumptions Register** | `docs/planning/hardware-assumptions.md` *(new)* | **49 falsifiable claims** about physical hardware, grouped as Phase R's walk-order checklist: R3 conventions/bindings/geometry (HA-01..19), R4 characterization (HA-20..44), R5 gains (HA-45..47), R6 model adequacy (HA-48..49). Each: claim · source file:line · confidence · settling measurement · owning chunk · **blast radius if wrong**. Plus: a scannable index table, a considered-and-excluded table (8 items with reasons), the reconciliation greps, and maintenance rules for future contributors (Phase E noise priors, F1 mechanism fakes) |
| Register citations in the tree | 18 headers + 3 test files *(modified, comments only)* | Every `PROVISIONAL (A4)` label upgraded to `PROVISIONAL (A4: HA-nn)`; every unlabeled harvest source (conversion binding contracts, tracking geometry, plant gains, loop-rate premise, `bootSettleTime`, `driftHorizon`, the brownout-survival premise, the current-limit gap, the no-fix-pose unknown) now cites its entry. Zero code changes — the suite byte-count of logic is untouched |
| **The ARM compile gate** | `.github/workflows/ci.yml` *(modified)* | New `arm-compile-gate` job: installs `gcc-arm-none-eabi`, **generates** the header list (`find include/shulib -name '*.hpp' \| sort` → one TU — a new header is covered the moment it exists; a hard-coded list would rot), cross-compiles under the exact strict flag set used locally (`-std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp`). The workflow comment states the honest scope: **compile** gate, not link/run — those need PROS's toolchain and the robot (R1/R3) |
| Phase A closed | `docs/planning/roadmap.md`, `docs/planning/build-order.md` *(modified)* | "You are here" → **Phase A COMPLETE, next C1 (`IMotion` + motion primitives)**; the A4 task entered under M2 with evidence and flipped `[x]`; the "not yet CI-guarded — closes at A4" notes flipped with evidence; the **no-Phase-B** note made explicit under the phase table (the lettering gap is the hardware-reversal's kept fossil, not a typo) |

**New tests: none, by design.** The brief: "A4 adds little or no test surface — that is expected;
do not pad it." Suite unchanged: **429 cases / 681,086 assertions / 3 deliberate skips** (two M3
acceptance stubs + the R3 GPS field-cal oracle — which is now register entry **HA-01**, so the
skip finally has a tracked owner and an unskip protocol).

---

## 2. The register, by the numbers

- **49 entries.** Confidence: **33 invented · 13 reasoned · 2 measured-elsewhere · 1 mixed**
  (HA-44: VEX-documented step shape, unmeasured onset). Two-thirds invented is the honest measure
  of what building without hardware costs — the brief predicted the list is *supposed* to be long.
- **By owner:** R3 × 19 · R4 × 25 · R5 × 3 · R6 × 2.
- **Coverage of the queues:** all **25** A3 §8 seed claims subsumed (each maps to one or more
  entries); A2's placeholder-gain and honest-limit declarations entered (HA-45..48); the two
  June-era conversion-layer flags entered (HA-01 with its skipped oracle, HA-02..HA-10 as the
  full binding-contract set); build-order's named seeds all present (PROS call latency HA-33,
  loop rate HA-32, cartridge/gearing HA-14/15, battery sag curve HA-40..42, field surface
  HA-39, F5 number-match HA-18).
- **Blast-radius findings worth surfacing** (the column the brief calls highest-value):
  - **Most entries are contained by design** — behind the `hal/pros` seam (HA-01..11: a wrong
    convention is a one-function or one-sign adapter fix that never touches the core) or behind a
    single config constant (most of R4/R5). Writing that containment down per-entry is the
    architectural payoff of F4/§7 made checkable.
  - **The exception is HA-19** (brownout kills motors first; the CPU survives): the F2
    guaranteed-park design — §14's non-negotiable, plausibly the highest-expected-value code in
    the library — *presupposes* it. If wrong, that is a design rethink, not a constant. It is
    flagged as the largest single blast radius in the register and must be settled at R3 before
    F2's guarantee is trusted on a field.
  - **HA-20** (IMU drift ≤ 1°/min) is the F2 ceiling itself — zero margin at the bound by
    construction, per A3's headline measurement.
- **Nothing was resolved on the host, and nothing was found host-settleable.** Per the brief's
  rule, every entry was checked for host-settleability during harvest; no entry turned out to be
  a disguised host question (the closest call, HA-18's arithmetic determinism, still requires a
  V5 to execute — the ARM gate compiles it, only the robot runs it).

## 3. Bidirectional reconciliation — verified, both directions, zero orphans

Direction 1 — **every label names its entry** (must print nothing, and did):

```text
$ grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"
(no output — exit 1)          # 35 label sites total, all carrying register IDs
```

Direction 2 — **every header-sourced entry points back**: a script mapped all 46 header-sourced
entries to their cited files (56 file citations) and grepped each file for its ID:

```text
Direction 2 CLEAN: all 46 header-sourced entries back-referenced (56 file citations verified).
Register entry presence check: none missing — 49/49 present.
In-tree IDs without a register entry: none.  Distinct in-tree IDs: 46.
```

The three entries without in-tree sources — **HA-18** (roadmap's F5 on-V5 clause), **HA-33**
(build-order's PROS-call-latency seed; no adapter code exists until R1), **HA-47** (A3-COMPLETED
§3.7's modeling insight) — are exempt from direction 2 and say so in their Source fields.
46 in-tree + 3 exempt = 49. ∎

## 4. The ARM gate — and the proof it works

Local run of the exact CI commands (77 headers in the generated TU):

```text
$ find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort \
    | awk '{print "#include \""$0"\""}' > all_headers.cpp ; echo 'int main(){return 0;}' >> all_headers.cpp
TU includes 77 headers
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
    -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c all_headers.cpp -o /dev/null -Iinclude
ARM COMPILE: CLEAN                    # arm-none-eabi-g++ 13.2.1
```

**Gate proof (the brief: "an untested gate is a comment") — executed, observed, restored:**

| Step | Action | Observed |
|---|---|---|
| 1 | Inject `#include <immintrin.h>` (x86-only intrinsics header) into `include/shulib/sim/rng.hpp` | — |
| 2 | Full **host** build | **GREEN** — the host toolchain has the header; host CI alone is provably blind to this defect class. This is the observation that justifies the gate's existence |
| 3 | The exact ARM gate commands | **RED** — `fatal error: immintrin.h: No such file or directory`, pointing at the injected line; **exit 1** (the job fails) |
| 4 | Restore; re-run gate + full suite | ARM CLEAN (77 headers); suite green 429 / 681,086; `git diff` on `rng.hpp` empty |

Honest caveat: the gate was proven by running its exact commands **locally** — the GitHub Actions
runner itself was not exercised (nothing was pushed; the working tree is uncommitted by
instruction). The YAML was parse-validated (`jobs: [host-tests, arm-compile-gate]`); first push
verifies Actions execution, same as every prior CI change in this repo (the M0 record notes the
identical protocol).

## 5. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test -j && ./build/test/shulib_tests
[doctest] test cases:    429 |    429 passed | 0 failed | 3 skipped
[doctest] assertions: 681086 | 681086 passed | 0 failed |
[doctest] Status: SUCCESS!
```

```text
$ <the ci.yml PROS-free guard grep>      GUARD 1 PASS: core is PROS-free
$ <the ci.yml layering guard grep>       GUARD 2 PASS: layering holds, core is sim-free
$ <the new ARM gate, locally>            ARM COMPILE: CLEAN (77 headers)
```

Suite counts are IDENTICAL to the A3 baseline — required, since A4's tree changes are comments,
docs, and CI only. Nothing committed (`git status`: modified headers/tests/docs + new register,
progress log, this record — all staged for review, no commits made).

## 6. Decision log (every choice with a viable alternative)

### D1 — Entries grouped by OWNING CHUNK, not by sensor family
The register's stated consumer is Phase R walking a checklist: R3 gets a bench-ordered day-one
list, R4 a characterization list. Grouping by sensor (all-IMU, all-GPS) reads better as an
inventory but would make R3 skip around the document with a robot on the bench. Rejected: a flat
ID-ordered table (no walk order at all).

### D2 — Checkbox-block entry format + an index table, not one wide table
The brief's suggested columns are all present per entry, but blast-radius text is the payload and
does not survive a 7-column row (A3's §8 proved even one-line-per-claim gets cramped). Literal
`- [ ]` checkboxes make settling an entry a visible tick — the register doubles as R3's runbook.
The index table up front restores at-a-glance scanning. Rejected: pure prose (the brief forbids
it), pure table (unreadable blast radii).

### D3 — Label format `PROVISIONAL (A4: HA-nn)`, keeping the `PROVISIONAL (A4` prefix
Preserves every existing grep habit (A3's convention stays greppable as one prefix) AND makes the
orphan check mechanical: `grep "PROVISIONAL (A4" | grep -v "HA-"` must be empty forever — a
label added without an ID self-reports. Rejected: replacing the label with a bare `HA-nn` (loses
the provisional semantics at a glance); a separate manifest file (a second source of truth to
drift).

### D4 — 49 entries, not 25: the conversion contracts and premises are entries too
The A3 queue was magnitudes; the brief's harvest list also names the June-era conversion
contracts, geometry, gains, and loop assumptions. Those are *assertions* rather than numbers, but
they are exactly the claims R3 must verify, they are falsifiable as stated, and the GPS axis
assumption (HA-01) — an assertion, not a number — is the register's founding member. Where
inclusion was debatable, the brief's rule applied: "if you are unsure whether something is an
assumption, enter it" (HA-36 driftHorizon, HA-39 surface uniformity, HA-49 the unmodeled-limit
no-consequence claim). The counter-pressure got its own section: **considered-and-excluded**, 8
items with reasons, so exhaustive stays distinguishable from bloated.

### D5 — The ARM gate is a separate parallel CI job
Keeps host feedback latency unchanged, isolates the cross-toolchain install, and gives the gate
its own named check in the PR UI (a red "arm-compile-gate" is self-explaining). Rejected: a step
appended to `host-tests` (serializes an unrelated ~40 s install into the hot path; one red check
name for two unrelated failures).

### D6 — Gate-proof mutation: an x86-only *header*, not a host-only *API use*
`#include <immintrin.h>` fails at the ARM compiler's front door with an unmissable message, and —
the load-bearing half of the proof — the **host build stays green**, demonstrating the exact
blindness the gate exists to close. A `std::filesystem`/`<thread>` use might also fail on newlib
but non-obviously (some hosted facilities partially exist), muddying what was proven. The
mutation site (`rng.hpp`, transitively included by every sim header) maximized the blast surface
of the failure.

### D7 — `LC_ALL=C sort` in the generated TU
Include order must not depend on the runner's locale — a TU that reorders between runs is a
nondeterministic gate. (Header order *shouldn't* matter for self-contained headers — and A1's
convention keeps them self-contained — but the gate should not be the thing that discovers an
order sensitivity nondeterministically.)

### D8 — Line numbers in Source fields, with the HA tag as the durable anchor
The brief asks for file:line; lines rot under edits. Both: file:line as of A4 close for
precision, plus the in-file `HA-nn` tag as the greppable anchor the register's preamble points
to. Rejected: line-free citations (spec says file/line); rejected: relying on lines alone.

## 7. DoD checklist (brief §Definition of Done)

- [x] **`docs/planning/hardware-assumptions.md` exists; every current assumption entered as a
  falsifiable claim with source, confidence, settling method, owning chunk, and blast radius** —
  49 entries; harvest swept all 35 label sites, both completion records' honest-limits sections,
  the conversion headers, geometry, gains, loop-rate, and build-order's seed list; the
  considered-and-excluded table records what was left out and why.
- [x] **Bidirectional reconciliation complete, zero orphans, verified by grep and stated** — §3
  above: direction 1 empty; direction 2 script-verified 46/46 header-sourced entries (56
  citations); 3 non-header entries exempt and stated in-register.
- [x] **Headers updated to cite register IDs** — 35 label sites now `PROVISIONAL (A4: HA-nn)`;
  18 headers + 3 test files carry citations (comments only; logic untouched, suite counts
  identical).
- [x] **CI cross-compiles all v2 headers for ARM with a GENERATED header list, fails on error** —
  `arm-compile-gate` job; `find | sort | awk` generates the TU in-job; `-Werror` + compiler
  failure fails the job; scope comment explains compile-vs-link/run honestly.
- [x] **The gate is proven to work** — §4: host GREEN / gate RED (exit 1) on an injected
  x86-only construct, then restored and re-verified clean. *(Caveat, stated: proven via the exact
  commands locally; the Actions runner runs it on first push.)*
- [x] **Roadmap + build-order updated: Phase A closed, Phase C (C1) next; ARM-gate item flipped
  with cited evidence** — both "you are here" pointers moved; the A4 task `[x]` with evidence;
  no-Phase-B noted explicitly in build-order (and echoed in roadmap's pointer).
- [x] **Full suite green; both existing CI guards still pass** — §5: 429 / 681,086 / 3 skips;
  GUARD 1 PASS; GUARD 2 PASS.

## 8. Freeze Register note (documentation contract #6)

**No freeze.** A4 froze nothing and touched no frozen contract's semantics (all header changes
are comments). One registry-adjacent note: the register itself creates a *soft* convention —
`PROVISIONAL (A4: HA-nn)` — which future chunks must follow when inventing magnitudes
(documented in the register's Maintenance section). It is a documentation convention, not an API
contract; it is deliberately not a Freeze Register row.

## 9. Documentation contract discharge (all six)

1. **Roadmap checkbox flipped with cited evidence** — the A4 item under M2 (`[x]`, evidence
   inline); the M2 acceptance `[~]` now cross-references HA-20.
2. **"You are here" updated** — roadmap and build-order both point at Phase A complete / C1 next.
3. **Design notes, why-not-just-what** — the register's preamble (what it is, how to read it,
   rules), the per-entry blast-radius reasoning, and the workflow's scope comment (why
   compile-only, why generated).
4. **Test evidence recorded** — §5 (no new surface, by design, with the brief cited); the gate
   proof in §4 is this chunk's mutation record.
5. **Decisions recorded** — §6, eight decisions with rejected alternatives.
6. **Freeze Register** — §8, no freeze, with the soft-convention note.

---

## 10. Phase A retrospective — what the substitute for hardware bought

**What it set out to build** (build-order's Phase A preamble): with no robot and none coming for
a while, the four chunks *are* the validation infrastructure for everything downstream — make the
system observable (A1), closed-loop-testable (A2), hostile rather than agreeable (A3), and honest
about what remains unproven (A4).

**What it actually produced:** a per-tick diagnostics spine with byte-pinned terminal output,
compile-time TRACE stripping, and crash-proof fault discipline (A1: 3 inherited logger defects
designed out structurally; 7 mutations red). A host plant that closes the loop from commanded
voltage to synthesized sensors with ground truth exposed only to assertions, provably-independent
truth integration, and seeded byte-identical replay (A2: 48 cases / 25,320 assertions; 8
mutations red). Nine populated degradation seams modeling how V5 hardware actually lies,
composable and reproducible, plus a HealthMonitor and a 9-attack survival matrix under which
every pathology raises a fault with a finite pose (A3: 80 cases / 133,643 assertions; 7 mutations
red). And now the register + the CI gate that keep both the debt and the portability visible
(A4). Net: **429 cases / 681,086 assertions**, 77 ARM-clean headers, three CI enforcement points
(PROS-free, sim-layering, ARM compile).

**What it FOUND — the evidence the approach paid for itself:** A3's hostile world surfaced
**three real defects in the Localizer** that agreeable fakes had certified for months — boot-window
poisoning (10.82″ of permanent phantom translation on a *stationary* robot, cut to <0.05″),
the ready-flag-outruns-the-data-path leak (3.65″, reachable **only** through composed hostility —
the reducibility design paying off), and mid-run IMU loss misreported as `Uninitialized` (a
skills gate would have applied the wrong recovery in both directions). Plus a detection hole
recorded honestly (the frozen-encoder blindness, owners named), two physics surprises pinned as
tests (the stop-drain phantom; sag-only-bites-at-the-ceiling), and A2's independent numerical
find (the naive SE(2) integral's cancellation floor, confirming arcStep's design note). Every
one of these would otherwise have been discovered on a field, mid-season, as a mystery.

**What Phase A still cannot tell us — stated without flinching:** whether any invented magnitude
resembles reality (49 register entries say exactly which ones and who finds out); whether the
GPS axes are what we guessed (HA-01 — a mirror, if wrong); whether a loaded V5 sustains the loop
rate every per-tick argument assumes (HA-32/33); whether the brownout premise under the
guaranteed-park design holds (HA-19 — the register's largest blast radius); and whether code
that *compiles* for the V5 *runs* on one — the gate holds compilability, and only R1/R3 close
the rest. The honest summary of the phase: **logic is proven; constants are inventoried guesses;
and the boundary between those two is now a document, not a feeling.**

---

## 11. Deliberately left for later chunks

- **Settling anything in the register** → R3/R4/R5/R6 own every entry; the register's
  Maintenance section defines the settle protocol (measure, check the box, record the value,
  update the constant, cite the log).
- **Register growth** → Phase E (EKF noise priors) and F1 (mechanism fakes) are the named next
  contributors; the labeling convention is standing.
- **The Actions-runner execution of the new job** → first push (nothing committed, per
  instruction).
- **Broadening the PROS-free guard to all of `shulib/`** → C7, after the legacy cutover, as
  scoped since M0.
