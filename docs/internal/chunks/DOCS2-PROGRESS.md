# DOCS2 — live progress log

> Appended continuously, in real time. This is the honest record of exactly how far the work got,
> and it is what makes an interrupted chunk recoverable. Brief:
> [`DOCS2-full-api-reference.md`](DOCS2-full-api-reference.md). Predecessor: DOCS1.
>
> **Nothing is committed by this chunk.** Verification output is pasted verbatim, including
> failures.

---

## 2026-08-14 — session start

Created this log first, before reading anything else, per the chunk protocol.

**Required reading, done:**

- `docs/internal/RESUMING.md`
- `docs/internal/PROJECT-BRIEFING.md` (in full)
- `docs/internal/chunks/DOCS2-full-api-reference.md` (the brief)
- `docs/internal/chunks/DOCS1-COMPLETED.md` (DOCS2 exists as a consequence of it)

**Inherited from DOCS1, restated so this log stands alone:**

- The measured debt: 971 public items / 71 headers / **399 undocumented**, taken with
  `api_doc_tool.parse_header`, not estimated. Re-verified below rather than re-estimated.
- Four headers the generator **cannot parse at all** (public nested types); it refuses loudly
  rather than dropping members.
- The `TARGETS`-vs-coverage tension: four Freeze Register rows (F11–F14) each say in writing that
  their seam is "deliberately not in the api-doc coverage TARGETS until it freezes".
- The sentence most likely to go wrong again: `docs/api/README.md`'s "what is not here" paragraph.
  DOCS1 got it wrong once already (claimed the page "covers the surfaces that are frozen" — false,
  seven contracts are LOCKED and two are on the page).

**Standing constraints for this chunk (restated inline, not by reference):**

1. **Do not fix code.** An API defect found while documenting is a *finding to report*, not an
   edit to make. List them all.
2. **No filler.** `/// Sets the voltage.` on `setVoltage` satisfies the gate and teaches nothing —
   it converts a visible gap into a hidden one. The gate counts comments, not meaning.
3. **`docs/api/*.md` are generated.** Never hand-edited. Fix the header's `///`, regenerate.
4. **A page missing from `mkdocs.yml` nav does not fail the strict build** — it ships unreachable,
   silently. Every new page gets a nav entry by hand.
5. **Do not commit. Do not push.**

---
## Step 1 — re-verification of the measured scope, and what it found

**The brief's numbers reproduce exactly.** Re-run with `api_doc_tool.parse_header` over
`include/shulib`, excluding `sim/` and `**/fake/`:

```
subsystem         hdrs   items   undoc
hal                 41     238     151
control              6      63      54
motion              11     127      47
localization        13     143      43
diag                18     185      28
math                 4      33      26
chassis              3      98      18
units                2      14      13
kinematics           7      26      10
manipulation         3      25       8
sequence             1      19       1
TOTAL              112     971     399     (71 headers yield >=1 type — the brief's "71")
UNPARSEABLE: 4  (the four nested-type headers, exactly as briefed)
```

**But the instrument is miscalibrated, and this is the chunk's first real finding.**

### FINDING T1 — the parser is blind to 59 top-level public types, silently

`parse_header`'s type opener is

```python
re.match(r"^(class|struct|enum class)\s+([A-Za-z_]\w*)\s*(final)?\s*\{", stripped)
```

It requires `{` immediately after the (optional) `final`. So **every type with a base-class
list, and every enum with an explicit underlying type, is invisible** — not dropped with a
warning, not reported: never seen at all. Measured, 59 top-level definitions the parser never
sees, including:

- `enum class FaultCode : std::uint16_t` — the eleven fault codes
- `enum class Frame : std::uint8_t`, `enum class MotionState`, `enum class MotionOutcome`,
  `enum class TickPhase`, `enum class GateReason`, `enum class MechanismOutcome`,
  `enum class TagProvenance`, `enum class FrameType`, `enum class ReadStatus`
- `class Localizer final : public IPoseSource`, `EkfFusion`, `ComplementaryFusion`,
  `GpsCorrector`, `AprilTagCorrector`, `NullCorrector`
- `class MoveToPose : public IMotion`, `TurnTo`, `StrafeTo`, `HoldPose`, `DriveBrake`,
  `RunReporter`, `CommandIdStampSink`, `MotionStatsSink`
- `class RunGuard final : public motion::ITickPacer`
- `MatrixKinematics`, `TankKinematics`, `MotorMechanism`, `PneumaticMechanism`,
  `IMechanismOp`, `RunUntilConfirmed`, `ActuateAndConfirm`
- `TermSink`, `SdSink`, `NullSink`, `LevelFilterSink`, `RateLimitedSink`
- **all fourteen `hal/pros` adapters**
- `struct PreconditionError : std::logic_error`

That is close to **every concrete implementation class in the library.** The blindness never
fired before because `Chassis` and `Routine` — the only two TARGETS — have no base list.

### FINDING T2 — namespace-scope declarations are invisible too

The parser only walks *type bodies*. Nothing at namespace scope is seen:

| Shape | Count | Examples |
|---|---:|---|
| Free functions | 85 | `arcStep`, `desaturateUniform`, `xDrive`, `hDrive`, `fieldToRobot`, `robotToField`, `compensateForBattery`, every `*_conversion` helper, every `operator""_in`-class literal |
| Namespace-scope constants | 32 | `kMaxMotorVoltage`, all five `spec/accuracy.hpp` targets, `version.hpp`'s `kApiMajor`/`kApiMinor`/`kApiVersionString` |
| `using` type aliases | 11 | `units::Length`, `Time`, `Voltage`, `Velocity`, … — the whole F3 units vocabulary |

`spec/accuracy.hpp` and `version.hpp` parse to **zero items each** today.

### What T1 + T2 mean together

Three of the seven LOCKED contracts are **entirely invisible to the generator**: F1 (coordinate
frame — `enum class Frame : std::uint8_t` plus its two conversion functions), F2 (accuracy
targets — five namespace-scope constants), F3 (units and angle semantics — eleven `using`
aliases plus a template class whose head is dropped).

So the answer to the team lead's question — *why does a fourteen-subsystem library document two
types?* — is larger than "the generator was never pointed anywhere else." **It could not have
been pointed at most of them.** The brief's 971/399 is not the size of the debt; it is the size
of the part of the debt the broken instrument could see.

**The honest consequence: the true scope is bigger than the brief states, and cannot be known
until the parser is fixed.** Fixing it is therefore step one, exactly as the brief says — just
for a bigger reason than the brief knew.

---

## Step 2 — the ruling on `TARGETS` vs coverage

**RULED: shape (1) — generate and gate everything**, with two mechanical additions without which
shape (1) would be unsafe. Freeze Register rows F11–F14 are therefore **amended in the register**,
not silently contradicted.

### Why not shape (2), the shape the brief said to beat

Shape (2) is "generate pages for everything, gate only the frozen contracts". It looked strongest
before measuring, and three things sank it:

1. **Its claimed saving is mostly illusory.** The friction people imagine when they hear "gated"
   is *regeneration* friction — change a member, the build goes red until you re-run the tool.
   But `check-fresh` compares **the whole of `docs/api/` byte-for-byte against a fresh run**, so
   that friction lands on every *generated* header regardless of gating. The only thing gating
   adds on top is "a new public member must carry a `///` before the build is green" — which is
   already this project's written standard (`RESUMING.md`: *documentation is a deliverable, not
   an afterthought*). Shape (2) pays a tool change to avoid a rule the project already keeps.

2. **It builds the exact structure this repo has already been bitten by.** DOCS1's central lesson,
   in its own words: *"the hole was not in either list. It was in the gap between two lists that
   were maintained by hand and believed to agree."* Shape (2) creates a generate-list and a
   gate-list, by hand, and asks a future chunk to remember to move a header from one to the other
   on the day its contract freezes. Nothing enforces that. F11 freezes at F3; if nobody moves it,
   a locked contract sits ungated and the register says otherwise.

3. **It makes the reference a two-tier document whose second tier can rot to empty.** An ungated
   page renders `**UNDOCUMENTED.**` per member — honest at generation time, and no floor at all.
   A page that is 100% "UNDOCUMENTED" is generated, fresh, green, and worthless.

### Why not shape (3), frozen contracts only

It leaves the largest user-facing gap (`diag/`) unfixed, which is what the team lead called this
chunk to fix. It is also now *narrower than it sounds*: findings T1/T2 above show the generator
cannot currently see F1, F2 or F3 at all, so "frozen contracts only" would still require the
entire parser rewrite for a fraction of the payoff.

### Why the register's own argument does not survive contact

F11–F14 say the seams are *"deliberately not in the api-doc coverage TARGETS until it freezes."*
Read together, the four rows are one ruling cited three times: F12 gives its reason as "the F11
precedent", F13 as "the F11/F12 precedent", F14 as "the F11/F12/F13 precedent". Only F11 states a
reason at all, and it is "for the same not-frozen reason". Four recorded rulings, one argument.

That argument conflates two mechanisms that are not alike in this repo:

| | changing a **frozen** signature | changing a **gated** seam |
|---|---|---|
| What fires | `test/f6_signature_pin_test.cpp` / `routine_signature_pin_test.cpp`, at compile time, naming the row and the member | `check-coverage` (only if the member is *new* and has no `///`) and `check-fresh` |
| What it costs | a major API-version bump plus a migration note — *a breaking change to argue* | one `///` edit and `python3 tools/api_doc_tool.py generate` |

The brief's counter-argument is correct: **that is friction, not a freeze.** The doc gate has no
opinion about a member's shape, only about whether it is explained.

### The two additions that make shape (1) safe

Shape (1) as literally stated ("add all 115 headers to TARGETS") would import two rot risks. Both
are closed here rather than accepted:

- **A hand-maintained 115-entry `TARGETS` is the thing that rots.** A header added at R3 and
  forgotten is invisible to *both* the generator and the gate — the tool's own docstring calls
  that failure ("silently absent, and the reference still looks complete") worse than a stale
  document. So `TARGETS` becomes **derived from the tree by glob**, with a short, commented
  exclusion list. This is the repo's own precedent, stated in this very file for the example
  scan: *"a gate you must remember to update when adding a file is a gate that rots."*

- **A generated page missing from `mkdocs.yml`'s nav ships unreachable and silent** (measured:
  INFO, exit 0 — see the verification section). With ~115 pages a hand-maintained nav is
  guaranteed to drift. So the API section of the nav is **generated into `mkdocs.yml` between
  markers and verified by `check-fresh`**, the same shape `briefing_status.py` already uses for
  the briefing block. The failure mode the brief calls "the one this project fears most" becomes
  structurally impossible instead of gate-dependent.

### Rejected sub-alternatives, recorded

- **Flattening `Outer::Inner` into the parent's member list** (the obvious shortcut for nested
  types). Rejected: it renders `Frame` as if it were a peer of `BlackboxReader::open()`, and
  `Frame`'s own four members vanish entirely — which is precisely the failure the tool's error
  message exists to prevent. Nested types get their own section, their own anchors, and their own
  indented place in the contents and the alphabetical index.
- **One page per subsystem** (11–14 pages, a small nav). Rejected on measurement: `hal/` alone is
  41 headers and ~240 items; at the current page's density that is a ~200 KB single page.
- **Mirroring the include tree under `docs/api/`** (`docs/api/hal/pros/motor.md`). Rejected: it
  puts pages at three different depths, so every `../../include/...` and `../guide/...` link in
  the renderer becomes depth-dependent, and it renames `docs/api/chassis.md` and
  `docs/api/routine.md` — which are named by `test/api_reference_fidelity_test.cpp`,
  `test/CMakeLists.txt`, four public documents, and `docs/internal/verify/verify-d3.sh` (the
  **reviewer's** harness, which a chunk must not rewrite). Flat naming keeps every page at depth
  1 and every existing link correct.

### The page-naming rule, which follows from that

A page is named after its header, with any path *below the subsystem* folded in with `-`; the
subsystem itself is carried by the nav, not the filename.

```
chassis/chassis.hpp   -> docs/api/chassis.md       (unchanged)
chassis/routine.hpp   -> docs/api/routine.md       (unchanged)
hal/motor.hpp         -> docs/api/motor.md
hal/pros/motor.hpp    -> docs/api/pros-motor.md
version.hpp           -> docs/api/version.md
```

Uniqueness is **asserted by the tool**, not assumed: two headers mapping to one page is a loud
failure. (Measured, the only basename collisions in the tree are `hal/X.hpp` vs
`hal/pros/X.hpp`, which this rule separates.)

---

## Step 3 — the tool extension, and what an independent oracle found in it

`tools/api_doc_tool.py` rewritten below the rendering layer. **All 115 headers now parse; 0 refuse.**

**Re-measured with the fixed parser: 1,625 public entities, 631 undocumented.** The brief's
971/399 was the visible fraction. Every number in this chunk from here on is the 1,625 one.

| | brief (old parser) | measured (fixed parser) |
|---|---:|---:|
| Headers with public entities | 71 | 115 |
| Public entities | 971 | **1,625** |
| Undocumented | 399 | **631** |
| Headers that would not parse | 4 | **0** |

### What changed in the tool

1. **`TYPE_OPENER`** now accepts a base-class list, an enum's underlying type, `final`, and a
   `template` head — the three shapes that hid 59 definitions.
2. **Namespace scope is walked**, so free functions, constants and `using` aliases are entities.
   `namespace detail` is excluded (the only name-based exclusion, stated with its reason).
3. **Nested public types recurse** into their own `TypeDecl` with a `qualified` name, their own
   section, their own anchors, their own contents indentation and their own index rows. Private
   and protected nested types are walked (so their bodies are skipped correctly) and recorded
   nowhere.
4. **`_parse_enum_body` rewritten.** The old one computed brace depth per line, so a ONE-LINE enum
   (`enum class Role { Forward, Lateral };`) returned **zero enumerators** — a vacuously green
   coverage gate, inside the tool whose purpose is to prevent exactly that. Two exist in the tree.
5. **`TARGETS` is now a glob** with a commented exclusion list, and the page set, the coverage set
   and the nav are all derived from it.
6. **The mkdocs nav is generated** into `mkdocs.yml` between markers and checked by `check-fresh`.
7. **`check-coverage` takes an optional header** to narrow the *report* (never the gate).

### Three latent parser bugs fixed on the way, each of which only bites outside the old two targets

- **`_skip_body` mis-read a constructor initializer list containing a braced value.** It counted
  braces textually and stopped at the first balance, so `: MoveToPose(deps, math::Pose2d{}, …)`
  ended "the body" on its first line and parsing resumed mid-expression.
  `motion/hold_pose.hpp` and `motion/strafe_to.hpp` failed to parse at all on this.
- **`_find_terminator` did not know character literals**, and tracked brace depth only at depth 0.
- **Operator punctuation collapsed to nothing in anchors**, so `operator+`, `operator+=` and
  `operator==` on one type shared an anchor, were numbered against each other, and were then
  *labelled* "(overload 2)". Not a broken link — **a confident, wrong sentence about the API.**

### The independent oracle, and why it earned its place

A hand-rolled parser checked against its author's own expectations is the shared-model trap. So
the parse was diffed against **clang's real AST** (`clang++ -Xclang -ast-dump=json`, a front end
sharing no code, no regex and no assumption with the tool), entity by entity, over all 115
headers.

**It found four bugs my own self-test did not**, all of the same family — a FIELD mislabelled as
a function and then named after part of its own initializer:

| Declaration | Tool called it |
|---|---|
| `double integralLimit = std::numeric_limits<double>::infinity();` | `AxisGains::std::numeric_limits<double>::infinity` |
| `std::uint32_t abortFaultMask = faultBit(FaultCode::OdoStuck);` | `MotionSchedulerConfig::faultBit` |
| `units::AngularVelocity headingDriftRate{(1.0/60.0) * kPi/180.0};` | `EkfFusionConfig::headingDriftRate{` |
| `bool operator()(...) const` | `AlwaysConfirmed::operator` |

The first two have a `(` at depth zero *after* an `=`; the third has one inside a brace
initializer, which was not tracked; the fourth is the one case where the name contains the `(`
that every other rule keys on. All four now fixed, and all four pinned in the self-test.

### The removability gate fired, exactly as the brief predicted

Three header banners cited the development log and reached `docs/api/` verbatim, because the
generator reproduces banners in full:

```
docs/api/ekf_fusion.md:575    (build-order: "the simpler filter is easier to get right…")
docs/api/ekf_fusion.md:729    build-order.md asks for a consecutive-reject re-init; …
docs/api/odo_stall_check.md:184   speed (A3-COMPLETED §3.4, asserted by test).
```

All three rewritten in the headers to the neutral form ("the development plan", "measured during
the hostile-fakes campaign"). This is documentation, not behaviour.

### Page layout, decided and recorded

One page per header, flat under `docs/api/`, named after the header with any path *below* the
subsystem folded in (`hal/pros/motor.hpp` → `pros-motor.md`); uniqueness asserted by the tool.
`docs/api/chassis.md` and `docs/api/routine.md` keep their names, so
`test/api_reference_fidelity_test.cpp`, `test/CMakeLists.txt`, four public documents and D3's
reviewer harness all keep working. The alphabetical index moved to its own page
(`docs/api/all-entities.md`): as a section of the overview it was 180 KB of that page's 190 KB
and buried the scope paragraph under 1,625 table rows.

---

## Step 4 — the site has to stay READABLE, not just complete (team lead, mid-chunk)

> *"I really would like to make docs.shurobotics.com look really nice and easy to navigate…
> if things are easily readable and separated people won't feel overwhelmed with the
> documentation."*

That lands directly on this chunk, because this chunk is what makes the site big: **2 API pages
became 117 in one commit.** A reference that renders as one 117-item sidebar teaches a reader
that the library is complicated, which is the opposite of what it is for.

`mkdocs` was not installed on this machine, so the site had never been built locally. Installed
into a scratch venv at the **exact versions CI pins** (`mkdocs==1.6.1`,
`mkdocs-material==9.5.44`, from `.github/workflows/pages.yml`) so what is verified here is what
publishes.

### The nav gotcha, re-measured rather than taken on trust

Removed one page from the nav, left the file in place, built strict:

```
INFO    -  The following pages exist in the docs directory, but are not included in the "nav" configuration:
  - api/tracking_wheel.md
REAL EXIT CODE = 0
PAGE SHIPPED ANYWAY (unreachable from nav)
```

Confirmed. **INFO, exit 0, and the page ships where nobody can reach it.** That is why the nav is
generated from the same target list as the pages and compared byte-for-byte by `check-fresh`
rather than maintained by hand — with 117 pages, hand-maintenance is not a risk, it is a
schedule.

### What changed for readability

- **`navigation.tabs` (+ sticky).** Guide / Cookbook / API / Project become tabs, so the sidebar
  shows one area at a time. The 117-page API tree does not exist until you ask for it.
- **`navigation.sections` REMOVED.** With it, all fifteen subsystem groups render expanded, always.
  Without it, groups collapse and only the one you are reading opens — 17 sidebar items instead
  of 130.
- **`navigation.indexes`.** Clicking "API reference" (or "User guide", or "Cookbook") now lands on
  that section's overview instead of doing nothing.
- **The design commentary folds.** Every page reproduces its header's banner in full — that is the
  *why*, and it is the most valuable prose here — but on the longest headers it is 200 lines, and
  a wall of it under every page is how a reference stops being read. It is now a `<details>`
  block that starts **open under 45 lines and closed above it**, with the line count in the
  summary so nothing is hidden, only folded. Plain `<details>` + `md_in_html` rather than a
  Material admonition, because these documents are read on GitHub too and `??? note` renders
  there as literal question marks.
- **The overview leads with the short answer**, before it ever mentions its own size: *writing a
  routine? you need two of these pages* — `Chassis` and `Routine` — *everything else is the
  machinery underneath, and safe to ignore until you want it.*
- **Every page opens with a one-line census** ("declares **3** types (28 members) and **2** free
  functions"), so a reader can tell a two-constant header from a sixty-member scheduler at a
  glance.
- **The A–Z index is split by letter with a jump bar.** 1,625 rows in one table is a scroll, not
  an index.
- **Nav labels are spelled the way the code is**: `ICorrector`, `IFusionPolicy`, `PID`, `GPS`,
  `AprilTag corrector`, `SD sink`, `Motor (PROS)` — not `I corrector` / `Pid` / `Gps`. Derived
  from a small token table with plain capitalisation as the fallback, so a new header needs no
  edit to get a sane label.
- **A palette** (indigo, both schemes) so the site has a colour rather than defaulting to
  black-on-white.

### Verified, not assumed

```
$ mkdocs build --strict
INFO    -  Documentation built in 3.11 seconds
MKDOCS EXIT=0

147 pages built · 7 nav tabs · <details> renders as real HTML with its markdown processed
· "API reference" resolves to api/ as a section index · A–Z letter anchors present
```

---

## Step 5 — mutation campaign on the tool, and the three holes it found

Run in an **isolated copy** of the repo (`include/` symlinked, tool copied) so it could not
disturb the fourteen documentation agents working in the live tree. Break it, run it, observe
red, restore — sixteen mutations, each aimed at a property this chunk added or fixed.

**First pass: 13/16 RED, and the three GREEN ones were the valuable result.**

| Survivor | Why the self-test could not see it |
|---|---|
| **M3** — `_find_terminator` forgets character literals | The fixture's only char-literal constant was `= {';', '{', '}'}`, and those three literals **balance**. A parser blind to them lands in the right place *by luck*, so the mutation survived. |
| **M10** — a top-level `=` no longer ends the function search | No fixture field had a **call** in its initializer. |
| **M11** — braces are not a depth in `_looks_like_function` | No fixture field had a **parenthesised brace initializer**. |

M10 and M11 are the same family clang had already caught in the shipped tree
(`AxisGains::integralLimit` rendering as `AxisGains::std::numeric_limits<double>::infinity`) —
so the self-test was blind to a bug that had actually happened. That is the strongest argument
for running the campaign at all.

Fixture extended with the three missing shapes: an *unbalanced* character literal
(`inline constexpr char kSemicolon = ';';` and `kOpenBrace = '{';`), a field initialized by a
call, and a field with a parenthesised brace initializer — with a real function beside them so
the rule cannot be satisfied by calling everything a field.

**Second pass: 16/16 RED.**

```
RED    M1  type opener forgets base lists / enum underlying types
RED    M2  nested public types flattened into the parent's member list
RED    M3  _find_terminator forgets character literals
RED    M4  _skip_body reverts to naive textual brace counting
RED    M5  operator punctuation not spelled out in anchors
RED    M6  `namespace detail` treated as public
RED    M7  protected members included as public
RED    M8  one-line enums yield no enumerators (the pre-DOCS2 behaviour)
RED    M9  `using` declarations excluded again
RED    M10 _looks_like_function: a top-level `=` no longer ends the search
RED    M11 _looks_like_function: braces are not a depth
RED    M12 _member_name loses the operator special case
RED    M13 page_name drops the path below the subsystem (pros/ collides)
RED    M14 render_nav omits the last page of every group
RED    M15 private nested types recorded as public
RED    M16 an EMPTY /// counts as documentation again (D3's H3 hole)

16/16 mutations RED.
```

M16 is D3's own hole probe, re-run — a regression guard on a fix three chunks old.

---

## Step 6 — the documentation pass, and the proof that no code moved

Fourteen parallel agents, one batch of headers each, balanced on undocumented count (~45 each).
Each batch was then read back by a **different** agent whose only job was to hunt filler and
factual error — the writer never reviews its own prose.

### "Do not fix code" — checked mechanically, not asserted

Every touched header's committed and working versions were run through `g++ -fpreprocessed -dD
-E` (which strips comments without expanding includes) and compared **as token sequences**:

```
headers changed: 97
LAYOUT-ONLY (identical token sequence) in 3:
    include/shulib/hal/controller.hpp
    include/shulib/hal/motor.hpp
    include/shulib/localization/tracking_wheel.hpp
NO CODE CHANGED — every touched header has a byte-identical TOKEN
sequence to HEAD. Every difference is a comment, or a line break.
```

**Not one code token appeared, vanished or moved in 97 headers.**

The three layout-only headers each had a **one-line enum** expanded to one enumerator per line —
`enum class BrakeMode { Coast, Brake, Hold };` becoming three lines. That is required, not
incidental: a `///` run above a one-line enum documents only its first enumerator and a `///<`
only its last, so on a one-liner the middle ones cannot be documented at all. The tool states
that rule and calls the resulting gate failure "the pressure to expand it, and intended". Same
enumerators, same order, same values.

---

## Step 7 — full verification, and a stale command in the briefing's own §7

**Coverage is green:**

```
$ python3 tools/api_doc_tool.py check-coverage
doc coverage: 1625 public entities across 115 headers, all documented
```

**The last nine were both one-line enums** — `hal::LogLevel` and `Localizer::Quality`. Neither
was an oversight: a `///` run above a one-line enum documents only its first enumerator, so both
had to be expanded to one enumerator per line first. Both are now documented per enumerator,
including the two facts a reader most needs and neither name carries: that `Trace` is
*compile-time strippable* rather than merely filtered, and that `Quality`'s declaration order is
**not** a ranking (`Degraded` is worse than `DeadReckon` and sorts after it).

**Byte-stability across two consecutive runs:**

```
$ python3 tools/api_doc_tool.py generate && python3 tools/api_doc_tool.py check-fresh
FRESH PASS 1
$ python3 tools/api_doc_tool.py generate && python3 tools/api_doc_tool.py check-fresh
FRESH PASS 2
```

**All four doc gates, the suite, both guards, the ARM gate and the release gate:**

```
api_doc_tool self-test: OK
doc coverage: 1625 public entities across 115 headers, all documented
doc example scan: 387 quoted lines, all verbatim (4 source files)
removability: no public doc references docs/internal/
doc staleness audit self-test: OK (10 detector/filter cases)
doc staleness audit: clean (26 live docs, 0 numeric claims checked)

[doctest] test cases:    1121 |    1121 passed | 0 failed | 3 skipped
[doctest] assertions: 1523344 | 1523344 passed | 0 failed |
[doctest] Status: SUCCESS!

GUARD1 PASS — library is PROS-free (all of include/shulib except hal/pros/)
GUARD2 PASS — layering holds: core is sim-free
headers in the generated TU: 148
ARM GATE PASS

site source prepared: 148 files, 1893 source links rewritten
  publish stamp: added to 147 pages
  internal docs: absent (verified)
  escaping links: none (verified)
PREPARE_SITE PASS
```

The suite is **1121** cases, one more than at chunk start: DOCS2 adds a fidelity case pinning the
four shapes the old parser could not see, so a regression shows up as a missing row in a rendered
page rather than as a smaller number nobody is watching.

### The briefing deadlock is real, and happened exactly as briefed

The first build died on the briefing gate, not on a compile — the doc gates run before the binary
is relinked, so `briefing_status.py` derived the suite state from the STALE binary and reported
`RED — 1 case(s) failing`. That one failure was the D3 fidelity test looking for the member index
on `docs/api/README.md`, which this chunk moved to its own page. Ran
`briefing_status.py generate` → build → suite green, per the documented dance.

### FINDING — the briefing's own §7 verification command has been failing on a clean tree

`PROJECT-BRIEFING.md` §7 prints:

```sh
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib && echo GUARD1-FAIL || echo "GUARD1 PASS"
```

Run as printed, that reports **GUARD1-FAIL with 21 hits on a clean tree**, because R1a added
`include/shulib/hal/pros/` — the one path that exists to include PROS. CI has the path-anchored
form and §4 of the briefing *describes* the amendment correctly; the copy-pasteable command in §7
was never updated. This is the same failure as DOCS1's broken ARM `sed`, in the same document,
one section apart: a session following the canonical protocol meets a red gate on a clean tree
and has to guess whether the tree or the instruction is wrong. Fixed in place.

---

## Step 8 — the adversarial review, and the bug it found INSIDE the tool

Fourteen reviewers, each reading back a batch it did not write. **They did not come back empty:**

| | |
|---|---:|
| Comments written by the writers | **504** (plus this session's own edits) |
| Comments the reviewers found **factually wrong** | **50** |
| Comments the reviewers called **filler** | **8** |
| Code changes found | **0** (four layout-only notes, all the enum expansions above) |
| **API defects reported by the writers** | **73** |

The 73 are written up in [`DOCS2-API-DEFECTS.md`](DOCS2-API-DEFECTS.md), each with its evidence,
**reported and not fixed** per landmine L3.

### FINDING T3 — the tool reproduced its own headline failure, in the published tree

A writer would not fix a comment it was told to fix, and said why. Its reasoning was right, and
the bug was mine:

```cpp
Succeeded = 1,    ///< completed AND confirmed (where the operation defines a
                  ///< confirmation; completed, where it does not)
Unconfirmed = 2,  ///< the operation ran to completion and the confirmation
                  ///< said the world did not change — healthy mechanism…
```

A `///<` **continuation** line also starts with `///`, so `_strip_doc` accepted it as a *leading*
comment, it landed in `pending`, and it became the documentation for the **next** enumerator.
Live in `docs/api/mechanism_outcome.md` at the time it was found:

```
### `MechanismOutcome::Unconfirmed`   →   "< confirmation; completed, where it does not)"
### `MechanismOutcome::TimedOut`      →   "…said the world did not change — healthy mechanism…"
```

Three enumerators each carried a confident sentence **about a different value**, `Stalled` lost
the half of its sentence that names the fault it raises, and **the coverage gate scored all of
them as documented** because the text was non-empty. That is this tool's own headline failure —
"the page looks complete" — committed inside the tool, and no gate could see it. A reader did.

Fixed: `_strip_doc` now refuses `///<`, and a standalone continuation is appended to the member
it trails. Pinned in the self-test for both shapes (enumerator and field), because the fixture
had neither.

### The independent oracle, final run — 0 divergences

```
$ python3 clang_oracle.py            # all 115 headers, clang -Xclang -ast-dump=json
--- include/shulib/core/check.hpp
    EXTRA in api_doc_tool:     PreconditionError::logic_error
--- include/shulib/diag/sd_sink.hpp
    EXTRA in api_doc_tool:     SdSinkBuffers

TOTAL missing (clang saw, tool did not): 0
TOTAL extra   (tool saw, clang did not): 2
```

**Nothing clang sees is missing from the reference.** Both "extra" rows are real public
declarations the ORACLE's normalisation misses, not phantoms: `using
std::logic_error::logic_error;` (a using-declaration clang models as a shadow decl) and
`template <std::size_t, std::size_t> struct SdSinkBuffers` (a class template the oracle's
template-name stripping drops). Verified by hand in both headers.

---

## Step 9 — the correction pass, and the two rejections that make it real

The 58 review findings (50 wrong + 8 filler) went back out to fourteen agents with one standing
instruction: **verify each finding against the source before acting on it — the reviewer can be
wrong.** This project has been bitten in both directions (a shipped battery-model lie, and a
red-team that manufactured a false all-clear by telling its verifiers to default to refuting).

```
CONFIRMED-AND-FIXED  60
PARTIAL               2
REJECTED              2
```

**Both rejections were correct**, and they are the reason the stage verified rather than obeyed.
Both were `control/trapezoid_profile.hpp`: the reviewer's diagnosis of the CODE was right, but
the sentences it quoted no longer existed — I had already corrected them by hand. The fixer
proved that by grep (`"instantiates one per axis"` now appears only inside a note explaining that
it was false), independently re-verified the underlying claim, and declined to edit a correct
comment. A fix stage that trusted its reviewer would have churned two good comments.

**Ten further API defects surfaced during the correction**, because a reviewer's objection sent
someone back to the code and the code turned out to be the thing at fault. Appended to
[`DOCS2-API-DEFECTS.md`](DOCS2-API-DEFECTS.md) as section E — **83 total**.

### One thing I had to fix in my own edits

The token-level no-code check went RED on two headers — **mine**, not an agent's. Expanding the
one-line `LogLevel` and `Localizer::Quality` enums, I had added a trailing comma
(`Degraded,` where the original had `Degraded`). Semantically nothing in C++, but it is a token,
and the claim being made is that not one token moved. Removed:

```
LAYOUT-ONLY (identical token sequence) in 5:
    include/shulib/hal/controller.hpp
    include/shulib/hal/motor.hpp
    include/shulib/hal/telemetry_sink.hpp
    include/shulib/localization/localizer.hpp
    include/shulib/localization/tracking_wheel.hpp
NO CODE CHANGED — every touched header has a byte-identical TOKEN
sequence to HEAD. Every difference is a comment, or a line break.
```

### Mutation campaign, re-run against the FINAL tool

```
16/16 mutations RED.
```

Plus **M17**, added for the `///<` continuation fix — reverting `_strip_doc` to accept `///<`
turns the self-test red on all four of the new assertions:

```
- a wrapped ///< stays with its own enumerator: {'Alpha': '…runs long enough that it',
                                                 'Beta': '< wraps onto a continuation line'}
- the NEXT enumerator does not inherit the continuation
- a wrapped ///< on a FIELD stays with it
- …and does not become the next field's documentation
```

That is the shipped bug reproduced exactly, and now caught.

---

## Final state

`DOCS2-COMPLETED.md` written. Briefing block regenerated (**25 of 43 chunks**, suite green,
no interrupted chunks) and re-checked after the rebuild it triggers. **Nothing committed,
nothing pushed.**

---

## Step 10 — the consistency sweep, and the state at pause

**READ THIS FIRST IF YOU ARE RESUMING.** The tree is **dirty and the chunk is COMPLETE.** That
combination is normally a contradiction here — `RESUMING.md` says a dirty tree means an
interrupted chunk — so it is spelled out:

- `DOCS2-COMPLETED.md` exists, so `briefing_status.py` correctly reports **no interrupted chunks**.
- Nothing is committed and nothing is pushed, because the chunk brief said not to.
- Every gate below is green. **There is no unfinished work in this chunk.** The only thing left is
  the reviewer's decision to commit.

### Documents DOCS2 made stale, and what was done about each

| Document | Was | Now |
|---|---|---|
| `README.md` (repo root) | "every public member of the `Chassis` facade and the `Routine` recipe layer" | one page per header + the A–Z index; points routine authors at the two pages they need |
| `docs/README.md` | "Every public member of the two frozen surfaces is documented" | every public entity in every shipped header, with the gated-≠-frozen distinction stated |
| `test/README.md` | "a public member of the frozen `Chassis` or `Routine` surface" | any public entity under `include/shulib/`, named with file and line |
| `docs/changelog.md` | — | a 2.1 entry: no API change, but the promise and the contributor obligation both changed |
| `docs/roadmap.md` — M7 item | `[~]` "nothing is published" | `[x]`, published, expanded; **false since DOCS1 and missed by it** |
| `docs/roadmap.md` — "you are here" | "the chunk in flight is DOCS1" | both documentation chunks done; the release is what remains |
| `docs/roadmap.md` — D3's record | "nothing is published… no Pages configuration" | left standing as history, with a superseded note appended rather than a rewrite (DOCS1's L2) |
| `docs/roadmap.md` — rows F11–F14 | "deliberately not in the api-doc coverage TARGETS" | **amended**, each with the reason the precedent was overturned |
| `PROJECT-BRIEFING.md` §4 | "Four build-time doc gates" over two headers | the same four, over the whole tree, with the glob and the generated nav explained |
| `PROJECT-BRIEFING.md` §5 | no DOCS entries | DOCS1 and DOCS2 added, plus the R1b entry that was also missing |
| `PROJECT-BRIEFING.md` §7 | a GUARD1 command that **fails on a clean tree** | path-anchored, matching CI — see the finding in step 7 |
| `PROJECT-BRIEFING.md` §9 | five traps | **trap 6 added: an instrument's silence read as a zero** |
| `PROJECT-BRIEFING.md` §12/§13 | "THE IMMEDIATE TASK — DOCS1", "DOCS1 in flight" | the release to `main`; §13 also carries the one DOCS2 result that changes what a measurement here is worth |
| `PROJECT-BRIEFING.md` §18 | HTTPS "still not issued (verified: issued)" — self-contradictory | one question rides with the release: whether to push |
| `docs/internal/docs-publishing.md` | **"Nothing is published. There is no website"** | corrected, with the regeneration table rewritten: adding a header now requires *nothing*, because the target list is a glob |
| `docs/internal/guide-maintenance.md` | "a `///` comment in `chassis.hpp` / `routine.hpp`" | any header under `include/shulib/`, and the `mkdocs.yml` nav rewrite is named |
| `test/CMakeLists.txt` | doc gate `DEPENDS` named 2 headers + 3 pages | globs every header and every page — *a gate that does not re-run when the thing it checks changes is not a gate* |
| `test/api_reference_fidelity_test.cpp` | read the index from `docs/api/README.md` | reads `all-entities.md`; **a new case pins the four shapes the old parser could not see**; the failure message no longer prints a pointer instead of the member name |

### One reviewer harness now measures something else — NOT rewritten

`docs/internal/verify/verify-f1.sh:302` counts TARGETS by grepping `'"header":'` in
`tools/api_doc_tool.py`, to check that F1 added its header to a hand-written list. That list is
now a glob, so the count is no longer meaningful.

**Left untouched on purpose.** The harness belongs to the reviewer, and F1's agent overwriting one
destroyed the independence of that audit — the briefing says so explicitly. Verified it emits
`warn()`, which prints a NOTE and does **not** set `fail`, so `verify-f1.sh` still exits 0. It is
a stale probe, not a red gate, and it is recorded here so the next reviewer knows why rather than
discovering it as a surprise.

