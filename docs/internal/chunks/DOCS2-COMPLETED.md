# DOCS2 — COMPLETED (2026-08-14)

> The generated reference over the whole public API. Written FROM
> [`DOCS2-PROGRESS.md`](DOCS2-PROGRESS.md) (the live log), not instead of it.
> Predecessor: DOCS1. Successor: **the release to `main`**, then R3.
> Brief: [`DOCS2-full-api-reference.md`](DOCS2-full-api-reference.md).
> **Ships no library behaviour.** 97 headers were edited and every one of them is comments only —
> proven by comparing token sequences, not asserted.

## The one-sentence honest status

The reference went from 2 documented types to **1,625 public entities across 115 headers, all
documented and all gated** — but the brief's measurement of the work (971 items, 399
undocumented) was taken with a **broken instrument**: the parser could not see a type with a
base-class list, an enum with an underlying type, or anything at namespace scope, so three
LOCKED contracts produced no output at all and the real debt was **631**. Of the 504 comments
the writing pass produced, an adversarial review found **50 that the code contradicts and 8 that
were filler**; all were corrected, and **83 defects in the code itself** were reported and left
alone.

## The measurement, and why the brief's was low

| | brief (old parser) | measured (fixed parser) |
|---|---:|---:|
| Headers with public entities | 71 | **115** |
| Public entities | 971 | **1,625** |
| Undocumented | 399 | **631** |
| Headers that would not parse | 4 | **0** |

The brief's numbers **reproduce exactly** when re-run with the old parser, which is what makes
the gap a finding rather than a disagreement: both are correct measurements of different things.

### What the parser could not see

The opener regex required `{` immediately after an optional `final`:

```python
re.match(r"^(class|struct|enum class)\s+([A-Za-z_]\w*)\s*(final)?\s*\{", stripped)
```

so **59 top-level definitions were invisible** — not dropped with a warning, never seen. Every
`class X final : public Y`, every `enum class E : std::uint8_t`. That is close to every concrete
implementation class in the library: `Localizer`, `EkfFusion`, `GpsCorrector`, `MoveToPose`,
`TurnTo`, `RunGuard`, `MatrixKinematics`, `MotorMechanism`, `TermSink`, `SdSink`, all fourteen
`hal/pros` adapters, `FaultCode`, `MotionOutcome`, `Frame`.

It also only ever walked type BODIES, so nothing at namespace scope existed for it: **85 free
functions, 32 constants, 11 `using` aliases** — including `arcStep`, `desaturateUniform`,
`xDrive`, every unit literal, and the whole of `spec/accuracy.hpp` and `version.hpp`, which
parsed to **zero items each**.

**Three of the seven LOCKED contracts were invisible in their entirety** — F1 (the coordinate
frame), F2 (the accuracy targets), F3 (the units vocabulary). So the answer to the question that
called this chunk — *why does a fourteen-subsystem library document two types?* — is larger than
"nobody pointed the generator anywhere else." **It could not have been pointed at most of them.**

The blindness never fired before because `Chassis` and `Routine`, the only two targets, happen to
have no base-class list.

## The ruling: shape (1), generate and gate everything

Freeze Register rows **F11–F14 are AMENDED in the register**, not silently contradicted.

**Why not shape (2)** — split the generation list from the coverage list, which the brief named
as the shape to beat:

1. **Its saving is mostly illusory.** The friction people picture when they hear "gated" is
   *regeneration* friction, and `check-fresh` compares the whole of `docs/api/` byte-for-byte, so
   that lands on every *generated* header regardless of gating. The only thing gating adds is
   "a new public member must carry a `///`" — already this project's written standard.
2. **It rebuilds the structure DOCS1 was bitten by.** In DOCS1's own words: *"the hole was not in
   either list. It was in the gap between two lists that were maintained by hand and believed to
   agree."* Shape (2) creates a generate-list and a gate-list and asks a future chunk to remember
   to move a header between them on the day its contract freezes. Nothing enforces that.
3. **It makes the second tier able to rot to empty.** An ungated page renders `**UNDOCUMENTED.**`
   per member. A page that is 100% undocumented is generated, fresh, green and worthless.

**Why not shape (3)** — frozen contracts only: it leaves `diag/` unfixed, which is what the chunk
was called to fix, and it is narrower than it sounds, because F1/F2/F3 needed the entire parser
rewrite anyway.

**Why the register's own argument did not survive contact.** Read together, F11–F14 are one
ruling cited three times: F12 gives its reason as "the F11 precedent", F13 as "the F11/F12
precedent", F14 as "the F11/F12/F13 precedent". Only F11 states a reason at all. And that reason
conflates two mechanisms that are not alike here:

| | changing a **frozen** signature | changing a **gated** seam |
|---|---|---|
| What fires | a compile-time pin naming the register row and the member | `check-coverage` (only for a *new* member) and `check-fresh` |
| What it costs | a major API-version bump plus a migration note — *a breaking change to argue* | one `///` edit and a regeneration |

The brief's counter-argument is right: **that is friction, not a freeze.**

### The two additions that make shape (1) safe

Shape (1) as literally stated would import two rot risks; both are closed rather than accepted.

- **`TARGETS` is now a glob**, with a short commented exclusion list. A hand-maintained 115-entry
  list is the thing that rots, and a header added later and forgotten would be invisible to *both*
  the generator and the gate — the failure the tool's own docstring calls worse than a stale
  document. Same reasoning this file already recorded for the example-source glob.
- **The mkdocs nav is generated** into `mkdocs.yml` between markers and byte-checked by
  `check-fresh`, because a page absent from the nav was **measured** to ship unreachable:

  ```
  INFO    -  The following pages exist in the docs directory, but are not included in the "nav" configuration:
    - api/tracking_wheel.md
  REAL EXIT CODE = 0
  PAGE SHIPPED ANYWAY (unreachable from nav)
  ```

### Rejected alternatives, recorded

- **Flattening `Outer::Inner` into the parent's member list** — the obvious shortcut for nested
  types. Rejected: it renders `Frame` as a peer of `BlackboxReader::open()` and `Frame`'s own
  members vanish entirely, which is exactly the failure the tool's refusal existed to prevent.
  Nested types get their own section, anchors, contents indentation and index rows.
- **One page per subsystem** (11–14 pages). Rejected on measurement: `hal/` alone is 41 headers
  and ~290 entities — a ~200 KB single page.
- **Mirroring the include tree** (`docs/api/hal/pros/motor.md`). Rejected: three page depths make
  every `../../include/...` link depth-dependent, and it renames `docs/api/chassis.md` and
  `docs/api/routine.md`, which are named by a C++ test, `test/CMakeLists.txt`, four public
  documents and **D3's reviewer harness** — which a chunk must not rewrite. Flat naming keeps
  every page at depth 1 and every existing link correct.
- **Keeping the A–Z index inside the overview.** Rejected: it was 180 KB of that page's 190 KB
  and buried the scope paragraph — the one paragraph on this site that has already been corrected
  twice — under 1,625 table rows.

## What the independent oracle found that the self-test did not

A hand-rolled parser validated against its own author's expectations is the shared-model trap.
So the parse was diffed against **clang's real AST** (`clang++ -Xclang -ast-dump=json` — a front
end sharing no code, no regex and no assumption with the tool), entity by entity, over all 115
headers.

**It found four bugs the self-test could not**, all one family — a FIELD mislabelled as a
function and then named after part of its own initializer:

| Declaration | The tool called it |
|---|---|
| `double integralLimit = std::numeric_limits<double>::infinity();` | `AxisGains::std::numeric_limits<double>::infinity` |
| `std::uint32_t abortFaultMask = faultBit(FaultCode::OdoStuck);` | `MotionSchedulerConfig::faultBit` |
| `units::AngularVelocity headingDriftRate{(1.0/60.0) * kPi/180.0};` | `EkfFusionConfig::headingDriftRate{` |
| `bool operator()(...) const` | `AlwaysConfirmed::operator` |

**Final run: 0 missing across 115 headers.** The two remaining "extra" rows are real public
declarations the *oracle's* normalisation misses (`using std::logic_error::logic_error;` and
`template <…> struct SdSinkBuffers`), verified by hand in both headers.

## Three more latent parser bugs, each of which only bites outside the old two targets

- **`_skip_body` mis-read a constructor initializer list containing a braced value.** It counted
  braces textually and stopped at the first balance, so `: MoveToPose(deps, math::Pose2d{}, …)`
  "ended the body" on its first line and parsing resumed mid-expression.
  `motion/hold_pose.hpp` and `motion/strafe_to.hpp` did not parse at all.
- **`_find_terminator` did not know character literals**, and tracked brace depth only at depth 0.
- **Operator punctuation collapsed to nothing in anchors**, so `operator+`, `operator+=` and
  `operator==` on one type shared an anchor, were numbered against each other, and were then
  *labelled* "(overload 2)". Not a broken link — **a confident, wrong sentence about the API.**

## The bug the tool had inside itself, live in the published tree

A writer refused a fix it had been told to make, and explained why. It was right, and the bug was
the tool's:

```cpp
Succeeded = 1,    ///< completed AND confirmed (where the operation defines a
                  ///< confirmation; completed, where it does not)
Unconfirmed = 2,  ///< the operation ran to completion and the confirmation
                  ///< said the world did not change — healthy mechanism…
```

A `///<` **continuation** also starts with `///`, so `_strip_doc` accepted it as a *leading*
comment and it became the documentation for the **next** enumerator. `mechanism_outcome.md`
published `Unconfirmed` as *"< confirmation; completed, where it does not)"*, `TimedOut` with
Unconfirmed's sentence, and dropped the half of `Stalled`'s that names the fault it raises.
**The coverage gate scored all of them as documented**, because the text was non-empty.

That is this tool's own headline failure — *the page looks complete* — reproduced inside the
tool, invisible to every gate, and found by a reader. Fixed; pinned in the self-test for both
shapes, because the fixture had neither.

## Mutation campaign — and the three holes the first pass found

Run in an isolated copy so it could not disturb the fourteen agents working in the live tree.

**First pass: 13/16 RED, and the three GREEN ones were the point.**

| Survivor | Why the self-test could not see it |
|---|---|
| **M3** — `_find_terminator` forgets character literals | The fixture's only char-literal constant was `= {';', '{', '}'}`, and those three literals **balance** — a blind parser lands in the right place *by luck*. |
| **M10** — a top-level `=` no longer ends the function search | No fixture field had a **call** in its initializer. |
| **M11** — braces are not a depth in `_looks_like_function` | No fixture field had a **parenthesised brace initializer**. |

M10 and M11 are the family clang had already caught in the shipped tree — so the self-test was
blind to a bug that had actually happened. Fixture extended with an *unbalanced* character
literal, a call-initialized field and a parenthesised brace initializer, with a real function
beside them so the rule cannot be satisfied by calling everything a field.

**Second pass: 16/16 RED.** M16 is D3's own hole probe re-run — a regression guard three chunks
old still holding.

## The documentation, and what a reader would actually get

Fourteen writers, one batch each; then fourteen **different** agents read each batch back, with
one instruction: catch what a gate cannot.

| | |
|---|---:|
| Comments written by the writing pass | **504** (of which **177** promoted from existing `//` prose) |
| `///` lines added to headers | **2,155** |
| Comments the reviewers actually read | **478** |
| Found **factually wrong** — the code contradicts them | **50** |
| Found to be **filler** | **8** |
| Corrections applied after verification | **60 fixed · 2 rejected with evidence · 2 partial** |

**Both rejections were correct**, and are the reason the fix stage verified rather than obeyed:
the flagged sentences no longer existed (they had already been corrected), and the fixer proved
that by grep before declining. A fix stage that trusted its reviewer would have churned two
correct comments.

## API defects — 83, reported and NOT fixed

Full list with evidence: [`DOCS2-API-DEFECTS.md`](DOCS2-API-DEFECTS.md). 73 found while writing,
10 more found while *correcting*, when a reviewer's objection sent someone back to the code and
the code turned out to be at fault.

The brief predicted this would be a wide net, and it was. A few that stand out:

- **`~MotionScheduler() = default`** — destroying a scheduler with a motion still armed leaves
  the drive motors at their last commanded voltage. F2 closed exactly this hole for the blocking
  waits with `WaitUnwindGuard`; the destructor is the remaining path with identical consequences.
- **`MotionStatsSink::targetPose()`** returns the *previous* motion's target between motions —
  `beginMotion()` resets every aggregate except `target_` and `startPose_`. Only the scheduler's
  own `hasData()` guard hides it from the run summary.
- **`ProsRotation` / `ProsMotor` hold-last-good caches are zero-seeded** with no valid-yet flag,
  so a pod that faults from the first read publishes exactly the 0 rad the screen exists to never
  emit — and the ODO_STUCK cross-check is built to notice a frozen *non-zero* value.
- **`HealthMonitor`** does not check `brownoutRecoverVolts` for finiteness, only its ordering, so
  `+Inf` constructs and the brownout latch can never re-arm — one episode per run, silently.
- **`control::Watchdog` cannot deliver the §M2 "a motion can never hang" guarantee** against a
  control task that stops running: it is purely polled, from inside the very `tick()` that would
  be blocked, and nothing in the library owns an independent task.
- **`TrapezoidProfile`'s header taught an integration that does not exist** — *"the velocity
  target feeds Feedforward; the position target feeds the per-axis Pid"* and *"the motion layer
  instantiates one per axis"*. Nothing in the tree constructs one; the only consumer is its own
  test. This is `hal/battery.hpp` again: a banner teaching a model the code does not implement,
  faithfully promoted into a generated page by a writer doing what it was told. The prose is
  corrected; the *gap* is reported.

## The site, because a reference nobody can navigate is not a reference

Called mid-chunk by the team lead, and it lands squarely here: **this chunk is what makes the
site big** — 2 API pages became 117 in one commit. `mkdocs` was not installed on this machine, so
the site had never been built locally; installed into a scratch venv at the **exact versions CI
pins** (`mkdocs==1.6.1`, `mkdocs-material==9.5.44`), so what is verified is what publishes.

- **`navigation.tabs`** — the sidebar shows one area at a time; the API tree does not exist until
  you ask for it.
- **`navigation.sections` REMOVED** — with it, all fifteen subsystem groups render expanded,
  always. Without it, 17 sidebar items instead of 130.
- **`navigation.indexes`** — clicking a section lands on its overview instead of doing nothing.
- **Design commentary folds** — open under 45 lines, closed above, with the line count in the
  summary so nothing is hidden, only folded. Plain `<details>` + `md_in_html`, not a Material
  admonition, because these documents are read on GitHub too.
- **The overview leads with the short answer** before it mentions its own size: *writing a
  routine? you need two of these pages.*
- **Every page opens with a one-line census**; the A–Z index is split by letter with a jump bar;
  nav labels read `ICorrector`, `PID`, `AprilTag corrector`, `Motor (PROS)`.
- **Interface pages no longer repeat the same rule-of-five paragraph six times** — one comment
  covers a run of special members, and the page now says so and points at it.
- Blurbs strip trailing development-process citations (`(chunk C4, WS6/M2)`) — true in the
  header, noise in a one-line summary for someone who has never heard of chunk C4. The banner
  still carries them in full further down the page.
- The "what is not here" list's macro names and protected-section count are **derived from the
  tree**, not typed, because a hand-typed count in a document whose whole purpose is that it
  cannot go stale is the failure this project keeps hitting.

## Numbers

| Measure | Result |
|---|---|
| Public entities documented | **1,625** across **115** headers — coverage gate green |
| Pages generated | **117** + the A–Z index, all in the nav, nav byte-checked |
| Headers edited | 97 — **token-identical to HEAD**, comments only |
| Suite | 1,121 cases / 1,523,344 assertions / 3 skipped — **green** |
| Independent oracle (clang AST, 115 headers) | **0 missing** |
| Mutation campaign | **16/16 red** (after the first pass found 3 holes) |
| Both CI guards | PASS |
| ARM gate | 148 headers, unamended, clean |
| Doc gates | coverage · fresh (×2 consecutive runs) · examples · removability · self-test · staleness — **all PASS** |
| `prepare_site.py` (the release gate) | **PASS** |
| `mkdocs build --strict` (CI-pinned versions) | **PASS**, 147 pages |
| API defects | **83 reported, 0 fixed** |

## Not finished, named honestly

- **`[~]` "No filler" rests on one reading pass, not two.** Fourteen reviewers read 478 of the
  504 new comments and flagged 8. The remaining ~26 were in batches whose reviewer reported a
  smaller count than the writer claimed; nobody re-read them, and I sampled roughly ten headers
  myself rather than all 97. So the honest claim is: **every comment was written from the header
  it documents, and 478 of them were read back by an independent agent that was hunting filler
  and found 8.** It is not "1,625 members each have a sentence a reader would thank you for" — it
  is "1,625 members each have a sentence, and the ones that were re-read hold up."
- **`[~]` The 50 corrected comments were verified against source by the agent that fixed them,
  not by a third pass.** The two rejections show the verification is real. A fourth pass would
  cost another campaign and was not run.
- **The 83 API defects are untriaged and unsized.** No severities were assigned, deliberately:
  they were found by reading, not by measuring, and a confident ranking would be the same
  unearned claim the list exists to catch.
- **`protected` members are out of scope**, stated on the reference's own front page. One section
  in the tree (`motion/move_to_pose.hpp`), and it is the surface `StrafeTo`/`HoldPose` extend —
  so a reader subclassing `MoveToPose` still needs the header.
- **Preprocessor macros are out of scope** (`SHULIB_PRECONDITION`, `SHULIB_TRACE`). They are on
  the site in prose, because every page reproduces its header's banner, but not in the member
  lists or the index.
- **Nothing was committed and nothing was pushed.**

## Two things fixed where they live (Rule 4)

- **`PROJECT-BRIEFING.md` §7's guard command has been failing on a clean tree.** As printed it
  reports `GUARD1-FAIL` with 21 hits, because R1a added `include/shulib/hal/pros/` — the one path
  that exists to include PROS. CI has the path-anchored form and §4 *describes* the amendment
  correctly; the copy-pasteable command never got it. This is DOCS1's broken ARM `sed` again, in
  the same document, one section apart.
- **The roadmap still marked generated API docs `[~]` because "nothing is published."** The site
  has been live since DOCS1. Corrected, and rewritten for this chunk's scope.
- **The doc gate's `DEPENDS` named two headers and three pages.** It now covers every header and
  every page by glob — otherwise editing any other header left the coverage and freshness gates
  un-run until something unrelated forced a rebuild. That is this very file's own lesson: *a gate
  that does not re-run when the thing it checks changes is not a gate.*

## Handoffs

**To the release:** every gate green, including `prepare_site.py` and a strict `mkdocs build` at
CI's pinned versions. The site gains 117 pages and a generated nav; the nav block lives between
markers in `mkdocs.yml` and is byte-checked, so a hand edit there fails the build rather than
publishing an unreachable page.

**To F3 (and to whoever freezes F11):** the seam is documented and gated now, and the register row
says in writing that this freezes nothing. The freeze trigger is unchanged — a second real
consumer on hardware.

**To R3/R4:** `DOCS2-API-DEFECTS.md` has ten entries about `hal/pros` adapter behaviour that a
bench session can settle in minutes — the zero-seeded last-good caches most of all, because a
dead sensor currently reads as a stationary robot rather than as a fault.

**To whoever adds a header next:** you do not have to do anything to be covered. The target list
is a glob, the nav is generated, and the build will tell you, by name and line, about any public
entity you left unexplained.
