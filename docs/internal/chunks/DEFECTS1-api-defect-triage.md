# DEFECTS1 — triage and resolution of the 83 DOCS2 API defects

> **Brief.** Predecessor: DOCS2 (which produced the input). Successor: the release to `main`,
> then R3. Live log: [`DEFECTS1-PROGRESS.md`](DEFECTS1-PROGRESS.md). Input:
> [`DOCS2-API-DEFECTS.md`](DOCS2-API-DEFECTS.md).
>
> **Scope in one line:** triage all 83 reported API defects into FIX / ARGUE / REJECT / DEFER,
> then apply the FIXes with tests that would have caught them. **No new features, no unrelated
> refactors, no documentation sweeps beyond what these fixes require.**

---

## 1. Why this chunk is here, in this position

DOCS2 documented 1,625 public entities, which meant reading every one of them against its
implementation. Its landmine L3 was **report, do not fix**, so it reported: 83 defects, with
evidence, deliberately left in the tree. That was the right call for a documentation chunk — a
pass that both documents and changes behaviour cannot tell you which of the two broke the suite.

It is the wrong call for a release. The next chunk in the order is **the merge to `main`, which
is what publishes docs.shurobotics.com**, and DOCS2's own framing of why the documentation
chunks came first applies verbatim here: *anything wrong becomes publicly wrong at that moment,
under the project's own name.* Right now the published reference contains careful, accurate
sentences describing defects — `hal/pros/rotation.hpp` telling a reader a faulted first read
returns zero, `motion_scheduler.hpp` telling a reader the destructor leaves the drive energized.
Those sentences are honest. They are also an inventory of things we know are wrong and shipped
anyway.

Rule 4 now applies: **a flaw gets fixed where it lives.** DOCS2 was exempt by its own landmine.
This chunk is not.

## 2. What the input actually is, and the three ways it can mislead

**It is 83 claims, not 83 bugs.** Three distinct populations are mixed together and nothing in
the document separates them, deliberately:

1. **Real defects** with real failure scenarios.
2. **Deliberate decisions** ruled in an earlier chunk and recorded in a header banner, a chunk
   record, or the Freeze Register. Fixing one of these is not a fix; it is an unrecorded reversal
   of somebody's reasoning.
3. **Findings that are simply wrong.** DOCS2's own correction pass rejected 2 of 62 findings with
   evidence and **both rejections were correct**. That ratio is the reason the first job here is
   triage and not repair.

**No severity was assigned, deliberately** — the list's own words: *"they were found by reading,
not by measuring, and a confident ranking would be the same unearned claim the list exists to
catch."* So this chunk does the measuring the list refused to fake.

**The list also contains duplicates**, which the count of 83 does not acknowledge: sections A/D/I
were written by fourteen writers and section E by the correction pass, and the same defect
reached the list by two routes more than once. Duplicates are identified explicitly rather than
resolved twice, and **the 83 count is kept** — collapsing it would make the list disagree with
DOCS2's completion record for no gain.

## 3. The trap that governs every fix in this chunk

**Every one of these headers now carries `///` comments that document the current, defective
behaviour — accurately, on purpose.** DOCS2 was told that a sentence which is hard to write
honestly *is* the finding, so where it could not describe a defect away, it described the defect.

That means **fixing the code turns a published, accurate sentence into a lie in the other
direction.** Measured, not assumed — every one of these is live in the tree today:

| Item | The header's own words at HEAD |
|---|---|
| A28 | *"Destruction is DEFAULTED and does not cancel — a scheduler destroyed with a motion still armed leaves the drive at its last command"* |
| A27 | *"beginMotion() does NOT clear it, so between motions it still holds the PREVIOUS motion's target"* |
| E9 | *"A recover level of +Inf therefore CONSTRUCTS: the re-arm test in tick() … can then never be true"* |
| E7 | *"`minDt` is NOT checked, and nothing checks `minDt <= maxDt`"* |
| A1 | *"checked only for emptiness, never against the kinematics' wheel count"* |
| A10 | *"A copied mechanism therefore arrives already claimed()"* |
| A15 | *"Reads are LIVE: each accessor hits the device, so distance() and confidence() are two samples"* |

**So the rule for this chunk, and it is not negotiable: every behaviour change updates its `///`
in the same commit, then `python3 tools/api_doc_tool.py generate`, and the regenerated
`docs/api/` and `mkdocs.yml` nav block are committed with it.** `check-fresh` fails the build
until that happens, which is the gate doing its job — and D3's ordering hazard means it fires
*before* the signature pins, so on a frozen-surface mistake it will name the wrong problem first.

## 4. The freeze boundary, stated precisely because several items sit on it

Register rows **F1–F5, F6 and F10 are LOCKED**. The row that decides the most items here is F4,
and it is narrower than "the HAL":

> **F4 covers the signatures of exactly ten interfaces:** `IClock`, `IMotor`, `IRotation`,
> `IImu`, `IGps`, `IDistance`, `IOptical`, `IBattery`, `ITelemetrySink`, `IVision`+`ITagSource`.

**Outside F4:** `ILineDisplay`, `ICharSink`, `IBlockSink`, `IDigitalOut`, `IController`,
`IDigitalIn`, `IMechanism` — and **everything in `hal/pros/`**, because rows F11/F13/F14 say in
writing that *adapters are implementations, not contracts*. That single line makes the whole
cold-start cache family (D6/E5/E6/E3/A19) an ordinary fix rather than a frozen-surface argument,
because every one of them is fixable **inside the adapter** without adding a member to the seam.

**Being documented is not being frozen.** DOCS2 put the entire tree under the coverage gate and
amended rows F11–F14 to say so out loud: changing a frozen signature costs a major version bump
and a migration note; changing a gated-but-unfrozen member costs one `///` edit and a
regeneration. Do not mistake the gate for a freeze.

**A defect whose fix requires changing a frozen signature is ARGUE, not an edit** — stop and
write up the breaking-change case.

## 5. The four dispositions

| | Meaning | What it must carry |
|---|---|---|
| **FIX** | Real, and fixable here without breaking a frozen signature or an existing caller | a test that would have caught it; mutation proof if load-bearing; the `///` update |
| **ARGUE** | Real, but the fix is a breaking change or needs a ruling the chunk cannot make alone | the breaking-change case, written up, **not applied** |
| **REJECT** | The finding does not hold — refuted, already fixed, or a deliberate documented decision | **the code evidence that refutes it** |
| **DEFER** | Real, but owned by a later chunk (hardware, or another chunk's design) | the owning chunk, named |

**Every one of the 83 lands in exactly one, and none is silently dropped.** The triage is in §7.

## 6. Constraints, each with its reason

1. **Triage before repair.** Not a formality: three items in this list are *already fixed at
   HEAD* and one is *refuted by a two-line probe*. A chunk that started fixing would have
   "fixed" them.
2. **Tests are the deliverable, not the fix.** Every FIX gets a test that names, in a comment,
   the bug it would catch. Load-bearing logic gets mutation proof: break it, **rebuild**, run,
   **observe red**, restore. A mutation reasoned about but not executed does not count, and **a
   mutation that stays GREEN is a hole in the suite and the most valuable thing this chunk can
   find.**
3. **From-scratch oracles with a negative control** for anything where the test could share a
   model with the code (briefing trap 1, six chunks and counting). Run the scenario with the fix
   *absent* and prove the instrument can tell the difference.
4. **Recover, do not abort.** `plausibility_guard.hpp` states the volt-path design: non-finite
   values are recovered at the **motor edge**, not thrown at the math helpers, because *"a
   diagnostic that mutates the data path is worse than the bug it hunts"* and because A1's rule
   is that faults log and recover, never crash. Several finiteness items look like they want a
   throwing precondition and **must not get one** — a hostile sensor has to degrade a motion, not
   abort the auton.
5. **An invented constant gets an HA entry** with blast radius and the measurement that would
   settle it.
6. **Nothing pushed.** The team lead pushes.

## 7. The triage

**84 items: the 83 reported, plus one this triage found (`N1`).** Every ID appears exactly once.

| | Count | |
|---|---:|---|
| **FIX** | 59 | real, fixable here |
| **REJECT** | 15 | refuted, already fixed, or a recorded decision |
| **ARGUE** | 6 | real; the fix is breaking or needs a ruling |
| **DEFER** | 4 | real; owned by a later chunk |

### How this was arrived at, and where it disagrees with itself

Twelve read-only agents triaged the 83 against the implementation with probes; twelve more were
to verify adversarially, arguing both sides. **Eight of the twelve verifiers died on a session
limit**, so verification covers only clusters C1, C3, C5 and C11 (33 items). The rest is verified
by me directly and — for every FIX — by the build, the suite and the mutation campaign, which is
the stronger instrument anyway. **Where an item below is marked `[unverified-2nd]`, it carried
one triage pass plus my own reading, not an independent adversarial one.** That is stated rather
than smoothed over.

The two places the passes disagreed are the two most interesting rulings here, and in both the
second reader was right:

- **`D6` — triage said REJECT, verification said FIX, and verification found the thing both of us
  had missed.** DOCS2 caveated the two *adapter* doc sites but never touched the **F4 interface
  header**: `hal/rotation.hpp` still publishes, unqualified, *"A frozen reading, not a zeroed one,
  is what the loop's stuck-odometry cross-check is built to notice."* Its sibling `hal/motor.hpp`
  **does** carry its cold-start caveat. That asymmetry is the defect, stated better than the
  original finding stated it.
- **`I21` — verification proved the fix is possible and I am still ruling ARGUE**, on a ground the
  triage did not have. An identity-interval early-out makes `std::remainder` unreachable during
  constant evaluation, and clang then accepts `degrees(90.0)`; the sweep found 0 mismatches in
  1,028,583 samples including exact boundaries. But it delivers a **cliff**: `90_deg` becomes
  constexpr and `315_deg` does not, with no rule a caller can see — and buying that costs ~6 lines
  across LOCKED row **F3**, including its private trusted constructor and four accessors. A
  capability that works for some literals and not others is worse than one that consistently does
  not. That is a decision for the team lead, not an edit.

### FIX — 59

*Grouped by the commit that lands them.*

**The scheduler (highest consequence).**
`A28` destructor leaves the drive energized — measured at **8.412 V** on all four motors ·
`A27` `targetPose()` serves the previous motion's target between motions.

**Non-finite values that construct or leak silently.**
`E9` `brownoutRecoverVolts = +Inf` constructs and permanently disarms the brownout detector ·
`I3` `maxAcceleration = inf` accepted, `sample(0).accel = inf` · `E10` `sample(NaN)` returns a
**partially** finite state (accel finite) and `isDone(NaN)` is false · `D13` an infinite
`defaultTimeout` builds a watchdog that never expires · `A2` `brownoutLimited` reads **clean**
for NaN · `I2` `brownoutLimited` has no default initializer.

**Unguarded spans on the commanding path.**
`A1` motors indexed by wheel index with no count cross-check — out-of-bounds UB every tick ·
`A30` an empty span makes the stall check permanently un-trippable, and elements are unchecked.

**The mechanism claim token.**
`A24`/`A25` a stale finished operation safes a mechanism a *live* operation owns · `A10` a copied
mechanism is born claimed, pointing at the original's operation.

**Missing guards and defaults.**
`A3` non-`explicit` converting constructor · `A8` the tick-open check is bypassable · `A12` no
null check on the injected `FILE*` · `A20` indeterminate `id`/`confidence` on the vision structs ·
`A23` unchecked public indexing into the EKF state · `A32` config validated *after* the members
are built from it, so the wrong component's message fires.

**diag rendering.**
`D2` the promised static check does not exist **and would be red today** — `MECHANISM_STALLED` is
17 chars against a 15-char budget, stale since F1 · `A4` an embedded NUL makes a tag's own dial
unreachable · `A5` a truncated line can emit a **split UTF-8 ellipsis** · `A6` any column wider
than 10 is compacted away · `A7` an unpopulated result line reports a settled motion.

**hal/pros counters, constants and construction.**
`I9`+`E1` the fifth screening site does not count · `D5` an offset-rejected GPS reports zero
faulted reads forever · `I8` the sentinel and the mm→inch factor hardcoded beside their own named
constants · `E2` brake mode inherited from whatever program ran last · `A11` the unreportable
final flush, stated · `A14` `ProsDigitalOut(1, 2)` compiles clean and fires a solenoid HIGH at
boot on the wrong port · `I10` an overrunning tick body replays phantom catch-up ticks.

**localization.**
`E7` `minDt` unchecked and uncoupled to `maxDt` · `I13` the EKF reports a zero-budget fix as
applied with full confidence · `I15`+`E8` an applied correction is attributed to the
first-registered corrector · `I14` a declined tick increments no counter · `A22` an over-full tag
frame is truncated by arrival order, uncounted · `I16` an untyped radian threshold · `N1` **new:
a pod that enumerates late injects a one-tick phantom translation — measured 28.36 in — and the
odometry gates |Δθ| but never |Δtravel|.**

**motion.**
`D12`/`D14` the hold-mode watchdog is the sole bound on the boot wait and is never read on the
live path · `I18` centre-to-wheel distance is two independently settable fields.

**Documentation that the code contradicts** *(the fix is the sentence — see §3)*.
`D1` `D4` `D6` `D8` `D9` `D11` `D15` `D16` `D17` `D18` `I5` `I6` `I7` `I17` `I22` `A21`.

### REJECT — 15, each with the evidence that refutes it

| ID | Why it does not hold |
|---|---|
| `A9` | Both consequences are false. `IClock c = someProsClock;` **does not compile** (abstract type), and the assignment that does discards nothing — `sizeof(IClock)` is 8, the vptr alone; probe: `a.now()=5.0 b.now()=9.0` unchanged. The header already rules on it, with reasons. |
| `A13` | `pros::v5::Controller`'s only data member is the id fixed at construction; the const readers mutate nothing. `mutable` is there because PROS declared its getters non-const. |
| `A18` | The doc says "reads", the code counts reads. `HealthMonitor` raises `ImuLost` from `isReady()`, never from this counter — and **nothing in the library reads `faultedReads()` at all**. |
| `A19` | 0 °C reads exactly as healthy as 20 °C to the only consumer (a `>= 55 °C` threshold), so the symmetry "fix" changes no observable. A port dead at construction cannot produce a `ProsMotor` — the ctor read-back throws. |
| `D3` | Already fixed at DOCS2; `worstDt()` now reads *"since construction"* and the generated page carries it. |
| `D7`·`D10`·`I1` | Already fixed at DOCS2. `_strip_doc` excludes `///<` at HEAD; verified on the published pages — `Unconfirmed`, `TimedOut`, `Cancelled`, `FusionResult::audit` all carry their own complete sentences. |
| `E5`·`E6` | Duplicate reports of `D6`, closed by its fix. `E5`'s additional motor-half claim is separately refuted: `ProsMotor`'s constructor read-back **throws** on a port that did not answer. |
| `I4` | D2 ruled the typed/untyped boundary explicitly — *typed at the facade, seconds-double inside the motion stack* — and `Watchdog` sits inside the motion stack with three siblings shaped the same way. Not an outlier. |
| `I11`·`O1` | Already fixed at HEAD: `LogLevel`, `Localizer::Quality` and `BrakeMode` are all reflowed one-per-line with their own `///`. `check-coverage` passes tree-wide, which it could not otherwise. |
| `I12` | The premise is false. `maxNudgeRate == 0` is a **useful configuration** — a heading-only corrector — reporting `applied=0 conf=0.000`; a near-zero *gain* reports `applied=1 conf=0.900` for a fix that moved 1.8 attoinches. The constructor bans the one that lies and permits the one that works. |
| `I19` | Both values are recorded rulings (C2's D11; the project-wide "none yet" convention), every available change relocates the disagreement rather than removing it, and setting `lastExit_ = Running` would make `waitUntilSettled()` return a value its own contract says it never returns — on an **F6-frozen** facade. The discriminator is already named in `lastCompleted()`'s and `completedCount()`'s comments. |

### ARGUE — 6, written up and NOT applied

`A15` · `A16` — an atomic `confidence()`+`distance()` / `hasFix()`+`pose()` pair needs either a
method on **F4-LOCKED** `IDistance`/`IGps`, or an adapter-side sample window, which invents a
constant nobody has measured. · `A26` — a frame discriminator on `Twist2d`; every type-level cure
is a wide breaking change. · `A31` — narrowing `StrafeTo`'s inherited `setTarget` changes a public
inherited signature. · `I20` — `spec/accuracy.hpp` **is** register row **F2**, LOCKED. · `I21` —
above.

### DEFER — 4, with the owner named

`A17` → **R4** (the consuming-read rebase needs measured call patterns and real loop timing) ·
`A29` → **R3/R4** (no gear-ratio concept exists anywhere, and the A2 sim plant bakes 1:1 in too —
larger than this chunk) · `E3` → **F3** (no honest finite seed exists for hue; the chunk that
writes the first consumer owns the validity decision) · `E4` → **R4/T2** (a polled watchdog cannot
beat a stopped task without a supervisory task, which nothing in the library owns).

## 8. Out of scope, with the owner named

- **New behaviour of any kind** that is not the direct repair of a listed item.
- **The `hal/pros` items a bench session settles faster than argument** — flagged in the triage
  and handed to **R3**, which walks the assumptions register with the robot on the bench.
- **`tools/api_doc_tool.py` parser work** beyond what a listed item requires. DOCS2 rewrote that
  parser and validated it against clang's AST; re-opening it is not this chunk's business.
- **Retriaging DOCS2's documentation corrections.** The 50 fixed comments and 2 rejections are
  DOCS2's record, not this chunk's input.

## 9. Landmines

- **L1 — the `///` that documents the defect.** §3. The single most likely way this chunk ships
  something wrong is a code fix whose comment still describes the old behaviour, published to the
  site by the next merge.
- **L2 — the freshness gate fires before the signature pins.** D3 recorded it and it bit again at
  F1's verification. A red `check-fresh` on a frozen-surface mistake names the wrong problem.
- **L3 — the shared-model trap.** A test for a cache fix that uses the same cache, or a test for a
  clock fix that uses the same clock, passes while proving nothing.
- **L4 — a fix that reverses an earlier ruling.** A24/A25 are the live example: an existing test
  asserts the exact behaviour they call a defect. The fix has to close the hole *without*
  overturning the tested decision, or it has to argue for the reversal explicitly.
- **L5 — `PIPE`.** A mutation runner piped into `head` took a SIGPIPE mid-campaign and left a
  header with a line deleted. Count what you run, and never `git checkout` a file holding
  uncommitted work.
- **L6 — the from-scratch build deadlock.** The doc gates run before the binary is relinked, so
  `briefing_status.py` can derive the suite state from a stale binary and report RED. Generate,
  build, generate again, rebuild.
- **L7 — `briefing_status.py check` is RED for the whole chunk** by design, because
  `DEFECTS1-PROGRESS.md` has no completion record yet. Do not "fix" that by deleting the log.

## 10. Definition of done

- [ ] All 83 triaged FIX / ARGUE / REJECT / DEFER, each with evidence, **none silently dropped**
- [ ] Every FIX has a test that would have caught it; load-bearing ones mutation-proven with
      **observed** red
- [ ] Every behaviour change updated its `///` in the same commit; `docs/api/` regenerated and
      committed, nav block included
- [ ] Every ARGUE written up with the breaking-change case, **not applied**
- [ ] Every REJECT carries the code evidence that refutes it
- [ ] `DOCS2-API-DEFECTS.md` annotated in place with each item's outcome, so the list is
      self-describing
- [ ] Suite green; both CI guards; ARM gate over every header; all six doc gates; the release
      gate (`prepare_site.py`)
- [ ] Changelog updated if any public surface changed meaning; briefing block regenerated
- [ ] Nothing pushed
