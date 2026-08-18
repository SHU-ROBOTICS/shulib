# R3b — First closed loop on a tank drive

> **Closes M1's DoD and M2's on-robot clause.** Three additive pieces of library code, each
> host-provable against the A2 plant and the existing fakes long before any of them meets a robot.
>
> **Status of this brief:** §6 (piece 3) is COMPLETE and executable **now** — it depends on no
> measurement. §4 and §5 (pieces 1 and 2) are specified as far as they honestly can be; every
> decision that waits on a number names the R3a Batch-1 item that settles it, rather than guessing.
> **Do not close those two sections' open decisions by inference.**

---

## 1. Why this chunk is here in the order

`build-order.md` Stage 1 is `R3a → R3b` and says nothing else matters until it is done. R3a makes
the constants real; R3b is **the first moment anything in this project is *true* rather than
*consistent*.** Every accuracy number in the repo today — including E4's EKF comparison, which
decided the default fusion policy — is a simulation result.

R3 was split on 2026-08-17 because its DoD turned out to be blocked on **library code, not
hardware**. M1's DoD is *"identical numbers in a host test and on the V5, swapping only
`RobotContext`"*, and `RobotContext` precondition-requires a `gps`, a `tags` and a `vision` the
bench robot does not have. That is piece 3. Pieces 1 and 2 are the other two shapes that a real
7-motor, pod-less drivetrain exposed.

**The transferable lesson, already recorded in `build-order.md`:** the A4 register inventoried the
unproven **constants**; it did not inventory the unproven **shapes**. This chunk is the shapes.

---

## 2. What already exists to build on

| Thing | Where | Why it matters here |
|---|---|---|
| `RobotContext` + `RobotContextConfig` | `chassis/robot_context.hpp` | the composition root; its ctor validates all eight handles non-null |
| `NullSink` | `hal/null_sink.hpp` | **the in-tree null-object precedent** — `final`, lives in `hal/` not `hal/fake/`, documents its own cost contract |
| `FakeTagSource` / `FakeVision` | `hal/fake/` | the *test doubles* currently miscast as absent-device stubs in `main.cpp:166-167` |
| `IGps` / `ITagSource` / `IVision` | `hal/gps.hpp`, `hal/vision.hpp` | the three seams; read their contracts before writing anything |
| `ProsBlockSink`'s no-card ruling (T5) | `hal/pros/block_sink.hpp` | **the precedent for how absence is reported** — construct, degrade honestly, let the composition root say it out loud |
| `AprilTagCorrector` | `localization/apriltag_corrector.hpp:233` | reads `tags_.tags()` by value each tick — the one consumer of `ITagSource` |
| `robot_context_test.cpp` | `test/` | already asserts the fake defaults at lines 90-92; extend, do not replace |

---

## 3. Scope

**In:** all three pieces below.

**Out:**
- Anything needing a **GPS, tracking-wheel pods, a camera, or a competition robot** → **R3c**.
- Measured noise parameters → **R4**. Real gains → **R5**. Plant back-fit → **R6**.
- The `IVision`-consuming manipulation targeting → **M4 / F3**.

**Explicitly rejected:**
- Making `gps` / `tags` / `vision` **optional pointers** on `RobotContextConfig`. See §6.2 — this
  is the central ruling of the chunk and it goes the other way.
- Adding an `isPresent()` to the three L0 seams. See §6.4.

---

## 4. Piece 1 — multi-motor-per-side aggregation

`motion/command_pipeline.hpp:147-152` maps kinematic wheel → motor **1:1** (`motors[i]->setVoltage(...)`),
and `motion/motion.hpp:201-203` guards it with `>=`. So an over-provisioned tank drive is
**accepted**, two motors are commanded, and **the rest are never given a voltage or a brake mode —
silently.** `kinematics/tank.hpp:80` delegates this to "the HAL's business" and **no such HAL
facility exists.** Every real VEX drivetrain has 2–4 motors per side.

> **⚠ USE THESE NUMBERS, NOT `build-order.md`'s.** That entry says *"a 7-motor tank drive … five
> never given a voltage"* and is **stale**: `R3a-PROGRESS.md` §9.1 records that **port 13 was
> mechanically repaired**, so the bench drivetrain is **8 motors, 4 per side — symmetric**, and the
> shipped pipeline would command **2 and leave 6 dead, not 5.** §9.1 also states that the measured
> probe in §4.1 *"was taken at 7 motors and should be re-stated at 8."* Re-run the probe at 8 and
> record both, rather than editing the historical §4.1 number.
>
> Two consequences §9.1 draws that this chunk inherits: the drivetrain is now **symmetric in motor
> count**, so the "will not drive straight under an open-loop symmetric command" caveat is
> **retracted**; and a residual asymmetry may still exist from **per-side gearing** — a different
> cause, which the repair does not address. See §4.1 below: it is the biggest open item in the chunk.

### 4.1 THE PER-SIDE RATIO — `R3a-PROGRESS.md` §8.4, and the largest design risk here

§8.4's title is the finding: *"A29 is worse than 'no gear-ratio concept'. It needs a PER-SIDE
ratio."* Calypso's source says *"the right side is mechanically geared down a touch for traction"*
alongside `RIGHT_DRIVE_BIAS = 1.08`, so **if that drivetrain is the bench bot's, the two sides have
different gear ratios.** shulib cannot represent that anywhere:

| Site | Why it cannot |
|---|---|
| `motion/odo_stall_check.hpp:84` | `wheelRadius` is **one scalar for every drive motor** |
| `kinematics/tank.hpp:47-49` | `toWheels()` is `left = vx − ω·halfTrack`, `right = vx + ω·halfTrack` — **the same scale on both sides, by construction** |
| `hal/pros/motor.hpp:96` | `ProsMotor(port, gearset)` — no ratio parameter of any kind |

§8.4's verdict: *"An asymmetric drivetrain is outside the model, not merely unparameterized … the fix
R3b needs is a **per-side** ratio, not a single library-wide one — and that is a design change to how
a drivetrain is described, not a new constant."*

**RULING — the per-side ratio lives in the MOTOR GROUP (piece 1), never in kinematics.**
`IKinematics` is Freeze Register row **F5, ✅ LOCKED 2026-06-19**. Changing `toWheels()` to carry
asymmetry would break a locked contract for a fact about one robot's gearbox. Keep the split clean:
kinematics answers *"how fast should each side's wheel travel"* (symmetric, geometric, frozen), and
the group answers *"what voltage makes this side's wheel travel that fast"* (per-side, mechanical,
new). That is the same seam `kinematics/tank.hpp:80` already points at — *"how many physical motors
sit on each side is the HAL's business"* — and it resolves §8.4 without touching F5 at all.

**Piece 2 inherits it too:** drive-encoder odometry converts encoder counts to distance and therefore
needs the same per-side ratio. Do not solve it twice — one representation, consumed by both.

**⚠ THIS AMENDS R3a's `B1.2`.** As written, B1.2 asks for *"the external gear ratio"* — one number.
§8.4 makes it *"count teeth on **BOTH** sides,"* which it calls **"the single highest-value
measurement in the chunk."** A single answer is not sufficient and, if the sides differ, is actively
misleading. **Ask per side.**

**Shape:** a motor *group* that presents N physical motors behind one `IMotor`, fanning voltage and
brake mode to all of them and aggregating their readings back.

**OPEN — settled by R3a `B1.2` (external gear ratio):** whether the group needs a **ratio
parameter**. If the bench bot is direct-drive, the group is a pure fan-out. If it is geared, the
group is the natural place the ratio lands.

**A29 is bigger than it looks, and it can threaten this chunk's headline DoD.** DEFECTS1 defers
`A29` to R3/R4 with this reason: *"no gear-ratio concept exists anywhere, **and the A2 sim plant
bakes 1:1 in too** — larger than this chunk."* So if B1.2 comes back geared:

- `motion/odo_stall_check.hpp:84` (`units::Length wheelRadius{3.25 / 2.0}`) has a name and comment
  that are wrong on this robot, exactly as R3a's B1.2 row predicts; **and**
- the **A2 plant also assumes 1:1**, so a host test and the V5 would be modelling different
  drivetrains. M1's DoD is *"identical numbers in a host test and on the V5"* — **that comparison is
  not even checkable until the plant and the motor group agree about gearing.**

**Ruling: if B1.2 returns anything but direct drive, teaching the A2 plant the ratio is IN SCOPE for
piece 1**, because M1's DoD cannot close without it. If it returns direct drive, the whole question
defers to R4 untouched. **Do not pick until B1.2 is in.**

**Also open, and cheap to resolve once devices are known:** what a group does when one member motor
is disconnected. The `IMotor` seam already screens a sentinel and holds last-good (`hal/motor.hpp:49`);
a group must decide whether one dead member degrades the group or fails it. Recommend: degrade,
count, and make the count visible — consistent with principle 5.

---

## 5. Piece 2 — odometry without two dedicated rotation sensors

The chain `motion → IPoseSource → Localizer → PilonsOdometry → 2 × IRotation` is rigid:
`Localizer` takes a **concrete** `PilonsOdometry&`, there is no `IOdometry` seam, and
`PilonsOdometry` precondition-requires both a Forward **and** a Lateral wheel.

**LemLib does drive-encoder odometry and shulib currently cannot.** That is a competitive gap, not
only a bench inconvenience.

**The precedent that makes this clean:** `TankKinematics::forward()` already documents (`kinematics/tank.hpp:53`) *"vy ALWAYS
exactly 0 — this drivetrain cannot observe lateral motion."* A lateral-less odometry is the odometry
counterpart of a decision the kinematics layer has already made explicitly and in writing. Say so in
the header; do not invent a new justification.

**Shape:** extract an `IOdometry` seam, then add a drive-encoder implementation behind it.

**ASK BEFORE BUILDING — `R3a-PROGRESS.md` §8.5.1 may delete most of this piece.** Calypso's source
declares two rotation sensors (`horizontal(21)`, `vertical(-20)`, 1.5″ tracking wheels at offsets −4
and 0) that were **specified but never wired**. §8.5.1: *"If those sensors physically exist in a
parts bin, R3b's odometry problem largely evaporates and a POSITION-based auton becomes reachable"*
rather than being gated on a purchase. **Worth asking before anything else** — it is one question and
it can save the larger half of this piece.

Build the `IOdometry` seam regardless: it is the right shape either way, and `Localizer` taking a
concrete `PilonsOdometry&` is a coupling defect independent of which sensors exist.

**OPEN — settled by R3a's IMU convention group:** which way the odometry integrates. The IMU sign
(HA-02/03/04/05) decides it, and getting it wrong mirrors the world.
**OPEN — settled by R3a's measured loop rate:** HA-123's `vMax × dt` re-expression and the
`OdoStallCheck` window.

---

## 6. Piece 3 — the absent-device ruling ⟵ **executable now, zero dependencies**

### 6.1 The defect

`chassis/robot_context.hpp:62-73` precondition-requires `gps`, `tags` and `vision` non-null. Most
robots — including the bench bot — have none of the three. `src/main.cpp:166-167` currently ships:

```cpp
shulib::hal::fake::FakeTagSource tags{};   // stub until the M3 vision pipeline (R2 camera)
shulib::hal::fake::FakeVision vision{};    // stub until the M3 vision pipeline (R2 camera)
```

**A test fake and an absent-device null object are different things, and only one belongs in a
competition binary.** `hal/fake/` is the test-double tree: `FakeTagSource` carries `setTags()` and
`clear()` mutators and a `std::vector` member whose whole purpose is to be driven by a test. Linking
a competition binary against it means the robot ships with test scaffolding in it, and — worse —
means the source no longer distinguishes *"this robot has no camera"* from *"a test will inject
tags here later."*

### 6.2 THE RULING — keep the preconditions, supply explicit absence

**Do NOT relax the non-null preconditions.** The constructor's own comment states why they exist:
so a mis-wired robot *"fails at construction naming the handle it is missing, instead of
dereferencing null halfway through an auton."*

Making the three pointers optional would relax exactly that, and would push a null check into every
consumer — reintroducing the failure the composition root exists to prevent.

With null objects the precondition **stays and gets stronger**:

- an **accidental omission** is still `nullptr` and still fails loudly at construction;
- a **deliberate absence** is spelled out at the call site: `.gps = &absentGps`.

The distinction between *"I forgot"* and *"this robot has none"* becomes visible in the source,
which is the whole point.

### 6.3 What to build

Three classes, in **`include/shulib/hal/`**, beside `null_sink.hpp` — **not** in `hal/fake/`.

**Naming — a real decision, recorded.** Recommend `AbsentGps` / `AbsentTagSource` / `AbsentVision`.
*Rejected alternative:* `NullGps` / `NullTagSource` / `NullVision`, for consistency with `NullSink`.
The semantics genuinely differ: `NullSink` is a **working sink that discards**, whereas these say
**no such device is installed**. `.gps = &absentGps` at the composition root reads as a statement
about the robot's configuration rather than about a software pattern, and the composition root is
exactly where this chunk's problem lives. Consistency loses to precision here; say so in the header.

| Class | Contract |
|---|---|
| `AbsentGps` | `hasFix()` → **always false**. `pose()` → a fixed **finite** pose (origin). `rmsError()` → a large **finite**, non-negative constant. |
| `AbsentTagSource` | `tags()` → `{}` (an empty `std::vector` does not allocate). |
| `AbsentVision` | `objects()` → `{}`. |

**`IGps`'s own header already blesses this**, and it is the strongest argument in the chunk — quote
it in the new header: *"A permanently false hasFix() is a SUPPORTED mode, not a fault: Driving
Skills runs on a field with no GPS strip and the estimator dead-reckons."* `AbsentGps` is therefore
contract-legal with **zero changes to any consumer**.

### 6.4 Making absence VISIBLE — principle 5

An absent GPS that quietly reports no-fix forever looks identical to a GPS that is merely off the
strip. **Silent degradation is a bug** (E1's principle 5).

**RULING: the composition root announces it once at boot.** T5's missing-SD-card ruling is the
precedent — construction succeeds, the degradation is honest, and *"the composition root /
diagnostics layer owns saying it out loud."*

**REJECTED: adding `isPresent()` to `IGps` / `ITagSource` / `IVision`.** Verified against the
Freeze Register (`roadmap.md`, row F4): **F4 is ✅ LOCKED 2026-06-19**, and its text names
`IGps` and `IVision`+`ITagSource` explicitly among the ten frozen runtime HAL interfaces. This is
not a "check first" — it is settled. `hal/telemetry_sink.hpp:16` records what a seam change costs:
it *"would have broken NullSink, FakeTelemetrySink, and every future implementer at once."* The
composition root already knows what it wired, so no seam change is needed and none is justified.

**And the converse ruling, stated because silence in that register reads as "frozen" (D2's lesson):
the three new classes need NO Freeze Register row.** They are *implementations* of already-locked
interfaces, not new contracts. F13 settles the precedent in the register's own words — *"adapters are
implementations, not contracts, and no adapter surface freezes here either."* Adding an
implementation does not touch F4's locked row, exactly as F11/F13/F14's F4-additive siblings did not.

### 6.5 THE LIVENESS TRAP — the sharpest hazard in this piece

`localization/apriltag_corrector.hpp:228-230` states, deliberately:

> *"A poll that sees NOTHING is still information — 'we looked, the camera is alive, there was no
> tag' — and is recorded as such, which is how the off-camera path stays distinguishable from a
> dead vision task."*

**An `AbsentTagSource` that returns an empty vector is byte-indistinguishable from a live camera
seeing no tags — so the corrector will record "camera alive, no tags" about a robot that has no
camera at all.** That is a false diagnostic, produced by the very class added to make absence
honest, and it defeats a distinction the corrector was explicitly built to preserve.

**RULING: an absent source must not be polled.** The composition root knows the device is absent, so
it must not install or drive the corrector for it — rather than installing one and relying on empty
returns to be harmless. Absence is a **wiring** decision, resolved once at construction; it is not a
per-tick value that happens to look like nothing.

*(This also keeps `droppedTags()` and the corrector's liveness counters meaning what their headers
say they mean.)*

### 6.6 Also update

`src/main.cpp` — swap the two fakes for the absent objects, add an `AbsentGps`, and **fix the
comment block at lines 44-45** (*"Still fake, deliberately"*). Left as-is it becomes a lie the moment
this lands.

---

## 7. Test requirements

Per the roadmap's testing discipline: every test names a bug it would catch. Trivial confirmations
do not count.

**Piece 3 — the load-bearing set:**

1. **Absence costs nothing and changes nothing.** A `Localizer` driven with an `AbsentGps` produces
   numbers **identical** to one with no GPS corrector installed at all. This is the test that proves
   the ruling; everything else is supporting.
2. **`AbsentGps` invariants, swept over many ticks:** `hasFix()` false every time; `pose()` finite
   every time; `rmsError()` finite **and** non-negative every time. The finiteness assertions are
   not ceremony — `finite_guard_test.cpp` exists because this tree has been bitten.
3. **`AprilTagCorrector` + `AbsentTagSource`:** never corrects, never faults, over N ticks.
4. **Omission is still loud.** `RobotContext` built with a literal `nullptr` for `gps` / `tags` /
   `vision` still fires its precondition. **This is the negative test that proves absence is
   explicit rather than permissive — do not skip it.**
5. **A fully-absent context constructs**, and every accessor returns a usable reference.
6. **The liveness trap (§6.5).** Assert that a context wired with an `AbsentTagSource` does not
   drive an `AprilTagCorrector` at all — the corrector's "we looked and saw nothing" liveness
   record must NOT be produced for a robot with no camera.
7. Extend `test/robot_context_test.cpp:90-92` rather than replacing it — the fake-default assertions
   still hold for the test path.

**Required mutation checks** (deliberately break it, confirm RED, restore):

| # | Mutation | Must go RED |
|---|---|---|
| 1 | `AbsentGps::hasFix()` returns `true` | the dead-reckon-identity test (1) |
| 2 | `AbsentTagSource::tags()` returns one fabricated tag | the corrector no-op test (3) |
| 3 | `RobotContext` drops the `gps` precondition | the omission test (4) |
| 4 | `AbsentGps::rmsError()` returns `-1.0` | the invariant sweep (2) |
| 5 | the composition root polls the corrector anyway when the source is absent | the liveness-trap test (6) |

---

## 8. Definition of Done

- [ ] M1's DoD met — **identical numbers in a host test and on the V5, swapping only
      `RobotContext`** — and M1's badge flips
- [ ] A v2 tank auton runs on the robot **under the library's own steering**
- [ ] Piece 1: N motors per side all commanded, all braked, none silent; the 7-motor case from
      `R3a-PROGRESS.md` §4.1 passes with its negative control
- [ ] Piece 2: an `IOdometry` seam exists; drive-encoder odometry runs without pods
- [ ] Piece 3: the three absent-device classes exist in `hal/`, `main.cpp` ships no `hal/fake/`
      type, and the `main.cpp:44-45` comment is true again
- [ ] An absent tag/vision source is never polled; no false liveness record is produced (§6.5)
- [ ] All five mutation checks confirmed RED, then restored green
- [ ] HA-18 and HA-52's threshold half settled; HA-112's "drivable" half settled
- [ ] CI green: host suite, the PROS-free guard, and the ARM cross-compile of every v2 header

---

## 9. Documentation contract

> **Non-negotiable and enforced.** These are not conventions — six of them fail the BUILD, not just
> CI, and `test/CMakeLists.txt:135-142` states there is deliberately no opt-out: *"a gate you can
> switch off is not a gate."* Adding three public headers touches almost all of them.

### 9.1 The eight gates — run every one, from the repo root

```sh
python3 tools/api_doc_tool.py self-test
python3 tools/api_doc_tool.py check-coverage       # a public member with no /// FAILS, naming it
python3 tools/api_doc_tool.py check-fresh          # docs/api/ must match a fresh generation
python3 tools/api_doc_tool.py check-examples
python3 tools/api_doc_tool.py check-removability   # no public doc may link into docs/internal/
python3 tools/briefing_status.py check
python3 tools/doc_staleness_audit.py self-test
python3 tools/doc_staleness_audit.py
```

**All eight PASS on the tree this brief was written against (2026-08-18, `d78003c`).** Any failure
is therefore this chunk's, not inherited — see §11.

### 9.2 What three new public headers OBLIGE

1. **Every public member needs a `///` comment.** `check-coverage` fails the build naming the
   member. The generator and the gate share one parser precisely so they cannot disagree — there is
   no "document it later."
2. **Regenerate the reference:** `python3 tools/api_doc_tool.py generate`, then **commit
   `docs/api/`**. Otherwise `check-fresh` fails. Expect three new pages
   (`absent_gps.md`, `absent_tag_source.md`, `absent_vision.md` or whatever the filenames resolve to).
3. **Add each new page to `mkdocs.yml`'s nav** — the `api/` section, alphabetically within its
   group. `R3a-PROGRESS.md` §12.1 records why: *"DOCS2 measured that a page absent from the nav"* is
   effectively invisible. A generated page that nothing links to is not published.
4. **`python3 tools/briefing_status.py generate`**, then commit `PROJECT-BRIEFING.md`. Its STATUS
   block is **derived from the repo** (header counts, test counts); adding headers and tests makes it
   stale and `check` exits 1. **Never hand-edit inside the generated markers.**
5. **Re-run `doc_staleness_audit.py` and fix what it names.** Its numeric scope is
   `docs/guide/*.md`, `docs/cookbook/*.md`, `README.md`, `docs/README.md`, `docs/faq.md`,
   `test/README.md` — new tests move case/assertion counts in any of those.

### 9.3 Accurate-but-UNGATED — update these by hand or they silently rot

The audit's numeric scope does not cover them, so nothing will fail. Update them anyway.

| File | Claim | Why it moves |
|---|---|---|
| `docs/README.md:87` | *"All 1,625 of them"* (public entities) | three new headers add entities |
| `docs/internal/docs-publishing.md:20` | *"117 of them … covering 1,625 public entities"* | page count **and** entity count |

*(Header count is currently **148**; the ARM gate reports it. No gated live doc claims that number
today — do not introduce one.)*

### 9.4 Do NOT retro-edit the historical record

`chunks/*-PROGRESS.md` and `chunks/*-COMPLETED.md` record **what was true when written** and are
deliberately outside both audit scopes. `R3a-PROGRESS.md` §4.1's 7-motor probe stays as it is; §9.1
already records the correction. **Append, never rewrite** — that convention is why §9.1 could
withdraw an earlier ruling honestly instead of overwriting it.

### 9.5 The rest

- A live **`R3b-PROGRESS.md`, created FIRST and appended as the work happens** — not written at the
  end. It is the honest record if the chunk is interrupted.
- Every new header carries its reasoning at the density of the files around it. `hal/null_sink.hpp`
  is the model for how a small class documents a load-bearing omission.
- `hardware-assumptions.md`: HA-18, HA-52 (**threshold half only — HA-52 is owned R3/R4, R4 owns the
  rest**) and HA-112 (**"drivable" half only**) updated with measurements, not prose. Keep the
  register bidirectionally reconciled with the tree — zero orphans, grep-verified (A4's standard).
- `roadmap.md`: the **"you are here" pointer and M1's badge move in the same commit** as the work.
  Under-claim — `[~]` for partial. M1's badge flips **only** when its DoD is actually met.
- `build-order.md`: R3b's status line — **and fix its stale 7-motor/five-dead sentence** (§4).
- `docs/guide/14-what-it-cannot-do-yet.md`: it currently states the library has never driven a
  robot. If R3b closes, **that chapter is wrong until it is edited.**
- The **C7 removability check** if any public doc is touched: grep public docs for `internal/`,
  `chunks/`, `RESUMING`, `build-order` — must be empty.

## 10. Landmines

1. **Do not put the absent classes in `hal/fake/`.** That is the entire defect.
2. **Do not relax the `RobotContext` preconditions.** §6.2 is the ruling; going the other way
   silently un-fixes the bug this chunk exists to fix.
3. **`rmsError()` must be FINITE.** `std::numeric_limits<double>::infinity()` violates the `IGps`
   contract and will trip the finite guards. Use a large finite constant and document the choice.
4. **`main.cpp:44-45` becomes a lie** the moment the fakes are swapped. Update it in the same commit.
5. **Do not touch the three L0 seams** without checking F4's scope in the Freeze Register first.
6. **Pieces 1 and 2 have open decisions gated on R3a's Batch 1.** Building before measuring is
   guessing first — trap 1, which has bitten six chunks. If B1 has not come back, do §6 and stop.
7. **An empty return is not the same as an absent device (§6.5).** This is the trap most likely to
   be walked into, because the naive `AbsentTagSource` looks obviously correct and quietly
   manufactures a false liveness record.
8. **The assertion count is a function of a dirty working tree** (`R3a-PROGRESS.md` §12.2):
   `-dirty` is exactly 6 characters, the hash is captured at CMake **configure** time, and
   committing changes the answer. Reconfigure on a clean tree before trusting any baseline.
9. **Do not "fix" `R3a-PROGRESS.md` §4.1's 7-motor number.** It is a historical measurement, and
   §9.1 already supersedes it. Rewriting records is the failure this project's append-only
   convention exists to prevent.
10. Run every verification command **from the repo root** — the ARM header-list `sed` is anchored
   (`s|^include/||`) and silently emits nothing if run elsewhere.

---

---

## 11. Verification baseline — what green looked like before this chunk

Measured 2026-08-18 on a **clean tree** at `d78003c`, so any red Fable sees is its own:

| Check | State |
|---|---|
| `api_doc_tool.py` self-test / coverage / fresh / examples / removability | **5 × PASS** |
| `briefing_status.py check` | **PASS** |
| `doc_staleness_audit.py` self-test + audit | **2 × PASS** |
| `include/shulib/*.hpp` | **148 headers** |
| `docs/api/` | **117 pages** |

**Re-establish this baseline on a CLEAN tree before trusting any number** (landmine 7): the build
hash is captured at CMake *configure* time and `-dirty` is exactly 6 characters, so a dirty tree
shifts the assertion count and a plain `cmake --build` can carry a stale hash indefinitely.

---

*Companion to [`build-order.md`](../build-order.md) §R3b and [`R3a-tank-bench-validation.md`](R3a-tank-bench-validation.md).
Created 2026-08-18.*
