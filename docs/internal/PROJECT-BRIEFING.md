# shulib v2 — the complete project briefing

> **Paste this entire file into a new session.** It is the full context transfer: who I am, what the
> library is, everything that exists, how we work, what has bitten us, and what comes next. It is
> maintained as a document, not regenerated ad-hoc — if something here is wrong or stale, fix it here
> and say so.
>
> **Related and NOT duplicated here:** [`RESUMING.md`](RESUMING.md) owns the protocol in its canonical
> form · [`build-order.md`](build-order.md) owns the chunk order and "Current position" ·
> [`../roadmap.md`](../roadmap.md) owns milestones and the Freeze Register ·
> `chunks/*-COMPLETED.md` own the per-chunk records. **Link, don't restate** — any number copied
> between documents is a number that will go stale.

---

## 1. Who I am and what this is

I'm the VEX U programming chair at Seton Hall. **shulib v2** is a holonomic-native autonomous library
for VEX U robots, at `/home/gonzei/projects/shulib`, branch `shulib-v2`.

**The thesis:** LemLib — the library most VEX teams use — is **tank-only**, with no holonomic support
and no sensor fusion. We run X-drive and H-drive robots. shulib wins by being holonomic-native
(translation and rotation decoupled and simultaneous), fusion-based (odometry + IMU + GPS + AprilTags
behind swappable policies), and **usable by people who cannot write C++** — four tiers of progressive
disclosure, each a strict superset of the one below.

**It is a public repo.** shulib is meant to be usable by teams outside SHU, so the standards are not
academic: someone else will read this and trust it.

### What I want from you

Depth and honest pushback. **Tell me when something's a bad idea. Don't agree with me to be
agreeable.** Cut zero corners — always the thorough path over the convenient one. Verify before
claiming, and write your own check rather than trusting a report.

**Go slowly on documentation. I'd rather be slow and right than fast and wrong** — judges and future
team members will read all of it and won't know enough to notice when it's lying to them. **If a
sentence becomes hard to write honestly, that difficulty is the finding.**

Work thoroughly, and **push hard** — but the standards in §7 do not bend for speed.

---

## 2. Read this first, in this order

1. **[`RESUMING.md`](RESUMING.md)** — the working protocol, canonical.
2. **[`ORIENTATION.md`](ORIENTATION.md)** — plain-English, no jargon: what this is and where it stands. Start here if the chunk letters mean nothing to you yet.
3. **[`chunks/R1b-COMPLETED.md`](chunks/R1b-COMPLETED.md)** and **[`chunks/R1a-BENCH-SESSION.md`](chunks/R1a-BENCH-SESSION.md)** — the most recent chunk record, and the first time this library met real hardware. *(The `HANDOFF-*.md` files are older and now historical — R1a and R1b landed after them.)*
4. **[`build-order.md`](build-order.md)** — "Current position" (long, worth it), then "The order at a
   glance".
5. Run these and read the output:
   ```sh
   git log --oneline -20
   git status
   ./build/test/shulib_tests | tail -3
   ```

### Current status — GENERATED, never hand-typed


> **How this document stays true — read this once, and read the last paragraph twice.**
>
> Staleness here has two causes and **only one of them is mechanical.**
>
> 1. **The generated block below** carries everything derivable from the repo: position, next
>    chunk, interrupted chunks, suite counts, headers, hardware assumptions, the Freeze Register,
>    both CI guards run live, freeze-pin counts, verification harnesses, outstanding
>    `TODO(chunk)` markers, honest partials, every deliberately-skipped test **by name**, and
>    flagged open defects. It is rewritten by `python3 tools/briefing_status.py generate`, and a
>    **build gate fails if it drifts.**
> 2. **Three narrower classes are checked by `tools/doc_staleness_audit.py`**, also at build time:
>    a document stating a figure the repo contradicts, two documents disagreeing with each other,
>    and a document naming a file that does not exist. It has a self-test proving each detector
>    can fire.
>
> **Everything else in this document is UNGATED, and that is now stated rather than disguised.**
> Whether the trap list, the standards, the architecture tour or "what each chunk taught" are
> still *true* is not checked by anything, because nothing can check it.
>
> There used to be a `DURABLE-REVIEWED-AT` stamp here claiming to force a human re-read.
> **It was removed at R1a, because a build agent moved it by itself** — which is not misbehaviour
> but the design: the chunk that triggers the gate (by writing its own completion record) is the
> same actor that can silence it, so it holds the trigger and the key, and no gate can tell a
> person re-reading from a machine editing a marker. A checkbox anything can tick is worse than
> no checkbox: a green one reads as assurance. The audit that replaced it checks **less** than the
> stamp claimed to, and everything it checks is real.
>
> **So this is a reading task, and it is yours.** After each chunk, ask: did it add a trap?
> invalidate a standard? change a layer or a seam? make any sentence here false? Nothing will
> remind you. Four of this project's staleness incidents were found exactly this way and by
> nothing else.

<!-- BEGIN GENERATED STATUS — regenerate with: python3 tools/briefing_status.py generate -->

> **Everything below is DERIVED FROM THE REPO, not typed.** A build gate
> (`briefing_status.py check`) fails if it drifts, so it cannot go stale
> silently. Each line names where it comes from — re-check any of it.
>
> **Nothing here is derived from the commit graph** — no HEAD SHA, no
> commits-ahead count, no commit list. Those three cannot be written into a
> committed file without lying, because each is a function of the commit
> being made; they made this gate unsatisfiable and are deliberately gone.
> Run `git log --oneline -20` and `git status` for them — §2 says so already,
> and a command cannot go stale.

**Position:** 25 of 44 chunks complete

- **Next up:** DEFECTS1 — triage and resolve the 83 API defects DOCS2 reported, then the RELEASE to main, then R3 — first motion.  
  *(source: `build-order.md`'s `Next:` pointer)*
- ⚠️ **INTERRUPTED CHUNK(S): DEFECTS1** — a `-PROGRESS.md` exists with no completion record. **Read that log before anything else.**
- **Suite:** 1,151 cases / 1,505,874 assertions, 3 skipped — **RED — 2 case(s) failing**  
  *(source: `./build/test/shulib_tests`. Assertion counts flatter — they measure seeds swept. Mutation results are the measure this project trusts.)*
- **Public headers:** 148  *(source: `find include/shulib -name '*.hpp'`; the ARM gate compiles every one)*
- **Hardware assumptions:** 123 registered, **7 settled** — next free is **HA-124**  
  *(source: `docs/hardware-assumptions.md`. Nothing is settled until hardware measures it.)*

**Completed chunks** *(source: the `-COMPLETED.md` records, which are the project's own definition of done)*:

> `A1` · `A2` · `A3` · `A4` · `C1` · `C2` · `C3` · `C4` · `C5` · `C6` · `C7` · `C8` · `D1` · `D2` · `D3` · `DOCS1` · `DOCS2` · `E1` · `E2` · `E3` · `E4` · `F1` · `F2` · `R1a` · `R1b`

**Freeze Register** *(source: `docs/roadmap.md`, which owns it)*:

| Row | Contract | Status |
|---|---|---|
| **F1** | Coordinate frame — origin = field center, +X right, +Y… | ✅ LOCKED |
| **F2** | Accuracy targets — heading < 1.0° (hard); ~1.0″ pose; ~… | ✅ LOCKED |
| **F3** | Units & Angle semantics — internal inches + radians + s… | ✅ LOCKED |
| **F4** | HAL interface signatures — the 10 runtime HAL interface… | ✅ LOCKED |
| **F5** | IKinematics contract — twist (vx,vy,ω) ⇄ wheels + desat… | ✅ LOCKED |
| **F6** | Public Chassis API — the whole facade surface, by group… | ✅ LOCKED |
| **F7** | robotProfile sub-schema inside .vexbot — drivetrain/odo… | 🎯 pending |
| **F8** | paths[] sub-schema + command-id vocabulary inside .vexbot | 🎯 pending |
| **F9** | SHUL/2 telemetry wire protocol (v1) — the wire serializ… | 🎯 pending |
| **F10** | Public Routine API (the Tier-2 recipe layer) — construc… | ✅ LOCKED |
| **F11** | Mechanism seam + operation contract — hal::IMechanism /… | 🚧 open by design |
| **F12** | Sequence engine (sequence/run_guard.hpp) — RunGuard (th… | 🚧 open by design |
| **F13** | Driver-input seam (hal/controller.hpp) — IController (a… | 🚧 open by design |
| **F14** | Digital-input seam (hal/digital_in.hpp) — IDigitalIn (o… | 🚧 open by design |

⚠️ **Register rows F1–F5 are NOT chunks F1–F5.** Row F2 is the accuracy targets; chunk F2 was the sequence engine. The collision is in shipped code (`spec/accuracy.hpp`). **Never edit rows F1–F5.**

**Guards, run just now:** PROS-free PASS · sim-layering PASS · freeze pins: F6 50, F10 52

**Verification harnesses:** `verify-d1.sh`, `verify-d2.sh`, `verify-d3.sh`, `verify-e2.sh`, `verify-e3.sh`, `verify-e4.sh`, `verify-f1-chunk-selfcheck.sh`, `verify-f1.sh`, `verify-f2.sh` *(the reviewer's, not the chunk's — a chunk must not rewrite one)*

**Honest partials:** 6 `[~]` items in the roadmap, each naming its owner *(under-claiming is a standard here, so a nonzero count is health, not debt)*

**Deliberately skipped tests (3)** — each is evidence that does not exist yet, usually pending hardware:

- `test/accuracy_spec_test.cpp` — [acceptance][M3] end-of-60s fused pose within the row-F2 targets
- `test/accuracy_spec_test.cpp` — [acceptance][M3] vision docking nests a 1.6in pin within kDockedPositionError
- `test/gps_conversion_test.cpp` — gpsSensorPose: FIELD-CAL axis oracle — bench-measure before trusting

**No open defects** are flagged with the 🔴 convention in `docs/internal/`.

**What just happened:** run `git log --oneline -20`. It is deliberately not reproduced here — see the note at the top of this block.

<!-- END GENERATED STATUS -->

**If `git status` is dirty, a chunk was interrupted.** Read that chunk's `-PROGRESS.md` — appended in
real time, so it is an honest record of exactly how far the work got — before doing anything else.
**Never start a new chunk on a dirty tree.**

---

## 3. The governing constraint, and it governs everything

**The library has never driven a robot.**

On 2026-08-12 the package was built, uploaded and booted on a V5 brain. It constructed its whole
object graph on ARM and printed its diagnostics banner over USB, including a live `strafeAuthority`
query through the frozen facade. **It drove nothing, because it could not** — every motor and sensor
in that binary was a fake.

Since R1a (2026-08-13) the `hal/pros` adapters EXIST — **fourteen** after R1b — and
`src/main.cpp` wires the R1a ten. On 2026-08-13 the bench runbook **was run**: eight physical
motors commanded at +2.0 V through the real `ProsMotor`, physical sensors read, and seven
unit-scale beliefs settled — on ONE robot, ONCE. **And the constraint stands unchanged: no
control loop has ever closed, no wheel has ever turned under the library's own steering, no
path has ever been followed.** The adapters are host-proven against `test/pros_shim/`, which
tests them against our *beliefs* about PROS (HA-94…122) and can never test the beliefs;
R1b's five mechanism-sensor adapters have not touched a physical device at all.
"The adapters exist", and now "a bench session ran", must not drift into "it works on a
robot" any more than "it booted" was allowed to.

The 2026-08-12 run proves the build/upload/boot path works and that no host-only assumption breaks
on ARM. It proves **nothing** about motion, accuracy or control — and it predates the adapters.

**Do not let "it booted" drift upward into "it works on a robot."** That distinction is currently
defended in six places across the README and the guide. Keep it defended — it is the single easiest
way for this project to start lying.

Everything is validated against the **A2 host plant**: a simulator that converts voltage into motion
behind the unmodified HAL fakes, made hostile at A3 (drift, garbage windows, sentinels, sag, slip,
latency, jitter — reproducible from a seed). It proves **logic, not constants**. The hardware-assumptions register is the
inventory of everything it cannot prove; **its size and settled count are derived live in the
generated block above and in the register's own status line — this sentence deliberately
carries neither, because it has gone stale here twice.**

---

## 4. The architecture — what exists, by layer

```
L0  hal/          the hardware seams (the ONLY thing that differs robot/sim/test)
L1  math/ units/  frames, angles, poses, typed quantities
L2  control/      Pid, Feedforward, TrapezoidProfile, ExitGroup, Watchdog
    kinematics/   IKinematics + X-drive / H-drive / tank presets, desaturate
    localization/ odometry, Localizer, fusion policies, correctors
    motion/       IMotion + primitives + MotionScheduler
    manipulation/ mechanism operations (F1)
    sequence/     the run guard (F2)
    diag/         DebugRecord, sinks, faults, blackbox, run summary
L3  chassis/      Chassis facade (frozen F6) + Routine recipe layer (frozen F10)
                  RobotContext — the composition root, the one object that differs
```

**Key files worth knowing by name:**

| File | What it is |
|---|---|
| `chassis/chassis.hpp` | The public facade. **FROZEN (F6).** Blocking verbs + `drive(speeds, Frame)` |
| `chassis/routine.hpp` | Tier-2 fluent recipe chain. **FROZEN (F10)**, except `then()` |
| `chassis/robot_context.hpp` | Composition root. Not frozen; grows additively |
| `motion/motion.hpp` | `IMotion` + the tick / cancel / wait-for-live contracts + `applyCancelSafeState` |
| `motion/motion_scheduler.hpp` | The **only loop in the tree**. `tick`/`waitUntilSettled`/`waitUntil`/`cancel`/`async`, re-entrancy guards, the fault policy |
| `localization/localizer.hpp` | Fused estimate + categorical quality; heading is IMU-owned |
| `localization/i_fusion_policy.hpp` | The swap point: complementary (default) ↔ EKF |
| `hal/mechanism.hpp` | `IMechanism` (F1) — two virtual members, `applySafeState()` + `name()`, plus a claim token |
| `manipulation/mechanism_op.hpp` | Bounded, tickable mechanism operations |
| `sequence/run_guard.hpp` | The run-scoped deadline + guaranteed end-of-run action (F2) |
| `version.hpp` | API version + the breaking-vs-additive policy. The mechanism behind every freeze |
| `spec/accuracy.hpp` | The **F2 register row** targets (NOT chunk F2 — see §8) |

**Two CI guards, both mutation-proven:** nothing in `include/shulib/` may include `<pros/*>` (except
`hal/pros/`, path-anchored since R1a), and nothing outside `sim/` may include `shulib/sim/`.
**Four build-time doc gates** run at 1% of the build: undocumented public entity, stale generated
reference, drifted example, internal link leak. Plus the briefing gate and the staleness audit.

**Since DOCS2 the coverage gate covers the WHOLE tree** — every public entity under
`include/shulib/` (minus `sim/` and `**/fake/`), not just `Chassis` and `Routine`. The target list
is a glob, so a header added later is covered because it exists rather than because someone
remembered; `docs/api/` is a page per header; and the `mkdocs.yml` API nav is generated between
markers and byte-checked, because a page absent from the nav was measured to publish *unreachable*
with exit code 0. **Being gated is not being frozen** — that distinction is stated on the
reference's own front page and in the amended register rows F11–F14.

**R1a amended the PROS-free guard for the first time** — `include/shulib/hal/pros/` is the one
exempt path, **anchored to that exact path** (in CI *and* in this tool's own guard_state, which
had to learn the same amendment). The cheap `--exclude-dir=pros` spelling was measured missing a
violation planted at `include/shulib/localization/pros/`, and the miss was re-demonstrated live
during R1a's guard proof. The ARM compile gate is **not** amended: its generated glob picked the
adapters up (124 → 139 headers) and they compile clean behind the two-flag diagnostic fence.
One R1a trap for every future adapter: **PROS's `common.mk` resolves includes with `-iquote`
only**, so an adapter's PROS includes must use the QUOTED form — angle brackets compile
everywhere except the one build that matters.

---

## 5. What each completed chunk actually delivered

*(The authoritative list of which chunks are done is in the generated status block above.
This section is the durable part: what each one taught or proved, which does not change.)*

**Phase A — the ground to stand on**
- **A1** `DebugRecord` + `TermSink` + fault discipline. Faults log and recover, never crash.
- **A2** The host plant + closed-loop sim harness. *The simulator is the robot you don't have* — and
  for estimator work it's better, because you possess ground truth.
- **A3** Hostile sensor fakes. Every V5 misbehaviour class injectable and composable. **Found 3 real
  Localizer defects.**
- **A4** Hardware Assumptions Register + the ARM compile gate.

**Phase C — make it move**
- **C1** `IMotion` + primitives. `MoveToPose` runs three **decoupled** per-axis loops — the holonomic
  thesis, mutation-proven. Every motion watchdog-bounded.
- **C2** `MotionScheduler`. One active motion, structurally. Cancel safe state defined (0 V + Brake).
  Fault policy decided (abort on ODO_STUCK only).
- **C3** H-drive + the full pseudo-inverse. **The 15″ H-bot runs the same motion code as the 24″
  X-bot, unmodified.**
- **C4** The `Chassis` facade — one composition root, structural command-id stamping.
- **C5** Per-motion results, session header, run summary. The run is legible end to end.
- **C6** Legacy salvage audit — 34 files classified, port list empty by audit.
- **C7** Cutover: `legacy/` deleted, `make` produces an uploadable package.
- **C8** The 15-chapter manual, with examples compiled and quoted verbatim so CI catches rot.

**Phase D — make it usable**
- **D1** Tier-2 recipe API (`Routine`) — eager fluent chain; the facade held its second consumer with
  zero changes.
- **D2** **F6 freeze** — and the first freeze in the project that means something mechanical.
- **D3** Cookbook + generated API reference + **F10 freeze**. All four doc gates moved to build time.

**Phase E — bound the drift**
- **E1** SD blackbox + flight recorder + estimator introspection. A run is recoverable with no laptop.
- **E2** `GpsCorrector` — drift is bounded, and the claim is sized to the evidence.
- **E3** `AprilTagCorrector` — absolute heading correction.
- **E4** 5-state SE(2) EKF. **It LOST the comparison its own DoD demanded** (0.351″ vs 0.225″, losing
  7 of 8 seeds), so the complementary filter stays the default. **That result was measured against
  invented noise — R4 could flip it.**

**Phase F — sequencing**
- **F1** The `Mechanism` seam + fakes. `IMechanism` earns its existence on **one verb**
  (`applySafeState`) because that is what F2's guard needs across heterogeneous mechanisms.
  Per-mechanism **declared** safe state (a lift holds; an intake coasts), with a physics split: motor
  ops safe on every exit, discrete actuators keep their commanded state on success and safe only on
  cancel — *a clamp whose safe state is "open" would fling its game piece the instant a grab
  succeeded.*
- **F2** The `sequence/` engine + guaranteed end-of-run action. See §11.

**Phase R — the robot (opened at R1a)**
- **R1a** The `hal/pros` adapters (drivetrain + driver + the shim). What it taught: adapter glue IS
  host-testable — against beliefs, never past them (the shim's honest limit, its defaults made
  ADVERSARIAL so a shared model cannot quietly cancel its own errors); PROS's build resolves
  includes `-iquote`-only (quoted includes are mandatory in adapters); a device keeps another
  program's configuration, so configure-and-read-back is the idiom; and an observationally
  equivalent mutant (M4b, get_heading at the heading() seam) is closed with a textual contract
  pin, not a theater test. `IController` landed as row F13 (NOT frozen); T1 was delivered here.
- **R1b** The five mechanism-sensor adapters + the `IDigitalIn` seam (row F14). Trap B:
  `pros::Distance::get_distance()` returns an **in-band 9999** for "no object" — 393.66 inches of
  phantom wall, not an error.

**The documentation chunks — they ship no behaviour and both changed what the project can claim**
- **DOCS1** Read every public and internal document end to end; 60-odd stale claims corrected,
  including a **red release gate** found before a single document was read. Its lesson, and the
  one worth carrying: *the hole was not in either list — it was in the GAP between two lists that
  were maintained by hand and believed to agree.*
- **DOCS2** The reference over the whole public API: **2 documented types → 1,625 public entities
  across 115 headers**, all gated, with the target list a glob and the site nav generated. What it
  taught is in §9 trap 6, and it is the most transferable thing in this file: **the brief's own
  measurement was wrong because the instrument was blind, not because anyone miscounted.** It also
  produced 83 API defects, reported and unfixed, in `chunks/DOCS2-API-DEFECTS.md` — several worth
  a bench session's attention.

---

## 6. The Freeze Register — what is locked and what is not

*(The live register — every row and its current status — is in the generated block
above, parsed from `roadmap.md`, which owns it.)*

**F11 and F12 say "not frozen" out loud** because D2 learned that silence in that register reads as
"frozen". They freeze after a second real consumer stresses them — the build → second consumer →
freeze path F6 and F10 both took.

**Enforcement is structural.** `test/f6_signature_pin_test.cpp` and
`test/routine_signature_pin_test.cpp` fail the build **naming the row and the member** if a frozen
signature drifts — including a member **losing or gaining** `noexcept`.

---

## 7. How we work — the chunk loop

**One chunk at a time**, in `build-order.md` order.

### 1. Write a detailed brief
`docs/internal/chunks/<CHUNK>-<slug>.md`. **F2's brief is the best template.** It carries:
- why this chunk is here in the order
- what already exists to build on — **read the actual files**, cite `file:line`
- scope: **in**, **out** (naming the chunk that owns it instead), **explicitly rejected**
- the load-bearing constraints, each with its reasoning
- **the tensions to rule on, each requiring a rejected alternative**
- test requirements including the required mutations
- a DoD checklist
- **an explicit documentation scope** — which chapters change, and why
- landmines

**The brief is where the thinking goes.** Every recent brief found a contradiction nobody had noticed
— one where `build-order.md` demanded something a previous chunk had already ruled out, one where the
project's own flagship example had never compiled.

**Where it's cheap, MEASURE BEFORE YOU BRIEF.** F2's brief carried fifteen executable measurements,
and several overturned the design I would otherwise have written into it. shulib is header-only, so a
standalone probe compiles in seconds without touching the build directory:
```sh
g++ -std=gnu++20 -I include -I test -I test/vendor probe.cpp -o probe
```

### 2. Commit the brief

### 3. Run it with Fable
`Agent` tool, `model: "fable"`, `subagent_type: "general-purpose"`, `run_in_background: true`.

The prompt is a skill in itself. It must carry: **required reading in order**; **the non-negotiable
constraints restated inline** (the agent reads the prompt more carefully than the attachment); the
test bar with mutations named; **the verification commands verbatim**; the documentation contract;
**"do not commit, do not push"**; **"create `<CHUNK>-PROGRESS.md` FIRST and append continuously"** (I
watch it with `tail -f`, and it is what makes an interrupted chunk recoverable); and **the traps that
have bitten before, phrased for this chunk specifically**.

### 4. Verify independently — never take the report at face value
**This is the step that makes the process real.**

```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6

# GUARD 1 is PATH-ANCHORED — the amendment R1a made and this file did not carry.
# The unanchored form printed here until DOCS2 (2026-08-14) reports 21 hits on a
# CLEAN tree, because hal/pros/ exists to include PROS. A verification command
# that fails when nothing is wrong is worse than none: the next session has to
# guess whether the tree or the instruction is at fault, which is exactly what
# the broken ARM `sed` did to DOCS1. §4 described the amendment correctly all
# along; this command had not been re-run since.
if grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib \
     | grep -v '^include/shulib/hal/pros/'; then echo "GUARD1 FAIL"; else
  echo "GUARD1 PASS — PROS-free outside hal/pros/"; fi
if grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' \
     include/shulib; then echo "GUARD2 FAIL"; else echo "GUARD2 PASS — core is sim-free"; fi

find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo 'int main(){return 0;}' >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
```

`docs/internal/verify/` holds per-chunk harnesses. **Read `verify/README.md` first** — its central
lesson is *a red gate is a question, not a verdict.*

**Then do the part no script can do:** write your own oracle for the chunk's most load-bearing claim,
with **a negative control** — run the same scenario with the feature absent and prove your instrument
can tell the difference. Every time this has been done it found or confirmed something real.

**The harness at `docs/internal/verify/verify-<chunk>.sh` belongs to the REVIEWER.** F1's agent
overwrote the reviewer's harness with its own self-check and destroyed the independence of the audit.
Say so in the chunk prompt.

### 5. Commit
Only after verifying. Conventional style; the body explains the **reasoning** and names honest
partials. Trailer:
```
Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
```
**Do not push** unless asked.

---

## 8. Standards that do not bend

- **Evidence, not vibes.** A checkbox flips only with cited evidence (file + test + counts). `[~]` for
  partial, naming the chunk that owns the rest. **Under-claim before over-claiming.**
- **Tests must try to break the code.** Every test names, **in a comment, the bug it would catch** —
  not what it checks.
- **Mutation testing is mandatory** for load-bearing logic: break it, **rebuild**, run, **observe
  red**, restore. *A mutation you reasoned about but did not execute does not count.* **Gate the
  runner on build success** — a non-compiling mutation read off a stale binary looks green.
  **A mutation that stays GREEN is a hole in the suite and the most valuable thing you can find.**
- **Assertion counts flatter.** 1.5M assertions measures seeds swept, not independent checks. The
  README says so explicitly. Mutation results are the measure this project trusts.
- **Rule 4:** a chunk that finds a flaw in an earlier chunk **fixes it there**, not around it.
- **Clean-room:** re-derive, never port. Re-deriving `arcStep` this way caught a real legacy bug.
- **Documentation is a deliverable**, continuous, and never the thing compressed to save a chunk.
- **Plain English, no slogans.** If a line sounds like a tagline, cut it.
- **`docs/internal/` must stay cleanly removable** — no public doc may link into it. `main` is a squash
  of `release/v2` with that directory dropped, and the docs site publishes from `main`.
- **Constants are provisional.** Anything invented gets an `HA-nn` entry with blast radius and the
  measurement that would settle it. **The next free number is in the generated block above** —
  it is not restated here, because it was wrong here for two chunks. Don't tune parameters until a sweep passes —
  that is fitting the test, not the code.

---

## 9. The traps — every one of these has actually happened

### 1. Shared models cancel their own errors (six chunks and counting)
If a test shares a conversion, a camera model, a kinematic model, a motion model **or a clock** with
the code under test, the error **cancels on both sides and the test passes while proving nothing.**
C1, C3, C4, E2, E3 and F2's campaign were each bitten.

The counter: a from-scratch oracle with hand-computed literals, **plus a negative control**. Examples
that earned their keep — E2's oracle found the *existing* test imported the very constant it was
pinning; E3's independent projector confirmed a reversed corner winding returns `valid=true` with
heading 180° wrong; F2's verification measured 0.0 in of post-deadline travel *and* 360 in with the
guard absent, which is what makes the zero mean anything.

### 2. A biased skeptic manufactures a false all-clear
A red-team this session reported **23 findings and 0 survivors** — because the refuting agents were
told to "default to refuted". **Two of the "refuted" findings were real** and both changed a brief.
*A red gate is a question; a green verdict from a skeptic you biased is also a question.* If you use
adversarial verification, make the verifier argue both sides and report confidence, never a verdict.

### 3. Prose goes stale in ways no gate can see
The four doc gates are solid and have **no opinion about whether a sentence is still true.** Found by
*reading*, in single sessions: a chapter claiming 57 assumptions when there were 67; a README claiming
659 tests twenty lines from a claim of 867; an API chapter describing an exception that had been
deleted; a chapter claiming no cookbook existed after D3 shipped one. **Budget reading time, not just
gate time.**

### 4. Frozen means frozen
F6 and F10 change only by major version bump plus a migration note. If a pin fires on a signature,
**stop and report** — that's a breaking change to argue, not an edit to make. Note that the doc
**freshness** gate fires *before* the pins, naming the wrong problem; D3 documented this and it bit
again at F1's verification.

### 5. Name collisions between chunks and freeze rows
Chunk F1/F2/F3/F4 vs **Freeze Register rows F1–F5** (frame, accuracy, units, HAL, kinematics — all
LOCKED). **The collision is in shipped code**: `spec/accuracy.hpp` is titled "the F2 accuracy
targets". **Never edit rows F1–F5.** The verify harnesses gate on this.

### 6. An instrument's silence read as a zero (DOCS2)

DOCS2's brief carried a careful measurement of its own scope — 971 public items, 399
undocumented — taken by running the generator's own parser over the tree rather than estimating.
Every number reproduced exactly. **Every number was also 60% low**, because that parser's opener
regex required `{` immediately after an optional `final`, so it could not see a type with a
base-class list, an enum with an explicit underlying type, or *anything* declared at namespace
scope. It did not warn, or skip, or count wrong: it never saw them. **Three LOCKED contracts —
the coordinate frame, the accuracy targets, the units vocabulary — produced no output at all**,
and had produced none since D3.

The blindness never fired because the two headers it was ever pointed at happen to have no base
class. *A gate that has only ever run on the easy case has not been tested; it has been lucky.*

**The counter, and it is now this project's standard for any parser-shaped tool:** check it
against a *different* implementation. Diffing the parse against clang's real AST — a front end
sharing no code, no regex and no assumption with it — found four more bugs the tool's own
self-test could not, all of them fields mislabelled as functions and then named after part of
their own initializer. A hand-rolled parser validated against its author's expectations is
trap 1 wearing a different coat.

Two smaller forms of the same lesson, from the same chunk:
- **A mutation that stays GREEN is the campaign's real output.** Three of sixteen survived the
  first pass, and all three survived because the *fixture* lacked the shape, not because the code
  was right. One survived because three character literals happened to *balance*, so a blind
  parser landed in the right place by luck.
- **The tool reproduced its own headline failure inside itself.** A `///<` continuation line also
  starts with `///`, so it became the *next* member's documentation — three enumerators on a
  published page each carried a confident sentence about a different value, and the coverage gate
  scored all three as documented. No gate could see it. A reader did.

### Process failures that actually happened
- A mutation runner **piped into `head`** took a SIGPIPE mid-campaign and left a header with a line
  deleted. Trap `PIPE`, count what you ran, and **never** `git checkout` a file holding uncommitted
  work.
- A **non-compiling mutation nearly read as green** off a stale binary (C4). Gate on build success.
- A **results table was nearly fabricated** before the measurement existed (E4). Write
  `PENDING MEASUREMENT`, measure, then write the number.

---

## 10. What good work looks like here

The best moments in this project have all been **retractions**:
- **E2** measured its accuracy claim, found 7/8 and 6/8 rather than 8/8, discovered a shorter scenario
  that *did* pass 8/8, and **refused to switch to it** as scenario-shopping.
- **E3** reported a headline bug, realised it had **inferred a sign** from an absolute value, and
  **struck its own finding.**
- **E4** reported that **the EKF lost** the comparison its own DoD demanded, did not tune toward the
  expected answer, and made the simpler filter the default on the strength of that measurement.
- **F2** found a green mutation *by planning the campaign* and closed it before running.

**A chunk that reports only wins has not looked hard enough.** Measure first, then claim, and size the
claim to the evidence. A test that disagrees with you is doing its job — consider that your
expectation was wrong before you edit the assertion.

---

## 11. F2 in detail — the most recent chunk, and the one most likely to be over-quoted

`sequence/run_guard.hpp` owns the run-scoped deadline and the **guaranteed end-of-run action**. D-8
(the routine watchdog E1 re-homed) and the end action are **one primitive with two policies**.

It is an `ITickPacer` decorator, **inert until `run()`**. At the caller's `endActionAt` it cuts the
active motion with zero latency, **refuses** every later motion (a latch — cancel-only expiry measured
*inert and marginally counterproductive*), performs cancel-all strictly before the caller's end
action, and enforces an unconditional `hardStopAt` floor.

**The library refuses to know your strategy.** No field coordinate, no park pose, no default lead
time, no default match length. Both instants and the action are caller-supplied. A team that ends
somewhere else, or doesn't park at all, is not fighting the library. `endInMidfield` is F4's, with the
students' own numbers.

**What it cannot do, and this is written in the header and guide ch. 14 rather than buried:** claim
real-brain timing margins (loop rate and PROS latency are invented until R4); preempt pure user code
(no background tasks exist); cut the frozen F10/F6 waits, which pay their full remainder — **a limit
that is tested, not hidden**; or end an unconditional retry loop. And **"zero post-deadline travel" is
a simulator result** — the plant is memoryless, so on a robot with mass the honest claim is *"no new
commanded motion"*.

**Verified by four independent reviewer probes**, each with a negative control: the guarantee held
across three stalls (0.23 in from the caller's pose; 95 in away with no guard), the latch measured
0.0 in against 360 in unguarded, the no-defaults audit translated the manoeuvre 1450 in off any VEX
field and got agreement to 9.1e-12 in, and the C2 fix regressed nothing.

---

## 12. What is left

*(The completed list and the position are in the generated block above; this table is the
shape of the remainder, not a count — a count here goes stale every chunk.)*

| Phase | Chunks | Gate |
|---|---|---|
| **The release to `main`** | next | none — DOCS1 and DOCS2 are both done; one open question, whether to push |
| **R — Robot arrival** | R2–R6 *(R1a, R1b done)* | **hardware (available now)** |
| **T — Driver control** | T2–T3 *(T1 delivered at R1a)* | none |
| **G — No-code authoring** | G1–G4 | G1 ungated; G2–G4 need VexBuilder |
| **H — Ecosystem** | H1–H3 | H2 needs VexBuilder's sim |
| **F′ — Scoring primitives** | F3–F4 | hardware + final mechanism decisions |
| **E′ — Field accuracy** | E5–E6 | hardware + a field |
| **I — Second robot** | I1–I2 | both robots |

**Milestones:** M0 ✅ · M1 🎯 closes at R1–R3 · **M2 open on exactly one clause — a robot that moves**
· M3 at E6 · M4 at F4 · M5 at G4 · M6 at H3 · M7 largely delivered (one honest gap: no beginner has
read the cookbook cold).

---

## 13. THE IMMEDIATE TASK — the release to `main`

> *This section is the one a pasted-in session acts on first, so it is the one most worth
> distrusting. It said "THE IMMEDIATE TASK — R1b" for a day after R1b shipped, and
> "— DOCS1" for a day after that.*

**DOCS1 and DOCS2 are both DONE.** The immediate task is **the release to `main`** — the merge
that publishes docs.shurobotics.com — and after it **R3** (day-one validation, which walks the
assumptions register top to bottom and **closes M1 and M2's on-robot clause, open since June**),
then **R4** (sensor characterization — replaces A3's invented noise magnitudes with measured ones;
**the highest-information work available**, because E4's headline result rests on invented
numbers).

**Why the two documentation chunks came first:** merging to `main` is what publishes. Anything
wrong in the documentation becomes *publicly* wrong at that moment, under the project's own name.
The documentation pass is not tidying before a release — **it is the release gate.**

**DOCS1 (2026-08-14)** read every public and internal document end to end and corrected 60-odd
stale claims, including a release gate that was already red. **DOCS2 (2026-08-14)** took the
generated reference from 2 documented types to **1,625 public entities across 115 headers**, all
gated. Both records are in `chunks/`.

**One DOCS2 result belongs in this section rather than only in its record**, because it changes
what a measurement here is worth: the brief's own scope numbers were taken with the generator's
parser, and that parser was **structurally blind** — it could not see a type with a base-class
list, an enum with an underlying type, or anything at namespace scope, so three LOCKED contracts
produced no output at all and the real debt was 60% larger than measured. Nothing was wrong; the
instrument reported silence and silence read as zero. When a number here comes from a tool, ask
what that tool cannot see. (See trap 6 in §9.)

**The two open questions that ride with the push are down to one**: whether to push at all.
HTTPS was settled on 2026-08-14 — certificate issued, enforced, verified from outside with full
certificate validation.

### The R1 record — kept because its lessons are still live

**R1 split into R1a + R1b on 2026-08-13**, split by consumer. R1's old scope line said "the 9 F4
adapters"; reading the tree found **fifteen** — four earlier chunks each wrote "R1 owns this" into
their own headers (`digital_out.hpp:24`, `char_sink.hpp:11`, `block_sink.hpp:34`,
`line_display.hpp:8`) — plus two new seams, a host test shim, `main.cpp` and a bench session.

- **R1a — delivered:** `IClock`, `IMotor`, `IRotation`, `IImu`, `IGps`, `IBattery`, `ICharSink`,
  `ILineDisplay`, `IController` (new seam, Phase T's T1 delivered here), plus the real tick pacer,
  the shim, both guard changes, `main.cpp`. Host-proven only; the bench runbook is R3's opening.
- **R1b — delivered (2026-08-14):** `IDistance`, `IOptical`, `IDigitalOut`, `IBlockSink`, and a new
  `IDigitalIn` seam. Everything a *mechanism* needs. Host-proven only — **none of the five has ever
  touched a physical sensor**; runbook steps 16–20 exist to settle HA-113…122. R1b inherited R1a's fence pattern, shim framework, adapter idiom
  (screen → hold-last-good → expose a faultedReads counter), the QUOTED-include rule, and **trap B**:
  `pros::Distance::get_distance()` returns an IN-BAND 9999 for "no object" (not PROS_ERR — 393.66
  inches of phantom wall), and `get_confidence()` is only meaningful above 200 mm.

**One correction to a list that has been quoted repeatedly:** `ITelemetrySink` needs **no** PROS
adapter — it already has `NullSink`/`TermSink`/`SdSink`. What is PROS-backed is one layer below it,
`ICharSink` and `IBlockSink`. The old "nine F4 adapters" list double-counted one and hid the other.

**Three things measured before the brief was written, each of which changed it:**

1. **The PROS SDK headers fail the ARM compile gate's own flags** — `-Wshadow` in `pros/rtos.hpp:1903`,
   `-Wsign-conversion` in `pros/motors.hpp:77` and `pros/rotation.hpp:58`. Third-party source we do
   not own, so `build-order.md`'s DoD ("the ARM build compiles them") was **not achievable as
   written**. A two-flag `#pragma GCC diagnostic` fence around the include block fixes it, and with
   the fence the **ARM gate needs no exclusion at all** (124 headers + a PROS-including adapter
   compile clean as one TU). A negative control proved the fence does not protect shulib's own code.
2. **The PROS-free guard must be path-anchored**, exempting exactly `include/shulib/hal/pros/`. The
   cheap `--exclude-dir=pros` form was measured **missing** a violation planted at
   `include/shulib/localization/pros/` — D3's "a gate's exclusion list is where its holes live",
   demonstrated rather than argued.
3. **The adapters are host-testable, which the tree assumed they were not.** An adapter was measured
   compiling, linking and *running* on the host against a hand-written `pros/` shim under full strict
   flags. Without it the adapters are the only code 1.5M assertions cannot reach.

*(The "what comes next" sentence that used to close this section has moved to the top, where a
resuming session reads it first.)*

---

## 14. Sensors — the recommendation, and the question I owe you an answer to

**Seams that exist:** `motor` (encoder + velocity + current + temperature), `rotation`, `imu`, `gps`,
`distance`, `optical`, `vision` (AI Vision: objects *and* AprilTags), `battery`, `clock`,
`digital_out`, `line_display`, and — since R1a — `controller` (axes/buttons/isConnected, row F13,
not frozen; PROS adapters now exist for **all fourteen** — the R1a nine at R1a and the mechanism
five at R1b).

**Seams that DO NOT exist:** **digital input** (limit switches, bumpers — R1b's), analog input
(potentiometers, line trackers), legacy 3-wire (ultrasonic, optical shaft encoders).

**For the new competition bots, ranked by accuracy-per-port:**

1. **IMU** — one, centre-mounted, isolated from vibration and away from motors. Heading is the largest
   error term and is IMU-owned by design; the `< 1.0°` target lives or dies here.
2. **Two tracking wheels (forward + lateral) on Rotation Sensors** — unpowered, spring-loaded. These
   are what make odometry survive wheel slip, and X/H drives slip.
3. **GPS** — the only absolute position source; bounded drift over a 60 s run depends on it. **Needs
   the field strip; Driving Skills has none**, so it must degrade gracefully (E2 handles this).
4. **AI Vision** — AprilTags give absolute *heading* (E3 proved the corrector), objects give bearings
   for manipulation. R2 is already scoped for it.
5. **Optical on each scoring mechanism** — the sensor-confirm on every grab/place that §14 calls
   non-negotiable: *never advance on a failed grab.*
6. **Distance** — dock confirm and a cheap wall-reference cross-check.
7. **Limit switches for lift homing** — **only if the lift homes against a stop.**

**THE LIFT-HOMING QUESTION — asked 2026-08-13, and the answer was *not decided yet*.** Ruling: **add
the digital-input seam anyway**, in R1b. Cheap now, expensive to discover at R3 with the robot on the
bench. F1 already built **stall** detection (current + velocity), so if the answer comes back "stall"
the seam is a small unused sibling — that cost is accepted and stated rather than discovered.

**This is a deliberate departure from F1's standard** (`IMechanism` earned its two virtual members on
one verb a real consumer needed) and it stays defensible only on two conditions, both written into
R1b's scope: a digital input has one degree of freedom, and every other question about it —
what "pressed" means physically, whether there is a validity channel — is already ruled by
`IDigitalOut`'s header. And **R1b ships the seam, the adapter and the fake, and NO homing routine**:
homing is F3's, which is season content the students author (§16).

**The question is still open and still owed an answer before F3.**

---

## 15. Hardware reality — MEASURED 2026-08-13, not assumed

The robot available is the **old COMPETITION bot**, not the "tank practice bot with not much on it"
this section described until the first bench session read the device list off the brain. That
correction is the reason the section now leads with measurements.

**What is actually on it** (`R1a-BENCH-SESSION.md` has the full record):

- **Twelve motors** on ports 1, 2, 3, 5, 11, 12, 13, 14, 15, 16, 17, 18 — all green (18:1) cartridges
- **Drive: LEFT 15/16/17/18, RIGHT 11/12/14**, established by spinning one side at a time by hand
- **IMU on port 4**, alive and calibrating
- A radio, and **possibly an ADI expander — UNVERIFIED** (read from an out-of-range port index; re-check before relying on it). The brain's 8 built-in ADI ports exist regardless.
- **NO rotation sensors. NO GPS.**

**Two mechanical faults found, one proven twice:** port 13 spins free (18 mA against its
neighbours' 500–950 mA, and it never moved when its wheels were turned by hand) — a chain or gear is
off. Port 16 under-reports its side-mates by ~20% at an inconsistent ratio — probable slip,
undiagnosed. A drive motor that under-reports travel biases odometry quietly.

**What this robot CAN settle:** the platform layer — the adapters, unit conversions, brownout
threshold, real loop rate under load, PROS call latency, IMU drift and calibration window, and the
**tank** kinematics path.

**What it CANNOT settle, contrary to what this section used to claim:** anything needing a **GPS**
(noise, latency, the field-cal axis oracle in `gps_conversion_test.cpp` — still skipped) or
**tracking wheels** (encoder refresh, the odometry push test). It has neither.

**And it validates none of the holonomic thesis** — no strafe authority, no pseudo-inverse, no
per-axis decoupling, no H-drive geometry. Those entries stay open until a competition-season robot
exists, and the register **says so** rather than implying coverage.

**Two traps:** **never carry gains across chassis** — kS/kV/kA and PID gains are mass-, friction- and
geometry-dependent, and a measured-looking wrong number is worse than an honest placeholder. And
**record anything measured on this robot with that provenance** — one robot, once, is an observation,
not proof of portability.

**A third trap, learned the hard way at the first session:** `registry_get_plugged_type()` is
**ZERO-indexed** (0–20) while every PROS device API is **ONE-indexed** (1–21). Mixing them yields
plausible wrong answers, never an error — it produced a confident report of two dead devices that
were both healthy. No shipped adapter uses the registry; keep it that way.

**Bench notes:** I'm in `dialout`; the brain enumerates as `/dev/ttyACM0` (system) and `/dev/ttyACM1`
(user). `pros terminal` needs a real TTY, so under automation wrap it
(`script -qec "pros terminal" /dev/null`) — a raw read of `ttyACM1` yields nothing, because the user
port is framed rather than plain serial.

---

## 16. The guardrail

The **library** is built this way. The **competition routines** (Phase F′) and **authored paths**
(Phase G) are strategy **my students must write and be able to defend** — those chunks deliver
primitives and engines and stop short of authoring the season's content. Architect, teach and review;
**don't write their auton.**

This is why F2 ships no park pose and why `buildStack` / `matchLoadCycle` / `endInMidfield` /
`strategyMode` belong to F4, not to the engine.

---

## 17. Release and the docs site

`shulib-v2` → merge into `release/v2` → drop `docs/internal/` → squash that tree onto `main`. The
history shows it: `git log --first-parent main`.

The site publishes from `main` to **docs.shurobotics.com** via GitHub Pages. Publishing from `main` is
deliberate: the development record cannot reach the public site **by construction** rather than by
configuration. `tools/prepare_site.py` refuses to build if an internal doc would be published.

**The rule that must survive:** the publish job runs `python3 tools/api_doc_tool.py check-fresh`
before rendering. Publishing a stale reference is worse than publishing none, because a published
document looks authoritative.

**RESOLVED 2026-08-14 — the site is HTTPS-only.** Let's Encrypt certificate for
`docs.shurobotics.com` issued, "Enforce HTTPS" on, and HTTP now 301-redirects to HTTPS
(verified from outside with full certificate validation, not `curl -k`).

**The cause is worth keeping, because the fix is not obvious.** DNS was correct and the custom
domain was set, but `https_certificate` was **null** — GitHub had never requested a certificate
at all. It validates DNS when the domain is *set*, and the domain had been set before DNS
resolved; nothing retries on a schedule, so it would have sat on plain HTTP indefinitely.
Re-saving the same value is a no-op. **Removing the custom domain and re-adding it** forces
revalidation, and the certificate was approved within seconds. If Cloudflare's proxy is re-enabled, SSL mode must be
**Full (strict)**, never Flexible.

---

## 18. Open decisions for me

1. **Which sensors are going on the robots?** Still open. The lift-homing half is **answered** (§14):
   undecided, so R1b built the digital-input seam anyway — but **the real answer is still owed
   before F3**, which is the chunk that would consume it.
2. **Whether to push.** Still live, and now the ONLY question riding with the release — HTTPS was
   settled on 2026-08-14 (certificate issued, enforced, verified from outside with full
   certificate validation; §17 has the cause, which is not obvious). `main` is current only
   through Phase D — verified by content, never by commit distance — so E1, E2, E3, E4, F1, F2,
   R1a and R1b are all absent from it, and none of that work is public. Merging publishes all of
   it at once, including both documentation passes and **117 new API pages**.
3. **R5 timing.** Building `tools/sysid` on the tank bot gives a validated tool and throwaway numbers,
   since gains never transfer across chassis. It may be worth deferring until a competition robot
   exists and slotting G1 or H1 in instead. **This is the one place in the order I want your judgment
   rather than mine.**
