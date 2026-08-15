<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/sequence/run_guard.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `run_guard.hpp`

RunGuard — the run-scoped deadline owner and the guaranteed END-OF-RUN ACTION.

This header declares **4** types (29 members).

Extracted from [`include/shulib/sequence/run_guard.hpp`](../../include/shulib/sequence/run_guard.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class GuardedWaitResult`](#enum-class-guardedwaitresult)
  - [`Satisfied`](#guardedwaitresult-satisfied)
  - [`TimedOut`](#guardedwaitresult-timedout)
  - [`RunExpired`](#guardedwaitresult-runexpired)
- [`struct RunGuardConfig`](#struct-runguardconfig)
  - [`endActionAt`](#runguardconfig-endactionat)
  - [`hardStopAt`](#runguardconfig-hardstopat)
  - [`mechanisms`](#runguardconfig-mechanisms)
  - [`validate`](#runguardconfig-validate)
- [`struct RunGuardReport`](#struct-runguardreport)
  - [`scoringCut`](#runguardreport-scoringcut)
  - [`endActionRan`](#runguardreport-endactionran)
  - [`endActionSucceeded`](#runguardreport-endactionsucceeded)
  - [`floorFired`](#runguardreport-floorfired)
  - [`postExpiryCancels`](#runguardreport-postexpirycancels)
  - [`anonymousClaimsReleased`](#runguardreport-anonymousclaimsreleased)
  - [`pacesSeen`](#runguardreport-pacesseen)
  - [`scoringEnded`](#runguardreport-scoringended)
  - [`endActionEnded`](#runguardreport-endactionended)
- [`class RunGuard`](#class-runguard)
  - [`RunGuard`](#runguard-runguard)
  - [`RunGuard (overload 2)`](#runguard-runguard-2)
  - [`RunGuard (overload 3)`](#runguard-runguard-3)
  - [`operator=`](#runguard-operator-eq)
  - [`operator= (overload 2)`](#runguard-operator-eq-2)
  - [`~RunGuard`](#runguard-destructor-runguard)
  - [`pace`](#runguard-pace)
  - [`expired`](#runguard-expired)
  - [`remaining`](#runguard-remaining)
  - [`running`](#runguard-running)
  - [`waitFor`](#runguard-waitfor)
  - [`pause`](#runguard-pause)
  - [`run`](#runguard-run)

<a id="enum-class-guardedwaitresult"></a>

## `enum class GuardedWaitResult`

```cpp
enum class GuardedWaitResult
```

The guard's wait verdict (banner: verdict honesty). DISTINCT from WaitResult on purpose: RunExpired is a fact about the RUN, not the wait, and it must be impossible to read as success.

*enum class, declared at [`include/shulib/sequence/run_guard.hpp:146`](../../include/shulib/sequence/run_guard.hpp#L146).*

<a id="guardedwaitresult-satisfied"></a>

### `GuardedWaitResult::Satisfied`

```cpp
Satisfied
```

the predicate became true before any deadline

*enumerator, declared at [`include/shulib/sequence/run_guard.hpp:147`](../../include/shulib/sequence/run_guard.hpp#L147).*

<a id="guardedwaitresult-timedout"></a>

### `GuardedWaitResult::TimedOut`

```cpp
TimedOut
```

the WAIT's own timeout elapsed first (run still live)

*enumerator, declared at [`include/shulib/sequence/run_guard.hpp:148`](../../include/shulib/sequence/run_guard.hpp#L148).*

<a id="guardedwaitresult-runexpired"></a>

### `GuardedWaitResult::RunExpired`

```cpp
RunExpired
```

the RUN's deadline passed — stop scoring; wins ties with Satisfied (a satisfied-but-expired wait must still halt the chain — the measured predicate-folding trap)

*enumerator, declared at [`include/shulib/sequence/run_guard.hpp:149`](../../include/shulib/sequence/run_guard.hpp#L149).*

<a id="struct-runguardconfig"></a>

## `struct RunGuardConfig`

```cpp
struct RunGuardConfig
```

One run's schedule + reach. Everything is REQUIRED and caller-supplied: there is deliberately no default here to invent (banner).

*struct, declared at [`include/shulib/sequence/run_guard.hpp:156`](../../include/shulib/sequence/run_guard.hpp#L156).*

<a id="runguardconfig-endactionat"></a>

### `RunGuardConfig::endActionAt`

```cpp
units::Time endActionAt{0.0}
```

When scoring stops and the end action starts, measured from run() start. The caller computes the lead ("park takes ~6 s") — the library has no number to offer that would not be an invented one.

*field, declared at [`include/shulib/sequence/run_guard.hpp:160`](../../include/shulib/sequence/run_guard.hpp#L160).*

<a id="runguardconfig-hardstopat"></a>

### `RunGuardConfig::hardStopAt`

```cpp
units::Time hardStopAt{0.0}
```

The unconditional safe floor, measured from run() start. At this instant every device is forced safe and everything — the end action included — is refused. Must be >= endActionAt; the gap is the end action's runway (equal instants = zero runway: legal, and the end action's motions will all be refused — supply distinct instants if it must MOVE).

*field, declared at [`include/shulib/sequence/run_guard.hpp:167`](../../include/shulib/sequence/run_guard.hpp#L167).*

<a id="runguardconfig-mechanisms"></a>

### `RunGuardConfig::mechanisms`

```cpp
std::span<hal::IMechanism* const> mechanisms{}
```

Every mechanism the run touches (may be empty). cancel-all reaches operations through the claim's registered claimant (mechanism.hpp); list a mechanism here or the guard cannot see it at the deadline.

*field, declared at [`include/shulib/sequence/run_guard.hpp:171`](../../include/shulib/sequence/run_guard.hpp#L171).*

<a id="runguardconfig-validate"></a>

### `RunGuardConfig::validate`

```cpp
void validate() const
```

Reject a schedule that could not mean anything, before a run arms: both instants finite, endActionAt > 0, hardStopAt >= endActionAt, and no null in `mechanisms`. run() calls it at the door, so a bad number is a loud error at the call site instead of a deadline that silently never arrives.

*function, declared at [`include/shulib/sequence/run_guard.hpp:177`](../../include/shulib/sequence/run_guard.hpp#L177).*

<a id="struct-runguardreport"></a>

## `struct RunGuardReport`

```cpp
struct RunGuardReport
```

What one guarded run did — the guard's own account, kept SEPARATE from every motion verdict the caller's code saw (banner: verdict honesty).

*struct, declared at [`include/shulib/sequence/run_guard.hpp:191`](../../include/shulib/sequence/run_guard.hpp#L191).*

<a id="runguardreport-scoringcut"></a>

### `RunGuardReport::scoringCut`

```cpp
bool scoringCut = false
```

True iff the deadline latched scoring off (false: scoring returned on its own and the end action started early — the caller was done).

*field, declared at [`include/shulib/sequence/run_guard.hpp:194`](../../include/shulib/sequence/run_guard.hpp#L194).*

<a id="runguardreport-endactionran"></a>

### `RunGuardReport::endActionRan`

```cpp
bool endActionRan = false
```

the callable was invoked (always, unless a throw unwound run())

*field, declared at [`include/shulib/sequence/run_guard.hpp:195`](../../include/shulib/sequence/run_guard.hpp#L195).*

<a id="runguardreport-endactionsucceeded"></a>

### `RunGuardReport::endActionSucceeded`

```cpp
bool endActionSucceeded = false
```

its verdict, per the four accepted return types

*field, declared at [`include/shulib/sequence/run_guard.hpp:196`](../../include/shulib/sequence/run_guard.hpp#L196).*

<a id="runguardreport-floorfired"></a>

### `RunGuardReport::floorFired`

```cpp
bool floorFired = false
```

hardStopAt arrived during the run

*field, declared at [`include/shulib/sequence/run_guard.hpp:197`](../../include/shulib/sequence/run_guard.hpp#L197).*

<a id="runguardreport-postexpirycancels"></a>

### `RunGuardReport::postExpiryCancels`

```cpp
int postExpiryCancels = 0
```

Scheduler cancels the guard performed after the latch (the first is the cut; the rest are refused retries). Zero plant travel either way.

*field, declared at [`include/shulib/sequence/run_guard.hpp:200`](../../include/shulib/sequence/run_guard.hpp#L200).*

<a id="runguardreport-anonymousclaimsreleased"></a>

### `RunGuardReport::anonymousClaimsReleased`

```cpp
int anonymousClaimsReleased = 0
```

Anonymous claims force-released at cancel-all (should be zero — register claimants).

*field, declared at [`include/shulib/sequence/run_guard.hpp:203`](../../include/shulib/sequence/run_guard.hpp#L203).*

<a id="runguardreport-pacesseen"></a>

### `RunGuardReport::pacesSeen`

```cpp
int pacesSeen = 0
```

pace() calls observed while the run was live. ZERO after a run whose scoring did real work means the Chassis was NOT constructed with this guard as its pacer — the guard was never in the loop and its guarantee never applied (Warn-logged).

*field, declared at [`include/shulib/sequence/run_guard.hpp:208`](../../include/shulib/sequence/run_guard.hpp#L208).*

<a id="runguardreport-scoringended"></a>

### `RunGuardReport::scoringEnded`

```cpp
units::Time scoringEnded{0.0}
```

clock at scoring()'s return, from run start

*field, declared at [`include/shulib/sequence/run_guard.hpp:209`](../../include/shulib/sequence/run_guard.hpp#L209).*

<a id="runguardreport-endactionended"></a>

### `RunGuardReport::endActionEnded`

```cpp
units::Time endActionEnded{0.0}
```

clock at the end action's return, from run start

*field, declared at [`include/shulib/sequence/run_guard.hpp:210`](../../include/shulib/sequence/run_guard.hpp#L210).*

<a id="class-runguard"></a>

## `class RunGuard`

```cpp
class RunGuard final : public motion::ITickPacer
```

The run-scoped deadline owner (file banner). Construct it around the real pacer, give the Chassis the guard AS its pacer, then wrap the whole auton in run(). Inert by construction: until run() is live, pace() is a pure pass-through — zero clock reads, zero behavior change (the D3 §2.1 instruction: a deadline must be opt-in and inert by default; wiring the guard in must not change an existing routine by one tick).  motion::ITickPacer& real = ...;             // plant pacer / R1's delay sequence::RunGuard guard{real}; chassis::Chassis chassis{deps, guard, cfg}; // the guard IS the pacer ... const sequence::RunGuardReport rep = guard.run(chassis, runCfg, [&] { /* scoring: Routine chain, verbs, guard.waitFor(...) */ }, [&] { /* end action: YOUR pose, YOUR re-verify */ return true; });  Not copyable/movable: the Chassis holds a reference to it as the pacer.

*class, declared at [`include/shulib/sequence/run_guard.hpp:229`](../../include/shulib/sequence/run_guard.hpp#L229).*

<a id="runguard-runguard"></a>

### `RunGuard::RunGuard`

```cpp
explicit RunGuard(motion::ITickPacer& inner) noexcept
```

`inner` advances the real world (host: step the plant; robot: delay to the tick boundary) and must outlive the guard.

*function, declared at [`include/shulib/sequence/run_guard.hpp:233`](../../include/shulib/sequence/run_guard.hpp#L233).*

<a id="runguard-runguard-2"></a>

### `RunGuard::RunGuard (overload 2)`

```cpp
RunGuard(const RunGuard&) = delete
```

Pinned where it is constructed: the Chassis holds this object BY REFERENCE as its pacer, so a copy would be paced by nobody and a move would leave the Chassis pacing a corpse. The destructor releases nothing — the guard owns no device and holds only non-owning pointers to the inner pacer and, while a run is live, the chassis's scheduler, clock and telemetry.

*function, declared at [`include/shulib/sequence/run_guard.hpp:240`](../../include/shulib/sequence/run_guard.hpp#L240).*

<a id="runguard-runguard-3"></a>

### `RunGuard::RunGuard (overload 3)`

```cpp
RunGuard(RunGuard&&) = delete
```

*Covered by the comment on [`RunGuard (overload 2)`](#runguard-runguard-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/sequence/run_guard.hpp:241`](../../include/shulib/sequence/run_guard.hpp#L241).*

<a id="runguard-operator-eq"></a>

### `RunGuard::operator=`

```cpp
RunGuard& operator=(const RunGuard&) = delete
```

*Covered by the comment on [`RunGuard (overload 2)`](#runguard-runguard-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/sequence/run_guard.hpp:242`](../../include/shulib/sequence/run_guard.hpp#L242).*

<a id="runguard-operator-eq-2"></a>

### `RunGuard::operator= (overload 2)`

```cpp
RunGuard& operator=(RunGuard&&) = delete
```

*Covered by the comment on [`RunGuard (overload 2)`](#runguard-runguard-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/sequence/run_guard.hpp:243`](../../include/shulib/sequence/run_guard.hpp#L243).*

<a id="runguard-destructor-runguard"></a>

### `RunGuard::~RunGuard`

```cpp
~RunGuard() override = default
```

*Covered by the comment on [`RunGuard (overload 2)`](#runguard-runguard-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/sequence/run_guard.hpp:244`](../../include/shulib/sequence/run_guard.hpp#L244).*

<a id="runguard-pace"></a>

### `RunGuard::pace`

```cpp
void pace() override
```

The pacer seam (banner: how the deadline reaches running code). The deadline checks run BEFORE the world advances — the ordering is load-bearing (0.0000 in vs 10.79 in of post-deadline travel, measured) and pinned by test. Inert pass-through when no run is live.

*function, declared at [`include/shulib/sequence/run_guard.hpp:250`](../../include/shulib/sequence/run_guard.hpp#L250).*

<a id="runguard-expired"></a>

### `RunGuard::expired`

```cpp
[[nodiscard]] bool expired() const
```

True once the CURRENT phase's deadline has passed: endActionAt during scoring, hardStopAt during the end action. The retry-loop idiom: `while (!guard.expired() && ...) { ... }` — an unconditional retry loop is the one stall the guard cannot end (banner, honesty section).

*function, declared at [`include/shulib/sequence/run_guard.hpp:270`](../../include/shulib/sequence/run_guard.hpp#L270).*

<a id="runguard-remaining"></a>

### `RunGuard::remaining`

```cpp
[[nodiscard]] units::Time remaining() const
```

Time left before the current phase's deadline (never negative). During the end action this counts down to the hard stop — the "hold position until the buzzer" budget.

*function, declared at [`include/shulib/sequence/run_guard.hpp:278`](../../include/shulib/sequence/run_guard.hpp#L278).*

<a id="runguard-running"></a>

### `RunGuard::running`

```cpp
[[nodiscard]] bool running() const noexcept
```

True only while run() is executing — scoring OR the end action. That window is exactly when expired(), remaining(), waitFor() and pause() may be called at all (outside it they trip a precondition) and exactly when pace() checks deadlines rather than passing straight through. False before the first run and again the moment run() returns: the robot belongs to the caller then.

*function, declared at [`include/shulib/sequence/run_guard.hpp:289`](../../include/shulib/sequence/run_guard.hpp#L289).*

<a id="runguard-waitfor"></a>

### `RunGuard::waitFor`

```cpp
template <typename Pred> [[nodiscard]] GuardedWaitResult waitFor(Pred&& pred, units::Time timeout)
```

Block until `pred` holds, the wait's own `timeout` elapses, or the run's live deadline passes — the return says which, and RunExpired wins a tie with Satisfied (banner: verdict honesty). Implemented over C2's waitUntil with a composite predicate, so every C2 guard (finite timeout, stalled-pace loudness, no blocking verbs in `pred`) applies unchanged; at the deadline it returns with zero latency and `pred` is not called again — a scoring predicate that ticks an operation stops being ticked the instant scoring time is over (the latch, applied to waits). The active motion keeps ticking throughout, exactly as C2's wait — until the pace-side latch cuts it.

*function, declared at [`include/shulib/sequence/run_guard.hpp:304`](../../include/shulib/sequence/run_guard.hpp#L304).*

<a id="runguard-pause"></a>

### `RunGuard::pause`

```cpp
[[nodiscard]] GuardedWaitResult pause(units::Time duration)
```

Sleep `duration`, or less if the run's live deadline arrives first — Satisfied means the full duration was slept, RunExpired means the run cut it short (TimedOut is unreachable: the sleep IS the timeout). The deadline-aware twin of Chassis::wait / Routine::pause, which cannot be cut (banner: T4) — the "wait for the alliance partner, but never past the budget" beat. `duration` must be finite and > 0.

*function, declared at [`include/shulib/sequence/run_guard.hpp:324`](../../include/shulib/sequence/run_guard.hpp#L324).*

<a id="runguard-run"></a>

### `RunGuard::run`

```cpp
template <typename Scoring, typename EndAction> RunGuardReport run(chassis::Chassis& chassis, const RunGuardConfig& config, Scoring&& scoring, EndAction&& endAction)
```

Execute one guarded run (file banner carries the whole design): 1. arm — capture the run start from the chassis clock; deadlines become absolute instants; the pacer checks go live; 2. `scoring()` — your auton, written against the ordinary frozen surface (Routine chains, blocking verbs, guard.waitFor). It ends when it returns — early because it finished, or because the deadline cut its motions/waits and its chain stopped; 3. cancel-all — active motion cancelled, every listed mechanism's claimant cancelled, claims cleared, declared safe states applied. STRICTLY before step 4 (a stalled operation's unreleased claim would make the end action's own operation throw at start()); 4. `endAction()` — YOUR final act, running in your own call context through the same public verbs, bounded by the hard floor. Return void (always "performed"), bool, ExitReason (Settled = success) or MechanismOutcome (Succeeded = success) — then()'s exact convention. Its verdict lands in the report and the log, never in any motion verdict your scoring code saw; 5. final cancel-all + disarm — the guard hands the robot back safe and goes inert. If scoring() or endAction() THROWS (a precondition — a programming error), the guard cancels-all and safes on the unwind and RETHROWS: a broken program stays loud, and the guard does not drive to a pose on its behalf (converting a throw into a park would hide the bug).  `chassis` MUST be the one constructed with THIS guard as its pacer — the guard has no way to verify that wiring, so it counts: a finished run that saw zero pace() calls Warn-logs that the guarantee never applied (RunGuardReport::pacesSeen).

*function, declared at [`include/shulib/sequence/run_guard.hpp:369`](../../include/shulib/sequence/run_guard.hpp#L369).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 121 lines, click to expand</summary>

```text

 RunGuard — the run-scoped deadline owner and the guaranteed END-OF-RUN ACTION
 (chunk F2, WS8/M4; master plan §14's non-negotiable, D-8's discharge).

 Before this file, every bound in the tree was scoped to ONE motion, ONE wait,
 or ONE mechanism operation. Nothing bounded a whole routine and nothing knew
 when the match ends: eleven steps at the 5 s default timeout is a legal
 200-second routine inside a 15-second match, behaving exactly as designed.
 RunGuard is the owner that outlives a motion: it holds a run-scoped start
 time, two caller-supplied deadlines, a latch, and one caller-supplied action
 that fires when scoring time is up — D-8 (the routine-level watchdog) and
 the end-of-run action as ONE primitive with two policies, not two features.

 ── What "guaranteed" means here, and what it does not (read before quoting) ────────
 F2 proves a SCHEDULING property, against the host plant: a deliberately
 stalled scoring loop still ends with the caller's end action performed, with
 the clock driven to the match limit. It CANNOT claim the timing margin is
 right on a real brain — real loop rate under load and PROS call latency are
 unmeasured until R4 — and NOTHING PREEMPTS PURE USER CODE: there are no
 background tasks, so code that never lets a finished shulib call end its
 loop (an unconditional retry `while` that ignores guard.expired()) keeps
 control forever and the end action runs only when it returns. The guard
 makes every shulib call after the deadline finish quickly and refuses to let
 new ones make progress; it cannot take the CPU from you. Two lateness holes
 are structural and documented at the waits section below.

 ── The library refuses to know your strategy ───────────────────────────────────────
 No field coordinate, no park pose, no default lead time, no default match
 length lives here or anywhere in shulib. The library knows only that SOME
 caller-supplied action fires at SOME caller-supplied instant: a team ending
 tucked against a goal supplies that pose; a team raising a lift under a
 height limit supplies that; §14's own robots supply a park AND a Toggle
 re-verify (the action is a callable, so it composes). A default lead time
 would be an invented number governing whether the robot scores — HA-51's
 invented 5 s default is the cautionary tale, cited on purpose. Both
 instants are REQUIRED, validated, and relative to run() start.

 ── Two instants, because two different things want to happen (T2) ──────────────────
   * endActionAt — STOP SCORING, early enough to still reach the end
     position: the scoring latch fires, the active motion is cut, mechanisms
     are cancelled into their declared safe states (cancel-all STRICTLY
     precedes act: a stalled operation's unreleased claim would otherwise
     make the end action's own operation throw at start() — measured), and
     the end action runs with the remaining runway.
   * hardStopAt — BE SAFE, unconditionally: every device is forced safe and
     everything, the end action included, is refused from here on. Fires
     even if the end action is still running — safety is not negotiable,
     going somewhere is. Safing is the library's to own once a deadline
     exists at all; where to GO is strategy and stays the caller's.

 ── How the deadline actually reaches running code (T1 — the measured design) ───────
 The ONLY seam that regains control mid-motion is the tick pacer, so RunGuard
 IS an ITickPacer: construct it around the real pacer and hand it to the
 Chassis constructor. Every pace() while a run is live checks the deadlines
 BEFORE advancing the world — that ordering is load-bearing and pinned by
 test: check-then-step measured 0.0000 in of post-deadline travel,
 step-then-check measured 10.79 in. At the cut, the guard calls
 scheduler.cancel() from pace() — legal, and since F2 pinned in C2's
 re-entrancy list — which unwinds waitUntilSettled on the same iteration:
 the blocking verb returns Cancelled (its honest verdict: the motion WAS
 cancelled), a Routine records the stop and skips the rest, and control
 returns to the caller, who is now standing after run()'s scoring call.
 run() then performs cancel-all and the end action — in the caller's own
 call context, through the frozen facade's ordinary blocking verbs, so F2
 adds NO second loop owner and never re-implements C2's loop.
   Rejected: a supervisory scheduler.tick() loop (a second loop owner
 duplicating what the existing waits already do); async()-from-pace() to
 hijack the caller's wait into driving the end action (measured to WORK and
 to LIE — the caller's moveTo returned Settled describing the park, zero log
 lines; the worst outcome measured in the campaign); a THROWING pacer
 (measured: 11.4 V under Coast); cancel-only expiry with no latch (measured
 INERT — cancel/restart resets the deceleration and the run arrives sooner).

 ── The latch (T7): after the deadline, motions are refused, not raced ──────────────
 Once endActionAt passes, any motion outside the end action is cancelled at
 the next pace() — one commanded tick, zero plant travel (the ordering pin
 above), every occurrence counted, the first Warn-logged. A retrying caller
 gets each retry cut the same way and the world does not move. The end
 action is EXEMPT (the guard knows when it is running it); the hard floor
 exempts nothing. After run() returns the guard is inert again — the robot
 belongs to the caller (post-run code, driver control) and a guard that kept
 refusing forever would fight the next mode.

 ── The waits (T4): what is deadline-aware and what CANNOT be ───────────────────────
 waitFor()/pause() here are deadline-aware: they return RunExpired at the
 live deadline with zero latency (implemented as a composite predicate over
 C2's own waitUntil — reusing its guards, not its shape) and RunExpired WINS
 a tie with Satisfied, because a chain that reads "satisfied" keeps scoring
 past the buzzer (measured: a deadline folded into a plain predicate returns
 Satisfied, which Routine::waitFor maps to success). The FROZEN F10/F6 waits
 (Routine::pause / Routine::waitFor / Chassis::wait / Chassis::waitUntil)
 CANNOT be deadline-aware without a breaking change: a scheduler-level wait
 checks its predicate and its own timeout and nothing else — 2801 cancels
 were fired into one and all were invisible. The lateness bound for frozen
 code is: THE UNEXPIRED REMAINDER OF THE WAIT'S OWN TIMEOUT at the instant
 the deadline fires, summed over every wait/pause step executed after that
 instant (a Routine usually pays one term: its first post-deadline motion is
 refused and stops the chain; consecutive pauses each pay). Budget waits
 tightly or use the guard's own.

 ── Reaching the mechanisms (T6) ────────────────────────────────────────────────────
 cancel-all walks the caller-supplied span<hal::IMechanism*>: a registered
 claimant (mechanism.hpp's F2 hook) is cancelled — inert, safe, claim
 released; an ANONYMOUS claim is force-released with a Warn (the guard
 cannot render an unknown operation inert — register a claimant); then
 applySafeState() lands the declared state regardless. applySafeState alone
 is NOT enough and the guard never relies on it alone: a live operation
 re-commands its voltage on its next tick, restoring voltage but not brake
 mode — the half-safe `brake=Hold, V=9.0` that passes any mode-only check.

 ── Verdict honesty (T5) ────────────────────────────────────────────────────────────
 GuardedWaitResult is a sequence-layer vocabulary, minted because no existing
 one can say "the RUN's budget expired" distinctly from "this wait's own
 timeout elapsed" — WaitResult::Satisfied does not mean success and
 RoutineStopCause is Tier-2's. Map in, never re-mean. The end action's
 verdict is reported in the RunGuardReport and logged — never as the
 caller's motion verdict, and never silently.

 FREEZES NOTHING. Single consumer today (F4's student-authored routines are
 hardware-gated; D1 ruled G2 out) — the register says "open by design" out
 loud, and nothing here may freeze on one consumer's evidence.
```

</details>
