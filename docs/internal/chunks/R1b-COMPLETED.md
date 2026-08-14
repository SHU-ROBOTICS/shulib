# R1b — COMPLETED (2026-08-14, in the working tree pending review/commit)

> `hal/pros` adapters for the mechanism seams — the second half of R1.
> Written FROM `R1b-PROGRESS.md` (the live log), not instead of it. Nothing committed,
> nothing pushed — verification is the reviewer's, per the standing process.
> Predecessor: R1a. Successor: **R3** (first motion, walked from the extended runbook);
> R2 owns the vision adapter; F3 owns every routine that consumes these seams.

## The one-sentence honest status

The five mechanism-sensor adapters exist and are host-tested and mutation-proven against the
extended PROS shim — the 9999 no-object rule, the construction-actuates rule and the no-card
refusal all implemented and observed under mutation — **and none of the five has ever touched
a physical sensor**: their beliefs are HA-113…122 (two flagged as absent from the vendored
source), the bench runbook grew steps 16–20 to settle them, and the library has still never
driven a robot.

## Built, file by file

**New seam (F4-additive, register row F14, NOT FROZEN — and built on an OPEN question):**

- `include/shulib/hal/digital_in.hpp` — `IDigitalIn`, ONE member (`state()`, the raw level).
  No debounce (a filter constant belongs to the consumer that knows what the switch is for),
  no edge state (reuse `ButtonEdge` per consumer — the T2 ruling), no validity channel
  (`IDigitalOut`'s ruling inherited). The header says out loud that the lift-homing
  switch-or-stall question is UNANSWERED and the seam exists on the cheap-now ruling; if the
  answer is "stall" it is a small unused sibling, a cost accepted in writing.
- `include/shulib/hal/fake/fake_digital_in.hpp` — level + `readCount()`.

**New pure conversions (PROS-free, the imu/gps_conversion pattern):**

- `include/shulib/hal/distance_conversion.hpp` — mm→inches (1/25.4), confidence ÷63
  (clamped), and the NAMED constants the T4 rule hangs on (`kDistanceNoObjectMm = 9999`,
  `kDistanceConfidenceAvailableAboveMm = 200`) with a 3-clause ADAPTER BINDING CONTRACT.
- `include/shulib/hal/optical_conversion.hpp` — hue clamp to the doc's own [0, 359.999],
  sat/bri defensive clamp, proximity ÷255 — with the polarity flagged UNMEASURED (HA-117).

**Adapters — `include/shulib/hal/pros/` (header-only, quoted includes, two-flag fence):**

- `distance.hpp` — `ProsDistance`. THE 9999 RULE (T4): raw 9999 → `confidence() == 0.0`
  (the seam's existing "no usable return" channel), `distance()` stays finite and FAR (the
  honest conversion, 393.66 in — never a held stale object, which is the dangerous direction
  for capture-confirm), no fault raised, `faultedReads()` does NOT count it (an empty intake
  is a normal state). THE CLOSE-RANGE RULE: a valid reading ≤ 200 mm reports confidence 1.0
  — the returned distance IS the detection; the raw channel is not consulted where PROS says
  it does not exist. PROS_ERR (device failure) is a DISTINCT state: hold-last-good +
  faultedReads, and the hold's INITIAL value is the far no-object distance, never 0.0 (a
  sensor dead from boot must not read "object touching the lens"). Deliberately unbound:
  `get_object_size()` / `get_object_velocity()` (each imports another in-band sentinel; no
  consumer).
- `optical.hpp` — `ProsOptical`. Hue/sat/bri pass through (clamp only), proximity ÷255.
  Per-channel sentinel screens on a seam with NO validity channel: PROS_ERR_F on the double
  channels, PROS_ERR on proximity — hold-last-good, never zero (hue 0.0 IS red), never an
  infinity (F4 finiteness). Deliberately unbound: LED pwm (a mechanism decision), gestures,
  raw RGBC.
- `digital_out.hpp` — `ProsDigitalOut`. CONSTRUCTION IS A PHYSICAL ACTION (T3): PROS drives
  the line at construction and defaults it LOW, so the initial state here is a REQUIRED ctor
  argument with no default, documented to agree with `PneumaticMechanism`'s declared safe
  state (test-pinned). One class, two ctors (T6): brain port or {smart, adi} expander pair.
  `commanded()` reports the COMMAND, never the world; refused writes (PROS_ERR) are counted
  in `faultedWrites()` — exposure, not policy.
- `digital_in.hpp` — `ProsDigitalIn`. Binds `get_value()` LEVELS only; the consuming
  `get_new_press()` is banned (HA-121, the ADI sibling of HA-104) and guard-test-pinned.
  PROS_ERR screened: unscreened, INT32_MAX ≠ 0 reads a DEAD port as PRESSED — a homing
  switch permanently "pressed" is a lift climbing into its hard stop. Two ctors (T6).
- `block_sink.hpp` — `ProsBlockSink`, the device E1 named R1b the owner of. Gated on
  `usd_is_installed()`; NO CARD → construction succeeds, `write()`/`flush()` false from the
  first call, `isOpen()` carries the fact for the composition root to report once (T5 —
  a missing card must never stop a robot). Owns the `/usd/` prefix in exactly ONE place:
  callers hand it a BARE name (leading '/' is a loud precondition — the double-prefix trap),
  and the mount root is injectable so host tests run the SAME join+open path against a real
  temp directory. Verbatim fwrite (short write = false, prefix kept — the format decodes to
  the cut); `flush()` = fflush, the platform's strongest persist (no fsync in PROS's surface
  — HA-122's flagged half). Owns its FILE*, so non-copyable/non-movable.

**Shim extensions — `test/pros_shim/pros/` (semantics transcribed from the vendored docs
BEFORE the adapters were written, each belief cited file:line and registered):**

- `distance.hpp` — ADVERSARIAL DEFAULT: raw 9999 WITH raw confidence 63 (exactly what a real
  sensor over an empty intake hands an adapter — a naive adapter reads "393 in, fully
  confident" and fails); a poisoned below-200 mm confidence channel (default 0) so a
  pass-the-raw-through close-range mistake reads "object touching, zero confidence" and
  fails.
- `optical.hpp` — PROS_ERR_F / PROS_ERR sentinel model per channel.
- `adi.hpp` — DigitalOut's ctor ACTUATES and keeps PROS's real `= LOW` default (the shim
  models PROS faithfully; the DISCIPLINE lives in our adapter), with a per-line write
  HISTORY so a boot glitch is visible; letters normalize to 1–8; expander pairs; DigitalIn's
  `get_new_press()` REALLY consumes per line (a mis-bound adapter starves the second
  consumer on behaviour, not a counter) and is counted.
- `misc.hpp` — `pros::usd::is_installed()` with ADVERSARIAL no-card default (placed outside
  `inline namespace v5`, matching the vendored nesting).
- `shim_control.hpp` — includes + `resetAll()` extended (distances/opticals/adi/usd).

**Tests (6 new files, +37 cases / +255 assertions; every case names the bug it would catch):**
`distance_conversion_test`, `optical_conversion_test` (hand-computed literals — 200 mm =
7.874015748031496 in, 9999 mm = 393.66141732283464 in, 128/255 = 0.5019607843137255 — never
the constant under test), `pros_distance_adapter_test` (9999 rule, close-range rule + its
201 mm boundary, PROS_ERR vs 9999 distinction, dead-from-boot-reads-FAR),
`pros_optical_adapter_test` (wiring + both sentinel families), `pros_digital_adapter_test`
(ctor-actuates-once with glitch-free history, set 1/0, command-vs-world divergence on a
refused write, expander + letter addressing, the TWO-CONSUMER level test, dead-port-reads-
pressed screen, ButtonEdge reuse, FakeDigitalIn), `pros_block_sink_adapter_test` (verbatim
bytes incl. 0x00/0x0A, no-card refusal + file-not-created, open-failure-never-throws, a REAL
refusing device — `/dev/full` — for the short-write false, the buffered-write honest limit
landing on flush(), the loud double-prefix precondition). Plus `pros_adapter_fence_test`
extended: `get_new_press` textually banned in `digital_in.hpp`, and a NEW adapter-wide
angle-bracket `<pros/>` ban — R1a's -iquote lesson pinned in-suite, load-bearing here
because `src/main.cpp` does not compile the R1b adapters, so `make` can never catch it for
them.

**Docs:** changelog entry (deliverable #7); three FAQ entries (the 393-inch question, the
pneumatic-fires-at-boot question, the SD path/no-card question); guide ch. 13 (the
construction-actuates warning + confirm-sensor pointers, in the mechanism-building
extension); guide ch. 14 (edited with the most care — see Findings 1); the register
HA-113…122 + status-line narrative and counts; roadmap (WS2 `[~]` extended with cited R1b
evidence, you-are-here, the R1b chunk entry, register row F14). Internal: the live
`R1b-PROGRESS.md` (first action of the chunk), this record, runbook steps 16–20.

## Numbers

| Measure | Before | After |
|---|---|---|
| Suite | 1083 cases / 1,523,069 asserts / 3 skipped | **1120 / 1,523,324 / 3** (green) |
| ARM gate | 139 headers | **148 headers**, UNAMENDED, clean |
| PROS-free guard | pass | pass, and re-proven LIVE to bite (M13) |
| Layering guard | pass | pass (unchanged) |
| `make` | package | package (exit 0, hot+cold present; R1b adapters not in it — see Not finished) |
| Robot include semantics | — | scratch ARM probe of all five adapters under common.mk's exact `-iquote` form: PASS |
| Doc gates | all green | all green (coverage/fresh/examples/removability/staleness; briefing regenerated) |

## The mutation table — every one EXECUTED and OBSERVED

Runner: exact-match exactly-once edits, **build-gated** (a failed build can never read
green), full suite per mutation, restore from scratchpad copies (never `git checkout`),
**mtime bumped on restore** (the R1a-review stale-binary lesson), SIGPIPE ignored. M13 was
executed live during the guard proof.

| # | Mutation (brief §6) | Result (observed) |
|---|---|---|
| 1 | drop the mm→inch scale | **RED** (4 cases / 7 asserts) |
| 2 | **pass 9999 through as a real reading with nonzero confidence** (T4) | **RED** (1 case / 1 assert — the T4 test read confidence 1.0 on an empty intake) |
| 3 | drop the confidence ÷63 | **RED** (3 cases / 4 asserts) |
| 4 | drop the proximity ÷255 | **RED** (3 cases / 8 asserts) |
| 5 | conversion computed, raw returned (the C5 D-5 / E1 wiring hole) | **RED** (4 cases / 6 asserts) |
| 6 | remove sentinel screening on one reader (ProsDigitalIn) | **RED** (1 case / 3 asserts — the dead-port-reads-PRESSED trap fired) |
| 7 | invert `IDigitalOut::set()` | **RED** (2 cases / 4 asserts) |
| 8 | `commanded()` reports the world (updates only on accepted writes) | **RED** (1 case / 1 assert — the refused-write divergence test) |
| 9 | bind `IDigitalIn` to `get_new_press()` | **RED THREE WAYS** (3 cases / 6 asserts): the two-consumer STARVATION test on real consume behaviour, the ButtonEdge N-consumer test, and the textual pin. A single-consumer test would have passed — confirmed by reading the failure set |
| 10 | `write()` returns true when the device refused | **RED** (1 case / 1 assert — caught by the REAL refusing device, `/dev/full`; the no-card path alone could NOT have caught this one) |
| 11 | remove the `usd_is_installed()` check | **RED** (1 case / 4 asserts — sink opened anyway, wrote "successfully", created the file) |
| 12 | remove a new shim header's `#error` (adi.hpp) | **RED** (1 case / 2 asserts) |
| 13 | widen the guard to `--exclude-dir=pros` + plant `localization/pros/__smuggled.hpp` | **EXECUTED LIVE**: the widened form MISSED the plant (the measured hole, re-demonstrated); the shipped path-anchored form CAUGHT it file:line; the in-suite mirror went RED on the same plant. Plant removed, both re-verified clean |
| 14 | move a fence `pop` past the adapter code (distance.hpp) | **RED** (1 case / 1 assert — the fence-scope guard) |

**14 of 14 executed, 14 of 14 observed at the mandated outcome. Zero unexplained greens.**

## HA register entries added (HA-113…122), each with its runbook step

113 distance mm (reasoned · step 18) · 114 the in-band 9999 — exactness of the value is the
unverified half (reasoned · step 18) · 115 confidence 0–63 above 200 mm; below-200 value a
STATED UNKNOWN the bench log settles either way (reasoned · step 18) · 116 optical
hue/sat/bri ranges (reasoned · step 19) · 117 proximity 0–255 **and the larger-is-closer
POLARITY, which the vendored source does NOT state** (**invented**, flagged like HA-99 ·
step 19's covered/uncovered pair, mandated BEFORE any threshold uses proximity) ·
118 optical sentinels PROS_ERR_F / PROS_ERR (reasoned · step 19 + step 14) · 119 DigitalOut
DRIVES THE LINE AT CONSTRUCTION; at-construction (vs first-tick) timing is the unverified
half (reasoned · step 20, air disconnected first) · 120 ADI addressing 1–8 ≡ letters +
expander pairs; **whether OUR robot has an expander is an open QUESTION, not a belief** —
R1a's report came from an out-of-range registry index (reasoned · step 17) · 121 DigitalIn
level + PROS_ERR refusal + consuming new_press (reasoned · step 20 settles the refusal
value) · 122 usd probe, **fopen's /usd/ prefix (NOT in the vendored source, flagged)**, and
fflush-is-the-strongest-persist (reasoned, flagged · step 16 incl. the yank-after-flush
byte count).

## Decisions where a viable alternative existed

1. **9999 → far-and-finite distance + confidence 0**, over hold-last-good on 9999: a held
   stale reading reports a PRESENT object after the field of view empties — the dangerous
   direction for capture-confirm. Rejected: raising a fault (an empty intake is a normal
   state; cries wolf every tick — the brief's own rejection, confirmed).
2. **Close-range confidence = 1.0 at ≤ 200 mm**, over passing the raw channel through: PROS
   documents the channel as nonexistent there, and the shim's poisoned-channel default
   demonstrates the pass-through reading "object touching, zero confidence" — a
   capture-confirm refusing the grab it most needs. The 201 mm boundary is test-pinned.
3. **Initial hold for a boot-dead distance sensor = the far no-object value**, over 0.0:
   zero inches is "object pressed against the lens" — a confirmed capture from a sensor
   that was never plugged in. Test-pinned ("dead from boot reads FAR").
4. **`ProsBlockSink` takes a bare name + injectable mount root (ONE join code path)**, over
   a robot-only path ctor plus a test-only FILE* ctor: the second shape leaves the prefix
   logic as untested glue — the exact E1 sink-wiring hole this chunk's mutation 5 exists
   for. The trailing-'/' and no-leading-'/' preconditions are loud rather than normalizing,
   matching the house rule that a silently-corrected mistake is a mistake kept.
5. **`/dev/full` as a real refusing device in the write-failure tests**, over mocking the
   refusal in the shim: mutation 10 needs an OPEN file whose writes fail — the no-card path
   cannot catch a lying `return true` after a real fwrite. Probed in this environment
   before relying on it (large fwrite → short count; small fwrite buffers then fflush →
   EOF); the test REQUIREs `/dev/full` loudly rather than skipping (a skip is decoration).
6. **`faultedWrites()` on ProsDigitalOut** (write-side exposure), over silence or raising:
   the seam has no validity channel by CONTRACT, so the refusal is exposed for the loop
   layer, mirroring R1a's screen→hold→expose refinement of T7. `commanded()` still reports
   intent — the divergence is what makes a refused write visible at all (mutation 8 pins
   it).
7. **The shim keeps PROS's dangerous `= LOW` ctor default; our adapter refuses it** — the
   shim models PROS faithfully (a shim with our discipline baked in cannot catch a
   discipline-skipping adapter); the write HISTORY makes a boot glitch observable, not just
   the final state.
8. **The angle-bracket ban added to the fence test**, over relying on `make`: R1b's
   adapters are invisible to `make` (main.cpp does not include them — L3), so R1a's
   only-caught-by-make lesson had NO guard here at all; the in-suite scan is now the first
   thing that fires. The `-iquote` semantics themselves were proven by a scratch ARM probe
   using common.mk's exact form (measured, not shipped).
9. **Runbook steps appended as 16–20 after the write-the-record step 15**, over renumbering:
   every register entry cites its step by number; renumbering would break R1a's citations
   for zero bench value. Step 15 gained one line saying it applies after whichever steps
   ran.

## Findings in EARLIER chunks (Rule 4: fixed in the layer that owns them)

1. **Guide ch. 14 carried two claims falsified by R1a's own bench session** — "No shulib
   code has ever controlled a motor or read a real sensor" and "no adapter has ever touched
   a physical device… until [the bench session] runs". Both were true when written and
   false since 2026-08-13: the session commanded eight motors and read real sensors (the
   changelog's own hardware-validation entry records it). Found by READING (the trap-5
   shape, again); fixed in ch. 14 to the platform-layer-only truth, worded so the governing
   constraint ("never driven a robot" = no loop ever closed) is stated MORE precisely, not
   weakened.
2. **The vendored `adi.hpp` documents the expander smart-port range inconsistently** —
   "smart port: 1-21" in the ENXIO errno text vs "from 1-22" in the same functions' param
   docs (vendored adi.hpp:59 vs 92). Not fixable by us (vendored source); recorded inside
   HA-120's territory and noted in the progress log so the bench uses 1–21 (the range every
   other device API documents).
3. *(Process, not a defect)* `tools/briefing_status.py check` goes red the moment a chunk's
   tree diverges from the generated block (counts, the interrupted-chunk flag) — by design;
   regenerated at the end of this chunk after the completion record existed, per the tool's
   own model.

## Not finished, named honestly

- **`[~]` Nothing on hardware, and no claim otherwise:** none of the five adapters has ever
  read a physical sensor. Runbook steps 16–20 exist precisely because that is R3's, with
  the team lead at the table.
- **`src/main.cpp` deliberately untouched** (L3): it still carries the invented
  X-drive-with-tracking-wheels config the R1a bench session proved matches no robot the
  team owns, and it does NOT construct any R1b adapter. Consequence, stated: `make` builds
  the package without compiling the new adapters — the in-suite angle-bracket ban and the
  scratch `-iquote` probe cover the gap until the re-wiring chunk (owner: the chunk that
  fixes the port map, per the brief).
- **The lift-homing question remains open** — not R1b's to answer. `IDigitalIn` waits on
  it; register row F14 says so.
- **`IVision`/`ITagSource`** — R2's, untouched.
- **HA-115's below-200 mm raw behaviour and HA-121's refusal value are stated UNKNOWNS**,
  not beliefs — the runbook logs what the device actually does and the register gets the
  answer, whichever way it goes.
- **The buffered-write honest limit stands:** `ProsBlockSink::write()` can report true for
  a small write the card will refuse at flush — pinned as documented behaviour in a test
  and in the FAQ, not hidden. E1's format tolerates the truncation by design.

## Handoffs

**To R3:** HA-113…122 with runbook steps 16–20 (step 17 — the expander question — first if
time is short); the flagged pair that must be measured before anything thresholds on them
(HA-117 proximity polarity, HA-122 fopen prefix). **To F3:** the sensor-confirm seams are
real — `IDistance`/`IOptical`/`IDigitalOut`/`IDigitalIn` all have PROS backings; the
confirm-predicate pattern in guide ch. 13 now points at the live traps. **To the
main.cpp-rewiring chunk:** five adapters wait to be constructed; `ProsDigitalOut` demands
the initial state and `ProsBlockSink::isOpen()` wants its one-time boot report. **To the
reviewer:** the tree is dirty and buildable; `docs/internal/verify/verify-r1b.sh` was not
created, read, or touched.
