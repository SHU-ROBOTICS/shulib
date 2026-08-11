# Chunk C3 — COMPLETED (2026-08-06)

> Completion record for [`C3-hdrive-kinematics.md`](C3-hdrive-kinematics.md) — `hDrive()` + the
> pseudo-inverse: the chunk where **the 15″ H-bot runs the same motion code as the 24″ X-bot,
> unmodified**. Everything below is **as actually observed** — commands run, outputs captured,
> all 13 mutations executed and watched (the live sequence is in
> [`C3-PROGRESS.md`](C3-PROGRESS.md)). Changes are in the working tree, uncommitted, pending
> review, per the brief.
>
> **The headline:** the M2 Definition of Done — *"the same auton runs the H-bot"* — is now a
> MEASURED statement, not a design intention. The identical scheduler-driven routine (same
> generator, same seeds, literally the same waypoint list) lands at X-bot accuracy on the H-bot
> at every length (clean n=40: **H 0.238 in vs X 0.236 in**, flat in move count; hostile worst
> **H 4.03 in vs X 4.13 in**) while paying only **~1–4% extra time** — because the H-drive's
> beyond-authority fallback is **turn-WHILE-drive**, not turn-then-drive, and the A/B case
> proves it beats the pure crab it falls back from (1.92 s vs 2.71 s over the same 40 in).
> Zero edits to the motion layer's behaviour were needed — the one motion-layer diff is the
> A1-reserved `strafeFallbackActive` telemetry population. And the campaign's most valuable
> observation is a mutation that could NOT be caught by any closed-loop test (§4.1): geometry
> sign errors cancel end-to-end through the shared plant, so the independent rigid-body oracles
> in `h_drive_test.cpp` are the only thing standing between a wrong preset and a silently
> self-consistent wrong world.

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| `hDrive()` + `HDriveConfig` | `include/shulib/kinematics/h_drive.hpp` *(new)* | The H-drive as a MatrixKinematics preset (§13 #15 hybrid backend, the `xDrive()` idiom): rows [1,0,−w/2] / [1,0,+w/2] / [0,1,a] derived from rigid-body kinematics; wheel order 0=left, 1=right, 2=strafe; authority = strafeSpeedRatio × strafeTractionDerate (HA-54); every config input precondition-validated |
| The pseudo-inverse | `include/shulib/kinematics/matrix_kinematics.hpp` *(modified)* | `forward()` generalized to `(AᵀA)⁻¹Aᵀ` (the M1 deferral, discharged): (AᵀA)⁻¹ computed once at construction via the closed-form symmetric adjugate; DUAL PATH — tables the pre-C3 precondition accepted run the historical projection VERBATIM (bit-identity by construction), only previously-REJECTED tables take the general path |
| The conditioning guard | same | Relative Gram determinant `relDet = det(AᵀA)/(Σh²·Σv²·Σturn²)` ∈ [0,1], REJECT ≤ 1e-6 at construction — closes the true-rank hole the orthogonality relaxation opens (§4.2); scale-free, NaN-proof |
| F5 doc clarification | `include/shulib/kinematics/kinematics.hpp` *(modified, DOC ONLY)* | `strafeAuthority()`'s semantics restated as C1's D11 reading, now CONFIRMED (§2.1): the sustainable \|body vy\| as a fraction of the linear speed budget; the historical "\|vy\|/\|vx\|" phrasing retired as ill-defined. No signature, no behaviour — see the freeze note (§11) |
| `strafeFallbackActive` population | `include/shulib/motion/move_to_pose.hpp` *(modified, telemetry only)* | The A1-reserved field gets its one producer, in the ONE shared pipeline (serves StrafeTo/HoldPose too): flagged iff the authority clamp bound beyond a 1%-of-maxLin legibility floor; behaviour of the pipeline UNCHANGED (the flag observes the clamp, it does not alter it) |
| H-bot stand-in rig | `test/motion_test_rig.hpp` *(modified, additive)* | `hBotKinematics()` — 11″ track, strafe wheel 4″ aft (off-centre ON PURPOSE: the non-orthogonal path is what the suites must exercise), ratio 1.0, derate 0.35. PROVISIONAL (A4: HA-55) |
| Register entries | `docs/hardware-assumptions.md` *(modified)* | HA-54 (the traction derate — the underivable part of authority) + HA-55 (the stand-in geometry); register 53 → 55, reconciled both directions (grep-verified) |

**No CI edits needed:** `h_drive.hpp` lives in `include/shulib/kinematics/`, inside both guard
scopes; the ARM gate's generated list picked it up automatically (**87 headers**).

**New tests:** `h_drive_test.cpp` (9), `motion_hdrive_test.cpp` (11),
`motion_hdrive_routine_test.cpp` (3), `matrix_kinematics_test.cpp` (+6, and one pre-existing
case re-commented — see §5 D10). **29 new cases / 53,630 new assertions**
(527/859,931 → **556/913,561**; skips unchanged at 3). Grading discipline unchanged: truth from
`h.truePose()`, the estimate reserved for what the code read; the H-drive's plant truth flows
through the NEW general-path `forward()`, which is why its correctness is oracle-proven, not
assumed (§4.1).

---

## 2. The two questions the brief ordered this chunk to answer

### 2.1 C1's D11 strafe-authority interpretation → **CONFIRMED** (and the F5 phrasing corrected at source)

C1 interpreted the clamp as **|body vy| ≤ strafeAuthority() · maxLinearSpeed** and flagged it
"awaits C3 confirmation". C3 derived the H-drive's physics and the answer is unambiguous: the
limit is the strafe wheel's own sustainable surface speed — an **absolute lateral cap,
independent of vx**. The literal "|vy|/|vx| ratio" form in F5's old comment is not merely
awkward at vx = 0; it is **wrong in both directions**: it would forbid the pure strafe an
H-drive legitimately performs, and it would *admit more strafe at high vx* than the wheel can
deliver. D11 needed no renegotiation — the clamp stays exactly one line in exactly one place,
now exercised by a real authority-0.35 drive with the truth-side physics pinned (the crab's
true |vy| peaks at **21.0 in/s, exactly authority·maxLin**, and never meaningfully above —
§8).

Per build-order rule 4 the correction landed at the source: `kinematics.hpp`'s contract comment
now states the confirmed semantic and retires the ratio phrasing, explicitly marked as a
doc-clarification within F5 (no signature, no behaviour, both consumers already implement it —
§11 records the freeze-discipline reasoning).

### 2.2 The "automatic turn-then-drive fallback" → resolved as **turn-WHILE-drive, engine-level sequencing REJECTED** (D4)

The brief's landmine pair — "never sequence rotation before translation" (C1) vs "turn-then-
drive is the correct fallback for a drive that cannot strafe past its authority" — dissolves
once the physics is written down. What the H-drive's fallback actually is, in shipped code:

* The C1 clamp bounds |body vy| to what the hardware can deliver; **vx and ω stay at full
  authority**. Translation continues at the achievable lateral rate — the motion NEVER stalls,
  never refuses, never sequences.
* When the commanded target heading cooperates (the auton says "go there AND face that way"),
  the closed loop **demonstrably migrates translation from the weak strafe axis to the strong
  drive axis as heading converges** — observed: rotation and translation provably overlap
  mid-flight (ω > 0.5 rad/s while speed > 10 in/s), the fallback flag engages by record 5 of
  the leg and releases before the last quarter, and the leg completes in **1.92 s vs the pure
  crab's 2.71 s** over the same 40 in displacement. Turn-WHILE-drive: the fallback is not
  merely bounded, it is *better than the thing it falls back from* — which is what makes it
  the CORRECT behaviour rather than a degraded apology.
* A literal sequenced decomposition (rotate to face, drive, rotate back) was rejected at the
  engine level for three independent reasons, each fatal alone: (1) it would violate
  StrafeTo's hold-heading contract — the one verb whose entire meaning is "translate without
  rotating"; (2) it invents intermediate headings the auton author never commanded, mid-field,
  which is a safety and predictability failure (a robot pirouetting through traffic on its own
  initiative); (3) it is DOMINATED by the simultaneous form whenever it would help (measured
  above). A routine author who wants the sequenced form writes `TurnTo` + `MoveToPose`
  explicitly — on shulib it is a choice, never the engine's only mode, which keeps the LemLib
  comparison honest.
* **Visibility is contractual**: `strafeFallbackActive` is populated at the producer,
  TermSink renders " SFB" (the A1 formatter, live since this chunk), and "a silent fallback is
  a failing test" is literal — mutation M6 (flag suppressed) turns SIX independent cases red
  (§7).

The master plan's §13 #5 phrase "turn-then-drive fallback" is hereby given its precise
reading: it names the H-drive's authority-limited *mode* (translation carried increasingly by
the drive axis, enabled by free rotation), not a sequencing strategy. The locked row's
substance — motion/Chassis layer clamps and triggers; kinematics never clamps; telemetry-
visible — is implemented exactly as written. Flagged for the coordinator: the master-plan
wording could be touched up to "turn-while-drive" at the next editorial pass; C3 did not edit
a locked decision row on its own authority.

---

## 3. The X-vs-H routine numbers (the headline requirement: reported NEXT TO each other)

Same generator, same seeds, same cadence (C1's: a pure TurnTo after every 3rd waypoint, a
StrafeTo every 7th), run **through the C2 scheduler** (`async` + `waitUntilSettled` on a
`PlantPacer`) on both drivetrains — the same auton, literally.

**Clean plant (seed 77) — the count-compounding pin, both drives:**

| n | X finalErr | X worst | X time | H finalErr | H worst | H time |
|---|---|---|---|---|---|---|
| 5 | 0.228 in | 0.229 in | 9.56 s | 0.225 in | 0.231 in | 10.17 s |
| 10 | 0.00003 in | 0.229 in | 18.38 s | 0.00003 in | 0.231 in | 19.12 s |
| 20 | 0.0040 in | 0.238 in | 35.02 s | 0.0040 in | 0.238 in | 36.53 s |
| 40 | 0.236 in | 0.238 in | 74.80 s | 0.238 in | 0.238 in | 78.19 s |

Error is **FLAT in move count on BOTH drives** and the H-bot's accuracy is X's to within
thousandths of an inch — the authority clamp participates in every lateral leg and adds **zero
per-move error**. The H-bot pays **time, never accuracy**: +6.4% at n=5 shrinking to +4.5% at
n=40. (Unplanned cross-check, worth recording: the X column reproduces C1/C2's recorded
baseline **to the digit** — 0.228175 in at n=5 — through a freshly-written copy of the
generator, confirming the waypoint pipeline is exactly deterministic.)

**Full composed A3 hostility (seeds 11/22 — C1's exact scenario), worst per cell:**

| n | X final (s11/s22) | H final (s11/s22) | H worst head | X time | H time |
|---|---|---|---|---|---|
| 5 | 1.00 / 0.63 in | 0.91 / 0.65 in | ~0.25° | 11.4–12.8 s | 10.9–13.3 s |
| 10 | 2.61 / 0.78 in | 2.36 / 0.90 in | ~0.58° | ~24 s | ~24–25 s |
| 20 | 4.13 / 0.64 in | 3.92 / 0.50 in | ~1.03° | 44–46 s | 45–46 s |
| 40 | 3.15 / 2.35 in | 3.16 / 2.50 in | ~1.28° | 90–95 s | 93–95 s |

Worst anywhere: **X arrival 4.13 in — H arrival 4.03 in** (Σ sweep time: X 346.9 s, H 351.4 s
— only **+1.3%**). The difference between the drives is *noise around the drift process*, not
a drivetrain penalty: on several cells the H-bot lands *closer* than the X-bot, because at M2
the error budget is owned by IMU drift (time), not by kinematics. **That is the information
in the difference**: the second drivetrain confirms the error attribution C1 made on the
first.

**The H-bot's three-way regression** (pooled over every waypoint of every hostile routine):

| regressor | H slope | X slope (C1, for comparison) |
|---|---|---|
| error vs move index | 0.066 in/move (raw — count-as-proxy-for-time; clean sweep proves count itself contributes ~nothing) | 0.062 in/move |
| **error vs elapsed time** | **0.029 in/s — the real carrier** | 0.028 in/s |
| error vs distance | 0.0032 in/in | 0.0031 in/in |

Same structure, same magnitudes: the H-bot's M2 drift story IS the X-bot's. The asserted
hostile bound (6.0 in) is derived as C1 derived X's: measured Σ-time is ~1.013× X's, so the
drift-physics ceiling stays ~12 in at the HA-20 worst bias; 6.0 = observed 4.03 + ~50% margin,
2× inside the ceiling. **The count-vs-time discriminator on H** (8×30 in vs 16×15 in, 3
seeds): family B ends at 1.32 in mean vs A's 0.80 in — B pays its extra ~10 s of settle-tail
drift, no 2× compounding (the discriminator matters MORE on H, whose lateral settle tails are
longer — and it still reads clean).

Why only ~1.3% extra time and not the naive 1/0.35: **the turn-while-drive migration keeps
MoveToPose legs near X speed** — heading legs rotate the displacement onto the drive axis —
so only the explicit 1-in-7 StrafeTo legs pay the 21 in/s crab. (Found, honestly recorded: my
own bound-derivation comment initially guessed 1.2–1.4×; the measurement said otherwise and
the comment was corrected to the observed number — §4.5.)

---

## 4. Findings (each handled where it lives)

### 4.1 FOUND + OBSERVED (the campaign's most valuable result): geometry sign errors are INVISIBLE to every closed-loop test — the independent oracles are the only defence
Mutation M4 flipped the strafe row's ω-coupling sign (`[0,1,−a]`). Every closed-loop motion,
routine, hostile, and scheduler test stayed **GREEN**: the plant's truth flows through the same
mutated `forward()` that inverts the same mutated `toWheels()`, so the simulated world is
*self-consistently wrong* and the controller converges perfectly inside it. Only the three
from-scratch rigid-body oracle cases in `h_drive_test.cpp` (wheel speeds re-derived in-test
from `v_point = v_body + ω×r`, never from the preset's rows) went red. `drive_plant.hpp`'s
header predicted this hazard class when it chose to share F5 kinematics; C3 has now *observed*
it and built the defence the prediction called for. On hardware, this bug class surfaces only
at R3's bench signature checks — HA-55's settle procedure is that bench check, now with a
demonstrated reason to exist.

### 4.2 FOUND (design-time, closed before any test ran): relaxing orthogonality OPENS a rank hole the old code never had
The pre-C3 per-column rank check ("no all-zero column") was sufficient only because the
orthogonality precondition guaranteed independence. Three individually-nonzero but
linearly-DEPENDENT columns (two parallel wheel directions) pass the per-column check while
det(AᵀA) = 0 — and a NEAR-degenerate table passes any exact test while amplifying wheel noise
unboundedly. The conditioning guard (relDet, §5 D8) exists precisely to close the hole the
relaxation would otherwise open; mutation M3 proves it load-bearing (2 cases red including the
2-wheel table the OLD code rejected via orthogonality — the new code must keep rejecting it
for the new reason).

### 4.3 FOUND by mutation M10, doc corrected: the per-column rank check is now behaviour-preserving redundancy
With the conditioning guard present, deleting the rank check changes NOTHING observable: a
zero column drives det to 0 (or relDet to 0/0 = NaN, which the `>` comparison also rejects),
so every rejected table is still rejected — only the diagnostic message loses its specificity
("tank belongs in TankKinematics"). The mutation stayed GREEN; per the campaign discipline it
was analyzed rather than waved through: it is NOT a detection hole (the tested contract —
rejection — is fully preserved by the second guard; the suite deliberately does not pin
message text, an A1-era convention), and the header comment was corrected to say exactly what
is now true: the check is defence-in-depth kept for its actionable message, citing the
mutation. A doc that overclaims "still required" would have been the real bug.

### 4.4 FOUND (fp forensics from mutation M2): the bit-identity pin is sensitive ONLY because it includes the √2-based tables
Forcing every table down the general path turned exactly ONE case red (the checksum test — the
C2-M12 "sole detector" pattern repeating), but with a twist worth recording: the synthetic
engine table and tank survived even on the general path, because their column sums are exact
powers of two — `(Σv²Σt²)/(ΣΣΣ)` re-derives their reciprocals bit-exactly. Only the X-drive's
√2-based sums (Σh² = 2.0000000000000004 — fl(√2)² ≠ 2) drift by ulps. A regression pin built
on "nice" synthetic numbers alone would have been BLIND to path-routing bugs; the production
X-drive geometry is what gives the checksum its teeth.

### 4.5 FOUND + corrected in my own work: the H time-penalty intuition was wrong by 20×
The bound-derivation comment in the routine suite initially reasoned "H covers the same path
in ~1.2–1.4× the time". Measured: **1.013×** (§3). The intuition failed because it priced
every leg at the crab rate; the turn-while-drive migration means almost no leg pays it. The
comment now states the measured reality — recorded here because it is exactly the class of
plausible-sounding guess this project's evidence discipline exists to catch, and this time the
guess was mine.

### 4.6 FOUND (test-authoring, recorded honestly): a "strafe leg" must be checked in the BODY frame
The first version of the scheduler-chain case aimed its StrafeTo along field −Y with the robot
at heading −90° — which is body-FORWARD, zero strafe demand, and the SFB assertion failed on a
correct library. My test-geometry error, not a code defect; fixed by aiming the leg along
field +X (body-left at that heading), with the lesson pinned in-comment: fallback tests must
verify the body-frame displacement, not trust the verb name.

---

## 5. Decision log (every choice with a viable alternative)

### D1 — The H-drive is a `MatrixKinematics` PRESET (`hDrive()`), not a dedicated class
Master plan §13 #15 LOCKED the hybrid backend ("the linear holonomic drives are one impl,
MatrixKinematics") and `xDrive()` set the preset idiom; the brief's "HDriveKinematics" name is
realized the same way "XDriveKinematics" was. Rejected: a dedicated class (would duplicate the
engine or wrap it pointlessly, and the whole point of C3's pseudo-inverse is that the H-drive
becomes *data*).

### D2 — Authority = derivable ceiling × registered derate; the honest split
`strafeSpeedRatio` (strafe-wheel top surface speed / drive top surface speed — gearing × wheel
radius, genuinely derivable, 1.0 for the same-hardware stand-in) × `strafeTractionDerate` (the
part NO geometry can supply: one lightly-loaded omni pushing the whole robot across foam —
HA-54, default 0.35 = the master plan's locked provisional). Stated plainly per the brief's
"derive it" demand: on this bot the derivable part contributes 1.0 and the ENTIRE 0.35 is a
registered guess — which is the honest answer, not a failure to derive. Rejected: shipping one
opaque number (hides which half R5 must measure); deriving the derate from motor torque/normal
force models (the A2 honesty boundary forbids modeling what cannot be measured — it would be a
guess wearing a derivation's clothes).

### D3 — D11 CONFIRMED; the F5 comment corrected rather than left misleading
§2.1. The alternative — leaving "max sustainable |vy|/|vx|" in a frozen header while both
consumers implement something else — would make the frozen DOC the only wrong artifact in the
tree, and F6's facade documentation would inherit the confusion. A doc-only clarification of a
frozen contract, marked as such in the header itself, is the minimal correct move (freeze note
§11). Rejected: implementing the literal ratio clamp (physically wrong both directions, §2.1).

### D4 — The fallback is turn-WHILE-drive; sequenced turn-then-drive rejected at engine level
§2.2, with the three independent rejection reasons and the measured dominance. Also rejected:
making the fallback a property of a new H-specific motion primitive (the per-drivetrain
special case the landmine names — the whole chunk exists to prove none is needed).

### D5 — `strafeFallbackActive` populated at the PRODUCER, in the one shared pipeline
The record's producer has first-hand knowledge of whether the clamp bound; one line in
MoveToPose's tick serves StrafeTo and HoldPose too (they share the pipeline). This is the
A1-reserved population point (`debug_record.hpp` line 15 scheduled "strafe fallback at C3";
the field comment says "— C3"), so it is C3's assigned work, not a motion-contract change —
the pipeline's BEHAVIOUR is untouched (the flag observes the clamp; mutation M8 vs M6/M7
separate the two cleanly). Rejected: a sink-side decorator inferring the clamp from
|vy| == limit equality-riding (fragile inference, needs kinematics + config plumbed into a
sink, and cannot distinguish "clamped" from "legitimately exactly at the limit"); a public
`strafeFallbackActive()` accessor on IMotion (new API surface on a contract F6 is about to
freeze, with no in-tree consumer — records suffice; flagged as an F6 question, §11).

### D6 — The flag's legibility floor: 1% of maxLinearSpeed, deliberately NOT registered
Flag iff `|vy demand| − vyLimit > 0.01·maxLinearSpeed` (0.6 in/s at HA-50 defaults). Without a
floor the flag lights on sub-perceptible PID chatter — on tank (vyLimit 0) near-settle noise
would flag entire healthy runs, and on the X-drive ulp-level rotation dust does the same
(OBSERVED: mutation M9 turned the X never-SFB pin red with the floor removed) — and a
permanently-on flag is as undebuggable as a silent one. Host-decidable telemetry-legibility
constant ⇒ not an A4 entry (register rule 1: hardware claims only; the C2 kMaxStalledPaces
precedent) — the brief's "HA-nn if chosen" yields to the register's own rule, reasoning
recorded here and in-header. Guarded in BOTH directions (M6 silent / M7 lying / M9 floorless —
all red).

### D7 — The pseudo-inverse keeps TWO computation paths, selected once at construction by the pre-C3 acceptance predicate VERBATIM
Previously-accepted tables run the historical three-division projection literally; only
previously-rejected tables take the general `(AᵀA)⁻¹` multiply. This buys the strongest
possible no-regression statement — bit-identity for every table the old code accepted, by
construction AND by measurement (§6, §7 M2) — at the cost of one boolean and six doubles.
Rejected: general-path-only (ulp drift on every existing drive: every recorded C1/C2 accuracy
digit silently invalidated, the no-regression claim weakened to "approximately", and §4.4
shows the drift is real for the production geometry).

### D8 — Conditioning metric: relative Gram determinant, REJECT (precondition), floor 1e-6
relDet ∈ [0,1] by Hadamard, exactly det of the column-normalized Gram: scale-free (a 220 in
turn lever is not degeneracy — pinned), NaN-rejecting, and computable from quantities already
in hand. relDet ≥ 1e-6 bounds κ(normalized Gram) ≤ ~7e6 ⇒ ≥ ~9–10 significant digits in
forward(), ample for in/s odometry; every physical drive sits far above (X 1.0; H-bot 0.938;
the guard fires only on genuine design errors — two wheel directions the table cannot tell
apart). REJECT over FLAG: kinematics is a pure-math leaf with no fault channel, construction
red-on-failure is the tree's discipline, and a drivetrain that cannot be inverted should fail
loudly at build, not limp. Boundary verified empirically on both sides (ε-family relDet ≈
1.25e-3·ε²: accepted-and-accurate at 1.24e-5, rejected at 1.25e-7). Not an HA entry (D6's
reasoning: host-decidable numerics).

### D9 — The no-regression proof: XOR-of-bit-pattern checksums captured from the PRE-change build
Before touching the header, a capture program built against pristine commit-7fcb3d4 headers
swept 4096 wheel-speed combinations per table (exact binary-fraction inputs, verified
-O0 ≡ -O2) and recorded XOR-folded bit patterns + a hexfloat spot value; the test replays the
sweep and demands equality. Any 1-ulp change anywhere in 4096×3 outputs flips a checksum.
Rejected: tolerance-based comparison (cannot state "strict generalization", only
"approximately unchanged"); hardcoding thousands of individual goldens (the checksum is
exactly as sensitive and 300× smaller, with one spot value kept for failure localization).

### D10 — The old "non-orthogonal table rejected" test kept, re-commented, not deleted
Its table ({{1,1,0},{1,0,1}}) still throws — for the NEW reason (2 wheels can never be rank-3;
the conditioning guard catches it). The case was renamed and its comment rewritten to name the
bug it now catches (relaxation without rank protection); the brief's "every existing
kinematics test still passes untouched" holds literally at the assertion level, with the one
comment-level update recorded here rather than hidden.

### D11 — H-bot stand-in geometry: off-centre on purpose (HA-55)
11″ track / strafe wheel 4″ AFT of centre / ratio 1.0 / derate 0.35. The offset is nonzero
DELIBERATELY: an on-centre stand-in would route every motion suite down the orthogonal fast
path and the pseudo-inverse would ship untested at the system level. The on-centre case is
covered as the explicit degenerate test instead. Values invented (no 15″ robot exists even on
paper); registered with R3 as owner.

### D12 — The routine suites run THROUGH the C2 scheduler
One suite discharges two DoD items (routine accuracy + "the scheduler runs a full H routine"),
uses the engine a real auton will use, and inherits C2's engine-bookkeeping REQUIREs (started
== settled == legs, zero cancels/aborts, on BOTH drivetrains). C2's bit-identity result makes
this equivalent to C1's hand loop by proof, not assumption. X re-runs alongside H in the same
file so the side-by-side is measured in one place, not quoted across documents.

### D13 — The shared `maxWheelSpeed` budget assumption, stated not hidden
v1 desaturates all three wheels against one budget — correct at strafeSpeedRatio ≈ 1 (the
stand-in). A strafe wheel geared much slower than the drive would need a per-wheel budget;
that is an R5/Frontier refinement flagged in-header and at §11 for F6, rather than silently
absorbed into the freeze.

---

## 6. Test inventory (what each would catch — every case names its bug in-file)

**h_drive_test.cpp (9)** — the independent-oracle file (§4.1 is why it exists): from-scratch
rigid-body projection over 200 random geometries×twists (ANY row error, incl. the
closed-loop-invisible class); tank-subset equivalence (a second subtly-different differential);
pure-strafe/pure-rotation signatures (the off-centre wheel must roll during rotation or drag);
hand-derived closed-form inverse + round-trip ×200 (a wrong square-case pseudo-inverse — this
feeds the plant's truth); authority = ratio×derate with placement-independence sweep
(invented or placement-coupled authority); derate-0 dead-strafe limit (authority 0, kinematics
still rank-3); strafe-wheel-on-centre degenerate (orthogonal path, same authority, ω never
leaks into vy); config rejections (9 malformed configs).

**matrix_kinematics_test.cpp (16, +6 new)** — bit-identity checksums vs the pre-C3 build over
4096-point sweeps ×4 tables + hexfloat spot (ANY numerical change to previously-accepted
forward(), incl. fast-path routing — §7 M2's sole detector); non-orthogonal square table
constructs + round-trips exactly (relaxation not actually delivered / wrong inverse);
normal-equation certificate on 200 INCONSISTENT wheel vectors against a fully-lopsided
redundant table (any wrong (AᵀA)⁻¹ entry — round-trips alone cannot see these, §7 M1); 400
seeded random tables construct-or-reject + round-trip with a non-vacuity floor (>300 accepted);
conditioning boundary both sides + exact-parallel + NaN evasion (a guard at the wrong
magnitude, or silent garbage near the boundary); scale-disparity acceptance (a raw-determinant
guard would reject legitimate big-lever geometry); the pre-C3 cases retained verbatim (incl.
the re-commented rank-in-disguise rejection — D10).

**motion_hdrive_test.cpp (11)** — the M2-DoD file: 12-seed random-pose invariant sweep on the
H-bot (any hidden X assumption in the motion layer; per-tick voltage/battery/finiteness/
physicality/progress invariants; record-audited authority ≤ vyLimit + flag-implies-riding-
the-limit; ≥4 trials must genuinely enter fallback so the sweep cannot go vacuous); TurnTo
in place + never-SFB (rotation leaking translation or flag noise); DriveBrake stop-and-stay +
HoldPose recovering from a LATERAL shove (disturbance rejection through the weak axis);
**StrafeTo beyond authority** (truth-side crab physics: peak exactly 21.0 in/s, ≤ 1.05×limit,
heading held < 2° — the fallback cannot rotate what StrafeTo pins; >50 SFB records; quiet exit
record); **TermSink end-to-end " SFB"** (the whole producer→record→formatter chain); X-drive
never-SFB ×3 lateral-heavy targets (structural claim + flag honesty); tank StrafeTo TimedOut
AND visible (C1's honest timeout now telemetry-explained); tank sub-perceptible demand
never-SFB (the D6 floor's designed detector); **the turn-while-drive A/B** (overlap observed
mid-flight, flag engages-then-releases, aligned MoveToPose beats the crab — sequencing OR a
never-disengaging fallback OR a worse-than-crab fallback all red here); composed FullHostility
MoveToPose+StrafeTo ×3 seeds (H-specific divergence under hostile estimates; finiteness every
tick); the C2 scheduler chain with id-stamped SFB and consistent counters.

**motion_hdrive_routine_test.cpp (3)** — §3's tables: clean X-vs-H flat-in-count (per-move
compounding that only appears when the clamp participates — plus the H-time sanity bound that
caught mutation M5); hostile X-vs-H sweep with the three-way regression and derived 6.0 in
bound (unbounded/blowing error on the second drivetrain, count-growth hiding behind drift);
the H count-vs-time discriminator (compounding paid twice at doubled move count).

Honesty notes: (1) the plant does not model the strafe wheel's traction limit — authority < 1
is enforced by the MOTION-layer clamp (the contract under test), not by sim physics; the suite
proves the contract at 0.35, R5 measures the number (HA-54's entry says exactly this). (2) The
2%-miscalibration displacement diagnostic was not repeated on H (drivetrain-independent — it
exercises tracking-wheel calibration, identical hardware on both bots); C1's X-drive version
stands for both. (3) `motion_hdrive_test`'s HoldPose shove uses the harness's direct
`commandBodyTwist` as the disturbance (same tick the hold commands, last-write-wins) — a
harness idiom, not a physics model of contact.

---

## 7. Mutation checks (13 — each executed: break → build → run → OBSERVE → restore → re-green)

> Restores from cmp-verified pristine copies, never `git checkout` (the C3 work itself is
> uncommitted — the C1 process lesson, followed). After the campaign: all 4 touched headers
> cmp-identical to pristine, zero mutation markers in `include/`/`test/` (grep), suite
> re-green 556/913,561. **Twelve red; one deliberately-probed green, analyzed and answered
> (M10) — not a suite hole (behaviour-preserving redundancy), and it produced a doc
> correction.**

| # | Mutation | Observed result |
|---|---|---|
| M1 | General path: g01/g02 swapped (adjugate slip) | **RED** — 3 cases / 3 assertions: normal-equation certificate, random-table sweep, boundary accuracy. SPECIFICITY OBSERVED: h_drive tests stay green because the H Gram has hv=ht=0 ⇒ g01=g02=0 (swapping zeros) — the fully-populated-Gram tests are the only detectors of this slip class, which is why the normal-equation table has every off-diagonal nonzero |
| M2 | Fast-path predicate dead (`orthogonal_ = false`) | **RED** — 1 case / 9 assertions: ONLY the bit-identity checksums (the C2-M12 sole-detector pattern). §4.4's fp forensics: synthetic + tank checksums SURVIVED (power-of-two sums); only the √2-based X-drive tables drift — the production geometry is what arms the pin |
| M3 | Conditioning guard vacuous | **RED** — 2 cases / 3 assertions: rank-in-disguise + both rejection arms of the boundary case |
| M4 | hDrive strafe row ω sign flipped ([0,1,−a]) | **RED** — 3 cases / 3 assertions: rigid-body oracle, rotation signature, hand inverse. **Every closed-loop test GREEN** — the shared-kinematics cancellation, predicted by drive_plant.hpp, now observed (§4.1) |
| M5 | Authority ignores the derate (= 1.0) | **RED** — 10 cases / 15 assertions: every authority pin AND every SFB-visibility case (no clamp ⇒ no fallback ⇒ flagged>0 REQUIREs fail — the silent-fallback discipline catches a wrong NUMBER, not just a missing flag) + the X-vs-H time sanity bound (H "beat" X — impossible at real authority) |
| M6 | **The SILENT fallback** (flag forced false) | **RED** — 6 cases / 6 assertions, six independent visibility pins: beyond-authority audit, H sweep fallback floor, scheduler id-stamped SFB, TermSink terminal line, engage-then-clear trajectory, tank visible-timeout. The brief's sentence is now a 6-way tested property |
| M7 | The LYING flag (forced true) | **RED** — 5 cases / 5 assertions: X never-SFB, tank floor case, both flagged-off-limit audits, trajectory lastFlagged bound — flag honesty pinned in both directions |
| M8 | C1's authority clamp dropped (C1 mutation #5, re-run against the real drive) | **RED** — 6 cases / 154 assertions: C1's fractional-authority + tank vy-audit detectors still alive, PLUS the C3 real-drive detectors (truth crab at 60 in/s where 21 is deliverable; everExceeded audits; X-vs-H routine). D11's clamp now double-guarded: synthetic contract shape AND real physics |
| M9 | SFB legibility floor removed (fraction = 0) | **RED** — 2 cases / 2 assertions: the designed tank detector AND (unplanned, informative) the X never-SFB pin — ulp-level rotation dust lights a floorless flag even on a full-authority drive. The floor is load-bearing for the structural X claim too |
| M10 | Per-column rank check vacuous (the redundancy probe) | **GREEN — analyzed, answered (§4.3)**: the conditioning guard subsumes the rejection (zero column ⇒ det 0 / relDet NaN ⇒ reject); behaviour-preserving, message-only delta, suite convention does not pin messages. Response: header doc corrected to claim exactly what is true, citing this mutation |
| M11 | Clamp applied to vx instead of vy (axis mixup) | **RED** — 8 cases / 156 assertions across BOTH drivetrain families (tank forward speed strangled; vy unclamped on H) |
| M12 | forward() = pre-C3 projection for EVERY table ("was the pseudo-inverse necessary?") | **RED** — 9 cases / 344 assertions: all 5 new kinematics cases + hand oracle + derate-0 round-trip AND two PHYSICS cases — H TurnTo no longer rotates in place (strafe-wheel roll misattributed as phantom translation in the plant's truth) and beyond-authority crab physics break. Shipping the H-drive on the old math would have produced a visibly wrong world — the chunk is provably necessary |
| M13 | TermSink " SFB" rendering dropped | **RED** — 2 cases / 2 assertions: the C3 end-to-end legibility case AND A1's own flags-iff-set golden (which set this field at A1, waiting three chunks for its producer) |

---

## 8. The beyond-authority physics and the fallback A/B (what the contract is worth)

From ground truth (`trueBodyTwist()`), never from the motion's own records:

| Scenario (H-bot, authority 0.35 ⇒ vyLimit 21 in/s) | Observed |
|---|---|
| StrafeTo 30 in pure lateral (demand ≈ 60 in/s) | settles at 0.6 in-class accuracy; **true crab peak 21.0 in/s — exactly the limit**, never > 1.05×; heading held to 0.0° throughout (the fallback cannot rotate what StrafeTo pins); >50 SFB records; exit record quiet |
| MoveToPose 40 in lateral, target heading aligned (+90°) | **1.92 s** — rotation and translation overlap mid-flight (turn-WHILE-drive observed); SFB engages by record 5, releases before the last quarter as the drive axis takes over |
| StrafeTo of the same 40 in (the pure crab) | **2.71 s** — the fallback beats the crab by 29%; falling back is the CORRECT strategy, measured |
| X-drive, any leg incl. pure lateral | SFB never set (structural: authority 1.0 + the norm cap ⇒ the clamp cannot bind; pinned) |
| Tank StrafeTo 24 in lateral | TimedOut honestly (C1 D12) AND >50 SFB records — the drive now TELLS you why it failed |
| Tank along-axis with 0.1 in lateral hair | zero SFB (the legibility floor: 0.3 in/s of undeliverable chatter is not a fallback) |

---

## 9. What we now know for certain, and what we do not

*(Written for a reader who was not here. "Certain" = proven by a passing, mutation-guarded
test against plant ground truth, across the swept space described.)*

**Now known for certain — on the A2 plant, under A3's hostile world:**
- **Two robots, one motion layer — the M2 DoD holds.** The 15″ H-bot (3 wheels, off-centre
  strafe, authority 0.35) runs C1's five primitives and C2's scheduler with ZERO
  motion-layer behavioural changes, no drivetrain branches anywhere, and lands the SAME
  scheduler-driven auton at X-bot accuracy: clean routines flat in move count on both drives
  (0.225–0.238 in, tolerance-class, 5→40 moves), hostile worst H 4.03 in vs X 4.13 in, same
  drift-carried error structure (0.029 vs 0.028 in/s time slopes). The abstraction did not
  fail — and C4 can now build the facade on evidence instead of hope.
- **The H-bot pays time, never accuracy, for its limited authority — and only ~1–4% of it.**
  The turn-while-drive migration keeps MoveToPose legs near X speed; only explicit pure-crab
  legs pay the 21 in/s rate. The naive 1/0.35 intuition is wrong by ~20× (measured, §4.5).
- **The fallback is real, correct, and NEVER silent.** It engages exactly when a leg
  out-demands the strafe wheel, disengages when the drive axis takes over, beats the pure
  crab it replaces (1.92 vs 2.71 s), never rotates a held heading, and is visible at every
  level — record flag, scheduler-stamped stream, terminal " SFB". Suppressing it fails six
  independent tests; lying it on fails five more.
- **C1's D11 is the confirmed F5 semantic.** |body vy| ≤ authority × linear-speed-budget is
  what the H-drive's physics dictates (an absolute lateral cap); the |vy|/|vx| ratio phrasing
  is retired from F5's docs as wrong in both directions. The clamp itself needed zero changes
  and is now guarded from both the synthetic (C1) and real-drive (C3) sides.
- **The pseudo-inverse is a strict generalization, to the bit.** Every table the pre-C3 code
  accepted computes forward() through the identical expressions and produces bit-identical
  results (XOR-checksum-pinned against the pristine build, over 4096-point sweeps); the
  general path is exact for the square H table (hand-inverse-oracle to 3.6e-15) and a true
  least-squares inverse for redundant tables (normal-equation certificate on inconsistent
  inputs). F5's signatures and behaviour: untouched.
- **Ill-conditioned geometry cannot ship.** Parallel or near-parallel wheel directions are
  rejected at construction on both sides of a documented, empirically-verified relDet
  boundary; the guard is scale-free and NaN-proof, and it also covers the true-rank hole that
  relaxing orthogonality opened (which the old per-column check provably cannot see).
- **Preset geometry is oracle-guarded — and must be.** A sign error in the H rows survives
  EVERY closed-loop test (observed, mutation M4: the shared plant makes the wrong world
  self-consistent); the from-scratch rigid-body oracles are the only host-side defence, and
  they caught every geometry mutation thrown at them.

**NOT yet known — and who owns finding out:**
- **The real authority number.** The 0.35 is 100% registered guess (HA-54): the geometric
  ceiling contributes 1.0 on the stand-in hardware and the entire derate awaits R5's sysid.
  It could plausibly be anywhere from ~0.15 (heavy bot, slick omni) to ~0.8 (light bot, good
  normal force). Everything C3 proved — clamp, fallback, visibility, accuracy structure —
  holds for ANY value; only the TIME numbers shift.
- **The real geometry.** Track width, strafe wheel position/gearing (HA-55) are invented
  stand-ins; R3 measures the built chassis and re-runs these suites. The one structural
  sensitivity is benign in both directions (near-centre ⇒ orthogonal fast path; absurdly far
  ⇒ the conditioning guard rejects loudly).
- **Whether one strafe omni delivers ANY useful sustained crab on real foam** under a real
  robot's mass distribution — the qualitative bet behind building an H-drive at all. Sim
  cannot answer it; R5's first lateral sysid run will.
- **Combined strafe+rotation at the limit.** The scalar authority deliberately prices
  sustained PURE strafe; the ω·a coupling eats into the strafe wheel's budget during
  simultaneous rotation, handled today by desaturate()'s direction-preserving scale. Whether
  that budget-sharing matches real traction behaviour is R5/R6 territory (and a per-wheel
  budget is the named refinement if the built bot gears the strafe wheel differently — D13).
- **Everything C1/C2 already listed** — gains/tolerances, real braking, the true drift story,
  scheduler-on-hardware timing — unchanged by this chunk.

---

## 10. Deliberately left for later chunks (named handoffs)

- **→ C4 (`Chassis` facade)**: (1) accept `HDriveConfig`/`hDrive()` naturally in the builder
  (a drivetrain is config data — keep it that way). (2) Decide the two F6 exposure questions
  flagged in §11 (authority passthrough; fallback-state surface). (3) The facade's docs must
  carry the turn-while-drive semantics as API behaviour ("on an H-drive, lateral-dominant
  moveTo legs run authority-limited and flag SFB") — users WILL observe it.
- **→ C5 (results/summary)**: SFB is on the record stream per tick; the per-motion result
  line could cheaply carry "fallback: n ticks (p%)" — the raw material is stamped and
  discriminated already. C5's call.
- **→ R3**: measure HA-55 (track, strafe position/diameter/gearing); run the bench signature
  checks (pure strafe moves only wheel 2; rotation rolls it at a·ω) — §4.1 is the demonstrated
  reason those checks exist.
- **→ R5**: sysid HA-54 (sustained lateral speed on foam, loaded); re-run the C3 suites with
  the measured value; revisit the shared-wheel-budget assumption (D13) if the built strafe
  gearing differs.
- **→ Phase E**: nothing new — the H-bot's error is the same drift story, which is E's
  existing mandate; the second drivetrain CONFIRMED the attribution rather than adding to it.
- **→ Frontier**: per-wheel desaturation budgets (D13); strafe-authority-aware path planning
  (a G-phase/VexBuilder planner could prefer drive-axis-aligned approach headings on the
  H-bot — the 1.92-vs-2.71 s measurement is the value proposition, now a number).

---

## 11. Freeze Register note (documentation contract #6)

**No freeze at C3.** Two freeze-adjacent acts, recorded prominently:

- **F5 (`IKinematics`) received a DOC-ONLY clarification** (§2.1/D3): `strafeAuthority()`'s
  comment now states the D11-confirmed semantic (fraction of the linear speed budget) and
  retires the ill-defined "|vy|/|vx|" phrasing, with the change marked and dated in-header.
  No signature, no behaviour, no consumer changed — both existing consumers already
  implemented the confirmed reading, which is exactly why the doc had to follow them. The
  precondition RELAXATION in MatrixKinematics (orthogonality → conditioning) is the
  M1-deferral path the header scheduled since 2026-06-19 and is F5-safe by the original
  decision-check (accepts strictly more, changes nothing accepted before — bit-proven).
- **`DebugRecord.strafeFallbackActive` is now POPULATED** (A1 reserved it; C3 produces it;
  F9 will serialize it at H1 unchanged — the field's shape was never touched).

**F6 flags — C2 listed eight shapes the facade will inherit; C3 adds/amends these** (the
H-drive is the reason they exist, so this is the chunk that must name them):

1. **The facade does NOT need a new fallback-state surface.** The fallback is already
   observable per-tick via the record stream (id-stamped through the scheduler, " SFB" on the
   terminal). Recommendation to C4: do not add a `strafeFallbackActive()` accessor to the
   public verbs — a live-polled bool invites control-flow coupling to a telemetry concept;
   if F′-routines ever need to BRANCH on it, that is a D1-recipe/waitUntil-predicate design
   discussion, not a facade getter. Decide deliberately at C4, not by absorption.
2. **`strafeAuthority()` passthrough: expose it.** The facade holds the kinematics; a
   read-only `chassis.strafeAuthority()` passthrough costs nothing and routine authors
   budgeting lateral legs legitimately want it (the 21 in/s number is the difference between
   a 2 s and a 3 s leg). Cheap now, additive later — but F6 should decide, not inherit.
3. **The turn-while-drive semantics are facade-level API documentation** (handoff to C4
   above) — F6's docs must carry them the way they must carry C1's wait-for-live and C2's
   pre-empt semantics.
4. **`MotionConfig.maxWheelSpeed` is a SINGLE shared budget** (D13). If F6 freezes
   MotionConfig's shape into the facade's construction surface, a strafe-geared H-drive later
   needs an ADDITIVE per-wheel budget field — fine, but name it now so the freeze review
   checks the additive path exists rather than discovering a reshape.
5. **Drivetrain-as-config holds.** `hDrive()`/`xDrive()`/`TankKinematics` are all
   value-constructible from plain geometry structs — C4's builder should preserve exactly
   this (the VexBuilder `.vexbot` drivetrain fields map 1:1 onto these configs; §13 #14's
   cross-team ask is unchanged).
- C2's eight flags stand unamended otherwise.

---

## 12. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    556 |    556 passed | 0 failed | 3 skipped
[doctest] assertions: 913561 | 913561 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle (HA-01), unchanged.
Full suite wall time ≈ 1.2 s.)

```text
$ <the ci.yml PROS-free guard grep, scope unchanged — kinematics/ already covered>
GUARD 1 PASS: core is PROS-free (incl. kinematics/h_drive)
$ <the ci.yml layering guard grep, scope unchanged>
GUARD 2 PASS: layering holds, core is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # generated list, ALL v2 headers
ARM CROSS-COMPILE: CLEAN (87 headers)
```

Working tree left uncommitted for review, per the brief. Post-mutation integrity: all 4
mutated headers cmp-identical to pristine copies; zero mutation markers in `include/`/`test/`
(grep-verified). Register reconciliation re-run clean both directions (no `PROVISIONAL (A4`
label without an HA id; HA-54/55 anchored in-tree).

---

## 13. DoD checklist (brief §Definition of Done)

- [x] **`HDriveKinematics` implemented; strafe authority derived from geometry** — as the
  `hDrive()` MatrixKinematics preset (D1, per the locked hybrid backend); rows derived from
  rigid-body kinematics and independently oracle-pinned; authority = derivable speed ratio ×
  HA-54 derate, with the honest split stated (the derivable part is 1.0 on the stand-in; the
  0.35 derate is the registered guess — D2). Nothing invented is unlabeled.
- [x] **`MatrixKinematics::forward()` generalized to the pseudo-inverse, orthogonal cases
  proven unchanged** — dual-path (D7), previously-accepted tables BIT-IDENTICAL by XOR
  checksum against the pre-C3 build (D9; mutation M2 red), general path exact for the square
  H table and normal-equation-certified for redundant ones (mutations M1/M12 red).
- [x] **Turn-then-drive fallback works, is visible via `strafeFallbackActive`, and is never
  silent** — resolved as turn-WHILE-drive with sequencing rejected on three grounds (§2.2/D4);
  engages/clears/beats-the-crab measured (§8); visible producer→record→scheduler-stamp→
  terminal; silent = 6 red cases, lying = 5 red cases (mutations M6/M7/M13).
- [x] **C1's primitives and C2's scheduler run on the H-drive with no motion-layer changes** —
  all five primitives + a scheduler chain + full scheduler-driven routines, zero behavioural
  motion-layer edits (the one motion/ diff is the A1-reserved telemetry population, D5 — the
  pipeline's behaviour is byte-equivalent, and the pre-C3 suite passed unchanged against the
  new headers before any new test landed).
- [x] **H-drive routine accuracy measured, error flat in move count, three-way regression
  reported** — §3, X reported next to H in the same tables from the same runs: clean flat both
  drives, hostile worst 4.03 vs 4.13 in, H slopes 0.066/0.029/0.0032 vs X 0.062/0.028/0.0031,
  + the H count-vs-time discriminator.
- [x] **C1's D11 strafe-authority interpretation confirmed** — CONFIRMED (not corrected), with
  the physics argument (§2.1); the stale F5 phrasing corrected at its source per rule 4 (D3),
  doc-only, freeze-note recorded (§11).
- [x] **Ill-conditioned geometries rejected or flagged** — REJECTED at construction (D8):
  relDet guard, boundary-verified both sides, scale-free, NaN-proof; closes the §4.2 rank
  hole; mutation M3 red. The threshold is documented in-header with its derivation;
  deliberately not an HA entry (register rule 1 — the reasoning is recorded, not skipped).
- [x] **Any invented constant carries an `HA-nn` entry** — HA-54 (derate) + HA-55 (stand-in
  geometry), register 53 → 55, bidirectional reconciliation re-run clean. The two
  host-decidable numerics constants (conditioning floor, SFB legibility floor) are
  deliberately NOT registered per register rule 1, each with the reasoning in-header (D6/D8).
- [x] **Suite green under strict `-Werror`; both guards pass; ARM gate passes** —
  556/913,561; both guards (scopes unchanged); 87/87 headers (§12).

Beyond the brief, at the escalated bar: the bit-identity checksum regression (with the §4.4
fp forensics showing why it has teeth), the normal-equation least-squares certificate, the
shared-kinematics cancellation OBSERVED and answered with independent oracles (§4.1), the
turn-while-drive A/B time measurement, X re-run beside H from the same code so the side-by-side
is one experiment, and a 13-mutation campaign whose single green was probed on purpose,
analyzed, and converted into a documentation correction rather than waved past.
