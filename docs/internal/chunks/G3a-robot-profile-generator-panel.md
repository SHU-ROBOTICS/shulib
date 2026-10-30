# G3a — the robot file, its generator and the optional robot panel: robot two described as data, generated into marked regions byte for byte, one validator, and a VS Code form over it — amended 2026-09-29: a new robot with its build, mechanisms from templates, wiring and controls

> **THE TEAM LEAD'S ANSWERS (2026-10-30)** to the evening amendment's D11–D20 (its "## Amendment 2026-10-29
> (evening)" and the checker's C1–C16, each fixed in place) — **"Accept all (Recommended)" for D11–D15, D17, D19 and
> D20, and these three:**
> - **D16 — (c), A TRUE CURVE FOLLOWER** (not the recommendation): a pursuit- or boomerang-style controller on a tank,
>   a smooth path form, its end behaviour, the facade verb, the Routine step and F8's `"arc"` (additive). It RE-OPENS
>   C1's D12 ("never aim at the point") — by this answer, for the curve follower only. ~3½–5½ builder-days (rough, no
>   precedent here) plus robot tuning; it cannot be built and trusted on a robot by Nov 21, so it is its OWN library
>   chunk, **CURVE1**, with its own brief and go, AFTER the route maker lands. Until then the route maker offers
>   "through points" (stopping at each, with the tank warnings on bends over 15°), as D16 (a) describes.
> - **D18 — 30 s for a match's autonomous, 60 s for skills** (VEX U), as recommended.
> - **D19 — the publisher name is `shulib`** (the brief's example was `shu-robotics`); the team lead checks and
>   registers it on the VS Code Marketplace and Open VSX before the first store release (after Nov 21). Releases are
>   cut only on his `robot-panel-v*` tag, as recommended.
> - **So:** D11 (a) the feel inside Part 5; D12 (a) every tuning number typeable with the library's default shown, the
>   turning radius derived (half the measured track width); D13 (c) a route becomes generated `Routine` code in a
>   `[routes]` region of the autons file, plus an F8 export with one optional `shulib` object per waypoint; D14 (a)
>   per step stop / skip to a named step / carry on, the deadline always stopping; D15 (a) a drive that ends early
>   never fires its partway trigger; D17 (a) red/blue as two autons, the mirror axis read from the manual (Appendix A)
>   and put to him before Part R1's go; D20 (a) the five slots: the Nov 23 competition work first (MECH-A6 first),
>   then make-and-drive, the route maker, the string-lift safety, the walls page, the camera page — the mechanism
>   pages (Part 6) before drive tuning (Part T); robot one's hand-written backup starts now (its facts by ~Nov 5).
> - **Each part is still its own go.** The library runs STEP1-A and STEP1-B are their own chunk (**STEP1**), briefed
>   from this brief's R.13 before Part R1. The coordinator's draft stays in `.claude/coordinator/briefs/`; this copy
>   is the brief of record.

> **AMENDED 2026-10-29 (EVENING) — read "## Amendment 2026-10-29 (evening) — the team lead's panel scope" FIRST;
> where it and anything else in this file disagree, it wins.** After seeing the panel's concept page the team lead
> ruled: (1) the driving feel becomes panel data (REVERSES D4 (a)); (2) drive tuning per robot; (3) the camera and wall
> pages by Nov 21 (if tight, the route maker comes first); (4) a FULL route maker — every auton step, care on
> each, mechanism timing with partway triggers, red/blue mirroring, a timeline; (5) "GitHub now, store later". The
> amendment adds D11–D20 for him, Part T (drive tuning), a route lane (R1, R2) and the library runs STEP1-A/B, splits
> Part S into S1 (walls) and S2 (camera), and re-plans the parts and the five job slots for Nov 21. *(fix 2026-10-30:
> a checker's 16 findings on the amendment, each re-verified and fixed in place — "### The checker's findings on this
> amendment (C1–C16)"; the dates RE-DERIVED on five slots with every review round holding a slot: make and drive
> ~Nov 14–18; routes ~Nov 18–22; walls ~Nov 18–22; the camera page ~Nov 25–Dec 2 — it misses the 21st; robot one's
> backup must start by ~Nov 6, D20.)* **Status: D11–D20 and each part's go WAIT for him; Part 5 waits for D11, Part 3
> for D19.**

> **THE TEAM LEAD'S ANSWERS (2026-10-29)** to D6′–D10 as the re-check of 2026-10-29 put them — **every one as
> recommended**:
> - **D6′ (a) — DECIDED: two lanes, after MECH-A5 merges.** The TOOL lane Part 1 → Part 5 → Part 2 → Part 4 (Parts 1
>   and 5 are the foundation: robot two's whole file — drivetrain, mechanisms, controls — generated and
>   freshness-gated) and the PANEL lane Part 3 (after Part 1) → Part 6 → Part 3b → Part R; Part S after VIS1-B and
>   L6-B, Part AF after AF1, Part 7 after MECH's rest. Part 1 starts when MECH-A5 merges (his ordering ruling of the
>   same day). ~11–15½ builder-days to a robot made and driven in the panel with its mechanisms (two lanes;
>   verification on top); Parts 1–6 ~17¾–23¾; everything before G3 ~24¾–34¼ (+½–1½ in Part 5 if AF1 lands first).
> - **D6′'s slot question — DECIDED, as recommended:** when a usage warning drops the ceiling to two jobs, G3a's PANEL
>   lane yields first, then AF1, then G3a's TOOL lane; MECH-A6 and L6-B never yield before the ~Nov 16 code freeze. At
>   one job: the 2026-11-23 track's job until the freeze; after it, the tool lane or AF1 — his call then.
>   *(fix 2026-10-30, C6: the evening amendment's D20 (a) would SUPERSEDE this yield order — Part 6, a panel-lane part,
>   goes before Part T, a tool-lane part, because Part 6 is on the critical path to "make and drive"; D20 puts that to
>   him. Until he answers, this answered order stands.)*
> - **D7 (a) — DECIDED:** robot two's typed table is flipped to the camera end with `flip-front`, as Part 1's own
>   logged step right after the byte-for-byte differential; the step cites the record (2026-09-25), not a front.
> - **D8 (a) — DECIDED:** a new robot gets robot two's five programs — Bench Tests and Calibrate (its bring-up builds,
>   Part 2), then Drive, the Auton Tester and Competition (Part 4).
> - **D9 (a) — DECIDED:** the shared marked regions for a new robot's build plumbing are set up before the first
>   generated robot exists; the tool refuses a clashing id (a retired build tag such as `tank_library`, a C++ keyword,
>   a namespace `src/` names unqualified, a prefix of another robot's id or of the team files) and Calibrate on
>   anything but a tank (CAL1 R13).
> - **D10 (a) — DECIDED:** a new robot needs three facts — its name, its drivetrain kind and its cartridge; drive
>   ports are optional (typed ports give build-time collision checks, never a boot fallback) and the front comes from
>   Calibrate's push. Option (c) (the panel copies the record's ports into the file) NOT taken.
> - **Each part is still its own go; nothing of G3a builds before MECH-A5 merges.** *(amended 2026-10-29 evening: the PANEL SPRINT of the same afternoon built Part 1 BEFORE MECH-A5 merged, as D6′ (b), robot two's D7 flip held to Part 5's first step — build-order :1817-1826; each part is still its own go)* HA-181 stays G3a's reserve (next
>   free HA-216). The coordinator's draft stays in `.claude/coordinator/briefs/`; this copy is the brief of record.

> **RE-CHECKED 2026-10-29 FOR THE TEAM LEAD'S NEW ORDERING — read "## Re-check 2026-10-29 — amendments" below
> FIRST; where it and anything else in this file disagree, it wins.** His ruling of 2026-10-29: G3a's FOUNDATION (the
> robot file and its generator into C++) starts right after MECH-A5 lands, BESIDE VIS1, L6 and the auto-flipper (AF1)
> — not after them and DOCS3; the panel's sensor and auto-flipper pages plug in as those land; DOCS3 then documents
> everything, the panel included; up to three jobs at once when budget allows. The re-check re-plans the parts into a
> TOOL lane (Part 1 → Part 5 → Part 2 → Part 4; Parts 1 and 5 are the foundation: robot two's whole file — drivetrain,
> mechanisms, controls — generated and freshness-gated) and a PANEL lane (Part 3 after Part 1, Part 6, Part 3b — the
> NEW ROBOT page, CK6 —, then Part R —
> the routes ruled 2026-10-08, missing until now), plus Part S (the camera and wall-sensor pages, VIS1 D2 / L6 D5) and
> Part AF (the auto-flipper page), and re-sizes every part. It strikes from the body what the 2026-10-06 front ruling
> and CAL1's brief superseded (the front's words, `ChassisTable::front`, Calibrate's three hunks, Part 5's watch) and
> replaces the 09-29 mechanism vocabulary with MECH's as landed (§6.3, plus A4-D26/A4-D27 accepted 2026-10-29).
> **A checker's eleven findings on this re-check (2026-10-29, the same day) were each re-verified — all real — and
> fixed in place, marked *(re-check 2026-10-29, CKn)*; "### The checker's findings on this re-check (CK1–CK11)" lists
> them.** They add the rebase rule's point 6 (new vocabulary after Part 5 — AF1, MECH's rest, L4), the slot cost of
> two lanes and a question on which job yields (D6′), the `changeAt` limit at 160 (not 130), the per-robot namespace
> as a derived name, the re-zero `static_assert` fixture, the sensor mounts' front, and blanket marks over the body's
> retired 09-29 mechanism vocabulary.
> **Status (2026-10-29): WAITS for the team lead's answers to D6′–D10 as the re-check puts them ("The decisions, as
> they are put to the team lead NOW"), and his go for each part. Nothing is built. HA-181 stays G3a's reserve — the
> register's Held line carries it and the briefing's next free skips it (next free HA-216).**

> **THE FRONT, RULED 2026-10-06 (the team lead: "itll be hard to say which way is the front through the panel ... maybe
> we just get it from the calibrate?" — "Yes, with a flip warning").** The panel NEVER asks for the front, and the robot
> file states none: the end that LEADS Calibrate's push is the front, carried by the calibration record, and Calibrate
> warns when a new record's front is opposite to the previous record's. So: §4.1's new-robot page asks THREE facts (name,
> kind, cartridge); D10 (a)'s cost "the front cannot be checked" and its `.front` words in the chassis table and
> Calibrate's front line are SUPERSEDED (nothing typed to check); D7 stays recommended only to keep robot two's typed
> table — the no-record fallback *(re-check 2026-10-29, CK11 (d): NOT a fallback — the boot never drives from robot
> two's typed table, which is not a valid description (no track width, no ratio; RC13, D7); its readers are Bench
> Tests, the boot's CROSS-CHECK line and CAL1's typed check)* — agreeing with its record. **And CAL1 (the standard Calibrate, shipped with the library,
> the same day)** means Part 2's "a new robot's Calibrate build" calls the library's Calibrate with the robot's stated
> facts; re-check Part 2 against CAL1's brief before it starts. D6′, D8, D9 and D10 still wait for his answers. *(re-check 2026-10-29: done — CAL1 §1.7 strikes Part 2's Calibrate hunks and `ChassisTable::front` (RC3); D6′–D10 are re-put in the re-check, D7 included.)*

> **THE TEAM LEAD'S ANSWERS (2026-09-29, ~03:55): D1–D6 all as recommended ("Accept all").** They carry into the
> amended brief, which adds the panel requirements of `docs/internal/MECH-PANEL-requirements.md` §4 (new robot,
> mechanisms with templates, wiring — robot file only, Q1 (a) —, controls, calibration view) and a groundwork part
> built alongside the MECH core (speedup D, approved 2026-09-28 ~17:35). **The front:** "the front will be different
> depending on the robot" — per-robot and human-stated (already the design); robot two's own front is to confirm
> with him (its record: the camera end; its typed table: the 15/16 end).
> **→ AMENDED 2026-09-29 to those requirements: read "## Amended 2026-09-29 — the panel requirements" below FIRST.**
> It adds four decisions for the team lead (D7–D10), replaces D6's two parts with seven (a groundwork built
> beside the MECH core, a mechanism phase after it, the rest's extension after the rest of MECH), and marks
> every body passage it supersedes.
> **→ RE-CHECKED 2026-09-29 (a checker's 13 findings, all real): "## Re-check 2026-09-29 — amendments" lists
> each disposition.** The starter driver file moves from Part 2 to Part 4, after the core, so the groundwork
> depends on nothing of the mechanism vocabulary. Cookbook 06's driver-file listings name the file they quote.
> The front's words reach Calibrate. The `[robot guards]` region reads the hand-written robots from the
> Makefile. D6′ (the seven parts) is put to the team lead. Parts 1–6 are re-sized to ~16¾–22¾ days. *(re-check 2026-10-29: the front's words no longer reach Calibrate — struck, RC2/RC3; D6′ and the sizes are re-planned, RC1.)*

> **Execution brief (DRAFT for the team lead's go).** Written 2026-09-27 (evening) from a read-only pass
> over `shulib-v2` at `a62067b` (SRC1's layout; T4 Part 1a landed; TANK1 Part A in flight in its own
> worktree). A first drafting run was cut off at ~12:52 before it wrote anything; its probes
> (`gen/`…`gen5/`, `marked/`, `tree/`, `proto/`, `robot_two.robot.json`, `mut_port8.json`) were reviewed
> and **nothing from them is relied on**: every result below was re-run in `v2/`. Two of its choices were
> wrong and are changed here — its robot file said the front was the camera end while listing the LEFT
> side as 11–15 (the camera end's left is 16–20: the file contradicted itself), and its markers could not
> tell a hand edit from a stale region.
> **Status:** ~~WAITS for the team lead's go and his answers to D1–D6. Nothing is built. The team lead pushes
> the branch before Part A's build starts.~~ *[superseded 2026-09-29: D1–D6 accepted; the amendment's D7–D10
> and a go per part now wait — see the amended section's status line.]* A checker's 15 findings were re-verified and each is
> answered below ("Re-check — amendments"). Every section the findings touched is amended in place;
> the re-check's probes are in `recheck/`.
>
> **The team lead's words (2026-09-26):** *"how hard would it be to make something like a panel where its
> not code but its actually like plug and play with robot configs and managing configs. its not like
> vexbuilder that is an entire massive project but its like specifically for managing the robots in shulib
> and its a small extension in vscode that is in addition but not needed"*; then *"lets do it for before
> implementing .vexbot and shupaths probably isnt going to be a thing it would jsut be connected to the
> .vexbot file ... I genuinely want to complete the full library asap"*. **The runtime-discovery ruling
> (2026-09-10, restated 2026-09-27) holds:** a human states only what cannot be measured — cartridge,
> drivetrain kind, which end is the front, *(re-check 2026-10-29: no longer — the end that leads Calibrate's push is the front, 2026-10-06)* the ports and sides as wired, sensors, mechanisms and their
> buttons; track width, wheel diameter, drive-motor reversal and gear ratio are MEASURED by shulib
> Calibrate (the record on the SD card, `shulib_drivetrain.bin`) and SHOWN read-only, never typed.
>
> **Runs:** ~~Part A after T4 Part 1b lands (D6), then Part B, each its own go, each in an isolated worktree
> (`.claude/worktrees/g3a`, branch `g3a`, from the tip at launch).~~ *[superseded 2026-09-29: seven parts,
> each its own go, each in an isolated worktree `.claude/worktrees/g3a-<n>` (branch `g3a-<n>`, from the tip at
> launch) — the amended section's parts table. T4 Part 1b no longer exists (folded into Part 3, 2026-09-28).]* *[re-check 2026-10-29: ten parts in two lanes — the re-check's parts table; worktrees `.claude/worktrees/g3a-<part>`.]* **Executor model: `opus`. Do not
> commit.** Live log: `docs/internal/chunks/G3a-PROGRESS.md`, **created FIRST**, appended as you go, a
> heading per part. **Hardware Assumptions numbers: none needed (§1.12); HA-181 is reserved — an executor
> who meets a hardware belief stops and tells the coordinator before taking it.** IMU1 takes none.
>
> **Probes** (evidence, never project code) — `.claude/coordinator/scratch/brief-G3a/v2/`:
> `orig/` (the tree at `a62067b`, `git archive`; its three table/driver files are byte-identical to the
> repo's — sha256 checked); `robot_tool.py` (the prototype generator/validator/decoder, ~1 030 lines);
> `extract.py` (builds robot two's file FROM the tree's bytes — no transcription); `robot_two.robot.json`
> (§1.1, verbatim); `marked/` (the tree with the five regions adopted); `gate_marked.log`, `gate_t2.log`
> (the full src build gate on the adopted and the panel-edited trees); `minisuite.sh` + `build_marked/`,
> `build_t2/` (the seven host test TUs that read robot two's tables, built with the suite's flags);
> `mut/` (hand-edit and robot-file mutations), `mutp/` + `mutp_tu.cpp` (a third mechanism vs the pin),
> `mutc/` (a loosened C++ predicate); `agree.py` + `agree_runner.cpp` + `agree/` (the validator/C++
> agreement harness; `agree/report.txt`); `sep_probe.sh` (a generated per-robot file vs step 0);
> `record_probe.cpp` + `rec_agree.py` + `rec/` (the host record decoder against the REAL encoder and
> decoder); `regions_pre1b.txt` / `regions_post1b.txt`; `panel/` (the extension prototype: pure model,
> `node --test`, the `.vsix`) and `vscode-iso/` (the `.vsix` installed into an ISOLATED VS Code profile —
> the user's own profile was not touched).
> **Re-check probes** (`recheck/`, 2026-09-28): `robot_tool.py` (v2's prototype with the amendments
> below, ~1 330 lines); `battery.py` + `battery.log` (26 cases, each in a FRESH copy of `v2/marked/src`:
> **26/26 PASS**); `muts/` (eight mutants, one per new rule, **each turns its cases red**);
> `agree_pin/` (the agreement harness with the selection pin evaluated: **27 agree, 0 disagree**; the
> validator's pin removed → fixture 24 **DISAGREE**); `p4/` + `build_p4/` (P4 rebuilt from scratch);
> `f1/` (the injections, and a line-splice the checker did not try); `panel/` (`@vscode/vsce` 3.9.2 on
> Node 20: model tests 5/5, `.vsix` packaged, the real repository URL).

## Amendment 2026-10-29 (evening) — the team lead's panel scope

> **READ THIS FIRST. Where it and anything below disagree, this section wins** (the re-check of 2026-10-29 below is
> history wherever this section changes it). Written 2026-10-30 from a READ-ONLY pass over main `6798aa5` (IMU1 Part A
> merged) and the worktrees `mech-a5` (`d37fa79`, MECH-A5 as finished, awaiting the coordinator's merge), `g3a-p1`
> (Part 1 built, in review), `l6-a1` and `vis1-a1` (in their review rounds), against `build-order.md`'s G3a entry
> (:1830-1840 at `81eb666`), the deviations table's sprint row (:2333 at `81eb666`; *fix 2026-10-30, C16: the
> first pass cited :1817-1838 and :2320*), NOTES 10-29 ~19:30–21:00 and the team lead's rulings of
> that evening. Probes E1–E4: `.claude/coordinator/scratch/brief-G3a-amend-1029b/` (evidence, never project code; "host
> test plant" where the library's model is meant). Every body passage these rulings make false is corrected in place
> and marked *(amended 2026-10-29 evening)*; the list is the section "Body passages corrected in place", below.
> **Fixed 2026-10-30 after a checker's 16 findings (C1–C16, each re-verified against main `81eb666` — MECH-A5 merged
> 2026-10-30 ~09:35, so every `mech-a5` citation below now reads the same lines on main):** one blocker (a partly
> stated settle, gains or feedforward group zeroed the library's other members — fixed by emitting assignments onto a
> default `ChassisConfig`, probe F1), seven majors and eight minors — all real (two with a detail corrected, below); every fix is in place, marked *(fix 2026-10-30, Cn)*,
> and listed in "### The checker's findings on this amendment (C1–C16)" near this section's end. Fix probes F1, F6,
> F13 and F14 sit beside the checker's own (C1–C3) in the probe directory.
> **Status: WAITS for the team lead's answers to D11–D20 and his go for each part and for the two library runs
> STEP1-A and STEP1-B. Part 5's launch waits for D11 (under D11 (a) the feel's tool half is inside Part 5); Part 3's
> for D19 (the extension's public name). Nothing in this amendment is built.**

### What he ruled (2026-10-29, evening), in his words

After seeing the concept page (`.claude/coordinator/panel_concept/robot-panel-concept.html`, published ~19:30): *"the
functionality of the panel is really bad, the customization needed to actually be able to make a bot is not there"*;
*"the route maker does not allow for care in what to do and is very rudimentary"*. His answers, each from an
AskUserQuestion (NOTES 10-29 ~19:30–20:15; memory `robot-config-panel-g3a` — the option texts he picked are in that
session's record; quotation marks below mark only the words recorded verbatim):

1. **DRIVING FEEL becomes panel data by Nov 21** — arcade or tank sticks, dead zones, curves, the ramp. This REVERSES
   D4 (a) of 2026-09-29 ("THE FEEL stays hand-written").
2. **DRIVE TUNING per robot by Nov 21** — feedforward, the gains, top speeds, settle tolerances, timeouts — with R5a's
   measured gains shown beside them.
3. **The CAMERA and WALL pages (Part S) by Nov 21** — VIS1-A2/B and L6-A2/B resume; if it is tight, the ROUTE MAKER
   comes first (his ruling, as recorded).
4. **A FULL ROUTE MAKER** — he picked all four: (a) every auton step as an ordered list on the field; (b) care on every
   step — speed cap, timeout, arrival tolerance, and what happens if it fails (stop the auton / skip it / carry on);
   (c) mechanism timing — start-and-keep-driving or wait-until-done, waits on a time or a sensor, triggers PARTWAY
   through a drive; (d) red/blue mirroring and a 15 s / 1 min timeline (rough estimates — the panel never simulates,
   his ruling of 2026-10-08).
5. **Distribution: "GitHub now, store later"** — every panel version a `.vsix` on a GitHub Release of the public repo
   (SHU-ROBOTICS/shulib, MIT); after Nov 21, once it has run on a real robot, the VS Code Marketplace and Open VSX
   under a club publisher account he creates. Each actual release still needs his word.
6. *(Context only — a separate brief.)* The team's string mechanism is a LIFT; before the panel lands the library gains
   a per-mechanism CURRENT CAP, SNAPPED-STRING DETECTION (by motor load by default, an output rotation sensor optional)
   and LIVE TUNING ON THE ROBOT.

He chose "describe it" over a second concept page. What he was told that evening (memory): the feel and the tuning
move "make and drive a robot" to about Nov 14–20, and routes plus the sensor pages both by the 21st "need a lucky
run". The schedule below re-derives that from the parts.

### 1. The FEEL becomes panel data — ruling (1)

**What exists (opened).** `teleop::DriverProfile` (`include/shulib/teleop/input_shaping.hpp` :187-205): per axis
(`forward`, `left`, `yaw`) an `AxisShaping {deadband, expo}` (:142-159; `valid()` = deadband in [0, 1), expo in
[0, 1]), ONE `SlewConfig {ratePerSecond}` (`slew_limiter.hpp` :40-45; 0 or below = OFF) and a `DriveMode`
(`drive_modes.hpp` :51-63: `ArcadeBody`, `ArcadeField`, `TankSticks`). `teleop::DriverLoopConfig` (`teleop_loop.hpp`
:196-223): `tickPeriod` (a driver file never states it — MECH-A5's `DriverFile` fills the brain's tick and refuses any
other, `src/common/driver_file.hpp` :84-90 in `mech-a5`), `rezeroButton`, `releaseBrakeMode` and `protection` (T4
Part 2's numbers, HA-175, HA-176). Robot two's, in `mech-a5`'s `src/team/robot_two_driver.hpp`: `kFeel{.mode =
ArcadeBody}` and `kLoop{.rezeroButton = Button::Y, .releaseBrakeMode = Coast}` (:56-63), BELOW MECH-A5's
`// [/driver file two]` (:54), as RC5 asked.

**Found (E2).** Nothing in the build refuses an out-of-range feel. A generated `kFeel` with a curve of 1.5 compiles and
passes through `DriverFile` (E2's `feel_bad_expo_nocheck.cpp`, exit 0); the shaping functions then void their
monotonic promise in silence (`input_shaping.hpp` :127-129), and `DriverFile` checks only the re-zero and the tick
(:81, :84). A control on the re-zero button IS refused by `DriverFile`'s `static_assert` (E2, as CK9 said). Hand-typed,
the out-of-range number was a typing risk; written by a form, it needs a refusal the build owns.

**The design (D11 (a)).**
- **The robot file** gains a `driving` block; every field is optional, and an absent one is the library's default
  (HA-112):
  ```
  "driving": {
    "mode": "arcade" | "arcadeField" | "tankSticks",       // DriveMode
    "forward": {"deadband": 0.05, "curve": 0.0},           // AxisShaping {deadband, expo}
    "strafe":  {"deadband": 0.05, "curve": 0.0},           // DriverProfile::left
    "turn":    {"deadband": 0.05, "curve": 0.0},           // DriverProfile::yaw
    "rampPerSecond": 0.0,                                  // SlewConfig::ratePerSecond; 0 = off
    "rezeroButton": "Y",
    "onRelease": "coast" | "brake" | "hold",               // releaseBrakeMode
    "provenance": "who set it, after which driving session",
    "notes": ["..."]
  }
  ```
- **The C++:** a `feel` region in each driver file over THE FEEL's heading, `kFeel` and `kLoop` (robot two's: `mech-a5`
  :56-63, adopted byte for byte, the differential logged). The emitter writes only the STATED fields, in declaration
  order (N11), and never `.tickPeriod` or `.protection`. The cookbook's chapter 6 quotes robot two's feel lines, so a
  feel change is not done at `generate` in THIS repository: the witness list names the listing (D5's rule).
- **The words that describe the feel** *(fix 2026-10-30, C11)*. Robot two's driver-file header (main `81eb666`) says
  what the feel does in prose a panel change would make false: the stick lines (:17-20 — arcade: "left stick … drive",
  "right stick … turn"), "Y field-centric re-zero" (:29), "let go of the sticks and the drive coasts" (:30), and THE
  FEEL's own comment (:57-58, "The library's default: arcade …, a smooth deadband of 0.05, no curve, no ramp"). So:
  the feel's comment lines are INSIDE the `feel` region, written from `driving`; and the header's whole
  stick-and-button map (:17-32 — the stick lines, the button lines, the re-zero and release lines, the presets note)
  becomes ONE `control words` region (the region A5-D2 asked for), written from `controls` AND `driving` — the
  vocabulary's wording per control and per mode, robot-specific sentences (such as "the camera end, on robot two's
  record of 2026-09-25") from the file's `notes`, robot two's adopted byte for byte with its differential logged. The
  map is one block because its feel lines and button lines are interleaved (:17-20, :29-30 feel; :21-28, :31-32
  buttons): two regions would need the header reordered by hand first. §1.2's hand-editable list loses "the sticks"
  and "the re-zero line" (only the protections paragraph stays hand-written). Both regions' prose joins the part's
  prose pass (pace rule 4).
- **The validator:** `AxisShaping::valid()` per axis, mirrored and pinned by run-time agreement fixtures; the re-zero
  against the controls (CK9's compile-refusal fixture now reads the re-zero FROM THE FILE — M34 re-worded); and one
  rule of the tool's own, named as stricter than the C++: a negative or non-finite ramp is refused (the C++ reads it as
  OFF). `arcadeField` carries a note: it needs a trusted heading and falls back to robot-centric without one
  (`teleop_loop.hpp` :225-233).
- **One hand edit outside the regions, in Part 5:** `src/common/driver_file.hpp` gains `static_assert(Feel.valid(),
  "...")`, so the build refuses an out-of-range feel, hand-written or generated, with its compile-refusal fixture
  (rule 4).
- **The FEEL page (Part 6):** the mode; per axis the dead zone and the curve, with a response plot whose points come
  from the TOOL (a `curve` query computing `shapeAxis` in Python, pinned by agreement fixtures against
  `teleop::shapeAxis` — the panel draws the curve, it never computes it); the ramp; the re-zero button (a dropdown that
  warns when a control takes it); what the drive does on release.
- **A new robot (Part 4):** the starter driver file's `feel` region holds the library's defaults, emitted from an
  empty `driving` block.

**Parts and sizes.** Part 5 **+¾–1** (the block, the emitter, the validator, the region and its differential, the
`static_assert` and its fixture, the robot-file chapter's feel section; *fix 2026-10-30, C11: +¼ for the
`control words` region over the whole stick-and-button map and the feel's own comment*); Part 6 **+½–¾** (the FEEL
page and the curve query). If Part 5 launches before he answers: D11 (b).

**D11 — The driving feel as data: which part carries it, and do the drive protections come with it? RECOMMENDED
(a).**
- **(a) RECOMMENDED.** The design above: the tool half inside Part 5, the page inside Part 6; the drive protections
  (the over-current cut and its numbers, the dropped-motor rule) stay the library's defaults and are not on the page.
  Why Part 5: it already adopts robot two's driver-file regions, and without this it would build "read the re-zero out
  of THE FEEL" (D4 (a)'s rule) only for this ruling to delete it. What it costs against D4 (a): the feel becomes
  profile schema (F7 at G3; the note to VexBuilder grows, and Q20's "keep fields you do not know" covers it); a student
  sets the feel in a form, though the generated lines stay in the driver file for anyone to read.
- **(b)** The same work as its own **Part F**, right after Part 5 (~1–1½, both halves) — only if Part 5 has launched
  before his answer. Cost: robot two's driver file is adopted twice, and Part 5's re-zero reading is built, then
  replaced.
- **(c)** (a), plus the drive protections on the page. Cost: +¼–½; a form can switch the over-current cut off, and
  every one of those numbers is unmeasured (HA-175, HA-176) — T4's ruling kept the cut ON with a push check before any
  match.

### 2. DRIVE TUNING per robot — ruling (2)

**What exists (opened).** `motion::MotionConfig` (`include/shulib/motion/motion_config.hpp` :150-249): `wheelFf`
(:154), `translation` and `heading` `AxisGains` (:158-160), `maxLinearSpeed` / `maxAngularSpeed` / `maxWheelSpeed`
(:164-168), the translation, heading and brake `SettleConfig`s (:172-184), `defaultTimeout` (:188), `rotationRadius`
(:192), `stall` (`OdoStallCheckConfig`, :199 — `window`, `minSpinTravel`, `motionRatio`, plus `wheels` and
`independentMotionSource`, which `Robot::effectiveTuning()` sets for a tank, and its OWN `rotationRadius`, 7.0 in,
HA-17/HA-52, which nothing sets — `odo_stall_check.hpp` :129; *fix 2026-10-30, C12: the first pass said the builder
sets the stall check's `rotationRadius` too*) and `tankSteer` (:205, TANK1, HA-179). **The nested structs' OWN
defaults are zero** (*fix 2026-10-30, C1*): `SettleConfig` (`control/settled_util.hpp` :30-32 — "a default-constructed
SettleConfig effectively never settles"), `AxisGains` (`motion_config.hpp` :56-58, `integralLimit` ∞) and
`FeedforwardGains` (`control/feedforward.hpp` :37-40); the library's non-zero values are `MotionConfig`'s member
initializers (:154, :158/:160, :172-184). `TankSteerConfig` and `OdoStallCheckConfig` carry their own non-zero defaults. `validate()` (:219-248) refuses non-finite gains and non-positive budgets; it does not descend into the settle
or stall configs, whose owners check them when a motion is built. A robot's comes from `RobotConfig::tuning`
(`config/robot_config.hpp` :520, :576 — `chassis::ChassisConfig {motion, scheduler}`, `chassis/chassis.hpp` :201-204).
**Where robot two's comes from today: nowhere typed.** `bootConfigFrom` leaves `c.tuning` as
`chassis::ChassisConfig{}`, "the library's PROVISIONAL defaults, UNTOUCHED", on purpose (`src/common/boot_source.hpp`
:132-136, :208 — the same in `mech-a5`), and `robot::Robot::effectiveTuning()` sets only the stall check's geometry for
a tank (`include/shulib/robot/robot.hpp` :1052-1059). Every program that boots robot two drives on HA-50/51/52/179's
placeholders. The X-drive's `RobotConfig` (`src/robots/xdrive_config.hpp`) is hand-written and stays so. **R5a** (Part
A landed 2026-10-06; B, C, D not built, none in the sprint): Part B puts the measured characterization in
`RobotConfig` and lets it OWN `wheelFf` and the three speed budgets when present — "a typed `wheelFf` is honoured only
without one", the budgets by D3's rule STARTING FROM the typed ones — and "G3a's profile never carries it"
(`R5a-gains-and-noise-on-the-sd-card.md` :746-756); its numbers live on the SD card (`shulib_characterization.bin`,
bound to the drivetrain record). Part C (Calibrate's GAINS step) comes after CAL1 (`CAL1-standard-calibrate.md`
:1355).

**Found (E1).** A `chassis::ChassisConfig` member appended to a constexpr aggregate with a `{}` default keeps every
existing initializer compiling; a partly stated one keeps the library's defaults for every other TOP-LEVEL field; both
pass `validate()` — host `-Werror` and ARM. ~~a partly stated one keeps the library's defaults for every other
field~~ *(fix 2026-10-30, C1 — the BLOCKER: false for a NESTED group. E1 stated each nested group in full. The
checker's probe C1 (`checker/nested_default.cpp`) designated `.translationSettle = {.maxError = 0.75}`,
`.heading = {.kD = 0.1}` and `.wheelFf = {.kV = 0.17}`: the unstated members took the nested structs' zeros —
`maxErrorRate` 0 and `settleTime` 0 (library 1.0, 0.1), heading `kP` 0 (library 4), `kS` 0 (library 1) — and all
three pass `validate()`; `SettledUtil` accepts zeros (it requires only ≥ 0, `settled_util.hpp` :50-52). On a robot:
with `maxErrorRate` 0 the error rate is never exactly 0 under noise, so every translation never settles and times out;
heading `kP` 0 means no heading correction. Re-verified by reading the three structs (above).)* **Found (F1, the fix
probe, `fix1/lambda.cpp`):** the region written as a constexpr lambda that ASSIGNS only the stated fields onto a
default `chassis::ChassisConfig` keeps every unstated member — nested or not — at the library's value BY
CONSTRUCTION (`static_assert`s on `maxErrorRate`, `settleTime`, heading `kP`, `kS`); the table stays `inline
constexpr`; host `-std=c++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror` (runs, `validate()`
passes) and ARM `arm-none-eabi-g++ 13.2.1 -std=gnu++20 … -Werror` exit 0 (`fix1/lambda.log`).

**The design (D12 (a)).**
- **The robot file** gains `driveTuning`; every field is optional (absent = the library's provisional default, shown
  with its HA number). ~~in the C++'s own units — inches, seconds, radians — so the emitted literal IS the typed
  number (the page shows degrees beside radians)~~ *(fix 2026-10-30, C16: ONE angle unit at the human edge across the
  panel's two files — DEGREES, F3's rule "degrees only at the API edge" (`docs/roadmap.md` :137), as the route file
  already does (R.4). The angle-bearing fields carry a `Deg` in their name (`maxAngularSpeedDegPerS`,
  `headingSettle.maxErrorDeg`, `…maxErrorRateDegPerS`, `heading.integralLimitDegPerS`, `tankSteer.maxSteerDeg`,
  `tankSteer.crossGainDegPerIn`, `tankSteer.yawCorrectionLimitDegPerS`); the heading GAINS need no conversion (kP is
  1/s, kI 1/s², kD unit-free whatever the angle unit). The emitter converts with ONE function, pinned by an agreement
  fixture against `math::Angle::degrees(x).value()`, and writes the typed degrees in a comment beside each radian
  literal, so the C++ still shows the number a student typed. Inches and seconds unchanged.)*:
  ```
  "driveTuning": {
    "feedforward": {"kS": ..., "kV": ..., "kA": ...},     // used until a GAINS run is on the card (R5a)
    "translation": {"kP": ..., "kI": ..., "kD": ..., "integralLimit": ...},
    "heading":     {"kP": ..., "kI": ..., "kD": ..., "integralLimitDegPerS": ...},
    "maxLinearSpeed": ..., "maxAngularSpeedDegPerS": ..., "maxWheelSpeed": ...,   // R5a's D3 rule starts from these
    "translationSettle": {"maxError": ..., "maxErrorRate": ..., "settleTime": ...},
    "headingSettle": {"maxErrorDeg": ..., "maxErrorRateDegPerS": ..., "settleTime": ...}, "brakeSettle": {...},
    "defaultTimeout": ...,
    "tankSteer": {"crossGainDegPerIn": ..., "maxSteerDeg": ..., "fadeDistance": ..., "leadTime": ...,
                  "yawObserverTime": ..., "yawCorrectionLimitDegPerS": ...},
    "stallCheck": {"window": ..., "minSpinTravel": ..., "motionRatio": ...},
    "provenance": "who tuned it, how, when"
  }
  ```
  Never in it: `scheduler` (the fault policy and the loop monitor), the stall check's `wheels` and
  `independentMotionSource` (`Robot::effectiveTuning()`'s), and anything the robot measures — ~~`rotationRadius`~~
  *(fix 2026-10-30, C12: `rotationRadius` is OUT of the typed schema, both copies — `MotionConfig::rotationRadius`
  and the stall check's own. Each is a centre-to-wheel distance, half the track width on a tank, which Calibrate
  MEASURES (the runtime-discovery ruling; D12's own "never anything the robot measures"); both stay 7.0 in (HA-17,
  HA-52), shown READ-ONLY on the page with that note. Deriving them from the record is a library item, logged in §8,
  not G3a's.)*
- **The C++:** `ChassisTable` gains `chassis::ChassisConfig tuning{};`, appended (E1: the bench bot's table and every
  existing initializer unchanged). A robot's `.tuning = …` sits in a new `[chassis tuning]` region at the END of its
  initializer — robot two's after `[chassis]`'s end marker, or after `.camera` / `.wallSensors` if VIS1-B or L6-B land
  first; a member added later goes after this region's end marker (the rule Part 1's region already states).
  *(fix 2026-10-30, C1)* **The emitted form is a constexpr lambda that ASSIGNS each stated field onto a default
  config** (F1), one line per stated field in the struct's declaration order, never a designated initializer of a
  nested group:
  ```
  .tuning = [] {
      chassis::ChassisConfig c{};
      c.motion.translationSettle.maxError = 0.75;
      c.motion.headingSettle.maxError = 0.0174532925199433;  // 1 deg
      c.motion.maxLinearSpeed = units::Velocity{50.0};
      return c;
  }(),
  ```
  Every unstated member keeps `MotionConfig{}`'s value by construction, and the generated code duplicates no library
  default (a later change to a provisional default reaches every robot that did not state that field). The other fix
  the checker offered — write every member of a touched group, filling the unstated ones from a copy of the library's
  defaults pinned by an agreement fixture — was NOT taken: it copies library numbers into team files, where they read
  as chosen. Robot two's file states no tuning, so its region is EMPTY and its programs do not change; a host test
  pins robot two's effective tuning equal to `chassis::ChassisConfig{}`, field by field (`test/boot_source_test.cpp`'s
  `requireDefaultTuning()`, :219-258, already walks every struct by member count).
- **The boot:** *(fix 2026-10-30, C12: re-read — the first pass named the wrong callers)* the pure core takes the
  table's tuning as a parameter: `decideBootSourceFrom` (`boot_source.hpp` :357) and `bootConfigFrom` (:143, :181)
  gain it, and the PROS shim `decideBootSource` (`boot_source.cpp` :35-54 — the one place that reads `kChassis`) passes
  `kChassis.tuning`. The programs call `decideBootSource(kTag)` (`competition_program.cpp` :668,
  `auton_tester_program.cpp` :464, `drive_program.cpp` :632) and DO NOT change. The test calls do: ~30 calls of the two
  core functions in seven test files (`boot_source_test.cpp` 20, `mechanism_wiring_test.cpp` 5,
  `driver_panel_claws_test.cpp` 4, `robot_two_driver_test.cpp` 3, `drive_program_source_test.cpp`,
  `boot_check_test.cpp`, `driver_heading_test.cpp` 1 each) — each passes the tuning it means, with no defaulted
  parameter (a default would let the shim forget it in silence; M49 stays red either way). A stated tuning the library
  would refuse REFUSES the boot with one line naming the robot file's field — never a robot whose every motion faults on
  its first tick (`core/check.hpp`: the scheduler turns a broken precondition into `FAULT_ABORT`). The refusal is a PURE
  function in `boot_source.hpp` (for example `tuningProblem(const chassis::ChassisConfig&) -> const char*`, the first
  failing field's robot-file name, "" when none) — not a call that throws on the brain. It checks THREE things on the
  table's tuning: `MotionConfig::validate()`'s conditions (tankSteer's included — `validate()` calls
  `tankSteer.valid()`, :245), each settle group's three bounds ≥ 0 (`SettledUtil`'s, `settled_util.hpp` :50-52), and the
  stall check's three TYPED fields (`window` finite and > 0, `minSpinTravel` > 0, `motionRatio` inside (0, 1) —
  `odo_stall_check.hpp` :163-168). The stall check's `wheels` and `independentMotionSource` are filled AFTER, by
  `Robot::effectiveTuning()` from the record (`robot.hpp` :1052-1059), and checked where they are today (the default
  wheels are a valid stand-in, `drive_geometry.hpp` :66-70). Host tests pin `tuningProblem` against the real checks,
  each catching the real `PreconditionError`.
- **The validator:** ~~those four checks~~ *(fix 2026-10-30, C12: THREE — `validate()` already covers tankSteer)* those
  three checks mirrored, pinned by run-time agreement fixtures (each catches the real `validate()`'s or constructor's
  `PreconditionError` on the host); plus ONE rule of the tool's own, named as stricter than the C++: a stated
  translation or heading settle group's `maxError` and `maxErrorRate` must be > 0 (the C++ accepts 0, which "effectively
  never settles", `settled_util.hpp` :26-28 — exactly C1's failure, typed on purpose).
- **The DRIVE TUNING page (in Part T):** each field with the library's default and its HA number beside it, the typed
  value with its provenance, and, for `feedforward` and the three top speeds, a MEASURED column: "none on this robot's
  card yet — Calibrate's GAINS step (R5a Part C) is not built; until then the typed values drive the robot". When R5a
  Part B lands, its characterization decoder joins `tools/robot_record.py` (the drivetrain record's pattern: Python,
  pinned to the C++ decoder) and the column fills; R5a's brief carries that, as the chunk that lands second (the
  coordinator amends R5a's OUT line, ~:1310, which names the display G3a's). The two `rotationRadius` values shown
  read-only (above).
- **A new robot (Part 2):** its table carries `.tuning` from its file — added by whichever of Part 2 and Part T lands
  second (the tool cannot emit a block it does not know yet).

**Parts and sizes.** NEW **Part T** — DRIVE TUNING (the tool lane's work, its page in the panel; beside Part 2, after
Part 5's build and Part 3's merge): ~~**~2¼–3**~~ **~2½–3¼** (the block and its three mirrored checks plus the tool's
own settle rule ~1; the degrees conversion and its agreement fixture ~¼ — *fix 2026-10-30, C16*; the member, the
lambda emitter, the boot copy and the pure refusal with their host tests and the ~30 test-call updates ~½–¾; the
region and the docs ~¼; the page ~½–¾). Part 2 **+¼**.

**D12 — Where the drive tuning lives, and which numbers a team may type. RECOMMENDED (a).**
- **(a) RECOMMENDED.** The design above: every number typeable, with feedforward and the three top speeds marked
  "used until Calibrate measures them" — R5a's own rule, unchanged.
- **(b)** As (a), but feedforward and the top speeds are NOT typeable — measured only (the runtime-discovery ruling of
  2026-09-10). Cost: nothing can change them before R5a Part C, which is not in the sprint, so robot two drives on
  placeholders past Nov 21.
- **(c)** A separate per-robot tuning table with its own selection function. Cost: a third per-robot table — step 0's
  `TABLE_FILES`, the selection pins and the gate's tables all change — for the same data.

### 3. The CAMERA and WALL pages — ruling (3), re-timed

**What changed.** Ruling (3) un-pauses VIS1-A2/B and L6-A2/B (the sprint row, :2333 at `81eb666`, had paused VIS1-A2/B and ALLY1,
L4, AF1; L6 kept). VIS1-A1 and L6-A1 are in their review rounds. Part S needs only the BLOCKS: VIS1's camera block
(`config/camera_config.hpp`, VIS1-A1) and robot two's `.camera` (VIS1-B — which documents into VIS1-A2's new camera
chapter, so in practice after A2); L6's walls block (`config/wall_sensor_config.hpp`, L6-A2) and robot two's
`.wallSensors` (L6-B, on the 2026-11-23 track).

**Re-timed: Part S splits in two.**
- **S1 — the WALLS half** (the walls block in the tool, robot two's `.wallSensors` adopted, the SENSORS page's walls):
  after L6-B and Part 3; **~1½–2**. It rides the competition track (L6-B lands ~Nov 6–9) and can land by Nov 21.
  *(fix 2026-10-30, C6: re-derived — L6-B merges ~Nov 8–10 with MECH-A6 first, and S1 lands ~Nov 18–22: at the edge
  of Nov 21, D20.)*
- **S2 — the CAMERA half** (the camera block, robot two's `.camera`, the page's camera): after VIS1-B and S1;
  **~1½–2**. Under "route maker first", VIS1-A2 → VIS1-B get slots only from ~Nov 16, so S2 misses Nov 21 (the
  schedule below). *(fix 2026-10-30, C6: ~Nov 25–Dec 2 — the serial chain cannot land before ~Nov 25 even at the low
  sizes.)*
- Together **~3–4** (was 2½–3½; +½ for the split). The robot-two rule: the hand-written `.camera` / `.wallSensors`
  lines are adopted into a `[chassis sensors]` region (or by moving an ADJACENT region's end marker) — whichever leaves
  no hand-written line inside a region; Part T's `[chassis tuning]` may sit between. The CALIBRATION page's flip
  warning (CK10) names the walls from S1 on and the camera from S2 on.

### 4. THE ROUTE MAKER — Part R's full section — ruling (4)

> Supersedes the re-check's parts-table row for Part R ("waypoints … saved in F8's `paths[]`", ~4–6, RC10) and, for
> routes, §1.16's "export: hooks only". Ruled 2026-10-08 (MECH-PANEL §4 item 7b: a field view; mechanism steps from
> the robot's route ids; F8 so VexBuilder can take the route and add its physics; NO simulation, a geometry visual
> yes) and widened 2026-10-29 evening (above).

**What exists (opened).**
- **`chassis::Routine`** (`include/shulib/chassis/routine.hpp`; F10 FROZEN, MECH-A4's block not yet): EAGER steps that
  block in the order they read — `startAt`, `moveTo`, `driveTo`, `reverseTo`, `strafeTo`, `turnTo`, `face`, `faceAway`,
  `followTrajectory`, `brake`, `hold`, `pause`, `waitFor`, `then` — and MECH-A4's `move`/`startMove`,
  `home`/`startHome`, `run`/`startRun` (for a time, or `Until`: `seesPiece`, `stopsSeeingPiece`, `stalls`,
  `.orAfter(t)`), `set` (a pneumatic position — no start form), `spinUp`/`startSpinUp`, `stop`, `waitUntilDone`
  (:568-671). Per step, `MotionOptions` = `timeout`, `maxLinearSpeed`, `maxAngularSpeed` only (`chassis.hpp`
  :212-239) — **no arrival tolerance**; its field SET is "deliberately additive-open" (:209-211). **One error policy,
  frozen:** the first failure parks the drive and skips the rest (:109-133); "a routine that must CONTINUE past a
  failure branches on the verbs directly" (:134-139); A4's verdict policy: SUCCEEDED and a counted UNCONFIRMED (robot
  two's sensorless grab) continue, every other verdict stops (:543-552). F10's ADDITIVE path: "new steps, new
  observers, new RoutineResult fields, appended RoutineStopCause enumerators" (:21-24).
- **`followTrajectory` STOPS at each point** ("sequential moveTo legs, settling at each", `chassis.hpp` :324-355).
  There is no smooth-curve verb; F8 names `"arc"` a gap until the roadmap's blending item (`vexbuilder-integration.md`
  :281 and the paragraph after its rename table).
- **F8** (`config/route.hpp`; the Freeze Register's row, `docs/roadmap.md` :142): `Waypoint {x, y, heading,
  headingMode, motion ∈ Start/MoveToPose/DriveTo/TurnTo/StrafeTo/Hold, reverse, Constraints {maxLinear, maxAngular,
  timeout}, markers[4] of an id and ≤ 2 int args}`, ≤ 32 waypoints. Markers fire only AFTER a leg settles (ruling R2,
  `runner/path_runner.hpp` :45-56); the route stops on the first failure (:33-44); MECH-A4's route ids are blocking
  only (`runner/mechanism_routes.hpp` :15-25: "a step that starts mid-leg is the mechanism core's later 'triggers
  partway through a drive'"). ADDITIVE under F8: new enumerators and built-in ids at the end, new `RunReport` fields,
  new registry members, **new optional fields in the file schema**. NOT on that list: new members on
  `Waypoint`/`Constraints`/`Marker` (the pin test checks names and types by concept, `test/f8_signature_pin_test.cpp`
  :24-35, so an appended member would compile — but the register does not call it additive: that would be a ruling).
- **The auton list:** `competition::AutonEntry {name, kind, routine | route, start, endActionAt, hardStopAt,
  endAction}` — a name ≤ 29 printable ASCII, no '%', unique, not NONE (`competition/auton_registry.hpp` :93-94,
  :294-318); `AutonKind` Match or Skills (:110-114); at most **16** entries (:90). Robot two has two (`src/team/autons.cpp`
  in `mech-a5`: the 40 in auton — "runs in the 30 s autonomous period", `endActionAt = 27_s`, `hardStopAt = 29_s`,
  :71-84 — and the acceptance run).
- **The wall reset** (L6-A2, in flight): `competition::resetFromWall(ctx, {.wall, .sensor, .maxCorrection, …})`
  through `then()`, `.continues()` true when SEATED or when REFUSED and not `required`; FIELD starts only (L6 brief
  :613-660).
- **A sensor wait is expressible today:** `MechanismService::find(declaration)` and `Slot::sees()` are public
  (`mechanism/service.hpp` :1767-1806, :288).

**Found (E3).** ONE generated route ~~using every step form above~~ *(fix 2026-10-30, C14: the first pass said
"every step form above"; E3 has the fifteen forms named next and no others)* — `face` and `driveTo` with a turn cap, a
speed cap and a timeout, `startRun` (for a time) then `reverseTo` then `waitUntilDone`, `set`, a `waitFor` on the
claw's piece sensor, `turnTo`, `faceAway`, `moveTo` (a fixed heading), `followTrajectory`, `run` (for a time),
`pause`, `hold`, `brake` — plus a red/blue pair of registry entries compiles against MECH-A5's tree, host (`-Werror`)
and ARM; re-run at main `81eb666` after A5's merge, host and ARM exit 0 (`fix14/e3_main.log`). **NOT in E3** (robot
two has none of those kinds, or the form is not in the library yet): `move`/`startMove`, `home`/`startHome`,
`spinUp`/`startSpinUp`, `stop`, a power run UNTIL a condition — each compiled and run by
`test/mechanism_step_forms_test.cpp` (it calls every one of them); `strafeTo` — `test/chassis_routine_test.cpp` and
the F10 pin `test/routine_signature_pin_test.cpp`; the keep-heading `moveTo` (the mixed-tier idiom over
`Chassis::pose()`, `chassis.hpp` :480) — compiled by no probe yet: S37 compiles it; the wall reset — not landed
(L6-A2). A missing position is refused at build by name (`no_such_position_on_this_mechanism`); a pneumatic has no
start form (`startSet` does not exist). **So the route maker needs only four things the library does not have:** an
arrival tolerance per step, an on-failure choice per step, triggers partway through a drive, and a smooth curve.
*(fix 2026-10-30, C2/C3: and two rules for when steps are skipped or cut — a cut step always stops the chain, and a
wait for a mechanism whose start never ran is a counted failure, not a throw — STEP1-A/B, R.13.)*

#### R.1 What a route compiles to — D13

**D13 — What a route drawn in the panel becomes. RECOMMENDED (c).**
- **(a) F8 data, run by the library's route runner (`PathRunner`).** Gain: ONE format with VexBuilder, end to end — the
  10-08 ruling's words. Cost: F8 cannot say face, face away, start-and-keep-driving, wait until done, wait on a
  sensor, a partway trigger, a per-step tolerance or a per-step on-failure choice. Each is a new enumerator, a new
  built-in id or a new `Waypoint` member (the last a reading of the freeze he must make); start forms as route ids push
  names past F8's 23 characters (`<name>_start_until_piece`); a wall reset as a route id needs the auton's run guard and
  start mode, which a route handler does not have (`mechanism_routes.hpp`'s step context carries the chassis and the
  service only); VexBuilder must accept every addition; 32 waypoints and 4 markers each cap the route. Library work
  ~6–9 builder-days before the panel half: misses Nov 21.
- **(b) Generated C++ only:** each route becomes `chassis::Routine` code in a marked region of the robot's autons file.
  Gain: every step the library has today (E3) plus STEP1-A/B's three; nothing in F8 moves; the students read their
  auton as ordinary Routine lines and can explain each one — the generator is a fixed program turning their choices
  into code, nothing else writes it; no cap but the registry's 16. Cost: VexBuilder cannot take a panel route —
  reversing the 10-08 ruling's reason for F8.
- **(c) RECOMMENDED: both — the robot runs the generated C++ of (b); the panel also EXPORTS each route as F8 `paths[]`
  for VexBuilder,** with what F8 cannot say carried in ONE optional `shulib` object per waypoint (F8: "new optional
  fields in the file schema" are additive — `docs/roadmap.md` :142). F8's in-memory types and `PathRunner` do not
  change. Gain: (b)'s, and the 10-08 ruling kept — VexBuilder simulates the motions it knows and keeps the rest (if it
  honours Q20). Cost: +½–¾ in Part R1 for the exporter and its fidelity test; the export is lossy for VexBuilder's
  simulation in what it does not understand (start forms, triggers, choices, tolerances) — the exported file and the
  panel say so; a route VexBuilder edits and sends back merges into the route file at G3.

#### R.2 Scope — IN
- **The route file** `src/team/<id>_routes.json` (one per robot; A5's per-robot team-file rule makes `<id>_*` the
  robot's), checked against its robot file — the schema in R.4.
- **`tools/robot_routes.py`** — standard library only, its own module (routes are a separate file type, and the route
  lane must not share `tools/robot_tool.py`'s lines with the tool lane — the rebase rule's point 5). Loaded BY PATH like
  `robot_record.py` (CK11 (b)); `robot_tool.py routes …` forwards to it. It reads the robot file through
  `robot_tool.py`'s loader (one parser). Commands: `validate`, `generate`, `check` (freshness), `describe` (the step
  vocabulary and the robot's mechanisms, presets, sensors and walls, for the page), `timeline`, `mirror` (a preview),
  `export-paths` — each with `--json`.
- **The robot's FOOTPRINT** in the robot file — `footprint {lengthIn, widthIn, provenance}`, human-stated (no program
  measures it; both robots are 15" this season) — added to `robot_tool.py`'s schema by Part R1 (a few lines; the
  rebase rule's point 1). Drawing only; no C++.
- **The `[routes]` region** in the robot's autons file (R.11).
- **The step vocabulary** (R.5), **care on every step** (R.6), **mechanism timing** (R.7), **mirroring** (R.8), **the
  timeline** (R.9), **the F8 export** (R.10).
- **The ROUTES page (Part R2):** the field (the season's walls, Loaders, Load Zones and start pieces from L6's
  `override_walls.hpp` — the tool's copy pinned against the C++ constants by an agreement fixture; goals and other
  elements from the manual, labelled "a drawing aid, never a check"); the ordered step list (add from the vocabulary,
  reorder, duplicate, delete, label); each step's form (its care and timing); the robot's outline to scale at each
  step's end pose; partway triggers drawn as ticks on their leg; skip arrows; a step-through that moves the outline from
  end pose to end pose in order (geometry only); the mirror preview; the timeline bar; every refusal on its JSON path.

#### R.3 Scope — OUT
- **A smooth curve** (D16 — its decision block is R.7a; *fix 2026-10-30, C4: the first pass had none*).
- **Strafe on a tank** — refused; offered on a holonomic robot only, and generation is tank-only until §5 Q22 (b).
- **Drive to a piece** (L4, after the panel) and **the alliance colour chosen on the brain** (ALLY1, after the panel —
  D17).
- **Any simulation** (the 10-08 ruling): the step-through moves the outline between END poses; it never shows speed,
  drift or timing. VexBuilder is the one simulator; shulib's host test plant stays inside the library's tests.
- **The expanded (24") outline and collision checks against field elements** — the outline is a drawing aid; "does it
  fit?" is a person looking.
- **Importing a `.vexbot`'s paths** (G3); **editing hand-written autons** (the panel writes only its region); **measured
  step times** (D18 (c), later); **"eject this route as hand-written code"** (a student can copy the generated lines out
  of the region by hand; a tool command for it is later).

#### R.4 The route file — the contract Parts R1 and R2 build against
```
{
  "schemaVersion": 1,
  "robot": "robot_two",                         // the robot file's id; every name below is checked against it
  "routes": [ {
    "name": "left three",                       // ≤ 29 printable ASCII, no '%', unique, not NONE; ≤ 24 if mirrored
    "kind": "match" | "skills",
    "start": {"mode": "field", "x": -55.7, "y": -60, "headingDeg": 90} | {"mode": "relative"},
    "drawnFor": "red" | "blue",                 // field starts; relative starts too under the midline flip (R.8)
    "mirror": true,                             // also generate the other side (D17); refused on a relative start
                                                //   under the half turn (it would mirror to itself, R.8)
    "endActionAtS": 27, "hardStopAtS": 29,      // tool defaults: 27/29 match, 57/59 skills (his to change)
    "notes": ["..."],
    "steps": [ {
      "label": "second piece",                  // optional; unique; a skip target
      "do": "driveTo", "x": -24, "y": -36,      // the step and its own fields (R.5)
      "care": {"maxSpeedInPerS": 40, "maxTurnDegPerS": 0, "timeoutS": 2.5,
               "toleranceIn": 1.0, "toleranceDeg": 0,
               "onFail": "stop" | "carryOn" | {"skipTo": "second piece"}},     // R.6
      "during": [ {"at": {"percent": 60} | {"afterIn": 20} | {"afterDeg": 45},
                   "then": {"do": "move", "mechanism": "lift", "preset": "high", "boundS": 2}} ],  // R.7
      "notes": ["..."]
    } ]
  } ]
}
```
Units: inches, seconds, degrees in the file (F8's file uses degrees for headings, `vexbuilder-integration.md` §4.1.2);
converted once at generation. `0` or absent in `care` = the robot's default (F8's and `MotionOptions`' sentinel).
*(fix 2026-10-30, C16)* The robot file's `driveTuning` uses DEGREES too (§2): one angle unit at the human edge across
the panel's two files (F3), one conversion function in the tools, pinned against the C++.
**No end action** *(fix 2026-10-30, C16)*: the route file has no `endAction`; every generated entry leaves
`AutonEntry::endAction` null — the library's brake (`auton_registry.hpp` :267-268; robot two's 40 in auton does the
same, `autons.cpp` :83). A PARK (a second routine run at `endActionAt`) is OUT for Nov 21 — a later, additive field
(`"endAction": {"route": NAME}`) whose routine the tool would check against the time left before `hardStopAt`.

#### R.5 The step vocabulary → the library call each becomes
| Step (`do`) | Its fields | Generated C++ (*fix 2026-10-30, C14:* E3 compiled the rows marked ᴱ; the other "today" rows are compiled by the tests the Found (E3) paragraph names; S37 compiles every row at R1's tip) | Library | F8 export |
|---|---|---|---|---|
| (the route's start) | `start` | `entry.start = StartPose::field(x, y, θ)` / `relative()` | today | `"start"` |
| drive to | `x, y` | `r.driveTo(x, y, opts)` ᴱ | today | `"drive"` |
| back into | `x, y` | `r.reverseTo(x, y, opts)` ᴱ | today (L9) | `"drive"` + `reverse` |
| move to a pose | `x, y, headingDeg` | `r.moveTo(Pose2d{x, y, θ}, opts)` ᴱ | today | `"move"`, `"fixed"` |
| move, keep heading | `x, y` | `r.moveTo(Pose2d{x, y, <the pose estimate's heading, read when the step runs>}, opts)` (the mixed-tier idiom) | today | `"move"`, `"free"` |
| turn to | `headingDeg` | `r.turnTo(θ, opts)` ᴱ | today | `"turn"` |
| face a point | `x, y` | `r.face(x, y, opts)` ᴱ | today | `"turn"` at the bearing from the PLANNED pose + `shulib.face` |
| face away | `x, y` | `r.faceAway(x, y, opts)` ᴱ | today (L9) | as above + `faceAway` |
| strafe to | `x, y` | `r.strafeTo(x, y, opts)` — holonomic robots only | today | `"strafe"` |
| through points (stops at each) | `points[]` | `r.followTrajectory({…}, opts)` ᴱ — each point's heading is the bearing of its leg from the PLANNED pose (R.5's tank warnings) | today | consecutive `"move"`s |
| a smooth curve | — | — | **NO** (D16) | — |
| pause | `seconds` | ~~`r.pause(t)`~~ `r.then([&] { return ctx.guard.pause(t) == sequence::GuardedWaitResult::Satisfied; }, "pause")` — *fix 2026-10-30, C2: the GUARD's pause, which the deadline and a mid-auton disable can cut; `Routine::pause` is a frozen wait that runs to its own end (`auton_registry.hpp` :218-220; `run_guard.hpp` :94-108); compiled host and ARM, `fix13/`* | today | marker `wait_ms` |
| hold position | `seconds` | `r.hold(t, opts)` ᴱ | today | `"hold"` |
| brake | — | `r.brake(opts)` ᴱ | today | marker `brake` |
| wall reset | `wall, sensor, maxCorrectionIn` | `r.then([&]{ return competition::resetFromWall(ctx, {…}).continues(); }, "label")`; `required` from the step's on-failure | after L6-A2 merges; FIELD starts only | `shulib.wallReset` |
| home | `mechanism, boundS, wait` | `r.home(m, b)` / `r.startHome(m, b)` | today | `<name>_home [ms]` (blocking) / `shulib` |
| arm or lift to a preset / a value | `mechanism, preset \| value, boundS, wait` | `r.move(m["p"], b)` / `r.move(m.to(v), b)` / `startMove` | today (a value past a limit: no build) | `<name>_<preset> [ms]` / `shulib` |
| run a power mechanism | `mechanism, volts, seconds \| until (+ orAfterS), boundS, wait` | `r.run(m, V, t)` / `r.run(m, V, seesPiece[.orAfter(t)], b)` / `startRun` | today | `<name>_run`, `_until_piece`, `_until_gone` / `shulib` |
| a pneumatic position | `mechanism, position` | `r.set(m["p"])` ᴱ (blocking: its move time) | today | `<name>_<position>` |
| spin up a flywheel | `mechanism, preset, boundS, wait` | `r.spinUp(m["p"], b)` / `startSpinUp` | today | `<name>_<preset>` / `shulib` |
| stop a mechanism | `mechanism` | `r.stop(m)` | today | `<name>_stop` |
| wait until a started mechanism is done | `mechanism, boundS` | `r.waitUntilDone(m, b)` ᴱ — only where a start of `m` DEFINITELY ran first (R.5's refusals) | today | `shulib` |
| wait for a piece / for it to go | `mechanism, seen, timeoutS` | ~~`r.waitFor(…)`~~ `r.then([&] { return ctx.guard.waitFor([&] { … find(m) … sees() … }, t) == sequence::GuardedWaitResult::Satisfied; }, "label")` — *fix 2026-10-30, C2: the guard's wait, as pause* (E3 compiled the frozen form) | today | `shulib` |

`opts` is the step's care (R.6). The tool refuses: a mechanism or preset the robot file does not have, or a step of
the wrong kind for it; `until: seesPiece` on a mechanism with no piece sensor (the C++ refuses it at run time); strafe
on a tank; a wall reset on a relative start, or before L6-A2 has landed (`describe` lists it only then; once S1 lands
the robot file's wall sensors are checked, before that the page warns "the brain refuses the reset if that side has no
sensor"); `through` with no points; a pause or hold of 0.
*(fix 2026-10-30, C3)* **Every run-time precondition of a mechanism step is a refusal in the tool** — a generated
route must never THROW out of a match auton (a throw ends the auton; the guard safes on the unwind,
`match_runner.hpp` :55-57). `routine.hpp` :558-563 lists them; three more are refused: **a time condition longer than
its bound** (`orAfterS` > `boundS`); **`until: stalls` on a mechanism with no stall signature**; **homing a mechanism
declared `neverHomed`** — and every other precondition the service's commands state (the R1 run re-reads
`mechanism/service.hpp` and lists them all; S37 has one fixture each). And the `waitUntilDone` precondition ("this routine started no step on that mechanism",
`routine.hpp` :682-685; a start is recorded only when its step RUNS — `noteStarted`, :918-922, after
`skipIfStopped`): the tool refuses a `waitUntilDone(m)` unless a start form on `m` ran on EVERY path to it — a
forward pass over the steps in which a skip (`onFail: {skipTo}`) is a jump from its step to its label, so a start
between a skip and its label does not count for a wait at or after the label. A partway TRIGGER's start never counts
as definite; a wait after a trigger is allowed only because STEP1-B defines it (R.13: the wait then answers "the trigger
never fired" as a counted failure — never a throw, never an older step's verdict). STEP1-A also defines the run-time
side for hand-written chains (a wait whose start was jumped over is a counted failure), so the tool's refusal is the
first line and the library the second.
**Warnings on a tank** *(fix 2026-10-30, C5)* — generation is tank-only (R.3), and a tank `moveTo` is steered only for
a leg that STARTS within `tankSteer.maxSteer` (15° by default; Part T's typed value when stated) of its target heading;
otherwise it runs unsteered, logs "turn first: face()", and on a laterally offset point "times out beside the point"
(`chassis.hpp` :99-124; `routine.hpp` :303-311 and :321-333; robot two's `autons.cpp` :55-60). From the PLANNED poses
(each step's end pose from the one before; a field start's pose, or (0, 0, 0) for a relative one) the tool WARNS, on
the step, and offers to insert the turn: a `driveTo` whose planned start heading is more than `maxSteer` off the
bearing to its point (offer `face` first); a `reverseTo` more than `maxSteer` off the bearing + 180° (offer
`faceAway`); a `moveTo` to a pose — and "move, keep heading" — whose planned start heading is more than `maxSteer` off
its target heading, or whose planned start lies off the target's LINE (through the point, along its heading) by more
than tan(`maxSteer`) × the leg's length (an offset a 15° bend cannot remove — no size is promised, `chassis.hpp`
:117-122); and every bend in a `through` list of more than `maxSteer` (the next leg starts unsteered). A warning, not a
refusal: the live pose may differ from the plan. The page shows it on the step.

#### R.6 Care on every step — D14
| Care field | Applies to | Library | Note |
|---|---|---|---|
| speed cap | every drive step | `MotionOptions::maxLinearSpeed` — today | 0 = the robot's `maxLinearSpeed` (Part T) |
| turn cap | every drive and turn step | `MotionOptions::maxAngularSpeed` — today | |
| time limit | every step | `MotionOptions::timeout` / a mechanism step's `bound` — today | a mechanism step's is REQUIRED (A4: never a library default) |
| arrival tolerance (inches, degrees) | drive, turn, move steps | **NEW — STEP1-A:** two `MotionOptions` fields, 0 = the robot's settle config | F6's field set is additive-open (`chassis.hpp` :209-211) |
| on failure | every step | **NEW — STEP1-A:** a per-step choice; the frozen policy stays the default, word for word | D14 |

**D14 — What "if it fails" offers, and what "skip" means. RECOMMENDED (a).**
- **(a) RECOMMENDED: three choices.** **Stop the auton** (today's rule: park the drive, skip every later step).
  **Skip ahead to a named later step** (park the drive, skip the steps in between — counted and logged — and go on from
  that step; the panel draws the jump; a skip whose target never comes skips to the end, said). **Carry on** (log it,
  count it, go to the next step). Defaults: every drive step STOP; every mechanism step A4's policy (a counted
  sensorless grab continues, anything else STOP); a wall reset STOP = `required`, CARRY ON = not required, ~~(SKIP
  unstated)~~ *(fix 2026-10-30, C9)* SKIP = `required` (a reset that is REFUSED must count as a failure for the skip to
  happen; with `required` false `.continues()` reads true and the chain would go on, L6 brief :613-660). Carry on
  after a drive that did not arrive gets a warning: "the next steps start from wherever the robot stopped". The frozen
  error policy (F10) stays the default for every step that does not choose, unchanged — the choices are opt-in per step,
  F10's additive path (the register's F10 row gains one sentence).
  **A cut always stops** *(fix 2026-10-30, C2)*. The run guard ends scoring at `endActionAt` by CANCELLING the motion:
  the verb returns `Cancelled`, "a Routine records the stop and skips the rest" (`run_guard.hpp` :60-75), and Routine
  counts that exit as `MotionFailed` (`routine.hpp` :113-117, :208-210). The frozen waits cannot be cut; the guard's
  lateness bound assumes "its first post-deadline motion is refused and stops the chain" (`run_guard.hpp` :94-108).
  Under carry on or skip the chain would go on after the cut: each later frozen wait would run its full time past
  `endActionAt`, later mechanism steps would still be issued, and the end action (the brake) could slide past
  `hardStopAt`, where its own motions are refused. So: **a `Cancelled` verdict — a motion's or a mechanism step's —
  and any failure once the run's guard reads `expired()` (the deadline passed, or the run was abandoned) STOPS the
  chain, whatever the step chose.** The Routine learns the guard from the auton's context (`AutonContext::guard`; a
  Routine built from a bare `Chassis` has only the `Cancelled` rule). And the generator emits the GUARD's waits (R.5's
  pause and sensor-wait rows), which return at the deadline with zero latency.
  **What the result says under carry on and skip** *(fix 2026-10-30, C9)*. The frozen `RoutineResult` fields keep
  their meanings, read as "the chain's STOP": `ok` false once any step failed (its frozen meaning); `stoppedAt`,
  `stoppedName`, `cause`, `exit`, `mechanismVerdict` describe the step that STOPPED the chain, and stay 0 / "" /
  `None` / `Running` when the chain ran to its end; `skipped` counts only the steps skipped after a stop. NEW appended
  fields carry the rest: the steps that failed and the chain went on (carry on or skip), the first of them (index,
  name, cause), and the steps a skip jumped over. The two result screens read `ok` today and print "ROUTINE STOPPED"
  for any not-ok routine (`auton_tester_run.hpp` :248, :402; `competition_run.hpp` :550, :729); they gain a result
  word for a routine that finished with failures — for example "FINISHED, 2 FAILED", within `ui::kLargeCols` (18,
  `calibrate_panel.hpp` :82) — and a stop line only when `stoppedAt` > 0.
- **(b) Two choices** — stop or carry on ("skip it" = carry on). Cost: no way to give up on one scoring action and go
  for the next one.
- **(c) Choices per GROUP of steps** (a scoring action as one block). Cost: a group editor on the page and a nesting
  concept in the library; more to build, more to explain.

#### R.7 Mechanism timing — D15
- **Wait until done** — the blocking forms (today). **Start and keep driving** — `startMove`, `startHome`, `startRun`,
  `startSpinUp` (today); a pneumatic has no start form (E3). **Wait until a started mechanism is done** —
  `waitUntilDone` (today). The tool warns when a started mechanism is never waited for: it keeps its behaviour until
  the next command, the end of the auton or driver control (A4).
- **Wait on a time** — `pause`, or a power run for a time (today). **Wait on a sensor** — a power run until it sees
  (or stops seeing) a piece, or `waitFor` on a mechanism's piece sensor (today; a failing sensor reads UNKNOWN, which
  satisfies neither, so the wait times out into the step's on-failure choice).
- **Triggers partway through a drive** — **NEW — STEP1-B** (D15).

**D15 — A trigger partway through a drive: what if the robot never gets that far? RECOMMENDED (a).**
- **(a) RECOMMENDED.** A trigger is set on a drive or turn step: at a percent of the leg (the fraction of its
  straight-line distance — or its angle, for a turn — covered, from the pose estimate), or after so many inches or
  degrees. It STARTS a mechanism (start forms only — a blocking one would stop the drive) and fires at most once. **A
  leg that ends before the point — timed out, failed, skipped — never fires it** (MECH-PANEL §5 Q19 (i), the R2
  precedent: "a marker that fires at the wrong place is ... strictly worse than a marker that does not fire at all").
  The page marks the trigger "fires only if the robot gets there". This also answers Q19's first half: routes too, now
  — as generated code; the F8 export carries the trigger in its `shulib` object.
- **(b)** As (a), but a trigger the leg never reached fires when the leg ends (Q19 (ii)). Cost: a lift raised where
  the robot stopped short, not where the author meant.

*(fix 2026-10-30, C3)* **D15's second question — a leg that ARRIVES short of its mark. RECOMMENDED (i).** A leg ends
SETTLED once the robot is inside its arrival tolerance, not at the full length. So a mark at 100 %, an `afterIn` near
the leg's end, or a turn whose live angle (`face()` reads its bearing from the live pose) is smaller than planned can
leave the trigger unfired on a leg that SUCCEEDED; the chain then goes on, and a `waitUntilDone` on that mechanism
would throw (no start) or — worse — collect an OLDER step's verdict as done (`routine.hpp` :666-695: the wait answers
the step this routine last began on that mechanism, and "a step that already ended answers at once").
- **(i) RECOMMENDED: a leg that ends SETTLED fires its unfired trigger at the settle** — the robot did get there,
  within the tolerance the step accepts; a leg that ends any other way never fires it (D15 (a), unchanged). The tool
  also refuses a percent above 100 and an `afterIn` / `afterDeg` longer than the PLANNED leg (a typo guard).
- **(ii)** Never fire short: the tool refuses a mark it can predict is unreachable (a percent above
  100 × (1 − tolerance ÷ planned length), an `afterIn` past planned length − tolerance). Cost: the live pose can still
  miss a mark the plan reached; the step then succeeds with its mechanism never started.
- **Either way — the wait after a trigger that never fired** (a leg that timed out, failed or was skipped, or (ii)'s
  short arrival): STEP1-B arms the trigger's mechanism so that a later `waitUntilDone` on it answers "the trigger never
  fired: the robot did not reach the mark" as a COUNTED FAILURE under that wait's own on-failure choice — never a throw,
  never an older step's verdict (R.13, ST20/ST21).

#### R.7a Smooth curves — D16 *(fix 2026-10-30, C4: the first pass had no decision block for D16, though "follow a curve" was in the team lead's own list)*
**What exists (opened).** "Through points" is `followTrajectory`: sequential `moveTo` legs, "settling at each"
(`chassis.hpp` :324-355). Each leg must meet BOTH settle criteria — the robot within 0.5 in at ≤ 1 in/s for 0.1 s,
and its heading within 0.02 rad at ≤ 0.3 rad/s for 0.1 s (the library's provisional defaults, HA-51,
`motion_config.hpp` :172-178) — and the motors are stopped on the exit tick (`move_to_pose.hpp` :58-59). So the robot
STOPS at every point; options apply per leg (:325-327). On a tank each leg is steered only when it starts within
`maxSteer` (15°) of its target heading (`chassis.hpp` :99-124): a point that bends the path by more than 15° gives an
unsteered leg that can time out beside its point — R.5's tank warnings flag it. No curve verb exists: F8 names `"arc"`
a gap that "arrives additively with the roadmap's blending item" (`docs/roadmap.md` :142), "a measured Frontier" with
no size (:1989-1990); the master plan CUT "min-velocity motion chaining (stop-and-settle first)" from v1
(`docs/shulib-v2-master-plan.md` :515). And TANK1's design rule: steering toward the author's LINE is allowed, turning
toward the POINT is not (`tank_steer.hpp` :11-14, C1's D12) — a pursuit or boomerang controller aims at a point, so it
re-opens that rule.

**D16 — What "follow a curve" means by Nov 21. RECOMMENDED (a).**
- **(a) RECOMMENDED: no curve by Nov 21.** The route maker offers "through points" (it stops at each), with R.5's tank
  warnings on every bend over 15°. Size: 0. Cost: a route that should flow through a bend stops at each point — the
  deceleration into the point plus the 0.1 s settle hold, per point.
- **(b) Rolling through points** — "motion chaining", the master plan's deferred item: a leg that is not the last ends
  once the robot is within a stated distance of its point, at speed — no rate check, no settle hold, no heading
  criterion, the motors NOT zeroed — and the next leg starts from there. Library: a `MoveToPose` exit option and its
  `MotionOptions` field (F6-additive), a rolling form of `followTrajectory` (an F6 overload) and its Routine step; on a
  tank every bend must stay under `maxSteer` (15°) or the next leg is unsteered, so a "curve" is many points a few
  degrees apart. **~1½–2½ builder-days** (rough — sized by analogy to TANK1 Part A, sized 1¼, whose review found a
  loop-damping blocker, `tank_steer.hpp` :24-30), its own run after STEP1-A (both edit `MotionOptions`), plus robot
  tuning — how close "through" is and what speed it carries into the next leg — that no host test plant settles. Under
  D20 (a)'s order it would land after the routes, ~Nov 20–24: it misses Nov 21 unless something else gives way.
- **(c) A curve follower** — a pursuit or boomerang-style controller on a tank, the roadmap's blending item: a new
  motion class, a path form (points with headings → a smooth path), its end behaviour, the facade verb and the Routine
  step, F8's `"arc"` (additive), host-test-plant tests, and a ruling that re-opens C1's D12 ("never aim at the point").
  **~3½–5½ builder-days** (rough — no precedent in this repository: ~2–3 times TANK1 Part A for the motion class,
  ~1–1½ for the path form, the verb, the pins and the docs), plus robot tuning. It cannot be built and trusted on a
  robot by Nov 21.

#### R.8 Mirroring — D17
- **The two flips (E4).** On the field frame (+X right as seen from the RED station, +Y toward BLUE, walls at
  ±70.205 in — L6's `override_walls.hpp` :5-10 in `l6-a1`): **across the midline** `(x, y, θ) → (x, −y, −θ)`, or **a
  half turn** `(x, y, θ) → (−x, −y, θ + 180°)`. Both undo themselves. WHICH ONE Override's field needs is **not stated
  anywhere in this repository** (UNVERIFIED): the coordinator reads the manual's field drawing (Appendix A) before Part
  R1's go and puts it to the team lead to confirm; the tool implements both and the season states one.
- **What each changes besides the poses (E4).** Across the midline, the robot's left and right swap: a red route's
  LEFT sensor on the left wall becomes the blue route's RIGHT sensor on the same wall, and a RELATIVE-start route becomes
  `(x, −y, −θ)` (turns reversed). A half turn keeps the robot's sides: the LEFT sensor now faces the RIGHT wall, and a
  relative-start route is unchanged. The tool rewrites wall-reset steps accordingly, and under the midline flip it warns
  on every step that depends on a side of the robot ("this route uses the robot's left side; the mirrored route uses its
  right").
- *(fix 2026-10-30, C15)* **What the tool can and cannot know.** The robot file states no mechanism's SIDE (only the
  wall sensors carry one, L6's walls block), so a per-step warning can cover wall resets only. Under the midline flip
  the WHOLE route is a mirror image — a claw on the robot's left now meets the field on the other side — so the page
  shows a ROUTE-LEVEL warning on every midline-mirrored route: "the mirrored route is a mirror image: anything on one
  side of the robot is now on the other side of its path — check each mechanism step". **Relative starts:** under the
  midline flip a relative route's mirror DIFFERS (`(x, −y, −θ)`), so `drawnFor` is allowed on a relative route there;
  under the half turn a relative route mirrors to ITSELF, so `mirror` is REFUSED on a relative route (two identical
  entries would waste two of the registry's 16 places).
- **The lead for the manual read** *(fix 2026-10-30, C15)*: the manual's sheet A16 ("AprilTag numbering/locations")
  puts tags 1–4 on the field TWICE each, "mirrored" (`HANDOFF-remaining-work.md` :154-155) — where each pair sits
  settles the flip: a tag at (x, y) with its twin at (−x, −y) is a half turn; at (x, −y), the midline. The walls,
  Loaders, Load Zones and start pieces in `override_walls.hpp` are symmetric under BOTH flips, so they cannot settle it.
- **D17 — How a mirrored route reaches the robot. RECOMMENDED (a).**
  - **(a) RECOMMENDED: two picker entries per mirrored route** — "<name> RED" and "<name> BLUE" — generated as a pair,
    the flip the season's. No library change, no run-time colour. Cost: a pair takes two of the registry's 16 places,
    so names stay ≤ 24 characters (" BLUE" is five); the drivers pick the right one before the match.
  - **(b) One entry; the colour chosen on the brain at run time.** Needs ALLY1 (our alliance colour), which the sprint
    put after the panel. Cost: not by Nov 21; an ALLY1 change later can collapse each pair into one entry additively.

#### R.9 The timeline — D18
- **The bar** shows each route against its period and its entry's stop times (`endActionAt`, `hardStopAt`). **The
  periods: 30 s for a VEX U head-to-head autonomous and 60 s for Autonomous Coding Skills** (`<VUT4>`, the manual of
  2026-07-02 — memory `vexu-override-facts`, and `HANDOFF-remaining-work.md` :150-153, read from manual v1.0 on
  2026-09-24; robot two's autons file already says "the 30 s autonomous period"). The
  "15 s" in his words is V5RC's period, not VEX U's — D18 asks him to confirm.
- **D18 — What the timeline shows. RECOMMENDED (a).**
  - **(a) RECOMMENDED: two bounds, arithmetic only.** *(fix 2026-10-30, C8: the first pass called both "true" bounds;
    neither was, as written.)* The **FASTEST** time, an estimate FROM THE PLANNED POSES: each drive leg's
    max(0, length − its arrival tolerance − the previous step's) ÷ its speed cap (the robot's top speed where none is
    stated), each turn's max(0, angle − its tolerance) ÷ its turn cap, plus every stated pause, hold and timed run. Not
    a guarantee: a leg ends once it is inside its tolerance, it starts where the previous step ACTUALLY stopped, and
    `face`/`driveTo` take their bearings from the live pose (`routine.hpp` :86-95) — so it is labelled "about the
    fastest possible", never "true". The **SLOWEST ALLOWED** time: the sum over steps of each one's limit — a drive or
    turn's timeout (the robot's `defaultTimeout` where none is stated) TIMES ITS LEGS (`followTrajectory`'s options
    apply per leg, each leg its own watchdog, `chassis.hpp` :325-327), plus one control tick per motion (a motion can
    end up to a tick after its timeout); a mechanism step's bound plus the library's 0.25 s slack (`routine.hpp` :906)
    and a tick; a `waitUntilDone` the same; a guard pause its time; a guard sensor wait its timeout; a wall reset its
    own limit (read from L6-A2's API at R1's tip) — and the whole capped at `endActionAt`, where the guard cuts scoring
    (R.6's "a cut always stops"); the bar shows the end action from there to at most `hardStopAt`. Started mechanisms
    and partway triggers add nothing. ~~A route whose fastest time passes `hardStopAt`~~ A route whose FASTEST time
    passes `endActionAt` cannot finish its scoring (warned); one whose SLOWEST does may be cut (warned). No "likely"
    time is shown. The bounds are checked, not trusted: Part R1's test runs every fixture route on the host test plant
    and its time must fall between them (R.12, S42 — with fixtures built to break each bound) — the plant is the
    library's test oracle; the panel shows only the arithmetic. *How it stays arithmetic, not a second simulator:* it
    reads only numbers the student typed (or the robot file's defaults) and the planned geometry — no acceleration, no
    friction, no settle model, nothing VexBuilder would compute — and it is shown as two numbers with their words,
    never one predicted time; D18 asks him to confirm that line.
  - **(b)** (a) plus a "likely" time from an assumed acceleration and settle. Cost: that IS a small simulator, which
    he ruled out on 2026-10-08 — it would disagree with VexBuilder's and the robot's, with no way to say which is right.
  - **(c)** (a), plus, later, the times MEASURED on the robot (the Auton Tester's SD log, once it records each step's
    duration — a small `src/` change after the panel). Real numbers, not a model.

#### R.10 The F8 export (D13 (c))
`robot_routes.py export-paths ROUTES.json --route NAME` writes F8's `paths[]` entries: `"start"`, then one waypoint per
drive step (the mapping in R.5's last column); each blocking mechanism step a MARKER on the waypoint before it (route
ids, `kBuiltinCommands`' `wait_ms` / `brake`); everything F8 cannot say in ONE optional `shulib` object on the waypoint
it belongs to (start forms, `waitUntilDone`, sensor waits, partway triggers, tolerances, on-failure choices, wall
resets, the face target). A route that needs more than 32 waypoints or more than 4 markers on one waypoint is NOT
exported — refused, naming the step; never truncated. A mirrored route exports both sides. The coordinator's note to
VexBuilder (§4.10) gains the `shulib` object and the ask to keep it (Q20).

#### R.11 Where the generated code goes
- **One `[routes]` region** ~~at the end of the robot's autons file's namespace~~ *(fix 2026-10-30, C13: there it
  does not compile — `registerAutons()` calls a function declared after it; the checker's probe C3,
  `checker/routes_end.log`: "'registerPanelRoutes' was not declared in this scope")* **ABOVE `registerAutons()`, after
  the hand-written autons' anonymous namespace** (robot two's: between `autons.cpp` :72 and :74), holding its OWN
  anonymous namespace: one function per route and side (`route_<identifier>_<side>`, `chassis::Routine r{ctx,
  "<name>"}` …), then `void registerPanelRoutes(competition::AutonRegistry&)` registering each entry (name, kind,
  start, `endActionAt`, `hardStopAt`; `endAction` left null — R.4). Internal linkage, no declaration in `autons.hpp`,
  nothing unused (the gate fails `-Wunused-function`). Probe F13 (`fix13/`): a copy of `autons.cpp` with the region
  there — EMPTY, and with a generated red/blue pair using the guard's pause and a capped `driveTo` — compiles host
  `-Wall -Wextra -Wunused-function -Wconversion -Wsign-conversion -Wshadow -Werror` and ARM, exit 0
  (`fix13/routes_above.log`). ONE hand-written line
  calls it from `registerAutons()` — added once by Part R1 for robot two (`src/team/autons.cpp`, outside the region),
  by Part 4's starter for a new robot. Robot two's region is adopted EMPTY (`registerPanelRoutes` with no entries):
  its five packages do not change in behaviour (checked by the host suite and the gate). Hand-written autons stay
  outside the region, untouched.
- **The registry's 16:** the tool counts the hand-written `registry.add(` calls outside the region and refuses a route
  file that would pass 16; the host suite's registration test (every entry of every robot registers) is the truth.
- **Robot two's code freeze:** a route generated after the ~Nov 16 freeze changes robot two's competition build — the
  coordinator lands none into robot two's file between the freeze and 2026-11-23 without the team lead's word.

#### R.12 Tests (adversarial) and mutations — Parts R1 and R2 (numbered on from S36 and M35)
**Part R1 (self-test and host suite; each case FAILS first on a deliberately broken tool):**
- **S37 every step form** generates code that compiles on the host and ARM (E3's method, at the tip); one fixture per
  refusal in R.5 and R.6 (a mechanism, preset or position the robot lacks; `seesPiece` without a sensor; strafe on a
  tank; a wall reset on a relative start; a skip to an earlier or missing label; a duplicate label; a trigger on a
  non-drive step or with a blocking form; a name over 29, or over 24 when mirrored; a 17th entry). *(fix 2026-10-30:
  + the three run-time preconditions and every other the R1 run lists (C3) — `orAfterS` > `boundS`, `stalls` with no
  stall signature, homing a `neverHomed` mechanism; + the definitely-started check (C3) — a start inside a skip's range
  with the wait after its label REFUSED, the same start before the skipping step ACCEPTED, a wait whose only start is
  a trigger ACCEPTED (STEP1-B's rule); + the tank warnings (C5) — a `driveTo` 90° off its planned bearing, a `reverseTo`
  not preceded by `faceAway`, a `through` bend of 30°, a keep-heading `moveTo` to a point 10 in off its line over
  20 in each WARN, and the same steps after `face` / `faceAway` / with 10° bends do not; + the keep-heading `moveTo`
  and the guard's pause and sensor wait compile (C14, C2).)*
- **S38 the twin:** each fixture route's generated function and a hand-written `Routine` chain of the same steps drive
  two identically seeded host test plants BIT-IDENTICALLY, per tick (G2's `memcmp` method).
- **S39 the export's fidelity:** for every fixture route of F8-expressible steps only, `PathRunner` on the exported
  route (emitted as a `constexpr config::Route` fixture) and the generated function drive two seeded host test plants
  bit-identically; every exported route passes `Route::valid()`.
- **S40 the `shulib` object** carries every non-F8 field; a route needing a 33rd waypoint or a 5th marker is refused,
  naming the step.
- **S41 mirroring:** each flip undoes itself (byte for byte in the file); poses as E4; a wall reset's wall and sensor
  per flip; a relative route unchanged under the half turn and `(x, −y, −θ)` under the midline flip; both entries
  registered. *(fix 2026-10-30, C15: "undoes itself" alone passes for a flip that wrongly negates a signed field twice,
  so ALSO: after ONE flip every non-geometric field equals the original — volts, bounds, times, care, labels,
  mechanism, preset and position names, kinds, notes; `mirror` on a relative route under the half turn is REFUSED,
  `drawnFor` on one under the midline flip ACCEPTED; every midline-mirrored route carries the route-level warning.)*
- **S42 the timeline is honest:** on the host test plant every fixture route's run time lies between the fastest and
  the slowest bound — including a fixture robot whose `defaultTimeout` is not the library's. *(fix 2026-10-30, C8:
  the fixtures are built to BREAK each bound — a fixture robot whose `defaultTimeout` is 8 s, LONGER than the
  library's 5 s, with a step that times out on it (M39's mutant then reads 5 s and goes red; with a shorter default it
  would survive); a `through` step whose every leg is speed-capped so it takes most of its timeout (M56); a blocked
  leg that times out; a slow-capped, 3 in-tolerance short leg (the fastest estimate must still sit below it).)*
- **S43 care reaches the robot:** a stated speed cap, turn cap, time limit and tolerance reach the generated
  `MotionOptions`; on the host test plant a 2 in tolerance settles before a 0.5 in one.
- **S44 the on-failure choices:** with a forced failure (a blocked plant; a STALLED mechanism fixture) STOP, SKIP and
  CARRY ON produce exactly their step counts and log lines. *(fix 2026-10-30, C2/C9: + a generated CARRY ON route and
  a SKIP route, each cut at its `endActionAt` mid-motion and mid-pause, run NO further step and the end action starts
  within one tick of the cut; a wall reset under SKIP is emitted `required`.)*
- **S45 partway triggers:** the mechanism's first command lands in the tick where the leg's progress first reaches the
  mark; a leg that times out first never fires it. *(fix 2026-10-30, C3: + a leg that SETTLES short of a 100 % mark
  (D15's second question as he answers it); a `waitUntilDone` after a trigger that never fired is a counted failure —
  never an exception, never an earlier step's verdict on the same mechanism.)*
- **S46 freshness:** a hand edit inside `[routes]` → `check` names the lines; a stale region → STALE; robot two's
  region adopted empty, its differential logged.
- **S47 the field copy:** the tool's walls, Loaders, Load Zones and start pieces equal `override_walls.hpp`'s
  constants (an agreement fixture).
- **S53 nothing generated throws** *(fix 2026-10-30, C3)*: for every fixture route, the host test plant runs it once
  per step with THAT step forced to fail (each choice: stop, skip, carry on) — no run raises an exception; each ends
  with the counts its choices predict.
**Part R2 (`node --test`, pure model):** step add / move / delete / label keep the list valid; a refusal lands on its
JSON path; the mirror preview and the timeline come from the tool's `--json` (the page computes neither); a skip arrow
points at its label. The manual VS Code check: draw robot two a route of every step kind, mirror it, export it, and
read the generated code.
**Mutations (a FRESH run each; observe red; restore; log):**
- **M36** the midline flip forgets to negate the heading → S41 red;
- **M37** the midline flip keeps a wall reset's sensor side → S41 red;
- **M38** a skip target matched by prefix ("second" finds "second piece") → S37/S44 red;
- **M39** the slowest bound takes the library's 5 s default instead of the robot's `defaultTimeout` → S42 red;
- **M40** the export drops `reverse` → S39 red;
- **M41** a start form emitted as its blocking form → S38/S45 red;
- **M42** the position tolerance emitted on the heading field → S43 red;
- **M43** the validator accepts a preset the robot lacks → S37's fixture DISAGREES with the build's refusal;
- **M44** the generator writes past the region's end marker → S46 red;
- **M45** the field copy's Loader moved 1 in → S47 red;
- *(fix 2026-10-30)* **M53** the definitely-started check ignores skips → S37's skip fixture is accepted and S53's run
  THROWS → red; **M54** the turn-first check forgets `reverseTo`'s + 180° → S37's warning fixture red; **M55** the
  midline flip negates a power run's volts → S41's one-flip assertion red (it survives "undoes itself"); **M56** the
  slowest bound counts a `through` step's timeout once, not per leg → S42 red; **M57** the pause emitted as the frozen
  `r.pause(t)` → S44's mid-pause cut red (the pause runs past `endActionAt`); **M58** a `through` step's point headings
  taken from the step's own end pose instead of each leg's planned bearing → S37's 10°-bend fixture warns → red.

#### R.13 The library half — STEP1-A and STEP1-B (their own runs, `include/` work)
G3a's parts write no `include/` (§6: `git diff --stat -- include/` is empty for every part), so the route maker's
library half is its own chunk, **STEP1 — per-step care and timing in autons**, in two runs, each its own go. This
section is its brief's scope; the coordinator writes each run's prompt from it once D14 and D15 are answered.
- **STEP1-A — the arrival tolerance and the on-failure choice. ~~2–2¾~~ ~2¾–3½ builder-days** *(fix 2026-10-30:
  +¾ for the cut rule (C2), the result fields and screens (C9) and the jumped start (C3))*.
  - `chassis/chassis.hpp`: `MotionOptions` gains a position tolerance and a heading tolerance (typed; 0 = the robot's
    settle config); `effectiveConfig()` (:571-577) applies them to `translationSettle.maxError` /
    `headingSettle.maxError`. F6-additive (the field set is "additive-open").
  - `chassis/routine.hpp`: the per-step choice (D14) as opt-in, F10-additive members — for example a one-shot
    `r.onFail(…)` consumed by the next step and a `r.label("…")` step that ends a skip (the spelling is the run's,
    within these rules): the frozen policy is the default, bit for bit; carry on and skip are counted in APPENDED
    `RoutineResult` fields; `ok()` keeps its frozen meaning (false once any step has failed).
  - *(fix 2026-10-30, C2)* **A cut always stops:** a `Cancelled` verdict (a motion's exit or a mechanism step's) and
    any failure once the run's guard reads `expired()` stop the chain whatever the step chose (R.6). The context
    constructor also borrows the guard when the context has one (`requires { c.guard }`; `AutonContext::guard`,
    `auton_registry.hpp` :218-220), declared after A4's members like them; a Routine from a bare `Chassis` keeps the
    `Cancelled` rule only. (`routine.hpp` may include `sequence/run_guard.hpp` — that header includes `chassis.hpp`, not
    `routine.hpp`, so there is no cycle; or the template constructor keeps a thunk — the run's spelling.) The frozen default is untouched (it already stops on both).
  - *(fix 2026-10-30, C3)* **A jumped start:** when a skip jumps over a start form, the Routine marks that mechanism
    so a later `waitUntilDone` on it answers a COUNTED failure ("its start was skipped") under the wait's own choice —
    never the precondition throw (`routine.hpp` :682-685, kept for a wait with no start at all), never an older verdict.
  - *(fix 2026-10-30, C9)* **What the result says** (R.6's definitions): the frozen fields read as the chain's STOP;
    APPENDED fields for the failures the chain went past, the first of them, and the steps a skip jumped. **The two
    result screens** — `src/programs/auton_tester_run.hpp` (:218-248 the result word, :395-410 the stop line) and
    `src/programs/competition_run.hpp` (:517-550, :720-735) — gain the word for a routine that finished with failures
    (within `ui::kLargeCols`, 18) and print the stop line only when `stoppedAt` > 0; their tests
    (`test/competition_run_test.cpp`, the Auton Tester's run test) gain the cases. Every new line fits the line budgets
    (`kTesterLineBytes`, `kCompetitionLineBytes` — NOTES 10-29 ~21:00 logged COMP-b's routine-stop line at 214 B worst
    case against 200, CUT with its flag; STEP1-A must not lengthen it).
  - Tests (each FAILS first): ST1 every existing routine and recipe test passes UNCHANGED (the frozen default);
    ST2 a tolerance of 0 is bit-identical to today on the host test plant; ST3 a 2 in tolerance settles earlier;
    ST4 skip-to skips exactly the steps between, each logged and counted, and resumes at the label; ST5 a skip whose
    label never comes skips to the end, said; ST6 carry on continues, `ok()` false, the failure recorded; ST7 a
    mechanism step's own verdict policy under each choice (A4's counted grab still continues); ST8 the F6 and F10 pins
    gain the new members. *(fix 2026-10-30)* ST15 a CARRY ON chain and a SKIP chain cut at `endActionAt` — mid-motion,
    mid-frozen-pause and mid-guard-wait — run no further step, and the end action starts within one tick of the cut
    (or, mid-frozen-pause, of that pause's end — the frozen bound); ST16 a mechanism step ending CANCELLED under CARRY ON
    stops the chain; ST17 every frozen and appended `RoutineResult` field under each choice, and both screens' result
    words and stop lines (a routine that carried on past two failures reads its new word, not "ROUTINE STOPPED"); ST18
    a `waitUntilDone` whose start a skip jumped answers a counted failure, no exception.
  - Mutations: SM1 the choice ignored (always stop) → ST4/ST6 red; SM2 a label matched by prefix → ST4 red; SM3 carry
    on leaves `ok()` true → ST6 red; SM4 the position tolerance applied to the heading → ST3 red. *(fix 2026-10-30)*
    SM9 CARRY ON ignores a `Cancelled` exit → ST15 red; SM10 `expired()` not consulted → ST15's guard-wait case red;
    SM11 the screens' word reads `ok` alone → ST17 red; SM12 a jumped start leaves the mechanism unmarked → ST18
    throws → red.
  - Docs: guide 09 (the recipe API — the choices and the tolerance); cookbook 02 (when a step fails — a "skip to the
    next piece" recipe); the changelog; `version.hpp` (API 2.6, additive); the Freeze Register's F6 and F10 rows (one
    sentence each). *(fix 2026-10-30, C9: + the Auton Tester's and Competition's result words wherever the worksheet
    and guide quote them.)*
  - *(fix 2026-10-30, C9)* **Files:** `include/shulib/chassis/{chassis,routine}.hpp`, `version.hpp`, the F6/F10 pins,
    their tests, AND `src/programs/{auton_tester_run,competition_run}.hpp` with their tests — shared with ALLY1, L6-B
    and AF1 (the shared-files table); whichever lands second re-reads.
- **STEP1-B — triggers partway through a drive. ~~1½–2~~ ~2¼–3 builder-days** *(fix 2026-10-30, C10/C3: +¾–1 for
  the public seam and the unfired trigger; MECH-PANEL's own size for this item is "~1–2 (+~1 if routes are in scope,
  §5 Q19)", `MECH-PANEL-requirements.md` :764)*. After STEP1-A's build ends; it re-reads STEP1-A's tip (both edit
  `chassis.hpp`).
  - A pure progress function (the fraction of a leg's straight-line distance — or angle — covered, from the pose
    estimate, kept as a running maximum so a wobble cannot fire twice), and a Routine step form that arms a START form
    on the next drive or turn step (D15). The tier rule holds (`routine.hpp` :86-104: a recipe is never the only way):
    ~~the Tier-3 spelling — the scheduler's `async` plus `waitUntil` (`chassis.hpp` :516-520) with the progress check —
    is written first, in the cookbook, and the Routine form delegates to it.~~ *(fix 2026-10-30, C10: that spelling
    needs pieces a Tier-3 caller cannot reach — `effectiveConfig` (`chassis.hpp` :571-580) and `runBlocking` /
    `DetachGuard` (:541-566) are private, and `bearingTo` is private to Routine (PathRunner duplicates it,
    `path_runner.hpp` :24-28) — and a Routine step must be "exactly one Chassis call" (`routine.hpp` :86-88), so a
    Routine form built from those pieces would itself be motion logic.)* **So STEP1-B adds an F6-additive PUBLIC seam
    in `chassis/chassis.hpp`:** an overload of each blocking drive and turn verb (`moveTo`, `turnTo`; `driveTo`,
    `reverseTo`, `face`, `faceAway` reach them through Routine's one-argument delegation) that takes a small per-tick
    observer — a plain function pointer and a non-owning context, no heap (`runner::Handler`'s pattern) — called once
    per tick of the leg with the leg's progress, never after its exit tick. That overload IS the Tier-3 spelling the
    cookbook shows; the Routine step delegates to it, one Chassis call. The F6 pin gains it.
  - *(fix 2026-10-30, C3)* **The trigger that never fires:** arming a trigger marks its mechanism; if the leg ends
    without firing it, a later `waitUntilDone` on that mechanism answers a COUNTED failure, "the trigger never fired:
    the robot did not reach the mark" (an APPENDED `RoutineStopCause`), under the wait's own choice — never the
    precondition throw, never the verdict of a step the routine began on that mechanism earlier. D15's second question
    as he answers it: under (i) a leg that ends SETTLED fires its unfired trigger at the settle.
  - *(fix 2026-10-30, C10)* **It takes MECH C's item** "mid-drive triggers for routines ~1–2" (`MECH-PANEL-
    requirements.md` :764): MECH's rest loses it (the coordinator strikes it there at STEP1's plan commit).
  - Tests: ST9 fires once, in the tick where progress first reaches the mark; ST10 never before; ST11 a leg that times
    out, fails or is skipped first never fires; ST12 by inches and by degrees; ST13 the started mechanism runs while the
    drive continues (the service is ticked — A4's honest limit); ST14 a blocking form is refused at build (MECH-A1's
    compile-refusal harness). *(fix 2026-10-30)* ST19 the seam's observer is called once per tick of the leg and never
    after its exit tick, and the overload without an observer is bit-identical to today's verb (the F6 pin gains it);
    ST20 a `waitUntilDone` after a trigger that never fired is a counted failure, no exception; ST21 the same on a
    mechanism the routine started EARLIER does not answer that earlier step's verdict; ST22 a leg that settles short of
    its mark — D15's second question as answered.
  - Mutations: SM5 `>` for `>=` at the mark → ST9 red; SM6 fires on a timed-out leg → ST11 red; SM7 fires twice after
    a wobble → ST9 red; SM8 a blocking form accepted → ST14 red. *(fix 2026-10-30)* SM13 the wait after an unfired
    trigger answers the earlier step's verdict → ST21 red; SM14 the observer called once more on the exit tick → ST19
    red; SM15 the settle-short rule inverted → ST22 red.
  - Docs: guide 09; cookbook 03 (timing and partners) or 07 (mechanisms); the changelog; the Freeze Register's F10 row.
- **Who they collide with.** `chassis/routine.hpp`: MECH-A4's block (landed), MECH's rest (later), L6-C (not taken);
  `chassis/chassis.hpp`: nothing in flight (L6-A2 edits `motion/motion_scheduler.hpp` and `chassis/robot_context.hpp`
  — not these) — STEP1-A then STEP1-B, and D16 (b)'s run if he takes it, each after the one before; *(fix 2026-10-30,
  C9)* `src/programs/{auton_tester_run,competition_run}.hpp`: ALLY1, L6-B, AF1 — whichever lands second re-reads; `mechanism/service.hpp`: NOT edited (the sensor wait uses `find()` / `sees()`, already public) — so the
  string chunks (current cap, slack detection, in the service and the kinds) do not collide; `version.hpp`: STEP1-A
  takes 2.6; a string chunk landing in the same window shares or follows it (the coordinator's, at merge). No
  hardware belief is expected; if one appears, the executor stops and asks (HA-216 is the next free).

#### R.14 Part R's parts and sizes
- **R1 — the route tool and the code** (the route lane): `tools/robot_routes.py`, the route file, the footprint, the
  validator, the generator and robot two's `[routes]` region, the mirror, the timeline, the F8 export, R.12's S37–S47
  and S53 (*fix 2026-10-30: + the definitely-started check, the run-time refusals, the tank warnings, the guard's
  waits, the mirror's route-level warning and relative rules, the per-leg slowest bound — C2, C3, C5, C8, C15*),
  docs (a new guide chapter, "Routes from the robot panel", at the next free number at its tip; the cookbook's autons
  recipe; `vexbuilder-integration.md` §4's export and the `shulib` object; the robot-file chapter's footprint).
  ~~**~4–5½.**~~ **~4½–6½** *(fix 2026-10-30: +½–1)*. Starts when Part 5's BUILD ends (it needs the mechanism vocabulary in the robot file); its care and
  timing emitters need STEP1-A and STEP1-B merged by its tip — if STEP1-B is late, R1 lands without triggers and a
  small follow-up adds them (D20).
- **R2 — the ROUTES page** (the route lane): R.2's page, its model tests, the manual check. **~4–5½.** Starts when
  Part 3 has merged, BESIDE R1, against R.4's contract; whichever lands second re-reads the other at its tip.
- **Part R in all: ~~8–11~~ ~8½–12** (was ~4–6, RC10 — that was the 10-08 waypoints-and-markers page); **with
  STEP1 ~~11½–15¾~~ ~13½–18½** *(fix 2026-10-30)*.

### 5. Distribution — ruling (5)

**What exists (opened).** §1.7's package (`@vscode/vsce` 3.9.2, the real repository URL, `npx vsce package`, "no
marketplace" — now false) and §1.8's CI job (packages, publishes nothing). The repository: `shulib-v2` (development,
pushed by the team lead) and `release/v2` / `main` (public snapshots with `docs/internal/` dropped — `docs/internal/
RELEASING.md`; `tools/release.py`: `stage` never pushes, `push` never builds). The v2 probe packaged under a
placeholder publisher, `shulib-local` (P9).

**The design (Part 3, +½–¾).**
- **The package, release-ready:** `package.json` — `name`, `displayName`, `description`, `version` (0.x until it has
  run on a real robot), `publisher` (D19), `license: "MIT"` with the repository's `LICENSE` copied in, `repository`,
  `bugs`, `homepage`, `engines`, `categories`, `keywords`; a `CHANGELOG.md`; a `README.md` written for someone OUTSIDE
  the club — what the panel is; what it needs (VS Code 1.90 or later, Python 3, and a shulib project with
  `tools/robot_tool.py` — today a clone of SHU-ROBOTICS/shulib, PKG1's starter later); how to install a `.vsix`
  (Extensions → "Install from VSIX…", or `code --install-extension`); what it never does (build, upload, simulate);
  its limits (tank robots; host- and build-gated; not yet run on a robot); where to report a problem.
- **A release job,** `.github/workflows/robot-panel-release.yml`: runs ONLY on a tag `robot-panel-v<version>` the team
  lead pushes (or a manual run he starts); refuses unless the tag equals `package.json`'s version and the CHANGELOG has
  that version's section; `npm ci`, `npm test`, `npx vsce package`; checks `npx vsce ls` against an allow-list (nothing
  else ships); then `gh release create` with the `.vsix` attached and the CHANGELOG section as its notes
  (`permissions: contents: write`, the workflow's own token — no stored secret). Nothing is published to a store.
- **Docs:** `docs/internal/RELEASING.md` gains "Releasing the robot panel" (the tag, the version rule, his word every
  time); the robot-file chapter says where the `.vsix` comes from.
- **After Nov 21 (OUT of G3a's Nov 21 scope):** the Marketplace and Open VSX — a small later chunk (`vsce publish` /
  `ovsx publish` from the same job, the tokens as repository secrets, the club's accounts, which he creates).

**D19 — The panel's public name, and where its releases are cut. RECOMMENDED (a).**
- **(a) RECOMMENDED.** The publisher name is chosen NOW (for example `shu-robotics` — his to pick, and the name he
  will register at the stores) and used from the first GitHub release; the extension's full id is `<publisher>.<name>`.
  Release tags go on `release/v2` commits, so what the public `main` shows matches the released panel. *(fix
  2026-10-30, C16: (a) works only if the name is free at BOTH stores — the VS Code Marketplace's publishers and Open
  VSX's namespaces are separate registries; if either has it taken, (a) collapses into (b) at the store. So the team lead
  checks the name at both — or registers it at both — BEFORE the first release tag; the release job's README and
  `package.json` carry it from then.)*
- **(b)** A placeholder publisher now, renamed at the store. Cost: the store copy is then a DIFFERENT extension to VS
  Code — everyone who installed a GitHub `.vsix` must uninstall it and install again.
- **(c)** (a), but tags on `shulib-v2` commits. Cost: a released panel can be ahead of the public `main` and its docs.

### 6. String mechanisms — where they plug in (context; their own brief designs them)
- **The current cap and the snapped-string (slack) signature** are new members on a motor kind (the Lift at least);
  **the optional output rotation sensor** is a new named sensor slot in `wire(...)`. If they land BEFORE Part 5's tip,
  Part 5 adopts them as landed (+¼–½ per member set). AFTER (likely): the rebase rule's point 6 (i) — each chunk
  extends `tools/robot_tool.py` itself (the emitter in the kind's declaration order, its one-sentence refusals,
  `describe`'s vocabulary, the WIRING slot, the agreement fixtures — rule 4). **This amendment has Part 6 render every
  kind's members and every wiring slot FROM `describe`** (+¼–½ in Part 6), so the MECHANISMS and WIRING pages show
  such vocabulary with no page code. The bench step's thresholds (string on vs unhooked) enter the robot file like any
  stated fact, with provenance.
- **The live-tuning copy-back** comes through the TOOL — a subcommand (for example `robot_tool.py import-tuning FILE`)
  through the one writer, each value with a provenance ("tuned live on the brain, <date>") — and a button on the
  MECHANISMS page (and on Part T's DRIVE TUNING page if drive gains are live-tunable) that runs it. The live-tuning
  chunk carries both (rule 6 (i); the button +¼–½ in its own size), unless Part 6 or Part T is still unbuilt when it
  lands. **One question its brief must answer, because it decides G3a's side:** MECH-PANEL §5 Q24 recommended ONE
  tuning store on the SD card (R5a's pattern); copying live-tuned numbers INTO the robot file is Q24 (b). G3a works
  either way (an SD store is shown read-only beside, like R5a's).
- **The rebase rule's point 3 list** gains the string chunks; **point 6** applies to them.

### Tests (adversarial) and mutations for the feel, the tuning, the release and the pages (numbered on from R.12)
Each case FAILS first on a deliberately broken tool or page; each mutation is run fresh, seen red, restored and logged.
- **S48 (Part 5) the feel.** Robot two's `feel` region regenerates byte for byte (its differential logged); every
  `driving` field round-trips through `fmt`; a dead zone of 1.0 or a curve of 1.5 is refused by the validator AND by
  the build (`static_assert(Feel.valid())`, a compile-refusal fixture — the two agree); a negative ramp is refused by
  the tool alone (its own rule, named as such); a re-zero a control takes is refused by both (CK9's fixture, now reading
  the file's `rezeroButton`). *(fix 2026-10-30, C11: + robot two's `control words` region — the whole stick-and-button
  map — regenerates byte for byte; a `driving` file with `tankSticks` and `onRelease: brake` emits stick and release
  lines that say so.)*
- **S49 (Part T) the tuning.** Each of the ~~four~~ THREE mirrored checks *(fix 2026-10-30, C12: `validate()` covers
  tankSteer)* agrees with the real C++ (run-time fixtures catching `PreconditionError` from `MotionConfig::validate()`
  — tankSteer cases included —, `SettledUtil`, `OdoStallCheck`'s three typed fields), and the pure `tuningProblem`
  agrees with them fixture for fixture; the tool's own settle rule (a stated `maxError` or `maxErrorRate` of 0)
  refuses, named as stricter; a stated field reaches `RobotConfig::tuning` through `decideBootSourceFrom` and
  `bootConfigFrom` (`test/boot_source_test.cpp`); robot two's effective tuning equals `chassis::ChassisConfig{}` field
  by field; a stated tuning the C++ refuses refuses the boot, naming the robot file's field; a generated robot with a
  stated tuning compiles on the host and ARM (E1's method). *(fix 2026-10-30, C1)* **A partly stated NESTED group keeps
  the library's other members, field by field:** fixtures stating only `translationSettle.maxError`, only
  `heading.kD`, only `feedforward.kV`, only `tankSteer.maxSteerDeg` and only `stallCheck.window` — each other member
  of that group equals `MotionConfig{}`'s (the checker's C1 values: `maxErrorRate` 1.0, `settleTime` 0.1, heading
  `kP` 4, `kS` 1). *(C16)* A degrees field reaches the C++ as its radians, agreeing with `math::Angle::degrees`.
- **S50 (Part 3) the release check** (a small script the job runs, with its own `node --test` cases): a tag that is not
  `package.json`'s version is refused; a version with no CHANGELOG section is refused; a file outside the allow-list in
  `vsce ls` is refused.
- **S51 (Part 6) the FEEL page.** The curve's points come from the tool's `curve` query (the page computes none),
  pinned against `teleop::shapeAxis` by agreement fixtures; the re-zero dropdown shows the tool's refusal on its JSON
  path.
- **S52 (Part 6) the pages render from `describe`.** A fixture `describe` with an invented member on a kind and an
  invented wiring slot renders on the MECHANISMS and WIRING pages with no page code (§6's plug-in point).
- **Mutations:** **M46** the validator accepts a curve of 1.5 → S48's fixture DISAGREES; **M47** the feel emitter writes
  `.tickPeriod` → ~~`DriverFile`'s `static_assert` turns S48 red~~ S48's byte-for-byte differential red *(fix
  2026-10-30, C14: `DriverFile` ACCEPTS `.tickPeriod == kBrainTick` — `src/common/driver_file.hpp` :84 — so the likely
  mutant, `.tickPeriod = 10_ms`, compiles; only the differential sees the extra line)*; **M48** ~~the tuning emitter
  writes `heading` before `translation` → the generated table does not compile (S49)~~ *(re-cut 2026-10-30, C1: with
  assignments the order no longer decides compilation)* the tuning emitter writes a nested group as a designated
  initializer of its stated members (`c.motion.translationSettle = {.maxError = 0.75};`) → S49's partly-stated case
  red (`maxErrorRate` reads 0, not 1.0); **M49** the boot copies `ChassisConfig{}` instead of the table's tuning → S49
  red; **M59** the degrees conversion uses π/360 → S49's degrees fixture red; **M60** the tool's settle rule dropped →
  S49's zero-rate fixture is accepted → red; **M50** the release check compares the tag with `name` instead of `version` → S50 red;
  **M51** the curve query applies the curve before the dead zone → S51's agreement red; **M52** a page hard-codes the
  Lift's members → S52 red.

### The parts, re-planned for Nov 21 (replaces the re-check's parts table; the old numbers are kept)

| Part | Lane | What (changes in bold) | Starts after | Size (builder-days) | vs 10-29 (morning) |
|---|---|---|---|---|---|
| **1** | tool | as the re-check | BUILT 2026-10-29 (sprint, D6′ (b)); in review | 2½–3½ | = |
| **5** | tool | as the re-check, **+ D7's flip as its first logged step (sprint, D6′ (b), CK5), + THE FEEL's tool half (D11 (a)), + the `control words` region over the whole stick-and-button map (C11)** | Part 1's build; MECH-A5 merged (DONE 2026-10-30, `81eb666`); **D11 answered** | **~~5¼–7¼~~ 5½–7½** | +1–1½ (flip ¼–½, feel ¾–1) |
| **T** (NEW) | tool + page | **DRIVE TUNING** (§2) | Part 5's build ends; Part 3 merged; beside Part 2 | **~~2¼–3~~ 2½–3¼** (C16's degrees) | new |
| **2** | tool | as the re-check, **+ a generated robot's `.tuning` (or Part T adds it, whichever lands second)** | Part 5 | **2¼–3¼** | +¼ |
| **4** | tool | as the re-check, **+ the starter's `feel` region, + the starter autons file's `[routes]` region and its call line (if R1 has landed; else R1 adds them for generated robots)** | Part 2 | **2¼–3¼** | +¼ |
| **3** | panel | as the re-check, **+ the release-ready package and the release job (§5)** | Part 1's build; **D19 answered** | **3¾–4½** | +½–¾ |
| **6** | panel | as the re-check, **+ the FEEL page; + every member and slot rendered from `describe`** | Parts 3 and 5 | **4¼–5¾** | +¾–1¼ |
| **3b** | panel | the NEW ROBOT page | Part 6; Part 2 merged | ¼–½ | = |
| **R1** (NEW split of R) | route | the route tool and the code (R.14) | Part 5's build ends; D13–D18 answered; STEP1-A/B by its tip | **~~4–5½~~ 4½–6½** (C2, C3, C5, C8, C15) | R was 4–6 |
| **R2** (NEW split of R) | route | the ROUTES page (R.14) | Part 3 merged; beside R1 | **4–5½** | |
| **S1** | panel | the walls half (§3) | L6-B merged; Part 3 | **1½–2** | S was 2½–3½ |
| **S2** | panel | the camera half (§3) | VIS1-B merged; S1 | **1½–2** | |
| **AF** | panel | unchanged | AF1 (after the panel) | ½–1 | = |
| **7** | both | unchanged | MECH's rest | 2–3½ | = |
| **Make and drive** (1, 5, T, 2, 4, 3, 6, 3b) | | | | **~~22¾–31~~ ~23¼–31½** | 1–6 + 3b was ~18–24¼ |
| **Routes** (R1, R2) | | | | **~~8–11~~ ~8½–12** (+ STEP1 ~~3½–4¾~~ ~5–6½, library) | was ~4–6 |
| **All before G3** (1–6, 3b, T, R1, R2, S1, S2, AF) | | | | **~~34¼–47~~ ~35¼–48½** (+ STEP1) | was ~24¾–34¼ |

**The library runs beside it (not G3a's parts; their own go):** STEP1-A ~~2–2¾~~ ~2¾–3½ and STEP1-B ~~1½–2~~ ~2¼–3
(R.13; *fix 2026-10-30*). D16 (b), if he takes it, would be a third (~1½–2½).

**The lanes (three G3a lanes and the library):**
- **Tool lane:** 1 → 5 → 2 → 4, with **T beside 2** (T's tool half touches `chassis_table.hpp`'s struct, a new region
  and `boot_source.*`; Part 2 the SET regions — different lines; both edit `tools/robot_tool.py`, so whichever lands
  second re-reads and re-extracts, rules 1–2).
- **Panel lane:** 3 → 6 → 3b; S1 when L6-B has merged; S2 when VIS1-B has merged.
- **Route lane (NEW):** R1 and R2 side by side from Part 5's build end (R2 needs Part 3 merged); `tools/robot_routes.py`
  is R1's alone, so the route lane shares only `robot_tool.py`'s few schema lines (the footprint), the CMake file and
  the docs with the other lanes.
- **Library:** STEP1-A → STEP1-B (STEP1-B's build when STEP1-A's build ends).
- The sprint's overlap holds: each lane's next BUILD starts when its previous BUILD ends; reviews and fixes port
  forward; nothing is skipped (three reviewers, fixes, a fresh verifier, the coordinator's checks, every part).

### The slot schedule under the five-job ceiling, and the honest dates — D20

> *(fix 2026-10-30, C6/C7 — this section RE-DERIVED.)* The first pass's table hid review rounds inside its cells: around
> Nov 8–10 it ran six jobs (Part 5's review, T's build, R1, Part 2, Part 6, MECH-A6). Its demand (100–115) undercounted
> the sizes it listed; its camera and walls dates did not follow from its own sizes; it stated L6's order with MECH-A6
> as a rule when L6's brief makes it a choice; it put Part 6 before T without saying that departs from D6′'s answered
> yield order; and it left robot one out. Below: re-derived by probe F6 (`fix6/slots2.py`, adapted from the checker's
> C2 — a list scheduler on FIVE slots in which every review round HOLDS a slot for 2 days like a build, merges ½ day
> after the review, D20 (a)'s order, a lane's next build when its previous BUILD ends, this fix round's sizes), for
> both of L6's orders and with or without robot one's plumbing (`fix6/slots2_*.log`, `fix6/occ_*.log`). The model
> never runs more than five jobs (it prints its peak: 5).

**The demand** *(fix 2026-10-30, C6: recounted)*. From Oct 30 to Nov 21 (22 days): the 2026-11-23 track — CAL1-A2b
(building, ~1 left), CAL1-B (~1⅞–2⅛), MECH-A6 (~2–3), L6-A1 (in review), L6-A2 (~1¾–2), L6-B (~1½–2), IMU1-B (~½):
~8⅝–10⅝ to build; G3a's make and drive — 5, 3, T, 2, 6, 4, 3b: ~20¾–28; the route maker — STEP1-A, STEP1-B, R1, R2:
~13½–18½; the camera and walls — VIS1-A2, VIS1-B, S1, S2: ~7–9; the string chunks — a brief first, then runs guessed
at ~1 build day each (three); robot one's plumbing, if D20's robot-one line takes it: ~1–1½. Builds ~53–69 slot-days;
review rounds ~24 × 2 plus the three in flight ~4 = ~52. **About 105–121 slot-days** (+3 with robot one's plumbing)
against **at most 110** (five slots × 22 days) — and on 2026-10-29 the usage limit dropped the ceiling to ONE job
twice. Not everything lands by Nov 21; the order decides what does.

**D20 — The order of the five slots until Nov 21, and what gives first. RECOMMENDED (a).**
- **(a) RECOMMENDED, highest first:** (1) **the 2026-11-23 track** until the ~Nov 16 freeze — CAL1-A2b/B, MECH-A6,
  L6-A2/B, IMU1-B (his standing rule: MECH-A6 and L6-B never yield before the freeze) — with **MECH-A6 launched
  FIRST** and L6-B after A6 merges *(fix 2026-10-30, C6: a CHOICE, not a rule — L6's brief allows "L6-B before A6
  launches; if A6 has launched, after A6 merges", `L6-wall-reset.md` :1934; NOTES 10-30 ~09:35 queues A6 first; F6: A6
  first finishes the track ~Nov 8–10, L6-B first ~Nov 16–17, against the freeze, for the same panel dates within a
  day)*; (2) **make and drive** — the tool and panel lanes, Part T beside Part 2, and **Part 6 before T when only one
  slot is free** (6 is on the critical path: Part 3b, the NEW ROBOT page, waits on Part 6's build; in F6, T merges at
  or before the day make and drive lands in all four runs — a day or more before in three). *(fix 2026-10-30, C6: that SUPERSEDES D6′'s answered yield order — "the PANEL lane
  yields first, then AF1, then the TOOL lane" — for Part 6 against Part T only; D20 (a) asks him to accept it.)* (3)
  **the route maker** — STEP1-A, STEP1-B, R1, R2; (4) **the string-mechanism safety chunks** (their brief first); (5)
  **the walls page** (S1, after L6-B); (6) **the camera** — VIS1-A2 → VIS1-B → S2. On a budget warning, jobs yield from
  the bottom. If the route maker runs late, **the partway triggers slip first** (STEP1-B and R1's trigger emitter);
  everything else of it lands.
- **(b)** The string chunks above the route maker. Cost: routes most likely miss Nov 21.
- **(c)** The camera and walls above the route maker. Cost: reverses his ruling that, if it is tight, the route maker
  comes first.
- **D20's robot-one line** *(fix 2026-10-30, C7)*. The sprint row records robot one as "Panel + hand-written backup"
  (build-order :2333), and the G3a entry "made in the panel if the tool lane lands ~Nov 16 and tests well on the robot,
  with the students' hand-written table as the backup" (:1838-1840). Under this amendment the tool lane — Part 2 (a new
  robot's bring-up builds) and Part 4 (its Drive, Auton Tester and Competition), all a new robot needs by command —
  lands ~Nov 13–18, and with the NEW ROBOT page ~Nov 14–18: at or after that ~Nov 16. So the backup cannot wait for the
  panel. **When the backup must start:** robot one's facts from the team (its cartridge, mechanisms and their ports,
  controls) by ~Nov 5; its build plumbing — a second hand-written robot (RC18: "its own chunk, the coordinator's",
  ~1–1½ builder-days, rough) — launched by ~Nov 6 on the 11-23 track, merged ~Nov 10; the students' tables, driver
  file and autons (theirs to write, G4) ready for a first bench session by ~Nov 14 — a week of robot time before Nov 23.
  **The latest a panel-made robot one is worth a bench session:** Part 4 merged by ~Nov 18, so a session on Nov 19–20
  (Calibrate's push, Drive, the Auton Tester) leaves the team Nov 21–22 to choose which build competes. A panel-made
  robot one must carry a DIFFERENT robot id from the hand-written one (one id cannot be both; RC18's conversion —
  delete the hand-written lines, then `new-robot` — is a separate call, after Nov 23).
  - **(a) RECOMMENDED:** start the backup now, as above; try a panel-made robot one on the bench only if Part 4 has
    merged by ~Nov 18; robot one competes on its hand-written build unless that trial is clean. Cost: the plumbing's
    slot moves make and drive by ~0–1 day at the low sizes and ~2–4 at the high (F6: ~Nov 14–18 → ~Nov 14–22).
  - **(b)** Panel first, the backup on a cut-off: if Part 4 has not merged by ~Nov 16, start the backup then. Cost:
    started ~Nov 16, the backup has about a week for the plumbing, the students' code and a bench session — late in
    exactly the case it exists for.

**The five jobs under (a) — F6's schedule** (MECH-A6 first, LOW sizes, no day lost; at the HIGH sizes every band from
Nov 4 slides 1–4 days — Part 5's build ends ~Nov 8, not ~Nov 6). Each cell lists exactly the jobs that hold the five
slots — a build or a review round; a review is a job.

| Days | The five jobs (builds and review rounds) | Waiting |
|---|---|---|
| Oct 30 | CAL1-A2b build · MECH-A6 build · Part 1 review · L6-A1 review · VIS1-A1 verify | the answers (D11, D14, D19) → Part 5, STEP1-A, Part 3 |
| Oct 31–Nov 1 | CAL1-A2b review · CAL1-B build · MECH-A6 build → review · Part 3 build · Part 5 build | L6-A2 (L6-A1 and VIS1-A1 merge ~Oct 31–Nov 1), STEP1-A |
| Nov 2–3 | CAL1-B review · L6-A2 build · MECH-A6 review → STEP1-A build · Part 3 build · Part 5 build | IMU1-B (A6 merges ~Nov 3) |
| Nov 4–5 | IMU1-B build → review · L6-A2 review → L6-B build · Part 5 build · STEP1-A build · (Part 3's build ends ~Nov 4; its review waits for a slot) | Part 3 review |
| Nov 6–7 | L6-B review · Part 3 review · Part 5 review · STEP1-A review · Part 2 build | Part 6, R1, T (Part 5's build ended ~Nov 6) |
| Nov 8–9 | Part 2 build → review · Part 6 build · R1 build · STEP1-B build · a string run → Part 4 build | T, R2 (Part 3 merges ~Nov 8) |
| Nov 10–11 | Part 2 review → Part 4 review · Part 4 build · Part 6 build · R1 build · T build (STEP1-B review from ~Nov 11) | R2, S1 |
| Nov 12–13 | Part 3b review · Part 4 review · Part 6 review · STEP1-B review → R1 review · T review → R2 build | S1, VIS1-A2 |
| Nov 14–16 | R1 review · R2 build · S1 build → review · string runs and their reviews · VIS1-A2 build | VIS1-B |
| Nov 17–18 | R2 review · S1 review · a string review · VIS1-A2 review · VIS1-B build | S2 |
| Nov 19–25 | VIS1-B review → S2 build → S2 review (one job; four slots free) | — |

**The honest dates** (each "if the builders run every day"; every lost day — a usage stop, a reboot, a network drop —
moves every date about one day). *(fix 2026-10-30, C6: re-derived by F6 — both of L6's orders; the high ends include
robot one's plumbing where marked.)*

| Milestone | Lands | Slack to Nov 21 | What it needs |
|---|---|---|---|
| **Make and drive a robot** — the NEW ROBOT page, mechanisms, wiring, controls, the FEEL, DRIVE TUNING (Parts 1, 5, T, 2, 4, 3, 6, 3b) | **~Nov 14–18** (~Nov 14–22 if robot one's plumbing takes a slot) | ~3–7 days (~−1–7 with the plumbing) | D11 and D19 answered by ~Oct 31 |
| **The tool lane alone** — a new robot by command, its five programs (Parts 2, 4) | **~Nov 13–18** | ~3–8 days | as above |
| **Routes** — the full route maker (R1, R2, STEP1-A/B) | **~Nov 18–22** | ~−1–3 days | D13–D18 answered by ~Nov 1; STEP1-A's slot by ~Nov 3 |
| **The walls page** (S1) | **~Nov 18–22** (~Nov 24 with robot one's plumbing at the high sizes) | ~−1–3 days | L6-B merged by ~Nov 8–10 |
| **The camera page** (S2) | **~Nov 25–Dec 2** | misses by ~4–11 days | the serial chain VIS1-A2 → VIS1-B → S2 after the routes and the walls (at the LOW sizes alone: VIS1-A2 from ~Nov 15, 2¼ + 1¾ + a 2-day review + ½ + 1½ + 2 + ½ ≈ Nov 25) |
| **The 2026-11-23 track** (CAL1-B, MECH-A6, L6-B, IMU1-B) | **~Nov 8–10** (A6 first) / ~Nov 16–17 (L6-B first) | before the ~Nov 16 freeze | |
| **The string chunks** | **~Nov 19–23** | unknown | their brief's sizes (guessed) |

**What misses Nov 21 if the builders lose days.** With NO day lost: make and drive and the tool lane land with days to
spare; routes and the walls page land at the edge (their high ends are Nov 22); the camera page does not, and a string
run may not. **Two days lost:** routes and the walls page are a coin toss — the partway triggers slip first; make and
drive still lands. **Four days lost:** routes land a few days after Nov 21 (the step list, care, timing other than
triggers, mirroring and the timeline first), the walls page misses; make and drive lands at the edge at the high
sizes. **Seven days lost:** make and drive misses. **Not in the model, each making these dates later, never earlier:**
the coordinator's merges are serial (about 27 between now and Nov 21, one build directory, each with its gates and
mutations — F6 lets two merge the same hour); a review round with a blocker takes longer than 2 days (MECH-A4's took
from 2026-10-22 to 2026-10-29); the string runs are guesses.

**Robot use before Nov 23.** Every one of these dates is host- and build-gated. A robot MADE in the panel and DRIVEN
needs a bench session after it lands; a route needs the Auton Tester on the robot. No G3a part changes what robot two's
competition builds DO (the differentials, robot two's empty `[chassis tuning]` and `[routes]` regions, the feel and
control-words regions byte for byte); the coordinator checks that at each merge after the ~Nov 16 freeze. Robot one:
D20's robot-one line.

### Body passages corrected in place (each marked *(amended 2026-10-29 evening)*)
- **The header:** a pointer to this section (first); the 2026-10-29 answers block's "nothing of G3a builds before
  MECH-A5 merges" (the sprint built Part 1 first, D6′ (b)).
- **D6′ (the re-check):** "two G3a lanes … under the three-job ceiling"; the size "to a robot MADE and DRIVEN … ~11–15½";
  the three-job slot table (→ D20's five-slot schedule).
- **The re-check's parts table:** its heading (→ "The parts, re-planned for Nov 21"); Part 5's "THE FEEL outside — D4"
  (→ D11); Part 3 (→ §5); Part R (→ R1/R2, D13); Part S (→ S1/S2).
- **The adoption table's Controls row** ("THE FEEL outside (D4, RC5)"); **RC5**; **RC6** (the re-zero is stated, not read).
- **The rebase rule's point 3 list** (+ the string chunks and R5a Part B).
- **D4** — its accepted text REVERSED (a blockquote under its heading), and the 09-29 note "**D4** holds".
- **§1.2** "what a student may still hand-edit" (THE FEEL); **§1.3** (three new regions: `feel`, `[chassis tuning]`,
  `[routes]`); **§1.6** the re-zero fixture; **§1.7** the read-only re-zero and "no marketplace"; **§1.8** the release
  job; **§1.11** the profile gains `driving`, `driveTuning` and `footprint`; **§1.13** the starter's FEEL; **"What
  new-robot creation writes"** item 7; **§1.14** the controls page's read-only re-zero; **§1.16** the routes export.
- **§2** Part 5's M34; **§4** "Moved IN", "Stays OUT: THE FEEL" and the OUT line's "THE FEEL (D4); autons"; **landmine
  28**; **§8** "would generate THE FEEL"; **the plain words'** "Also coming".
- *(fix 2026-10-30)* **Corrected again, marked *(fix 2026-10-30, Cn)*:** the header's dates (C6); the answers block's
  D6′ slot question (C6: D20 would supersede it); **D6′'s** "two G3a lanes" line (C6); **§1.2**'s hand-editable list
  (C11: the sticks and the re-zero line are generated); **§1.3**'s three regions (C1: the lambda form; C11: the
  `control words` span; C13: `[routes]` above `registerAutons()`); **§1.7**'s packaging (C16: the publisher name
  checked at both stores); **§1.11**'s profile (C16: degrees at the human edge; C12: no `rotationRadius`); **§8**'s
  found list (C12: `rotationRadius` not derived from the record).

**Shared files these parts add (beside the re-check's table, the rebase rule unchanged):**
| File | Who else edits it | Part | The rule |
|---|---|---|---|
| `src/team/robot_two_driver.hpp` (THE FEEL) | MECH-A5 (landed by then); AF1 (after the panel) | 5 (`feel` region) | as `[driver file two]` |
| `src/common/driver_file.hpp` | MECH-A5 (creates it) | 5 (one `static_assert`) | A5 first |
| `src/common/boot_source.{hpp,cpp}` and the seven test files that call the boot core (§2) | VIS1-B, L6-B (their overloads), SAFE-SAVE B2, R5a Part D | T (the tuning parameter, the shim's `kChassis.tuning`, the pure refusal) | whichever lands second re-reads; T writes no other boot line |
| `src/robots/chassis_table.hpp`'s struct and robot two's initializer | VIS1-B (`camera`), L6-B (`wallSensors`) | T (`tuning`, `[chassis tuning]`), S1/S2 | each member appended; each region ends before what follows |
| `src/programs/{competition,auton_tester,drive}_program.cpp` | ALLY1, L6-B, AF1 | ~~T (pass `kChassis.tuning`),~~ 4 *(fix 2026-10-30, C12: T does not touch the programs — they call `decideBootSource(kTag)`, and only the shim reads `kChassis`)* | whichever lands second re-reads |
| `src/programs/{auton_tester_run,competition_run}.hpp` and their tests | ALLY1, L6-B, AF1 | STEP1-A (not G3a — the result word and stop line, C9) | whichever lands second re-reads |
| `src/team/autons.cpp` | the team (hand-written autons) | R1 (`[routes]` and its one call line) | the panel writes only its region |
| `tools/robot_routes.py` (new) | — | R1 | R1's alone |
| `.github/workflows/robot-panel-release.yml` (new) | — | 3 | |
| `include/shulib/chassis/{chassis,routine}.hpp`, `version.hpp`, the F6/F10 pins | MECH's rest (later), the string chunks (version only), D16 (b)'s run if taken | STEP1-A/B (not G3a) | R.13; STEP1-B after STEP1-A (both in `chassis.hpp`, C10) |
| `src/team/robot_two_driver.hpp`'s header (the stick-and-button map) | AF1 (a control for the flipper) | 5 (`control words`, C11) | the panel writes only its regions |

### The probes — `.claude/coordinator/scratch/brief-G3a-amend-1029b/` (evidence, never project code)
| # | What | Result |
|---|---|---|
| **E1** | `t1/tuning_member.cpp`: an aggregate with an appended `chassis::ChassisConfig tuning{}`; an old initializer without it; a generated one stating six fields in declaration order (main's headers) | Host `g++ -std=c++20 -Wall -Wextra -Werror` and ARM `-fsyntax-only` (RESUMING §3's flags) pass; the old table keeps 60 in/s, the stated one 50, unstated fields the defaults; both `validate()` (`t1/t1.log`) |
| **E2** | `t2/`: a generated feel (every `DriverProfile` field) beside robot two's `kControls`, through `mech-a5`'s `DriverFile`; the re-zero on the claw's button A; a curve of 1.5 with and without a generator-side check | Compiles host and ARM; the re-zero clash fails `DriverFile`'s `static_assert` ("a mechanism's button is the loop's re-zero button"); **the curve of 1.5 compiles when nothing but `DriverFile` checks it** (`feel_bad_expo_nocheck.cpp`, exit 0) — hence D11's `static_assert(Feel.valid())` (`t2/t2.log`) |
| **E3** | `t3/route_gen.cpp`: a generated route of ~~every step form today's library has~~ fifteen step forms (*fix 2026-10-30, C14: the list in §4's Found (E3); the other forms are compiled by the tests it names*), with caps and timeouts, and a red/blue pair of `AutonEntry`s, against `mech-a5`'s tree; a missing position; a pneumatic start form | Host `-Werror` and ARM: exit 0; a missing position: `no_such_position_on_this_mechanism` at build; `startSet`: "has no member named 'startSet'" (`t3/e3.log`) |
| **E4** | `t4/mirror.py` (`python3 -I`): the midline flip and the half turn on the F1 frame; a wall reset's sensor side after each; a relative route under each | Both undo themselves; midline: the LEFT sensor on the left wall becomes the RIGHT sensor on the left wall, relative `(x, −y, −θ)`; half turn: the LEFT sensor faces the RIGHT wall, relative unchanged (`t4/e4.log`) |
| **C1–C3** (the checker's, 2026-10-30) | `checker/nested_default.cpp` (a partly stated nested group); `checker/slots.py` (a five-slot list schedule with reviews holding slots); `checker/routes_end/` (the `[routes]` region at the namespace's end) | C1: the unstated members of a nested group are ZERO, all pass `validate()` (`checker/nested_default.log`); C2: make and drive Nov 14–18, routes Nov 17–23, walls Nov 18–22, camera Nov 27–Dec 1 (`checker/slots_*.log`); C3: "'registerPanelRoutes' was not declared in this scope" (`checker/routes_end.log`) |
| **F1** (fix) | `fix1/lambda.cpp`: the `[chassis tuning]` region as a constexpr lambda assigning only the stated fields onto a default `ChassisConfig`, inside an `inline constexpr` table | Every unstated member — nested included — equals `MotionConfig{}`'s (`static_assert`s); host `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror` runs and `validate()` passes; ARM exit 0 (`fix1/lambda.log`) |
| **F6** (fix) | `fix6/slots2.py` + `fix6/occupancy.py` (`python3 -I`): C2's model with this round's sizes, both of L6's orders, with and without robot one's plumbing | Peak 5 jobs in every run; the dates in D20's table (`fix6/slots2_{A6first,L6Bfirst}{,_i1}.log`, `fix6/occ_A6first_{low,high}.log`) |
| **F13** (fix) | `fix13/team/autons_{empty,one}.cpp`: robot two's `autons.cpp` with the `[routes]` region ABOVE `registerAutons()` in its own anonymous namespace — empty, and a generated red/blue pair (the guard's pause, a capped `driveTo`) | Host `-Wall -Wextra -Wunused-function -Wconversion -Wsign-conversion -Wshadow -Werror` and ARM: exit 0, both (`fix13/routes_above.log`) |
| **F14** (fix) | E3's `t3/route_gen.cpp` re-compiled against main `81eb666` (MECH-A5 merged) | Host and ARM exit 0 (`fix14/e3_main.log`) |

### The decisions, as I would put them to the team lead (plain words)

**D11 — The driving feel.** I recommend: the feel (arcade or tank sticks, dead zones, curves, the ramp, the re-zero
button, coast or brake on release) moves into the robot file, and the panel writes it into the driver file. It goes
into the mechanism part that is about to start (Part 5), so that part is built once. The safety cut-outs (the
over-current cut) stay as they are, not on the page. Also: today nothing stops a bad curve number from building; we add
that check.
*Other choices:* do the feel as its own small part after Part 5; or also put the safety cut-outs on the page.

**D12 — Drive tuning.** I recommend: every tuning number can be typed in the panel, each with who set it and when. The
panel shows the library's default beside each one. Feedforward and the top speeds are typed "until Calibrate measures
them" — Calibrate's gains step is not built yet, so for now the typed numbers are what the robot uses. A typed number
the library would reject stops the boot with a message naming it, instead of every drive failing. Angles are typed in
degrees, as everywhere else in the panel. The robot's turning radius is NOT typed: it is half the track width, which
Calibrate measures (the panel shows it, read-only). *(Fixed after the check: typing just one number
of a group — say only the settle distance — used to zero the rest of that group without any warning; the generated
code now changes only the numbers you typed.)*
*Other choices:* feedforward and top speeds measured only (then nothing changes them before Nov 21); or a separate
tuning table (more plumbing, same result).

**D13 — What a route becomes.** I recommend: the panel turns each route into normal auton code (the same `Routine`
lines a student would write) inside a marked section of the autons file. That is what the robot runs. The panel also
exports the route in VexBuilder's format, with the extra details in one extra field, so VexBuilder can still take it.
*Other choices:* run routes as data (much more library work; misses Nov 21); or code only (VexBuilder cannot use panel
routes).

**D14 — "If this step fails".** I recommend three choices per step: stop the auton; skip ahead to a named later step;
or carry on. Today's rule (stop) stays the default. Two safety rules come with it: when the auton's time runs out
(the field stops scoring at your end time), the auton always stops, whatever a step chose — otherwise "carry on"
could keep it going into the time the robot needs to brake. And the brain's result screen gets a new word for "it
finished, but 2 steps failed", instead of saying ROUTINE STOPPED.
*Other choices:* just stop or carry on; or choices per group of steps.

**D15 — Triggers partway through a drive.** I recommend: "at 60 % of this drive, start the lift". If the drive ends
early — timed out or failed — the trigger never fires, because firing in the wrong place is worse than not firing.
*Other choice:* fire it at the end of the drive anyway.
*A second question:* a drive counts as "arrived" when it is close enough, so it can finish just short of a mark set at
100 %. I recommend: if the drive ARRIVED, fire the trigger then (the robot did get there). Either way, a later "wait
for the lift" after a trigger that never fired counts as a failed step, never a crash.
*Other choice:* never fire short; the panel refuses marks it can tell are too close to the end.

**D16 — Smooth curves.** I recommend: no smooth curves by Nov 21. The panel offers "drive through these points"; the
robot stops briefly at each one, and on our tank drive the panel warns about any bend sharper than 15 degrees. A real
curve follower is its own library job after the panel. It cannot be built and trusted on the robot by the 21st.
*Other choices:* "roll through the points" without stopping (a smaller library job, about 1½–2½ builder-days plus
tuning on the robot; it would land around Nov 20–24); or a true curve follower now (about 3½–5½ builder-days plus
tuning on the robot, and it changes a design rule — the library never aims the robot at a point).

**D17 — Red and blue.** I recommend: a mirrored route shows up as two autons on the brain, "... RED" and "... BLUE".
Before we build it, I check the game manual for how the field mirrors — flipped across the middle, or turned half way
around. The repo does not say yet; the AprilTag sheet in the manual (A16) should settle it. If it is flipped, the
robot's left and right swap: the panel then warns on the whole mirrored route that anything on one side of the robot
ends up on the other side, because the robot file does not say which side each mechanism is on.
*Other choice:* one auton that asks for the colour on the brain (needs the alliance-colour chunk, after the panel).

**D18 — The timeline.** I recommend: the panel shows about the fastest a route could take and the slowest it is
allowed to take — plain arithmetic from the distances on the field and the speeds and time limits you typed. Nothing
else goes into it: no acceleration, no friction, no physics. So it never becomes a second simulator; VexBuilder stays
the only one. No "likely" time. One check: VEX U's autonomous is 30 seconds, not 15 (15 is the high-school game).
Skills is 60. Is 30 right for the bar?
*Other choices:* add a "likely" time (a simulator, which you ruled out); later, show times measured on the robot.

**D19 — The public name.** I recommend: pick the club's publisher name now (for example `shu-robotics`) and use it from
the first GitHub release. Changing it later turns the store version into a different extension, and everyone would
have to reinstall. Before the first release, please check the name is free on BOTH stores (the VS Code Marketplace and
Open VSX) — or register it on both; if one store has it taken, we pick another name first. Releases go on the public
release branch. Each release still waits for your word.
*Other choices:* a placeholder name now; or releases from the development branch.

**D20 — What gets the five job slots, and what slips.** I recommend this order: the Nov 23 competition work first,
until the Nov 16 freeze (MECH-A6 first, then the wall reset's robot part); then "make and drive a robot"; then the
route maker; then the string-safety work; then the walls page; then the camera. If routes run late, the partway
triggers slip first. One change to your earlier rule: you said the panel's pages give way before the tool work when
we are short of slots. I ask to let ONE panel part (the mechanism pages, Part 6) go before ONE tool part (drive
tuning, Part T), because the new-robot page waits on Part 6.
*Honest dates, if the builders run every day (re-checked with every review counted as a job):* make and drive ~Nov
14–18; routes ~Nov 18–22; the walls page ~Nov 18–22; the camera page ~Nov 25–Dec 2 (it misses the 21st). Each lost
day moves every date a day. Two lost days make routes and the walls page a coin toss. Four lost days mean routes land a
few days after the 21st; make and drive lands at the edge.
*Robot one:* the panel's "make a robot" lands around Nov 13–18, which is too late to be robot one's only plan. I
recommend starting robot one's hand-written backup now: its facts (motor cartridge, mechanisms and their ports,
buttons) from the team by about Nov 5, a small build-setup job by about Nov 6, the students' code ready to test by
about Nov 14. If the panel lands by Nov 18, we can try a panel-made robot one on the bench on Nov 19–20, and the team
picks which one competes. This costs make and drive up to a few days at worst.
*Other choices:* string work before routes; or the camera before routes (against your ruling that routes come first
when it is tight). For robot one: wait for the panel, and start the backup only if it is not in by Nov 16 (then the
backup has about a week).

**Also asked (gos, not decisions):** the go for Part T, for the route lane (R1, R2) and for the library runs STEP1-A
and STEP1-B; leave to read the game manual's field drawing for D17; robot one's facts (D20); and the publisher name
checked at both stores (D19).

### The checker's findings on this amendment (C1–C16; 2026-10-30, each re-verified at main `81eb666`)
| # | Sev. | The finding | Re-verified | Fixed where |
|---|---|---|---|---|
| C1 | blocker | A partly stated nested tuning group zeroes its other members; "Found (E1)" false for nested groups | **Real.** The three structs' own defaults read (`settled_util.hpp` :30-32, `motion_config.hpp` :56-58, `feedforward.hpp` :37-40); the checker's `nested_default.log`; `validate()` does not descend (:209-248) | §2: the Found corrected; the emitter assigns onto a default config (F1, chosen over "write every member", which copies library numbers into team files); the tool's own settle rule; S49's partly-stated cases; M48 re-cut |
| C2 | major | Carry on / skip after the guard's cut (`Cancelled`) breaks F2's lateness bound and delays the park | **Real.** `run_guard.hpp` :60-75, :94-108; `routine.hpp` :113-117; `auton_registry.hpp` :218-220 | R.6 "a cut always stops"; R.5's guard waits (F13 compiles them); STEP1-A's rule, ST15/ST16, SM9/SM10; S44; M57 |
| C3 | major | `waitUntilDone` after a skipped start or an unfired trigger throws or returns a stale verdict; three run-time preconditions not refused | **Real.** `routine.hpp` :558-563, :666-695, :918-922, :1049-1061 (start recorded only when the step runs) | R.5's refusals and the definitely-started check; D15's second question; STEP1-A's jumped start (ST18, SM12); STEP1-B's unfired trigger (ST20/ST21, SM13); S37, S45, S53, M53 |
| C4 | major | D16 had no decision block | **Real** (grep: no "**D16 —**" block; "4–6" underived) | R.7a: what exists, three options with sourced sizes; the plain words |
| C5 | major | No tank warning for a leg that needs a turn first | **Real.** `chassis.hpp` :99-124; `routine.hpp` :303-311, :321-333; `autons.cpp` :55-60 | R.5's "Warnings on a tank"; S37's fixtures; M54, M58 |
| C6 | major | The slot table ran six jobs; the camera and walls dates did not follow; demand undercounted; L6/A6 order a choice; D20 departs from D6′ | **Real.** Re-derived (F6): demand ~105–121; camera ~Nov 25–Dec 2; walls ~Nov 18–22; routes' high end Nov 22 in BOTH orders at this round's sizes (the checker's Nov 23 was at the first pass's sizes with A6 waiting for L6-B's merge) | D20 re-derived: the five-jobs table, the dates, A6-first as a choice, the yield-order supersession put to him; the header; the answers block |
| C7 | major | Robot one missing | **Real** (no "robot one" in the amendment; build-order :1838-1840, :2333) | D20's robot-one line (when the backup starts; the latest useful panel date; options); the plain words |
| C8 | major | D18's bounds were not true bounds | **Real.** `chassis.hpp` :325-327 (per-leg watchdog); `routine.hpp` :86-95 (live bearings) | R.9 rewritten (an estimate from the planned poses; per-leg timeouts, a tick per motion, capped at `endActionAt`; how it stays arithmetic); S42's breaking fixtures; M56 |
| C9 | minor | Result screens and `RoutineResult` under carry on / skip; program files; SKIP's `required` | **Real.** `auton_tester_run.hpp` :248, :402; `competition_run.hpp` :550, :729; `routine.hpp` :231-237 | R.6's definitions; STEP1-A's files, collisions, ST17, SM11; D14 SKIP = `required` |
| C10 | minor | STEP1-B's Tier-3 spelling needs private Chassis pieces; MECH C's item not struck | **Real.** `chassis.hpp` :541-580 private; `path_runner.hpp` :24-28; `MECH-PANEL-requirements.md` :764 | STEP1-B: an F6-additive public observer seam in `chassis.hpp` after STEP1-A; ST19, SM14; MECH C's item taken (struck at STEP1's plan commit) |
| C11 | minor | The driver file's header still describes the feel by hand | **Real** (`robot_two_driver.hpp` :17-20, :29-30, :57-58 at main) | §1's "words that describe the feel": `control words` over the whole map, the feel's comment in `feel`; §1.2; S48; Part 5 +¼ |
| C12 | minor | Part T's boot plumbing described wrongly; stall `rotationRadius`; three checks not four | **Real, one detail corrected:** the default stall wheels are a VALID stand-in (3.25 in, 1:1 — `drive_geometry.hpp` :66-70), not unset — the substance stands (the real wheels come from `effectiveTuning()`, after) | §2's boot (the shim and core only; ~30 test calls; a pure refusal); three checks; `rotationRadius` out of the schema (measured geometry); the shared-files table; S49 |
| C13 | minor | `[routes]` at the namespace's end does not compile | **Real** (`checker/routes_end.log`) | R.11: above `registerAutons()` in its own anonymous namespace (F13 compiles it); §1.3 |
| C14 | minor | E3 overclaimed; M47's kill path wrong | **Real.** E3 has fifteen forms (`t3/route_gen.cpp`); `driver_file.hpp` :84 accepts `kBrainTick` | §4's Found (E3) and R.5's column; the probes table; F14 (E3 at main); M47 re-worded |
| C15 | minor | Mirroring: handedness, relative routes, a weak involution test, the A16 lead | **Real.** `HANDOFF-remaining-work.md` :154-155 | R.8's two new bullets; R.4's `drawnFor` / `mirror`; S41; M55; D17's plain words |
| C16 | minor | D19's store name at both stores; mixed angle units; no end action; a stale citation | **Real** (`auton_registry.hpp` :267-268; build-order :2333) | D19 (a); §2 and R.4 in degrees (F3); R.4's "No end action"; the citations; S49, M59 |

### What stays unverified
- **Which way Override's field mirrors** (D17) — not in the repository; the manual's field drawing is read before R1.
- **VEX U's period lengths** — from the manual of 2026-07-02 as recorded in memory; v2.1 (2026-10-08) could not be
  fetched (L6-A1's note in `override_walls.hpp`); D18 asks him.
- **Every size here** — rough: R1 and R2 are new kinds of work (a generator of auton code; a field-drawing page); STEP1's
  are sized against L9 (two steps), MECH-A4's step block and MECH-PANEL's own "~1–2 (+~1 with routes)" for the
  triggers; the string chunks are unsized until their brief. *(fix 2026-10-30)* D16's (b) and (c) have NO precedent in
  this repository — sized by analogy to TANK1 Part A; robot one's plumbing (~1–1½) is a guess.
- **The dates** — they assume no usage stop, a ~2-day review round per run and ~½-day merges; 2026-10-29 dropped to one
  job twice. *(fix 2026-10-30)* F6 counts every review round as a job and never runs more than five, but it does not
  model serial merges, review rounds that run long (MECH-A4's: a week), or the string runs' real sizes — each makes the
  dates later, never earlier.
- *(fix 2026-10-30)* **That the guard's waits tick the mechanism service** as `Routine::pause` does — the guard's waits
  are "a composite predicate over C2's own waitUntil" (`run_guard.hpp` :95-97), which runs the pacer chain the service
  sits in, so they should; S38's twin and STEP1-A's ST15 are the proof, not this reading.
- *(fix 2026-10-30)* **Whether the publisher name is free at both stores** (D19) — not checked; the team lead's.
- *(fix 2026-10-30)* **Robot one's facts** (cartridge, mechanisms, ports, controls) — the team's; D20's robot-one line
  needs them by ~Nov 5.
- **Whether VexBuilder keeps the `shulib` object** (Q20, unanswered) — until it does, a route VexBuilder rewrites may
  come back without its care and timing; G3's import then takes them from the route file, never defaults them.
- **Whether a robot made in the panel drives** — nothing in this amendment runs on a robot; the first robot session
  after "make and drive" lands is the check.
- **Whether the string chunks land before or after Part 5** — decides rule 6 (i) or adoption (§6).
- **The DRIVE TUNING page's measured column** — empty until R5a Part B and Part C, neither in the sprint.

---


## Re-check 2026-10-29 — amendments

> **READ THIS FIRST. Where it and anything below disagree, this section wins.** Written 2026-10-29 from a READ-ONLY
> pass over main `73044ca` and the in-flight worktrees `.claude/worktrees/mech-a4` (MECH-A4's final fix; its
> `MECH-PROGRESS.md` "## MECH-A4" entries), `cal1-a2a` (CAL1 Part A2a's fix round) and `vis1-a1` (VIS1-A1's build,
> reviews pending), against the plan as committed — `build-order.md`, `REMAINING-WORK.md`, `MECH-PANEL-requirements.md`
> and the MECH, CAL1, VIS1, L6 and IMU1 briefs as answered — and the team lead's ruling of 2026-10-29 (the header).
> Probes V1–V8 (V7–V8 the second pass's, in `checker2-1029/`): `.claude/coordinator/scratch/brief-G3a-recheck-1029/` (evidence, never project code; "host test
> plant" where the library's model is meant). Every body passage found FALSE is corrected in place and marked
> *(re-check 2026-10-29, RCn)*; the history (the 09-27 and 09-29 re-check tables, the N/P probe rows, the accepted
> D1–D6) is left as written. The robot two block in `src/robots/chassis_table.hpp` is byte-identical to `a62067b`'s
> (`git diff a62067b HEAD`: only its line numbers moved, +3), so Part 1's differential input is unchanged.
> **Second pass, the same day:** a checker's eleven findings on this section were each re-verified against the tree
> and the plan — all real — and fixed here and in the body, marked *(re-check 2026-10-29, CKn)*; probes V7 and V8 in
> `checker2-1029/`. The list is "### The checker's findings on this re-check (CK1–CK11)" below the RC table.

### The decisions, as they are put to the team lead NOW (each replaces its 2026-09-29 text below)

**D6′ — The parts, their order and their sizes. RECOMMENDED (a): ten parts in two lanes; the FOUNDATION (Parts 1
and 5) starts when MECH-A5 lands, as he ruled; the panel lane starts when Part 1 lands (when MECH-A6 lands, if the
three slots are full — CK3); Parts R, S and AF plug in as their prerequisites land; DOCS3 follows the panel.**
- **What changed (RC1, RC3, RC4, RC9, RC10).** By the time Part 1 starts, MECH-A5 has landed (and CAL1, which N9
  puts before A5): the mechanism vocabulary is LANDED, not proposed, and nothing the core rewrites is still moving
  except MECH-A6's bench page (`src/programs/bench_r3a.{hpp,cpp}` only). So the 09-29 reason for deferring the
  mechanism half to "after the core" is gone, and robot two's WHOLE file — drivetrain, mechanisms, controls — can be
  the foundation. CAL1 took every Calibrate hunk Part 2 had and the watch Part 5 had. Part R (routes) was ruled
  2026-10-08 and was missing. VIS1 and L6 put the camera's and the side sensors' facts in the robot file, so the panel
  gains a SENSORS page (Part S); AF1 will add a flipper page (Part AF).
- **(a) RECOMMENDED** — the re-planned parts table below.
  - **The tool lane, strictly in order:** Part 1 (robot two's drivetrain, the tool, the freshness gate) → Part 5
    (mechanisms and controls in the tool; robot two's mechanism and controls regions) → Part 2 (a new robot's
    bring-up build) → Part 4 (its match builds and starter files). The old numbers are KEPT so every body reference
    stays valid; only the order changed (5 now precedes 2, so a new robot has its mechanisms from creation — CAL1's
    R11).
  - **The panel lane:** Part 3 (the shell, DRIVETRAIN, CALIBRATION, Save) starts when Part 1 lands — or, when the
    three slots are full, when MECH-A6 lands (the slot table below; Part 3 has ~1¼–2¼ builder-days of slack against
    Part 5, so that wait costs the critical path nothing); Part 6 (the mechanism pages) when Parts 3 and 5 are in;
    then **Part 3b** (the NEW ROBOT page, ~¼–½) **in the panel lane, after Part 6 and once Part 2 has landed**; then
    Part R. *(re-check 2026-10-29, CK6: with two lanes Part 2 lands at ~9–12½ builder-days, after Part 3's tip
    (~5¾–9), so 3b is always needed; in the panel lane it ends at ~10¾–14½, inside the ~11–15½ path; "right after
    Part 2" in the TOOL lane would have made the path 1 + 5 + 2 + 3b + 4 ≈ ~11¼–16. With ONE lane, Part 2 is in
    before Part 3 starts and the page goes inside Part 3.)*
  - **Two G3a lanes at once only when a slot is free** under the three-job ceiling *(amended 2026-10-29 evening: the ceiling is FIVE jobs since the sprint, and there are three G3a lanes — tool, panel, route — plus the library runs STEP1-A/B; the order of the slots is D20's; fix 2026-10-30, C6: D20 (a) would also supersede this D6′'s answered yield order for Part 6 against Part T — put to him)*; otherwise ONE lane, in the order
    1 → 5 → 2 → 3 (NEW ROBOT inside) → 6 → 4 → R. Part S when VIS1-B and L6-B have landed (its camera half may go
    after VIS1-B alone); Part AF when AF1 has landed; Part 7 after MECH's rest. **DOCS3 after the panel's pre-G3
    parts** (his ruling: it documents everything, the panel included); no G3a part waits for DOCS3.
  - **Size:** Parts 1–6 **~17¾–23¾** builder-days — **+1 at both ends** against 09-29's ~16¾–22¾ *(re-check
    2026-10-29, CK4: the first pass said "+½"; 09-29's "+~½ if the core leaves the watch to Part 5" is CAL1's now,
    so against 09-29's figure WITH the watch, ~17¼–23¼, it is +½)*; with R, S and AF **~24¾–34¼** (R, S and AF
    rough until their sections; AF is the page only — CK1, rule 6; +½–1½ in Part 5 if AF1 lands before Part 5);
    Part 7 ~2–3½. **To a robot MADE and DRIVEN in the panel with its mechanisms (Parts 1–6 and 3b): ~11–15½
    builder-days with two lanes, ~17¾–23¾ with one** — verification on top either way. *(amended 2026-10-29 evening: with the feel, Part T and the flip, make-and-drive is ~22¾–31 builder-days of parts, landing ~Nov 14–18 by the evening amendment's schedule)*
  - **What two lanes cost in slots — the window after MECH-A5 (re-check 2026-10-29, CK3).** *(amended 2026-10-29 evening: SUPERSEDED by the five-slot schedule, D20; kept as history)* At A5's merge FOUR
    jobs want to start, and three of them edit `src/robots/chassis_table.hpp`: G3a Part 1 (~2½–3½), VIS1-B
    (~1¾–2¼) and L6-B (~1½–2) — VIS1's and L6's accepted plans put both "after MECH-A5 merges and before MECH-A6
    launches" (VIS1 :502, L6 :564; "whichever goes second re-reads the other"; L6-B is in this window if A5 launches
    before L6-A2 merges — likely: CAL1 A2b + B ~2⅝–3⅛ against L6-A1 + A2 ~4½–5½) — and MECH-A6 (~2–3, Bench Tests
    station 13, the mechanisms' bring-up for 2026-11-23), which by those plans launches after them. ALLY1's
    `src/` half (after A5; AF1 and L4 need our colour) and AF1 (~8¾–11½, rough) wait behind them. With three slots
    free, the order recommended:

    | From (builder-days after A5 merges) | Slot 1 | Slot 2 | Slot 3 | Waiting |
    |---|---|---|---|---|
    | 0 — A5 merges | **G3a Part 1** | VIS1-B | L6-B | MECH-A6 (by plan, behind VIS1-B and L6-B), ALLY1's `src/` half, AF1, Part 3 |
    | ~1¾–2¼ — VIS1-B and L6-B land | Part 1 | **MECH-A6** | ALLY1's `src/` half, then **AF1** | Part 3 |
    | ~2½–3½ — Part 1 lands | **Part 5** | MECH-A6 | AF1 | Part 3 (its slack against Part 5: ~1¼–2¼) |
    | ~3¾–5¼ — MECH-A6 lands | Part 5 → 2 → 4 | **Part 3** → 6 → 3b → R | AF1 | everything else (L4, L6-C, VIS1's rest, IMU1-A if still queued) |

    - **What it costs.** At 0, Part 1 takes the slot ALLY1's `src/` half or AF1 would have had, so AF1 starts
      ~1¾–2¼ later. From MECH-A6's landing to the tool lane's end (~7¼–10¼ builder-days; ~1½–2 weeks with
      verification), G3a holds TWO of the three slots and AF1 the third, so nothing else starts. Nothing on the 2026-11-23 track waits for G3a:
      MECH-A6, VIS1-B and L6-B keep their places (VIS1's accepted rule — "if a slot is short, VIS1 waits, never
      MECH" — is kept).
    - **ASKED — when his budget warning drops the ceiling to two jobs, which yields first?** **Recommended:** G3a's
      PANEL lane first (it has slack until Part 5 lands), then AF1, then G3a's tool lane; MECH-A6 and L6-B never
      yield before the ~Nov 16 code freeze. **At one job:** the 2026-11-23 track's job until the freeze; after it,
      G3a's tool lane (the foundation) — or AF1, if he wants the automatic Toggle flip before the panel. His call.
- **(b) (a), but Part 1 starts as soon as a slot is free, BEFORE MECH-A5 — beside CAL1 A2b/B and MECH-A5 — with
  robot two's D7 flip held until A5 merges, and then done as the FIRST logged step of Part 5 (+¼–½ there, on the
  critical path — CK5).** Part 1 shares no file A5 rewrites except `tools/src_build_gate.py`
  (A5 edits step 0's per-robot functions, CAL1-B the Calibrate entry names and a self-test plant; Part 1 adds the
  `[generated]` step — different functions), `test/CMakeLists.txt` (each adds its own tests) and the test files D7's
  re-pins touch (hence D7 held; the flip writes robot two's robot file and `[chassis]`, so it goes in the tool lane,
  never the panel lane). **Gain:** ~2–3¼ builder-days on the critical path (Part 1's ~2½–3½, less the flip's
  ~¼–½ that moves into Part 5 — CK5: the first pass said "about Part 1's own length"), more if Part 3 also runs
  inside the CAL1/A5 window (~6⅜–8⅛ builder-days: CAL1 A2b ~¾–1 + B ~1⅞–2⅛ + A5 ~3¾–5). **Cost:** one more job
  in that window (a slot is free there only if VIS1-A2, L6-A1/A2, IMU1-A and ALLY1's library half leave one);
  Part 1 rebases on CAL1-B's and A5's gate edits; and it departs from his "right after MECH-A5", so it is his call.
  Under (b) Part 1 could land before 2026-11-23; it gives the competition nothing (RC18).
- **(c) One lane only.** Cost: the panel makes and drives a robot ~6¾–8¼ builder-days later; simplest
  verification; G3a holds ONE slot, so AF1 and the rest run beside it without a slot question.
- **(d) Fewer, larger parts** (2+4, 3+6). Cost as at 09-29: each merged part's documents pass rule 7's ~5, and the
  panel learns less from the tool between parts.

**D7 — Robot two's typed table. KEEP RECOMMENDED (a): flip it to the camera end with `flip-front`, as Part 1's own
logged step right after the differential — the step now cites the record, not a statement about the front (RC14).**
- **Does it still matter now that Calibrate decides the front? Yes, less than at 09-29.** The front is the end that
  led Calibrate's push (2026-10-06); robot two's record (2026-09-25) faces the camera end. The TYPED table still faces
  the 15/16 end, and at the tip three things read it: Bench Tests' DRIVE station and MOTOR WATCH (they drive opposite to
  the drivers' programs, which boot from the record); the boot's CROSS-CHECK line, which tells the drivers on EVERY
  boot from a record that "the MEASURED record and the typed chassis table disagree about 10 port(s)"
  (`src/common/boot_source.hpp` :537-550; 2 after the flip — ports 14 and 19, which the record left out); and, once
  the geometry is typed, CAL1's typed check (D6 (a)), which unflipped would hold a camera-end push on FRONT FLIPPED and
  then say FIX YOUR TYPED DRIVE PORTS — CAL1 §5: "D7 should land before or with the geometry". The boot never drives
  from robot two's table (it is not a valid description: no track width, no ratio), so nothing on the field changes.
- **What changes in the step:** no front words (the robot file states none); the new provenance cites the record —
  "…; front = the camera end (record 2026-09-25); …" — and must still fit 206 characters (V5: a 197-character wording
  keeps all seven facts; a 208-character one would be cut). The re-pin grep finds **22** files at `73044ca` against 18
  at `ee2e399`, of which the 09-29 lists named 11 (V6): new are CAL1-A1's `front_check.hpp`, its test and its API
  page, `drive_program.hpp` and `median_nonfinite_test.cpp`; `teleop_program.cpp` is gone. It moves again with
  MECH-A5, CAL1-B, VIS1-B and L6-B: Part 1 re-greps at its tip and sorts every hit into §3's three classes, as before.
  ~¼–½ day inside Part 1.
- **(b) Keep the 15/16 end.** Cost: Bench Tests keeps driving opposite to the drivers; the CROSS-CHECK line keeps
  saying 10 on every boot; whoever types robot two's geometry must flip it first anyway (CAL1 §5).
- **(c) Re-measure the signs camera-end-first (MOTOR WATCH) before flipping.** Cost: a robot session; N1 and CAL1's
  P24 already predict the answer (8 of 8 SAME).
- **(d) Flip it NOW, by hand, as a small change of its own** (no tool yet; Part 1 then adopts the flipped bytes).
  Gain: Bench Tests and the CROSS-CHECK line agree with the drivers before 2026-11-23. Cost: its own go and
  verification now, in `src/robots/chassis_table.hpp` while VIS1-B and L6-B are queued for the same file; the flip
  is then not the tool's logged step.

**D8 — Which programs a NEW robot's build carries. KEEP RECOMMENDED (a): the five robot two has — Bench Tests and
Calibrate (bring-up, Part 2), then shulib Drive, the Auton Tester and the Competition program (match builds,
Part 4).**
- **Does the list match today's builds? Yes.** The gate builds seven (`tools/src_build_gate.py` :385-386): bench,
  xdrive, and robot two's five — `tank` (Bench Tests), `tank-drive`, `tank-calibrate`, `tank-autontest`,
  `tank-competition`; `PROGRAM=library` is retired (`RETIRED_BEACONS` :427). VIS1 (D3 (a): station 12, no new
  program) and L6 (station 11's WALL page) add none; ALLY1's and AF1's plans add none. A generated robot's Bench Tests
  gets MECH-A6's station 13, VIS1-B's camera page and L6-B's wall page for free: each reads the build's tables.
- **What changed:** Part 4 no longer waits "after the core" — the core's `src/` half (A5) has landed before Part 1,
  so Part 4 follows Part 2 in the tool lane. The programs still name robot two's driver and autons directly at the
  tip (`competition_program.cpp` :233, :531, :702, :791, :795; `drive_program.cpp` :165; `auton_tester_program.cpp`
  :492) and A5 rewrites those lines (`using RobotTwoDriver = DriverFile<…>`, MECH §1.13), so Part 4 re-reads them
  at its tip. Step 0's driver/auton rule now builds on A5's per-robot namespace rule (`_robot_namespaces`,
  `TEAM_FILE_ROBOT`) instead of beside it, and the starter driver file is written in A4/A5's vocabulary (RC11).
- **(b) Bring-up builds only** and **(c) bring-up plus the Competition program only**: costs as at 09-29.

**D9 — Where a generated robot's build plumbing is written. KEEP RECOMMENDED (a): SET regions, adopted before the
first generated robot exists.**
- **The src gate today (re-read at `73044ca`):** the rule that rules out a helper macro still holds — a block that
  reads a build switch is a guard only when every branch holds `#error` alone (`_parse_selection_header`, :687-735,
  the test at :720; the header :203-206); the gate's tables are still a hand-written second copy of the chain, "a
  build added to one place and not the others fails here" (:378-384, tables :385-449); `TABLE_FILES` (:494-495) still
  names the two table files and its regexes accept the library's `mechanism::MechanismTable` that A5's `using`
  brings in (MECH §1.13, P11). **Two things A5 adds that D9 needs no region for:** `_robot_namespaces` DERIVES each
  robot's namespace from the mechanism table file (a generated robot's namespace is covered the day it is generated),
  and the `TEAM_FILE_ROBOT` map is only for team files not named `src/team/<ns>_*` (a generated robot's are).
- **One rule added (CAL1 R13):** the tool refuses `calibrate` in a non-tank robot's `build.programs`, so the guards'
  general form keeps Calibrate on tanks (the facts' `drive` kind is a tank for every `ChassisTable` robot — CAL1-A2a's
  `CalibrateFacts::drive`).
- **One collision added (RC21):** a generated id whose Bench Tests beacon equals a RETIRED one — `tank_library` →
  `shulib-robot-variant=tank-library`, which step 6 refuses in every package with T4 Part 3's "RETIRED" message — is
  refused by name; S22 gains it.
- **And the namespace (CK8, V8):** from MECH-A5 on, a generated robot's id IS its C++ namespace under
  `shulib::bench` and its team-file prefix `src/team/<id>_*` — so a keyword, a namespace `src/` names unqualified
  (`std`, `hal`, `robot`, …) and an id that prefixes another robot's id or team files are refused by name (§1.13,
  S22). It needs no region: `_robot_namespaces` derives the namespace from the table file.
- **(b) per-robot marker insertion** and **(c) the gate derives its rows**: costs as at 09-29.

**D10 — A NEW robot's drive ports. KEEP RECOMMENDED (a): optional; a new robot's required facts are THREE — name,
drivetrain kind, motor cartridge (the front is never asked, 2026-10-06).**
- **Against CAL1's typed-port check (D6 (a)) — a correction (RC13).** The 09-29 text said typed ports are "the
  fallback". For a GENERATED robot they never are: the boot drives from a typed table only when it is a VALID
  description (`tableToDescription(t).valid()`, the test CAL1's `withTypedDriveFrom` also uses), which needs measured
  signs and the geometry — and the tool refuses both (`typedTable`) on every robot but robot two. So a generated
  robot's typed ports reach neither the boot nor CAL1's typed check; its first Calibrate gets the FIRST RECORD note
  whether or not ports are typed.
- **What typed ports still give:** build-time port checks — a mechanism role, the camera or a wall sensor on a drive
  port is refused at build (`mechanismsClearOf`, A5; L6's "port 4 is both a wall sensor and a drive motor in the typed
  table"); and the panel's WIRING page shows them taken.
- **Its costs, re-stated:** "the front cannot be checked" is GONE (no front is typed; the record carries the push's
  end, and CAL1 warns when a new record flips it). "A mechanism wired to a drive motor is not caught at build" is
  still true for a portless robot, but it is now caught at Calibrate: CAL1's WATCH names a declared mechanism port
  that turned with the push (CAL1 D4 (a), HA-187). (The other order — a record taken BEFORE the port was declared —
  is the boot's: its declared-port rule refuses a record that drives a declared mechanism port, and MECH-A5 extends
  it to sensor ports.)
- **(b) Required.** Cost: contradicts the ruling and MECH-PANEL §4.1 (three facts), and the typed ports would still
  reach nothing at boot.
- **(c) (a), plus the CALIBRATION page offers to copy the record's drive ports into the file, unsigned, with the
  record as provenance.** Gain: the build-time checks above, with nothing typed. Cost: +¼ in Part 3, and a second copy
  that goes stale when the robot is re-cabled (it reaches nothing at boot, so a stale copy only misleads the build's
  port checks until the next copy).

### The parts, re-planned (replaces the 2026-09-29 parts table; the old numbers are kept)

> *(amended 2026-10-29 evening)* **This table is itself REPLACED by "The parts, re-planned for Nov 21" in the evening amendment above:** Part 5 + the D7 flip and THE FEEL's tool half; NEW Part T (drive tuning); Part 3 + the release; Part 6 + the FEEL page and the `describe`-driven rendering; Part R split into R1 and R2 (~8–11) with the library runs STEP1-A/B; Part S split into S1 (walls) and S2 (camera). Kept as history.

| Part | Lane | What it builds | Starts after | Size (builder-days) | Docs | vs 2026-09-29 |
|---|---|---|---|---|---|---|
| **1 — robot two's drivetrain, the tool, the freshness gate** (FOUNDATION) | tool | as 09-29 (validate / generate / check / fmt / adopt / describe / flip-front / self-test, the marker protocol, the text-field, baseline, collision and provenance-length rules; the agreement test's chassis half; D5's chassis pin; the freshness gate in the host doc gate, the src gate's `[generated]` step and CI) **minus the front's words** (RC2); robot two's file without a `front` key; `[chassis]` adopted with the differential, the region ENDING before any appended sensor member (RC9, V3); D7's flip (if (a)) | **MECH-A5 lands** (D6′ (b): a free slot before it, D7 after A5) | **~2½–3½** (−¼ the front; +¼ the re-pin floor: 22 files at the tip, 11 named at 09-29) | ~6 | = |
| **5 — mechanisms and controls in the tool** (FOUNDATION) | tool | MECH §6.3's vocabulary AS LANDED (five kinds, named sensor slots, `gearing` tooth pairs, `range`, `gravity`, the five homing forms, a lift's `shape`, `presets`, `stall`, `unhomedVolts`, `tuning`, pneumatic `positions`; `wiring` as the `wire(...)` facts; the controls with A4-D27; per-robot namespace; lowerCamelCase identifiers with `id`); route ids mirrored, never stored; each role's `changeAt` (≤ 160 characters — RC8 as corrected by CK7, V7); templates as data; the mechanism edits with their effects; robot two's THREE regions — `mechanisms` (MECH's one block), `control words`, `controls` (the `kControls` statement only, THE FEEL outside — D4, RC5 *(amended 2026-10-29 evening: plus a `feel` region over THE FEEL — D4 reversed, D11)*) — with the differential against A5's tree; the agreement test's mechanism half (run-time: `specProblem`, `wiringProblem`, the route-id refusals; controls: a compile-refusal set — RC6 — with ONE `DriverFile` fixture, a control on the re-zero button matched by its `static_assert` text, CK9); robot two's flipper and any vocabulary AF1 has landed by Part 5's tip (rule 6) | Part 1; MECH-A5 (A4 merged) | **~4½–6** (09-29 Part 5 ~3½–5, its "+~½ for the watch" released to CAL1; −¼ no handles file and three regions not four, +½ the landed vocabulary's ~128 refusals, +½ the controls' compile-refusal agreement, +¼ route ids and `changeAt`) | ~5 | moved before Part 2 |
| **2 — a new robot's bring-up build** | tool | as 09-29 (the SET regions with their differential; `new-robot`; the guards' general form; the Makefile's hand-written robots in the generated shape; the gate's merge lines and its self-test robot; `kEveryRobotId`) **minus everything Calibrate-specific** (CAL1 §1.7: banner, refusal words, the two front lines, the front `static_assert`, `ChassisTable::front`); **plus** the generated robot's mechanism table and namespace from creation (Part 5 is in; S34 moves here), CAL1 R13, and the namespace as a derived name in the collision rule (C++ keywords, a shadowed namespace, an id that prefixes another's team files — CK8, V8) | Part 5 | **~2–3** (−½ Calibrate; +¼ mechanisms from creation) | ~4 | after 5, not 1 |
| **4 — its match builds and starter files** | tool | `DriverFor<RobotId>`, `autonsFor(RobotId)`; the STARTER driver file in A4/A5's vocabulary (`controls(k<Id>Mechanisms, …)`, `using <Camel>Driver = DriverFile<…>`) and the starter auton file; their SET regions; step 0's selection names on A5's namespace rule; the bound listings (`api_doc_tool.py`) — **six** whole-run listings at the tip (RC12) | Part 2 | **~2–3** (−¼ no span form, N4 moot; −¼ A5's namespace rule) | ~5 | no longer "after the core" |
| **3 — the panel shell** *(amended 2026-10-29 evening: + the release-ready package and the release job, +½–¾ — §5 of the evening amendment)* | panel | as 09-29 (extension, custom editor, tool runner, `describe`-driven vocabularies, diagnostics by JSON path, model tests, `.vsix`, CI job; Save) with **DRIVETRAIN** (no front words; `flip-front` offered only for typed sides the CALIBRATION page finds FLIPPED) and **CALIBRATION** (the record AND its backup, `.bin` and `.bak`, `record_store.hpp`'s fallback rule mirrored; the port map in `writePortMap()`'s form; `compareFronts` mirrored as ONE rule pinned against the C++; FIRST RECORD wording) — RC2, RC3; the **NEW ROBOT** page (three facts and the id) inside Part 3 only with ONE lane; with two, it is **Part 3b** (~¼–½, panel lane, after Part 6 and Part 2 — CK6) | Part 1, or MECH-A6's landing when slots are full (CK3) (NEW ROBOT: Part 2) | **~3¼–3¾** (−¼ the front; +½ the backup and the front check mirrored) | ~5 | after 1, not 2 |
| **6 — the mechanism pages** | panel | MECHANISMS (templates mapped to the five kinds, the stall with the guide's recipe beside it), WIRING (the named slots, never free roles; the honest limits), CONTROLS (hold, toggle, preset with its time limit, the two homing controls, autons-only; A4-D27's rule; A4-D26's words), each mechanism's route ids shown, the CALIBRATION page's mechanism warning | Parts 3 and 5 | **~3½–4½** (+½ the landed vocabulary's forms) | ~4 | = |
| **R — routes in the panel** (ruled 2026-10-08; NEW to this brief, RC10) *(amended 2026-10-29 evening: SUPERSEDED by Part R's full section — R1 and R2, ~8–11; "saved in F8's `paths[]`" becomes an EXPORT beside generated code, D13)* | panel | the field view (waypoints, headings, reversed legs), mechanism steps picked from the robot's route ids, saved in F8's `paths[]`; the geometry-only visual (path, headings, the robot's outline to scale, markers, a step-through) — never a simulation; one validator (a marker naming an id the robot does not have is refused); the robot's FOOTPRINT added to the robot file | Part 6 | **~4–6** (rough, REMAINING-WORK row 20b) | its own section first | new |
| **S — the sensor pages: the camera and the side walls** (NEW, RC9) *(amended 2026-10-29 evening: split into S1 (walls, after L6-B) and S2 (camera, after VIS1-B), ~3–4)* | both | the `camera` (VIS1 §6.6) and `walls` (L6 §6.7) blocks in the tool, validated by mirroring `RobotConfig::valid()`'s camera and wall checks (agreement fixtures); robot two's `.camera` and `.wallSensors` adopted by moving `[chassis]`'s end marker below them (the differential re-run); a generated robot's table carries them; the SENSORS page (ports, the MEASURED mounts with tolerances and provenance, UNSET shown as the Step 0 to run); **the mounts are measured from the RECORD's front** — the end that led Calibrate's push, the library's +X — so the page names the record they were measured against, and the CALIBRATION page's flip warning names the camera and wall mounts (a flipped front mirrors every mount and swaps the left and right wall sensors — CK10) | VIS1-B and L6-B (the camera half may go after VIS1-B alone); Part 3 | **~2½–3½** (rough; ~1½–2 each half) | ~4 | new |
| **AF — the auto-flipper page** (NEW) | panel | the page over AF1's robot-file vocabulary. **The vocabulary in the tool is NOT Part AF's** (CK1, rule 6): if AF1 lands after Part 5, AF1 extends `tools/robot_tool.py` itself (its plan line's "panel generator ~½–1½" stays in AF1) or states its facts outside the regions; if AF1 lands before Part 5, Part 5 adopts it as landed (+½–1½ there) | AF1 lands; Part 6 | **~½–1** (rough: the page only; was ~1–2½ with the generator) | ~3 | new |
| **Parts 1–6** | | | | **~17¾–23¾** (+1–1½ with X-drive generation, §5 Q22 (b), in Part 2) | | 09-29: ~16¾–22¾ — **+1** (CK4: +½ against 09-29's ~17¼–23¼ with the watch, now CAL1's) |
| **All before G3** (1–6, 3b, R, S, AF) | | | | **~24¾–34¼** (CK1: AF is the page only; +½–1½ in Part 5 if AF1 lands before it) | | first pass: ~25¼–35¾ |
| **7 — the rest** | both | unchanged (§ the 09-29 table) | MECH's rest; after DOCS3, into its structure | ~2–3½ | — | = |

**The order and the lanes, in one line each.**
- **Tool lane:** 1 → 5 → 2 → 4 (one at a time; they share `tools/robot_tool.py`, `src/` and the gate).
- **Panel lane:** 3 from Part 1's landing (or MECH-A6's, when slots are full — CK3) → 6 after Parts 3 and 5 → 3b
  (NEW ROBOT) once Part 2 is in → R (CK6). Part 3 touches
  `tools/robot-panel/`, the record decoder (in its own module, `tools/robot_record.py`, so `tools/robot_tool.py` gains
  only the subcommand's few lines — the ONE exception to §1.2's "one file", CK11 (b): both files are standard
  library only; `robot_tool.py` loads its sibling BY PATH (`importlib.util.spec_from_file_location` on
  `Path(__file__).with_name("robot_record.py")`), so `python3 -I` and any working directory work; the panel (§1.7)
  and PKG1 (§1.10) carry both files), `test/CMakeLists.txt`'s record step and its documents — shared with the tool lane
  only by those few lines, the CMake file and the robot-file chapter (the rebase rule below).
- **Plug-ins:** S after VIS1-B and L6-B; AF after AF1; 7 after MECH's rest.
- **Critical path to a robot made and driven in the panel:** two lanes — max(1+5+2+4, 1+5+6+3b) ≈ ~11–15½ (3b in
  the panel lane ends at ~10¾–14½; CK6); one lane 1 → 5 → 2 → 3 → 6 → 4 ≈ ~17¾–23¾. Part 3 waiting for MECH-A6's
  slot (CK3) does not move it: Part 3 then ends at ~7–9 and Part 5 at ~7–9½.
- **DOCS3** comes after the panel's pre-G3 parts (1–6, R, S, AF). None of them needs DOCS3: each keeps its own
  documents current in TODAY's structure (the standing rule — docs in the same commit), and DOCS3 re-homes them. Part 7
  writes into DOCS3's structure. `G3a-COMPLETED.md` is written by whichever pre-G3 part lands last.

**How each size was reached** (beyond the table's notes): every 09-29 size is the starting point; each delta is a
piece of work that left (the front's words −¼ in Parts 1 and 3; Calibrate's hunks −½ in Part 2; the watch's "+~½"
in Part 5 released — it was never in that part's base; the handles file and the polarity region −¼ in Part 5; N4's
span form and the step-0 rule's base −½ in Part 4) or arrived (the re-pin floor +¼; the landed vocabulary +½ in
Part 5 and +½ in Part 6; the controls' compile-refusal agreement +½; route ids and `changeAt` +¼; the backup and the
front-check mirror +½ in Part 3; a new robot's mechanisms from creation +¼ in Part 2). Part 3b is carved out of
Part 3's size (the NEW ROBOT page was always in it), not added. Part AF left AF1's generator to AF1 or Part 5
(rule 6; −½–1½; CK1). The landed vocabulary is
counted from the headers (V1): `spec.hpp`'s 70 `Refusal`s, `wiring.hpp`'s 16 `EntryProblem`s and 9 list-level
sentences, `route_ids.hpp`'s 4, and `driver_inputs.hpp`'s 29 `refused::` names (12 of them one per button: "button X
does two jobs") — about 128, against the ~99 MECH §6.3 counted on 10-08 (`wiring.hpp`'s own ~50 compile-time
`refused::` names are the same rules as its run-time sentences, at build). R, S and AF are rough until their sections
are written; R's figure is REMAINING-WORK's.

### What G3a adopts — the mechanism vocabulary as landed, VIS1's camera block, L6's walls block

| Block | Source (as landed / as answered) | What the robot file carries | What G3a generates | Part |
|---|---|---|---|---|
| **Mechanisms** | MECH §6.3 (the field-by-field record G3a's §1.14 re-check works from) and the LANDED headers: `include/shulib/mechanism/{kinds,spec,wiring,route_ids}.hpp` at `73044ca` (MECH-A1–A3; every word of §6.3 is there — `Arm Lift Flywheel Power Pneumatic Forward Reversed rotationSensor distanceSensor opticalSensor limitSwitch cosine constantGravity noGravity restsAtStop homeBySwitch homeByStall absoluteSensor neverHomed spool linkage measured within High Low grabs releases gears direct`, `kinds.hpp` :1152-1241) | `mechanisms[]`, each with `kind` ∈ power / arm / lift / flywheel / pneumatic and that kind's members IN THE C++ DECLARATION ORDER (N11: a designator out of order does not compile — the re-check's own probe hit it). *(CK11 (a): the order, from `kinds.hpp` at `73044ca` — Power :679 `name motors cartridge pieceSensor safe stall maxVolts`; Arm :717 `name motors cartridge gearing angleSensor safe stall range gravity home presets unhomedVolts tuning`; Lift :811 `name motors cartridge gearing shape angleSensor safe stall range gravity home presets unhomedVolts tuning`; Flywheel :928 `name motors cartridge gearing angleSensor safe stall atSpeed presets tuning`; Pneumatic :1075 `name positions start safe moveTime pieceSensor confirmWithin` (+ `changeAt`). REQUIRED besides the below: every kind's `name`; a motor kind's `cartridge` and `safe`; a flywheel's `gearing` and `atSpeed` (`within(rpm, lasting)`, :944); optional: a power's `maxVolts` (12 V by default).)* `motors: ["forward", "reversed"]`; the named sensor slots; `gearing` as tooth pairs or `"direct"` (never a bare ratio); `range`; `gravity`; `home`; a lift's `shape`; `presets`; `stall` (REQUIRED on an arm, a lift and a flywheel — the template cannot invent one); `unhomedVolts`; `tuning`; a pneumatic's `positions [{name, levels, confirm}]`, `start`, `safe`, `moveTime`, `pieceSensor`, `confirmWithin`; an optional `id`; per-mechanism `notes` (the caveat comments) | the declarations and `inline constexpr MechanismTable k<Robot>Mechanisms = mechanisms(wire(…), …)` inside the robot's namespace (`using namespace mechanism::words;`); each role's `.changeAt` = its JSON path (`Motor::changeAt` :117, `Sensor::changeAt` :167, `Pneumatic::changeAt` :1091); robot two: ONE region over MECH's `[robot two mechanisms]` block (MECH §6.2 item 2 — no separate polarity, handles or binding region) | 5 |
| **Wiring** (Q1 (a): only here) | `wire(m, motors(…), angleSensor(n), pieceSensor(n or 'X'), homeSwitch('X'))` / `wire(m, lines('A', …))` (`wiring.hpp`); `wiringProblem()` :295 | `wiring[] {mechanism, motors[], angleSensor, pieceSensor, homeSwitch, lines[]}` — the named slots, never free role names | the `wire(...)` lines, in the mechanisms' order | 5 |
| **Controls** | `controls(list, whileHeld(m, f, b, V), toggle(m or m["speed"], b), preset(m["name"], b, t), homeOnPress(m, b, t), homeAtDriverStart(m, t), autonOnly(m))` (`driver_inputs.hpp`, MECH-A4 worktree; 24 at most, `DriverControls::kCap`); **A4-D27** (accepted 2026-10-29): a PRESET on a switch- or stall-homed arm or lift needs `homeAtDriverStart` or `homeOnPress`, and `.unhomedVolts` answers only for a mechanism that is HELD (V2: refused by name / passes); **A4-D26** (accepted): while the driver-start home runs, that mechanism's own buttons wait — run-time only; **A4-D13** (the freeze line, OPEN): nothing in the robot file changes, and F7 still freezes at G3 | `controls.whileHeld[]`, `.toggles[]`, `.presets[]` (with `timeLimitS`; none on a pneumatic position), `.homeOnPress[]`, `.homeAtDriverStart[]`, `.autonOnly[]`, each with its `words` | the `kControls = controls(...)` statement and the header's button words — robot two: the `controls` and `control words` regions, THE FEEL outside (D4, RC5) *(amended 2026-10-29 evening: THE FEEL is generated too — a `feel` region from the robot file's `driving` block, D11)*; a generated robot: the same two regions in its starter driver file (Part 4) | 5, 4 |
| **Route ids** | DERIVED, never stored (MECH §1.16; `route_ids.hpp`, 4 refusals: too long, alike, a built-in, more than 32) | nothing | nothing — the tool mirrors the rule and the four refusals by sentence; Part 6 shows each mechanism's ids; Part R picks from them | 5, 6, R |
| **Camera** | VIS1 D2 (a), §1.7, §6.6 (answered 2026-10-18); `config/camera_config.hpp` in `vis1-a1` (not yet landed): `CameraConfig {port, settings (hal::AiVisionSettings: colours[], objects, expectedClasses, colourMerge), placement (CameraPlacement: measured, forward, left, height, yawDeg, pitchDeg, tolerances, provenance), note}`; robot two's values as a `ChassisTable` member, `.camera` (VIS1-B) | `camera {…}`, every measured number with its provenance (who, how, when, under what light); `forward`, `left`, `yawDeg` are measured from the RECORD's front (the body frame's +X, `camera_config.hpp` :35-49 in `vis1-a1` — the end that led Calibrate's push), so a re-Calibrate that flips the front mirrors them (CK10) | `.camera = {…}` in the robot's chassis table; robot two's by moving `[chassis]`'s end marker below it | S |
| **Side walls** | L6 D5 (a), §1.4, §6.7 (answered 2026-10-22): `WallSensorsConfig {left, right: WallSensorConfig {port, mount: SideMount {measured, forward, left, height, facing, positionTolerance, facingTolerance, rangeBias, rangeAccuracy, coneHalfAngle, toggleChecked, provenance}}, limits: WallResetLimits {…}, note}` — vocabulary fixed by L6-A2; robot two's as `ChassisTable::wallSensors` (L6-B) | `walls {…}`; `left`/`right`, each mount's `forward`, `left` and `facing` (CCW from forward) are measured from the RECORD's front — a flipped front swaps the two sensors and mirrors their mounts (CK10) | `.wallSensors = {…}` | S |
| **The flipper** | AF1 (not briefed; a MOTOR ARM on both robots, "a generic automatic action in the MECH service" — build-order :1720-1723 at `73044ca`) | the flipper as an ordinary `arm` in `mechanisms[]` (today's vocabulary) plus whatever new form AF1's brief names | the Arm through Part 5's emitter; a NEW form by rule 6 — AF1 extends the tool itself if it lands after Part 5, or Part 5 adopts it if AF1 lands first (CK1) | 5 or AF1; page AF |
| **The footprint** (RC10) | MECH-PANEL §4 item 7b ("the robot's outline to scale"); both robots are 15" this season | `footprint {lengthIn, widthIn}`, human-stated with provenance (no program measures it) | nothing in C++ (the panel only) — Part R's section decides | R |

**Also carried into §1.11 (the `.vexbot` profile):** the camera and walls blocks become profile schema the moment
Part S emits them, and VexBuilder's published `robotProfile` has `sensors: [ { port, type } ]` only (L6 §6.7, RC32) —
the coordinator's note to VexBuilder (§4.10) gains both blocks beside the mechanisms' roles and wiring.

### Building beside VIS1, L6, AF1, L4 and MECH-A6 — the shared files, the risks and the rebase rule

| File | Who else edits it (when) | G3a's part | The risk, and the rule |
|---|---|---|---|
| `src/robots/chassis_table.hpp` | VIS1-B (`ChassisTable::camera{}` and robot two's `.camera`) and L6-B (`wallSensors{}` and robot two's), both after MECH-A5 merges and before A6 launches — i.e. beside Part 1 | 1 (`[chassis]`), 2 (`[chassis tables]`, `[chassis selection]`), S (moves the end marker) | **Highest.** Both append to robot two's initializer, inside the bytes Part 1 wraps. The rule: `[chassis]` ENDS before any appended member (V3: host and ARM, all three robots, the pins evaluated); a chunk landing after Part 1 writes its member AFTER the end marker and its notes beside it — never in the region's comment; a drivetrain fact (e.g. the provenance) changes through the robot file and `generate`. If VIS1-B or L6-B lands first, Part 1 extracts what is there |
| `include/shulib/config/robot_config.hpp` | VIS1-A1 (the camera's hooks), L6-A2 (the walls' hooks) | none — G3a writes no `include/` | The validator mirrors `RobotConfig::valid()`; a claim another chunk adds is mirrored by Part S, and until then the tool's own stricter `sensors[]` port claims cover a collision |
| `include/shulib/robot/robot.hpp` | VIS1-A1 (camera accessors, the boot line's camera words), L6-A2 (`walls %s` in `describe()`) | 3 (mirrors `writePortMap()`, :427) | Low: VIS1-A1's diff leaves `writePortMap()` alone and L6-A2 edits `describe()`; the record agreement test calls the real function, so any change turns it red |
| `src/common/boot_source.{hpp,cpp}`, `test/boot_source_test.cpp` | MECH-A5 (`bootConfigFrom`), VIS1-B and L6-B (the blocks' copy, a decision overload), SAFE-SAVE B2 (the read), L9 Part B (into the library) | 1 (D7's class-(B) comments and the CROSS-CHECK sibling case — test only) | G3a writes no `boot_source.*`; Part 1 re-greps at its tip |
| `tools/src_build_gate.py` | CAL1-B (Calibrate's entry names, a self-test plant), MECH-A5 (`_robot_namespaces`, `TEAM_FILE_ROBOT`) — both before Part 1 under D6′ (a) | 1 (`[generated]`), 2 (`[gate builds]`, merge lines, self-test robot), 4 (selection names) | None under (a); under D6′ (b) Part 1 rebases on both (different functions) |
| `src/build_selection.hpp`, `Makefile` | CAL1-B (the calibrate messages, :89-91 / :176-180) | 2 | CAL1-B first (N9) |
| `src/robots/mechanism_table.hpp`, `src/team/robot_two_driver.hpp` | MECH-A5 (rewrites both) | 5 (regions), 2 (`[mechanism selection]`, `[mechanism tables]`) | A5 first; after Part 5 nothing edits inside the markers |
| Robot two's `mechanisms`, `control words` and `controls` regions, once Part 5 has landed *(re-check 2026-10-29, CK1)* | **AF1** (robot two's flipper — a motor ARM, "a generic automatic action in the MECH service", on both robots; every declared mechanism needs a control or `autonOnly(m)` INSIDE `controls(...)` — `driver_inputs.hpp` :190-191's `refused::`); **MECH's rest** (any new vocabulary robot two uses); **L4** (only if it needs a robot-file fact — not briefed) | 5 (the regions); AF (the page) | Existing vocabulary (a plain Arm, a `whileHeld`, an `autonOnly`): rule 3. NEW vocabulary (a new kind, member or control form): rule 6 — a form inside `controls(...)` or a designated initializer cannot be written outside a region, so it is rule 6 (i) only |
| F8's route format; Part R's step vocabulary | **L4** (drive to a piece — a route step if its brief adds one; ~1, after VIS1 and ALLY1; not briefed) | R | Part R's own section (written before its go) re-reads L4 as landed; L4's brief is asked to say whether it adds a route step (§5) |
| `.github/workflows/ci.yml` *(CK11 (c))* | any chunk that adds a CI step (MECH-A1 added `ctest -R compile_refusal`; CAL1's and IMU1's briefs re-read its guards) | 1 (the freshness step in the host job), 3 (the panel's own job) | Low: Part 1 adds a step, Part 3 a separate job; whichever lands second re-reads the file at its tip (rule 1) |
| `src/programs/{competition,auton_tester,drive}_program.cpp`, `auton_tester_{run,panel}.hpp` | MECH-A5; ALLY1's `src/` half (the colour row); L6-B (the tester's wall line); AF1 (likely) | 4 (a few lines each) | Whichever lands second re-reads |
| `test/{chassis_table,mechanism_table,build_selection}_test.cpp` | VIS1-B and L6-B (robot two's blocks pinned), MECH-A5 (rewrites the mechanism test) | 1 (D7's re-pins), 2 (`kEveryRobotId`) | Re-read at the tip |
| `docs/` — the guide's chapter numbers, the changelog, the glossary, the README, `vexbuilder-integration.md` | every chunk | every part | The coordinator merges; a chapter takes the next free number at its tip (RC17) |

**THE REBASE RULE (G3a's, beside the project's PORT RULE):**
1. Whichever lands second re-reads every shared file AT ITS TIP and rebases onto the merged form — never a hand-merge
   of a copy taken before the other merged.
2. A G3a part that lands second RE-EXTRACTS every region whose bytes moved (extract from the tree, adopt, `check`;
   the differential re-run and logged); it never edits a region's bytes by hand.
3. A chunk that lands after a region exists never writes inside its markers: it changes the robot file and runs
   `python3 tools/robot_tool.py generate`, with `check` in its verification — or, for a block the tool cannot yet emit
   (the camera and walls before Part S), it writes OUTSIDE the region. The coordinator says so in every such chunk's
   prompt: VIS1-B, L6-B, L6-C, ALLY1's `src/` half, AF1, L4 *(CK1)*, MECH-A6, R5a Part C, SAFE-SAVE B2, L9 Part B,
   MECH's rest *(amended 2026-10-29 evening: and the string-mechanism chunks and R5a Part B — point 6 too)*.
4. A chunk that adds a C++ refusal the robot file can reach adds its fixture to G3a's agreement set in the same change,
   once Parts 1 and 5 have landed (else the validator silently lags the build).
5. Never two G3a parts in one lane at once; the two lanes share only the files named above.
6. *(re-check 2026-10-29, CK1)* **A chunk that lands after Part 5 and needs NEW vocabulary a region must carry** (a new
   kind, a new member on a kind, a new control form inside `controls(...)`) does ONE of two things, in its own change:
   - **(i)** extends `tools/robot_tool.py` — the emitter, the validator with its one-sentence refusal, `describe`'s
     vocabulary and the agreement fixtures (rule 4: run-time, or compile-refusal for a `consteval` form) — then
     changes the robot file and runs `generate`, with `check` and the tool's `self-test` in its verification; or
   - **(ii)** states its facts in a SEPARATE statement outside every region (VIS1-B's and L6-B's practice for `.camera`
     and `.wallSensors` before Part S), which a later G3a part adopts by moving an end marker.

   (ii) is impossible for a form that must sit INSIDE `controls(...)` or inside a mechanism's designated initializer,
   so such a form is (i) only. A chunk that only USES existing vocabulary (an `arm` with today's members, a
   `whileHeld`, an `autonOnly`) is rule 3. A chunk that lands BEFORE Part 5 needs neither: Part 5 adopts its form as
   landed (and grows by the emitter's share). The coordinator writes this into the briefs and prompts of AF1, L4 and
   MECH's rest, and every chunk after Part 5 (§5). **Why:** the tool cannot emit a form it has never seen, and the
   freshness gate refuses a hand edit inside a region — without this rule such a chunk could land neither way.

### The findings (each: what the tree or the plan says now; what the brief said; the correction; the body sections it touches)

| # | Now (tree / plan) | The brief said | The correction | Body sections touched |
|---|---|---|---|---|
| RC1 | The team lead's 2026-10-29 ordering (NOTES 10-29 ~12:20): foundation right after MECH-A5, beside VIS1/L6/AF1; DOCS3 after the panel; up to 3 jobs | Parts 1 → 2 → 3 "beside the MECH core" from T4 Part 3; Parts 4–6 "after the core"; DOCS3 (plan of 10-08) before any G3a build | D6′ and the parts table above; "beside the core" is moot (A5 lands first; only MECH-A6, `bench_r3a.*`, runs beside) | header; D6′; parts table; "The order"; §5 |
| RC2 | The front, ruled 2026-10-06: never asked, never typed; the record carries the push's end; Calibrate warns on a flip (MECH-PANEL §4.1) | The header said so; the BODY still had `drivetrain.front` (REQUIRED), `--front`, `ChassisTable::front`, two Calibrate prints, a front `static_assert`, the 160-character rule, S35/M30, landmines 21/25/26, the DRIVETRAIN page's words and read-back, the CALIBRATION page's "front cannot be checked", §1.11's `front` ask and the plain words | Struck everywhere, marked. `flip-front` STAYS without `--front`: it is the tool's answer to typed sides that face the wrong end (robot two's D7; CAL1's FIX YOUR TYPED DRIVE PORTS; the CALIBRATION page's FLIPPED) | §1.1; §1.2; §1.3; §1.7; §1.11; §1.13; §2; §3; §4; §6; §7; plain words; "What new-robot creation writes" |
| RC3 | CAL1 answered 2026-10-08: §1.7 — a generated robot's Calibrate needs NOTHING Calibrate-specific (its glue reads the build's tables); §5 — Part 2's three hunks and `ChassisTable::front` struck, the WATCH is CAL1's (D4 (a), HA-187), R11 (mechanisms before the first Calibrate), R13 (Calibrate only on a tank), Part 3's page reads `.bin` and `.bak` and mirrors `compareFronts` | Part 2 carried the hunks; Part 5 was the watch's fallback owner (+½); landmine 22 "until that exists" | Part 2 −½; Part 5's +½ released; R11 met by the new order (5 before 2) and the NEW ROBOT page's words ("declare every mechanism motor before Calibrate, or unplug it"); R13 a tool refusal (D9); landmines 17 and 22 re-worded | §1.13; §2 (Parts 2, 5); §3 Part 2; §5 items 5, 7; §6; §7 |
| RC4 | MECH-A1–A3 landed the vocabulary (`fd36e07`, `98392b2`, `43ac361`); A4 adds `driver_inputs.hpp`, `direction_check.hpp`, `runner/mechanism_routes.hpp` (worktree). MECH §6.2–§6.4 say what G3a adopts | §1.14 PROPOSED its own vocabulary (free-form roles, `units`, `autonOnly` on the mechanism, a handles SET region, four robot-two regions incl. a polarity region) | Superseded by the adoption table above (§6.3 as landed): five kinds, named slots, no handles file, robot two's regions THREE (`mechanisms`, `control words`, `controls`); the per-robot namespace keeps names from colliding across robots; route ids derived | §1.1 (mechanism rows); §1.2 (region kinds); §1.3; §1.6; §1.14; "What new-robot creation writes" items 5, 12–14; §2 Part 5; landmines 4, 5, 20 |
| RC5 *(amended 2026-10-29 evening: D4 REVERSED — THE FEEL becomes its own generated `feel` region, D11; MECH-A5 ended `[driver file two]` before THE FEEL, so the two regions never overlap)* | MECH §1.1 puts THE FEEL (`kFeel`, `kLoop`) inside the driver file's `// [driver file two]` block, and §6.2 item 2 offers that block to G3a as one region | D4 (a), ACCEPTED 2026-09-29: THE FEEL stays hand-written, the tool READS the re-zero | G3a's `controls` region wraps only the WHICH BUTTON heading and the `kControls` statement; THE FEEL stays outside (D4 holds). The coordinator asks MECH-A5's prompt to end `[driver file two]` before THE FEEL (else Part 5's `adopt --lines` takes the range). MECH §6.2's "TWO regions" drops the header's button words; G3a keeps a `control words` region over them (generated from each control's `words`) unless A5 removes the words from the header | §1.2; §1.3; D4's note; §5 |
| RC6 | `controls(...)` is `consteval`: every controls refusal is a NAME (`refused::button_R1_does_two_jobs`, …), with no run-time sentence; `specProblem()` (`spec.hpp` :1425-1429, documented "for G3a's agreement test") and `wiringProblem()` (`wiring.hpp` :295) ARE callable at run time on a declaration the build would refuse | §1.6: the agreement test runs the real checks at run time | The agreement test's spec, wiring and route-id halves stay run-time (V1: a stall-less Arm gives "position: the stall signature (.stall) is UNSET -- …"; a drive-port role gives "a mechanism role is bound to a DRIVE port"); its CONTROLS half is a compile-refusal set — each fixture a TU expected to fail naming its `refused::` function — on MECH-A1's harness (`test/compile_refusals/`, `ctest -R compile_refusal`) (V2), **plus ONE `DriverFile<kControls, kFeel, kLoop>` fixture with a control on the re-zero button, matched by its `static_assert` message (MECH N6, n57; A5 lands `src/common/driver_file.hpp`) — the one controls refusal that is not a `refused::` name, and the one D4 makes the tool mirror by READING the re-zero from THE FEEL** *(CK9)* *(amended 2026-10-29 evening: the re-zero is now STATED in the robot file's `driving` block and the fixture reads it there, D11)*. (n56, the brain's tick, is THE FEEL's, outside the robot file — no fixture.) +½ in Part 5 | §1.6; §2 Part 5 |
| RC7 | A4-D27 and A4-D26 ACCEPTED 2026-10-29; A4-D13's freeze line ruled OPEN | §1.14's controls had neither | The validator and the CONTROLS page mirror A4-D27 (V2); the page words A4-D26; A4-D13 changes nothing in the robot file | §1.14; §2 Parts 5, 6 |
| RC8 | The direction check names a role's `changeAt` (the generator's JSON path) and cuts it at **160** characters *(CK7 — the first pass said "130–160 (`%.130s`…`%.160s`)"; corrected: `changeAt` reaches the sentence through `changeHere()`'s `%.200s` into a 224-byte buffer and is printed at `%.160s` — `direction_check.hpp` :168, :224, :242, :272, :329, the GENERATED branches, MECH-A4 worktree, identical to main's staged copy; the `%.130s` (:315) and `%.150s` (:262) print `wireHere()`'s `wire(...)` place, never a `changeAt`; V7)*; its WIRING place is the captured `file:line` of `wire(...)` (`wireHere()`, :113-120; `MechanismWiring::file/line`, `wiring.hpp` :105-110) — for a generated robot, a line INSIDE a generated region | §1.14: "the generated table carries each role's JSON path" | A new literal-length rule: a generated `changeAt` ≤ **160** characters (V5: 84 for robot two; 117 at a 31-character id and a 23-character name; V7: 160 whole and 161 cut in all five generated sentences, the longest 377 bytes of the 384-byte `fix` — so the buffer is not the limit), refused naming the `%.160s` lines it protects; re-read at Part 5's tip, and against MECH-A6's station 13 if it prints the sentence through a smaller buffer. The wiring gap is an `include/` change, not G3a's: the coordinator asks **MECH's rest or a small library chunk** (CK11 (e): not MECH-A6, which writes only `src/programs/bench_r3a.*` and its test — MECH §2) for a `changeAt` on the wiring entry; until then the region's begin marker ("edit that file or the robot panel") is the pointer, and the robot-file chapter says so | §1.2; §1.14 (wiring page); §5; §7; §8 |
| RC9 | VIS1 D2 (a) and L6 D5 (a): the camera's and the side sensors' facts are robot-file data, typed as `ChassisTable` members (`.camera`, `.wallSensors`, appended with defaults) by VIS1-B and L6-B, both "after MECH-A5, before A6" | `sensors[]` "generates nothing yet (R2b)"; nothing about either block | Part 1's `[chassis]` ends before any appended member (V3); Part S brings both blocks into the tool and the SENSORS page; `sensors[]` keeps the radio and, until Part S, the camera's and walls' ports as claims; §1.11 gains both blocks and L6's VexBuilder gap | §1.1; §1.3; §1.11; §5; §7 |
| RC10 | Part R, ruled 2026-10-08 (REMAINING-WORK row 20b; MECH-PANEL §4 item 7b): routes drawn in the panel after Part 6, a geometry-only visual, no simulation (`3c643f1`, `2fce38a`) | absent | Added as Part R (~4–6 rough, its own section before its go). It needs the route ids (`route_ids.hpp`, landed; the binder `runner/mechanism_routes.hpp`, A4), F8's route format, and a fact the robot file lacks — the FOOTPRINT, for "the robot's outline to scale" — human-stated (no program measures it) | parts table; §1.16; §4; §8 |
| RC11 | In A4/A5's vocabulary a driver file is `controls(list, …)` + `DriverFile<…>` (MECH §1.13, `src/common/driver_file.hpp`, not yet in the tree); `controls(mechanisms())` with no controls compiles (V2 `kStarter`); a mechanism with no control and not autons-only is refused by name; A5 adds step 0's per-robot namespace rule | Part 4's starter in "today's `DriverBindings`" with span-spelled empty lists (N4), `DriverFor` beside a new step-0 rule | The starter emits `controls(k<Id>Mechanisms, …)` from the file's controls (`autonOnly` where the file says so) and `using <Camel>Driver = DriverFile<…>`; N4, S27, M20 and landmine 12 are moot; step 0's rule builds on A5's (−¼ each) | §1.13; §1.5; §2 Part 4; §7 (12, 16, 19) |
| RC12 | SIX whole-run listings at `73044ca` (V4): cookbook 06's three, chapter 16's two, chapter 17's one; MECH-A5 rewrites cookbook 06's three (MECH §4.1: the directive only "if G3a Part 4's rule has landed") | five, at the COMP-t tip | Six; Part 4 re-reads at its tip | §1.5; §3 Part 4 |
| RC13 | CAL1 D6 (a) and the boot use `tableToDescription(t).valid()` — signs AND geometry — before trusting a typed table | D10: typed ports "are the fallback" | False for a generated robot (the tool refuses signs and geometry there): typed ports reach neither the boot nor CAL1's check; they give build-time port checks and the WIRING page only (D10 above) | D10; §1.1; §1.7; §7 (22) |
| RC14 | The typed table's readers at the tip: Bench Tests' DRIVE station and MOTOR WATCH; the boot's CROSS-CHECK (`boot_source.hpp` :537-550, `countPortDisagreements` :340); CAL1's typed check once the geometry is typed | D7's reasons included the retired drive program and Teleop (`teleop_program.cpp` :48-52 — retired at T4 Part 3); the provenance cited "team lead [date]" for the front | D7 above: kept, re-justified; the provenance cites the record; the grep floor 22 files (V6) | D7; §1.1 "Why front"; §6 |
| RC15 | The 220-byte `provenance : %s` line is Bench Tests' (`bench_r3a.cpp` :196 buffer, :3200 line); the old drive program's line is gone (T4 Part 3); the description's own provenance (96 bytes) is filled from the GEOMETRY provenance (`tableToDescription`, `chassis_table.hpp` :384-420) | `bench_r3a.cpp` :191/:3192 and `drive_program.cpp` :134/:303 | The 206-character rule stands, cited to Bench Tests' line | §1.1 table; §1.2 |
| RC16 | HA-181 is in the register's Held line (`docs/hardware-assumptions.md` :39); the briefing's next free skips held numbers since L9 Part A1 (`fa915b0`); next free HA-216 | §5's three coordinator steps (OWED) | DONE; Parts S and AF display VIS1's, L6's and AF1's registered beliefs and add none; Part R's footprint is a statement, not a belief | §1.12; §5 |
| RC17 | Guide chapter 18 is MECH's mechanisms chapter (MECH-A3); CAL1-A2a's calibrating chapter and VIS1-A2's camera chapter each take the next free at their tips | "18 once COMP-b has landed" | The robot-file chapter takes the next free number at Part 1's tip (21 if CAL1-A2a and VIS1-A2 land first); DOCS3 may renumber | §3 |
| RC18 | Robot one has no build (I1). *(CK5: the first pass said "no G3a part can land before 2026-11-23" — false under D6′ (b), where Part 1 could.)* **No GENERATED robot can exist by the ~Nov 16 code freeze for 2026-11-23 under any D6′ option:** Part 2 follows CAL1 A2b + CAL1-B + MECH-A5 + Part 5 + Part 2 in one chain — ~12⅞–17⅛ builder-days from A2b's start even with Part 1 already in (b), ~19–26 with verification at half again (A2a is still in its fix round) | "Robot one is the first real user" | If robot one competes on 2026-11-23, its build is hand-written first (its own chunk, the coordinator's); D9 then reads it from the Makefile as a hand-written robot, and converting it to a generated robot later (delete its hand-written lines, then `new-robot`) is a separate call | §1.13 |
| RC19 | MECH §6.2 honoured G3a's eight requests: (1) `mechanismTableFor()` kept, no second selection function; (2) robot two's data in one block a region wraps; (3) `changeAt` per role; (4) the pin is the address only; (5) Calibrate is CAL1's; (6) "only robot two declares mechanisms" rewritten the chassis way; (7) the watch in CAL1; (8) the per-robot loop iterates `kEveryRobotId` once Part 2 lands; (9) MECH used HA-182..186 | §5's list "for the core's writer" | Each DONE or carried as stated; the file split with the core is moot (A5 lands first); Part 2's two marker lines in `mechanismTableFor()` and the test loop land after A5 | §1.4; §5 |
| RC20 | Line numbers moved (`73044ca`) | `ee2e399`'s / `a62067b`'s | The citation table below | §0.7 and every section citing them |
| RC21 | The gate's `RETIRED_BEACONS` (:427) holds `shulib-robot-variant=tank-library`, refused in every package | the collision rule covered live builds only | `new-robot` refuses an id whose derived beacon equals a retired one (`tank_library`); S22 gains it | D9; §1.13; §2 Part 2 |

### The checker's findings on this re-check (CK1–CK11; 2026-10-29, each re-verified — all real)

| # | The finding | Re-verified (evidence) | Fixed — where |
|---|---|---|---|
| CK1 (major) | Rule 3 has no answer for a chunk that lands after Part 5 needing NEW vocabulary inside a region (AF1 first); Part AF had taken AF1's "panel generator ~½–1½"; no ask to AF1's or L4's brief; L4 named nowhere | REAL. AF1's plan line (build-order :1720-1723 at `73044ca`): "a generic automatic action in the MECH service", a motor ARM on both robots, "plus the panel generator (~½–1½)". Every mechanism needs a control or `autonOnly` inside `controls(...)` (`driver_inputs.hpp` :190-191, mech-a4). Rule 3's "write OUTSIDE the region" cannot place a form inside `controls(...)` or a designated initializer. NOTES 10-29 ~12:20: the AF1, L4 and DOCS3 briefs are being pre-written | Rule 6 added; shared-files rows for robot two's regions after Part 5 (AF1, MECH's rest, L4) and for Part R's step vocabulary (L4); L4 in rule 3's list; Part AF re-sized to the page (~½–1), the generator AF1's (after Part 5) or Part 5's (before); the flipper row; §5's asks to AF1, L4 and MECH's rest |
| CK2 (major) | ~25 body passages in the retired 09-29 vocabulary unmarked although the re-check said every FALSE passage was marked | REAL — every cited line opened: §1.2's header ("Everything below holds"), validator table rows (`driver_controls.hpp` predicates, `mechanismTableProblem`), the "≠ 1 motor / ≠ 1 line" refusal (would refuse MECH §6.3's two-motor lift and three-position pneumatic), `k<Name>OpenCommand` and polarity constants, the baseline rule's value lines, the warnings (`openLevelVerified`, "claw pivot" at `calibrate_program.cpp` :1131 — CAL1-B replaces it), "the polarity region is ALWAYS present"; §1.3's in-region prose; §1.10's types; §1.11's mapping and renames; S16, S17; §3's prose list and greps; landmines 3–5. Builders read their part's sections only (focusRead) | Blanket marks at each passage, pointing to the adoption table and the ~128 landed refusals; the "≠ 1" refusal, `k<Name>OpenCommand`, the polarity constants and `openLevel*` struck by name |
| CK3 (major) | D6′ omits the slot arithmetic after MECH-A5: Part 1, VIS1-B and L6-B all wait for that window and all edit `chassis_table.hpp`; two lanes cost A6/AF1 slots under the three-job ceiling | REAL. VIS1 :6, :502 and L6 :564 put VIS1-B and L6-B "after MECH-A5 merges and before MECH-A6 launches"; VIS1's rule "if a slot is short, VIS1 waits, never MECH" (:1690-1691); three jobs at most (NOTES 10-29 ~12:20) | D6′ (a): the slot table, what each lane displaces, and the question put to him (which yields at two jobs; at one) |
| CK4 (minor) | "+½" is wrong: Parts 1–6 grew by +1 | REAL. 09-29: 2½–3½, 2¼–3¼, 3–3½, 2½–3½, 3½–5, 3–4 = 16¾–22¾; now 17¾–23¾ | "+1 (+½ against 09-29's ~17¼–23¼ with the watch)" in D6′, the table and the summary |
| CK5 (minor) | (b)'s held D7 flip has no home; its gain overstated; RC18 contradicts it | REAL | The flip is Part 5's first logged step under (b) (+¼–½ there); gain ~2–3¼; RC18 re-worded to "no GENERATED robot by the ~Nov 16 freeze", with the chain's arithmetic |
| CK6 (minor) | Part 3b's slot unresolved: "right after Part 2" in the tool lane moves the path to ~11¼–16 | REAL. With two lanes Part 2 lands at ~9–12½, after Part 3's tip (~5¾–9) | 3b runs in the PANEL lane after Part 6 and Part 2 (ends ~10¾–14½); the path stays ~11–15½; with one lane the page is inside Part 3 |
| CK7 (minor) | "`changeAt` ≤ 130" rests on a misread: `%.130s` cuts the `wire(...)` place | REAL. **V7** (`checker2-1029/v7_changeat.cpp`, the mech-a4 headers, `-Werror`): 160 characters WHOLE and 161 CUT in all five generated sentences; the longest `fix` 377 of 384 bytes | The rule is ≤ 160, citing the `%.160s` lines (RC8, §1.2, Part 5's row) |
| CK8 (minor) | The per-robot namespace (A5) is a derived name S22 does not cover | REAL. MECH §1.13 (`73044ca` :1966-1981): `_robot_namespaces`, team files `src/team/<ns>_*`. **V8** (`checker2-1029/v8_ns_*.cpp`): ids `std`, `hal`, `robot` as `shulib::bench::<id>` break `src/`'s unqualified `std::` / `hal::` / `robot::` (1 458 / 368 / 74 uses); `new` and `and` do not compile; `robot_three` passes. The prefix case (`robot` owning `robot_two_driver.hpp`) is by derivation — A5's gate code has not landed | §1.2's collisions, §1.13's id rules, S22 (and the id pattern tightened) |
| CK9 (minor) | RC6's controls compile-refusal set misses `DriverFile`'s re-zero `static_assert` | REAL. MECH N6 (`73044ca` :568-574, n57) | RC6, §1.6, §2 Part 5 |
| CK10 (minor) | The camera and wall-sensor mounts are front-relative, the robot file states no front | REAL. `CameraPlacement` (+X forward, `yawDeg` CCW from +X; `camera_config.hpp` :35-49, vis1-a1); L6's `SideMount` (`facing` CCW from forward, L6 :680-681) | The adoption table's camera and walls rows, Part S, the CALIBRATION page (§1.7), §3's chapter, §8 for VIS1, L6 and CAL1 |
| CK11 (minor) | (a) `atSpeed` (and `name`, `cartridge`, `safe`) missing from the Mechanisms row; (b) `robot_record.py` vs "one file"; (c) no CI row; (d) "the no-record fallback" unmarked; (e) the wiring `changeAt` ask sent to MECH-A6, whose scope is `src/programs/bench_r3a.*` only | REAL, all five (`kinds.hpp` :928-944; §1.2's title; `.github/workflows/ci.yml`; the 10-06 header; MECH §2's A6 row) | (a) the row lists each kind's members in order; (b) the one exception stated (loaded by path); (c) the CI row; (d) marked; (e) to MECH's rest or a small library chunk |

### Every citation the decisions depend on, re-opened at `73044ca`

| The brief cites (`ee2e399` / `a62067b`) | At `73044ca` | Note |
|---|---|---|
| `src/build_selection.hpp` :69-71 two-robot guard; :72-84 program guards | :79-81; :82-97 (count :82-85, drive :86-88, calibrate :89-91, autontest :92-94, competition :95-97) | CAL1-B rewords :89-91's message |
| `enum class RobotId` :99-103; the chain :115-139; `robotBeacon()` :148-158 | :113-117; :130-158 (six branches and `#else`); :167-178 | |
| `Makefile` `ROBOT` block :96-105; `PROGRAM` block :144-164 | :95-104; :166-193 (calibrate :176-180) | CAL1-B rewords :178 |
| gate tables :307-351; header :300-306; `TABLE_FILES` :400-401; `BUILD_PROGRAM` / `PROGRAM_ENTRY_SYMBOLS` :413-429 | :385-449; :378-384; :494-495; :507-530 | 7 builds; `RETIRED_*` :423-427 |
| gate "only `#error`" :138-141; `per_robot_table_names()` :522-552; `_stray_selection_problems()` :652-677 | header :203-206 and `_parse_selection_header` :687-735 (:720); :654; :811 | MECH-A5 adds `_robot_namespaces`, `TEAM_FILE_ROBOT` |
| `chassis_table.hpp` `ChassisTable` :66-87; robot two :89-140; `chassisTableFor()` :203-213; pins :224-237; `describeMissing` :295-308; `tableConsistent` :449-519 | :69-90; :92-143 (bytes identical); :206-216; :227-240; :298; :469 | VIS1-B and L6-B append two members |
| `mechanism_table.hpp` robot two :245-310; `mechanismTableFor()` :321-331; disjoint :611; pins :619-625 | :230-295 (`kRobotTwoMechanisms` :252); :304; `mechanismsDisjointFromChassis` :395; :502-523 (:522-523 "only robot two") | ALL rewritten by MECH-A5 (§1.13: `mechanismsClearOf`, the address pins) |
| `calibrate_program.cpp` :1044/:1068 banner, :1086 robot line, :1129-1131 drop, :1141 refusal, :1239 push | :1049/:1078; :1096; :1139 | CAL1-B replaces the file with its glue |
| `src/team/robot_two_driver.hpp` :18-19 (the drivers' front) | :16-17 | MECH-A5 rewrites below the button map |
| `teleop_program.cpp` :48-52; `drive_program.cpp` :134/:303 | gone (T4 Part 3); the 220-byte line is `bench_r3a.cpp` :196/:3200 | RC15 |
| `boot_source.cpp`'s header (the CROSS-CHECK) | `boot_source.hpp` :340 (`countPortDisagreements`), :537-550 | |
| `tools/api_doc_tool.py` globs (COMP-t tip :1836-1837, :1861) | `EXAMPLE_SOURCE_GLOBS` :1947; `WHOLE_RUN_SOURCE_GLOBS` :1972 | six whole-run listings (V4) |
| host lists: `build_selection_test.cpp` :35-36; `mechanism_table_test.cpp` :389-397; `chassis_table_test.cpp` :554-569 | :35-36; :384; :554-566 | |
| `driver_controls.hpp` :110-111 (no empty `[]`); `DriverBindings::bind()` :306 | RETIRED by MECH-A5 | RC11 |
| `R3a-BENCH-WORKSHEET.md` WHICH END IS FORWARD ~:522-529 | :452 | |
| `docs/hardware-assumptions.md` :84/:88 (held numbers in prose) | :39 (the Held line, HA-181 included) | RC16 |
| `calibration_record.hpp` :84 file name | :92 `kRecordFileName`, :102 `kRecordBackupFileName` (SAFE-SAVE A) | |
| `robot.hpp` `writePortMap()` ~:448 | :427 | VIS1-A1 leaves it alone |
| CAL1 brief (its §1.7, §5) | `docs/internal/chunks/CAL1-standard-calibrate.md` :2161-2180, :2895-2925 | |
| MECH brief §6.2–§6.4, §1.1, §1.13, §1.16 | `MECH-core-mechanism-library.md` :2543-2612, :839-1076, :1882-1988, :2035 | |
| VIS1 D2, §1.7, §6.6, §6.7; L6 D5, §1.4, §6.7 | `VIS1-ai-vision-colours.md` :512-525, :1036-1050, :1747-1800; `L6-wall-reset.md` :667-677, :1402-1418, :1993-2001 | |

### The re-check's probes — `.claude/coordinator/scratch/brief-G3a-recheck-1029/` (evidence, never project code)

| # | What | Result |
|---|---|---|
| **V1** | `vocab/probe_vocab.cpp`: robot two's mechanism region in the form MECH §1.1 says A5 leaves it, with `changeAt` filled as the generator would, against main's LANDED headers (`tip/` = `git archive 73044ca`); a stall-less Arm declared and never wired; `vocab/wire_drive.cpp`: `wiringProblem()` at run time | Host (the suite's flags, `-Werror`) and ARM compile; run: "robot two list: 3 mechanisms"; `specProblem(broken arm)` at RUN time: "position: the stall signature (.stall) is UNSET -- a held mechanism must never sit at stall current"; claw pivot 6's `changeAt` reads back; `wiringProblem` (an intake on drive port 11): "a mechanism role is bound to a DRIVE port" |
| **V2** | `controls/`: against MECH-A4's worktree headers — robot two's controls; a generated robot's empty starter `controls(mechanisms())`; R1 twice; A4-D27's preset beside `.unhomedVolts` with no homing control; held-only with `.unhomedVolts` | Robot two's, the starter (0 controls) and held-only compile on the host and ARM; R1 twice: `refused::button_R1_does_two_jobs`; the D27 case: `refused::a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control`. (A first draft put `.presets` after `.unhomedVolts` and failed "designator order … does not match declaration order" — the emitter must write each kind's members in the header's order, N11) |
| **V3** | `split/`: the tip's `chassis_table.hpp` + VIS1-A1's `config/camera_config.hpp`; `ChassisTable` appends `CameraConfig camera{}`; robot two's `[chassis]` region ends before a hand-written `.camera = {.port = 3, .note = …}` | Host and ARM `-fsyntax-only` pass for the bench, tank (robot two's pins evaluated) and X-drive selections; the bench bot's and the X-drive's tables omit the member and stay valid |
| **V4** | `bound_scan.py` (the 09-29 probe, unchanged) on `tip/` | Today's rule: "6 listings of whole-run files"; unnamed under the bound rule, all six fail: cookbook 06 blocks 5–7, chapter 16 blocks 1–2, chapter 17 block 1 |
| **V5** | `lengths/lengths.py` | Robot two's provenance 203; D7's 09-29 wording 200; a record-citing wording 208 (CUT at 206) and 197 (fits); `changeAt` 84 and 117, both ≤ 130 *(and so ≤ 160, the real limit — V7, CK7)* |
| **V6** | D7's re-pin grep at `73044ca` | 22 files (18 at `ee2e399`, of which the 09-29 lists named 11); the new ones listed under D7 |
| **V7** *(CK7)* | `checker2-1029/v7_changeat.cpp`: `judgeDirection` / `judgePneumatic` from the MECH-A4 worktree's `direction_check.hpp` (identical to main's staged copy), a 23-character name, 15-character position names, every role's `changeAt` (`source`) at 130, 159, 160 and 161 characters, each GENERATED branch (FightOrJam, MotorFights, SensorReversed, MechanismReversed, a pneumatic's swapped levels); host `g++ -std=c++20 -Wall -Wextra -Werror` | 160 WHOLE in all five sentences; 161 CUT in all five; the longest `fix` 377 of 384 bytes (`v7_changeat.log`) |
| **V8** *(CK8)* | `checker2-1029/v8_ns_*.cpp`: a generated id as `namespace shulib::bench::<id>`, beside `src/`'s own unqualified `std::size_t`, `hal::`, `robot::` | `std`, `hal`, `robot`: refused ("'size_t' in namespace 'shulib::bench::std' does not name a type", …); `new`, `and`: "namespace name required"; `robot_three`: compiles (`v8_ns_*.log`) |

### What changed since the last re-check (2026-10-06)
- **Landed:** MECH-A1 (`fd36e07`), A2 (`98392b2`), A3 (`43ac361`) — the vocabulary G3a adopts; SER1 (`2506067`; the
  gate's step 9); CAL1 Part A1 (`ceb17c0`; `front_check.hpp`'s `compareFronts`); the suite's bounded runner
  (`c09e022`).
- **In flight:** MECH-A4's final fix (`driver_inputs.hpp`, `direction_check.hpp`, `mechanism_routes.hpp`; A4-D26 and
  A4-D27 accepted and A4-D13 left OPEN on 2026-10-29); CAL1 Part A2a's fix round (`CalibrateFacts`); VIS1-A1 built,
  reviews pending (`config/camera_config.hpp`).
- **Answered:** CAL1 (2026-10-08; D6 (a)); the MECH core (2026-10-08, N1–N13, §6.2–§6.4 for G3a); the route ids and
  Part R (2026-10-08); DOCS3 and the auto-flipper before the panel (2026-10-08) — reversed to "beside" / "after"
  2026-10-29; VIS1 (2026-10-18; D2, ALLY1); L6 and IMU1 (2026-10-22; L6 D5; IMU1 shares no file with G3a).
- **Ruled 2026-10-29:** the foundation after MECH-A5, beside VIS1, L6 and AF1; the sensor and flipper pages as those
  land; DOCS3 after the panel; up to three jobs.
- **Done by the coordinator:** HA-181 held in the register; the briefing's next free past held numbers.
- **Checked the same day (second pass):** a checker's eleven findings on this section, each re-verified — all real —
  and fixed in place (CK1–CK11; probes V7, V8).

---

## Amended 2026-09-29 — the panel requirements

> **READ THIS FIRST. Where it and the body disagree, this section wins; every body passage it supersedes is
> struck through and marked *[superseded 2026-09-29 …]*, and the new scope's design is in the body's new
> §1.13–§1.16.**
> **Why.** On 2026-09-28 the team lead asked the panel to make a robot "super simple" — create a NEW robot with
> its build, add / remove / duplicate / reorder MECHANISMS from templates, bind their ROLES to ports on a WIRING
> page, give them CONTROLS — and ruled mechanisms port-agnostic. His requirements are
> `docs/internal/MECH-PANEL-requirements.md` §4 (the robot panel), P2 and P8, §4.10 and §5 Q20–Q22 (his; their
> recommendations stand until he rules), and §5's answers of 2026-09-29: **Q1 (a)** — bindings live ONLY in the
> robot file (edit the wiring, rebuild, upload; the wiring page has NO "bind on the brain"; the direction check on
> the brain REPORTS a wrong direction and names the robot-file line to change); **Q18 (a)** — generated typed
> handles, checked at build; **Q8 (a)** — a mechanism may be marked auton-only; **Q13 (a)** — three-position
> pneumatics state each position's levels; the front is per robot and human-stated. The ORDER is ruled
> (2026-09-28 ~17:15): the MECH core before the panel, the rest after it; **speedup D** (approved ~17:35) builds
> the panel's groundwork beside the core. **T4 Part 1b was folded into T4 Part 3 (2026-09-28): every "after T4
> Part 1b" in this brief now reads "after T4 Part 3".** D1–D6 (accepted 2026-09-29) hold; the table below says
> where the new scope re-shapes one.
> **Written** 2026-09-29 from a read-only pass over `shulib-v2` at `ee2e399`. `src/`, `tools/` and the
> `Makefile` are byte-identical to `a62067b` there (git diff: only `include/` and `test/` moved, TANK1 Part A),
> so every line reference in the body still holds. New probes: `.claude/coordinator/scratch/brief-G3a-amend/`
> (N1–N8 below). **The MECH core's brief is being written in parallel and did not exist when this was written:
> every mechanism field in §1.14 is PROPOSED from MECH-PANEL-requirements §3 and is marked "to be re-checked
> against the MECH core brief (`briefs/MECH-core.md`) before Part 5 starts".**
> *[Re-check 2026-10-29: SUPERSEDED by the header's 2026-10-29 status line.]*
> **Status: WAITS for the team lead's answers to D6′ and D7–D10 and his go for each part. Nothing is built.
> HA-181 stays G3a's reserve** (an executor who meets a hardware belief stops and tells the coordinator). The
> reservation is recorded only in this brief so far; §5 lists what the coordinator must do so that no other
> brief takes it (re-check 2026-09-29, #12).

### The new decisions (a recommendation for each; only questions the new scope raises)

> *[Re-check 2026-10-29: the 2026-09-29 text below is HISTORY. D6′–D10 as they are put to the team lead NOW are in "## Re-check 2026-10-29 — amendments": D7–D10 keep their recommendations with new reasons (RC2, RC3, RC13, RC14, RC21); D6′ is re-planned (RC1). Passages below that the re-check found false are marked where they stand.]*

**D7 — Robot two's front. RECOMMENDED (a): robot two's file states the CAMERA end — the end its calibration
record and its drivers use — by `flip-front`, as its own logged step right after the byte-for-byte differential.** *(re-check 2026-10-29: kept; the file states no front words since 2026-10-06 — the flip turns robot two's TYPED sides; the reasons and the provenance are the re-check's D7, RC14.)*
- **The question.** The team lead (2026-09-29): "the front will be different depending on the robot" — per
  robot and human-stated, which the design already is. Robot two's own front is his to confirm. Its calibration
  record of 2026-09-25 and its drivers use the camera end (the 11/20 end; `src/team/robot_two_driver.hpp` :18-19
  already says so). Its typed table says the 15/16 end (`src/robots/chassis_table.hpp` :91-96, :133).
- **(a) RECOMMENDED.** Part 1 adopts robot two's `[chassis]` region byte for byte FIRST, so the differential
  proves the tool on untouched bytes. Then it runs `flip-front` as a separate, logged data change whose
  provenance is his answer.
  - **Probed (N1).** As typed today, **0 of the record's 8 ports** agree with the file by side AND sign. Flipped,
    **8 of 8** agree. Ports 14 and 19 were left out at calibration either way. Flipping twice restores the
    sides and the signs; the provenance and the front's words are whatever the caller supplies (N1 re-supplied
    the originals by hand, which is why its file came back byte for byte — re-check 2026-09-29, #13).
  - **What changes, all in Part 1's one change:**
    - the region's `.left`, `.right` and `.provenance` lines. The provenance KEEPS the census, MOTOR WATCH and
      cartridge facts, adds the flip, and says the signs were DERIVED by negation, not re-measured. It must fit
      the banners' 220-byte `provenance : %s` line: 206 characters, the tool's rule (§1.2). **Probed at the
      re-check** (`recheck2/n1/`, since N1's first run REPLACED the provenance): "ports + IMU 2 + radio 1:
      census 2026-09-10; SIGNS: MOTOR WATCH 2026-09-10, negated by flip-front [date] (not re-measured); front
      = the camera end (team lead [date]); cartridge BLUE off a motor" is 200 characters with the dates
      filled in, keeps all seven facts, and gives 8 of 8. The flipped region, written into a copy of the tree,
      compiles on the host with robot two's pins and its mechanism disjointness evaluated (the tank define),
      and in the ARM compiler. A first fact-keeping wording of 210 characters would have been cut by the
      banner;
    - the notes' "THE FRONT is therefore the end where ports 15 and 16 sit", which becomes dated history;
    - **every restatement of robot two's typed sides, signs or front, found by GREP at the tip and never
      from a hand-made list** (re-check #8: the list here missed five sites). Across `test/`, `src/`,
      `include/shulib/` and the public docs, grep
      `LEFT -11|-11 \+12|\{-11, 12|\{20, -19|\+20 -19|15/16|15 and 16|sideOf\(1[56]\)|left side's -11|right side's \+16`
      and sort each hit into §3's three classes, logged in the PROGRESS file. Today's hits (at `ee2e399`):
      - **(C) test literals the flip makes FALSE, re-pinned in the same change:** `test/chassis_table_test.cpp`
        :57-59 (the hand copy `robotTwo()`, which the REAL-table case :573-600 forces to match) and every check
        on that copy — :89-105, :114-116, :148, :168, :385-394 (`sideOf(15) == -1`/`sideOf(16) == +1` at
        :393-394 included) — and the real table's own literals at :580-581;
      - **(B) present-tense comments the flip makes false, rewritten:** `test/mechanism_table_test.cpp` :85-86
        (the real table's "LEFT -11 +12 …"), :558 ("the left side's -11") and :561 ("the right side's +16");
        `test/boot_check_test.cpp` :42-43 ("chassis_table.hpp's provenance"); `test/boot_source_test.cpp`
        :163 ("its front is the 15/16 end"); and, if they are still in the tree after T4 Part 3,
        `src/programs/drive_program.cpp` :39 and `teleop_program.cpp` :51;
      - **(A) the tests' OWN fixtures that use robot two's pre-flip signs and never read the table, left as
        they are:** `calibration_record_test.cpp` :60-61, `boot_source_test.cpp` :167-169 and :524,
        `boot_check_test.cpp` :49-50, `brake_stop_stack_test.cpp` :92-93, `drivetrain_description_test.cpp`
        :58-59, `pros_device_provider_test.cpp` :420-471 and `pros_block_source_adapter_test.cpp` :102-103.
        The executor confirms each by grepping it for `kRobotTwoChassis` and `chassisTableFor`, and re-dates
        any comment that calls the fixture "robot two's table";
      - `boot_source_test.cpp` test 6i (:526) stays as dated history, since it reproduces the robot's
        2026-09-25 log line (10 disagreements). **Part 1 adds a sibling case:** the flipped table against the
        same record disagrees on exactly 2 ports, 14 and 19 (the table lists them and the record does not).
        That is the boot's CROSS-CHECK after the flip, as a host test. Probed against the real
        `countPortDisagreements()` (`recheck2/crosscheck/`): 10 as typed, 2 flipped;
    - the worksheet's WHICH END IS FORWARD (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md` ~:522-529) and
      Station F, re-dated.
  - **Why it matters.** After T4 Part 3 retires the old "shulib Drive", the typed signs are read by Bench Tests
    (its DRIVE station, MOTOR WATCH) and by the boot's record/table cross-check (`src/common/boot_source.cpp`'s
    header). After the flip, both agree with the record, and every program drives toward the end the drivers
    use. Today Bench Tests and the drivers' programs drive opposite ways on the same stick
    (~~`src/programs/teleop_program.cpp` :48-52 describes it~~ *(re-check 2026-10-29: Teleop retired at T4 Part 3; the CROSS-CHECK is `src/common/boot_source.hpp` :537-550, and CAL1's typed check reads the table once its geometry is typed — RC14)*).
  - **Size:** ~¼–½ day, inside Part 1 (the grep-driven re-pin and the 6i sibling case included).
- **(b) Keep the 15/16 end.** Cost: the panel shows FRONT REVERSED for robot two until someone flips it, and
  Bench Tests keeps driving opposite to the drivers.
- **(c) Re-measure the signs camera-end-first (MOTOR WATCH) before flipping.** Cost: a robot session. N1
  already predicts the answer (the negated signs agree with the record 8 of 8), so it would confirm rather than
  decide.

**D8 — Which programs a NEW robot's build carries. RECOMMENDED (a): the five robot two has after T4 Part 3
and COMP-b, in two steps — the bring-up builds in the groundwork (Part 2), the match builds after the MECH core
(Part 4).** *(re-check 2026-10-29: kept; the list matches today's builds; Part 4 now follows Part 2 in the tool lane — the core's `src/` half lands before Part 1 — and its starter is written in A4/A5's vocabulary, RC11.)*
- **What exists then** (the T4 and COMP briefs as re-checked 2026-09-28):
  - robot two's builds are Bench Tests (`ROBOT=tank`), shulib Drive (the driver file, `PROGRAM=drive`), shulib
    Calibrate, the Auton Tester (`PROGRAM=autontest`, COMP-t) and the Competition program
    (`PROGRAM=competition`, COMP-b); `PROGRAM=library` is REMOVED at T4 Part 3;
  - the Makefile and `src/build_selection.hpp` refuse each of them on any robot but `ROBOT=tank`;
  - the programs name robot two's driver class (`RobotTwoDriver`) and its auton list (`registerAutons()`,
    `src/team/autons.cpp`) DIRECTLY.
- **(a) RECOMMENDED.**
  - **Part 2 gives a new robot its BRING-UP builds:** Bench Tests (the census, and the MECH core's bench page
    and direction check) and Calibrate. Both already read only `kChassis` and `kMechanisms` (probed N2: a
    generated robot's two builds pass the real gate).
  - **Part 4 gives it its MATCH builds:** shulib Drive, the Auton Tester and the Competition program select the
    robot's driver file and auton list through `kRobotId`, as the tables are selected — `DriverFor<RobotId>`
    (one specialization per driver file; a robot with none is a compile error when its build selects it) and
    `autonsFor(RobotId)` (a case per robot, no default). Each generated robot gets a STARTER DRIVER FILE and a
    STARTER auton file with an EMPTY list: the students author the autons (the G4 guardrail). *(Re-check
    2026-09-29, #2: the starter driver file was Part 2's. It moved here because its bind call and its controls
    lists are the controls vocabulary the MECH core rewrites, and the bring-up builds never include it.)*
  - **Step 0 gains the rule the tables already have:** a program that names a robot's driver class or auton
    function outside the selection FAILS. Probed (N6): today's derived per-robot set is the tables and their two
    selection functions only, so a program naming `RobotTwoDriver` in another robot's build would pass silently.
    That is the defect class G1 Part 2c measured for tables.
  - **Why Part 4 is not in the groundwork:** it edits the drive, auton-tester and competition programs, which the
    MECH core's `src/` half is expected to edit too. The mechanism service is a pacer decorator composed with the
    guard the competition program builds, and the auton's mechanism lookup is handed over there
    (MECH-PANEL §3.0, §3.4). Two builders in one program file at once is the collision the one-thing-in-flight
    rule exists for. And Part 4 now carries the starter driver file, whose bind call and control lists are
    the core's vocabulary (re-check #2). If the core's brief leaves those programs and that vocabulary alone,
    Part 4 can join the groundwork. That is D6′ (b), which the coordinator brings to the team lead at the
    core's brief.
  - **Cost:** Part 4 ~2½–3½ days (re-sized at the re-check: +½ for the starter driver file moved in, +½ for the
    doc gate's bound listings, #1). The src gate grows by five builds per generated robot (~11 s a build measured
    here, so ~1 min per robot). The gate's self-test builds a generated robot, so CI exercises the path even
    while the repository carries none.
- **(b) Bring-up builds only.** Cost: "creates the robot's build too" becomes half true. The robot cannot be
  driven or run an auton from its own build until someone hand-writes the programs' selection.
- **(c) Bring-up plus the Competition program only** (no drive-practice program, no auton tester). Cost: it
  saves two builds per robot (~20 s of gate time), but the drivers practise in the competition program's
  `opcontrol()`, and the students test autons only through the competition program under field control or the
  controller's Timed Run (the team has no competition switch).

**D9 — Where a generated robot's BUILD plumbing is written. RECOMMENDED (a): SET regions — one marked region
per location, holding every generated robot's lines, adopted before the first robot exists (empty, except
`[robot guards]`, which also holds the hand-written robots' guards, read from the Makefile).** *(re-check 2026-10-29: kept; re-read against the gate at `73044ca` — the re-check's D9, with CAL1's R13 and RC21's retired beacon.)*
- **The problem.** New-robot creation must write where no robot file owns anything: the one chain, the
  `RobotId` enum, `robotBeacon()`, the two selection functions, the Makefile's `ROBOT` block and the gate's
  tables. D2's regions each belong to ONE robot file.
- **(a) RECOMMENDED.**
  - Each location gets ONE region, generated from every robot file that has a `build` block (sorted by id).
    Its marker names `src/robots/*.robot.json`.
  - Part 2 adopts all of them before any robot is generated (the always-present polarity region's precedent,
    re-check #15.1), so creating or deleting a robot never inserts or removes a marker. With no generated robot,
    every region but one is empty. **`[robot guards]` is not** (re-check 2026-09-29, #9). Its guards are whole
    `#if … #error … #endif` blocks, and each one names EVERY robot define:
    - the COUNT of robot defines, over all robots;
    - "`PROGRAM=P` needs a robot whose programs list it", over the robots that list `P`.
  - **Why the guard blocks name the hand-written robots.** Two rules rule out the alternatives:
    - a `//` marker cannot sit inside a backslash-continued `#if`, so a region cannot hold part of a guard;
    - the gate allows nothing but `#error` inside a block that reads a switch (`tools/src_build_gate.py`
      :138-141), so no helper macro may carry the hand-written robots' part.
  - **Where the tool reads the hand-written robots' facts: the MAKEFILE, never a list typed in the tool.**
    - Part 2's one-time Makefile edit states the hand-written robots in the generated robots' shape, OUTSIDE
      the region (`SHULIB_ROBOT_DEFINE_tank := -DSHULIB_ROBOT_TANK_2026`, `SHULIB_ROBOT_PROGRAMS_tank := …`, the
      same for `xdrive`; the bench bot is the chain's `#else` and has no define).
    - The `ROBOT` block uses `$(SHULIB_ROBOT_DEFINE_<robot>)` itself, so these lines cannot drift from what
      `make` does.
    - The tool reads every `SHULIB_ROBOT_DEFINE_`/`SHULIB_ROBOT_PROGRAMS_` pair, both inside and outside the
      region. A define line with no programs line is a `ToolFault`.
    - `[robot guards]`' marker names both sources.
    - **Probed** (`recheck2/guards/`): the guards emitted from the Makefile equal, line for line, the regions
      N2 and N3 built with (whose facts `newrobot_apply.py` had typed in Python). The real gate PASSES all
      8 builds with the `ROBOT` block reading the variables ("defines token-exact"; 3 m 28 s).
    - A chunk that adds a program to robot two edits `SHULIB_ROBOT_PROGRAMS_tank` (it must anyway, or `make`
      refuses the program) and runs `generate`. Until it does, `check` names the region STALE, never "edited
      by hand" (§5).
  - Robot two's build stays HAND-WRITTEN and grandfathered, like its `typedTable`. Its file has no `build`
    block; the tool refuses one there and refuses a missing one on any other robot.
  - In the `Makefile` and `tools/src_build_gate.py` the markers are `#` comments. The tool compiles the gate's
    new text with Python's `compile()` before it writes (a `ToolFault` otherwise), the generate⇒check
    invariant's second half for a non-C++ target.
  - **Probed:** with no generated robot, the real gate PASSES (6 builds, N3). With one generated robot it
    PASSES (8 builds, 2 m 25 s, N2), and the host TUs that read the selection pass 101/101 cases (6 827
    assertions). Three step-0 mutations each fail by name (N6).
- **(b) Per-robot regions: the tool inserts a new marker pair per robot at each of ~10 places.** Cost: each
  place is found by an anchor string in hand-written code (fragile); deleting a robot leaves markers to remove
  by hand; ten more region kinds per robot.
- **(c) The gate DERIVES its rows from the robot files at run time.** Cost: the gate stops being an independent
  second copy of the chain ("a build added to one place and not the others fails here, which is the point of
  having the tables", `tools/src_build_gate.py` :300-306). A hand edit of the chain would be checked against the
  same source that generated it.

**D10 — A NEW robot's drive ports. RECOMMENDED (a): optional (the four §4.1 facts are the only required ones).** *(re-check 2026-10-29: kept, re-justified — THREE facts (the front is never asked); the "fallback" below is false for a generated robot (RC13); the front cost is gone and the drive-port hazard is caught by CAL1's watch (RC3).)*
- **The question.** §5 Q21 is his, and its recommendation (a) — the drive's ports stay typed in the robot file —
  stands. Under (a), must a NEW robot type its ports before its first build? §4.1's new-robot page asks four
  facts: name, drivetrain kind, cartridge, front. The runtime-discovery ruling (2026-09-10) says only the
  cartridge, the kind and the front are human-stated.
- **(a) RECOMMENDED: optional on a new robot.**
  - An empty side is what `kNoChassis` already is: a legal table that `describeMissing` names.
  - Calibrate discovers the ports: it takes every motor the brain's registry reports, minus the declared
    mechanism ports. The programs then boot from the record.
  - When a team does state ports, ~~they are the fallback~~ *(re-check 2026-10-29: never, on a generated robot — RC13)*, and the calibration page compares them with the record.
  - Robot two keeps its typed ports, which the differential needs.
  - Probed (N2): a generated robot with no typed ports builds Bench Tests and Calibrate through the real gate.
  - **Its costs, stated (re-check 2026-09-29, #3 and #10):**
    - *[Re-check 2026-10-29: SUPERSEDED — no front is stated (2026-10-06); the record carries the push's end (RC2).]* **The front cannot be CHECKED.** The human-stated front generates no C++ that could compare it with
      anything. The record has no front (landmine 10), and the panel's FRONT REVERSED compares port SETS, which a
      portless file does not have. So the robot's real front is whichever end the operator leads the push with
      and answers FRONT FIRST to, at Calibrate.
      - **What G3a does about it (Part 2; probed, `recheck2/front/`):** the words reach that moment. The chassis
        table carries them as `.front`. Calibrate prints them beside its `robot :` line and again before the
        push ("LEAD THE PUSH WITH THE FRONT THE ROBOT FILE STATES (…), then answer FRONT FIRST"). A Calibrate
        build of a robot whose file states no front fails to compile (§1.13).
      - **What it cannot do:** the screen's WHICH END FIRST? is the library's panel, which G3a does not change.
        So the words reach the serial log, not the screen (§8). And the calibration page of a portless file
        says "no typed ports: this file's front cannot be checked against the record".
    - **The mechanism-versus-drive port rules check nothing at build.** The wiring rule "not a DRIVE or IMU
      port" (§1.14) and `mechanismsDisjointFromChassis(kMechanisms, kChassis)` (`mechanism_table.hpp` :611)
      compare against TYPED drive ports, and a portless robot has none. So a role wired to a real drive motor
      passes validate, generate and the build.
      - **Calibrate then drops that port silently** as a declared mechanism port
        (`calibrate_program.cpp` :1129-1131). The record never carries it, so the Part 6 calibration-page
        warning ("a role on a port the record DRIVES") cannot see it either. The robot drives with one motor
        short, and that motor answers the mechanism's buttons.
      - **The fix is in Calibrate.** Calibrate should WATCH the declared mechanism ports during the push,
        reading them and never commanding them. It prints "port N is declared as [mechanism]'s [role] but
        TURNED WITH THE PUSH: it may be a drive motor, check the wiring" (§5 asks the MECH core's brief, which
        edits that hunk; Part 5 is the fallback owner).
      - *[Re-check 2026-10-29: it exists — CAL1's WATCH, D4 (a), HA-187 (RC3).]* **Until that exists, only the drivers notice.** The hazard starts at Part 5, the first part where a
        generated robot can have mechanisms (landmine 22).
- **(b) Required.** Cost: it contradicts the ruling for every new robot, and a mistyped port is caught only by
  the record comparison, after a calibration, anyway. It would restore the build-time port check of #10 for the
  ports the team typed correctly.

**D6′ — The parts, their order and their size (it REPLACES D6, which he accepted on 2026-09-29). RECOMMENDED
(a): the seven parts of the table below, each its own go. Parts 1 → 2 → 3 are the groundwork beside the MECH
core, strictly one after another; Parts 4, 5 and 6 come after the core; Part 7 comes after the rest of MECH.** *(re-check 2026-10-29: RE-PLANNED — the re-check's D6′ and parts table replace this; RC1.)*
*(Added at the re-check, 2026-09-29, #5: the seven-part table replaces a decision he accepted, and it moves the
match builds (Part 4) out of speedup D's groundwork. That is his to rule, not a detail.)*
- **(a) RECOMMENDED.**
  - Part 4 is outside the groundwork, because it edits the drive, auton-tester and competition programs, which
    the core's `src/` half is expected to edit too (D8).
  - Part 4 also carries the starter driver file, whose controls are the core's vocabulary (re-check #2).
  - Parts 1–6 are ~16¾–22¾ builder-days.
- **(b) (a), but Part 4 joins the groundwork if the core's brief leaves those three programs and the controls
  vocabulary alone.** The coordinator brings it to him at the core's brief, with the file split. Cost: none if
  the core truly leaves them. If it does not, two builders work in one program file, which is what the
  one-thing-in-flight rule forbids.
- **(c) Fewer, larger parts** (for example 1+2 and 5+6). Cost:
  - one go and one verification fewer each;
  - each merged part's documentation passes ~9–10 documents, past rule 7 (~5);
  - the panel's design learns less from the tool (D6's own reason for splitting).

**Decided here, overrulable, with the reasoning in the body:**
- the templates are DATA in the tool (`describe --json`), never in the panel (D3's one source);
- a mechanism may be saved as a **draft** (`"draft": true`: carried in the file, generating nothing, its empty
  numbers shown as warnings rather than refusals) so a half-made mechanism does not leave the tree failing
  `check` (§1.14);
- Duplicate copies a mechanism's settings but NOT its wiring or its controls (either would be refused as a
  port or a button used twice);
- the mechanism edits (add / remove / duplicate / rename / move) are TOOL commands the panel calls, so a
  text-editor user gets the same edit and the same list of effects (D3: one writer);
- removing a robot is a manual procedure the guide states, not a panel button (§1.13);
- the new-robot id is refused when any derived name, DEFINE or BEACON equals another build's, including the
  hand-written ones: `tank_drive` → `shulib-robot-variant=tank-drive` is robot two's drive build; `probe` plus
  `probe_calibrate` give one beacon; `tank_2026` → `SHULIB_ROBOT_TANK_2026` is robot two's define, and the
  define is the ONLY name of that id that collides (re-check #11) (§1.13).

### Every change the amendment makes

*(re-check 2026-10-29: A1's seven parts, A2's part numbers, A4's four robot-two regions (three now, RC4/RC5), A7's vocabulary (MECH §6.3 as landed), A14's "front" (robot two's TYPED sides, RC14), A15's step-0 rule (on MECH-A5's, RC11), A18's sizes and A20's OUT list are revised by the re-check; the rest hold.)*

| # | Change | Body sections | Source |
|---|---|---|---|
| A1 | **Parts and order.** D6's two parts (A, then B, A "after T4 Part 1b") are replaced by seven parts in three phases (the table below). Every "after T4 Part 1b" reads "after T4 Part 3" | header *Runs*; D6; §1.1's post-1b deltas; §1.3; §5 | speedup D; T4 1b folded into Part 3 |
| A2 | **A NEW ROBOT is IN** (it was OUT, §4): the robot file AND its build — the RobotId and chain branches, the `make ROBOT=` value, the tables in the two TABLE_FILES, the gate's rows (Part 2); the match programs, a starter driver file (moved from Part 2 at the re-check, #2) and a starter auton file (Part 4) | new §1.13; §4 | §4.1; D8–D10 |
| A3 | **The robot file's shape.** Mechanisms carry ROLES, never ports; a separate `wiring` block binds role → port and is the ONLY place a binding lives (Q1 (a)). New: `build` (generated robots), `kind`, per-kind settings, `presets`, `controls.presets`, `autonOnly`, `draft` | §1.1 (marked), new §1.14 | P2, Q1, Q8, Q13; MECH core |
| A4 | **Robot two's regions split across parts.** `[chassis]` is adopted in Part 1: its bytes are unchanged since `a62067b`, and P1's differential is re-run at the tip. `[pneumatic polarity]`, `[mechanisms]`, `[control words]` and `[controls]` are adopted in Part 5 against the tree AS THE MECH CORE LEAVES IT (the core rewrites the mechanism table into roles and a binding and adds presets to the driver file), and their differential is re-done there by the same extract-then-check method | §1.3, §0.6 P1 | the core rewrites those bytes |
| A5 | **§1.1's verbatim robot two file** stays as history and as Part 1's drivetrain half, verbatim. Its mechanism half is re-made in Part 5 from the tree | §1.1 | A4 |
| A6 | **D5 (a) splits.** The chassis pin's half lands in Part 1. The mechanism pin's half goes to whichever of the MECH core and Part 5 rewrites that pin; the coordinator tells the core's brief | §1.4; D5 | A4 |
| A7 | **The mechanism vocabulary and the eight templates**, each with its settings (the core's kinds now; Catapult's, Conveyor's and Intake's extra settings with the rest) | new §1.14 | §4.2, §3 |
| A8 | **Remove / duplicate / reorder / rename, and Remove's effect on autons.** Under Q18 (a) the build fails and GCC names each auton FUNCTION that used the handle (probed N5); the tool lists the uses before the save | §1.14 | §4.2, Q18 (a) |
| A9 | **The wiring page.** Roles → ports, with live warnings; NO "bind on the brain" (Q1 (a)); 3-wire lines marked "not checkable on the brain"; the wrong-kind warning is limited to what the drivetrain record carries (the drive ports and the IMU) | §1.14; §1.7 | §4.3; Q1 (a) |
| A10 | **Controls:** hold, toggle and presets (the core). The rest's controls come in Part 7 | §1.14 | §4.4; Q8 |
| A11 | **The calibration record view** (read-only, with the disagreement warning) moves into the groundwork (Part 3) | §1.7 | §4.5 |
| A12 | **Save** = validate + generate + check + the witness list, through ONE validator. An optional "Build this robot" command runs `make`; it is never a gate and never runs on save | new §1.15; §1.7 | §4.6; D3 |
| A13 | **Import and export are G3's and later**: the hooks are named, not built | new §1.16 | §4.7-4.8 |
| A14 | **Robot two's front** (D7): `flip-front` after the differential | §1.1 "Why front"; §8 | his answer |
| A15 | **The gate.** SET regions in `tools/src_build_gate.py` (D9). `TABLE_FILES` is unchanged: a generated robot's tables live in the two table files, and step 0 derives them (probed N2: "8 per-robot names", `kProbeBotChassis` among them). Step 0's driver/auton rule (Part 4). The gate's self-test generates and builds a robot | §1.5 | D8, D9 |
| A16 | **New landmines:** zero-length control lists (N4); markers in a Makefile and in Python; the Calibrate banner that says "robot two"; ids and beacons that collide; the per-robot driver-class hazard; the gate's growing time | §7 | the probes |
| A17 | **VexBuilder:** §3.1's revision becomes roles plus a binding block (P2, §4.10); `build` is shulib-only; ask Q20 | §1.11 | P8 |
| A18 | **Sizes:** Parts 1–6 ~16¾–22¾ days (re-check #4: the draft's "~15½–21" mis-added its own table, whose high ends summed to 21½, and the re-check's moves add ~1¼ more). The requirements' §7.1 priced G3a amended at ~13–18½; the difference is D8's Part 4 and five more documentation passes. Part 7 ~2–3½ | the parts table | honest sizing |
| A19 | **Documentation per part** (pace rule 7; Part 1 is ~6 public documents, one past it, and §3 says why) | §3 | |
| A20 | **The OUT list.** Adding a robot moves IN. New OUT items: removing a robot from the panel; X-drive generation unless Q22 (b); guided binding; the rest's kinds and controls until Part 7 | §4 | |

### The parts, their order and their sizes (replaces D6's)

> *[Re-check 2026-10-29: SUPERSEDED by the re-check's parts table — ten parts in two lanes, the old numbers kept, Part 5 moved before Part 2, Parts R, S and AF added, every size re-checked (RC1). The table below is the 2026-09-29 plan.]*

| Part | Phase | What it builds | Starts after | Size (builder-days) | Docs |
|---|---|---|---|---|---|
| **1 — the tool, robot two's drivetrain, the gate** | groundwork, beside the MECH core | `tools/robot_tool.py` (validate / generate / check / fmt / adopt / describe / flip-front / self-test; the marker protocol; the text-field, baseline and collision rules; the banner-length rule for `drivetrain.provenance`, re-check #13); robot two's file (identity, drivetrain, `typedTable`, sensors — the mechanism keys are REFUSED until Part 5, by name); `[chassis]` adopted with the differential; D7's flip (if (a)) with its grep-driven re-pins and the CROSS-CHECK sibling case; D5's chassis pin with its named predicate; the agreement test's chassis half; the freshness gate (the host doc gate, the src gate's `[generated]` step, CI) | T4 Part 3 lands — when the core's `src/` half starts; its library half runs alongside T4 Part 3 (Q25, MECH-PANEL §5 :536) | **~2½–3½** | ~6 |
| **2 — a new robot's bring-up build** | groundwork | the SET regions adopted before any robot exists (with the differential: the zero-robot emission equals the adopted bytes); `new-robot`; the one-time hand edits (the guards' general form; the Makefile's hand-written robots in the generated shape, which its `ROBOT` block reads and the tool reads for `[robot guards]`, re-check #9; the Makefile's generated-robot branch and program lists; the gate's merge lines; `kEveryRobotId` and the tests reading it; `ChassisTable::front`, re-check #3); Calibrate's banner from `kChassis.robot`, the front's words printed before the push, and its front `static_assert`; the gate's self-test case that generates and BUILDS a robot. **Nothing under `src/team/`** (the starter driver file moved to Part 4, re-check #2) | Part 1 | **~2¼–3¼** | ~4 |
| **3 — the panel shell** | groundwork | the extension (custom editor, tool runner, `describe`-driven vocabularies, diagnostics by JSON path, model tests, `.vsix`, CI job); the NEW-ROBOT page (a dry run lists every file it will write); the DRIVETRAIN page (the front and its read-back, "the OTHER end is the front", ports, IMU, sensors, `typedTable` read-only); the CALIBRATION page (the record, read-only, with the disagreement warning, and "no typed ports: this file's front cannot be checked against the record" for a portless file; the Python decoder and its host agreement test); Save | **Part 2** (re-check #5: strictly after it, since one groundwork part is in flight at a time and its new-robot page needs Part 2's `new-robot`) | **~3–3½** | ~5 |
| **4 — a new robot's match builds and its starter files** (D8 (a)) | mechanisms, after the MECH core | `DriverFor<RobotId>` and `autonsFor(RobotId)`; the STARTER DRIVER FILE (moved from Part 2, re-check #2), in the controls vocabulary the core leaves, with its regions, its host TU and `generate` writing a missing one when a robot's programs first list a match program; the starter auton file; SET regions for the driver-file includes and the auton cases; the program guards read each robot's program list; step 0's driver/auton rule with its self-test plants; **the doc gate's bound listings** (`tools/api_doc_tool.py`: a listing of a driver file names the file it quotes, and cookbook 06's three listings gain their directive, re-check #1); the gate's self-test builds the generated robot's match programs | the MECH core lands (its `src/` half included); Part 2 | **~2½–3½** | ~5 |
| **5 — the mechanisms in the tool** | mechanisms | the vocabulary of §1.14 (roles, wiring, the core's four kinds and their settings, presets, hold / toggle / preset controls, `autonOnly`, `draft`), the templates as data, the typed handles (Q18 (a)), every MECH core C++ refusal mirrored (the agreement test grows), the mechanism edits as tool commands with their effects list; robot two's four mechanism/control regions adopted with the differential against the core's tree; D5's mechanism pin and the "only robot two declares mechanisms" pin (`mechanism_table.hpp` :624-625, re-check #7), if the core left them; a new robot's mechanism table, BUILT through the gate (S34); Calibrate's watch of the declared mechanism ports, if the core left it (re-check #10) | the MECH core lands; §1.14 re-checked against its brief; Part 1 | **~3½–5** (+~½ if the core leaves the declared-port watch to Part 5) | ~5 |
| **6 — the mechanism pages** | mechanisms | the MECHANISMS page (add from a template / remove with its effects / duplicate / reorder / rename; every setting with the reason for any refusal), the WIRING page, the CONTROLS page, and the calibration page's warning for a role on a port the record drives | Parts 3 and 5 | **~3–4** | ~4 |
| **Parts 1–6** | | | | **~16¾–22¾** (+~½ as Part 5 says; +1–1½ if §5 Q22 is (b), X-drive generation, in Part 2) | |
| **7 — the extension for the rest** | the rest, after the rest of MECH, before G3 | the settings and controls the rest adds: Catapult's cycle, Conveyor's counting, jam reverse, interlocks, loaded gains, caps and heat, the tip-protection height, preset cycling, sticks, long-press and double-tap, macros, the partner controller, guided binding's "which port moved" written into the robot file | MECH's rest | ~2–3½ (MECH-PANEL §7.2; its own brief re-sizes it) | — |

**How each size was reached** (re-sized at the re-check, 2026-09-29, #4: the draft's total "~15½–21" mis-added
its own table, whose high ends summed to 21½; the re-check's moves then add ~1¼ at each end):
- **Part 1.** Old Part A (3–3½) built the whole tool with five regions. Part 1 keeps the frame, the rules, the
  chassis emitter and the gate. It leaves the four mechanism/control emitters and their fixtures to Part 5
  (about a third of the ~2 000–2 400-line tool), and adds D7's flip with its re-pins (~¼–½).
- **Part 2.** The requirements' §7.1 priced new-robot creation at 1½–2½. Add ~½ for the gate's self-test case
  that builds a generated robot and for the tests' robot list. The re-check takes out the starter driver file
  (−¼) and adds the front's words at Calibrate and the Makefile-read guards (+½): ~2¼–3¼.
- **Part 3.** Old Part B (2½) built the form and the record view. Add the new-robot page with its dry run, a
  drivetrain page split from the form, and the Save flow (~½–1).
- **Part 4.** New with D8 (a), and not in §7.1. It was priced at 1½–2½: two selections, a starter auton file, a
  step-0 rule with its self-test plants, the guards for three programs, and three more builds in the gate's
  self-test. The re-check adds the starter driver file (+½) and the doc gate's bound listings (+½, with five
  self-test cases and two mutations): ~2½–3½.
- **Part 5.** §7.1's "the whole mechanism vocabulary with every refusal the C++ checks make and the agreement
  test ~3–5", plus ~½ for the templates as data and the re-adoption. If the core leaves the declared-port watch
  to it, add ~½.
- **Part 6.** §7.1's panel growth (3–5), minus the new-robot page, which moved to Part 3.
- **The total** is above §7.1's ~13–18½ by Part 4 (~2½–3½) and by five more verification and documentation
  passes. Pace rule 7 keeps each part to about five documents. **Part 1 is ~6**, one past it, and §3 says why.

*[Re-check 2026-10-29: SUPERSEDED — the foundation starts when MECH-A5 lands (the team lead, 2026-10-29); the re-check's "The order and the lanes".]* **The order.** The MECH core follows T4 Part 3 (MECH-PANEL §7.2); its library half runs alongside T4 Part 3, and
its `src/` half after it (Q25).
- **Parts 1 → 2 → 3 run beside the core's `src/` half, strictly in that order.** One groundwork part is in
  flight at a time, in its own worktree, and each starts when the one before it lands (re-check #5: the draft
  let Part 3 start after Part 1, which one-in-flight forbids while Part 2 is unbuilt). The file split with the
  core is in §5.
- **Parts 4 and 5 start after the core lands**, in either order: Part 4 touches the programs, the driver files
  and the doc gate; Part 5 touches the tables. Part 6 follows Part 5.
- **Part 7 comes after the rest of MECH, before G3**, so F7 freezes the whole vocabulary once.

### What new-robot creation writes — every file, in marked regions only (D2, D9)

`robot_tool.py new-robot` writes everything or nothing. `--dry-run --json` lists exactly these files, and the
panel shows that list before Create.

**Part 2 — the bring-up build (programs `tester`, `calibrate`):**
1. **`src/robots/<id>.robot.json`** — NEW: the robot file. It holds the identity; the drivetrain kind,
   cartridge ~~and front~~ *(re-check 2026-10-29: no front, RC2)*, with ports and IMU optional (D10); `build: {"programs": [...]}`; and the provenance.
2. **`src/build_selection.hpp`** — five SET regions:
   - `[robot guards]`: the robot-define count guard and each program's "needs a robot that lists it" guard,
     whole blocks that name every robot's define, the hand-written robots' read from the Makefile (D9);
   - `[robot ids]`: the `RobotId` enumerator;
   - `[robot list]`: `kEveryRobotId`, which the host tests iterate;
   - `[chain]`: one branch per program the robot lists;
   - `[robot beacons]`: `robotBeacon()`'s case.
3. **`Makefile`** — the SET region `[make robots]`: `SHULIB_GENERATED_ROBOTS`, and the robot's define and
   program list, which the hand-written `ROBOT` and `PROGRAM` blocks read (the hand-written robots' own
   define and program lines sit outside it, in the same shape, D9).
4. **`src/robots/chassis_table.hpp`** — the SET regions `[chassis tables]` (`k<Id>Chassis`~~, with `.front`
   from `drivetrain.front`, re-check #3~~ *(re-check 2026-10-29: no `.front`, RC2/RC3; from Part S, `.camera` and `.wallSensors`*) and `[chassis selection]` (`chassisTableFor()`'s case). This is one
   of the two `TABLE_FILES`; a table in any third file fails step 0 (P7).
5. **`src/robots/mechanism_table.hpp`** — the SET region `[mechanism selection]` (`mechanismTableFor()`'s
   case). ~~It returns `kNoMechanisms` until Part 5 gives the robot a table.~~ *(re-check 2026-10-29: Part 5 now lands BEFORE Part 2, so the case returns the robot's own table from creation, generated into `[mechanism tables]` — RC1, CAL1 R11; the two "groundwork touches" of core files below are moot, A5 lands first — RC19)* This is one of the TWO groundwork
   touches of files the MECH core rewrites; the other is the host test loop in `test/mechanism_table_test.cpp`
   (§5, re-check #6).
6. **`tools/src_build_gate.py`** — the SET region `[gate builds]`: the robot's rows (build name, make
   arguments, defines, variant beacon, `RobotId`, program) and its robot beacon. Hand-written lines merge them
   into `VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`, `VARIANT_BEACONS`, `BUILD_ROBOT`, `ROBOT_BEACONS` and
   `BUILD_PROGRAM`.
*(Part 2 writes nothing under `src/team/`. Item 7 was Part 2's; it moved to Part 4 at the re-check, #2.)*

**Part 4 — the match builds (`drive`, `autontest`, `competition`) and the starter files add:**

7. **`src/team/<id>_driver.hpp`** — a NEW STARTER, written once and never overwritten: *(re-check 2026-10-29: in A4/A5's vocabulary — `controls(k<Id>Mechanisms, …)` and `using <Camel>Driver = DriverFile<…>`; the span spelling below is moot, an empty `controls(list)` compiles — RC11, V2)* the class, THE FEEL (the
   library's defaults; *(amended 2026-10-29 evening: a generated `feel` region, D11)*), the loop and the checks, in the controls vocabulary the MECH core leaves (its bind call
   and its lists, presets included, are the core's). Its `[control words]` and `[controls]` regions are
   generated; they hold EMPTY lists, spelled as spans (`std::span<const …>{}`, N4), until the robot has
   mechanisms. Everything outside the markers is the team's. `new-robot` writes it when the robot's programs
   list `drive` or `competition`; `generate` writes it for a robot created before Part 4 when its
   `build.programs` first lists one, and never overwrites an existing file.
8. **`src/team/<id>_autons.cpp`** — a NEW STARTER: `register<Id>Autons()` with an empty list (the students'
   file, G4). Written by the same rule, for `autontest` or `competition`.
9. **`src/team/drivers.hpp`** (hand-written in Part 4) — its SET region `[driver files]`: one `#include` per
   generated robot's driver file. Each starter driver file carries its own `DriverFor<RobotId::<Id>>`
   specialization.
10. **`src/team/autons.hpp`** — the SET region `[auton selection]`: `autonsFor()`'s case per generated robot.
11. **The Part 2 regions again:** `[robot guards]`, `[chain]`, `[make robots]` and `[gate builds]` gain the
    three match programs.

**Part 5 — the mechanisms add:**

12. **`src/robots/mechanism_table.hpp`** — the SET region `[mechanism tables]`: the robot's table, binding and
    polarity in the MECH core's shape. Its `[mechanism selection]` case then returns that table.
13. ~~**The MECH core's handles file** — a SET region of typed handles (Q18 (a)). Its place and spelling are the
    core's.~~ *(re-check 2026-10-29: there is none — the declarations ARE the handles, MECH §6.3; RC4.)*
14. **The driver file's `[controls]`** — its lists fill.

**Never written by new-robot:** a host test (the tests iterate `kEveryRobotId`), a document,
`test/CMakeLists.txt`, `.github/workflows/ci.yml`, `tools/api_doc_tool.py`, anything under `include/shulib/`.
(Part 4's HAND edit of `tools/api_doc_tool.py` is what keeps a starter driver file from weakening the doc gate,
re-check #1.)

### The amendment's probes — `.claude/coordinator/scratch/brief-G3a-amend/` (evidence, never project code)

*(re-check 2026-10-29: N4 is moot in the new vocabulary (V2); N5's message becomes "'claw' was not declared in this scope" (MECH §6.3); N2's and N3's gate counts are `ee2e399`'s — 6 hand-written builds then, 7 now.)*

| # | What | Result |
|---|---|---|
| **N1** | **Robot two's front** (`front_probe.py`, `front_probe.log`; the re-check prototype's emitter, record decoder and comparison) | **As typed (the 15/16 end): 0 of the 2026-09-25 record's 8 ports agree by side and sign. Flipped (the camera end): 8 of 8.** Ports 14 and 19 are LEFT OUT either way. Flipping twice restores the sides and the signs; the file came back byte for byte only because the probe re-supplied the old provenance and front words by hand (re-check #13). The chassis region's `.left`, `.right` and `.provenance` lines change. This run REPLACED the provenance; the fact-keeping provenance D7 specifies is probed in `recheck2/n1/` |
| **N2** | **A second robot's build** (`newrobot_apply.py` on `tree_new/`: the tree at `ee2e399` plus `probe_bot`, INVENTED values — tank, GREEN, no ports, no mechanisms, programs tester + calibrate — written the way §1.13 specifies, SET regions included) | **The real `src_build_gate.py check` PASSES: all 8 builds link** (`gate_new.log`, 2 m 25 s: "[chain-table] 8 builds, 4 robots"; "8 per-robot names"). `minisuite.sh` (the eight host TUs that read the selection, the tables and the driver files, built with the suite's flags, plus one TU per driver file): **101/101 cases, 6 827 assertions** (6 816 on the untouched tree; the 11 extra are the probe robot inside the per-robot loops). The probe rewrote two of the three hand-typed robot lists (`build_selection_test.cpp`'s and `mechanism_table_test.cpp`'s loop); `chassis_table_test.cpp` :548-570 was untouched, and its `kEveryRobotId` case is probed at the re-check (`recheck2/everyrobot/`). The starter driver file compiles in TODAY's controls vocabulary. It moved to Part 4 at the re-check, since the core changes that vocabulary |
| **N3** | **The SET regions EMPTY** (`tree_empty/`) | The gate **PASSES, 6 builds** (`gate_empty.log`, 2 m 03 s). minisuite 101/101, 6 816 |
| **N4** | **Zero controls** (`zero/zero.cpp`) | `static constexpr HoldControl kHolds[] = {};` is an ERROR on the host (`zero-size array`, under -Wpedantic -Werror) AND in the ARM build (`could not convert … to std::span`), as `driver_controls.hpp` :110 warns. The starter's `std::span<const HoldControl> kHolds{}` compiles in both (N2) |
| **N5** | **Remove versus autons** (`handles/`; the handle's real spelling is the MECH core's) | With the handle's line gone, GCC: `In function 'void fortyInches(Routine&)': 'claw' is not a member of 'mech'`, and the same for `skillsRun`, each with file:line. A grep for the handle finds the same two lines |
| **N6** | **Step 0 with a generated robot** (`step0.py` on `mut/`) | (a) the gate's generated rows removed → 4 problems naming both beacons and the robot; (b) the starter driver file naming `kRobotTwoChassis` outside a `static_assert` → named; (d) the chain's calibrate branch removed → "0 branches of THE ONE CHAIN carry its beacon". The derived per-robot set is the tables and the two selection functions only — no driver class (D8's step-0 rule) |
| **N7** | **The Makefile** (`make -n` on `tree_new/`) | `ROBOT=probe_bot PROGRAM=drive` and `PROGRAM=library` → the existing `$(error)`s. `ROBOT=bogus` lists `probe_bot` among the valid values. `ROBOT=bench PROGRAM=calibrate` is still refused, but its message still says "needs ROBOT=tank", so Part 2 rewords it |
| **N8** | **The gate's own self-test with a generated robot** (`gate_selftest_new.log`) | **`src_build_gate.py self-test` passes on `tree_new/`: "OK (190 detector cases, each against a real build)"**, in 9 min. The merged rows break none of its plants. None of its cases builds the generated robot, which is why Part 2 adds one |

---

## Re-check 2026-09-29 — amendments

A checker returned 13 findings on the amendment (its probes are in `brief-G3a-amend/check/`). **Each was
re-verified by hand, and all 13 are real.** Twelve are fixed at their root in the sections named below. One
(#12) needs files this brief does not own, so it is recorded here and in §5 as the coordinator's to do.
**Continuation note:** a first revise run was stopped at the usage limit (~13:50) before it edited this brief.
It left one probe, `docscan/` (finding #1). That probe was re-run, and every case's verdict lines came back
identical (`docscan/bound_scan.rerun.log`), so it is used below. The dispositions were then written fresh.

**The re-check's probes** (`.claude/coordinator/scratch/brief-G3a-amend/`; evidence, never project code):

| Probe | What | Result |
|---|---|---|
| `docscan/` (#1) | `bound_scan.py`, a proposed check-examples rule (a whole-run listing NAMES its file), against today's rule, in seven copies of the tree: base, marked, marked + a starter driver file, a stale feel marked, a stale feel marked + starter, a stale feel unmarked + starter, and a directive naming a program file | Today's rule: the stale feel with a starter present PASSES (rc 0, "2 listings of whole-run files", the checker's `wr_with`). Proposed: base fails 3 (cookbook 06's blocks 5–7, each with a hint naming the owner); marked passes (3 whole-run listings); both stale cases FAIL naming block 6 whether or not a starter is present; the bad directive is refused |
| `recheck2/front/` (#3) | `ChassisTable::front`, `.front` in robot two's and the probe robot's tables, Calibrate's two prints, then its `static_assert`, on a copy of `tree_new/` | Both compilers refuse an omitted designated field without a default (`-Werror=missing-field-initializers`) and accept one with `= nullptr` (`desig*.cpp`). **The real gate PASSES all 8 builds, twice** (`gate_front.log`: the field and the prints; `gate_front2.log`: with the assert). Host mini-suite 101/101, 6 827. The assert, UNCONDITIONED, failed the probe robot's Bench Tests build too, because `calibrate_program.cpp` is compiled into every build (`sa_probe.log`). Conditioned on `kProgram == Program::Calibrate`: both Calibrate builds and the bench build build, the probe robot's Calibrate without `.front` FAILS naming the rule, and its Bench Tests still builds (`sa_probe2.log`) |
| `recheck2/guards/` (#9) | The Makefile states the hand-written robots in the generated shape, its `ROBOT` block reads them, and a `[robot guards]` emitter reads the Makefile (`guards_probe.py`) | The emitted guards equal the regions N2 and N3 built with, line for line, in both trees. **The real gate PASSES all 8 builds** with the `ROBOT` block reading the variables (`gate_guards.log`, "defines token-exact") |
| `recheck2/n1/` (#13) | D7's flip with the provenance KEPT (`front_probe2.py`), and the flipped region compiled in a copy of the tree (`compile.sh`) | 8 of 8 agree. All seven kept or added facts are present. The provenance is 200 characters and fits the banners' 220-byte line; a first wording of 210 would have been cut. Flipping twice restores the sides and signs whatever words the caller supplies. The region compiles on the host with the tank define (robot two's pins and mechanism disjointness evaluated) and in the ARM compiler |
| `recheck2/tip*` (#1, at the COMP-t tip) | `git checkout-index` of the main tree's STAGED COMP-t merge (never touching the tree itself), then `bound_scan.py` | Today's rule: 503 lines, 5 whole-run listings. Unnamed under the bound rule, all five fail (cookbook 06's three, the auton chapter's two). With five directives it passes (5 named, 5 whole-run). A stale feel plus a starter passes today's rule ("4 listings") and fails the bound one |
| `recheck2/crosscheck/` (#8) | The real `countPortDisagreements()` (`src/common/boot_source.hpp`), robot two's 2026-09-25 record against its typed table and against D7's flipped one | 10 as typed (test 6i's pin), **2 flipped** (ports 14 and 19, listed only by the table) |
| `recheck2/everyrobot/` (#13) | `chassis_table_test.cpp` gains a `kEveryRobotId` case beside its literal identity checks, on `tree_new/` | 102/102, 6 859 assertions. Mutation: the generated robot's selection returns `kNoChassis` → exactly that case red (4 failures) |

| # | Finding | Re-verified | Disposition — where it is fixed |
|---|---|---|---|
| 1 | A generated starter driver file silently weakens `check-examples` | **Real.** `known` is the union of every example source (`api_doc_tool.py` :1924-1927), and ANY whole-run file owning a listing's lines makes it a whole-run listing (:1961). A starter repeats robot two's feel and bind/run lines verbatim, so a stale quote of robot two's file survives. `docscan/` reproduces the checker's `wr_with`/`wr_without` and was re-run identically | **Fixed at the root, in Part 4**, the part that now writes the first second driver file (#2), before any generated robot is committed. **A listing of a WHOLE-RUN file names the file it quotes**, with `<!-- example-source: PATH -->` on the line above the fence, and is checked against that file ALONE: every line a line of PATH, and one unbroken run. The whole-run files leave the membership union, so an unnamed listing of driver-file lines fails, with a hint naming the file that holds them. A directive naming a file no `EXAMPLE_SOURCE_GLOBS` entry matches is refused. Cookbook 06 gains three directive lines (its three listings of `src/team/robot_two_driver.hpp`, the file its :213 links), and COMP-t's auton chapter two (COMP-t made `src/team/autons.cpp` a whole-run kind; see "Found while re-checking" below). Probed on the COMP-t tip too (`recheck2/tip*`). **Chosen over "the file the chapter links":** a link is prose the scan would have to parse, and a chapter may link a file it does not quote. A directive is one line per listing, and the scan checks it. §1.5, parts table, §2 Part 4 (S33, M28, M29), §3 Part 4, §5, landmine 24 |
| 2 | Part 2's starter driver file depends on the controls vocabulary the MECH core rewrites, and no bring-up build compiles it | **Real.** The starter binds `bindings_.bind(robot, kHolds, kToggles, …)` over `HoldControl`/`ToggleControl` spans with `controlsNameTable`/`controlsCoverTable` asserts (`newrobot_apply.py` :210-278). The core's draft binds `inputs.bind(svc, kHolds, kToggles, kPresets)` from a new library header (`brief-MECH-core/probe_driver.cpp` :130). A4 and §1.14 already say the core adds presets and relaxes the cover rule. N2 shows the bring-up builds never include it | **Fixed as proposed.** The starter driver file moves to **Part 4**, after the core, together with its regions, its host TU over `src/team/*_driver.hpp`, S23, S27, M20 and landmines 12 and 19. There the programs include every driver file, so the ARM builds compile it from the day it exists. Part 2 writes nothing under `src/team/`; the groundwork now depends on nothing of the mechanism vocabulary. A robot created between Part 2 and Part 4 gets its starter from `generate` when its `build.programs` first lists a match program (never overwriting a file). D8, parts table, "What new-robot creation writes", §1.2, §1.13, §2, §3, landmines 12 and 19, the plain words |
| 3 | A new robot's human-stated front reaches nothing and is checked by nothing when its ports are untyped | **Real.** `ChassisTable` has no front (:66-87). Calibrate takes the front from the on-screen FRONT/BACK answer (:100) and prints only `kChassis.robot` (:1086). FRONT REVERSED compares port SETS (landmine 10) | **Fixed, both halves.** (a) **The words reach the moment the front is decided** (Part 2): `ChassisTable` gains `const char* front = nullptr;` (the default keeps every hand-written and test-built table unchanged). The chassis emitter writes `.front`, so robot two's region gains that one line, regenerated after Part 1. Calibrate prints `front      : <words>` and, before the push, "LEAD THE PUSH WITH THE FRONT THE ROBOT FILE STATES (…), then answer FRONT FIRST". `static_assert(kProgram != Program::Calibrate \|\| (kChassis.front != nullptr && kChassis.front[0] != '\0'), …)` refuses a Calibrate build of a robot that states no front. `drivetrain.front` becomes literal-bound (printable ASCII, ≤ 160 characters, so both Calibrate lines fit its 240-byte log line). Probed: `recheck2/front/`. (b) **The limit, stated:** the screen's WHICH END FIRST? is the library's panel, and G3a makes no `include/` change, so the words reach the serial log only (§8). Nothing can CHECK a portless file's front against a record. The calibration page says so. D10's costs, §1.1, §1.7, §1.13, §2 (S35, M30), landmines 21 and 25, §8, the plain words |
| 4 | The size total is understated, and the documents-per-part claim contradicts the table | **Real.** The high ends summed to 21½, not 21. Part 1 listed ~7 documents against "each part stays at about five" | **Fixed.** Re-sized after this re-check's moves: Parts 1–6 **~16¾–22¾** (+~½ if Part 5 owns the declared-port watch; +1–1½ with X-drive generation). Part 1's FAQ entry moves to Part 3, which touches the FAQ anyway. Part 1 is then **~6 public documents, one past rule 7, and §3 says why:** the tool cannot ship without its chapter (rule 3), and the README's layout, the glossary and §3.1's drivetrain half would otherwise be false or incomplete. The parts table, A18, A19, "How each size was reached", §3 |
| 5 | Part 3's start contradicts one-in-flight; "the core's own start" contradicts Q25; D6's replacement is not put to the team lead | **Real, all three** (MECH-PANEL :536; NOTES.md :247-248) | **Fixed.** Part 3 starts after Part 2 (1 → 2 → 3, strictly). Part 1 starts "after T4 Part 3 lands, when the core's `src/` half starts; its library half runs alongside T4 Part 3". **D6′** (the seven parts, Part 4 outside the groundwork) is a decision awaiting him, with (a)–(c), and the status line names it. D6′, the parts table, "The order", the status line |
| 6 | Part 2 edits `test/mechanism_table_test.cpp`, which the core rewrites, but §5 claims ONE shared exception | **Real** (`newrobot_apply.py` :204-206 rewrites the loop at :397) | **Fixed.** §5 names it a SECOND shared hunk (one line) with the same rebase rule, and asks the core's brief to iterate `kEveryRobotId` in its rewrite once Part 2 has landed. `src/robots/mechanism_table.hpp` joins "Parts 1–3 write". §5, "What new-robot creation writes" item 5 |
| 7 | `mechanism_table.hpp` :624-625 ("only robot two declares mechanisms") blocks every generated robot's mechanism table | **Real** (read: `kRobotId == RobotId::RobotTwo \|\| &kMechanisms == &kNoMechanisms`; N2 could not see it, since its robot has none) | **Fixed.** §5's list for the core gains item 6: name the hand-written robots that declare none (the bench bot and the invented X-drive, by name, as `chassis_table.hpp` :234-237 does), plus the generic `&kMechanisms == &mechanismTableFor(kRobotId)` (as :224-225). Part 5 is the fallback owner. **S34** (Part 5) builds a generated robot with one mechanism through the gate. §5, parts table, §2 |
| 8 | D7's re-pin list is incomplete | **Real.** :393-394 read the same hand copy. The grep also finds :89-105 and :114-116 on it, plus five present-tense comments (`mechanism_table_test.cpp` :85-86, :558, :561; `boot_check_test.cpp` :42-43; `boot_source_test.cpp` :163). The class-(A) fixtures that never read the table were confirmed by grep | **Fixed at the root:** D7's step now FINDS them by grep, with the patterns and §3's three classes. Today's hits are listed as a floor, not a list to trust. Part 1 adds the CROSS-CHECK sibling case (the flipped table against the 2026-09-25 record: 2 disagreements, 14 and 19). D7, §6 |
| 9 | The `[robot guards]` SET region is not empty with no generated robot, and holds a second copy of the hand-written robots' build facts | **Real** (`tree_empty/src/build_selection.hpp` :69-76; `newrobot_apply.py` :60-61 typed them in Python). Checked while fixing: a helper macro is not an alternative. The gate allows only `#error` inside a switch-reading block (`src_build_gate.py` :138-141), and a `//` marker cannot sit inside a continued `#if` | **Fixed at the root.** The Makefile states the hand-written robots in the generated shape, outside the region, and its own `ROBOT` block reads those lines. The tool reads them to write the guards, and never types a hand-written fact. Probed: `recheck2/guards/`. "Adopted EMPTY" is reworded. §5's rule for a chunk adding a robot-two program: edit the Makefile line, run `generate`; `check` says STALE, never "edited by hand". Landmine 13 gains the continued-`#if` hazard. D9, §1.2, §1.13, §2 (S36, M31), §5, landmine 13 |
| 10 | D10 (a) omits a cost: without typed drive ports, the mechanism-versus-drive rules check nothing at build | **Real**, and a step worse than stated. Calibrate DROPS the declared port (:1129-1131), so the record never holds it, and the Part 6 calibration-page warning cannot see it either | **Fixed as far as G3a reaches:** D10 (a)'s costs and landmine 22 say it. §5 asks the core's brief, which edits that hunk, for a Calibrate WATCH of the declared mechanism ports during the push: read, never commanded, and a line naming any that turned with the drive. Part 5 is the fallback owner (+~½). The hazard begins at Part 5, when a generated robot can first have mechanisms. D10, §5, parts table, landmine 22 |
| 11 | S22 lacks the robot-define collision, the only collision for ids like `tank_2026` | **Real** (by derivation: `tank_2026` differs from robot two in make value, RobotId, tables, class and beacons, and collides only in `SHULIB_ROBOT_TANK_2026`; `xdrive_invented` likewise). With the Makefile-read guards, the collision would also make the count guard fail every `ROBOT=tank` build with a misleading "two robot defines" | **Fixed.** S22 gains `tank_2026` and `xdrive_invented`; **M27** drops the define check → S22 red. §1.13, §2, "Decided here" |
| 12 | The HA-181 reservation is recorded only in this brief, while the briefing calls it next free | **Real.** The briefing's "next free" is the register's highest table row + 1 (`tools/briefing_status.py` :111-117). The register records held numbers only in prose (`docs/hardware-assumptions.md` :84, "HA-162..HA-178 … never reissued"). COMP's brief takes HA-162, not HA-181 (`COMP-competition-program.md` :740), but any other brief reading the briefing would take HA-181 | **Recorded here and in §5; NOT fixable inside this file** (the brief may edit only itself). The coordinator: adds HA-181 to the register header's held-numbers sentence; makes the briefing's next free skip held numbers (a `briefing_status.py` change, its own small chunk), or states it by hand until then; and tells the MECH core's writer and every brief written before G3a lands that the first free number is **HA-182**. §5, the status line |
| 13 | N1's and N2's claims are stronger than the probes show | **Real, all three** | **Fixed and re-probed.** (a) `chassis_table_test.cpp` :548-570 keeps its literal identity checks and gains a `kEveryRobotId` case (probed, with its mutation red); N2's row and §0.7 say which lists the probe edited. (b) D7's fact-keeping provenance is now probed (`recheck2/n1/`). That exposed a limit the tool now enforces: `drivetrain.provenance` ≤ 206 characters, the banners' 220-byte `provenance : %s` line (S13's new case, M32). The kept facts are checked on robot two's REAL flip, in Part 1's logged step and §6, since free text cannot be checked for facts in general. (c) N1's flip-twice claim is reworded, and S19 no longer claims byte-for-byte. D7, N1/N2 rows, §0.7, §1.1, §1.2, §1.13, §2, §6, landmine 23 |

**Found while re-checking (not a finding; recorded so nothing is silent).** During this re-check the
coordinator merged COMP-t into the main tree. It was STAGED over `a5bd83f` and not yet committed; nothing
here touched it, and the probes read it through `git checkout-index` into scratch. It changes three things
this brief relies on:
- **Guide chapter 16 is COMP-t's** (`docs/guide/16-testing-an-auton-on-the-bench.md`), and 17 is COMP-b's
  (COMP brief :403). G3a's robot-file chapter takes the next free number at Part 1's tip: 18 once COMP-b has
  landed. Every "chapter 16" in this brief means that chapter (§3 says so at the file name).
- **`src/team/autons.cpp` is a whole-run kind**, so finding #1's fix covers it: five directives at the tip,
  not three (§1.5).
- **The line numbers cited in `tools/api_doc_tool.py`** (:1831, :1853, :1924-1966) are `ee2e399`'s; at the
  tip, the globs are at :1836-1837 and :1861. Part 4 re-reads the file, as every part re-reads its files at
  its tip.

The register still leaves HA-181 unclaimed: COMP-t took its held HA-177 and HA-178, and the briefing still
says "next free is HA-181", which is finding #12's point.

---

## Re-check 2026-09-27 — amendments

A checker returned 15 findings (its probes are in `check/`). **Each was re-run by hand; all 15 are
real.** Each one is fixed at its root in the section named in the last column. Two fixes differ from
what the checker proposed, and the table says why. Nothing here changes what D1–D4 recommend. D5 and D6
are rewritten with the corrected evidence, and the rulings that came out of the findings are listed
under "Decided here".

| # | Finding | Re-verified | Disposition — where it is fixed |
|---|---|---|---|
| 1 | Text fields can inject C++ (a newline, a marker, an unterminated literal) | **Real, and worse.** The checker's three cases reproduce (`recheck/f1/`). **One more:** a comment line ending in `\` passes validate, generate AND check, yet it comments out `inline constexpr ChassisTable kRobotTwoChassis = {`, and the host compile fails `-Werror=comment`. A trigraph (`??/`) fails `-Werror=trigraphs` | **Fixed.** The TEXT-FIELD RULE for every known text field: no control character (C0, DEL, C1 including NEL, U+2028/U+2029), no marker token, no backslash before whitespace or at the end, no trigraph. Literal-bound fields must also be printable ASCII. The emitters guard (a `ToolFault` writes nothing), `cstr()` escapes defensively, and generate re-checks the new text in memory before it writes (the generate⇒check invariant). Battery: 9 hostile cases each refused by JSON path, and a benign `*/ /* " \` comment compiles under the suite's flags. §1.1, §1.2, §1.6, §2 S13–S14, §7 landmine 11 |
| 2 | The ruling is not enforced for `typedTable`; the plain words are false | **Real** | **Fixed; one half changed from the proposal.** (a) is taken: `typedTable` is refused on every robot file but `robot_two`, the grandfathered block, named with its readers. Robot two's wheel diameter may never change (the baseline rule, #5). **Not taken:** "refuse ANY change to robot two's typed values". Moving a drive port forces new signs: the validator requires the signs to cover exactly the drive ports, and the pin requires `signsMeasured`. A frozen block would therefore deadlock. So a sign may change only together with a changed `drivetrain.provenance` (#5's rule), and the host pins stay a second witness. (b) cannot meet the DoD: the signs sit in the same `.left`/`.right` lines as the human-stated ports. Plain words corrected. §1.1, §1.2, plain words, "Decided here" |
| 3 | A controls edit fails `check-examples` (cookbook 06) | **Real** (`check/p4`: rc 1, `NOT VERBATIM ... block 5`; adopted tree rc 0). By design: `api_doc_tool.py` :1833-1853 | **Fixed.** Said in D5 and §1.9. `generate`, and the panel's save message, list the WITNESSES they find: every `test/` file naming the robot's tables, and every public document quoting a changed line. The scan finds cookbook 06 and six test TUs. Chapter 16 says it. Keeping cookbook 06 as a third witness is D5 (a) (§3) |
| 4 | P4's host numbers are wrong | **Real** (rebuilt from scratch, `recheck/p4`): 98 cases. The first crash ends the run (79 run, 14 fail). With the three crashing cases excluded, 24 more fail (93 assertions): **27/98, three by SIGSEGV** (`robot_two_driver_test.cpp` T8 :629; `teleop_claws_test.cpp` 1g :479, 1h :546 — each dereferences a binding or mechanism that the rig's unplugged port 8 leaves null, without a REQUIRE) | **Fixed.** P4 row, D5, §8 |
| 5 | "Provenance required when a value changes" could only be enforced by the panel | **Real** (only non-empty is checked) | **Fixed, in the TOOL, as the checker's option 1, with a better baseline.** The baseline is the tree's own regions, whose sha proves they are exactly what the robot's C++ carries now. No git and no `--since` are needed, so it also works for a PKG1 team without git. A value line that changed while its provenance lines did not is refused by `validate`/`generate`, never by `check`. The panel only shows the refusal. Battery: P4 with the same provenance → refused; with new provenance → generated. The same holds for an `openLevel` flip, a sign flip, and robot two's wheel. §1.2, §1.7, §2 S17 |
| 6 | Derived C++ names can collide | **Real**, and wider: an id `bench_bot` or `no` collides with the HAND-WRITTEN `kBenchBotChassis`/`kNoChassis`/`kNoMechanisms` | **Fixed.** The tool refuses: two polarity constants with one name, a pneumatic name with no letter or digit, and any derived name already defined outside this robot's regions (another robot's, or a hand-written one). Battery: 3 cases. §1.2, §7 landmine 5 |
| 7 | The harness never evaluates the selection pin | **Real** (`agree.py` :133 drops `static_assert*`) | **Fixed at the root.** D5 (a) rewrites the pin anyway, so its non-address half becomes ONE named `constexpr` predicate that both the `static_assert` and the agreement test call. Probed: fixture 24 agrees; with the validator's pin removed it DISAGREES. §1.4, §1.6 |
| 8 | Robot two's facts restated outside the regions, missing from §3 | **Real, and longer than listed.** A grep also finds `teleop_program.hpp` :17, :20-22; `teleop_program.cpp` :30-37, :149 (a RUNTIME string, quoted by the worksheet :1322), :167; `calibrate_program.cpp` :86-88 | **Fixed.** §3's prose pass is now a set of grep patterns plus a rule that sorts each hit into one of three classes, with today's hits listed |
| 9 | `@vscode/vsce` 4.0.0 needs Node ≥ 22; the probe's repository URL was invented | **Real.** Also, Node 20 reached end of life on 2026-04-30 (the Node.js release schedule) | **Fixed.** Pin `@vscode/vsce` **3.9.2** (engines `node >= 20`; probed on the Node 20 installed here: 5/5, packaged). CI runs **Node 22** (maintained, within 3.9.2's engines). The repository is `https://github.com/SHU-ROBOTICS/shulib` (`git remote`, `mkdocs.yml` :18). §1.7, §1.8 |
| 10 | No program prints a record's CRC | **Real** (grep: none) | **Fixed.** The panel prints the record in the SAME form as the boot's `[SES] ports` line (`robot::writePortMap()`, `include/shulib/robot/robot.hpp` ~:448: `MEASURED L … R … IMU2 ipr %.4f/%.4f TW%.2f`), and the record agreement test pins it against that function. The CRC is shown only as the file's identity. §1.7, §1.12 |
| 11 | A correct region with a wrong sha is called a hand edit | **Real** (reproduced) | **Fixed.** Lines equal to what the file generates count as fresh: `check` fails with "(marker only)", and `generate` rewrites the marker alone. Battery: check 1 → generate 0 → check 0. The converse case (a hand edit with a recomputed sha is overwritten as stale) is named in chapter 16, with the procedure after a merge. §1.2, §2 S15, §3 |
| 12 | D6 omits the queue ahead of T4 1b | **Real** | **Fixed.** D6 states the chain (TANK1 → COMP-t → COMP-b → T4 1b → Part A) and the pre-1b option with its real cost: T4 1b's brief would have to go through the generator. It also says that Part B needs only Part A |
| 13 | The tool's size is optimistic; commands and CMake steps are unlisted | **Real.** The prototype is now ~1 330 lines, still without self-test, describe, `--json` or stdin | **Fixed.** The tool is re-estimated at **~2 000–2 400 lines**; Part A **~3–3½ days**, Part B **~2½ days**. `agreement-fixtures`, `record-fixtures` and `flip-front` are listed, and so is Part B's `test/CMakeLists.txt` step. D6, §1.2, §1.8 |
| 14 | "G3 embeds it unchanged" holds in one direction only | **Real** (§6 "Missing fields get safe defaults"; VexBuilder's `robotProfile` is ❌, so a rename is free now but must be named) | **Fixed.** §1.11 marks each field as VexBuilder-emitted or shulib-only, says what G3 does when a shulib-only field is missing, and names every rename and re-meaning. D1 is reworded |
| 15 | Minor gaps | **All real** | 1. The polarity region is ALWAYS present, empty without a pneumatic (probed: generate, check and compile are clean). 2. P9 says the panel↔tool contract is unprobed; the vocabularies come from `describe`, refusals by JSON path. 3. DEPENDS is a `CONFIGURE_DEPENDS` glob over `src/`. 4. **`flip-front`** is the tool's atomic "the other end is the front" (swap the sides, negate the typed signs, require provenance); the panel's button calls it. The limit is named: only the record check (FRONT REVERSED) can verify chirality. 5. The worksheet is named: `docs/internal/chunks/R3a-BENCH-WORKSHEET.md`. 6. Unknown keys are warned at EVERY level (battery). §1.2, §1.5, §1.7, P9 |

---

## Decisions for the team lead (a recommendation for each; nothing below decides them silently)

> **All six ACCEPTED as recommended, 2026-09-29.** How the amendment re-shapes each one (the text below is
> kept as it was accepted):
> - **D1** holds: the file also carries `build` on a generated robot, and `wiring`, `presets` and the kinds
>   (§1.14).
> - **D2** holds and grows. The regions are still marked, but the build plumbing's regions are SET regions
>   (D9), two of them outside C++ (the `Makefile` and `tools/src_build_gate.py`, with `#` markers). Robot two's
>   four mechanism/control regions are adopted in Part 5, in the MECH core's shape. `TABLE_FILES` stays
>   unchanged (probed N2).
> - **D3** holds: the templates, the mechanism edits and their effects list are the TOOL's; the panel runs it.
> - **D4** holds: a starter driver file's FEEL, re-zero included, is the team's code, and the tool reads the
>   re-zero from it. *(Re-check 2026-10-29: still holds — MECH's `[driver file two]` block would put the feel inside a region; G3a wraps only the controls, RC5.)* *(amended 2026-10-29 evening: NO LONGER HOLDS — the team lead REVERSED D4 (a): the feel and the loop, re-zero included, become robot-file data (`driving`) generated into a `feel` region; D11)*
> - **D5 (a)** splits: the chassis pin in Part 1, the mechanism pin with the MECH core or Part 5 (A6). *(Re-check 2026-10-29: the mechanism half is DONE by MECH-A5, RC19.)*
> - **D6** is SUPERSEDED by the amended parts table: seven parts, each its own go, after T4 Part 3 rather than
>   Part 1b. The replacement is put to him as **D6′** (re-check 2026-09-29, #5), since it undoes a decision he
>   accepted.

**D1 — The robot file: its format, its name, its place. RECOMMENDED (a): JSON, one file per robot, at
`src/robots/<id>.robot.json` (robot two: `src/robots/robot_two.robot.json`), whose top-level object IS
the profile block.**
- **(a) RECOMMENDED.** `src/robots/` is SRC1's folder for "what each robot IS" (`src/team/README.md`
  says what a robot is "lives in `src/robots/`"). The file is the `robotProfile` object itself, with
  `schemaVersion` inside it. A robot file therefore goes into a `.vexbot` as `project.robotProfile`
  **unchanged**, and the generator reads the same object from either place. *(Re-check #14: the other
  direction is not "unchanged". A profile VexBuilder emits lacks the shulib-only fields; G3 fills each
  of them by a named rule or refuses it by name, as §1.11 lists.)* JSON because `.vexbot` is JSON (the block must
  be JSON there anyway) and Python's standard library reads and writes it — no dependency. It has no
  comments, so the facts' story travels in `provenance`/`notes` fields (today's C++ comments move into
  them: §1.1). Robot two's file is 196 lines (§1.1).
- (b) YAML or TOML (they have comments). **Cost:** YAML needs PyYAML (not in the standard library); TOML
  is read-only in Python's `tomllib`, so the panel could not save through the one writer; and G3 would
  translate it into JSON for `.vexbot`, so "embeds unchanged" stops being true.
- (c) Under `src/team/` (the first drafting run's choice: the students' folder). **Cost:** contradicts
  `src/team/README.md`'s own rule; the controls it carries are generated INTO `src/team/`, not stored there.
- (d) A top-level `robots/` folder. **Cost:** outside `src/`, where PKG1's starter project (the last
  chunk) will not carry it.

**D2 — Where the generated C++ goes. RECOMMENDED (a): MARKED REGIONS inside the three files that hold the
facts today; everything else in those files stays hand-written.**
- **(a) RECOMMENDED.** Five regions (§1.3): robot two's block in `src/robots/chassis_table.hpp`
  (:89-140), the claw's polarity constant (:171-183) and robot two's block (:245-310) in
  `src/robots/mechanism_table.hpp`, and in `src/team/robot_two_driver.hpp` the header's button words
  (:22-29) and WHICH BUTTON's `kHolds`/`kToggles` (:65-71). The helpers, the checks, the selection
  functions (`chassisTableFor()` :203, `mechanismTableFor()` :321) and the pins stay hand-written.
  **The src gate's step 0 needs no change to `TABLE_FILES`** (`tools/src_build_gate.py` :398-401): every
  per-robot table is still defined in one of the two table files, one selection function per file —
  G1 Part 2c final's constraint (build-order ~:1606-1608) holds by construction. **Probed:** the full
  gate PASSES on the adopted tree, all six builds (P2).
- (b) A generated per-robot header (`src/robots/robot_two_tables.hpp`) that the table files include.
  **Cost, probed (P7):** step 0 FAILS on it today — `_stray_selection_problems` (:652-677) reports
  `src/robots/robot_two_tables.hpp:35 names kRobotTwoChassis, a per-robot table`, because a table the
  selection function returns is defined outside `TABLE_FILES` — exactly the build-order's warning
  (~:1583). `TABLE_FILES` would have to become a derived set with new self-test cases, and the driver
  file needs a region anyway (T4's R1).
- (c) Generate the whole table files. **Cost:** the helpers and the nine checks would live in the
  generator's templates — two sources of C++ truth. Rejected.

**D3 — The generator's language. RECOMMENDED (a): Python, `tools/robot_tool.py`, standard library only;
the panel runs it as a process — ONE validator, ONE generator, ONE record decoder.**
- **(a) RECOMMENDED.** Every gate in the repo is Python; the host build already REQUIRES python3
  (`test/CMakeLists.txt` :165-177) and CI installs it (`.github/workflows/ci.yml` :33-34). A validate runs
  in ~20 ms (P9), fast enough for live warnings. **Cost to a team without Python:** they cannot run the
  generator or its gate; `make` still builds (it never runs Python, §1.10). PROS's own CLI is a Python
  program, but whether a PROS install puts `python3` on a Windows student's PATH is **UNVERIFIED** — the
  panel has a `shulib.python` setting for that.
- (b) TypeScript, shared by the panel and a Node CLI. **Cost:** the freshness gate then needs Node in the
  host build and in CI, and a team without Node could not check its own tree — the opposite of "in
  addition but not needed".
- (c) Both. Two validators; rejected by this chunk's own rule.

**D4 — Which lines the generator owns, and the re-zero button. RECOMMENDED (a): the five regions only;
THE FEEL and THE LOOP stay hand-written, the re-zero with them, and the tool READS the re-zero.**

> *(amended 2026-10-29 evening)* **REVERSED by the team lead (2026-10-29, evening — ruling (1): the driving feel becomes panel data).** D4 (a)
> was accepted 2026-09-29; it is now (b) in substance: THE FEEL (`kFeel`) and THE LOOP (`kLoop`: the re-zero and the
> release brake mode) are the robot file's `driving` block, generated into a `feel` region — never `.tickPeriod`
> (MECH-A5's `DriverFile` fills it) and, unless D11 (c), never `.protection`. The tool no longer READS the re-zero; the
> file STATES it, and the validator checks it against the controls. (a)'s reason — the feel is code a student writes and
> defends — gives way to his ruling; the generated lines stay in the driver file, readable; (b)'s cost (a larger F7
> surface for VexBuilder) is accepted. Where it lands: D11 (Parts 5 and 6 under (a)). The text below is kept as it was
> accepted.

- **(a) RECOMMENDED.** `kLoop.rezeroButton` (`robot_two_driver.hpp` :81) belongs to field-centric, which
  is the feel (and is OFF). The feel is the driver's code a student tunes and must defend (G4's
  guardrail). The tool reads `.rezeroButton = Button::<name>,` (exactly once, or it refuses) so its
  validator names `controlsAvoid` first. The panel SHOWS the re-zero read-only: "change it in THE FEEL".
- (b) The regions also cover THE FEEL (`kFeel`, `kLoop`). **Cost:** every `DriverProfile` field (mode,
  deadband, curve, ramp…) becomes profile schema — a larger F7 surface for VexBuilder — and the feel
  stops being code a student reads.
- (c) The re-zero alone generated as `static constexpr Button kRezero = Button::Y;` in WHICH BUTTON, named
  by `kLoop`. **Cost:** one hand edit of `kLoop`'s line (THE FEEL is no longer byte-for-byte) and cookbook
  06's feel listing re-quoted (:230-237); every button would then live in one place.

**D5 — Robot two's facts are restated OUTSIDE the regions, by hand, and a panel edit trips them.
RECOMMENDED (a): Part A drops the COUNTS from the two selection pins; the host tests' literal pins stay
(this repository's second witness), revisited at PKG1.**
- **What exists.** `chassis_table.hpp` :230-233 pins robot two's table as `signsMeasured` with
  `leftCount == 5 && rightCount == 5`; `mechanism_table.hpp` :619-623 pins `motorCount == 2 &&
  pneumaticCount == 1`. Six host test files read robot two's tables by value (`mechanism_table_test.cpp`
  :230-310 pins R1/R2/L1/L2/A, ports +6/+7, line 1, 12 V; :347-351 counts `.openCommand = ` in the
  table's source).
- **Probed.** Adding a third mechanism in the file: the validator names the pin first. Every
  `ROBOT=tank` build then fails at `mechanism_table.hpp`'s pin, `static assertion failed: ROBOT=tank
  must select robot two's mechanism table (the claw pivots on 6 and 7, the pneumatic claw on 3-wire A)`,
  while the host compiles (P5). Moving claw pivot 7 to port 8 on Up/Down: all six ARM builds PASS, but
  **27 of the 98 host test cases fail, three of them by SIGSEGV** (P4, corrected at the re-check: the
  first crash ends the run, which is why the draft counted only 14 of 79). **And
  `api_doc_tool.py check-examples` fails**, because cookbook 06 quotes robot two's WHICH BUTTON block
  verbatim and the whole-run rule was written for exactly this edit (`api_doc_tool.py` :1833-1853).
  The listing must be re-quoted by hand.
- **(a) RECOMMENDED.** Each pin keeps what makes it a SELECTION check: `&kChassis == &kRobotTwoChassis`,
  `&kMechanisms == &kRobotTwoMechanisms`, and `signsMeasured`, because the old drive program and Bench
  Tests' DRIVE station need signed ports. `signsMeasured` becomes one named predicate that the agreement
  test also calls (§1.4). Each pin drops the counts that a robot file now owns. The address checks
  already catch the defect the counts were written for ("hand robot two an EMPTY table", :613-618).
  **The witnesses stay:** the literal test pins, and cookbook 06's WHICH BUTTON listing (a third
  witness, recommended). In THIS repository every change to robot two is a reviewed change, and
  `generate` and the panel's save message both list the witnesses they find: every `test/` file that
  names the robot's tables, and every public document that quotes a changed line. For robot two today
  that is six test TUs and `docs/cookbook/06-driver-control.md` (probed). PKG1's starter has none.
  *(The alternative for the cookbook: quote a non-robot-two example there. **Cost:** the section is
  "A whole driver file: robot two's" by T4 1a's design, and the rule was written for this edit.)*
- (b) (a), and also turn the literal pins into structural ones now (the binder's by-name mapping checked
  against the generated constants rather than against `R1`). **Cost:** ~½ day, in T3's and T4's
  adversarial tests, several of which T4 Parts 1b/2/3 rewrite anyway — if chosen, do it after T4 Part 3.
- (c) Keep everything. **Cost:** a mechanism added in the panel never builds for the tank until someone
  edits a pin by hand — not "plug and play".

**D6 — The parts, their order and their size. RECOMMENDED (a): two parts, A then B, each its own go; A
starts after T4 Part 1b.**
> *[SUPERSEDED 2026-09-29 by the amended section's parts table: seven parts in three phases; "after T4 Part 1b"
> is now "after T4 Part 3", since Part 1b was folded into Part 3 on 2026-09-28; the queue below is history.
> Old Part A's content is split between Parts 1, 2 and 5; old Part B's between Parts 3 and 6. D6 (a)'s
> principle — separate parts, each its own go, the panel's design learning from the tool — carries over.]*
- **Part A — the robot file, the generator, the gate** (§1.1–§1.6, §1.9–§1.12): **about 3–3½ days**
  (re-estimated at the re-check). It covers:
  - the tool with its self-test, **~2 000–2 400 lines**. The re-check's prototype is ~1 330 lines and
    still lacks the self-test (S1–S20), `describe`, `--json`, stdin, the all-files scan, CRLF-keeping
    writes and `flip-front`. `api_doc_tool.py` is 2 733 lines and `src_build_gate.py` 2 313;
  - the agreement test in the host suite, and the five regions adopted with the differential logged;
  - D5's pins, with the named predicate;
  - the prose pass, which the re-check grew to ~15 source sites (§3), plus the gate and CI steps;
  - ~8 documents. That is past pace rule 7's ~5: the new guide chapter and §3.1's revision are
    unavoidable, and the source headers' prose must not stay false (§3).
- **Part B — the robot panel** (§1.7–§1.8): **about 2½ days.** It covers:
  - the extension: a custom editor over the file with a real form, live diagnostics, save → generate,
    the front button and the record view;
  - the record decoder, its agreement test and that test's CMake step;
  - the model tests, the `.vsix`, a CI job, and ~5 documents.
- **Together about 5½–6 days.** HANDOFF (~:546-547) said "~3–5 days". The difference is the rules the
  re-check added (text fields, the provenance baseline, collisions, the marker repair, `flip-front`,
  the witness list) plus an honest size for the tool.
- **The queue ahead of Part A** (build-order :632, the first bold `Next:`): TANK1 Part A (in flight,
  ~1¼ days by its brief), then COMP-t, then COMP-b, then T4 Part 1b (~1 day, T4 brief :45). Each goes
  on its own go, one in flight at a time. The team lead pushes the branch once the auton is complete,
  before G3a. **Under (a), Part A therefore starts after four more builds.** Part B needs only Part
  A's command line (`validate`, `describe`, `fmt`, `generate`, `record`). It does not need T4 Parts 2
  or 3, so once Part A lands, Part B and T4 Parts 2 and 3 can go in either order. That order is his.
- **Why after T4 Part 1b:** 1b removes the buttons from the mechanism table (T4 brief ~:1002-1015), so
  the table's emitter has ONE shape to target.
- **(c) Part A BEFORE T4 1b, right after COMP-b.** Part A ships the pre-1b emitter (probed: it is
  today's tree; P10 shows 1b later deletes seven emitted lines). **Real cost:** 1b becomes a change
  THROUGH the generator. Its table edit is a hand edit inside a region, which `check` refuses. So T4
  1b's brief must be amended to drop the seven lines in the emitter, move the provenance rewrites into
  the robot file, and regenerate. That is ~½ day of the coordinator's time plus an emitter shape that
  lives for one chunk. The total work is the same, and only the order changes. G3a starts about a day
  sooner, because it skips 1b's ~1 day. 1b, the one owner of the buttons, lands after Part A instead
  of before it, ~3–3½ days later. Choose (c) only if having the panel sooner matters more than having
  one owner of the buttons sooner.
- (b) One run of ~5½–6 days. **Cost:** one documentation pass of ~13 documents; the panel's design cannot
  learn from Part A's tool.

**Decided here, with the evidence (not his; named so nothing is silent):** pneumatics are stated as
POSITIONS plus one polarity, never as levels (§1.1, §1.11); the measured fields leave the human-stated
profile (§1.11); the sensors stay in the file although they generate nothing yet (they catch port
collisions the C++ cannot see — P6 fixtures 26-27); the panel gets its own CI job (§1.8); the record
decoder is Python, pinned to the C++ decoder by a test (§1.7). **From the re-check** (§1.2 has the
rules; battery and mutants in `recheck/`):
- **Robot two's `typedTable` is a named, grandfathered LEGACY block.** It exists because Bench Tests'
  DRIVE station and the old drive program read the signs as typed (`chassis_table.hpp` :28-36). The
  tool refuses it on every other robot file, so PKG1's starter can never type a sign or a wheel.
  Robot two's 2.75 in wheel may never change; the wheel is inert anyway, because the typed geometry
  can never be complete with track width and ratio held at 0. A robot-two sign may change only
  together with its provenance, which is a re-measurement. *(Amended 2026-09-29: or a `flip-front`, whose
  provenance says the signs were negated, not re-measured — D7.)*
- **A changed hardware fact needs a changed provenance.** The tool refuses it otherwise, and the
  baseline is what the robot's C++ carries now.
- **Text fields can never become code** (the text-field rule).
- **The front is changed only by `flip-front`**, or by hand in the text.

---

## The design in plain words (for the team lead)

Robot two gets **one file that says what it is**: `src/robots/robot_two.robot.json` — the drive ports and
which side, the cartridge, the IMU, ~~which end is the front,~~ *(re-check 2026-10-29: no — the front is the end that leads Calibrate's push)* the radio and the camera, the claw pivots and
the claw, and which button works each one. For every robot but robot two, nothing in it is a number the
robot can measure. The track width, the wheel size and which way each drive motor spins come from shulib
Calibrate's record on the SD card, which the panel shows but never lets you type. **Robot two's file has
one exception, and says so.** It keeps a LEGACY block: the drive signs MOTOR WATCH measured on
2026-09-10 and the 2.75 in wheel reported on 09-14. It keeps them only because Bench Tests and the old
drive program still read those numbers as typed. The panel shows that block read-only. The tool refuses
the block on any other robot and refuses a new wheel size. A sign may change only together with a note
of who re-measured it. The same rule covers every fact in the file: **change a port, a cartridge or a
polarity, and the tool refuses until the provenance says who stated the new value, and when.**

**The C++ is written from that file** by `tools/robot_tool.py generate`, into five fenced sections of
the three files that hold those facts today. Each fence says, in the code, "GENERATED from
src/robots/robot_two.robot.json — edit that file or the robot panel". Everything outside the fences —
how the tables are checked, how buttons are bound, the driving feel — stays code people write and
defend. Today's file reproduces today's C++ **exactly, byte for byte** (probed); the fences are the only
new lines.

**It cannot drift or be edited by accident.** The tests and CI regenerate every fenced section and fail
if one differs — "edit the robot file or the panel" — and the generator refuses to overwrite a section
someone edited by hand. Before it writes anything, it checks the file against every rule the C++ checks
would apply, and says which one (a button used twice, a claw on a drive port, a name too long for the
brain) — a test proves the two agree, and the C++ checks stay the final word. Nothing typed into the
file can turn into code: a line break, a marker or a stray backslash in a note is refused by name. In
THIS repository, changing a button also means re-quoting the cookbook's listing of robot two's buttons
and changing the tests that pin robot two's values; the tool lists both when it writes.

**The panel is optional.** It is a small VS Code extension: open the robot file and you see a form —
ports, mechanisms, a dropdown per button — with problems shown as you type, the file and the C++ updated
when you save, and the calibration record from the SD card shown read-only (it even says when the record
was calibrated with the other end as the front — true for robot two today). No build, test or gate needs
it; the robot builds exactly as it does now.

**Amended 2026-09-29 — what the panel now does, in plain words.**
- **A NEW robot.** You type a name, pick the drivetrain kind and the motor cartridge~~, and say which end is the
  front,~~ *(re-check 2026-10-29: three facts — the front comes from Calibrate's push)* and press Create. The tool writes the robot's file AND everything its build needs: its `make
  ROBOT=` value, its line in the one chain, its tables and its rows in the build gate.
  - The first builds are Bench Tests and Calibrate, which is how a new robot is brought up.
  - ~~Calibrate prints the front you stated right before you push the robot, so you lead with that end.~~ *(re-check 2026-10-29: the end you lead Calibrate's push with IS the front, and Calibrate warns if a later calibration flips it.)*
  - After the mechanism core, the robot also gets a starter driver file and the drive, auton-tester and
    competition programs, with a starter auton file whose autons the students write.
  - Ports are optional: Calibrate finds the drive's own.
  - **What that costs, said plainly:** with no ports typed, nothing can check the front you stated against
    the calibration, and a mechanism wired to a drive motor's port is not caught when you build. The
    calibration page says the first. For the second, the mechanism core is asked to make Calibrate spot a
    "mechanism" motor that turns with the drive. *(re-check 2026-10-29: superseded — no front is stated, and CAL1's Calibrate names a declared mechanism motor that turns with the push.)*
- **Mechanisms.** You add one from a template — Intake, Arm, Lift, Claw, Flywheel, Catapult, Conveyor or
  Blank. The template fills what it can and leaves EMPTY every number only the team can know (the gearing,
  the travel limits, the cartridge). The tool refuses the file until those are stated, or you mark the
  mechanism a draft. You can also remove, duplicate, reorder and rename mechanisms. Removing one also
  removes its buttons, presets and wiring, and first lists every auton that names it: those autons will no
  longer build, and the compiler names each one.
- **Wiring.** A mechanism names ROLES ("arm motor 1", "limit switch"), never ports. *(re-check 2026-10-29: the core's named slots — its motors in order, an angle sensor, a piece sensor, a home switch, its lines.)* The wiring page binds
  each role to a port, and warns about a port used twice, a drive port or an unbound role. Re-cabling means
  changing the wiring and rebuilding; nothing is bound on the brain (the team lead's ruling).
- **Controls.** Each mechanism gets hold-to-run, toggle or preset buttons, with a warning the moment one
  button does two jobs. *(re-check 2026-10-29: plus the two homing controls and "autons only".)*
- **Calibration.** The calibration record is shown read-only, with the places it disagrees with the file.
- **Saving.** Save runs the same checks the build runs, through the one checker the tool has.
- **Robot two's front.** The team lead is asked to confirm it. The recommendation is the camera end, the end
  its record and its drivers already use; flipped that way, the file agrees with the record on all eight
  motors the record has. *(re-check 2026-10-29: now "robot two's TYPED table" — D7, kept: Bench Tests and the boot's CROSS-CHECK line then agree with the drivers.)*
- **(Re-check 2026-10-29) Also coming:** routes drawn on the field (Part R), the camera and side-sensor page (Part S) and the auto-flipper's page (Part AF), each as its chunk lands; the documentation redo (DOCS3) after the panel. *(amended 2026-10-29 evening: and, by Nov 21, the driving feel and the drive tuning as pages, a FULL route maker — every auton step, care on each, mechanism timing, red/blue mirroring, a timeline — and the panel on GitHub for anyone; D11–D20)*

---

## 0. Why — what exists, re-read at `a62067b` (every line opened)

> **Re-read 2026-09-29 at `ee2e399`:** `src/`, `tools/` and the `Makefile` are byte-identical to `a62067b`,
> so §0.1–§0.5 hold as written. The amendment's new reading is in §0.7, its probes are N1–N8 (the amended
> section), and what T4 Part 3, COMP-t, COMP-b and the MECH core will have changed is re-read at each
> part's tip.
> **(Re-check 2026-10-29)** §0's line numbers are `a62067b`'s / `ee2e399`'s; the re-check's citation table gives `73044ca`'s, and every part re-reads its files at its tip.

### 0.1 The facts, and where they live today
- **The chassis table** (`src/robots/chassis_table.hpp`): `ChassisTable` :66-87; robot two's block
  :89-140 (a 32-line comment, then `kRobotTwoChassis` with signed ports `{-11, 12, -13, 14, -15}` /
  `{20, -19, 18, -17, 16}`, BLUE, IMU 2, `measured`/`signsMeasured` true, wheel 2.75, track width and
  ratio 0); the bench bot's :142-173 and the X-drive's UNSET table :175-197 (both stay hand-written);
  `chassisTableFor()` :203-213; the selection pins :224-237; `tableConsistent()` :449-519 (six checks,
  runtime); `describeMissing()` :295-308. Header rule :22-26: "When a fact arrives, THIS is the one place
  to edit".
- **The mechanism table** (`src/robots/mechanism_table.hpp`): `kTeleopRezeroButton` :169 (T4 1b removes
  it); `kClawOpenCommand` :171-183; the entry types :188-223; robot two's block :245-310; `kNoMechanisms`
  :313-314; `mechanismTableFor()` :321-331; the nine checks :350-492 and their asserts :583-611; the pins
  :619-625. Header: the RULE :38-42, THE POLARITY :61-70, THE HOST SUITE TESTS THESE VERY BYTES :81-91,
  WHO READS IT :93-115.
- **The driver file** (`src/team/robot_two_driver.hpp`, T4 Part 1a): the header's words :18-31 (the
  button lines :22-29), WHICH BUTTON :63-71, THE FEEL :73-83 (`.rezeroButton = Button::Y` :81), the six
  checks :119-134.
- **The five predicates** (`src/common/driver_controls.hpp` :136-255): `controlButtonsDistinct`,
  `controlsAvoid`, `holdVoltsValid`, `controlsNameTable`, `controlsCoverTable`.
- **The boot's conversion** (`src/common/boot_source.hpp`): `mechanismTableProblem()` :119-151 (calls the
  count and completeness checks first, then ≤ 8 mechanisms and names ≤ 23 characters),
  `toMechanismConfig()` :195-217 (a position becomes a level here), `kDeclaredMotorLabel` :102-106.
- **The config's own refusals** (`include/shulib/config/robot_config.hpp`): `kMaxMechanisms` 8,
  `kMaxMotorsPerMechanism` 4, `kMaxPneumaticsPerMechanism` 2 (:57-66); `MechanismConfig` :443-483;
  `RobotConfig::valid()` :545-599 onward (one owner per smart port, 3-wire lines once, names unique —
  the name rule is checked ONLY here, at boot).
- **The one chain** (`src/build_selection.hpp` :115-139) picks `kRobotId`; the tables select from it.

### 0.2 The gate's step 0 and the table files
`TABLE_FILES` (`tools/src_build_gate.py` :398-401) names the two table files; `per_robot_table_names()`
(:522-552) derives every per-robot name from them — fail-closed when a file's selection function is not
exactly one; `_stray_selection_problems()` (:652-677) fails any other src/ file naming one outside a
`static_assert`. The header states the rule at :148-161.

### 0.3 T4's ruling R1 and the whole-run rule written for G3a
T4 brief :309-320: the driver file owns which button does what; "G3a's robot panel writes a MARKED REGION
INSIDE the driver file ... freshness-gated like `check-examples`, and a hand edit inside it fails the gate
with 'edit the robot file or the panel'. Whether the region also covers the feel and the loop is G3a's to
decide." T4 Part 1a's review added the WHOLE-RUN rule to `check-examples` **for this chunk**
(`tools/api_doc_tool.py` :1833-1853: "G3a's robot panel is going to write exactly that part of the file"):
cookbook 06's listings of the driver file (:217-225, :230-237, :243-261) must each be one unbroken run.

### 0.4 The drafted profile and the ruling
`docs/vexbuilder-integration.md` §3.1 (:96-126) drafts `robotProfile` with `wheelDiameter`, `trackWidth`,
per-motor `reversed` and `gearRatio`, `odometry.imuPort`, and pneumatic LEVELS
(`pneumaticSafe`/`pneumaticStart`); §3.2 (:128-175) its field notes; §6 (:450-463) "unknown newer fields
are ignored, never fatal"; §9 (:524-534) "data, never code". It predates the runtime-discovery ruling and
has no `controls` (build-order ~:1588-1595). F7 freezes at G3, not here (~:1612-1615).

### 0.5 The calibration record
`include/shulib/calibration/calibration_record.hpp`: `kRecordFileName = "shulib_drivetrain.bin"` :84;
the v1 layout :123-143 (180 bytes, magic `SHDC`, CRC-32 over 0..175); `decodeRecord()`'s five refusals in
order :196-306, the fifth being `hal::DrivetrainDescription::valid()`
(`include/shulib/hal/drivetrain_description.hpp` :158-285, eleven sentences). The record carries no
wheel diameter and no ratio: it carries **inches per motor radian per side** (their product) and the
track width. SAFE-SAVE adds `shulib_drivetrain.bak` (its brief ~:482-483); R5a adds
`shulib_characterization.bin` (`SHCH`, its brief ~:137-139) and leaves "G3a's panel reading the sibling"
to G3a (~:1310).

### 0.6 The probes — results (every one re-run in `v2/`)
| # | What | Result |
|---|---|---|
| **P1** | **The differential.** `extract.py` builds robot two's file from the tree's bytes; `adopt` wraps the five blocks in markers; `generate` over the adopted tree | **Every region generated == the original, line for line:** chassis :89-140 (52 lines), polarity :171-183 (13), mechanisms :245-310 (66), control words :22-29 (8), controls :65-71 (7) — **146 lines, zero differences**. The adopted files minus their marker lines == the originals, byte for byte (all three). `generate` on the adopted tree changed **0 bytes**. The only added lines: the **10 markers** |
| **P2** | The gates on the adopted tree | `src_build_gate.py check` **PASS**, all six builds link, step 0 clean (1 m 15 s); `api_doc_tool.py check-examples`: 467 quoted lines, **3 listings of whole-run files, each one unbroken run** (the markers sit outside cookbook 06's listings); the seven host TUs that read robot two's tables: **6 774 / 6 774 assertions** |
| **P3** | A hand edit inside `[controls]` (R1 → X) | `check`: `FAIL: src/team/robot_two_driver.hpp:68-74: the lines inside [controls] were EDITED BY HAND ... edit the robot file or the panel`; `generate` REFUSED and left the file byte-identical; `--discard-hand-edits` restored the adopted bytes |
| **P4** | A panel-style edit: claw pivot 7 → port 8, Up/Down, new words | `check` names three stale regions; `generate` writes two files (the diff is exactly the port, the four button spellings, the words line, three sha's); **all six ARM builds PASS** (Teleop's pre-1b `controlsMatchTable` tie holds by construction); host (**corrected at the re-check**, rebuilt from scratch in `recheck/p4`): **98 cases; the first crash ends the run (79 run, 14 fail, 19 never run). With the three crashing cases excluded, 24 more fail (93 assertions): 27 of 98 in all, three by SIGSEGV.** The crashes are `robot_two_driver_test.cpp` T8 (:629, which dereferences `holdFor("claw pivot 7")` at ~:640 with no REQUIRE) and `teleop_claws_test.cpp` 1g (:479) and 1h (:546). Each dereferences a mechanism or binding that the rig's unplugged port 8 leaves null. `api_doc_tool.py check-examples` **fails** too: cookbook 06's WHICH BUTTON listing, block 5 (the checker's `check/p4`) |
| **P5** | A third mechanism (a clamp on 3-wire B, button B), generated with the validator bypassed | validator: `static_assert(robot two's selection: 2 motor and 1 pneumatic)`; `ROBOT=tank` compile: `mechanism_table.hpp:634: static assertion failed: ROBOT=tank must select robot two's mechanism table ...`; the host (bench selection) compiles — only the four tank builds see it |
| **P6** | **The agreement harness**: 31 fixtures (robot two with ONE thing broken each), emitted UNCHECKED; a C++ runner evaluates the real predicates, `tableConsistent`, `describeMissing`, `mechanismTableProblem` and `RobotConfig::valid()` at run time | **27 agree, 0 disagree, 4 schema-level** (a cartridge `purple`, a button `Q`, a mixed motor+line mechanism, duplicate names in the pre-1b emitter — the C++ cannot spell them). **The first run had 12 disagreements**, each a real gap closed at the root: the boot's `RobotConfig::valid()` also refuses a mechanism port fault; `mechanismTableProblem()` calls the completeness check; a signed table with an empty side; pre-1b artifacts. **Mutations, observed red and restored:** V1 the validator drops the volts rule → 2 red; C1 the C++ accepts 13 V → 1 red. **Re-check:** this harness filtered out the selection pin (`agree.py` :133); `recheck/agree_pin/` evaluates it through one named predicate — 27 agree, and the validator's pin removed → fixture 24 DISAGREES |
| **P7** | Robot two's chassis block moved into its own generated file | step 0: **1 problem**, `src/robots/robot_two_tables.hpp:35 names kRobotTwoChassis, a per-robot table` (D2 (b)) |
| **P8** | The host record decoder vs the REAL encoder and decoder | **20 / 20 agree**, every sentence byte-identical: the golden (`test/calibration_record_test.cpp` :84), robot two's 2026-09-25 record re-encoded by `encodeRecord()` (`L -16 +17 -18 -20 R +11 -12 +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01`, R3d-PROGRESS ~:2466), and 18 malformed files (each of the four prefix refusals, the CRC, and nine `valid()` refusals with the CRC fixed). **The front check on robot two:** `FRONT REVERSED: the record's LEFT is this file's RIGHT` and `LEFT OUT or unplugged at calibration: port(s) [14, 19]` — both true today (worksheet ~:522-529) |
| **P9** | The panel prototype | TS model **5/5** under `node --test` (no editor, no display); mutation P1 (an edit in place) → 1 red, restored; `vsce package` → a 5.77 KB `.vsix`; installed into an isolated profile (`--extensions-dir`/`--user-data-dir` in scratch): `shulib-local.shulib-robot-panel@0.0.1`; validate ~20 ms, 18 MB (the FILE command line). **Not probed (said at the re-check):** the panel↔tool contract. The prototype tool has no `validate - --json`, `fmt -`, `describe` or `generate --dry-run`, so the prototype `extension.ts`'s calls would fail against it; the prototype panel also hard-codes the cartridge list and maps refusals by LABEL. Part B builds the contract, and its model tests pin it. The `@vscode/vsce` 4.0.0 used here declares `node >= 22` (the local Node is 20). `recheck/panel/`: 3.9.2 (`node >= 20`), 5/5, packaged 5.82 KB, the real repository URL |
| **P10** | The post-1b emission | differs from today's by exactly the seven `.forward`/`.reverse`/`.volts`/`.toggle` lines |

*(Amended 2026-09-29.)*
- **P1's chassis row** (52 lines) is Part 1's differential, re-run at the tip.
- **Its polarity, mechanisms, control-words and controls rows, and P10,** describe bytes the MECH core
  rewrites into roles and a binding. Part 5 re-does their differential against the core's tree by the same
  method: extract from the tree, adopt, `check` with zero differences.
- **P4 and P5** stay true of the pre-core tree. Their lessons (the witnesses, the pins) carry over.

### 0.7 What the amendment re-read (at `ee2e399`; every line opened)
- **The one chain** (`src/build_selection.hpp`):
  - the guards: :69-71 is the two-robot guard (`XDRIVE && TANK_2026`, a pairwise form that does not grow);
    :72-84 are the program guards, each "needs ROBOT=tank";
  - `enum class RobotId` :99-103; the chain :115-139; `robotBeacon()` :148-158, a case per robot and no
    default.
  - The header's rule (:7-24) — the switches are read only here, and every branch sets exactly three
    constants — is what a generated robot's lines must obey. Probed N2: they do.
- **The Makefile:**
  - the `ROBOT` block :96-105, whose valid-values message is at :104;
  - the `PROGRAM` block :144-164: `drive`, `library` and `calibrate` are each refused unless `ROBOT=tank`, at
    :147-161. `calibrate`'s message says "takes the cartridge from robot two's chassis table".
- **The gate** (`tools/src_build_gate.py`):
  - `VARIANTS` :307, `BUILD_ARGS` :308-315, `VARIANT_DEFINES` :316-323, `ALL_DEFINES` :324 (derived),
    `VARIANT_BEACONS` :325-332, `BUILD_ROBOT` :339-346, `ROBOT_BEACONS` :347-351, `TABLE_FILES` :400-401,
    `BUILD_PROGRAM` :413-420, `PROGRAM_ENTRY_SYMBOLS` :421-429;
  - their header (:300-306): "a build added to one place and not the others fails here, which is the point
    of having the tables";
  - `per_robot_table_names()` :522-552 derives the per-robot set from `TABLE_FILES`: the tables and the two
    selection functions, and no driver class (N6).
- **The selection functions:**
  - `chassisTableFor()` (`chassis_table.hpp` :203-213) and `mechanismTableFor()` (`mechanism_table.hpp`
    :321-331), each a case per robot and no default (-Wswitch fails the gate);
  - the per-robot selection pins (`chassis_table.hpp` :230-237), which name each robot.
- **The host tests' robot lists**, typed literally:
  - `test/build_selection_test.cpp` :35-36 (`kEveryRobot`, three robots) and :65-67 (each robot's beacon);
  - `test/mechanism_table_test.cpp` :389-397 (a loop over three robots);
  - `test/chassis_table_test.cpp` :554-569.
  - A robot added to the enum and not to these lists is simply untested. Part 2 moves the list beside the
    enum as `kEveryRobotId`. `build_selection_test.cpp`'s list and `mechanism_table_test.cpp`'s loop read it
    (probed N2, which edited exactly those two). `chassis_table_test.cpp` :548-570 is not a loop: it is
    literal per-robot identity checks, which stay for the hand-written robots, and Part 2 ADDS a
    `kEveryRobotId` case beside them — every robot's table distinct, naming its own robot (probed at the
    re-check, `recheck2/everyrobot/`: 102/102; the generated robot handed `kNoChassis` → that case red).
- **An empty control list:** `src/common/driver_controls.hpp` :110-111 says "C++ has no empty `[]` array"
  (probed N4). `DriverBindings::bind()` (:306) takes `std::span`s, so a starter file spells an empty list
  as a span.
- **What the bring-up programs need from a robot:**
  - Calibrate reads only `kChassis` and `kMechanisms` (`calibrate_program.cpp` :1086, :1129, :1137-1237),
    and REFUSES an unset cartridge (:1137). That is why the new-robot page requires the cartridge;
  - its banner says "robot two MEASURES ITS OWN DRIVETRAIN" (:1044, :1068), which is false on another robot;
  - Bench Tests prints robot-two examples as dated history (`bench_r3a.cpp` :1463, :2505, :2667), which is
    harmless.
- **What the match programs name directly** (the T4, COMP-t and COMP-b briefs; not yet in the tree):
  - `RobotTwoDriver`, held in the programs' static state (T4 brief, COMP B11 as Part 1a replaces it);
  - `registerAutons()` from `src/team/autons.cpp` (COMP-t §3.1: "the one function both auton-running programs
    call");
  - "`PROGRAM=autontest` needs `ROBOT=tank` -- the team's autons (src/team/autons.cpp) are robot two's"
    (COMP-t §3.6).
- **Robot one** (HANDOFF :114-121, Q3): a tank whose ports Calibrate discovers. "Calibrate builds only for
  `ROBOT=tank` and takes the cartridge from robot two's table — robot one needs either its own variant or a
  stated cartridge." New-robot creation IS that variant. Robot one's cartridge and front are the team's to
  state; nothing here types them.

---

## 1. Scope — IN

### 1.1 The robot file (Part A — *amended 2026-09-29: Part 1 for the drivetrain half, Part 5 for the mechanism half*)

> **AMENDED 2026-09-29.**
> - *(re-check 2026-10-29)* **`drivetrain.front` leaves the file** (RC2); **`camera` and `walls`** join at Part S (RC9); the mechanism rows are MECH §6.3's as landed — the re-check's adoption table (RC4).
> - **Rows that hold, in Part 1:** `schemaVersion`, `id`, `identity`, `drivetrain.*`, `typedTable` and
>   `sensors[]`.
> - **`drivetrain.left`/`.right` on a NEW robot are OPTIONAL** (D10). An empty side generates `.left = {}`
>   with a count of 0, which is what `kNoChassis` is. When the sides are typed, they are stated looking toward
>   the front, as below.
> - **NEW rows:**
>   - `build` (generated robots only: `{"programs": [...]}` from `tester`, `calibrate`, and from Part 4 on
>     `drive`, `autontest`, `competition`). It is REFUSED on `robot_two`, whose build is hand-written (D9), and
>     REQUIRED on every other robot file;
>   - `wiring`, and the mechanism kinds with their settings (§1.14).
> - **The rows `mechanisms[]` (motor), `mechanisms[]` (pneumatic), `mechanismNotes` and `controls` are
>   SUPERSEDED by §1.14** (roles, never ports; the wiring separate — P2, Q1 (a)). Part 1's tool REFUSES those
>   keys by name ("mechanisms arrive with G3a Part 5; until then robot two's are the hand-written C++ in
>   src/robots/mechanism_table.hpp") rather than ignore them. Unknown keys elsewhere still warn.
> - **The verbatim file below** is kept as the evidence P1 rests on. Everything up to and including `sensors` is
>   Part 1's robot two file, re-checked at the tip. Its mechanism half is superseded, and Part 5 re-makes it
>   from the tree the MECH core leaves.
> - **"The post-1b deltas"** below are superseded. T4 Part 1b no longer exists; T4 Part 3 and the MECH core
>   change those bytes, and Part 5 re-extracts them.

**Fields** (the file IS the profile block — D1; draft `schemaVersion: 1`, F7 freezes at G3):

| Field | Human-stated? | Generates | Refused when |
|---|---|---|---|
| `schemaVersion` | — | nothing | ≠ 1 |
| `id` | yes | the table names (`k` + CamelCase + `Chassis`/`Mechanisms`) | not lower_snake_case |
| `identity.name` (`team`, `season` optional) | yes | `.robot` | empty, non-ASCII, > 31 chars |
| `drivetrain.kind` | yes | — | not `tank` (the X-drive is a C++ `RobotConfig`, `src/robots/xdrive_config.hpp`) |
| `drivetrain.cartridge` | yes | `.cartridge` | not red/green/blue |
| ~~`drivetrain.front`~~ *(re-check 2026-10-29, RC2: REMOVED — the robot file states no front (2026-10-06); the tool refuses the key by name; robot two's provenance and notes keep their front words as text)* | yes (words; WHICH end is changed only by `flip-front`, §1.2) | ~~nothing in C++ (the panel's record check)~~ *(re-check 2026-09-29, #3)* from Part 2: `.front`, the literal Calibrate prints before the push; a Calibrate build of a robot that states none fails to compile (§1.13). Also the panel's record check | empty; the text-field rule, literal-bound from Part 2 (printable ASCII); > 160 characters (Calibrate's 240-byte log line) |
| `drivetrain.left`, `.right` | yes (as wired, looking toward the front) | `.left`/`.right` (+ signs from `typedTable`) | `tableConsistent`'s rules |
| `drivetrain.imuPort` | yes | `.imuPort` | out of range, a drive port |
| `drivetrain.seenByCensus` | yes | `.measured` | not a bool |
| `drivetrain.provenance`, `.notes` | yes | the literal, the comment | provenance empty; the text-field rule (provenance: printable ASCII too); **a drive value changed from what the chassis region carries while the provenance did not** (§1.2's baseline rule); *(re-check 2026-09-29, #13)* a provenance longer than the banners print whole: 206 characters, the `provenance : %s` line in a 220-byte buffer (`bench_r3a.cpp` :191/:3192, ~~`drive_program.cpp` :134/:303~~; re-read at the tip — *re-check 2026-10-29: `bench_r3a.cpp` :196/:3200 only; the old drive program's line is gone, RC15*). Robot two's is 203 today, and D7's flipped one 200 |
| `typedTable` (optional, LEGACY — **robot two only**) | measured by MOTOR WATCH, **read-only in the panel** | `.signsMeasured`, signed entries, `.wheelDiameterIn`, `.geometryProvenance` | **present on any robot file but `robot_two` (the ruling)**; signs not exactly the drive ports; a non-zero `trackWidthIn` or `externalRatio` (the ruling); **robot two's `wheelDiameterIn` changed from what the region carries** (frozen legacy); **a sign changed while `drivetrain.provenance` did not** (a re-measurement names itself) |
| `sensors[]` (`kind`, `port`, `provenance`) | yes | nothing yet (R2b) *(re-check 2026-10-29: the radio; the camera and the side walls become the `camera` and `walls` blocks at Part S — RC9)* | a port another thing claims (the tool's own, stricter rule) |
| `mechanismNotes` | yes | the comment | — |
| `mechanisms[]` motor: `name`, `motors[{port, reversed}]`, `cartridge`, `safeMode`, `provenance` | yes (a MECHANISM's direction is the team's choice — Calibrate measures only the drive) | an entry | the table's checks 1-3, 8; `mechanismTableProblem`; `RobotConfig::valid()`; one motor per entry; a value changed while `provenance` did not |
| `mechanisms[]` pneumatic: `name`, `pneumatics[<A..H>]`, `openLevel`, `openLevelVerified`, `start`, `safe`, `provenance`, `openLevelNotes` | yes | the polarity constant + an entry | check 9; one line per entry; two pneumatics whose names give ONE constant; `openLevel` flipped while `openLevelNotes` did not (a flip is a measurement); a value changed while `provenance` did not; `openLevelVerified: false` is a WARNING |
| `controls.holds[]` (`mechanism`, `forward`, `back`, `volts`, `words`), `controls.toggles[]` (`mechanism`, `button`, `words`) | yes | `kHolds`/`kToggles` + the header's button words | the five predicates |

A **text field** is a string (the generator lays it out, wrapping new text at 100 columns) or a list (the
layout kept exactly: a literal's pieces, a comment's lines) — robot two's are lists, which is how it
reproduces byte for byte. **THE TEXT-FIELD RULE (re-check #1), for every text field the tool knows:**
- no control character of any kind (C0 including tab and CR, DEL, C1 including NEL, U+2028/U+2029),
  because one text entry is one line;
- no `>>>`/`<<<` before `GENERATED`, the marker token;
- no backslash before whitespace or at the end, which would splice two lines;
- no trigraph (`??` then `= / ' ( ) ! < > -`), because the compiler warns on it and `-Werror` refuses;
- in addition, a field that becomes a C++ **literal** the brain prints (`identity.name`, both
  provenances, mechanism names, a control's `mechanism`, and from Part 2 `drivetrain.front`) must be
  printable ASCII; and one the brain prints on a fixed-size line must fit it (`drivetrain.provenance`
  ≤ 206, `drivetrain.front` ≤ 160 — re-check 2026-09-29: `vsnprintf` cuts a longer one without a word).

The fields that become **comments** keep their UTF-8, because robot two's notes carry `—` and `§`. Why
it matters: a notes line ending in `\` passed the old validator, generate AND check, yet it commented out
`inline constexpr ChassisTable kRobotTwoChassis = {` and failed the compile (`recheck/f1/splice`); a
newline in a hold's words wrote a live `static_assert` line (`recheck/f1/words`). Unknown fields are a WARNING naming the key ("a typo?"), kept on every write,
never refused (§6's rule). The canonical writer (ONE writer: `robot_tool.py fmt`, and the panel saves
through it) orders keys by the schema; `fmt` on robot two's file is idempotent (probed).

**Robot two's file — VERBATIM as proposed (the tree at `a62067b`, pre-1b; produced by `extract.py` from
the tree's own bytes, and proven by P1). The re-check left it unchanged, and it passes the amended
validator. Its literal-bound text is ASCII, and its `—`/`§` sit only in comment-bound fields. On
`recheck/t0`, `validate --tree` gives 0 refusals, `check` is clean, `generate` changes 0 bytes, each
region file minus its markers equals `a62067b`, and `fmt` is idempotent:**

```json
{
  "schemaVersion": 1,
  "id": "robot_two",
  "identity": {
    "name": "2026 TANK CHASSIS (robot two)"
  },
  "drivetrain": {
    "kind": "tank",
    "cartridge": "blue",
    "front": "the 15/16 end: the end where ports 15 and 16 sit (left and right below are named looking toward it)",
    "left": [11, 12, 13, 14, 15],
    "right": [20, 19, 18, 17, 16],
    "imuPort": 2,
    "seenByCensus": true,
    "provenance": [
      "ports + IMU 2 + radio 1: census 2026-09-10 evening, all ten motors; ",
      "SIGNS: MOTOR WATCH, two front-first pushes 2026-09-10 evening, identical; ",
      "front = the 15/16 end (team lead); cartridge BLUE off a motor"
    ],
    "notes": [
      "ROBOT TWO -- the 2026 tank chassis (build team, stated 2026-09-10): five COUPLED motors",
      "per side driving four wheels per side.",
      "PORTS: reported by the team lead 2026-09-10, read off the robot while standing BEHIND",
      "it (the back of the robot against him, looking toward the front): LEFT 11 12 13 14 15",
      "and RIGHT 20 19 18 17 16, each listed BACK -> FRONT (11 and 20 are the rearmost motors,",
      "15 and 16 the frontmost). THE FRONT is therefore the end where ports 15 and 16 sit --",
      "the end away from a person who reads the left side as 11..15. Order within a side is",
      "informational: the table is a set.",
      "SIGNS: MEASURED 2026-09-10 evening by the tester's MOTOR WATCH -- two whole-robot pushes,",
      "FRONT-first, identical both times (R3b-PROGRESS Session 2 §10.3):",
      "    LEFT -11 +12 -13 +14 -15  |  RIGHT +20 -19 +18 -17 +16",
      "A '-' entry is a port PROS reverses (negative port number in ProsMotor); the alternation",
      "within a side is the coupled-gear-train signature, not a wiring fault. Port 18 travelled",
      "~20 % short of its side-mates on both pushes -- unexplained; watch it under power (A4",
      "register HA-130; the group's disagreement floor tolerates it, HA-133).",
      "CARTRIDGE: BLUE, read off a motor by the build team 2026-09-10 -- a reported measurement,",
      "so it may be set; it is still printed as a BELIEF wherever the adapter is about to WRITE",
      "it (the read-back cannot detect a wrong belief). IMU: PORT 2, mounted 2026-09-10 and seen",
      "by the census that evening; the radio is on port 1. Also reported, and deliberately NOT",
      "here because nothing here uses them yet: wheels 2.75 in, and \"600 rpm\" (reads as direct",
      "drive 1:1, UNCONFIRMED until tooth counts arrive -- odometry's business, R3b Part 2).",
      "`measured` flipped TRUE 2026-09-10 evening: station 1's census showed MOTOR on all ten",
      "table ports (after cables on 11, 15 and 19 were re-seated), the IMU on port 2 and the",
      "radio on port 1. The cartridge is still a belief read off a motor, not a measurement.",
      "GEOMETRY (R3b Parts 1-3, 2026-09-14; A4 register HA-124 wheel, HA-125 track width, HA-126",
      "ratio): wheel diameter 2.75 in REPORTED by the team lead",
      "(\"standard wheels\", the bench bot's measured size) -- a report, not yet a ruler on THIS",
      "robot; track width UNSET (tape measure, contact line to contact line, owed by the team",
      "lead); external ratio UNSET (\"600 rpm\" was reported, which reads as direct drive 1:1 but is",
      "a cartridge speed, not a tooth count -- the coordinator types 1.0 only when \"direct drive\"",
      "or the counts are stated). The library program REFUSES at boot while either is 0.",
      "kChassis in the four ROBOT=tank builds (chassisTableFor(RobotId::RobotTwo))."
    ]
  },
  "typedTable": {
    "measuredSigns": [-11, 12, -13, 14, -15, 20, -19, 18, -17, 16],
    "wheelDiameterIn": 2.75,
    "trackWidthIn": 0.0,
    "externalRatio": 0.0,
    "geometryProvenance": [
      "wheel 2.75 in REPORTED by the team lead 2026-09-14 (standard wheels; ",
      "not yet rulered on this robot); track width UNSET (tape owed); ratio ",
      "UNSET (\"600 rpm\" reported = a cartridge, not a tooth count)"
    ]
  },
  "sensors": [
    {
      "kind": "radio",
      "port": 1,
      "provenance": "census 2026-09-10 evening (the chassis notes)"
    },
    {
      "kind": "ai-vision",
      "port": 3,
      "provenance": "on robot two since 2026-09-24 (HA register); no library adapter reads it yet (R2)"
    }
  ],
  "mechanismNotes": [
    "ROBOT TWO — the 2026 tank chassis. Defined in every build (header: the host suite tests these",
    "bytes); kMechanisms only where kRobotId is RobotId::RobotTwo (mechanismTableFor()).",
    "THE PIVOTS' PORTS, CARTRIDGE, BUTTONS, VOLTS, SAFE STATE: REPORTED by the team lead 2026-09-26",
    "(T3's request, verbatim in its brief). NOT SEEN: the 2026-09-24/25 captures list motors only on",
    "11-18 and 20, so no census has shown a motor on 6 or 7 yet (A4 register HA-153). SIGNS: + on",
    "both, i.e. each motor's own forward; which physical way that moves each claw is UNMEASURED.",
    "SAFE STATE Hold: the team lead's \"holds its angle\" -- a Hold at 0 V reaches the motor only after",
    "DEFECTS2, and holding a LOADED pivot is unmeasured (A4 register HA-156). VOLTS 12: full power",
    "passes straight through while a button is held (a stall cut-off is out of T3), so a pivot held",
    "against its stop heats at the motor's DEFAULT current limit, 2500 mA, and nothing in shulib",
    "acts on a mechanism motor's temperature yet — \"shulib Teleop\"'s claw row SHOWS it (T3 Part 2),",
    "and the driver is the only cut-off (A4 register HA-154). THE PORTS are all the brain",
    "can check: a claw cable swapped with a drive cable is invisible to it (A4 register HA-155).",
    "THE PNEUMATIC CLAW: REPORTED by the team lead 2026-09-26 (T3 addendum §0) — 3-wire port A",
    "(line 1), toggled by A, CLOSED at program start, keeps its state when disabled; single or",
    "double, and the polarity, NOT KNOWN (A4 register HA-159): one line declared, the polarity",
    "kClawOpenCommand's guess. Its SAFE position, CLOSED, is the addendum's statement — a closed claw",
    "keeps what it holds — not the team lead's words. Nothing has seen it: shulib's census reads",
    "the 21 smart ports only, and the one program that drives its line — \"shulib Teleop\", since T3",
    "Part 2 (the line made at the START level, then A's toggle) — has not run on the robot."
  ],
  "mechanisms": [
    {
      "name": "claw pivot 6",
      "motors": [
        {
          "port": 6,
          "reversed": false
        }
      ],
      "cartridge": "red",
      "safeMode": "hold",
      "provenance": [
        "team lead 2026-09-26: port 6, RED, R1 forward / R2 back, 12 V while ",
        "held, holds on release; REPORTED, not yet seen by a census; sign + ",
        "= the motor's own forward (way UNMEASURED)"
      ]
    },
    {
      "name": "claw pivot 7",
      "motors": [
        {
          "port": 7,
          "reversed": false
        }
      ],
      "cartridge": "red",
      "safeMode": "hold",
      "provenance": [
        "team lead 2026-09-26: port 7, RED, L1 forward / L2 back, 12 V while ",
        "held, holds on release; REPORTED, not yet seen by a census; sign + ",
        "= the motor's own forward (way UNMEASURED)"
      ]
    },
    {
      "name": "claw",
      "pneumatics": ["A"],
      "openLevel": "high",
      "openLevelVerified": false,
      "start": "closed",
      "safe": "closed",
      "provenance": [
        "team lead 2026-09-26 (port A; polarity unknown): A opens, A again ",
        "closes, CLOSED at program start, keeps its state when disabled; ",
        "single or double solenoid NOT KNOWN, one line declared; safe = ",
        "CLOSED is the T3 addendum's (a closed claw keeps what it holds)"
      ],
      "openLevelNotes": [
        "THE CLAW'S OPEN POLARITY — **UNVERIFIED**. The 3-wire line level that OPENS robot two's",
        "pneumatic claw: `true` (line HIGH = open) is the addendum's starting GUESS, nothing more — the",
        "team lead reported the port, not the polarity (A4 register HA-159). **Station J.9 settles it:**",
        "the claw must be CLOSED after the start and OPEN on the first press of A. This constant is",
        "BACKWARDS only when the claw is open after the start AND that first press CLOSES it, with the",
        "valve's one cable in A (J.9.0); a claw that does not move on the press says nothing about the",
        "polarity (the port, the air or the valve — J.9.3). **THE ONLY CODE EDIT IS HERE** — the claw's",
        "entry reads it, its start and safe states are POSITIONS, and pneumaticCommandFor() turns every",
        "position into a level through it, so the start command and the safe command move with the flip.",
        "**The flip is a measurement, recorded where the guess is written:** the value is pinned in",
        "test/mechanism_table_test.cpp, as the pivots' signs are, and quoted by HA-159 and Station J.9 —",
        "the pin and both quotes change with it, in the same change."
      ]
    }
  ],
  "controls": {
    "holds": [
      {
        "mechanism": "claw pivot 6",
        "forward": "R1",
        "back": "R2",
        "volts": 12,
        "words": [
          "the claw pivot on port 6: forward / back at full power while held. Let go and",
          "it is put in Hold (whether Hold keeps a loaded claw up is unmeasured, HA-156)"
        ]
      },
      {
        "mechanism": "claw pivot 7",
        "forward": "L1",
        "back": "L2",
        "volts": 12,
        "words": ["the claw pivot on port 7: the same"]
      }
    ],
    "toggles": [
      {
        "mechanism": "claw",
        "button": "A",
        "words": [
          "the claw: each press opens or closes it. The FIRST time driver control starts",
          "in a run, the claw is told CLOSED, whatever autonomous left it at (the team",
          "lead's ruling, 2026-09-26); a program with no autonomous has left it CLOSED,",
          "so there it does not move. After that, a disable and the next start tell it",
          "nothing. Which line level closes it is a guess (HA-159)"
        ]
      }
    ]
  }
}
```

*(re-check 2026-10-29, RC2: the `front` KEY leaves robot two's file — the robot file states no front since 2026-10-06 — so Part 1 writes the file above without it; its sides stay "looking toward the 15/16 end" in the notes and the provenance until D7's flip. The paragraph is kept for its reasoning.)* **Why `front` is the 15/16 end — and what it shows.** The file must reproduce the TYPED table, whose
LEFT is 11–15 looking toward the 15/16 end. Calibrate's record of 2026-09-25 was calibrated with the
camera end (the 11/20 end) leading, so its LEFT is this file's RIGHT: the panel's record check says
`FRONT REVERSED` for robot two today (P8). That is the worksheet's warning
(`docs/internal/chunks/R3a-BENCH-WORKSHEET.md` ~:522-529, "WHICH END IS FORWARD") made visible, not a
defect of the file. **What no check can see:** whether a stated `front` agrees with the stated sides.
That is chirality, and only a measurement settles it, namely the record check. The panel therefore
reads the statement back ("looking toward <front>: LEFT 11 12 13 14 15, RIGHT 20 19 18 17 16"), and a
change of WHICH end is the front goes through `flip-front`, never through the words alone. ~~Whether robot two's file should be re-stated toward the camera end (LEFT 16–20, every
typed sign flipped) is a data change with worksheet consequences — §8, not G3a's.~~ *[superseded 2026-09-29:
the team lead ruled the front per robot and human-stated, and asked for robot two's own front to be put to him
here — **D7**, recommended (a): the camera end, by `flip-front` right after Part 1's differential. Probed N1:
flipped, the file agrees with the record on all 8 of its ports by side AND sign; as typed, on none. The file
above stays as the differential's input. If D7 is (a), its `front` reads "the camera end: the end where the AI
Vision (port 3) and ports 11 and 20 sit (left and right below are named looking toward it)"; its LEFT is 16 17
18 19 20 (`-16 +17 -18 +19 -20`) and its RIGHT 15 14 13 12 11 (`+15 -14 +13 -12 +11`), each side still listed
back to front; its provenance keeps the census, MOTOR WATCH and cartridge facts and adds the flip, within the
banners' 206 characters — "ports + IMU 2 + radio 1: census 2026-09-10; SIGNS: MOTOR WATCH 2026-09-10, negated by
flip-front [date] (not re-measured); front = the camera end (team lead [date]); cartridge BLUE off a motor", 200
characters, probed at the re-check (`recheck2/n1/`); its generated `.front` (Part 2) carries the new words; and
the notes' 15/16 sentences become dated history.]*

~~**The post-1b deltas the executor applies (the tree at its start is the truth; `check` proves them):**
(1) the four pivot/claw provenance strings and `mechanismNotes` as T4 Part 1b rewrote them (T4 brief
~:1007-1011 adds "(buttons: src/team/robot_two_driver.hpp since T4)"); (2) the emitter drops the seven
button/volts lines (P10); (3) nothing else — the `controls` block already holds the buttons 1b moves.
The executor writes the file BY HAND from this one plus those deltas, adopts, and `check` is the proof: a
typo shows as a stale region, never as a silent difference.~~ *[superseded 2026-09-29: T4 Part 1b was folded
into T4 Part 3, and the MECH core then rewrites the mechanism table and the driver file's controls. The METHOD
stands and is applied twice:*
- *Part 1 writes the drivetrain half of this file by hand and adopts `[chassis]`; `check` is the proof.*
- *Part 5 extracts the mechanism half from the tree the MECH core leaves, in §1.14's shape, adopts the four
  regions, and `check` is the proof: zero differences, and the marker lines the only new ones.]*

### 1.2 `tools/robot_tool.py` (Part A — *amended 2026-09-29: Part 1, grown by Parts 2, 4 and 5*) — standard library only, one file, with a `self-test`

> *(re-check 2026-10-29, CK11 (b): "one file" has ONE exception from Part 3 — `tools/robot_record.py`, the record decoder, standard library only, loaded by path from `robot_tool.py`; see the re-check's "The order and the lanes".)*
> **AMENDED 2026-09-29.** Everything below holds, with these additions. *(re-check 2026-10-29, CK2: NOT everything. The chassis half holds. The MECHANISM and CONTROLS passages of this section — the validator table's mechanism and control rows, the stricter refusals' "≠ 1 motor / ≠ 1 line" and `k<Name>OpenCommand` / polarity collisions, the baseline rule's mechanism value lines, the `openLevel*` and "claw pivot" warnings, "the polarity region is ALWAYS present" — are the RETIRED 09-29 vocabulary. Each is marked where it stands; what replaces them is the re-check's adoption table (MECH §6.3 as landed) and its ~128 landed refusals (`spec.hpp`'s 70, `wiring.hpp`'s 16 + 9, `route_ids.hpp`'s 4, `driver_inputs.hpp`'s 29, plus `DriverFile`'s re-zero `static_assert` — RC6, CK9).)*
> - **New commands:**
>   - `new-robot --id ID --name TEXT --kind tank --cartridge red|green|blue --front TEXT --provenance TEXT
>     [--left P… --right P… --imu P] [--programs …] [--dry-run --json]` (Part 2; §1.13; *re-check 2026-10-29: no `--front`, RC2*);
>   - `mechanism add FILE --template T --name N`, `mechanism remove|duplicate|rename|move FILE …`, each with
>     `--dry-run --json` listing its EFFECTS (Part 5; §1.14);
>   - the hooks `import-vexbot` and `export-vexbot`: reserved names, NOT built (§1.16).
> - **`describe --json`** also returns the templates with their settings (Part 5), the program names and the
>   region kinds, the SET ones included.
> - **Region kinds.**
>   - ROBOT regions, each owned by one robot file:
>     - `chassis` (robot two, Part 1);
>     - `pneumatic polarity`, `mechanisms`, `control words`, `controls` (robot two, Part 5, in the MECH core's
>       shape); *(re-check 2026-10-29: THREE — `mechanisms` (MECH's one block), `control words`, `controls` (the `kControls` statement only); no polarity region — RC4, RC5)*
>     - a generated robot's `control words` and `controls`, in its own driver file (Part 4 — moved from Part 2
      at the re-check, #2).
>   - SET regions, owned by `src/robots/*.robot.json` together (D9):
>     - `robot guards`, `robot ids`, `robot list`, `chain`, `robot beacons` (`src/build_selection.hpp`);
>     - `make robots` (`Makefile`);
>     - `chassis tables`, `chassis selection` (`chassis_table.hpp`);
>     - `mechanism selection` (`mechanism_table.hpp`);
>     - `gate builds` (`tools/src_build_gate.py`);
>     - from Part 4: `driver files` (`src/team/drivers.hpp`) and `auton selection` (`src/team/autons.hpp`);
>     - from Part 5: `mechanism tables` and the MECH core's handles file. *(re-check 2026-10-29: no handles file — RC4)*
>   *(Part 4 also HAND-edits `tools/api_doc_tool.py`, which is not a region: §1.5.)*
> - **`check`'s ownership rule becomes per part.** Robot two owns `chassis` from Part 1 and its four
>   mechanism/control kinds from Part 5. A generated robot owns its driver file's two kinds from Part 4. Every
>   SET kind exists EXACTLY ONCE in the tree from Part 2 on, generated from the set. All but `[robot guards]`
>   are empty with no generated robot.
> - **The hand-written robots, read from the Makefile (D9; re-check #9).** `[robot guards]` names every robot's
>   define. The tool takes the hand-written robots' defines and program lists from the Makefile's
>   `SHULIB_ROBOT_DEFINE_<robot>`/`SHULIB_ROBOT_PROGRAMS_<robot>` lines outside the region, which the Makefile's
>   own `ROBOT` and `PROGRAM` blocks read. It never types one.
>   - A define line with no programs line (or the reverse) is a `ToolFault`.
>   - The region is stale, never "edited by hand", when one of those lines changed and `generate` has not run.
>   - The marker says "GENERATED from src/robots/*.robot.json and the Makefile's hand-written robots".
>   - Probed: `recheck2/guards/`.
> - **The scan.** The tool scans `src/` and the two NAMED files outside it that carry SET regions, the
>   `Makefile` and `tools/src_build_gate.py`. It never scans anything else: its own source holds the marker
>   text.
> - **Marker prefixes.** `//` in C++; `#` in the Makefile and in Python.
> - **The invariant's second half for Python.** Before it writes `tools/src_build_gate.py`, `generate` runs
>   `compile()` over the new text; a failure is a `ToolFault` and nothing is written. The Makefile's region
>   holds only `NAME := value` lines built from validated ids and defines, which cannot contain `$`, `#` or
>   `\`.
> - **The collision rules grow** (§1.13): a generated id whose make value, define, `RobotId`, table name,
>   driver class or ANY of its builds' beacons equals another's, the hand-written robots included. The define
>   is checked on its own, since for `tank_2026` and `xdrive_invented` it is the ONLY name that collides
>   (re-check #11; S22, M27).
>   *(re-check 2026-10-29, CK8:)* **The robot's NAMESPACE is a derived name too** — its id, under `shulib::bench`
>   (MECH-A5's per-robot namespace, `_robot_namespaces`; a robot's own team files are `src/team/<id>_*`). The id is
>   REFUSED when: it is a C++ keyword or alternative token (`new`, `class`, `delete`, `and`, `not`, … — C++20's list,
>   carried by the tool); it names a namespace `src/` reaches UNQUALIFIED from inside `shulib::bench` (`std`, `pros`,
>   `hal`, `units`, `mechanism`, `config`, `robot`, `teleop`, `calibration`, `competition`, `chassis`, `diag`, `motion`,
>   … — DERIVED by scanning `src/` for `<name>::` and `include/shulib/` for `namespace <name>`, never a hand list; V8:
>   `std`, `hal`, `robot` break `src/`'s own `std::` / `hal::` / `robot::`); `<id>_` begins another robot's id or the
>   name of another robot's existing `src/team/` file, or another robot's `<id>_` begins this one (`robot` against
>   `robot_two`: A5's `src/team/<ns>_*` ownership would give `robot_two_driver.hpp` to both — by derivation; A5's gate
>   code is re-read at Part 2's tip). The id pattern tightens to `[a-z][a-z0-9]*(_[a-z0-9]+)*` (no `__`, which C++
>   reserves, no trailing `_`).
> - **The literal-length rule (re-check #13, #3):** a literal the brain prints on a fixed-size line must fit it:
>   `drivetrain.provenance` ≤ 206 characters (Part 1), `drivetrain.front` ≤ 160 (Part 2) *(re-check 2026-10-29: `drivetrain.front` is gone (RC2); ADDED: a generated `changeAt` path ≤ **160** characters, the `%.160s` that prints it in every generated sentence of the direction check (`direction_check.hpp` :168, :224, :242, :272, :329 — Part 5, RC8; CK7 corrects the first pass's "130", which is the `wire(...)` place's cut; V7); the 206 is Bench Tests' line, `bench_r3a.cpp` :196/:3200 (RC15))*. The limits are
>   re-read from the programs' buffers at each part's tip, and the tool names the line it protects.

**Commands** (the list completed at the re-check, #13 and #15.4):
- `validate [FILE|- --path REL] [--json]`: refusals and warnings. Each refusal is ONE sentence that
  names every C++ check that would refuse it and the JSON path it is about. The tree is the repo root
  found from the file's path. `--path` names the file when the text comes on stdin, as the panel sends
  it. It applies the baseline rule below.
- `generate [FILE...] [--discard-hand-edits] [--dry-run --json]`: applies the baseline rule, and prints
  the WITNESSES it finds for every region it changed (below).
- `check`: the freshness gate, over every `src/robots/*.robot.json` and every marker under `src/`. It
  never applies the baseline rule: after a generate there is nothing to compare, and a stale region
  fails anyway.
- `fmt FILE|-`: the canonical writer.
- `adopt FILE --lines B-E --robot ROBOTFILE --region KIND`: the one-time wrapping of existing lines,
  computing their sha. Part A uses it five times, and PKG1 reuses it.
- `describe --json`: the vocabularies (buttons, cartridges, safe modes, levels, 3-wire letters, sensor
  kinds, the limits, the region kinds), so the panel hard-codes none.
- **`flip-front FILE --provenance TEXT [--front TEXT]`**: the ONE way to change WHICH end is the front. *(re-check 2026-10-29: no `--front` — the file states no front words (RC2); `flip-front` swaps the TYPED sides and negates the typed signs: robot two's D7, and any typed robot whose CALIBRATION page or Calibrate says FLIPPED / FIX YOUR TYPED DRIVE PORTS)*
  It swaps `left` and `right`, negates every `typedTable` sign, and sets the provenance (and the
  front's words, when given). Flipping twice restores the sides and the signs byte for byte. It is the
  panel's "the OTHER end is the front" button (#15.4).
- **`agreement-fixtures OUT.hpp`** (§1.5, §1.6): the fixtures and the validator's verdicts, as data.
- `record FILE [--robot ROBOTFILE] [--json]` and **`record-fixtures OUT.hpp`**: Part B, §1.7.
- `self-test`.

**The marker protocol** (prototype-proven; the exact text):
```
// >>> GENERATED from src/robots/robot_two.robot.json [controls 958c317aa3] -- edit that file or the robot panel
    ...the generated lines...
// <<< GENERATED [controls]
```
- The begin marker names the robot file, the region KIND (`chassis`, `pneumatic polarity` *(CK2: retired — no polarity region, RC4)*, `mechanisms`,
  `control words`, `controls`) and the first 10 hex digits of the SHA-256 of the region's lines as last
  generated; the indent is the region's (`controls` sits inside the class, 4 spaces).
- **The tool is MARKER-DRIVEN:** it finds regions by scanning `src/` for markers, never from a
  hard-coded path list, so PKG1's starter (and any file adopted later) works unchanged (§1.10). `check`
  also requires every robot file to own each of the five region kinds exactly once *(re-check 2026-10-29: robot two owns `chassis`, `mechanisms`, `control words` and `controls`; a generated robot owns its driver file's two and lives in the SET regions; there is no polarity region — RC4)*. ~~**The
  `pneumatic polarity` region is ALWAYS present, and empty when the robot has no pneumatic** (re-check
  #15.1). Probed: removing the claw leaves an empty region, and generate, check and the compile are
  clean. So adding the first pneumatic, or removing the last, never needs a hand edit of markers.~~ *(re-check 2026-10-29, CK2: struck — there is no polarity region; a pneumatic's levels are its `positions` inside the `mechanisms` region, which is present whenever the robot has a mechanism list (MECH §1.1, `controls(mechanisms())` with none compiles, V2).)*
- **`generate` never overwrites a hand edit:** lines whose sha no longer matches their marker are
  refused, naming the lines, unless `--discard-hand-edits`; and it is **all or nothing** across every
  file (a hand edit anywhere, or a refused robot file, writes nothing anywhere — the v2 prototype wrote
  region by region; the re-check's plans every file first, then writes).
- **THE generate⇒check INVARIANT (re-check #1):** before it writes, `generate` runs `check`'s own
  comparison over the NEW text in memory. The markers must parse, each region must equal what the file
  generates, and each sha must match. A failure there is a `ToolFault` (a tool bug), and nothing is
  written. The emitters also refuse text the validator should have refused (`ToolFault`), and `cstr()`
  escapes anything unprintable, so a literal is always one line of valid C++.
- **A marker whose sha is wrong over lines that ARE what the file generates is NOT a hand edit**
  (re-check #11; a merge or a hand-typed marker does this). `check` fails with "(marker only) ... run
  generate", and `generate` rewrites the marker alone. The converse is named rather than prevented: a
  hand edit whose sha someone recomputed reads as "stale", and generate overwrites it from the robot
  file, which is the truth.
- **The witnesses (re-check #3):** for each region it changed, `generate` prints every test source
  (`test/*.cpp`, `test/*.hpp`) that names this robot's tables or its driver file, and every public document (`docs/` outside `docs/internal/`,
  `README.md`) holding one of the region's OLD lines (compared with the indentation stripped, as
  `check-examples` compares them), with "run the host suite and
  `api_doc_tool.py check-examples`; a quoted listing is re-quoted by hand from the new source". The
  list is derived by scanning, never hard-coded. Probed: for robot two it finds six test TUs and
  `docs/cookbook/06-driver-control.md`.
- **`check` fails, naming file and lines:** a hand edit ("the lines inside [controls] were EDITED BY
  HAND ... edit the robot file or the panel"), a stale region ("is not what src/robots/... generates ...
  run python3 tools/robot_tool.py generate"), a missing, nested, duplicated or malformed marker (any line
  holding `>>> GENERATED`/`<<< GENERATED` not in the exact form), a region naming a robot file that does
  not exist, a robot file refused by the validator, and a robot file not in canonical form.
- **What a student may still hand-edit around the regions:** everything outside the markers — the
  header's other lines (~~the sticks, the re-zero line,~~ the protections paragraph *(fix 2026-10-30, C11: the sticks,
  the re-zero and the release lines are inside the generated `control words` region now — the evening amendment's §1)*),
  THE FEEL *(amended 2026-10-29 evening: no longer — a generated `feel` region, D11)*, HOW IT RUNS,
  THE CHECKS. Inside: nothing.

**The validator — every C++ refusal, named first** (P6's harness is the proof; the table is the tool's
header comment and its self-test's list):

> *(re-check 2026-10-29, CK2:)* **Only the first two rows and the last row below hold** (the chassis: `tableConsistent`,
> `describeMissing`, the selection pin). **Every MECHANISM and CONTROL row is RETIRED** — the old `mechanism_table.hpp`
> checks 1-3 and 8-9, `mechanismTableProblem()`, `RobotConfig::valid()`'s mechanism half as the table fed it, and
> `driver_controls.hpp`'s `controlButtonsDistinct` / `controlsAvoid` / `holdVoltsValid` / `controlsNameTable` /
> `controlsCoverTable` (that header is RETIRED by MECH-A5). Their replacements are the LANDED refusals the adoption
> table names: `specProblem()` (`spec.hpp`, 70), `wiringProblem()` (`wiring.hpp`, 16 entry problems and 9 list
> sentences; its ~50 compile-time `refused::` names are the same rules at build), `route_ids.hpp`'s 4, and
> `controls(...)`'s 29 `refused::` names (`driver_inputs.hpp`) plus `DriverFile`'s re-zero `static_assert` (CK9).
> Part 5 writes the new table from those headers AT ITS TIP, one row per refusal family.

| Mistake in the file | C++ check(s) that refuse it | Where |
|---|---|---|
| a drive port out of 1..21, twice on a side, on both sides, the IMU on a drive port, a signed table with an empty side, a geometry value < 0 / non-finite | `tableConsistent` | `chassis_table.hpp` :449-519 (runtime: DRIVE, the old drive program) |
| no LEFT/RIGHT ports, no cartridge | `describeMissing` | :295-308 |
| a mechanism port out of range / declared twice | table check 3 + `RobotConfig::valid()` | `mechanism_table.hpp` :386-397; `robot_config.hpp` :578+ |
| a mechanism motor on a drive or IMU port | table check 8 + `RobotConfig::valid()` | :483-492 |
| a 3-wire line twice | table check 9 + `RobotConfig::valid()` | :399-410 |
| more entries than the table holds; a declared field unset | checks 1-2, and `mechanismTableProblem()` which calls them | :350-384; `boot_source.hpp` :125-147 |
| > 8 mechanisms; a name > 23 chars | `mechanismTableProblem()` | `boot_source.hpp` :132-145 |
| two mechanisms with one name; motors AND a line | `RobotConfig::valid()` (boot only — no static_assert sees it) | `robot_config.hpp` :545-567 |
| a button twice; a hold's forward = its back | `controlButtonsDistinct` (+ pre-1b checks 4-5) | `driver_controls.hpp` :136-150 |
| a control on the re-zero button | `controlsAvoid` (+ pre-1b check 6) | :152-163 |
| volts ∉ (0, 12] | `holdVoltsValid` (+ pre-1b check 7) | :165-175 |
| a control naming a missing mechanism / the wrong kind / twice | `controlsNameTable` | :197-228 |
| a mechanism no control names | `controlsCoverTable` | :230-255 |
| robot two's table not signed (after D5 (a): the counts are gone) | the selection pin's non-address half, ONE named `constexpr` predicate that the `static_assert` and the agreement test both call (re-check #7) | `chassis_table.hpp` :230-233 (§1.4) |

**The tool's own, stricter refusals.** The C++ cannot see these, and each refusal says so:
- a port a SENSOR claims (the radio, the camera);
- a typed track width or ratio (the ruling);
- an unknown enum spelling (the generator never spells a value it does not know: P6's schema-level
  fixtures);
- ~~a motor mechanism with ≠ 1 motor, or a pneumatic with ≠ 1 line (the table holds one; `RobotConfig`
  would take more, and the refusal says so);~~ *(re-check 2026-10-29, CK2: STRUCK — it would refuse MECH §6.3's two-motor lift and three-position (two-line) pneumatic; the landed limits are `kinds.hpp`'s — one to four motor roles, one or two lines — refused by `specProblem()` / `wiringProblem()`, mirrored, never made stricter);*
- **THE TEXT-FIELD RULE** (§1.1; the ASCII half is the old "non-ASCII name or provenance", since the
  brain prints them and R2a found a UTF-8 dash reaching a panel);
- **`typedTable` on any robot file but `robot_two`** (re-check #2);
- **derived C++ names that collide** (re-check #6): ~~two pneumatics whose names give one
  `k<Name>OpenCommand`; a pneumatic name with no letter or digit;~~ *(CK2: STRUCK — no `k<Name>OpenCommand` and no polarity constants exist after MECH-A5; a mechanism's C++ identifier is lowerCamelCase from its name or its `id` (MECH §6.3), refused when two give one identifier or a name gives none — the per-robot namespace keeps robots apart)* and any derived name (`k<Id>Chassis`,
  `k<Id>Mechanisms`, ~~a polarity constant~~, the namespace — CK8) already defined outside this robot's regions, whether by
  another robot's regions or by a hand-written name. Probed: ids `bench_bot` and `no` are refused
  against `kBenchBotChassis`/`kNoChassis`;
- **THE BASELINE RULE: a changed fact needs a changed provenance** (re-check #5; `validate` and
  `generate` only). *(re-check 2026-10-29, CK2: the drivetrain's value and provenance lines below hold. The MECHANISM half — "each polarity constant", `.signedPort`, `.safeMode`, `.adiLine`, the `= true/false;` of a polarity constant, `openLevelNotes` — is RETIRED: Part 5 re-states it against MECH-A5's form, a mechanism's VALUE lines being its kind's members and its `wire(...)` line, its PROVENANCE its `notes` (the region's comments); a swap of a pneumatic's `positions` levels is the measurement HA-159 / HA-185 and Station J.9 settle (MECH §1.13: "swap `Low` and `High` on the claw's `.positions` line"). Controls stay uncovered.)* The baseline is this robot's regions in the tree whose sha verifies, which is
  exactly what the robot's C++ carries now; no git and no second copy are needed. The SAME function
  splits the old region text and the newly generated text into entries: the drivetrain, the typed
  geometry, each mechanism by name, and each polarity constant. For each entry it separates VALUE lines
  (`.left`, `.right`, `.cartridge`, `.imuPort`, `.measured`, `.signsMeasured`; `.signedPort`,
  `.cartridge`, `.safeMode`, `.adiLine`, `.start`, `.safe`, and pre-1b the buttons and volts; the
  `= true/false;` of a polarity constant) from PROVENANCE lines (`.provenance`/`.geometryProvenance`
  literals; `openLevelNotes` for a polarity). An entry whose value lines changed while its provenance
  lines did not is refused: "<entry>: <fields> changed from what the robot's C++ carries, and its
  provenance did not — say who stated the new value, and when (a value is never invented)". Robot two's
  `.wheelDiameterIn` may not change at all (frozen legacy, #2). The rule does not cover renamed or new
  entries, which have no baseline and must simply carry a provenance. It does not cover controls
  (post-1b a button is the team's choice and has no provenance field), and it does not cover words.

**Warnings, never refusals:**
- an unknown field **at any level** (re-check #15.6: the v2 prototype warned only at the top level);
- ~~`openLevelVerified: false`;~~ *(CK2: retired with `openLevel`)* the `typedTable` block is legacy;
- ~~**a new motor mechanism is called "claw pivot" by the brain** (`calibrate_program.cpp` :1131 and
  `boot_source.hpp` :106 print that word for every declared motor; §8);~~ *(re-check 2026-10-29, CK2: moot once MECH-A5 and CAL1-B land — A5's boot lines name each mechanism by its own `name` (MECH §1.13) and CAL1-B replaces `calibrate_program.cpp` with its glue; Part 5 keeps the warning only if a "claw pivot" print survives at its tip)*
- ~~an `openLevel` flipped WITH its notes: its test pin, HA-159 and Station J.9 change with it, because a
  flip is a measurement. Without new notes, the flip is refused by the baseline rule.~~ *(CK2: re-stated — a swap of a pneumatic's `positions` levels WITH new `notes` warns that its test pin, HA-159 / HA-185 and Station J.9 change with it; without new notes the baseline rule refuses it)*

**Pins outside the regions are listed in the tool, not hidden in it:** each robot-specific rule the
validator mirrors (after D5, robot two's selection pin, through its named predicate) carries its file and
an anchor string. The self-test FAILS if the anchor is gone, so a pin removed from the C++ is removed
from the tool too, never left behind as a phantom refusal. The agreement test EVALUATES the same
predicate on every fixture (§1.6).

### 1.3 The five regions, adopted (Part A — *amended 2026-09-29: across Parts 1, 2 and 5*)

> *(amended 2026-10-29 evening)* **Three more regions, each adopted with `check` passing and its differential logged:** robot two's
> `feel` (Part 5, D11 — THE FEEL's heading, `kFeel` and `kLoop`, byte for byte); `[chassis tuning]` at the END of robot
> two's chassis initializer (Part T, D12 — adopted EMPTY: robot two states no tuning); `[routes]` in
> `src/team/autons.cpp` (Part R1, D13 — adopted EMPTY, with ONE hand-written call line outside it). Part S's
> `.camera` / `.wallSensors` adoption becomes a `[chassis sensors]` region or an ADJACENT end marker moved (§3 of the
> evening amendment). *(fix 2026-10-30: `[chassis tuning]` holds ONE constexpr lambda assigning the stated fields onto
> a default `ChassisConfig` — never a designated initializer of a nested group (C1, probe F1); robot two's `control
> words` region spans the header's whole stick-and-button map, :17-32, written from `controls` AND `driving` (C11);
> `[routes]` sits ABOVE `registerAutons()`, after the hand-written autons' anonymous namespace, in its own (C13, probe
> F13) — at the namespace's end it does not compile.)*

> **AMENDED 2026-09-29.** The method below is unchanged: adopt, `check` with zero differences, the differential
> logged with each file's sha256 before and after, and only THEN the prose pass. It now runs three times.
> - **Part 1:** robot two's `[chassis]` (`chassis_table.hpp` :89-140 at `ee2e399`, re-read at the tip). If D7
>   is (a), the flip is a SEPARATE, logged step after this differential. *(re-check 2026-10-29: :92-143 at `73044ca`, bytes identical; the region ENDS before any member VIS1-B or L6-B appends — RC9, V3)*
> - **Part 2:** the SET regions, adopted before any robot is generated (empty, except `[robot guards]`, which
>   holds the hand-written robots' guards read from the Makefile — D9, re-check #9). The one-time hand edits
>   come first:
>   - the guards in their general form;
>   - the Makefile's hand-written robots in the generated shape, with its `ROBOT` block reading them;
>   - the Makefile's generated-robot branch and its reading of the program lists;
>   - the gate's merge lines;
>   - `kEveryRobotId` with the tests reading it;
>   - ~~`ChassisTable::front`.~~ *(re-check 2026-10-29: struck — CAL1 §1.7, RC3)*
>
>   The zero-robot emission must then equal the adopted bytes (probed N3, and `recheck2/guards/` for the
>   guards read from the Makefile: the gate passes). Each hand edit is its own listed diff in the PROGRESS log.
>   ~~Robot two's `[chassis]` region then gains its `.front` line through `generate`, a logged one-line diff.~~ *(re-check 2026-10-29: struck, RC3)*
> - **Part 5:** robot two's four mechanism/control regions, from the tree AS THE MECH CORE LEAVES IT. *(re-check 2026-10-29: THREE regions, and Part 5 now runs before Part 2 — RC4, RC5)* "Post-1b,
>   `mechanisms` is seven lines shorter" below is superseded: the core rewrites those bytes, and Part 5 reads
>   them fresh.

In order: `adopt` each block (the exact lines of §0.1, re-read at the tip — post-1b, `mechanisms` is seven
lines shorter), then `check` MUST pass with zero regeneration differences — **that run's output is the
differential, pasted into the PROGRESS log with the sha256 of each file before and after**. Only THEN the
prose pass (§3's list), as separate, listed edits: the in-region prose through the robot file (e.g.
`openLevelNotes`' "THE ONLY CODE EDIT IS HERE" becomes "the only edit is `openLevel` in
src/robots/robot_two.robot.json"; the `.openCommand` trailing comment "flip the CONSTANT" likewise — the
emitter's fixed text changes with it), the out-of-region prose by hand. *(re-check 2026-10-29, CK2: the examples are RETIRED vocabulary — after MECH-A5 there is no `openLevelNotes` and no `.openCommand`; the claw's caveat becomes its `notes` in the robot file, and its one edit "swap each position's levels" on the claw's `positions` in src/robots/robot_two.robot.json (MECH §1.13's HA-159 text, re-pointed). Part 5 lists the in-region prose from A5's block at its tip.)*

### 1.4 The selection pins (Part A, D5 (a) — *amended 2026-09-29: the chassis pin in Part 1; the mechanism pin with the MECH core or Part 5*)

> **AMENDED 2026-09-29.**
> - **The chassis pin's rewrite is Part 1's**, as written below. It still holds after D7's flip, since the
>   flipped table is still signed with five motors a side.
> - **The mechanism pin's rewrite** (`mechanism_table.hpp` :619-623) is in the file the MECH core rewrites. *(re-check 2026-10-29: DONE by MECH-A5 — the address only, and the hand-written robots that declare none by name (MECH §6.2 items 4 and 6); nothing is left for Part 5, RC19)* The
>   coordinator asks the core's brief to leave that pin as the address check alone (D5 (a)). If the core keeps
>   the counts, Part 5 drops them.
> - **A generated robot gets NO per-robot pin.** Its table and its selection case are generated from one file
>   and checked fresh. The generic `&kChassis == &chassisTableFor(kRobotId)` pin (:224-225) and the host tests'
>   per-robot loop over `kEveryRobotId` cover it (probed N2).

`chassis_table.hpp` :230-233 becomes `kRobotId != RobotId::RobotTwo || (&kChassis == &kRobotTwoChassis
&& robotTwoTableSelectable(kChassis))`. Here `robotTwoTableSelectable(t)` is a `constexpr` predicate,
declared beside the pin, that returns `t.signsMeasured`; the name is the probe's, and the executor may
choose a better one. The agreement test calls the SAME predicate on every fixture (§1.6; re-check #7,
probed in `recheck/agree_pin/`). The pin's comment (:226-229) says why the counts went: the robot file
owns them, and the address check is the selection check. `mechanism_table.hpp` :619-623 becomes
`kRobotId != RobotId::RobotTwo || &kMechanisms == &kRobotTwoMechanisms`, its message without "(the claw
pivots on 6 and 7 ...)", its comment (:613-618) likewise. The host tests that restate the counts
(`chassis_table_test.cpp` ~:573+, `mechanism_table_test.cpp` ~:237/:289) are D5's literal pins — kept.

### 1.5 The gate and CI (Part A — *amended 2026-09-29: Part 1, grown by Parts 2 and 4*)

> **AMENDED 2026-09-29.** Part 1 does everything below. Then:
> - **Part 2:**
>   - the gate's tables gain the `[gate builds]` SET region and the hand-written lines that merge it (D9;
>     probed N2 and N6);
>   - **`TABLE_FILES` stays unchanged**: a generated robot's tables are written into the two table files and
>     derived from them (N2: "8 per-robot names");
>   - the doc gate's DEPENDS gains the `Makefile` and `tools/src_build_gate.py`, since they carry regions;
>   - **the gate's self-test gains a case that runs `robot_tool.py new-robot` into a temp copy of the tree,
>     then step 0 and that robot's bring-up builds**, so CI builds the generated path while the repository
>     carries no generated robot (N8: none of today's 190 cases builds one). About +40 s.
> - **Part 4:**
>   - *(re-check 2026-10-29: on MECH-A5's per-robot namespace rule, `_robot_namespaces` / `TEAM_FILE_ROBOT` — RC11)* step 0 derives the per-robot DRIVER classes (each `DriverFor<RobotId::X>` specialization) and AUTON
>     functions (each `autonsFor()` case). It refuses one named outside `src/team/`, outside the selection and
>     outside a `static_assert`, exactly as it refuses a table named outside `TABLE_FILES`. Its self-test gains
>     the plants: a program naming `RobotTwoDriver` directly → FAIL; the same name inside a `static_assert` →
>     pass;
>   - the generated-robot case builds the three match programs too;
>   - **`tools/api_doc_tool.py`: the bound listings (re-check 2026-09-29, #1).** A listing of a WHOLE-RUN file
>     names the file it quotes with `<!-- example-source: PATH -->` on the line above its fence, and is checked
>     against that file ALONE: every line a line of PATH, and one unbroken run.
>     - The whole-run files (`WHOLE_RUN_SOURCE_GLOBS`) leave the membership union `known`, so a stale quote of one
>       driver file can no longer pass because another driver file (a generated starter repeats robot two's
>       feel and bind/run lines verbatim) holds the same line.
>     - An unnamed listing whose lines only a whole-run file holds fails, with a hint naming the file.
>     - A directive naming a file no `EXAMPLE_SOURCE_GLOBS` entry matches fails.
>     - The example tests keep today's rule (membership).
>     - **COMP-t made `src/team/autons.cpp` a whole-run kind too.** It was merged into the main tree while this
>       re-check ran (staged over `a5bd83f`; `WHOLE_RUN_SOURCE_GLOBS = ["src/team/*_driver.hpp",
>       "src/team/autons.cpp"]`), so the rule covers it. A generated robot's `src/team/<id>_autons.cpp` does not
>       match that exact glob, so it is no example source; were the glob ever widened, the bound rule would
>       already hold.
>     - **Five listings take a directive at the COMP-t tip:** *(re-check 2026-10-29: SIX at `73044ca` — chapter 17's one too (V4); MECH-A5 rewrites cookbook 06's three; Part 4 re-reads — RC12)* cookbook 06's three (its WHICH BUTTON, feel and
>       how-it-runs listings, of the file its :213 links) and the auton chapter's two (of
>       `src/team/autons.cpp`). Probed on the staged tree:
>       - unnamed, all five fail (`recheck2/tip_bound_scan.log`);
>       - with the five directives, the scan passes: "5 listings name their file; 5 whole-run listings"
>         (`tip_marked_bound_scan.log`);
>       - a stale feel with a starter present passes TODAY's rule ("4 listings", rc 0) and fails the bound one
>         (`tip_stale`).
>     - The rule's header comment (:1833-1853) and `do_check_examples()`'s docstring say why.
>     - Probed with a stand-alone scan, `docscan/bound_scan.py`: today's tree fails the three unnamed listings
>       by name; named, it passes; a stale feel with a starter present fails (today's rule passes it); a bad
>       directive is refused.
>     - It lands with, or before, the first starter driver file, so no commit ever carries a second driver file
>       under the old rule. Its self-test cases are S33 in §2.
> - **The gate's time grows** by one ARM build per program per generated robot: ~11 s each here (8 builds
>   took 2 m 25 s and 6 took 2 m 03 s on this machine), so ~1 min per robot with five programs.

- **`test/CMakeLists.txt`**: the doc-gate command (:243-285) gains `robot_tool.py self-test` and
  `robot_tool.py check`, and DEPENDS on `tools/robot_tool.py` and **every file under `src/`** (one
  `file(GLOB_RECURSE … CONFIGURE_DEPENDS src/*)`). The tool is marker-driven, so any file may hold a
  region, and a list of "the three region files" would be the gate's blind spot (re-check #15.3). A
  second custom command writes
  `${CMAKE_CURRENT_BINARY_DIR}/generated/robot_tool_agreement.hpp` (`robot_tool.py agreement-fixtures`:
  the fixtures AND the validator's verdicts, as data) for `test/robot_tool_agreement_test.cpp` (§2).
- **`tools/src_build_gate.py`**: step 0 gains `[generated]` — it IMPORTS `robot_tool` and runs its check
  (one implementation), so the robot-build gate the coordinator runs every verification refuses a stale or
  hand-edited region too; its self-test gains a planted hand edit and a planted stale region (plants
  restored, like cases 26-28). **`TABLE_FILES` unchanged** (D2 (a)).
- **`.github/workflows/ci.yml`**: the documentation step (:77-86) gains `python3 tools/robot_tool.py
  self-test` and `check`.
- **`make` is untouched**: PROS's build never runs Python (§1.9).
- `tools/api_doc_tool.py`: ~~**no change** — `check-examples` passes on the adopted tree (P2).~~ *[re-check
  2026-09-29, #1: no change in Part 1 (P2 still holds: the markers sit outside cookbook 06's listings); Part 4
  changes `check-examples` to bound listings, above.]*

### 1.6 The agreement test (Part A — *amended 2026-09-29: the chassis half in Part 1, the mechanism half in Part 5*) — `test/robot_tool_agreement_test.cpp`

> **AMENDED 2026-09-29.**
> - **Part 1** builds the harness with the drivetrain fixtures: `tableConsistent`, `describeMissing`, the
>   pin's predicate, the text-field refusals, and `typedTable` on another robot.
> - **Part 2** adds the new-robot collisions to the schema-level list: an id against `tank`/`bench`/`xdrive`,
>   a beacon equal to another build's, and a define equal to another robot's (`tank_2026`, re-check #11).
> - **Part 5** re-grows the mechanism fixtures against the MECH core's C++: *(re-check 2026-10-29: the spec, wiring and route-id fixtures at RUN time through `specProblem()` / `wiringProblem()`; the CONTROLS fixtures as a compile-refusal set — each a TU expected to fail naming its `refused::` function (MECH-A1's harness) — RC6, V1, V2)* its binding checks, its kinds'
>   refusals (a non-monotonic lift table, limits the wrong way round, a preset outside its limits, a role
>   unbound, a line state no position names), and the controls predicates as the core leaves them.
> - The P6 fixtures written against the pre-core table are history.

Includes the generated header: ≥ the 31 probed fixtures (P6), each one mutation of robot two's file,
emitted UNCHECKED, plus the validator's verdict per fixture *(re-check 2026-10-29, CK9: the controls half also carries ONE `DriverFile<kControls, kFeel, kLoop>` fixture with a control on THE FEEL's re-zero button, expected to fail on `DriverFile`'s `static_assert` (MECH N6, n57) and matched by its message text — the controls refusal that is not a `refused::` name, and the one the tool mirrors by READING the re-zero, D4)* *(amended 2026-10-29 evening: the re-zero is STATED in the file's `driving` block since D4's reversal, D11)*; for each emittable fixture the test runs the
real checks (as `agree_runner.cpp` does) **and the selection pin's predicate** (`robotTwoTableSelectable`,
§1.4 — every fixture is robot two), and CHECKs that the C++ families equal the validator's, the pin's
family included (the v2 harness dropped it: re-check #7; `recheck/agree_pin/`: fixture 24 AGREES, and
DISAGREES when the validator's pin is removed); the schema-level fixtures are listed by name — now
including the text-field refusals, the collisions and `typedTable` on another robot. **Hostile but LEGAL
text** is emitted too (a `*/`, a `/*`, a quote, a mid-word backslash, a `?` — in a comment and in a
literal), so the host build COMPILES what the text-field rule lets through (probed:
`recheck/bat/benign_comment_text` compiles under the suite's flags). Every vocabulary value (twelve buttons, three cartridges, three
safe modes, eight lines) is spelled by at least one fixture, so a spelling the C++ does not have fails the
host build. Post-1b, the pre-1b table checks 4-7 leave the harness.

### 1.7 The robot panel (Part B — *amended 2026-09-29: the shell in Part 3, the mechanism pages in Part 6*) — `tools/robot-panel/`, an OPTIONAL VS Code extension

> **AMENDED 2026-09-29.** Everything below holds: the layout, the pinned toolchain, the custom editor over
> the JSON, one writer, diagnostics by JSON path, the record view, the Python setting and the room for H4. It
> is split into PAGES of one form.
> - **Part 3 — the shell and three pages:**
>   - **NEW ROBOT.** ~~The four §4.1 facts (name, kind, cartridge, front)~~ *(re-check 2026-10-29: THREE — name, kind, cartridge; and "declare every mechanism motor before Calibrate, or unplug it", CAL1 R11)* and the id, with the ports optional
>     (D10). The kinds come from `describe`, so the choices follow §5 Q22. The page runs
>     `new-robot --dry-run --json` and shows every file and region it will write (the list in the amended
>     section) before Create; Create runs it for real, all or nothing.
>   - **DRIVETRAIN.** *(re-check 2026-10-29: no front words, no read-back; `flip-front` is offered only for typed sides the CALIBRATION page finds FLIPPED — RC2)* The cartridge, the front's words with the read-back ("looking toward <front>: LEFT …,
>     RIGHT …"), "the OTHER end is the front" (→ `flip-front`, with the provenance the user types), the ports
>     per side, the IMU and the sensors; `typedTable` shown read-only (robot two only).
>   - **CALIBRATION.** *(re-check 2026-10-29: the record AND its backup (`.bin`, `.bak`), `record_store.hpp`'s fallback rule mirrored; the front comparison is `compareFronts` mirrored as ONE rule pinned against the C++, saying what FLIPPED cannot tell (a cable swap, CAL1 R10) — RC3)* The record, read-only, as below, with the disagreement warning (§4.5): FRONT REVERSED,
>     ports LEFT OUT, ports the file lacks, cartridge, IMU.
>     - *(re-check 2026-10-29: the words become "no typed ports — nothing in this file to compare; the record's front is the end that led its push, and Calibrate warns when a new record flips it", RC2)* **A file with no typed ports** (D10 (a)) gets no front verdict. The page says so instead: "no typed
>       ports: this file's front cannot be checked against the record. The record faces the end the operator
>       answered FRONT FIRST to at Calibrate, which printed this file's front words before the push" (re-check
>       2026-09-29, #3).
>     - A portless file never shows FRONT REVERSED; its silence is not agreement.
>     - *(re-check 2026-10-29, CK10)* **When a record's front is opposite to the previous record's** (`compareFronts`:
>       FLIPPED), the page also names every FRONT-RELATIVE fact the file states — the camera's `forward`, `left` and
>       `yawDeg`, and the wall sensors' sides and mounts (Part S) — as measured from the OLD front: "re-measure them, or
>       flip the push back: the camera and the wall reset would use mirrored mounts". From Part S on (before it the
>       file states none).
>   - **Save** (§1.15).
> - **Part 6 — three pages** (§1.14):
>   - **MECHANISMS.** The list in file order. Add from a template, remove (its effects listed first),
>     duplicate, move up or down, rename, and draft on or off. Each mechanism's settings come from its kind,
>     each with the tool's reason beside any value the build would refuse.
>   - **WIRING.** *(re-check 2026-10-29: the core's named slots, never free roles — RC4)* Every role → a port, the drive's ports, the IMU and the sensors shown as taken, and the live
>     warnings. There is NO "bind on the brain" (Q1 (a)).
>   - **CONTROLS.** Hold, toggle and preset per mechanism, and the live warnings. *(re-check 2026-10-29: plus `homeOnPress`, `homeAtDriverStart` and autons-only; A4-D27's rule and A4-D26's words — RC7)*
>   - The CALIBRATION page gains its mechanism warning: a role on a port the record says DRIVES, or on the
>     IMU's port. **Its limit (re-check #10):** it cannot see a drive motor that was wired as a mechanism role
>     BEFORE the calibration. Calibrate drops every declared mechanism port, so the record never holds that
>     port. That catch is Calibrate's declared-port watch (§5; D10's costs).
> - **Superseded:** the "Editable" list below, as far as it names mechanisms (add, remove, rename; motor or
>   pneumatic; port / 3-wire letter; cartridge; safe mode; open level; start; safe) and each control. That
>   now happens on the Part 6 pages in §1.14's shape, with ports on the WIRING page and never inside a
>   mechanism.
> - **The manual VS Code check (§2)** grows per part: Part 3 creates a probe robot, flips a front and opens a
>   record (against robot two's file, and against the portless probe robot's, whose page says the front cannot
>   be checked); Part 6 adds a mechanism from each template, removes one named by an auton, and re-wires a role.

- **Layout:** `package.json` (engines `vscode ^1.90.0`; devDependencies pinned exactly: `typescript`
  5.9.3, `@types/vscode` 1.90.0, `@types/node` 20.19.43, **`@vscode/vsce` 3.9.2** — whose engines admit
  `node >= 20`; 4.0.0, the v2 probe's, declares `node >= 22` (re-check #9; `recheck/panel/`: 3.9.2 on
  the Node 20 installed here, model 5/5, packaged); `"repository"` is the REAL
  `https://github.com/SHU-ROBOTICS/shulib` (`git remote`, `mkdocs.yml` :18), never a placeholder;
  `package-lock.json` committed; **no runtime dependencies**, so no bundler); `src/extension.ts` (VS Code
  glue only); `src/tool.ts` (runs `robot_tool.py`); `src/model.ts` (PURE: no `vscode` import);
  `src/views/robotPanel/` (the webview); `test/*.test.ts` (`node --test`); `README.md`.
- **The form** is a `CustomTextEditorProvider` for `*.robot.json` (`priority: default`, "Reopen with
  Text Editor" always available): the JSON document stays the source of truth, so undo, git and a text
  editor all work. **Editable:** name, cartridge (dropdown), ~~the front's WORDS,~~ *(re-check 2026-10-29: none, RC2)* the ports per side, the
  IMU, sensors, mechanisms (add, remove, rename; motor or pneumatic; port / 3-wire letter; cartridge;
  safe mode; open level; start; safe), each control's buttons, volts, words, provenance. **The panel
  enforces nothing about the file's content** (re-check #5 — one validator): "provenance required when
  a value changes" is the TOOL's baseline rule (§1.2), and the panel shows its refusal beside the
  provenance field like any other. **Every vocabulary comes from `describe`** — the prototype's
  hard-coded cartridge list goes (re-check #15.2). **"The OTHER end is the front"** is a button that
  runs `flip-front` (§1.2) with the provenance the user types; the form reads the statement back —
  "looking toward <front>: LEFT …, RIGHT …" (§1.1) *(re-check 2026-10-29: offered only for typed sides found FLIPPED; no words to read back — RC2)*. **Read-only:** `typedTable` (labelled "measured by
  MOTOR WATCH, legacy — robot two only"), the re-zero (read from THE FEEL — D4) *(amended 2026-10-29 evening: no longer read-only — the FEEL page edits it, D11)*, the calibration record.
- **Every edit** goes model → `robot_tool.py fmt -` → a `WorkspaceEdit` of the whole document (the ONE
  writer; unknown fields kept — P9's test). **Live warnings:** on change (debounced ~300 ms) it runs
  `validate - --path <file> --json` on the buffer and shows each refusal beside its field — **by its JSON
  path, never by label** (the prototype matched labels: re-check #15.2) — AND as a VS Code diagnostic
  (the Problems panel works with the plain text editor too). **Save:** VS Code saves the JSON; then
  `generate`. It **refuses to generate while any target file has unsaved changes** ("save or revert
  src/team/robot_two_driver.hpp first" — editor state, the panel's one rule of its own) and shows the
  tool's words verbatim, the witness list included ("check-examples will fail until
  docs/cookbook/06-driver-control.md is re-quoted" — re-check #3).
- **The calibration record, read-only:** "Show a calibration record" opens a `.bin` (the SD card, or a
  copy); the panel runs `robot_tool.py record FILE --robot <this file> --json` and shows: **the port map
  in the SAME form the boot prints on its `[SES] ports` line** — `MEASURED L -16 +17 -18 -20 R +11 -12
  +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01` for robot two's 2026-09-25 record (`robot::writePortMap()`,
  `include/shulib/robot/robot.hpp` ~:448; printed by `include/shulib/diag/session_info.hpp` ~:104-108) —
  so a copy is matched to what the brain logged BY EYE (re-check #10: no program prints the CRC); then
  source, per side the signed ports and inches per motor radian (the product of wheel size and ratio —
  never split into a "wheel diameter", which the record does not carry), the track width, the
  provenance, the file's CRC-32 as its IDENTITY (two copies with one CRC are the same bytes — never "what
  the brain printed"), and the comparison with the file (same front / FRONT REVERSED / ports LEFT OUT / ports the file lacks /
  cartridge / IMU). A refused record shows the decoder's sentence. **The decoder is Python** (the four
  prefix refusals, the CRC, and `valid()`'s eleven sentences — mirrored), pinned by a host test (§2): a
  display that says "decoded" where the brain says "refused" would be a lie. The same test pins the
  port-map line: it builds a `config::RobotConfig` from each decoded description and CHECKs
  `robot::writePortMap()`'s text equals the Python line. SAFE-SAVE's `.bak` and
  R5a's `shulib_characterization.bin` are shown when those chunks have landed (§5).
- **Python:** the setting `shulib.python` (default `python3`); a missing interpreter is ONE clear
  message, never a silent empty form.
- **Room for H4 (the robot console), none of it built:** one extension, one view per job
  (`src/views/<job>/`), one process runner, a typed message protocol between webview and host — the
  console is a second view on H1's wire later. Nothing in the panel assumes it is the only view.
- **Packaging:** `npx vsce package` → `shulib-robot-panel-<version>.vsix`, installed with
  `code --install-extension`; ~~**no marketplace**~~ *(amended 2026-10-29 evening: "GitHub now, store later" — each version a `.vsix` on a GitHub Release, cut by the release job on the team lead's tag; the Marketplace and Open VSX after Nov 21; the package release-ready — §5 of the evening amendment, D19; fix 2026-10-30, C16: the publisher name checked — or
registered — at BOTH stores before the first release tag)*; the `.vsix` and `node_modules/` are gitignored.

### 1.8 The panel's CI job (Part B — *amended 2026-09-29: Part 3*)
*(Amended: the job and the record agreement test's CMake step are Part 3's. Part 6 adds model tests only.)*
*(amended 2026-10-29 evening: Part 3 also adds `.github/workflows/robot-panel-release.yml` — on the team lead's `robot-panel-v<version>` tag only: test, package, check the file list, attach the `.vsix` to a GitHub Release; nothing to a store — §5 of the evening amendment, S50.)*
A separate job `robot-panel` (`actions/setup-node`, **Node 22** — Node 20 reached end of life 2026-04-30
per the Node.js release schedule, and 22 is within vsce 3.9.2's engines; re-check #9): `npm ci`,
`npm test`, `npx vsce package`. No other job depends on it; the host and ARM jobs run without Node.
Locally the Node 20 installed here builds, tests and packages it (probed).
**Part B's host-suite change (re-check #13):** `test/CMakeLists.txt` gains a custom command that runs
`robot_tool.py record-fixtures ${CMAKE_CURRENT_BINARY_DIR}/generated/robot_tool_record_agreement.hpp`
(DEPENDS on `tools/robot_tool.py`) and adds `test/robot_tool_record_agreement_test.cpp` with that
directory on its include path — the same shape as Part A's `agreement-fixtures` step (§1.5). The host
suite, not the panel's job, runs it: the decoder is Python, so no Node is involved.

### 1.9 What "optional" means, concretely
- **`make`** (PROS) runs no Python and no Node, reads no JSON — the robot builds exactly as today, with
  the generated lines checked in.
- **The host build and CI** need python3 — as they already do — and no Node.
- **A team without Node** edits the robot file as text and runs `generate` / `check` / `fmt`.
- **A team without Python** can build and upload; it cannot regenerate, and its host build / CI fails on
  a stale or hand-edited region — by design.
- **The honest gap:** a student who edits the robot file by hand and runs `make` without `generate`
  uploads the OLD regions; the host build, CI and the src gate catch it, `make` does not (and must not
  gain a Python step — "a gate you can switch off is not a gate", `test/CMakeLists.txt` :165-176). The
  panel runs `generate` on save, so panel users never meet it.
- **In THIS repository a controls change is not done at `generate`** (re-check #3): cookbook 06 quotes
  robot two's WHICH BUTTON block verbatim, so `api_doc_tool.py check-examples` — the host build's doc
  gate and CI — fails until the listing is re-quoted by hand from the new source (the whole-run rule,
  by design: `api_doc_tool.py` :1833-1853), and the host suite's literal pins fail until they change
  with it (D5). `generate` and the panel name both; PKG1's starter has neither.
- **(Amended 2026-09-29) A new robot needs no panel either.** `robot_tool.py new-robot` is the same
  command the panel runs. A team without Python adds a robot by hand, OUTSIDE the regions, as robot two's
  build was written (the guide's manual steps). The tool then knows nothing of that robot, and the gate's own
  tables must be edited by hand too.

### 1.10 How PKG1 reuses it
PKG1's starter project carries "a robot file, a driver file and an auton list" (build-order :1637). The
tool is marker-driven (§1.2), so the starter's files carry markers and the SAME tool, the SAME validator
and the SAME panel work unchanged. **PKG1's own decision, named here:** the emitters write this repo's
C++ types (`ChassisTable`, `MechanismTable`, `HoldControl`, `ToggleControl`), which live in `src/`, not in
the library *(re-check 2026-10-29, CK2: after MECH-A5 the mechanism and controls vocabulary IS the library's — `mechanism::MechanismTable` (`include/shulib/mechanism/wiring.hpp`), the kinds, `wire(...)`, `controls(...)` (`driver_inputs.hpp`); `HoldControl` / `ToggleControl` retire with `driver_controls.hpp`. Only `ChassisTable` and `DriverFile` (`src/common/driver_file.hpp`) stay in `src/`, so PKG1's decision shrinks to those two)*; the starter either ships them (a `src/`-side layer copied by the template) or the library
promotes them into `include/shulib/` (a public API change). Either way the robot file, the markers and
the panel carry over; only which file each region sits in changes. The starter's robot file can never
carry a `typedTable` (§1.2), so a PKG1 team's drive is calibrated from the start. The baseline rule
needs no git, because it compares against the starter's own regions. The witness list is empty there,
because the starter has no host pins or cookbook.
**(Amended 2026-09-29.)** With new-robot creation, the starter's robot can be a GENERATED robot: the starter
carries the SET regions (empty or holding its one robot), and the team's first act is `new-robot` or the
panel's NEW ROBOT page. Bench, xdrive and robot two are this repository's hand-written robots and never
travel. Where the SET regions' C++ lives in the starter (its own `build_selection.hpp`, `Makefile` and tables)
is PKG1's decision, like the types above.

### 1.11 How G3 embeds it — and §3.1 revised

> **AMENDED 2026-09-29 (P2, P8, §4.10; Q20 is his).**
> - *(re-check 2026-10-29, CK2: the two bullets below — `roles` `{role, device, reversed}`, "Rename 7" and `wiring[{mechanism, role, port}]` — are themselves RETIRED: MECH §6.3 as landed has NAMED slots, `motors: ["forward", "reversed"]` and `wiring[] {mechanism, motors[], angleSensor, pieceSensor, homeSwitch, lines[]}` (the adoption table). The re-meaning to name to VexBuilder is therefore "`motors[{port, reversed}]` → each kind's members + the named-slot `wiring[]`", and Part 5's documentation writes it.)*
> - **The rows naming `mechanisms[]` with `motors[{port, reversed}]` / `pneumatics[A..H]` are SUPERSEDED.**
>   §3.1's revision (Part 5's documentation) gives mechanisms ROLES (`{role, device, reversed}`), never ports,
>   plus a separate `wiring` block (role → port). Under Q1 (a) the wiring block is the ONLY binding, and it
>   travels in the profile; §3.9 (a) says the export carries "roles and their compiled ports".
> - **Rename 7 (new):** `mechanisms[].motors[{port, reversed}]` → `mechanisms[].roles[{role, device,
>   reversed}]` + `wiring[{mechanism, role, port}]`. It is a RE-MEANING for VexBuilder, named in the note
>   §4.10 asks the coordinator to send. It costs little before F7 freezes at G3 and a `schemaVersion`
>   migration after.
> - **The field kinds grow.**
>   - VexBuilder-emitted and REQUIRED: `mechanisms[].kind`, `roles`, `wiring`, and each kind's required
>     settings (§1.14).
>   - VexBuilder-emitted and OPTIONAL: `presets`, `controls.presets`, `autonOnly`.
>   - shulib-only: `build` (G3's reader fills it by a NAMED rule, D8's default program list) and `draft`
>     (never in a `.vexbot`: a draft is not a stated robot).
> - *(amended 2026-10-29 evening)* **The `driving` (D11), `driveTuning` (D12) and `footprint` (Part R1) blocks** become profile schema too — F7 at G3; the note to VexBuilder (§4.10) gains them, and the route file's F8 export carries its extras in ONE optional `shulib` object per waypoint (D13 (c)); Q20 (keep unknown fields) covers both. *(fix 2026-10-30: `driveTuning`'s angles are in DEGREES with `Deg` in their names, as the route file's — F3, C16; it carries no `rotationRadius`, which is measured geometry — C12.)*
> - *(re-check 2026-10-29)* **The camera and walls blocks** (VIS1 §6.6, L6 §6.7) become profile schema when Part S emits them; VexBuilder's `sensors: [ { port, type } ]` carries neither (L6 RC32), so the note to VexBuilder gains both. **The front is NOT a profile field** (2026-10-06, RC2). **The mechanism words are MECH §6.3's as landed** (MECH §6.4: from Part 5 to G3 a rename is a generator and agreement-test change).
> - **Every MECH field becomes profile schema (P8).** A panel → VexBuilder → import round trip could
>   therefore drop mechanism settings silently. Hence the NEW ask (§5 Q20, his; recommendation (a)):
>   VexBuilder keeps `robotProfile` fields it does not understand, byte for byte. Until it confirms, G3's
>   import takes a field the `.vexbot` lacks from the robot file, and never defaults it (§4.7).

The robot file's object IS `project.robotProfile`. *(re-check 2026-10-29, CK2: in this paragraph and the field-kind and rename lists below, every mechanism and control item — `motors[{port, reversed}]`, `pneumatics[A..H]`, `openLevel`, `openLevelVerified`, `openLevelNotes`, `mechanismNotes`, `safeMode`, "the volts", rename 5 — is the RETIRED 09-29 vocabulary; the mechanism fields are the adoption table's (each kind's members, named slots, `positions` with levels, the controls' six lists), REQUIRED exactly where `kinds.hpp` says REQUIRED; `front` is gone (RC2). The drivetrain, sensor and provenance items hold.)* §3.1 is rewritten to it: identity; drivetrain
{kind, cartridge, front, left, right, imuPort, seenByCensus, provenance, notes}; sensors; mechanisms with
`motors[{port, reversed}]` (a mechanism's direction is the team's choice) or `pneumatics[A..H]` with
`openLevel` + `start`/`safe` POSITIONS (one polarity moves start and safe together — the table's design,
`mechanism_table.hpp` :61-70; the config keeps levels, `boot_source.hpp` :209-217 converts); controls.
**Out of the human-stated profile:** `wheelDiameter`, `trackWidth`, drive `reversed`, `gearRatio` — measured.
VexBuilder's asks #2 and #7 (roadmap ~:2008-2030) asked for exactly those: §3.1 keeps them as an
OPTIONAL `drivetrain.designed{}` block (CAD values) that G3 may show beside the measurement and **never
drives from**. `odometry{}`, `corrections{}` stay drafted for G3. F7 does NOT freeze here.

**Both directions, said plainly (re-check #14).** shulib → `.vexbot`: a robot file IS a valid
`project.robotProfile`, unchanged. VexBuilder → shulib: VexBuilder emits a SUBSET, and §6's promise
("missing fields get safe defaults") meets the validator's refusals. §3.1's revision therefore marks
every field as one of three kinds:
- **VexBuilder emits it, and it is REQUIRED.** No safe default exists, so G3 refuses a missing one by
  name, which is §3.2's own precedent for `cartridge`/`safeMode`. These are `identity.name`;
  `drivetrain.kind`, `cartridge`, `front`, `left`, `right`; `mechanisms[]` `name`, `motors[{port,
  reversed}]`, `cartridge`, `safeMode`, `pneumatics`, `openLevel`, `start`, `safe`; and `controls`
  (the mechanism, the buttons, the volts). `front` is a NEW ask, because a wrong front drives the robot
  backwards and there is nothing safe to default it to. *(re-check 2026-10-29: WITHDRAWN — the front is the record's, never the profile's, RC2)*
- **VexBuilder emits it, and it is OPTIONAL.** These are `identity.team`/`season`, `drivetrain.imuPort`
  (none when absent), `sensors[]`, and `drivetrain.designed{}`.
- **shulib-only, optional in a `.vexbot`, and filled by G3's reader by a NAMED rule, never invented:**
  - `provenance` (drivetrain, mechanisms, sensors) ← "stated in VexBuilder project <file>, exported
    <date>". The `.vexbot` is where the statement came from, so this is a provenance, not an invented
    value;
  - `seenByCensus` ← `false` (nothing seen yet);
  - `openLevelVerified` ← `false` (a warning, as today);
  - `words` ← written from the control itself ("R1 / R2: <mechanism> forward / back");
  - `notes`, `mechanismNotes`, `openLevelNotes` ← absent (they are comments only);
  - `typedTable` ← NEVER in a `.vexbot`: it is robot two's legacy block and is refused elsewhere.

  Robot two's own file carries shulib-internal prose (C++ remarks, test-file paths, HA numbers) only in
  these optional notes fields.

**The renames and re-meanings of §3.1's DRAFT, each named to VexBuilder** in the revised §3.1 and in §7's
risk list. They cost VexBuilder nothing only because F7 is unfrozen and VexBuilder's `robotProfile`
block is still ❌ (§1's status table), so no emitter exists to throw away. §9 rule 6 ("renames and
re-meanings are not free") applies from G3's freeze on.
1. `drivetrain.motors[{port, reversed, cartridge, side, gearRatio}]` → `drivetrain.left[]`/`right[]`
   (ports as wired) plus one `drivetrain.cartridge`. `reversed` and `gearRatio` leave, because they
   are measured.
2. `drivetrain.wheelDiameter`, `trackWidth` → out (measured), kept only as the optional `designed{}`.
3. `odometry.imuPort` → `drivetrain.imuPort`.
4. `sensors[].type` → `sensors[].kind`. The vocabulary gains `radio` and `ai-vision` and loses `imu`,
   which is `drivetrain.imuPort`.
5. `pneumaticSafe`/`pneumaticStart` LEVELS → `safe`/`start` POSITIONS plus one `openLevel`. This is a
   RE-MEANING: a level becomes a position.
6. New fields: ~~`drivetrain.front`,~~ *(re-check 2026-10-29: no)* `controls`, and the provenance fields.

### 1.12 Hardware Assumptions
**None.** Every belief G3a displays is already registered (the mechanisms HA-153…159, the drive HA-124…130)
or is self-checking: the panel reads a copy of the card, and the CRC makes a changed byte loud. An OLD copy
is not a hardware belief — the panel prints the record's port map in the SAME form as the boot's
`[SES] ports` line (`robot::writePortMap()`, pinned by the record agreement test), so a copy is matched
to the brain's log by eye. (The draft said "the CRC-32 ... matched to what the brain printed": no program
prints a record's CRC — re-check #10; the CRC is shown as the file's identity only.) **HA-181 is reserved;** an executor who meets a real one stops and tells the coordinator. *(re-check 2026-10-29: held in the register's Held line, `docs/hardware-assumptions.md` :39; the next free is HA-216 — RC16.)*
**(Amended 2026-09-29) The amendment adds none.**
- A new robot's facts are human-stated, each with its provenance.
- D7's flip derives robot two's signs by negation and says so, and N1 shows them agreeing with the record.
- The templates invent no number (P6).
- A mechanism's hardware beliefs (a direction, a homing stop, a gravity angle) are the MECH core's to register.
  G3a DISPLAYS them and adds none.
- **HA-181 stays G3a's reserve.**

### 1.13 A new robot and its build (Parts 2 and 4) — NEW 2026-09-29
- **The command.** `robot_tool.py new-robot --id ID --name TEXT --kind tank --cartridge red|green|blue
  --front TEXT --provenance TEXT [--left P… --right P… --imu P] [--programs …] [--dry-run --json]`. *(re-check 2026-10-29: no `--front`, RC2)*
  - It writes the robot file, then runs `generate` over the whole tree, ALL OR NOTHING: a refusal writes
    nothing, the robot file included.
  - `--dry-run --json` returns exactly the list in the amended section ("What new-robot creation writes"),
    and the panel's NEW ROBOT page shows it before Create.
  - The default `--programs` is D8's: `tester calibrate` in Part 2; all five from Part 4.
  - It writes nothing under `src/team/` before Part 4 (re-check #2).
- **The id** is lower_snake_case. Everything derives from it:
  - `RobotId::<Camel>`, `k<Camel>Chassis` (and `k<Camel>Mechanisms` from Part 5);
  - `ROBOT=<id>` and the define `SHULIB_ROBOT_<UPPER>`;
  - the variant beacons `shulib-robot-variant=<dash>` (Bench Tests) and `…=<dash>-<program>`, and the robot
    beacon `shulib-robot=<dash>`;
  - the class `<Camel>Driver` and the files `src/team/<id>_driver.hpp` and (Part 4) `src/team/<id>_autons.cpp`;
  - *(re-check 2026-10-29, CK8)* the NAMESPACE `shulib::bench::<id>` that holds its mechanism list (MECH-A5's
    per-robot namespace), and with it the team-file ownership `src/team/<id>_*` (A5's `_robot_namespaces`).
- **The id is REFUSED** (the collision rule, S22) when any derived name equals another's, the hand-written
  robots included:
  - the make values `bench`, `xdrive`, `tank`, and any id that CamelCases to an existing enumerator
    (`robot_two` is robot two itself);
  - a DEFINE equal to another robot's: `tank_2026` → `SHULIB_ROBOT_TANK_2026` (robot two's) and
    `xdrive_invented` → `SHULIB_ROBOT_XDRIVE_INVENTED`. For these the define is the ONLY name that collides,
    and unrefused it would break every `ROBOT=tank` build at the count guard with a misleading "two robot
    defines" (re-check #11);
  - any derived BEACON equal to another build's: `tank_drive` → `shulib-robot-variant=tank-drive`, robot two's
    drive build; and `probe` plus `probe_calibrate` give one beacon between them. The NUL-terminated beacon
    match (`test/build_selection_test.cpp` :70-94) cannot tell equal beacons apart;
  - from Part 4, a driver or auton file that already exists: "src/team/<id>_driver.hpp exists — the team's
    file is never overwritten" (S23, moved with the starter);
  - *(re-check 2026-10-29, CK8; V8)* an id that cannot be a namespace there: a C++ keyword or alternative token
    (`new`, `class`, `delete`, `and`, `not`, …); a namespace `src/` reaches unqualified from inside `shulib::bench`
    (`std`, `pros`, `hal`, `units`, `mechanism`, `config`, `robot`, … — derived by scanning, §1.2); an id whose `<id>_`
    begins another robot's id or another robot's `src/team/` file name, or the reverse (`robot` against `robot_two`);
    an id with `__` or a trailing `_` (the pattern `[a-z][a-z0-9]*(_[a-z0-9]+)*`). Each would pass `new-robot` and
    then fail the build or the gate's step 0 with a message that never names the robot file.
- **The drivetrain.**
  - `kind`: tank. §5 Q22 is his: under (b) the X-drive is generated as a `config::RobotConfig`, the way
    `src/robots/xdrive_config.hpp` is, since no typed table serves a holonomic drive (+1–1½ days, in Part 2).
  - `cartridge`: REQUIRED. Calibrate refuses an unset one (`calibrate_program.cpp` :1137).
  - ~~`front`: REQUIRED, as words (printable ASCII, ≤ 160 characters). The words reach Calibrate (below).~~ *(re-check 2026-10-29: struck — no front is stated, RC2)*
  - ports: OPTIONAL (D10). When typed, they are stated looking toward the front.
  - `imuPort`: optional, since Calibrate finds the IMU from the registry.
- **The builds.**
  - Part 2 writes the SET regions of D9. The one-time hand edits it makes first, so the regions can exist:
    - the guards in their GENERAL form: the robot-define COUNT and "PROGRAM=calibrate needs a robot whose
      programs list it", in place of the pairwise and `!TANK_2026` forms at :69-84, with their messages
      reworded (N7: the Makefile's calibrate message still says "needs ROBOT=tank");
    - the Makefile's `else ifneq ($(filter $(ROBOT),$(SHULIB_GENERATED_ROBOTS)),)` branch, a valid-values
      message that lists the generated robots, and the program guards reading `SHULIB_ROBOT_PROGRAMS_<robot>`;
    - **the Makefile's hand-written robots in the generated shape** (re-check #9), outside the region:
      `SHULIB_ROBOT_DEFINE_xdrive`/`_tank` and `SHULIB_ROBOT_PROGRAMS_xdrive`/`_tank`. The `ROBOT` block uses the
      define lines itself, and the tool reads all of them for `[robot guards]` (D9);
    - the gate's merge lines;
    - `kEveryRobotId` beside the enum. `build_selection_test.cpp`'s list and `mechanism_table_test.cpp`'s
      loop read it, and `chassis_table_test.cpp` gains a `kEveryRobotId` case beside its literal identity
      checks (re-check #13);
    - *(Re-check 2026-10-29: STRUCK — CAL1 §1.7, RC3.)* **`ChassisTable` gains `const char* front = nullptr;`** after `robot` (re-check #3). The default is load-
      bearing: both compilers refuse an omitted designated field without one (`-Werror=missing-field-initializers`,
      probed), and the bench bot's, the X-drive's and every test-built table omit it.
  - All of these are probed: N2/N3, and `recheck2/guards/`, `recheck2/front/` and `recheck2/everyrobot/`.
  - *(Re-check 2026-10-29: STRUCK — CAL1 §1.7: the glue's facts carry the robot's name; RC3.)* **Calibrate's words** move from "robot two" to `kChassis.robot`: the banner (:1044, :1068), and the
    cartridge refusal's "type it into src/robots/chassis_table.hpp" (:1141), which becomes "state it in the
    robot file (src/robots/<id>.robot.json) or the robot panel, then generate, rebuild, upload". This is a
    hunk of its own, away from the MECH core's likely edit of the same file (:1120-1131, the declared
    mechanism ports); the coordinator checks the core's brief before Part 2 launches.
  - *(Re-check 2026-10-29: STRUCK — the 2026-10-06 ruling and CAL1 §1.7; RC2, RC3.)* **Calibrate and the FRONT (re-check #3; probed, `recheck2/front/`).** The file's front words reach the one
    program where a portless robot's front is decided:
    - `front      : <words>` is printed beside `robot      :` (:1086);
    - before the push, `LEAD THE PUSH WITH THE FRONT THE ROBOT FILE STATES (<words>), then answer FRONT FIRST`
      is printed beside the ONE PUSH line (:1239);
    - `static_assert(kProgram != Program::Calibrate || (kChassis.front != nullptr && kChassis.front[0] !=
      '\0'), "PROGRAM=calibrate needs the robot file to state which end is the FRONT (drivetrain.front)")`.
      **It is conditioned on the program because `calibrate_program.cpp` is compiled into EVERY build.**
      Unconditioned, it failed the probe robot's Bench Tests build and would fail the bench bot's.
    - These are two more hunks of that file (:1086, :1239), away from the core's :1120-1131.
    - **The limit:** the question on the SCREEN, "WHICH END FIRST?", is the library's panel
      (`include/shulib/calibration/calibrate_panel.hpp` :374), which G3a does not change. The words reach the
      serial log, not the screen (§8 names it for the next chunk that edits that panel).
- **The starter driver file (Part 4 — moved from Part 2 at the re-check, #2)** is written ONCE *(re-check 2026-10-29: in A4/A5's vocabulary — `controls(k<Id>Mechanisms, …)` from the file's controls, `autonOnly` where it says so, and `using <Camel>Driver = DriverFile<…>` (MECH §1.13's `src/common/driver_file.hpp`); the span bullet below is moot — an empty `controls(list)` compiles (V2) — and `DriverFor` follows A5's driver type; RC11)* and never
  regenerated outside its regions. It is written in the controls vocabulary the MECH core leaves, since its bind
  call and its lists ARE that vocabulary. The bullets below were written against today's `DriverBindings` and
  hold in spirit; Part 4 re-reads the driver file robot two has after the core.
  - Its THE FEEL is the library's defaults: ArcadeBody, re-zero Y, Coast on release. The team tunes it (D4). *(amended 2026-10-29 evening: in a generated `feel` region from an empty `driving` block; the team tunes it in the robot file or on the FEEL page — D11)*
  - Its checks name `mechanismTableFor(RobotId::<Id>)` inside `static_assert`s, which step 0 allows (N2).
  - Its empty control lists are `std::span<const HoldControl>{}` and `std::span<const ToggleControl>{}`: the
    `[] = {}` form fails both compilers (N4).
  - It carries `template <> struct DriverFor<RobotId::<Id>> { using type = <Camel>Driver; };`.
  - The ARM builds compile it from the day it exists, since Part 4's programs include every driver file. The
    host suite does too: Part 4 adds one host TU that includes every `src/team/*_driver.hpp` (a
    `CONFIGURE_DEPENDS` glob), probed as `minisuite.sh`'s per-driver TUs.
  - Its lines repeat robot two's (the library's default feel, the bind and run calls). That is why Part 4 binds
    every whole-run listing to its file first: cookbook 06's three and the auton chapter's two (§1.5, re-check
    #1).
  - A robot created before Part 4 gets it from `generate` when its `build.programs` first lists `drive` or
    `competition`; an existing file is never overwritten.
- **Part 4, the match builds (D8 (a)).**
  - `src/team/drivers.hpp` includes every driver file (the SET region `[driver files]`, robot two's include
    hand-written) and declares `template <RobotId> struct DriverFor;` with no definition. The drive and
    competition programs hold a `DriverFor<kRobotId>::type`, where T4 Part 3 and COMP-b wrote `RobotTwoDriver`.
  - `src/team/autons.hpp` gains `autonsFor(RobotId)`, with robot two's case hand-written (→ `registerAutons`)
    and the SET region `[auton selection]`. The Auton Tester and the Competition program call
    `autonsFor(kRobotId)(registry)`.
  - The starter auton file registers nothing, and says so in words the students read. Whether COMP-b's picker
    and the Auton Tester accept an EMPTY registry is checked at Part 4's tip: it is not in the tree today.
    If either refuses, Part 4 makes "no autons yet" a stated screen, never a crash.
  - Step 0's rule (§1.5).
- **Robot two keeps its hand-written build** (D9). Its file carries no `build` block, and adding one is
  refused.
- **Robot one** (HANDOFF Q3) is the first real user: *(re-check 2026-10-29: if robot one competes on 2026-11-23, its build is hand-written first and stays a hand-written robot until a separate call converts it — RC18)* the team runs `new-robot` with robot one's own
  cartridge and front. This brief types none of robot one's facts.
- **Removing a generated robot is OUT of the panel.** The guide states the manual steps:
  1. delete its robot file;
  2. run `generate`, which shrinks the SET regions;
  3. delete its `src/team/` files, which are the team's. After Part 4, a leftover driver file specializes
     `DriverFor` for a missing enumerator and fails the build, naming the file.

### 1.14 Mechanisms, templates, wiring and controls (Parts 5 and 6) — NEW 2026-09-29, PROPOSED

> **(re-check 2026-10-29) SUPERSEDED by MECH §6.3 AS LANDED — the re-check's adoption table is the vocabulary (five kinds, named slots, no handles file, route ids derived, the controls with A4-D27; RC4–RC8). The templates map one to one (MECH §6.3: Intake → power with an optional piece sensor; Arm → arm; Lift → lift; Claw → pneumatic, two positions, `grabs` on closed; Flywheel → flywheel; Catapult → power with a limit-switch piece sensor; Conveyor → power; an arm, a lift and a flywheel template carry an EMPTY `stall` the team must state). The text below is the 2026-09-29 proposal; where it differs, MECH's word wins.**
>
> **Every field in this section is PROPOSED from `MECH-PANEL-requirements.md` §3 (the core parts) and is TO
> BE RE-CHECKED AGAINST THE MECH CORE BRIEF (`briefs/MECH-core.md`) before Part 5 starts.** The robot file
> must carry EXACTLY the core's vocabulary, with nothing the core cannot build and nothing it builds missing.
> The coordinator does that re-check, logs it in the PROGRESS file and amends this section before Part 5's
> go. A name below that the core spells differently takes the core's spelling.

**The file's shape (P2, Q1 (a)).** Robot two's mechanisms, re-stated (ILLUSTRATIVE; Part 5 extracts the real
file from the tree the core leaves):
```json
"mechanisms": [
  { "name": "claw pivot 6", "kind": "power",
    "roles": [ { "role": "motor 1", "device": "motor", "reversed": false } ],
    "cartridge": "red", "safe": "hold", "provenance": ["…"] },
  { "name": "claw pivot 7", "kind": "power",
    "roles": [ { "role": "motor 1", "device": "motor", "reversed": false } ],
    "cartridge": "red", "safe": "hold", "provenance": ["…"] },
  { "name": "claw", "kind": "pneumatic",
    "roles": [ { "role": "line 1", "device": "solenoid" } ],
    "openLevel": "high", "openLevelVerified": false, "start": "closed", "safe": "closed",
    "provenance": ["…"], "openLevelNotes": ["…"] }
],
"wiring": [
  { "mechanism": "claw pivot 6", "role": "motor 1", "port": 6 },
  { "mechanism": "claw pivot 7", "role": "motor 1", "port": 7 },
  { "mechanism": "claw", "role": "line 1", "port": "A" }
],
"controls": {
  "holds":   [ { "mechanism": "claw pivot 6", "forward": "R1", "back": "R2", "volts": 12, "words": ["…"] },
               { "mechanism": "claw pivot 7", "forward": "L1", "back": "L2", "volts": 12, "words": ["…"] } ],
  "toggles": [ { "mechanism": "claw", "button": "A", "words": ["…"] } ],
  "presets": []
}
```
**The fields** (Part 5 validates each by mirroring the core's C++ refusal, proven by the agreement test; the
tool's own stricter rules each say that the C++ cannot see them):

| Field | Kinds | Refused when |
|---|---|---|
| `name` | all | > 23 characters; not unique; the text-field rule (a literal: printable ASCII). The typed handle derives from it (its spelling is the core's) |
| `kind` | all | not `power`, `position`, `speed` or `pneumatic` (the core). `cycle` comes with the rest |
| `roles[]` `{role, device, reversed}` | all | a role twice in one mechanism; a device the kind cannot use (a solenoid on a motor kind, a motor on a pneumatic); motors AND lines in one mechanism (the contract's "never both", kept by Q12's recommendation); > 4 motors or > 2 lines (`kMaxMotorsPerMechanism`, `kMaxPneumaticsPerMechanism`); `reversed` on a device with no direction. `device` ∈ motor, rotation, limit switch, distance, optical, solenoid (the core's sensors: Q3) |
| `cartridge` | motor kinds | not red, green or blue |
| `safe` | all | missing, with NO default (P6): `hold`/`brake`/`coast` for motor kinds; a named position for a pneumatic |
| `autonOnly` | all | — (Q8 (a); a mechanism with no control and not auton-only is refused, the core's relaxed `controlsCoverTable`) |
| `draft` | all | — (decided here: a draft generates NOTHING, its empty numbers are warnings, and a control naming a draft is refused) |
| position: `units` (`degrees`/`inches`), `gearing`, `sensor` (the motor or a rotation role), inches: `spoolDiameterIn` OR `linkage {armLengthIn, pivotHeightIn, stages}` OR `table [[in, deg], …]`; `limits {min, max}`; `gravity` (`cosine {horizontalDeg}`/`constant`/`none`); `homing` (`switch`, needing a limit-switch role / `stall {…}` / `absolute {offset}`, needing a rotation role); `profile {maxVelocity, maxAccel}`; `gains` | position | gearing ≤ 0; a table with < 2 pairs or not strictly increasing (§3.1); `limits` missing or min ≥ max; homing by switch with no switch role; every number the team owns EMPTY (P6). The core ships starting gains, measured later |
| speed: `atSpeed {bandRpm, forS}`, `gains` | speed | a band or time ≤ 0, or EMPTY |
| pneumatic: `openLevel`, `openLevelVerified`, `start`, `safe` (two positions, today's design), OR `positions {name: [level per line]}`, `start`, `safe` (three positions, Q13 (a)) | pneumatic | a line state no position names; a position with the wrong number of levels; no default anywhere |
| `presets[] {name, target}` | position, speed | a CONSTANT target outside the limits (refused before a match, §3.1 rule 1); a name twice |
| `wiring[] {mechanism, role, port}` | — | a role with NO wiring entry (the build refuses it under Q1 (a)); an entry naming a missing mechanism or role; a port twice across drive, IMU, `sensors[]` and wiring; a DRIVE or IMU port (P7; Q4 (b) names this rule for a later PTO); a smart device on a letter A–H, or a 3-wire device on 1–21; an expander port (Q5 (a)) |
| `controls.holds[]`, `.toggles[]`, `.presets[] {mechanism, preset, button, words}` | — | the five predicates (a button twice, the re-zero, volts, a name of the wrong kind, cover — auton-only allowed); a preset control on a kind without presets, or naming a missing preset; two controls of ONE kind on a mechanism (P5); a toggle on a kind the core cannot toggle (today's toggles are pneumatic only, so the Flywheel template's toggle waits on the core's rule) |

**The templates** (Part 5 holds them as DATA in the tool, and `describe --json` returns them; Part 6 is the
picker). A template's roles are created UNBOUND, and the wiring page binds them. Every number the team must
own is left EMPTY and refused until stated, or the mechanism is marked a draft (P6):

| Template | Kind (the core) | Roles it creates | Filled by the template | Left EMPTY for the team | Added with the rest (Part 7) |
|---|---|---|---|---|---|
| **Intake** | power | motor 1 (more on request); piece sensor, optional (distance or optical) | safe Coast; a hold control (in / out) | cartridge; the hold's two buttons and its volts | jam auto-reverse (on by default) and its stall thresholds |
| **Arm** | position, degrees | motor 1; rotation, optional; limit switch, optional | safe Hold; gravity cosine; smooth moves (the trapezoid profile); homing by stall, or by switch when that role is kept; the core's starting gains | cartridge; gearing; travel limits; the horizontal angle; the stall-homing thresholds; presets (none until named) | auto-tuning, which measures the horizontal angle (MECH D); loaded gains |
| **Lift** | position, inches | motor 1, motor 2 optional; limit switch, optional; rotation, optional | safe Hold; gravity constant; smooth moves; homing by switch when the switch role is kept, else by stall | cartridge; the spool diameter OR the linkage (arm length, pivot height, stages) OR a lookup table; travel limits; the homing thresholds; presets | the tip-protection height; loaded gains |
| **Claw** | pneumatic | line 1 (line 2 optional → three positions, Q13 (a)); piece sensor, optional | a toggle control | the open level (or each position's levels); the start and safe positions, with no default (today's rule); the toggle's button | — (Q2 (c): a grab is confirmed by time now and by the sensor once one is mounted; that is the auton step's, not a setting) |
| **Flywheel** | speed | motor 1, motor 2 optional | safe Coast; "at speed" as its done-condition; a toggle control (subject to the core's toggle rule) | cartridge; the target RPM (a preset); the at-speed band and time; the toggle's button | — |
| **Catapult** | power in the core (the cycle kind comes with the rest) | motor 1; limit switch "loaded" | "run until loaded" as a power mechanism with a switch done-condition | cartridge; the SAFE state (a loaded catapult's depends on its mechanism, so the team states it); the control's buttons and volts | the cycle kind: wind until loaded, fire, re-wind; slip gear or latch (Q12) |
| **Conveyor** | power | motor 1; entry sensor, optional; exit sensor, optional | safe Coast; a hold control | cartridge; the hold's buttons and volts | piece counting (entry, exit, debounce time); jam auto-reverse |
| **Blank** | chosen by the team | none | nothing | everything | — |

**The mechanism edits are TOOL commands** (D3: one writer; the panel calls them, and a text-editor user gets
the same edit). Each has `--dry-run --json`, which lists its EFFECTS before anything is written:
- **Add** (`--template T --name N`) creates the mechanism with the template's settings and unbound roles, plus
  the template's controls with EMPTY buttons.
- **Remove** also removes the controls (holds, toggles, presets), the presets and the wiring entries that name
  the mechanism. Removing the last pneumatic leaves the polarity region empty (re-check #15.1). Its handle
  then leaves the generated C++, so any C++ that names the handle FAILS THE BUILD (Q18 (a)):
  - GCC names each auton FUNCTION and line (probed N5: `In function 'void fortyInches(Routine&)': 'claw' is
    not a member of 'mech'`);
  - before the save, the effects list shows every line under `src/team/` and `test/` naming the handle's
    spelling, as file:line. A use through a macro or an alias escapes that search but not the compiler, and
    the list says so;
  - the host suite compiles the auton files too, so CI catches what a panel user ignores.
- **Duplicate** copies the kind, the roles (unbound), the settings and the presets, but NOT the wiring and NOT
  the controls (a port or a button used twice would be refused). A new name is required.
- **Move** (reorder): the file's order is the boot line's, the panel's and the table's. The baseline rule keys
  entries by name, so a move is not a value change.
- **Rename** changes the handle, so its C++ effects are Remove's. Inside the file, its controls, presets and
  wiring are renamed in the same edit, and it carries a provenance like any new entry.

**The wiring page (Part 6), under Q1 (a).**
- Every role of every mechanism gets a port picker (1–21 for a smart device, A–H for a 3-wire one). The
  drive's ports, the IMU and `sensors[]` are shown as taken.
- The live warnings are the tool's refusals, by JSON path: a port used twice, a drive or IMU port, the wrong
  port type, a role left unbound (the build refuses it).
- **"A drive or IMU port" can only mean a TYPED one** (re-check 2026-09-29, #10). On a robot whose drive ports
  are untyped (D10 (a)), the page says so beside the drive block: "drive ports not typed: a role wired to a
  drive motor cannot be caught here, and Calibrate will leave that motor out". The catch is Calibrate's
  declared-port watch (§5), and after a calibration the record's ports are shown as taken.
- Every 3-wire line reads "not checkable on the brain": the registry cannot see the 3-wire lines (HA-158).
- **The wrong-kind warning, honestly limited.** The drivetrain record carries only the drive's signed ports
  and the IMU (`calibration_record.hpp` v1), so the page can say "the record says port N DRIVES" and nothing
  about any other kind. The full kind check is the boot's, the MECH core's per-role boot line.
- There is NO "bind on the brain".
- **The direction check** (on the brain, the MECH core's, Q30) reports a wrong direction and names the
  robot-file line to change. From Part 5 on, the generated table carries each role's JSON path, e.g.
  `src/robots/robot_two.robot.json: mechanisms › "claw pivot 6" › roles › "motor 1" › reversed`. Until then,
  the core's hand-written table names its own C++ line (§5). *(re-check 2026-10-29, CK2/CK7: in the landed words and
  in ASCII — the brain prints it, and the text-field rule refuses `›` — e.g.
  `src/robots/robot_two.robot.json: mechanisms > "claw pivot 6" > motors > 1` (V1, 84 characters), at most 160.)*

**The controls page (Part 6).**
- Per mechanism: a hold (forward, back, volts), a toggle (a button), and presets (a button → a named preset).
  Every button is a dropdown from `describe`.
- The live warnings are the five predicates, the presets' rules, two controls of one kind, and auton-only.
- ~~The re-zero is shown read-only (D4).~~ *(amended 2026-10-29 evening: the re-zero is edited on the FEEL page, and the CONTROLS page warns when a control takes it — D11)*
- The rest's controls — preset cycling, sticks, long-press and double-tap, macros, the partner controller —
  come in Part 7.

### 1.15 Save (Parts 3 and 6) — NEW 2026-09-29
1. VS Code saves the JSON.
2. The panel runs `generate`, which validates first: any refusal writes nothing anywhere, and the refusals
   are shown by JSON path and in the Problems panel.
3. Then `check`.
4. Then the witness list, verbatim.

There is ONE validator. The panel's only rule of its own is "never generate under a dirty target buffer"
(§1.7). "The build's checks" are the validator's by construction: the agreement test proves the validator
names every C++ refusal. `make` stays the final word. The panel offers an OPTIONAL "Build this robot"
command that runs `make ROBOT=<id>` in a terminal. It is never a gate and never runs on save: a build takes
~11–18 s and needs the ARM toolchain. A saved file with refusals leaves the tree failing `check` until they
are fixed, which is why a half-made mechanism can be saved as a draft (§1.14).

### 1.16 Import and export — hooks only (G3 and Export) — NEW 2026-09-29
The robot file IS the profile (D1, P8), so both directions are thin. *(amended 2026-10-29 evening: ROUTES are exported now — Part R1's `robot_routes.py export-paths` writes F8 `paths[]` entries with a `shulib` object, D13 (c); the `.vexbot` import and export below stay G3's.)*
- **Import (G3)** reads a `.vexbot`'s `project.robotProfile`, validates it, writes
  `src/robots/<id>.robot.json` and runs `new-robot`'s generation. A field the file lacks is taken from an
  existing robot file, never defaulted (§4.7).
- **Export (after G3)** writes the robot file as `project.robotProfile`, with an empty `paths[]` and shulib's
  command list.
- **Reserved, NOT built in G3a:**
  - `robot_tool.py import-vexbot FILE.vexbot [--id ID]` and `robot_tool.py export-vexbot ROBOT.json
    OUT.vexbot`;
  - the panel's command ids `shulib.importVexbot` and `shulib.exportVexbot`, not registered, so no menu
    offers what does not exist;
  - one menu slot each, left for G3 in the extension's view layout (§1.7's room for H4).
- **Today's `.vexbot`** (v2.0.0) carries no profile, so an import would fill only what is there (§4.7).

---

## 2. Tests (adversarial) and mutations

> **AMENDED 2026-09-29 — where each test now lands, and the new ones.** Every case must FAIL first on a
> deliberately broken tool, and every mutation is a fresh run, observed red and restored, then logged.
>
> **Part 1.** S1–S20 below, cut to the drivetrain half. S1's differential is `[chassis]`. The fields S13
> covers are the drivetrain's, `typedTable`'s and `sensors[]`'s. S17 keeps its sign, wheel and
> words-only cases; its mechanism cases move to Part 5. S19 `flip-front`, and if D7 is (a), the real flip of
> robot two re-checked against N1.
> - **S2x** — the mechanism keys refused by name in Part 1.
> - **Re-check 2026-09-29 additions (#13):**
>   - **S13** gains the literal-length rule: a `drivetrain.provenance` of 207 characters is refused, naming the
>     banner line; 206 passes, and so does robot two's 203.
>   - **S19** no longer claims byte-for-byte. Flipping twice restores the sides and the signs whatever words the
>     caller supplies; the provenance and the front's words are the caller's, and the baseline rule requires a
>     new provenance.
>   - **The kept facts** are checked on robot two's REAL flip, in Part 1's logged step and in §6: the new
>     provenance holds the census, MOTOR WATCH and cartridge facts and the flip, fits 206 characters, and N1's
>     8 of 8 is re-run on the real file. Free text cannot be checked for facts in general, so the tool does
>     not try.
>   - **The CROSS-CHECK sibling case** (D7): the flipped table against the 2026-09-25 record disagrees on
>     exactly 2 ports (14 and 19).
> - **Mutations:** M1–M18 as they apply to the drivetrain half. **M32:** the literal-length rule dropped →
>   S13's 207-character case red.
>
> **Part 2:**
> - **S21 new-robot.** In a temp copy, it generates a second robot. `check` is clean; the gate's step 0 is
>   clean (the tool imports the gate as N6's `step0.py` does); the SET regions hold exactly its lines; and
>   NOTHING under `src/team/` is written (re-check #2).
> - **S22 the collisions.** Ids `tank`, `bench`, `xdrive` and `robot_two`; `tank_drive` (its beacon is robot
>   two's drive build's); `probe` plus `probe_calibrate`; two ids that CamelCase alike; **`tank_2026` and
>   `xdrive_invented`** (their DEFINE is robot two's / the X-drive's, the only name that collides — re-check
>   #11). *(re-check 2026-10-29: and `tank_library`, whose Bench Tests beacon is the gate's RETIRED one — RC21)*
>   *(re-check 2026-10-29, CK8:)* and the NAMESPACE cases — `new` and `and` (keywords), `std`, `hal` and `robot`
>   (namespaces `src/` names unqualified — V8), `claw` beside an existing `claw_bot` and the reverse (the team-file
>   prefix), `robot__two` and `robot_` (the pattern) — each refused naming `file.id`; `robot_three` passes and BUILDS.
>   **M33:** the namespace check dropped → those cases red.
> - **S24 the SET regions' own differential.** The zero-robot emission equals the adopted bytes (N3).
> - **S25 deleting the robot file.** `generate` empties the SET regions, `check` is clean, and the gate's step
>   0 is clean.
> - **S26** an emitter planted to write invalid Python into `[gate builds]` gives a `ToolFault` from
>   `compile()`, and nothing is written.
> - *(Re-check 2026-10-29: STRUCK — RC2, RC3.)* **S35 the front reaches Calibrate (re-check #3).**
>   - The generated robot's table carries `.front` equal to its file's words, and so does robot two's
>     regenerated region.
>   - A front of 161 characters is refused; 160 passes.
>   - In the gate's generated-robot case, the probe robot's Calibrate build links. With the plant "the
>     emitter drops `.front`" it FAILS on the `static_assert` naming the rule, while its Bench Tests build
>     still links (`recheck2/front/sa_probe2.log`).
> - **S36 the guards read the Makefile (re-check #9).**
>   - A plant adds `autontest` to the hand-written `SHULIB_ROBOT_PROGRAMS_tank`: `check` names `[robot guards]`
>     STALE (never "edited by hand"), and `generate` re-emits it with the new guard.
>   - A `SHULIB_ROBOT_DEFINE_` line with no programs line is a `ToolFault`, and nothing is written.
>   - The tool's source carries no hand-written robot define: a grep of `tools/robot_tool.py` for
>     `SHULIB_ROBOT_TANK_2026` and `SHULIB_ROBOT_XDRIVE_INVENTED` finds them only in S22's and S36's plant text.
> - **The src gate's self-test case** builds the generated robot's two bring-up builds.
> - **Mutations:**
>   - M19: new-robot writes the chain but not the gate's rows → the gate's generated-robot case red (N6 a);
>   - M21: the beacon-collision check dropped → S22 red;
>   - M22: the kEveryRobotId list region not emitted → the host per-robot loop misses the robot (a test that
>     counts `kEveryRobotId` against the `robot ids` region's lines turns red);
>   - **M27:** the define-collision check dropped → S22's `tank_2026`/`xdrive_invented` cases red;
>   - *(struck, RC3)* **M30:** the chassis emitter drops `.front` → S35 red, and robot two's `ROBOT=tank PROGRAM=calibrate`
>     build fails the gate on the `static_assert`;
>   - **M31:** the guard emitter types the hand-written robots instead of reading the Makefile → S36's plant
>     (a program added to the Makefile line) leaves the region "fresh", so S36 turns red;
>   - (M20 moved to Part 4 with the starter.)
> - **The `kEveryRobotId` case in `chassis_table_test.cpp`** (re-check #13): its mutation — the generated
>   robot's selection handed `kNoChassis` — is observed red (probed: 4 failures in exactly that case).
>
> **Part 3** (`node --test`, pure model):
> - the new-robot page shows the dry run's file list and calls the tool, never writing a file itself;
> - the drivetrain page's front button calls `flip-front`, never editing the words (the old P4 mutation);
> - the record view as below;
> - Save runs validate → generate → check, in that order, and stops at the first refusal.
> - Plus Part B's model tests and mutations below (P1–P4, R1–R3).
>
> **Part 4:**
> - **S23** (moved from Part 2) new-robot refuses an existing `src/team/<id>_driver.hpp` and writes NOTHING;
>   `generate` never overwrites an existing starter.
> - **S27** (moved from Part 2) an empty control list is emitted as a span, never `[] = {}` (N4), in the
>   controls vocabulary the core leaves. *(re-check 2026-10-29: moot — the starter emits `controls(k<Id>Mechanisms, …)`; S27 becomes "a starter for a robot with no mechanisms compiles with no controls, and one with a mechanism and no control is refused by name", V2; M20 likewise — RC11)*
> - The starter driver file compiles under the host flags (the host TU over every `src/team/*_driver.hpp`) and
>   in the generated robot's match builds; a robot created with `tester calibrate` gains its starters when
>   `drive` joins its programs.
> - **S33 the bound listings (`api_doc_tool.py self-test`; re-check #1; each case the `docscan/` probe ran):**
>   - an UNNAMED listing whose lines only a driver file holds FAILS, and its message names that file;
>   - the same listing NAMED passes, and is still held to one unbroken run (today's skip, reorder and
>     added-line cases, re-run with the directive);
>   - a stale named quote FAILS even when a SECOND driver file holds the stale line — the checker's `wr_with`;
>   - a directive naming a file no `EXAMPLE_SOURCE_GLOBS` entry matches FAILS;
>   - an example-test listing needs no directive (the membership rule is unchanged for it).
> - step 0's new rule in the gate's self-test: a program naming `RobotTwoDriver` or a robot's auton function
>   outside the selection → FAIL, and the same name inside a `static_assert` → pass;
> - a selected robot whose driver file lacks its `DriverFor` specialization → the compile error names the
>   robot;
> - the gate's generated-robot case builds the three match programs;
> - an EMPTY auton list → a stated screen, never a crash (checked at the tip).
> - **Mutations:**
>   - M20 (moved from Part 2): the empty-list emitter writes `[] = {}` → the starter's host TU fails to compile
>     (N4);
>   - M23: step 0's driver rule removed → its plant passes (red);
>   - **M28:** a named listing checked against the union of every source again → S33's `wr_with` case red;
>   - **M29:** the whole-run files kept in `known` → S33's unnamed-listing case red.
>
> **Part 5:**
> - the agreement test's mechanism fixtures against the MECH core's C++ (§1.6); *(re-check 2026-10-29: run-time and compile-refusal halves, RC6; A4-D27's case among them, RC7; and the `DriverFile` re-zero fixture matched by its `static_assert` text, CK9 — with its mutation **M34**: the validator stops reading the re-zero from THE FEEL → that fixture DISAGREES)* *(amended 2026-10-29 evening: M34 now — the validator stops checking the file's STATED re-zero against the controls → that fixture DISAGREES; the feel's own cases are S48 and M46–M47, D11)*
> - *(re-check 2026-10-29, CK7)* the `changeAt` length rule: a path of 161 characters refused naming the `%.160s` lines, 160 passes and reads back WHOLE through `judgeDirection` in a host case (V7's method); **M35:** the limit set to 200 → red;
> - **S28 the templates.** Each template's `add` produces a file refused ONLY at the EMPTY fields its row in
>   §1.14 lists, so a template that fills in a team's number is caught (P6);
> - **S29 Remove's effects.** They list every control, preset and wiring entry naming the mechanism, plus
>   every `src/team/`/`test/` line naming its handle *(CK2: no handles file after MECH-A5 — read "naming its C++ identifier, `<id>::<name>`, or one of its route ids")*; after the edit none remains in the file;
> - **S30** Duplicate carries no wiring and no controls; Move changes no value (the baseline rule is silent);
>   Rename renames the controls, presets and wiring;
> - **S31 draft.** A draft generates nothing, and a control naming a draft is refused;
> - **S32** a role with no wiring entry is refused, naming the role;
> - robot two's four regions *(CK2: THREE — `mechanisms`, `control words`, `controls`; RC4, RC5)*: their differential against the core's tree.
> - **S34 a generated robot WITH a mechanism, built (re-check #7).** *(re-check 2026-10-29: moves to Part 2, which runs after Part 5 — RC1)* In the gate's generated-robot case, the
>   probe robot gets one mechanism from a template, fully stated and wired, and its builds link. That proves
>   the "only robot two declares mechanisms" pin (`mechanism_table.hpp` :624-625) is gone or generalized, and
>   that the `[mechanism tables]` and handles regions compile for a robot that is not robot two.
> - *(Re-check 2026-10-29: STRUCK — the watch is CAL1's, RC3.)* If Part 5 owns Calibrate's declared-port watch (§5 item 7), its host test: a declared port whose motor
>   turned with the push is named; one that did not is not.
> - **Mutations:**
>   - M24: a template fills a team number → S28 red;
>   - M25: Remove leaves a preset control behind → S29 red;
>   - M26: Duplicate copies the wiring → S30 red.
>
> **Part 6** (`node --test`):
> - each page's model: a template picked → the tool's `mechanism add` is called;
> - a refusal lands on its JSON path, on the right page;
> - Remove shows the effects list before it calls the tool;
> - the wiring page marks every 3-wire line "not checkable on the brain".
> - The manual VS Code check grows as §1.7 says.

**Part A — `robot_tool.py self-test`** (plants in a temp copy of the relevant files, every plant
restored; each case must FAIL first on a deliberately broken tool):
S1 the differential on robot two (every region regenerates identically); S2 a hand edit → `check` names
the lines, `generate` writes nothing anywhere; S3 a stale region (robot file changed) → named;
S4 `--discard-hand-edits` restores; S5 a missing begin, a missing end, a nested begin, a duplicated
region, a near-miss marker (`// >> GENERATED`, `>>> Generated`, a wrong sha length) → each named;
S6 a region naming a robot file that does not exist; a robot file missing a region kind it needs;
S7 a non-canonical robot file → `check` names it, `fmt` fixes it, `fmt` twice is idempotent; S8 unknown
fields AT EVERY LEVEL (top, `drivetrain`, a mechanism, a motor, a hold) → a warning naming the path, kept
by `fmt`; S9 a CRLF copy of a region file → no false hand edit, and `generate`
keeps CRLF; S10 every byte outside the markers unchanged by `generate` (compared); S11 each validator
sentence pinned; S12 the pins-outside-regions anchors exist. **Added at the re-check** (each probed in
`recheck/battery.py`, 26/26, and each made red by its mutant in `recheck/muts/`):
- **S13 the text-field rule.** For EVERY known text field, each hazard is refused by its JSON path and
  nothing is written: `\n`, `\r`, `\t`, U+2028, NEL, the marker token, a trailing `\`, a `\`
  before a space, and a trigraph. A non-ASCII character in a literal-bound field is refused too. The
  benign text (`*/`, `/*`, `"`, a mid-word `\`, `—` in a comment) is ACCEPTED, and generate-then-check
  comes out clean.
- **S14 the generate⇒check invariant.** An emitter planted to emit a newline gives a `ToolFault`, and
  nothing is written anywhere.
- **S15 the marker-only repair.** A zeroed sha over correct lines: `check` fails "(marker only)",
  `generate` rewrites only the marker, and `check` is then clean.
- **S16 the collisions.** ~~Two pneumatics that give one constant; a pneumatic named only with
  punctuation;~~ *(re-check 2026-10-29, CK2: retired with `k<Name>OpenCommand` — in its place, two mechanisms whose names give one lowerCamelCase identifier, and a name that gives none, each refused unless an `id` separates them)* ids `bench_bot` and `no` against the hand-written tables; two robot files whose ids
  CamelCase alike (`robot_2`, `robot2`).
- **S17 the baseline rule.** P4's edit with the same provenance is refused, and with a new provenance
  it is generated. An `openLevel` flip without new notes is refused, and with new notes it passes with
  its warning *(re-check 2026-10-29, CK2: re-stated in the landed vocabulary — a swap of the claw's `positions` levels without new `notes` is refused, with new `notes` it passes with its warning; and a changed `wire(...)` port without a new provenance is refused, Part 5)*. A typed sign flip without a new `drivetrain.provenance` is refused. Robot two's wheel
  2.75 → 3.25 is refused. A words-only change passes. A renamed mechanism has no baseline and is not
  refused.
- **S18 `typedTable` on another robot** is refused.
- **S19 `flip-front`.** Sides swapped, every typed sign negated, the provenance required; flipping
  twice restores the file byte for byte except the provenance.
- **S20 the witness list.** A changed controls region names every `test/` file that names the robot's
  tables, and `docs/cookbook/06-driver-control.md`; an unchanged region names nothing.
**Part A — host suite:** `test/robot_tool_agreement_test.cpp` (§1.6). The existing robot-two tests run
unchanged (D5 (a)) — the adopted tree passed them (P2).
**Part A mutations (a FRESH run each; observe red; restore; log):** M1 the validator drops a rule →
agreement red (probed as V1); M2 a C++ predicate loosened → agreement red (probed as C1); M3 `generate`
ignores the sha → S2 red; M4 `check` compares only the sha, never regenerates → S3 red; M5 `fmt` drops
unknown keys → S8 red; M6 the emitter writes `12` for `12.0` → S1 red; M7 the marker regex accepts a
near-miss → S5 red; M8 the src gate's `[generated]` step removed → its self-test case red; M9 `generate`
writes region by region → S2's "nothing anywhere" red. **Added at the re-check:**
- M10: the text-field rule dropped → S13 red (probed: 9 red).
- M11: the emitters' guard dropped → S14 red (probed).
- M12: the marker-only repair dropped → S15 red (probed).
- M13: either collision check dropped → S16 red (probed: 1 and 2 red).
- M14: the baseline dropped → S17 red (probed: 4 red).
- M15: `typedTable` allowed on any robot → S18 red (probed).
- M16: the harness drops the pin's family → the agreement test red on the unsigned-table fixture
  (probed in `recheck/agree_pin/`: DISAGREE).
- M17: `flip-front` forgets the signs → S19 red.
- M18: the nested unknown-key warning dropped → S8 red (probed).
**Part B — `node --test`** (pure model): rows from robot two's file; an edit returns a new object and
changes one path; unknown fields survive; a refusal lands on its JSON path; not-JSON and a non-object are
named; the record view model (FRONT REVERSED, LEFT OUT 14 and 19 on robot two's 2026-09-25 record); the
"refuse to generate while a target is dirty" rule. **Host:** `test/robot_tool_record_agreement_test.cpp` —
the generated header holds Python-made record bytes (≥ P8's 20) with the Python decoder's sentence; the
test decodes each with `decodeRecord()` and CHECKs the sentences equal, and encodes robot two's
description and CHECKs the bytes equal the Python-side expectation. **Mutations:** P1 an in-place edit
(probed); P2 dropping unknown fields; P3 mapping by label instead of path; R1 the Python decoder skips the
CRC; R2 the Python `valid()` mirror drops "appears on BOTH sides"; **R3 the Python port-map line drops the
IMU** (the `writePortMap()` pin, re-check #10); **P4 the front button edits the words instead of running
`flip-front`** (the model test that it calls the tool) — each red.
**What cannot be tested headless, said plainly:** the webview's rendering, the custom editor's
lifecycle, a real click, VS Code's dirty-buffer handling and the Python lookup on Windows.
`@vscode/test-electron` could drive a real VS Code under xvfb, but it downloads VS Code — not in CI. The
executor runs a written manual check in a real VS Code and logs it: open robot two's file, change claw
pivot 7's forward to R1 (the refusal appears beside both rows), revert, change volts to 6, save, see the
two regions regenerate and the witness list name cookbook 06, `git diff`; change pivot 7's port to 8 and
see the baseline refusal beside its provenance, type a provenance, save; press "the OTHER end is the
front" and see the sides and the typed signs flip; restore.

---

## 3. Documentation (same change — pace rule 3; prose review logged — rule 4)

> **AMENDED 2026-09-29 — per part.** Old Part A's and Part B's lists below are split as follows. Every part
> also updates the internal documents — `build-order.md` (G3a's entry and the first bold `Next:`), HANDOFF,
> `REMAINING-WORK.md` and `G3a-PROGRESS.md` (a heading per part, created FIRST) — re-runs the C7 removability
> check, and runs `tools/briefing_status.py check`.
> - *(re-check 2026-10-29: the parts keep their numbers and their lists, run in the re-check's order, and write into TODAY's structure; DOCS3 re-homes them after the panel. Part S (~4: the robot-file chapter's camera and walls — *CK10: saying the mounts are measured from the RECORD's front, the end that led Calibrate's push, and that a flipped front means re-measuring them* —, `vexbuilder-integration.md` §3.1's two blocks, the glossary, the panel README) and Part AF (~3) are new; *(CK1: Part 5's robot-file chapter also gains "new vocabulary after Part 5: extend the tool, or state it outside the regions" — rule 6 — for the next chunk's author)* Part R's list comes with its section. `G3a-COMPLETED.md` moves from Part 6 to whichever pre-G3 part lands last.)*
> - **Part 1 (~6 public; re-check 2026-09-29, #4 — one past rule 7, said plainly):**
>   - the robot-file chapter, first version: the robot file's drivetrain half, the tool, the markers, `flip-front`, the
>     text-field, baseline and literal-length rules, and after a merge;
>   - `07-getting-set-up.md`;
>   - `vexbuilder-integration.md` §1's row and §3.1's drivetrain half;
>   - `roadmap.md`, `README.md` (its layout gains the robot file and `tools/robot_tool.py`), the glossary;
>   - if D7 is (a): the worksheet's WHICH END IS FORWARD and Station F (internal), and `roadmap.md` :191,
>     re-dated. `docs/hardware-assumptions.md` is only grepped: no new belief.
>   - Cookbook 06 is NOT touched in Part 1, since the controls regions are Part 5's.
>   - **Why ~6 and not ~5.** The FAQ's "why can't I type the track width?" moved to Part 3, which touches
>     the FAQ anyway and where the record view makes the question arise. The other six cannot wait:
>     - the tool cannot ship without its chapter (rule 3);
>     - §3.1 would contradict the robot file;
>     - the README's layout would omit the robot file;
>     - the glossary holds the terms the chapter introduces.
>
>     Splitting the tool from its chapter would ship an undocumented command.
> - **Part 2 (~4):**
>   - the robot-file chapter's "a new robot": `new-robot`, what it writes, removing a robot by hand, and adding one by
>     hand without Python; ~~the front's words at Calibrate and why a portless robot's front cannot be checked;~~ *(re-check 2026-10-29: "the front is the end you lead Calibrate's push with", RC2)*
>     the Makefile's hand-written robot lines and "edit, then generate" (re-check #3, #9);
>   - `07-getting-set-up.md` (`make ROBOT=<id>`);
>   - `RESUMING.md` §3, whose build count becomes "the hand-written builds plus each generated robot's";
>   - `README.md`;
>   - *(Re-check 2026-10-29: STRUCK — Calibrate prints neither, RC3.)* the worksheet's Calibrate station (internal): the `front      :` line and the LEAD THE PUSH line Calibrate
>     now prints.
>   - (`src/team/README.md`'s starter-driver-file paragraph moved to Part 4 with the file, re-check #2.)
> - **Part 3 (~5):** old Part B's list below (the robot-file chapter's panel section, `tools/robot-panel/README.md`,
>   `README.md`, the FAQ, `.gitignore`), plus the new-robot, drivetrain and calibration pages (the calibration
>   page's "no typed ports" message included). The FAQ gains Part 1's "why can't I type the track width?".
> - **Part 4 (~5):**
>   - the robot-file chapter (the match builds, and the starter driver file);
>   - `src/team/README.md` (the starter driver file, the starter auton file, `drivers.hpp`, `autonsFor`);
>   - the guide's auton chapter as COMP-t leaves it (the per-robot auton file), with its two
>     `<!-- example-source: src/team/autons.cpp -->` lines (re-check #1); *(re-check 2026-10-29: and chapter 17's one — six in all, RC12)*
>   - **cookbook 06: three `<!-- example-source: src/team/robot_two_driver.hpp -->` lines**, one above each of
>     its driver-file listings. They are invisible on the site, and the listings' text is unchanged (re-check
>     #1). The guide's README scope sentence ("the team's files under src/team/ that a chapter quotes") gains
>     "and names the file it quotes" if it describes how a listing is checked. The doc tool's own header
>     comment (:1833-1853) is source prose, rewritten with the rule;
>   - `RESUMING.md` (the build list).
> - **Part 5 (~5):**
>   - the robot-file chapter's mechanisms section;
>   - `vexbuilder-integration.md` §3.1–§3.2: roles plus wiring, rename 7, and the note to VexBuilder (§4.10); *(re-check 2026-10-29, CK2: in the landed words — each kind's members, the named-slot wiring, the controls' lists; "roles" and "rename 7" are retired, §1.11)*
>   - cookbook 06: the WHICH BUTTON listing re-quoted if the core's shape changed it;
>   - the glossary (role, wiring, template, draft) and the FAQ ("why can't I bind a port on the brain?" — Q1
>     (a)).
>   - Its prose-review grep is the old Part A's (below), re-run over the tree the core leaves.
> - **Part 6 (~4):** the robot-file chapter's panel pages, `tools/robot-panel/README.md`, `README.md`, and
>   **`docs/internal/chunks/G3a-COMPLETED.md`**. Part 6 closes G3a; Part 7 is a later chunk with its own brief. *(re-check 2026-10-29: the completion record moves to the last pre-G3 part — R, S or AF)*

**Part A** (~8 public documents plus the source prose — past pace rule 7's ~5, said plainly):
1. **NEW `docs/guide/16-your-robot-file.md`** *[re-check 2026-09-29: number 16 is now COMP-t's
   (`16-testing-an-auton-on-the-bench.md`, merged into the main tree while this re-check ran), and 17 is
   COMP-b's (COMP brief :403). The robot-file chapter takes the next free number at Part 1's tip, 18 when
   COMP-b has landed, as `docs/guide/18-your-robot-file.md`. "Chapter 16" anywhere else in this brief means
   this chapter.]* *(re-check 2026-10-29: 18 is MECH's mechanisms chapter; the next free number at Part 1's tip — 21 if CAL1-A2a's and VIS1-A2's chapters land first; DOCS3 may renumber — RC17)* — the robot file field by field (what a human states, what
   is measured and where it is shown), `generate`/`check`/`fmt`, the markers and what may be edited
   around them, what the checks refuse and in which words, robot two's file as the example, and "adding a
   robot" as the manual steps it is (RobotId, the chain, the Makefile's `ROBOT`, the gate's rows, the
   selection cases, then `adopt`). **Also, from the re-check:** the text-field rule and why (#1); robot
   two's legacy `typedTable` and why no other robot may have one (#2); "a changed fact needs a changed
   provenance" (#5); changing the front with `flip-front` (#15.4); the witnesses — in this repository a
   controls change also means re-quoting cookbook 06 and changing the host pins (#3); and **after a git
   merge** (#11): resolve the robot file first; in each region take one side WHOLE (its marker line
   included) and run `generate` — a region whose lines match the file only needs its marker rewritten,
   which `generate` does; if it names a hand edit, the region was mixed, so run
   `generate --discard-hand-edits` (the robot file is the truth); then `check`. Say plainly that a hand
   edit whose sha someone recomputed reads as "stale" and is overwritten from the robot file. Added to `docs/guide/README.md`'s table (:7-23) and `mkdocs.yml`'s nav
   (:120-135); chapter 7 links to it. (Numbered last to avoid renumbering chapters, now 18 (above); the
   executor may propose placing it after chapter 7 only with the coordinator.)
2. **`docs/guide/07-getting-set-up.md`** ~:125 (the driver file sentence) and the repo layout: the robot
   file, `tools/robot_tool.py`.
3. **`docs/cookbook/06-driver-control.md`** :213-225 and the "Why" bullets (:273-296): the WHICH BUTTON
   listing is generated from the robot file; "change a button" means the robot file or the panel, and
   then re-quoting this listing, because the documentation build fails until it is re-quoted (the
   whole-run rule; re-check #3).
4. **`docs/vexbuilder-integration.md`** §1 (:66 G3a's row), §3.1 (:96-126, rewritten — §1.11), §3.2
   (:128-175: the measured fields, positions + polarity, controls), §3.4 (:187-198: the codegen is G3a's
   generator; G3 feeds it), §7's F7 row (:473).
5. **`docs/roadmap.md`** — C′ row (:2037), F7 row (:141: "drafted in G3a"), "You are here".
6. **`README.md`** — one paragraph: robots as data, the tool (Part B adds the panel).
7. **`docs/guide/15-glossary.md`** — robot file, generated region, freshness gate.
8. **`docs/faq.md`** — "why can't I type the track width?" (the ruling; where the number comes from).
- **The out-of-region prose pass — by GREP, not by a hand-made list** (re-check #8: the draft's list
  missed more than half the sites). At the tip (post-1b), across `src/`, `include/shulib/`, public
  `docs/`, `README.md` and `test/` comments, grep:
  `port[s]? 6|port 7|6 and 7|R1 ?/ ?R2|L1 ?/ ?L2|R1 forward|L1 forward|3-wire (port )?A\b|port A\b|\+6 and \+7|one place to edit|HERE and nowhere|LIVE HERE|lives HERE|THIS is the`
  and put every hit in one of three classes, logged in the PROGRESS file:
  **(A) DATED HISTORY** — true as of a named date or chunk ("T3: the pivots on 6 and 7", "since T3's
  second part") — stays; the executor checks the date really is there. **(B) PRESENT-TENSE
  RESTATEMENT** of robot two's wiring or controls, or a "here" that now means the robot file — rewritten
  to point at the robot file ("robot two's entries: src/robots/robot_two.robot.json, generated into the
  region below") or re-dated ("as of T3"). **(C) A TEST LITERAL** — a witness (D5), kept; `generate`'s
  witness list names it.
  **Today's hits** (at `a62067b`, pre-1b; the executor re-greps): (B) `chassis_table.hpp` :22-26 ("THIS
  is the one place to edit"), :28-36 ("SIGNS LIVE HERE" — the robot file's `typedTable`);
  `mechanism_table.hpp` :13-24 (WHAT IT DECLARES — keep its design reasoning, re-date its facts), :38-42,
  :47 ("the port-7 pivot's forward button"), :54-59 ("+6 and +7 … flipping ITS sign here"), :61-70 (THE
  POLARITY), :72-79 (ONE LINE: "this entry"), :81-91, :93-115 (WHO READS IT gains the tool; :97's log line
  is an example, "e.g."), :622-623 (the pin's message — D5 (a) rewrites it); `robot_two_driver.hpp` :5-11
  ("change it HERE and nowhere else", as 1b restores it → "change src/robots/robot_two.robot.json, by
  hand or in the robot panel, and run the generator; the lines between the GENERATED markers below are
  written from it"); `src/team/README.md`; `calibrate_program.cpp` :86-88 and :1120 ("robot two's claw
  pivots, 6 and 7" → "as of T3"); `teleop_program.hpp` :20-22, `teleop_program.cpp` :30-37 and :167,
  `teleop_claws.hpp` :21 — "shulib Teleop" retires at T4 Part 3, so if Part 3 has landed they are gone,
  and if not they are re-dated. (A) `teleop_program.hpp` :17 and `teleop_program.cpp` :146-149 (a RUNTIME
  string, `kRunStatus`, quoted verbatim by the worksheet :1322 — dated "T3:", left);
  `xdrive_program.cpp` :96-97 (dated "— T3"; its :107-111 check "never a shoulder button" is a policy,
  not a restatement); the API doc of `include/shulib/` (`docs/api/boot_check.md` :468, "(T3 Part 2)" —
  and no `include/` change is made); `docs/guide/14-what-it-cannot-do-yet.md` :347-350 and
  `docs/roadmap.md` :363-365, :1735-1738 (dated to T3). `mechanism_table.hpp` :158 leaves with
  `kTeleopRezeroButton` in T4 1b. **In-region prose (through the robot file, after the differential):**
  `openLevelNotes` and the `.openCommand` comment (§1.3). *(re-check 2026-10-29, CK2: RETIRED — after MECH-A5 the in-region prose is each mechanism's `notes` and the claw's `positions` caveat; Part 5 lists it from A5's block at its tip.)*
- **Greps to run and read (rule 4), besides the pass above:** `mechanism_table.hpp`,
  `chassis_table.hpp`, `robot_two_driver.hpp`, `kClawOpenCommand`, `robotProfile`, "WHICH BUTTON",
  "robot panel", "G3a" across `docs/`, `README.md`, `src/`, `test/`. *(re-check 2026-10-29, CK2: `kClawOpenCommand` is gone after MECH-A5 — grep it to find STALE prose only; add `kRobotTwoMechanisms`, `driver_file.hpp`, `controls(`, `wire(` and `changeAt`.)*
- **Internal:** `docs/internal/build-order.md` (G3a entry status; the first bold `Next:`), `HANDOFF`
  (~:546-547), `G3a-PROGRESS.md` (first), the briefing (`tools/briefing_status.py check`).
- **Deliberately NOT touched:** `docs/changelog.md` (no library header changes; it records the library);
  `docs/api/` (no header changes — `check-fresh` proves it); `docs/hardware-assumptions.md` (§1.12).

**Part B** (~5): the robot-file chapter (the panel section, the record view, the manual
install); NEW `tools/robot-panel/README.md` (build, test, package, install, what is untested and why);
`README.md` (the optional panel); `docs/faq.md` ("do I need VS Code / Node?" — no); `.gitignore`;
internal: build-order, roadmap, HANDOFF, **`docs/internal/chunks/G3a-COMPLETED.md`** (Part B closes the
chunk). C7 removability re-run (no public document names `docs/internal/`).

---

## 4. OUT, and explicitly rejected

> **AMENDED 2026-09-29.**
> - **Moved IN:** adding a robot from the panel (§1.13, D8–D10); robot two re-stated toward the camera end
>   (D7, if (a)). *(Re-check 2026-10-29: and routes in the panel (Part R, ruled 2026-10-08), the camera and side-wall blocks with their page (Part S), the auto-flipper's page (Part AF) — RC9, RC10.)* *(amended 2026-10-29 evening: and the driving feel (D11), drive tuning (D12), the full route maker with its F8 export (D13), and the release job (§5).)*
> - **Stays OUT:** ~~THE FEEL (D4),~~ *(amended 2026-10-29 evening: THE FEEL moved IN — D11)* the `.vexbot` reader and the F7 freeze (G3), H4, PKG1, and any
>   `include/shulib/` change *(amended 2026-10-29 evening: still none in any G3a part — the route maker's library half is its own chunk, STEP1, R.13)*. The mechanism LIBRARY — the service, the kinds, the binding checks at boot, the
>   bench page, the direction check, the auton steps and the handles' C++ types — is the MECH core's; G3a
>   generates DATA for it and never writes that library.
> - **New OUT items:**
>   - removing a robot from the panel (a manual procedure, §1.13);
>   - X-drive (or H-drive) generation, unless §5 Q22 is (b);
>   - guided binding ("which port moved") and the SD binding record (Q1 (a): none; guided binding is the rest's,
>     Part 7);
>   - the rest's kinds, settings and controls until Part 7;
>   - a "Build this robot" button that gates anything (§1.15);
>   - import and export, whose hooks are named only (§1.16);
>   - the wrong-kind warning for anything the drivetrain record does not carry (§1.14);
>   - *(re-check 2026-10-29: moot — no front words, RC2)* the file's front words on Calibrate's SCREEN: the WHICH END FIRST? question is the library's panel, and
>     G3a makes no `include/` change (the words reach the serial log; §8, re-check #3).
> - **Newly rejected:**
>   - per-robot marker insertion for the build plumbing (D9 (b));
>   - the gate deriving its rows from the robot files (D9 (c));
>   - a `[] = {}` empty control list (N4);
>   - templates or mechanism edits implemented in the panel (D3).

- **OUT:** ~~adding a robot from the panel (the one chain's territory — the guide documents the manual
  steps);~~ *[moved IN 2026-09-29]* the bench bot's and the X-drive's tables (hand-written; the X-drive is a `RobotConfig`); ~~THE FEEL
  (D4); autons (COMP-t/COMP-b);~~ *(amended 2026-10-29 evening: THE FEEL is IN (D11); autons drawn in the panel are IN as generated code in a `[routes]` region (D13) — hand-written autons stay the team's)* the `.vexbot` reader, the SD-card runtime loader, `inferDrivetrain()`,
  `schemaVersion` negotiation and the F7 freeze (G3); the robot console (H4); the starter project (PKG1);
  Calibrate's and the boot's "claw pivot" words (§8); ~~re-stating robot two toward the camera end (§8);~~
  *[D7, 2026-09-29]* D5 (b)'s structural test pins unless chosen; any `include/shulib/` change.
- **Rejected:** a TypeScript (or second) validator; a JSON parser on the brain or runtime JSON (§9
  "no file parsing in the robot's hot path"); `make` running Python; generating the whole table files;
  markers without a sha (they cannot tell a hand edit from a stale region); the panel writing C++ itself.

## 5. Coordination — shared files, and who re-reads whom

> **(re-check 2026-10-29)** The MECH core's brief honoured items 1–9 below (MECH §6.2; each DONE or carried, RC19); the CAL1 brief took items 5 and 7 (RC3); the HA steps are DONE (RC16); "the file split with the core" is moot — the foundation starts after MECH-A5 (RC1). **The shared files with VIS1, L6, AF1, ALLY1 and MECH-A6, and the rebase rule, are the re-check's "Building beside VIS1, L6, AF1 and MECH-A6".** New asks for the coordinator: MECH-A5's prompt — end `[driver file two]` before THE FEEL (RC5); ~~MECH-A6 (or MECH's rest)~~ **MECH's rest or a small library chunk** *(CK11 (e): MECH-A6 writes only `src/programs/bench_r3a.*` and its test; a `wiring.hpp` field is an `include/` API change)* — a `changeAt` on the wiring entry (RC8); every chunk landing after a region exists — the rebase rule's point 3.
> **(re-check 2026-10-29, CK1) Asks to the briefs being written now (NOTES 10-29 ~12:20: AF1, L4, DOCS3) and to MECH's rest — the rebase rule's point 6:**
> - **AF1's brief:** (1) say whether AF1's robot-two half lands before or after G3a Part 5, and so which side carries the robot-file vocabulary — after Part 5, AF1 extends `tools/robot_tool.py` itself (emitter, validator sentence, `describe`, agreement fixtures) and KEEPS its plan line's "panel generator ~½–1½" in its own size; before Part 5, Part 5 adopts AF1's form as landed; (2) declare the flipper as an ordinary `arm` where today's members suffice, so only the automatic action is new vocabulary; (3) if the automatic action is a new form INSIDE `controls(...)` (it must be, if it takes the flipper's buttons or replaces its `autonOnly`), it is rule 6 (i) only — no "outside the region" path exists; if it is a separate statement (e.g. its own `inline constexpr` beside the list), rule 6 (ii) is open and Part AF adopts it later; (4) name every fact the flipper's automation states (the Toggle's colour source is ALLY1's, not the robot file's) so Part AF's page has a list; (5) any C++ refusal it adds that the robot file can reach carries its agreement fixture (rule 4).
> - **L4's brief:** (1) whether "drive to a piece" adds a ROUTE STEP to F8's format (Part R's vocabulary) — if so, Part R's section adopts it; (2) whether it needs any robot-file fact (it should not: the camera block is VIS1's, the confirming sensor a mechanism's `pieceSensor`) — if it does, rule 6.
> - **MECH's rest (its brief, later):** it may add library vocabulary freely; it uses new vocabulary in ROBOT TWO's regions only by rule 6 (i) after Part 5 — else robot two keeps today's vocabulary until G3a Part 7.
> - **VIS1-B, L6-B and CAL1 (§8, CK10):** the camera's and the wall sensors' mounts are measured from the record's front; a Calibrate that flips the front leaves them mirrored — worth one line each in their chapters, and in CAL1's flip warning on the brain.
>
> **AMENDED 2026-09-29.** T4 Part 1b no longer exists: it was folded into T4 Part 3 on 2026-09-28. Read every
> "1b" below as "T4 Part 3", which lands before any G3a part (the core follows it). Its first bullet's D6 (c)
> branch is moot. **New coordination:**
> - **The MECH core — the vocabulary.** Before Part 5's go, the coordinator re-checks §1.14 against the
>   core's brief (`briefs/MECH-core.md`), field by field and template by template, logs it, and amends §1.14.
>   The robot file carries exactly the core's vocabulary.
> - **The MECH core — what its brief should carry for G3a** (sent to its writer now, while it is drafted in
>   parallel):
>   1. keep `mechanismTableFor()` (or its successor) as a case-per-robot switch with no default, where Part 2
>      puts its `[mechanism selection]` SET region. A NEW per-robot selection function the core adds (a
>      binding's, a handles') needs a SET region of its own, adopted by Part 5;
>   2. write robot two's typed handles, table, binding and polarity in a form a region can wrap byte for byte
>      (one contiguous block per kind, no interleaved hand-written lines), so Part 5 adopts them unchanged;
>   3. give the direction check's "change it here" message ONE per-role string field in the table, which the
>      core fills by hand ("src/robots/mechanism_table.hpp, <robot>, <mechanism>, <role>") and Part 5
>      generates as the robot file's JSON path (Q1 (a));
>   4. D5 (a)'s mechanism half: the rewritten selection pin keeps the ADDRESS check and drops the counts;
>   5. Calibrate: the core's edit of the declared-mechanism-port drop (`calibrate_program.cpp` ~:1120-1131)
>      and Part 2's hunks are separate: the banner and refusal words (:1044, :1068, :1141), the two front lines
>      (:1086, :1239) and the front `static_assert` near the includes (re-check #3). Say which lands first;
>   6. **(re-check #7)** the pin "only robot two declares mechanisms" (`mechanism_table.hpp` :624-625) refuses
>      every generated robot's mechanism table. Rewrite it the way `chassis_table.hpp` :224-237 pins the
>      chassis:
>      - the generic `&kMechanisms == &mechanismTableFor(kRobotId)`;
>      - the hand-written robots that declare none, BY NAME: "the bench bot and the invented X-drive declare no
>        mechanisms".
>
>      If the core leaves it, Part 5 does it (S34 builds a generated robot with a mechanism);
>   7. **(re-check #10)** Calibrate WATCHES the declared mechanism ports during the push. It reads each one's
>      position without commanding it, gearing it or setting its brake mode (T3's rule for a claw stands), and
>      prints "port N is declared as [mechanism]'s [role] but TURNED WITH THE PUSH: it may be a drive motor,
>      check the wiring" for any that moved like a drive motor. That is the only catch for a portless robot
>      whose role is wired to a drive motor (D10's costs). The core edits that hunk, so the watch is its to
>      carry; if it leaves it, Part 5 does it (+~½ day);
>   8. **(re-check #6)** if the core's rewrite of `test/mechanism_table_test.cpp` keeps a per-robot loop, the
>      loop iterates `kEveryRobotId` once Part 2 has landed;
>   9. **(re-check #12)** its first free Hardware Assumptions number is **HA-182**. HA-181 is G3a's reserve.
> - **The file split with the core (speedup D).** Parts 1–3 build beside the core, each in its own worktree.
>   - **They touch no file the core's `src/` half rewrites, with TWO exceptions** (re-check #6: the draft
>     said one). Whichever lands second rebases each, and a conflict in either is a few lines:
>     - Part 2's two marker lines inside `mechanismTableFor()` (`src/robots/mechanism_table.hpp`);
>     - Part 2's one-line change of the per-robot loop in `test/mechanism_table_test.cpp` (:397, to iterate
>       `kEveryRobotId`), a file the core rewrites as its table's host test.
>   - **Parts 1–3 write** `tools/robot_tool.py`, `tools/src_build_gate.py`, `src/build_selection.hpp`, the
>     `Makefile`, `src/robots/chassis_table.hpp`, `src/robots/mechanism_table.hpp` (the two marker lines
>     above), `src/robots/*.robot.json`, the three host tests' robot lists (`test/mechanism_table_test.cpp`'s
>     included, above), the new host tests, `test/CMakeLists.txt`, `.github/workflows/ci.yml`,
>     `tools/robot-panel/` and the documents — plus Calibrate's hunks (above). **They write nothing under
>     `src/team/`** (the starter driver file is Part 4's, re-check #2).
>   - If the core's brief names any of these files, the coordinator sequences that part after the core.
>   - Parts 4–6 come AFTER the core by the ruled order, since they touch the programs, the mechanism table, the
>     driver files and `tools/api_doc_tool.py` (Part 4's bound listings; COMP-t edited the same globs — Part 4
>     re-reads them at its tip).
> - **COMP-t / COMP-b / T4 Part 3's programs (Part 4).** Part 4 replaces their direct `RobotTwoDriver` and
>   `registerAutons` with `DriverFor<kRobotId>` and `autonsFor(kRobotId)`. It re-reads them at its tip, and the
>   programs' host source pins move with them.
> - **The gate's owner.** Every chunk that adds a build (COMP-t's seventh, COMP-b's eighth) edits the gate's
>   hand-written tables OUTSIDE the `[gate builds]` region. Parts 2 and 4 re-read the gate at their tip.
>   **From Part 2 on, a chunk that adds a program to robot two (or any hand-written robot)** also edits the
>   Makefile's hand-written `SHULIB_ROBOT_PROGRAMS_<robot>` line (make refuses the program otherwise). It then
>   runs `python3 tools/robot_tool.py generate`, which re-emits `[robot guards]`, and never edits inside that
>   region. Until it does, `check` names the region STALE ("run generate"), never "edited by hand" (D9;
>   re-check #9). The coordinator says so in every such chunk's prompt.
> - **Hardware Assumptions numbers (re-check #12) — the coordinator's, since this brief may edit only itself.**
>   HA-181 is G3a's reserve, but only this brief says so.
>   - The briefing computes "next free" as the register's highest table row + 1 (`tools/briefing_status.py`
>     :111-117), so today it says HA-181 (`PROJECT-BRIEFING.md` :116).
>   - The register records held numbers in prose only ("HA-162..HA-176 are not in it: numbers other chunks
>     hold", `docs/hardware-assumptions.md` :88 at the COMP-t tip).
>   - Before any other brief or chunk can take HA-181, the coordinator:
>     1. adds HA-181 (G3a's) to that held-numbers sentence;
>     2. makes the briefing's next free skip held numbers — a small `briefing_status.py` change, its own go.
>        Until then it adds a note OUTSIDE the generated block ("HA-181 is held by G3a: the first free is
>        HA-182"), since a hand edit inside the block fails `briefing_status.py check`;
>     3. tells the MECH core's writer (item 9 above), and every brief written before G3a lands, that the
>        first free number is HA-182.

- **T4 Part 1b — the prerequisite.** Same three files; G3a targets 1b's shapes. If 1b has not landed,
  G3a waits (D6 (a)). **If the team lead chooses D6 (c)** (Part A first), the coordinator amends T4 1b's
  brief BEFORE 1b launches: its table change goes THROUGH the generator — the seven emitted lines leave
  the emitter, the pivots' provenance rewrites go into the robot file, and `generate` writes the region
  (a hand edit inside a region fails `check`); 1b's executor runs `robot_tool.py check` in its verify.
  Either way, T4 1b's and Part 3's prompts name the three crashing tests (§8).
- **T4 Parts 2 and 3.** Part 2 adds a header paragraph to the driver file — outside the regions. Part 3
  writes the driver file's final form (T4 brief §2 :1394-1435): if G3a lands first, the coordinator
  amends T4's §2 to carry the markers and Part 3's executor keeps them; if Part 3 lands first, G3a adopts
  its form. Part 3 retires `teleop_claws.hpp`/`controlsMatchTable`.
- **COMP-b / COMP-t.** `opcontrol()` runs the driver file; nobody but `generate` writes inside the
  markers after Part A — the coordinator says so in their prompts. COMP-t's `src/team/autons.cpp` and
  `EXAMPLE_SOURCE_GLOBS` are not touched by G3a.
- **TANK1** (in flight): no shared code file; shared plan documents only (build-order, roadmap, HANDOFF).
- **SAFE-SAVE:** the `.bak` rule (its brief ~:482-483): if it lands before Part B, B's record view shows
  both files and says which the brain uses (one rule, mirrored and pinned like the decoder); if after,
  SAFE-SAVE adds `.bak` to `robot_tool.py record` — the coordinator amends its brief.
- **R5a:** `shulib_characterization.bin` — the same rule; R5a's OUT line (~:1310) names it G3a's.
- **H4:** a second view in this extension later (§1.7). **PKG1:** §1.10. **G3:** §1.11.
- **IMU1, DIAG1 B, L9, H1:** no shared file.

## 6. Checks (the coordinator re-runs every one)

> **AMENDED 2026-09-29.** The commands below hold, for every part. The panel's line is Parts 3 and 6. Added:
> - **From Part 2 on, the coordinator's own new-robot probe:** in a scratch copy of the part's tree, run
>   `robot_tool.py new-robot` with an invented probe robot (never robot one's facts), then
>   `src_build_gate.py check` (all builds, the generated ones included) and the host suite. This is N2's
>   method, now through the real tool. **Part 2** also checks ~~the probe robot's Calibrate build prints its
>   front words, by grepping the linked package for them (re-check #3), and~~ *(re-check 2026-10-29: struck, RC3)* that adding a program to the
>   Makefile's hand-written `SHULIB_ROBOT_PROGRAMS_tank` makes `check` say STALE (#9). **From Part 4** the
>   scratch copy's `api_doc_tool.py check-examples` runs with the probe robot's starter driver file present
>   and one of robot two's quoted lines made stale. It must FAIL, naming the listing: the checker's `wr_with`,
>   through the real tool (#1).
> - **Every part:** `git diff --stat -- include/` is empty, and the PROGRESS log carries that part's
>   differential and mutations.
> - **Part 1 with D7 (a):** the flip's diff is exactly the region's `.left`, `.right` and `.provenance`
>   lines plus the notes, and N1's comparison re-run on the real file. The new provenance keeps the census,
>   MOTOR WATCH and cartridge facts and fits 206 characters, as `recheck2/n1/` showed. The re-pin grep's hits
>   are logged by class (D7), and the CROSS-CHECK sibling case gives 2. *(Re-check 2026-10-29: the re-pin floor is 22 files at `73044ca`, V6; the provenance cites the record and fits 206, V5 — RC14.)*

```sh
python3 tools/robot_tool.py self-test && python3 tools/robot_tool.py check
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check
python3 tools/api_doc_tool.py check-examples && python3 tools/api_doc_tool.py check-removability
# the ARM header gate (RESUMING §3), unchanged
# Part B:
(cd tools/robot-panel && npm ci && npm test && npx vsce package)
```
Plus: the differential in the PROGRESS log (sha256 before/after, `check` output); `git diff --stat --
include/` empty; every mutation's red run logged; the manual VS Code check logged (Part B).

## 7. Landmines
1. **Line endings.** Hash region lines WITHOUT their `\r`; write back in the file's own convention. A
   Windows checkout with `autocrlf` must not read as "edited by hand" everywhere (S9).
2. **Encoding.** Open every file with `encoding="utf-8"` (Windows' default is not); the notes carry
   `—` and `§`. JSON is written with `ensure_ascii=False`.
3. **Float spelling.** `volts: 12` is `12.0` in the table and `12_volt` in the driver file; both are
   deliberate (P1 proves them); `12.5_volt` is legal (`units/literals.hpp` :52/:58). *(re-check 2026-10-29, CK2: after MECH-A5 the table carries no volts — a hold's volts are `whileHeld(m, f, b, 12_volt)` in `controls(...)` (`driver_inputs.hpp` :12-17, mech-a4), and a power's `maxVolts` a `units::Voltage`; Part 5 re-reads each spelling the landed headers take — the hazard, one value spelled two ways, stands.)*
4. **The pre-1b emitter pairs controls to mechanisms by NAME** — two mechanisms with one name cannot be
   emitted (P6); post-1b the table carries no buttons and that fixture becomes emittable. *(re-check 2026-10-29, CK2: history — `controls(...)` names each mechanism by its C++ identifier, and `wiring.hpp` refuses two names alike.)*
5. **Every derived C++ name can collide, and the tool refuses each case (re-check #6; S16):** *(re-check 2026-10-29, CK2: the first three bullets are RETIRED with `k<Name>OpenCommand` — no such constant exists after MECH-A5; in their place, two mechanism names that give one lowerCamelCase identifier, or a name that gives none, unless an `id` separates them; CK8 adds the robot's NAMESPACE, §1.2 and §1.13)*
   - ~~`kClawOpenCommand` is named from the mechanism, not the robot, so a second robot with a pneumatic
     called "claw" would define it twice;~~
   - ~~within ONE file, "claw" and "claw!" give the same constant;~~
   - ~~"!!!" gives `kOpenCommand`;~~
   - an id's tables can hit a HAND-WRITTEN name: `bench_bot` gives `kBenchBotChassis` and `no` gives
     `kNoChassis`/`kNoMechanisms`;
   - `robot_2` and `robot2` give the same CamelCase.

   The C++ would catch each as a redefinition, but without naming the robot file. The tool scans for
   every `inline constexpr … kName =` outside this robot's own regions, so a new hand-written name is
   covered without a list.
6. **The src gate's walk reads `src/`:** a `.robot.json` in `src/robots/` is not a source to step 0 or
   step 1 (P2: 24 sources counted, the JSON not among them) — keep it so.
7. **`driver_controls_test.cpp` ~:548 reads every file under `src/`** for a heading-rule count — a robot
   file must never contain `raiseCount(diag::FaultCode::ImuLost)` or `headingSample(` (it cannot today).
8. **Never write under a dirty buffer** (§1.7); **never partially** (§1.2).
9. **The record's scales are a product.** Never show "wheel diameter" from a record: inches per motor
   radian is wheel radius × ratio, which Calibrate cannot separate (§0.5).
10. **The front check is by port SETS**, not by any stored "front" (the record has none): a port LEFT OUT
    must not flip the verdict (robot two's record lacks 14 and 19 and still reads FRONT REVERSED — P8).
11. **Generated C++ is text built from text (re-check #1).** A text field that can hold a newline, a
    marker token, a trailing backslash or a trigraph can write live code, an unterminated literal, a
    region the tool can never read back, or a line splice that comments out the next line of C++ while
    `check` stays clean. The text-field rule (§1.1) is the validator's half. The emitter guard, the
    escaping `cstr()` and the generate⇒check invariant are the tool's half. Keep both halves: a rule
    only in the validator is one missed call away from writing code.

**Added 2026-09-29 (the amendment):**

12. *(Re-check 2026-10-29: MOOT in A4/A5's vocabulary — `controls(list)` with none compiles, V2; RC11.)* **An empty control list is not `[] = {}`** (N4): that is an error on the host (`zero-size array`) AND
    in the ARM build (`could not convert … to std::span`). Emit `std::span<const HoldControl>{}` /
    `std::span<const ToggleControl>{}`, or the span form of whatever list types the core leaves. Robot two's
    non-empty `[]` form stays byte for byte. *(Part 4's, with the starter; re-check #2.)*
13. **Markers outside C++.** The `Makefile` and `tools/src_build_gate.py` take `#` markers.
    - A Makefile comment line is harmless anywhere in an `ifeq` chain, but a region must never sit inside a
      recipe, where a `#` line would be passed to the shell.
    - The gate's region must stay valid Python: `compile()` before writing, S26.
    - The tool scans exactly those two files outside `src/`, never itself.
    - **In C++, a region holds WHOLE preprocessor blocks, never part of one** (re-check #9). A `//` marker
      inside a backslash-continued `#if` would comment out the rest of the joined line. So `[robot guards]`
      holds whole `#if … #error … #endif` blocks naming every robot's define, the hand-written robots'
      included, read from the Makefile. A helper macro cannot carry the hand-written part either: the gate
      allows only `#error` inside a switch-reading block (`src_build_gate.py` :138-141).
14. **Every derived beacon must be unique across ALL builds**, the hand-written ones included. The gate
    matches beacons NUL-terminated, which tells prefixes apart but not equals: `tank_drive` or
    `probe`+`probe_calibrate` would give two builds one beacon (S22).
15. **The guards are written in the general form once, by hand, then adopted.** Today's pairwise
    `XDRIVE && TANK_2026` guard does not grow; the COUNT form does, and a third robot define with the
    pairwise guard would let two robot defines through silently.
16. **A program naming a robot's driver class or auton function directly selects a robot outside the
    chain,** the defect class G1 Part 2c measured for tables. Until Part 4's step-0 rule exists, the only
    guard is review. N6 shows step 0 does not see it today.
17. *(Re-check 2026-10-29: CAL1's — its glue prints the facts' name; RC3.)* **Runtime words that name robot two** in a program every robot builds: Calibrate's banner and its
    cartridge refusal's "type it into src/robots/chassis_table.hpp" (:1044, :1068, :1141). Part 2 derives
    the banner from `kChassis.robot` and points the refusal at the robot file. Bench Tests' robot-two
    examples are dated history and stay.
18. **The gate's time grows with every generated robot** (~11 s per build here; five builds per robot under
    D8 (a)). The self-test's generated-robot case adds ~40 s (Part 2) and more with Part 4. Say so in
    RESUMING when it lands.
19. ~~**A generated robot's starter driver file is compiled by NO ARM build until Part 4** (nothing includes
    it). Part 2's host TU that includes every `src/team/*_driver.hpp` is its only compile until then.~~
    *[re-check 2026-09-29, #2: the starter is Part 4's now, so every match build compiles it from the day it
    exists, and Part 4's host TU compiles it under the suite's flags. The lesson stands: a file no build
    includes is a file no build checks.]*
20. **The mechanism vocabulary is borrowed, not owned.** *(Re-check 2026-10-29: now landed — the re-check's adoption table, RC4.)* Every §1.14 field is the MECH core's. A field G3a
    invents that the core does not build becomes a promise the robot file makes and the robot does not keep,
    and it freezes with F7 at G3. Re-check before Part 5 (§5). The controls vocabulary is the core's too: the
    starter driver file's bind call and lists (Part 4, re-check #2).

**Added at the re-check (2026-09-29):**

21. *(Re-check 2026-10-29: SUPERSEDED — no front words exist, RC2.)* **A portless robot's front is words, not a check** (#3). Nothing compares a portless file's front with
    the record (landmine 10). Its only effect is Calibrate printing it before the push, in the serial log;
    the screen's question is the library's. The calibration page must SAY "cannot be checked", never show a
    portless file as agreeing.
22. **A portless robot's mechanism-versus-drive rules check nothing at build** (#10). *(Re-check 2026-10-29: the watch EXISTS — CAL1 D4 (a), HA-187; the other order (a record taken before the port was declared) is the boot's declared-port rule; RC3.)* With no typed drive
    port, `mechanismsDisjointFromChassis` and the wiring page's "a drive or IMU port" pass a role wired to a
    real drive motor. Calibrate then DROPS that motor silently, and the record never holds it. Only
    Calibrate's declared-port watch (§5 item 7) catches it. The hazard starts at Part 5.
23. **A literal the brain prints on a fixed-size line is cut without a word** (#13). `vsnprintf` truncates
    silently. `provenance : %s` in a 220-byte buffer (Bench Tests, the old drive program) takes 206
    characters, and Calibrate's 240-byte lines take a 160-character front. The tool refuses longer. Robot
    two's provenance is 203 today; a fact-keeping flip provenance of 210 would have been cut.
24. **A second driver file weakens a membership scan** (#1). `check-examples` unions every example
    source, so a starter that repeats robot two's lines (the library's default feel, the bind and run calls)
    rescues a stale quote of robot two's file. Part 4's bound listings fix it; the starter must never land
    before them.
25. *(Re-check 2026-10-29: moot for G3a — no Calibrate `static_assert` is added, RC3; the lesson stands for any program file.)* **`calibrate_program.cpp` is compiled into EVERY build** (#3, probed): a Calibrate-only `static_assert`
    at file scope must be conditioned on `kProgram == Program::Calibrate`, or it fails the bench bot's and
    every generated robot's Bench Tests build. The same holds for any program file.
26. *(Re-check 2026-10-29: moot — no `front` member is added, RC3; the same lesson now holds for VIS1-B's `camera{}` and L6-B's `wallSensors{}`, whose defaults keep every table valid, V3.)* **The `ChassisTable::front` default is load-bearing** (#3, probed): both compilers refuse an omitted
    designated field that has no default member initializer (`-Werror=missing-field-initializers`). The
    `= nullptr` keeps the hand-written tables and every test-built table valid without an edit.

**Added at the re-check (2026-10-29):**

27. **`[chassis]` ends before the appended sensor members** (RC9, V3). VIS1-B's `.camera` and L6-B's `.wallSensors`
    are the last members of robot two's initializer; the region's end marker sits above them until Part S moves it (a
    logged re-adoption with the differential). A chunk that puts the camera's or the walls' notes in the region's
    comment makes `check` say EDITED BY HAND.
28. **MECH's `[driver file two]` block holds THE FEEL** (RC5). G3a's `controls` region wraps the WHICH BUTTON heading
    and the `kControls` statement only; wrapping the whole block would generate the feel D4 keeps hand-written. *(amended 2026-10-29 evening: D4 reversed — THE FEEL gets its OWN `feel` region, D11; the `controls` region still wraps only WHICH BUTTON and `kControls`, so the two never overlap)*
29. **The controls' refusals have no run-time sentence** (RC6). `controls(...)` is `consteval`; its rules are
    `refused::` names. The agreement test's controls half compiles each fixture and expects the named failure; an
    evaluator at run time sees nothing.
30. **The emitter writes each kind's members in the header's declaration order** (N11; V2's first draft): a
    designated initializer out of order does not compile, and the order differs per kind (an `Arm`'s ends `home`,
    `presets`, `unhomedVolts`, `tuning`).
31. **A generated robot's WIRING place points into a generated region** (RC8). The direction check names `wire(...)`'s
    captured `file:line`; for a generated robot that line is inside `[mechanism tables]`. Until MECH adds a wiring
    `changeAt`, the robot-file chapter says that "check the wire(...) line at src/robots/mechanism_table.hpp:N" means
    the robot file's `wiring` entry for that mechanism.
32. **A derived beacon can equal a RETIRED one** (RC21): `tank_library` → `shulib-robot-variant=tank-library`, which
    the gate refuses in every package with T4 Part 3's message. The collision rule reads the gate's `RETIRED_BEACONS`.
33. **A `changeAt` longer than 160 characters is cut by the direction check** (RC8; CK7 corrects the first pass's
    "130" — that is the `wire(...)` place's cut, :315): the tool refuses it, naming the `%.160s` lines it protects
    (`direction_check.hpp` :168, :224, :242, :272, :329, MECH-A4 worktree; V7). Re-read them at Part 5's tip.
34. *(re-check 2026-10-29, CK8)* **A robot id becomes a C++ namespace** (MECH-A5): `std`, `hal`, `robot` shadow the
    namespaces `src/` names unqualified; a keyword does not compile; `robot` would own `robot_two`'s team files. The
    collision rule refuses each (§1.2, §1.13; V8).
35. *(re-check 2026-10-29, CK1)* **New vocabulary inside a region after Part 5** cannot be hand-written (the gate) and
    cannot be written outside a region when it lives inside `controls(...)` or an initializer — rule 6 (i): the
    chunk extends the tool.

## 8. Found while reading and probing (not G3a's to fix unless named above)
- **Three host tests CRASH instead of failing by name when robot two's table changes** (P4, corrected
  at the re-check; the first crash ends the run, hiding 19 more cases).
  - `test/robot_two_driver_test.cpp` T8 (:629; the dereference is at ~:639-640,
    `holdFor("claw pivot 7")->forwardButton()`). T4 Part 1b rewrites this file, so it is the natural
    owner.
  - `test/teleop_claws_test.cpp` 1g (:479; `motorClawBootLine(..., *t.robot, 1)`) and 1h (:546;
    `t.robot->mechanismAt(...)->tryClaim()`). Each dereferences a mechanism that the rig's unplugged
    port leaves null, with no REQUIRE. This file retires with `teleop_claws.hpp` at T4 Part 3.

  None of the three crash on today's table. They matter the moment the robot file changes, which is
  G3a's whole purpose, so the coordinator names them in T4 1b's and Part 3's prompts. *[2026-09-29: 1b is
  folded into T4 Part 3, which owns all three now; Part 5 re-runs P4's kind of edit on the core's tree and
  confirms none crashes.]*
- **"claw pivot" is printed for every declared motor mechanism** (`calibrate_program.cpp` :1131;
  `boot_source.hpp` :102-106 says so and says a new kind "changes this with it"). A panel-added intake
  would be called a claw pivot on the brain. G3a warns; the words fix belongs to the next chunk that edits
  Calibrate's census log.
- **Robot two's typed table faces the 15/16 end; its record faces the camera end** (P8; the worksheet
  ~:522-529). Once T4 Part 3 retires the old "shulib Drive", re-stating robot two's file toward the camera
  end (LEFT 16–20, signs flipped) would end the boot's CROSS-CHECK warning — a data change with Station
  F/G consequences, for the team lead. *[2026-09-29: now D7, recommended (a). N1 measured it: flipped, 8 of
  the record's 8 ports agree by side and sign; as typed, none.]*
- **The chassis notes' last line** ("kChassis in the four ROBOT=tank builds ...") is a C++ remark that now
  travels in the robot file; the prose pass may drop it (the selection function says it).

**Found by the amendment (2026-09-29):**
- **The host tests' robot lists are typed by hand** (`build_selection_test.cpp` :35-36,
  `mechanism_table_test.cpp` :397): a robot added to `RobotId` and not to them is silently untested. Part 2
  fixes this for every robot with `kEveryRobotId`.
- **The pairwise robot-define guard** (`build_selection.hpp` :69-71) and the "needs ROBOT=tank" program
  guards (:76-84, `Makefile` :147-161) were written for a fixed robot set. Part 2 writes them in general
  form.
- **Calibrate says "robot two" at runtime** in a program any tank could build (:1044, :1068), and its
  cartridge refusal points at `chassis_table.hpp` (:1141). Part 2.
- **The programs will name robot two's driver class and auton function directly** (the T4, COMP-t and COMP-b
  briefs), and step 0 cannot see it. Part 4 (D8).
- **The gate's own self-test builds no generated robot** (N8: 190 cases, none generated), so without Part 2's
  case the generated path would be exercised only when a team first creates a robot.
- **The drivetrain record carries only the drive's ports and the IMU**, so the panel's wrong-kind warning
  (§4.3) can cover only those. The full kind check is the boot's (the MECH core's).

**Found by the re-check (2026-09-29):**
- **Calibrate's front question on the SCREEN cannot show the robot file's words** without a library change.
  WHICH END FIRST? is `include/shulib/calibration/calibrate_panel.hpp` :374, a fixed literal inside the
  panel's layout budget. G3a prints the words to the serial log only (§1.13). The next chunk that edits the
  calibrate panel could take a short front label from the program (a budgeted line). Named here for it, not
  G3a's.
- **The Hardware Assumptions register records held numbers in prose only, and the briefing's "next free"
  ignores them** (`tools/briefing_status.py` :111-117 takes the highest table row + 1). A held number is
  therefore safe only while every brief writer remembers it. §5 has the coordinator's steps for HA-181.
- **Two chunks' guide chapters were planned against one number.** COMP-t's chapter 16 landed first, and
  COMP-b holds 17. A chapter number, like an HA number, is held in prose only. G3a's is now "the next free at
  Part 1's tip" (§3).
- **The whole-run rule rests on membership, which a second copy of a line defeats** (#1). The rule was written
  for G3a's panel, and a generated starter is exactly the second copy. Part 4 binds each listing to its file.

**Found by the re-check (2026-10-29):**
- **The wiring's place in a generated robot** (RC8) — MECH's (`include/`), for MECH's rest or a small library
  chunk *(CK11 (e): not MECH-A6, whose scope is `src/programs/bench_r3a.*`)*.
- **The robot's footprint** (RC10) — in no file; Part R's section adds it.
- **VexBuilder's `sensors` is `{port, type}` only** (L6 RC32, RC9) — the note to VexBuilder grows.
- **MECH §6.2's "two regions" would generate THE FEEL** (RC5) — against D4 (a). *(amended 2026-10-29 evening: D4 (a) reversed — THE FEEL is generated, in its own region; moot)*
- **Robot one has no build for 2026-11-23** (RC18) — I1's, the coordinator's to raise.

**Found by the second pass (the checker's findings, 2026-10-29):**
- **The camera's and the wall sensors' mounts are measured from the front, and the front is the record's**
  (CK10). `CameraPlacement` (+X forward, `yawDeg` CCW from +X — `camera_config.hpp` :35-49, vis1-a1) and L6's
  `SideMount` (`facing` CCW from forward, L6 :680-681) are stated in the body frame, whose +X is the end that led
  Calibrate's push (2026-10-06). A re-Calibrate pushed from the other end mirrors every stated mount and swaps the
  left and right wall sensors, and neither VIS1's nor L6's brief mentions it. G3a's CALIBRATION page warns (§1.7);
  CAL1's flip warning on the brain could name the camera and the wall sensors too — CAL1's, VIS1-B's and L6-B's to
  weigh, not G3a's.
- **A future chunk that needs new vocabulary inside a generated region had no lawful way to land** (CK1) — the
  rebase rule's point 6; AF1 is the first such chunk.
- **A robot id is a C++ namespace from MECH-A5 on** (CK8, V8) — `std`, `hal`, `robot`, keywords and team-file
  prefixes break the build or the gate's ownership rule with messages that never name the robot file; G3a's
  collision rule now refuses them.

**Found by the fix round on the evening amendment (2026-10-30, the checker's C1–C16):**
- **`rotationRadius` is a stand-in the record could answer** (C12). `MotionConfig::rotationRadius` and the stall
  check's own `rotationRadius` are both 7.0 in (HA-17, HA-52) on every robot; on a tank each is half the track width,
  which Calibrate MEASURES and the record carries, and `Robot::effectiveTuning()` (`robot.hpp` :1052-1059) applies
  the record's wheels to the stall check but not this. A small library item (the coordinator's to raise), not G3a's —
  G3a now shows both read-only rather than letting a team type a measured number.
- **A default-constructed `SettleConfig`, `AxisGains` or `FeedforwardGains` is a robot that never settles or never
  corrects, and nothing refuses it** (C1). `SettledUtil` accepts zeros (`settled_util.hpp` :50-52; its own header
  says a default "effectively never settles"); `MotionConfig::validate()` does not descend into the settle groups.
  Any hand-written `RobotConfig` that designates a nested group in part has the same trap the generator had; none in
  the tree does today (the X-drive's `src/robots/xdrive_config.hpp` names no tuning at all — grep, 2026-10-30). A
  library-side refusal (for example `validate()` refusing a zero `maxErrorRate`) would be a behaviour change for the
  library's owner to weigh, not G3a's.
- **COMP-b's routine-stop line can reach 214 B against a 200 B budget** (NOTES 10-29 ~21:00, MECH-A5's note 5 — CUT,
  flagged, never overflowed). STEP1-A's result words touch the same lines (C9) and must not lengthen them.
