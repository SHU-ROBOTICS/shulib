# DIAG1 Part B — a drive side's median leaves out a member whose reading is not finite

> **THE TEAM LEAD'S ANSWERS (2026-09-26, ~23:10): every recommendation accepted** — "Accept all" on
> D0–D9 as written below. Builders start after tonight's claw and auton merges (his ruling of ~14:15:
> "briefs now, build after tonight"); each part still starts on its own go, at its turn in the order
> the brief gives. Committed by the coordinator with its plan entries.

> **Execution brief (draft).** Written 2026-09-26 from a read-only pass over `shulib-v2` at `a01e3a2`:
> `hal/motor_group.hpp`, the drive-encoder odometry and its trust gate, the audit header, the PROS motor
> adapter, the calibrator's three median call sites and its push classification, the coupled-side
> monitor, the host test plant's member faults, the MotorGroup, calibrator and DIAG1 tests, and the
> public docs that name the median. The DIAG1 research report's finding about the median was recovered
> from the session transcript (it is not in the tree). Re-checked the same evening (below). The
> coordinator commits this brief under `docs/internal/chunks/` before the run. **STATUS: WAITS FOR THE
> TEAM LEAD'S GO**, and D0–D9 below need his answers first. **The go this brief carries out:** the
> team lead approved the DIAG1 brief on 2026-09-26 with **"Yes, as recommended"**. The recommendation
> he approved read: "the median's non-finite handling as its OWN later step (Part B, excluding
> non-finite values like ABSENT members)" (`DIAG1-odometry-tick-names-its-check.md`, header).
> **One executor run. Executor model: `opus`. Do not commit.** Live log: a new `## Part B` section
> appended to `docs/internal/chunks/DIAG1-PROGRESS.md`, written FIRST and appended as you go. Reserved
> Hardware Assumptions number: **HA-165 only**. Build in an isolated worktree
> (`.claude/worktrees/diag1-b`, branch `diag1-b`) off the then-current `shulib-v2` HEAD. **Honest
> size:** about half a day to a day for the executor, plus verification.

## Re-check 2026-09-26 (evening) — amendments

A checker read the first draft against `a01e3a2` and the two in-flight worktrees and returned one
major and ten minor findings. Each was re-verified against the tree: every cited file and line
opened, and the numeric claims re-run against the real header in a probe under
`docs/internal/evidence/brief-probes-2026-09-26/` (`DIAG1-B-probe_recheck.cpp`; written in the coordinator's scratch folder and copied at this brief's commit). **All eleven are real, and all are fixed in the text
below.**

1. **(major) D5 missed `finishPushCapture`'s own "moved" count** (`drivetrain_calibrator.hpp`
   ~:1029-1042: a `±Inf` delta became a drive port with a sign, counted in `drivePortCount`).
   **Fixed at the root.** D5 is rewritten around ONE "moved on the push" predicate that both the
   classification and `pushSummary()` use. The spread and ~:1261 edits are dropped, because after
   the fix no test could reach them. The new D9 names the calibrator's other live comparisons as OUT.
2. **DIAG1-B 9's harness contradicted itself.** **Fixed.** N9 names one path: the scheduler
   through the Chassis, as DIAG1 F3 does. A pacer copied into the new file injects after each plant
   step and records the trace. The groups are the context's drive motors and `deps.motorGroups`
   (GroupRig's composition), over `DriveEncoderOdometry`. Only the NaN sub-run is RED evidence.
3. **The case names collided with Part A's `DIAG1 B1..B11`.** **Fixed.** Part B's cases are
   `DIAG1 N1..N10` (N for non-finite; no test in `test/` uses the prefix). The rename covers §1, §2,
   HA-165's Source and every mutation's "kills".
4. **Class B's "Today" cell was wrong for more than one infinite member.** The probe gives
   `{1, +Inf, +Inf}` → `+Inf`. **Fixed** in the table and in D3. D5's spread sentence went with the
   spread edit, and D5 now states today's spread correctly: it depends on the values, not the order.
5. **D7 did not name its readers.** **Fixed.** D7 names the over-temperature fault and the debug
   records' `wheelCurrent`: under D7, a `+Inf` member no longer raises `MOTOR_OVER_TEMP`. The
   changelog entry carries the thermal change.
6. **The merge notes missed T3 Part 2's edits to the register, and the index rule would have put
   HA-165 in the wrong place.** **Fixed** in landmine 9: HA-165's index row goes in numeric order,
   and D0 now says that no CODE file overlaps and that three documents merge.
7. **The mutation list had gaps.** **Fixed**, adapted to the amended D5. The proposed ~:1261
   mutation is moot, because nothing there is edited any more. M12 and M13 now bypass the shared
   predicate. M17, M18 and M19 are new. N5 spells out M7's kill sequence and D8's four kinds of
   value, and N3 carries `{denorm_min, denorm_min}`.
8. **The documentation list had gaps.** **Fixed.**
   - Guide 11 ~:214 joins the known hits.
   - The changelog gets a CHANGED bullet for `medianInPlace`'s weaker post-condition.
   - `HA-165` is required in the test file, and reconciliation direction 2 is checked for both files.
   - `motor_group.hpp`'s stale `hal/motor.hpp` pointers are fixed: ~:30, and ~:101, which this
     re-check found.
   - The prose review now covers `include/` and `src/` comments as well.
   - The calibrator doc comments that restate the push rule are listed.
9. **The N6 and N7 wording could mislead the executor.** **Fixed.** `L=` is the finite median's
   CHANGE times ipr, and N7 names the member at 100 as the one that reads NaN.
10. **The quoted ruling put the recommendation in the team lead's mouth.** **Fixed** in the header.
    D3 and D7 now say "the recommendation you approved".
11. **Buildability and staleness.** **Fixed.**
    - §1 adds `<numeric>` and `<limits>`.
    - §5 says "every build the gate lists links" (six today, seven once COMP-b lands).
    - D6 gives scope as its reason to defer, and the ownership rule second.

## Decisions for the team lead (a recommendation for each; nothing below decides them silently)

- **D0 — Timing.** Part B cannot change anything robot two does today: `ProsMotor` replaces a
  non-finite read with the last good one before any group or the calibrator sees it
  (`hal/pros/motor.hpp` ~:75-84, ~:233-255; register HA-08). It removes undefined behaviour and
  defines what the library does with a contract breach. It is not on the first autonomous's critical
  path.
  **Recommendation:** run it after tonight's claw and auton merges (T3 Part 2, G1 Part 2c final,
  COMP-b), as the coordinator's notes plan for the next wave.
  - No CODE file overlaps theirs (§4), so it can also run beside them in its own worktree if you
    prefer.
  - Three DOCUMENTS are shared: `docs/changelog.md`, `docs/hardware-assumptions.md` and guide 11.
    T3 Part 2 edits all three, and COMP-b and 2c also edit the register's status paragraph. Those
    merges are predictable (§6, landmine 9).
- **D1 — Where the exclusion lives.** Recommendation: **inside `hal::medianInPlace`**
  (`motor_group.hpp` ~:129-134), the one median in the tree. The research report proposed the same.
  - `MotorGroup::medianOf` (~:326-338) and the calibrator's three call sites
    (`drivetrain_calibrator.hpp` ~:1260, ~:1449, ~:1536) all inherit it. Under D5 the two push-time
    sites (~:1260, ~:1449) only ever see finite values, so the calibrator's live exposure is
    `sideTravel()` (~:1536), which reads the shafts during the spin.
  - Why: the calibrator measures each side's scale through the same statistic the group reports, so
    a slow member cancels exactly (calibrator item 4, ~:92-95; HA-130). Two medians that treated a
    NaN differently would break that guarantee exactly when a reading is bad.
  - It also removes the undefined behaviour from the calibrator's sites, not only from the group's.
  - Rejected: filtering in `medianOf` only. The calibrator would keep calling `std::sort` on a NaN.
- **D2 — What a read returns when no present member is finite.** Recommendation: **a quiet NaN**
  (`std::numeric_limits<double>::quiet_NaN()`, never one of the inputs).
  - The drive-encoder odometry's existing path then handles it unchanged: the tick is
    `NONFINITE+TRAVEL`, position freezes, and the baseline is not re-armed
    (`drive_encoder_odometry.hpp` ~:158-163, ~:177-195).
  - Rejected: passing a non-finite input through, e.g. `+Inf` when every member reads `+Inf`. The
    answer would then depend on which non-finite values arrived: today `{+Inf, -Inf}` gives a
    negative NaN.
  - Rejected: a precondition failure. That would stop a program mid-match over a sensor read.
- **D3 — `±Inf` is excluded as well as NaN.** The recommendation you approved says "non-finite".
  Please confirm, because this changes values that are defined today.
  - Today ONE `Inf` member of a group of 3 or more is an ordinary outlier: it sorts to an end and
    the median stays finite. `{5, Inf, 1, 2, 3}` gives 3 today, while the median of the four finite
    members is 2.5. Two infinities can already reach the middle: `{1, +Inf, +Inf}` gives `+Inf`.
  - **Recommendation: exclude.** `+Inf` is PROS's own failure sentinel (`PROS_ERR_F`, HA-08), which
    means "no reading", not a position. An included `Inf` still moves the median by half a
    neighbour's gap, and for groups of 1 or 2 it makes the whole side non-finite.
- **D4 — The even-count median can overflow on finite values.** `0.5 * (a + b)` gives `+Inf` for
  `a = b = DBL_MAX` (observed). Recommendation: **`std::midpoint(a, b)`** (C++20 `<numeric>`).
  - It is bit-identical to `0.5 * (a + b)` wherever that sum does not overflow. The draft's author
    checked 2,000,000 pairs up to `DBL_MAX/2`, subnormals included, and found 0 differences. The
    checker's 1.5M full-range pairs (every pair whose sum does not overflow) also found 0. It
    compiles under the ARM gate's flags with `-Werror` (checked twice).
  - After this change, "the median is non-finite only when no member is finite" holds without
    exceptions.
  - Rejected: `0.5*a + 0.5*b`, which differs at subnormals (`{denorm_min, denorm_min}` gives 0).
    Also rejected: keeping the overflow and documenting it as an exception.
- **D5 — The calibrator's "moved on the push" rule. It is needed if D1 is accepted.**
  - **What is there.** The same comparison is written out twice.
    - `finishPushCapture` (~:1029-1042) makes a candidate a DRIVE port when
      `std::abs(d) >= movedThreshold`, takes its sign from `d > 0`, and counts it into
      `report_.drivePortCount`.
    - `pushSummary()` (~:1438-1451) counts "moved" with the same comparison. Its count feeds the DONE
      gate (`pushReady()`, ~:804-810; `s.moved < 2`, ~:1045) and the live panel (`movedNow`).
    - Against a non-finite push delta:
      - **NaN:** the comparison is false. A NaN candidate is already "not a drive port" today, and
        the push record calls it `still`.
      - **`±Inf`:** the comparison is true. The candidate becomes a drive port with a sign read off
        the sentinel. It is counted in `drivePortCount` (the serial record's "push: N of M
        candidates moved", `src/calibrate_program.cpp` ~:791-793; the panel's pulse total, ~:1691
        and ~:1712) and in `pushSummary().moved`, and it is pulsed in SideAssign. Its push travel
        at the scale (~:1255) is `+Inf` whichever end went first, because it is sign × delta.
  - **Why D1 forces the question.** With D1 alone, `pushSummary()`'s median (~:1449) and the scale's
    median (~:1260) would leave the `±Inf` candidate out, while both counts kept it. The DONE gate's
    count and its median would then describe different members.
  - **Recommendation, in three parts:**
    - **(a) One predicate.** A private helper, "moved on the push" =
      `std::isfinite(d) && std::abs(d) >= movedThreshold`, is the ONLY test that
      `finishPushCapture`'s classification and `pushSummary()` use.
      - A non-finite push delta is then NOT a drive port, which is what a NaN already is.
      - `drivePortCount`, `pushSummary().moved`, `pushReady()`, `movedNow` and the sign capture all
        describe the same members by construction.
    - **(b) Named, not silent**, the way the calibrator already leaves out a slipping gear ("LEFT
      OUT with a warning and the run goes on", header ~:34-36):
      - The PUSH TABLE line keeps the port and its reading, and its word becomes `NO READING`
        instead of `still` (`formatPushRecord`, ~:1868-1872).
      - `appendWarning` gains a second sentence, shown on the Done screen
        (`calibrate_panel.hpp` ~:836-842): "port N gave no reading at DONE -- not counted as a
        drive port; if it drives the robot, check it and run again".
    - **(c) No edit at the scale.** With (a), every drive port's push travel is finite by
      construction. The push capture's spread (~:1251-1259), the scale's median (~:1260) and its
      `median < minPushRad` check (~:1261) then only ever see finite values.
      - The first draft's two edits there are dropped. No test could reach them, so no mutation
        could check them.
      - A comment at ~:1251 says why no guard is needed there.
  - **What today's code does instead,** so that the change is described honestly:
    - The spread over a side with a `+Inf` member reads `+Inf`, or NaN when every member of the
      side is `+Inf`. This depends on the values, not their order (probe).
    - A side whose every member read `+Inf` at DONE has a median of `+Inf`, which passes ~:1261
      (`+Inf < min` is false). Its scale becomes `pushDistance / +Inf = 0`. That is refused later,
      by `SpinWidthInvalid` (~:1321-1330) or by the description's `ipr > 0` rule (~:1363-1372;
      `drivetrain_description.hpp` ~:260-261), but never at the step that went wrong. After (a),
      this case cannot arise.
  - **The net under (a) for a real drive motor that gets left out.**
    - When a side-mate is pulsed, the gear train back-drives it, and the NonDriveMoved check
      (~:870-889) refuses the run naming its port, but only while its VELOCITY reads finite.
    - A member whose velocity is also held or non-finite is not caught there. The run finishes
      without it, with the warning from (b) naming it.
    - Today a NaN gets the same outcome with no warning. A `+Inf` held through the pulses is
      expected, by reading ~:1115 and ~:1124-1126, to be refused today as `PULSE_CONTRADICTION`
      (its `own` is `Inf - Inf`, a NaN). That is a wrong reason, reached by accident. It has not
      been observed; N10 logs both outcomes.
  - **Alternative: refuse the run at DONE, naming the port.** This is stricter: a calibration could
    never finish past an unread motor. It is done right only with all of the following:
    - a new `Refusal` code (`refusalName`, `calibrate_panel.hpp`'s words);
    - an "unread" count in `pushSummary()` that keeps `pushReady()` false, because the header's rule
      (~:800-803) is that the gate is stricter than the finish, never looser;
    - a status field, so that the panel can say which port is unread;
    - the tests.

    I do not recommend it: it costs about twice (b), for a case that HA-08 says never reaches the
    calibrator on a V5. **Choose it if you want a saved description never to miss a motor
    silently.**
  - **Rejected:** (a) without (b). It would leave the port out silently, which is today's NaN
    behaviour extended to `±Inf`.
  - **If D5 is rejected while D1 is accepted,** the two counts and the two medians disagree on a
    `±Inf` candidate. The executor logs that as found-while-reading.
  - **In every case, it cannot fire on robot two:** `ProsMotor` hands the calibrator the last good
    value (HA-08).
- **D6 — The coupled-side monitor and a non-finite member velocity.** This is a finding, not part of
  the median.
  - Observed with the draft author's probe against the real header: when a member (member 2 of
    five) reads `+Inf` rad/s, it becomes the side's "fastest". After 25 ticks its four HEALTHY mates
    are persistently flagged (mask `0x1b`) and the broken one is not. A NaN member is never flagged
    at all (`teleop/coupled_side_monitor.hpp` ~:108-141).
  - The calibrator's spin-fight check feeds the same monitor (`fightDuringSpin`, ~:1492-1526) and
    names the first persisted member in index order. A member whose sign-corrected velocity reads
    `+Inf` along the command is never flagged itself, so the refusal (`SPIN_FIGHT`) names a
    healthy mate. This is reasoned from the code, not probed.
  - **Recommendation: OUT of Part B. No change to `MotorGroup::evaluateDisagreement()`.**
    - The reason is scope. This changes the monitor's semantics, the fix belongs in the monitor
      (flag a non-finite member under its own reason, and never let it be the fastest), and it
      needs its own tests in both of the monitor's callers.
    - Passing `present = false` from the group instead would hide the broken member.
    - Secondary reason: `include/shulib/teleop/*` is T3 Part 2's while that is in flight, although
      its diff does not touch this file.
    - It is a follow-up after T3 Part 2 merges. It cannot fire on robot two through `ProsMotor`.
- **D7 — The group's other reads.**
  - What is there:
    - `current()` is the mean per present member, and one NaN member makes it NaN.
    - `temperature()` is the max. It is NaN only when the FIRST present member is NaN
      (`std::max` keeps its first argument against a NaN), so the result depends on member order.
      A `+Inf` member makes it `+Inf`.
  - The recommendation you approved names the median, but its comparison, "like ABSENT members",
    covers all four reads: an absent member is excluded from every one (`motor_group.hpp`
    ~:92-98).
  - **Recommendation: include both.** Take the mean and the max over the present, finite readings,
    and return NaN when none is finite. `totalCurrent()` becomes the sum over the same members.
    This is the same file and the same rule, at the cost of about two more test cases and two more
    mutations.
  - **Readers, and what changes for each:**
    - **The over-temperature warning.** `motion::tickHealthObservables` takes the max over the drive
      motors (`motion.hpp` ~:261-264) and passes it to `HealthMonitor` (`health_monitor.hpp`
      ~:228-233). Today a member reading `+Inf` °C (PROS's sentinel, reaching the group only past
      `ProsMotor`) raises `MOTOR_OVER_TEMP temp=infC`. Under D7 that member is left out and no
      warning is raised. It is counted by D8 if D8 is accepted. A NaN group temperature is already
      dropped there, because `std::max(0.0, NaN)` keeps 0.
    - **The debug records' `wheelCurrent`:** `chassis.hpp` ~:581, `move_to_pose.hpp` ~:236,
      `turn_to.hpp` ~:235, `drive_brake.hpp` ~:207, `motion_scheduler.hpp` ~:1189.
    - No motion outcome changes: the scheduler's default abort mask is ODO_STUCK only
      (`motion_scheduler.hpp` ~:233).
    - `src/` reads members directly (`src/drive_program.cpp` ~:460, `src/bench_r3a.cpp` ~:1656).
  - If you rule them out, they go to the log as found-while-reading.
- **D8 — Make the exclusion observable.** Today an ABSENT member is excluded AND reported
  (`absentMask()`, `presentCount()`). A member left out for being non-finite would be silent.
  - **Recommendation:** add a cumulative `int nonFiniteReads() const noexcept` to `MotorGroup`.
    - It counts one per member reading left out, by any read (position, velocity, and under D7
      current and temperature).
    - It mirrors `ProsMotor::faultedReads()` (`hal/pros/motor.hpp` ~:290), including the
      `mutable` counter in a const read.
    - The count uses the SAME `!std::isfinite` test as the exclusion. Ideally both come from one
      inline helper, so the two cannot drift. N5 checks it over NaN, negative NaN, `+Inf` and
      `-Inf`, because a counter written with `std::isnan` would pass a NaN-only test.
  - No fault is raised, because raising is the loop layer's policy (`motor_group.hpp` ~:61-70). No
    consumer is added in Part B.
  - It is additive, so it gets a changelog line and no version bump.
- **D9 — The calibrator's other comparisons against a live reading** (found while re-checking D5).
  - Three checks compare a live reading and would misread a non-finite one:
    - the rest check (`firstMoving`, ~:1418-1425) and NonDriveMoved (~:870-889) both use
      `std::abs(velocity) > floor`, so a NaN velocity reads as "still" and `±Inf` as "moving";
    - the pulse's `own` (~:1115) is NaN when its baseline or reading is non-finite, which gives
      `PULSE_CONTRADICTION`;
    - its `others` (~:1117-1121) ignores a NaN through `std::max`.
  - These are the calibrator's policy for a bad LIVE reading, not the median.
  - **Recommendation: OUT of Part B.** Log them as found-while-reading, and make them a follow-up
    together with D6's monitor fix. Each needs its own refusal design, and none can fire on robot
    two (HA-08). Under D5, the no-reading warning is what names such a port meanwhile.

## 0. Why

- **Two findings started this.** The DIAG1 research found that `hal::medianInPlace` is
  `std::sort(v, v + n)` with `operator<` (`motor_group.hpp` ~:129-134). A NaN breaks the strict weak
  ordering `std::sort` requires, which is **undefined behaviour**. The research recommended excluding
  non-finite values the way `medianOf` already excludes ABSENT members (~:326-338). DIAG1 Part A left
  this out on purpose (`DIAG1-odometry-tick-names-its-check.md` §4; `DIAG1-PROGRESS.md` :13, :276,
  :450).
- **What the undefined behaviour does today.** Observed in scratch probes outside the tree, on the
  host (x86-64, g++ 13.3.0 libstdc++, `-O2`), against the real header. The robot's
  `arm-none-eabi` 13.2.1 library was not probed. The executor re-runs these probes and logs its own
  numbers.
  - `{3, NaN, 1, 2}` → NaN.
  - `{NaN, 1, 2, 3, 4}` → 2, but the median of the finite values is 2.5.
  - `{1, 2, NaN, 3, 4}` → NaN, while `{1, 2, 3, 4, NaN}` → 3.
  - `{x, x, NaN, x, x}` → NaN (the shape of N9's RED run), while `{x, x, ±Inf, x, x}` → x.
  - For 21 members (the legal maximum, `kMaxSideMembers = 21`, which takes the introsort path) with
    1-3 NaN, out of 20,000 trials about 18,000 gave a wrong finite value and about 1,300 gave NaN.
    The numbers depend on the seed: two runs gave 18,070/1,300 and 18,083/1,270.
  - So the answer depends on which slot is bad, and for large groups it is usually a plausible,
    wrong number.
- **Why it matters for DIAG1.** The drive-encoder odometry reads each side through this median
  (`drive_encoder_odometry.hpp` ~:8-10, ~:158-161). Its trust gate is what raises `ODO_STUCK`, and
  ODO_STUCK is the scheduler's default abort fault. A median that turns one bad member into a NaN
  side, or into a wrong finite side, either ends a motion on a fault the group should have absorbed,
  or passes on a wrong position.
- **Why it cannot happen on robot two today.** The drive members are `ProsMotor`s, and `ProsMotor`
  screens `PROS_ERR_F` to the last good value (HA-08; `hal/motor.hpp` ~:60-67). Part B is contract
  hardening:
  - it removes the undefined behaviour;
  - it gives one bad reading a defined meaning ("no reading this time", like an ABSENT member);
  - it keeps DIAG1's line honest when a reading does get through, from a future adapter, a nested
    group, or a host test.
  - **It does not explain the 2026-09-25 `ODO_STUCK` episodes. Do not write that it does.**

## 1. Scope — IN

1. **`hal::medianInPlace(double* v, std::size_t n)`** (`motor_group.hpp` ~:117-134). This is D1; the
   signature and both preconditions (`v` non-null, `n >= 1`) are unchanged.
   - **Compact the finite values to the front, keeping their order.** Use a copy-forward loop, never
     `std::partition` or swap-with-last (see §6 landmine 2).
   - If none is finite, return `std::numeric_limits<double>::quiet_NaN()` (D2).
   - Otherwise `std::sort(v, v + k)` and take the middle. For an even `k`, use
     `std::midpoint(v[k/2 - 1], v[k/2])` (D4).
   - "Non-finite" means `!std::isfinite(x)`, which covers NaN of either sign and `±Inf` (D3).
   - **Add `#include <numeric>` (`std::midpoint`) and `#include <limits>` (`quiet_NaN`)** to
     `motor_group.hpp` (~:104-108). Today neither is there, and the one-TU ARM gate can hide a
     missing include behind a transitive one.
   - The doc comment's "SORTS `v` IN PLACE" becomes "REORDERS `v`; its contents afterwards are
     unspecified". This weakens the post-condition of a public function (`docs/api/motor_group.md`),
     so it gets its own CHANGED line in the changelog (§3). Every caller passes a scratch array and
     none reads it afterwards: confirm this at all four sites (`medianOf`, calibrator ~:1251,
     ~:1440, ~:1531) and log it.
2. **`MotorGroup`** (`motor_group.hpp`).
   - `position()` and `velocity()` (~:207-216) need no code change beyond D1. Their doc comments
     become "the MEDIAN of the PRESENT members' FINITE readings; NaN if none is finite".
   - If D7 is accepted: `current()`, `totalCurrent()` and `temperature()` (~:218-238, `sumCurrent`
     ~:340-348) use the present, finite readings, and return NaN when none is finite.
     `temperature()`'s seed from `first()` (~:227) must no longer let a NaN first member win.
   - If D8 is accepted: add `nonFiniteReads()`, counted with the exclusion's own test.
   - `presentCount()`, `absentMask()` and `memberCount()` are unchanged. The exclusion is per read,
     never a latch.
3. **The calibrator, only if D1 and D5 are accepted** (`drivetrain_calibrator.hpp`). Every edit is in
   this one file:
   - (a) One private "moved on the push" helper, `std::isfinite(d) && std::abs(d) >=
     movedThreshold`. `finishPushCapture`'s classification (~:1032-1040) and `pushSummary()`
     (~:1442-1447) both call it; neither keeps its own copy of the comparison.
   - (b) `formatPushRecord` (~:1868-1872) prints `NO READING` for a non-finite `pushDeltaRad`.
     `appendWarning` (~:1571-1581) gains the no-reading sentence, and `finishPushCapture` appends it
     for each such candidate.
   - (c) No code change at ~:1251-1261. Add a comment there saying why: the helper makes every drive
     port's push travel finite.
   - Comments: item 4 (~:92-95), the PushCapture phase text (~:53-59), `PortObservation::sign`
     (~:358-360), `warning()` (~:746-748), `pushReady()` (~:794-803), `pushSummary()`
     (~:1435-1437) and `formatPushRecord`'s doc.
   - No other calibrator logic moves (D9).
4. **Comments only, never code, in `drive_encoder_odometry.hpp`:**
   - ~:8-10: "the MEDIAN of a hal::MotorGroup" gains "over the members whose reading is finite".
   - ~:37-47: on a group side, a non-finite side reading now means NO present member was finite; one
     bad member is left out of the median.
   - ~:151-157: the non-finite-reading comment, with the same point.
   - Its `update()` body, the gate and the audit are not touched (§4).
5. **Tests:** a NEW file `test/median_nonfinite_test.cpp`, with cases named `DIAG1 N1: ...` to
   `DIAG1 N10: ...`. Part A's `DIAG1 A1..F3`, including its `B1..B11`, keep their names. The test
   CMake globs `*_test.cpp` (`test/CMakeLists.txt` :77), so no build edit is needed. The existing
   test files are left untouched, because §2's "verdict intact" pin needs them unchanged. Helpers from
   other test files are COPIED into the new file (they sit in anonymous namespaces); they are never
   moved or edited in place.

### Exactly what changes, tick by tick (per side, per read)

The trust gate (`drive_encoder_odometry.hpp` ~:177-192), the audit and its rendering
(`diag/odom_tick_audit.hpp`), and `HealthMonitor` are not edited. **DIAG1 Part A's verdict is
therefore a function of the same code. Only its INPUT changes, and only on these ticks:**

| Tick class | Today (`a01e3a2`) | After Part B | DIAG1's verdict and line on that tick |
|---|---|---|---|
| **A. Every present member finite** (every tick on a V5 through `ProsMotor`, HA-08) | median of all present | **bit-identical**, with one exception (D4): when the two middle values' sum overflows (`\|x\| > DBL_MAX/2`, reachable only in host tests), `+Inf` today becomes the finite midpoint | **unchanged, bit for bit.** Only in the D4 exception: `NONFINITE+TRAVEL` today becomes `TRAVEL` |
| **B. Some present members non-finite, at least one finite** | with a NaN: undefined (NaN, or a wrong finite value, depending on the slot); with `±Inf` only: an order statistic over all members, finite while the infinities stay out of the middle slot(s) (e.g. one `Inf` among 3 or more), non-finite once they reach it (e.g. two `+Inf` of 3, or one of 1-2) | the median of the finite members: **bit-identical to a group with those members ABSENT** | the side reads finite. The tick is implausible only if the side's change, which is the real travel plus the **membership step** (HA-165), passes `maxTickTravel` (`TRAVEL`), or if the heading trips `ROT`. A tick that read `NONFINITE+TRAVEL` today (position frozen) now integrates. The baseline re-arms to the subset median, and the tick on which the member returns carries the step back (the two steps cancel in side travel, up to rounding) |
| **C. Every present member non-finite** | all NaN → NaN; all `+Inf` → `+Inf`; mixed `±Inf` → `±Inf` or NaN (`{+Inf, -Inf}` → a negative NaN) | quiet NaN (D2) | **same verdict:** `NONFINITE+TRAVEL`, position frozen, baseline not re-armed, and the next finite tick integrates across the gap as today. The line's `L=`/`R=` reads `NaN` where it could read `+Inf`/`-Inf` today |

The group's median has other readers, affected only on class B and C reads:

- **`DriveEncoderOdometry`'s construction baseline** (~:140-141): a partly non-finite group now gives
  a finite baseline.
- **`OdoStallCheck`** (`motion/odo_stall_check.hpp` ~:206-209 and its baseline ~:258-262): gets a
  finite spin travel. On robot two this check can give no verdict anyway (HA-131).
- **The calibrator:** `sideTravel()` (~:1536) during the spin (D1). The two push-time sites see
  finite values only once D5's helper lands.
- **The group's `velocity()`**: no library reader found. The programs read members directly
  (`src/drive_program.cpp` ~:458, `src/bench_r3a.cpp` ~:1654).

**`PilonsOdometry` is not affected**: its tracking wheels are not groups.

## 2. Tests (adversarial) and mutations

Compare values bit for bit (a `sameBits`/`memcmp` helper, as DIAG1's tests do), never with `Approx`,
except where a test says otherwise. Put `HA-165` in the file's header comment, beside N7 (the
register's reconciliation direction 2 needs it, §5).

- **DIAG1 N1 — `medianInPlace`, every placement.**
  - Sizes n = 1..8 and 21. At every index, one at a time, put a NaN, a NaN with its sign bit set,
    `+Inf` and `-Inf`. For n ≤ 8, also every PAIR of indices, with every combination of those kinds.
  - The oracle is independent: copy the finite values in order, sort them with an insertion sort
    written in the test, and apply the same odd/midpoint rule.
  - Kills M1, M2, M3 and M10.
- **DIAG1 N2 — no finite value.** n = 1..21, random mixes of NaN, negative NaN, `+Inf` and `-Inf`
  (including `{+Inf, -Inf}`). The result's bits equal `quiet_NaN()`'s: a fresh NaN, never an
  input's sign or payload. Kills M4 and M5.
- **DIAG1 N3 — all finite is today's function, bit for bit.**
  - The oracle is `legacyMedianInPlace`: a verbatim copy of `a01e3a2`'s `medianInPlace`
    (`motor_group.hpp` :129-134), commented as the pin.
  - Compare over ≥ 200,000 random arrays of n = 1..21, including `±0.0`, subnormals,
    `denorm_min`, duplicates, integers, and `|x|` up to `DBL_MAX/2`.
  - Fixed cases, independent of the random draw:
    - `{denorm_min, denorm_min}` gives `denorm_min`, as the legacy function does. `0.5*a + 0.5*b`
      gives 0, so M9 cannot survive by luck of the draw.
    - The named exception: `{DBL_MAX, DBL_MAX}` gives `DBL_MAX` where the legacy function gives
      `+Inf`.
  - Kills M8 and M9, and any reordering of an all-finite input.
- **DIAG1 N4 — the finite values keep their order.**
  - The input `{+0.0, NaN, -0.0, +0.0}` must give exactly what the dense input `{+0.0, -0.0, +0.0}`
    gives. The probe shows that a stable copy leaves `-0.0` in the middle, while swap-with-last and
    `std::partition` both leave `+0.0`.
  - Kills M6. If M6 survives, strengthen this case until it does not; never delete it.
- **DIAG1 N5 — a non-finite member is an absent member.**
  - For N = 1..8 and 21 and every slot, a group with a non-finite member (NaN, negative NaN, `+Inf`,
    `-Inf`) gives the same `position()` and `velocity()`, bit for bit, as the same group with that
    slot `nullptr`, and as a dense group over the finite members. This is the shape of MotorGroup
    test 8's "equals a group built over exactly those four" (`motor_group_test.cpp` ~:458-476).
  - Also cover a hole and a non-finite member together.
  - `presentCount()` and `absentMask()` do not change.
  - The exclusion is not sticky: the next finite read includes the member.
  - Every present member non-finite gives NaN.
  - **M7's kill sequence, spelled out:** read the group once with the member at a DISTINCTIVE
    finite value, one that would move the median (e.g. 1000 among mates at 1). Then make the member
    non-finite and read again. The result must equal the `nullptr` group's, bit for bit. A last-good
    cache would return the old median instead. A fresh group read once with a NaN does NOT kill M7,
    and N9 cannot kill it either, because there the cached value equals the mates'.
  - If D7: the same for `current()`, `totalCurrent()` and `temperature()`, with a NaN in slot 0
    (today's order case) and a `+Inf` temperature (today's `MOTOR_OVER_TEMP` case).
  - If D8: `nonFiniteReads()` rises by exactly one per member reading left out, for position and
    velocity reads alike (and D7's reads if accepted), over ALL FOUR kinds: NaN, negative NaN,
    `+Inf`, `-Inf`.
  - Kills M7, M11, M14 and M18 (and M15 and M16 if D7).
- **DIAG1 N6 — through the odometry.** `DriveEncoderOdometry` over two groups, for N = 4 and N = 5.
  Robot two has five motors a side. Its saved record reportedly carries four present per side,
  "LEFT 4 RIGHT 4", in `build-order.md`'s R3d paragraph.
  - One member reads NaN, then `+Inf`, then `-Inf`, while the side moves. On each such tick:
    - the tick is NOT implausible;
    - `L=` is (this tick's finite median − the previous finite median) × ipr, exactly: `dL` is a
      CHANGE (`drive_encoder_odometry.hpp` ~:160), not the median itself;
    - position integrates; it does not freeze.
  - Every member of the left side non-finite gives `NONFINITE+TRAVEL`. The rendered golden is
    ` why=NONFINITE+TRAVEL ... L=NaNin R=...`. Position freezes, and the next all-finite tick
    integrates across the gap exactly.
  - Every member at `+Inf` renders `L=NaNin` (class C's named change).
- **DIAG1 N7 — the membership step, exact.**
  - Members `[100.0, 100.5, 101.0, 101.5, 80.0]` rad; the 80 plays port 18 (HA-130). The median is
    100.5. With member 2 (the 101.0) NaN for one tick, the median is `midpoint(100, 100.5) = 100.25`,
    so `dL = -0.25 × ipr`; the next tick gives `+0.25 × ipr`. The two ticks' travel sums to the
    full-set travel.
  - A two-member side `[0, 100]` at 1 in/rad: **member 1 (the one at 100) reads NaN**. The median
    then falls from 50 to 0, which gives `L=-50.000in` and `why=TRAVEL`. A NaN on member 0 would give
    `+50`, so the test names the slot. The golden shows that the step is visible when it is big. The
    next tick is `+50` and still implausible, so the implausible episode is ONE `ODO_STUCK` line (the
    rising edge), not two.
- **DIAG1 N8 — a sweep of at least 1000 scripted ticks**, in DIAG1 B8's shape
  (`odom_tick_audit_test.cpp` ~:556).
  - Two groups of five. Each tick draws a random member state: finite moves, a non-finite value at a
    random slot, several at once, or a whole side non-finite.
  - The oracle is computed from the script alone: per side, the finite median or NaN; then DIAG1's
    gate expressions, copied, applied to those readings.
  - Every tick, assert the side readings, the verdict, and every audit flag and value.
  - Log how often each category was exercised.
- **DIAG1 N9 — end to end on the host test plant, through the scheduler.**
  - **The rig** (a new struct in the new file) is robot two's shape:
    - a tank `SimHarness`;
    - five coupled members a wheel, wired as `motor_group_plant_test.cpp`'s `CoupledMembers`
      (~:57-96, copied);
    - two `MotorGroup`s, which are the `RobotContext`'s `driveMotors` AND `deps.motorGroups`, as in
      `GroupRig` (~:100-143, copied), but over **`DriveEncoderOdometry`**, not `GroupRig`'s
      `PilonsOdometry`. Take the geometry from DIAG1's `DriveEncoderRig`
      (`odom_tick_audit_test.cpp` ~:1191-1221): `DriveGeometry::fromDiameter(plantConfig()
      .plant.driveWheelDiameter, 1.0)`, 12 in track;
    - `Localizer`, `FaultLatch`, `HealthMonitor` and a `Chassis`.
  - **The runner:** a pacer copied from `motion_test_rig.hpp`'s `PlantPacer` (~:148-170) that, after
    each `plant().step()`, does two things:
    - It records the trace: truth sample, both primaries' commanded volts, and the left group's
      `position()`. Compare with the checks of `requireIdentical` (`motor_group_plant_test.cpp`
      ~:206-218, copied).
    - It runs the injection hook: at pace P0 it sets `plant.setMemberFault(0, 2,
      MemberFault::Frozen)` and writes that `FakeMotor`'s position to the bad value. At P0 + 30 it
      restores `MemberFault::None`, and the next plant step writes the member again.
      `drive_plant.hpp` ~:424-433 (`case Frozen: return;`) is why no change to the host test plant
      is needed.
  - **The motion:** a `MoveToPose` through the scheduler (`sched.async(m);
    sched.waitUntilSettled()`), exactly as DIAG1 F3 runs it, under the default abort mask
    (ODO_STUCK aborts). Choose P0 mid-motion.
  - **The baseline:** the same rig with no injection.
  - Three sub-runs, each on a fresh rig: NaN, `+Inf`, `-Inf`. Coupled members read identical values
    in the plant, so each trace must be **bit-identical to the baseline** (truth, volts, group
    position, exit, tick count), with zero `ODO_STUCK` lines.
  - `Frozen` also holds the member's velocity. If the monitor raises `MOTOR_GROUP_DISAGREE`, log it:
    it does not change the motion, and D6 owns it.
  - **Run this case on the UNCHANGED tree first and log the result.**
    - **The NaN sub-run is the RED evidence.** `{x, x, NaN, x, x}` gives NaN, so the tick reads
      `NONFINITE`, which raises ODO_STUCK, which aborts: expect `Cancelled` with
      `abortFault == OdoStuck`, as in F3.
    - **The `±Inf` sub-runs are GREEN on the unchanged tree**: one infinity among five sorts to an
      end. They are regression pins, not evidence that the case sees the defect. Say so in the log.
- **DIAG1 N10 — the calibrator, only if D1 and D5.** Use `calibration_test_rig.hpp` (robot two's
  shape: five coupled members a wheel plus two idle candidates, ports 11-20, 5 and 6). Inject
  through `RunOptions::hook` with `MemberFault::Frozen` and `FakeMotor::setPosition`. Freeze the
  member on the first tick that `rig.cal.pushReady()` is true. The hook runs before the scripted
  DONE, and every velocity is then under the rest floor, so the held velocity cannot block a rest
  window later. Freezing earlier, while the robot is still slowing, can hold a velocity above the
  floor and stall the run.
  - **(a) Held.** Port 13 (wheel 0, member 2) reads `+Inf` from just before DONE to the end of the
    run.
    - While the push is watched, `status().movedNow` leaves it out (9).
    - At DONE: its `sign` is 0, and `drivePortCount` is 9, equal to the live count.
    - Its PUSH TABLE line names port 13 with `NO READING sign +0`. Assert the port and the words;
      the number's spelling is the C library's.
    - The run ends `Done` with LEFT = 11, 12, 14 and 15, and `warning()` names port 13.
    - The LEFT scale (`leftMedianRad`, `leftInchesPerRadian`) equals a clean run's, bit for bit
      (the members read identical values).
    - Repeat with `-Inf`, and with NaN. On the unchanged tree, log what each gives: the NaN is
      already "not moved" but silent; a held `±Inf` is expected to be refused
      `PULSE_CONTRADICTION` at port 13's own pulse.
  - **(b) The net.** As (a), but restore `MemberFault::None` after the FRONT answer. The member then
    follows its train, so the first LEFT pulse back-drives it, and the run is refused
    `NON_DRIVE_MOVED` with `refusalPort() == 13`.
  - **(c) `sideTravel()`, which is D1 at ~:1536.** One LEFT member's position reads NaN on the tick
    a spin direction finishes. That direction's `SpinRecord` `L`/`R` equal the clean run's, bit for
    bit. The executor documents how the hook finds that tick.
  - The whole existing calibrator suite passes unchanged.
  - Kills M12, M13, M17 and M19.
- **The "verdict intact" pin.**
  - The whole pre-existing suite passes unchanged. That includes all of `test/odom_tick_audit_test.cpp`
    (Part A's `DIAG1 A1-F3`, with ITS `DIAG1 B9` above all), MotorGroup 7's bit-identity sweeps and
    every calibrator case.
  - Record case and assertion counts before and after, and diff the per-test assertion counts with
    the xml reporter, as DIAG1's 09:49 entry did. Explain every existing test whose count moved.

**Mutations.** Run each on a fresh binary (§5 recipe), observe it RED, restore it, and log it. A
mutation that was not run does not count.

- M1: the filter removed (`k = n`, sort everything).
- M2: only NaN excluded (`std::isnan`).
- M3: only `±Inf` excluded (`std::isinf`).
- M4: `k == 0` returns `0.0`.
- M5: `k == 0` returns `v[0]`.
- M6: swap-with-last compaction.
- M7: a last-good cache in the group (a non-finite member read is replaced by that member's last
  finite read). Killed by N5's spelled-out sequence.
- M8: the midpoint reverted to `0.5 * (a + b)`.
- M9: the midpoint as `0.5*a + 0.5*b`. Killed by N3's fixed `{denorm_min, denorm_min}`.
- M10: the exclusion moved into `MotorGroup::medianOf` only.
- M11: the exclusion made sticky.
- M12 (D5): `pushSummary()` bypasses the shared helper and uses the old `d >= movedThreshold`.
- M13 (D5): `finishPushCapture`'s classification bypasses the helper and uses the old
  `std::abs(d) >= movedThreshold`.
- M14 (D8): `nonFiniteReads()` misses velocity reads.
- M15 and M16 (if D7): M15, `temperature()` seeded from a NaN first member again; M16, `current()`
  dividing by `presentCount()` while summing only the finite readings.
- M17 (D5): the push record's word reverted to `still` for a non-finite delta.
- M18 (D8): the counter counts `std::isnan` only. N5's `±Inf` reads kill it.
- M19 (D5): the no-reading warning not appended.

## 3. Documentation (same change)

- **`include/shulib/hal/motor_group.hpp`:**
  - The "Read path: MEDIAN, not mean" section (~:26-37) gains a paragraph, "A READING THAT IS NOT A
    NUMBER", that covers:
    - the rule and why it matches the absent member;
    - that it is per read, never a latch;
    - NaN when none is finite;
    - why `±Inf` too (HA-08);
    - why not a group-side last-good cache (the adapter already holds last-good; a second one would
      be the frozen-member drag the median exists to survive);
    - why not the research's majority rule (NaN if half or more are non-finite). The recommendation
      that was approved chose exclusion;
    - the membership step and its bound: for one member left out, at most half the larger gap
      beside the median in sorted order; for a two-member side, half the gap between the two;
      **A4 register HA-165**;
    - the undefined behaviour it removes, with the executor's own probe numbers.
  - The same section's pointer to ProsMotor's screen, "hal/motor.hpp:47-54" (~:30), is stale: the
    text is at `hal/motor.hpp` ~:60-67 now. The ownership pointer "hal/motor.hpp:57-65" (~:101) is
    stale too; it is now ~:70-78. Fix both. Cite the IMotor doc by name as well as by line, so the
    pointer survives the next edit.
  - "WHAT CHANGES, AND WHAT DOES NOT" (~:92-98): the new exclusion leaves the slot and present
    counts alone.
  - The `medianInPlace` doc (~:117-128), the class doc (~:136-143), the `position()`/`velocity()`
    docs (~:207, ~:212), `medianOf`'s comment (~:334-337); plus D7's and D8's members if accepted.
- **`include/shulib/localization/drive_encoder_odometry.hpp`:** comments only (§1 item 4).
- **`include/shulib/calibration/drivetrain_calibrator.hpp`:** the comments listed in §1 item 3 (D1,
  D5).
- **`docs/guide/11-reading-the-diagnostics.md`:** prose only, no new example line, so the chapter's
  "pinned byte-for-byte" claim is unaffected.
  - The `NONFINITE` bullet (~:247-251): on a side driven by a motor group, a `NaN` for `L=`/`R=`
    means no member of that side gave a finite reading; one bad member is left out of the median.
  - The `L=`/`R=` bullet (~:255-261).
  - The ODO_STUCK table row (~:214): its "a reading that came back NaN/Inf (`NONFINITE`)" now needs
    EVERY member of a group side to be non-finite. Judge it and fix it if it reads otherwise.
- **`docs/hardware-assumptions.md`:**
  - A **HA-165** entry, in Group R4 directly after HA-160 (~:2013-2046). Model it on HA-160; the
    substance is below.
  - Its index row in numeric order: after HA-161's (~:263) in this tree, and at merge after the
    highest number below 165 present (landmine 9).
  - The status paragraph (~:39-80): one sentence, "DIAG1 Part B added ONE, HA-165, invented", and
    the count as it is true for THIS tree.
  - HA-08's blast radius (~:404-407): one sentence saying that a motor-group side now also leaves a
    non-finite member out of its median, and that the calibrator no longer makes a non-finite push
    reading a drive port (if D5). Mark it AMENDED in its index row (~:115), as DIAG1 did for HA-123.
  - Cite evidence by date, never by a `docs/internal` path.
  - **HA-165, what the entry must carry:**
    - *Claim:* the present members of one drive side stay close enough in CUMULATIVE position,
      through a match's driving and slip, that leaving one out of the side's median moves it by
      far less than `maxTickTravel` (36 in) divided by the side's inches per radian.
    - *Source:* `motor_group.hpp` (the label); `test/median_nonfinite_test.cpp` DIAG1 N7 (the exact
      step, its return, and a step past the bound reported as `TRAVEL`). Both files carry the
      string `HA-165`.
    - *Confidence:* **invented**. The members' spread over a match has never been recorded. The
      evidence that it can grow is HA-130 (port 18 about 20 % short per push, so its offset grows
      with the distance driven). It is exercised only when a member read is non-finite, which
      HA-08's screen says never reaches a group on a V5.
    - *Settle (R4):* after a match-length drive, read every member's cumulative position and compute
      the largest neighbour gap times inches per radian, for the saved record's member set.
    - *Blast radius:* one tick carries the step, and the return tick carries it back. They cancel in
      the side's travel, and in the pose only up to the heading turned between them. Under 36 in the
      pose steps silently; over it, `why=TRAVEL` → `ODO_STUCK` → the active motion ends (HA-123).
      *Contained:* HA-08 must fail first.
- **`docs/changelog.md`:** a new top section, "Changed since 2.2 (DIAG1 Part B)".
  - CHANGED: the behaviour for non-finite member readings, in position and velocity (plus current and
    temperature under D7).
  - CHANGED: `medianInPlace`'s post-condition, from "sorts `v`" to "reorders `v`, contents
    unspecified", and its non-finite rule.
  - CHANGED (D5): the calibrator no longer makes a candidate whose push reading is not finite into a
    drive port. It names such a candidate in the PUSH TABLE (`NO READING`) and in the Done screen's
    warning.
  - CHANGED (D7): a member reading `+Inf` °C no longer raises `MOTOR_OVER_TEMP`.
  - ADDED: `nonFiniteReads()` (D8).
  - No version bump: `MotorGroup`, `medianInPlace` and the calibrator are outside the Freeze
    Register. F4 freezes `IMotor`'s signatures, which are untouched.
  - Host test plant only; nothing has run on a robot.
- **Prose review, logged before it is claimed** (pace rule 4).
  - Grep `docs/` and `README.md` (generated `docs/api/` and `docs/internal/` excluded), AND the
    comments in `include/` and `src/`, for: `median`, `medianInPlace`, `MotorGroup`,
    `motor_group`, `non-finite`, `NaN`, `NONFINITE`, `PROS_ERR_F`, `last good`/`last-good`,
    `ABSENT`, `faultedReads`, `pushSummary`, `spread`, `drivePortCount`, `movedThreshold`.
  - Read every hit. Edit only the files in this chunk's scope. For every other hit, log it as
    "still true" or as a merge note.
  - Known hits to judge:
    - `docs/faq.md` ~:24-36 (held values; probably still true);
    - guide 11 ~:214 and ~:241-262;
    - `hardware-assumptions.md` ~:1113-1125 (HA-130) and ~:1983-1994 (HA-123);
    - `roadmap.md` ~:1427-1437 (a dated delivery record: read, do not edit);
    - `changelog.md` ~:478;
    - `robot/robot.hpp` ~:974-976 ("a frozen member is already excluded by the group's median":
      still true);
    - `sim/drive_plant.hpp` ~:397-400 (the Frozen signature "the group's median must ignore":
      still true);
    - `src/teleop_program.cpp` ~:765-771 (§4).
- **`docs/api/` regenerated** (`api_doc_tool.py generate`) after the last header edit. **The briefing
  regenerated LAST** (`briefing_status.py generate`), then `check`.
- **Log and record:** `DIAG1-PROGRESS.md`'s `## Part B` section, with its own header block (chunk,
  brief, tree at start, executor model, go, scope, and "nothing has run on a robot").
  **`DIAG1-COMPLETED.md` is owed by DIAG1 as a whole, and Part B closes DIAG1:** draft it to cover
  both parts, with evidence from both logs. The coordinator edits it at merge.
- **Not touched by the executor, and why (for the commit body):**
  - `docs/roadmap.md`: the coordinator's, and G1 Part 2c final has it modified.
  - `docs/internal/build-order.md` and `HANDOFF-remaining-work.md`: the coordinator writes the status
    lines at merge, as at DIAG1 Part A's merge.
  - `docs/guide/06` and `12`: only if the prose review finds them stale.
  - `include/shulib/calibration/calibrate_panel.hpp`: it already shows `warning()` on the Done
    screen, so no edit is needed.
  - `src/`: §4.

## 4. OUT

- **The trust gate and DIAG1 Part A's verdict:** the `update()` body of `drive_encoder_odometry.hpp`,
  `pilons_odometry.hpp`, `diag/odom_tick_audit.hpp`, `diag/health_monitor.hpp` and
  `motion/motion.hpp`. Only comments change in `drive_encoder_odometry.hpp`.
- **Work in flight elsewhere:**
  - `include/shulib/teleop/*`, including the coupled-side monitor (D6): T3 Part 2's.
  - `src/teleop_program.*`, `src/boot_source.*`, `src/teleop_claws.hpp` and the comments in
    `src/mechanism_table.hpp`: T3 Part 2's. `include/shulib/calibration/boot_check.hpp`: T3 Part 2's.
  - `src/main.cpp`, `src/build_selection.hpp` (the one chain that chooses program and robot), the
    robot selection in `src/chassis_table.hpp` / `src/mechanism_table.hpp`, `tools/src_build_gate.py`,
    the `Makefile` and CI: G1 Part 2c final's.
  - The competition program in `src/`, `main.cpp`'s chain, and the worksheet's Station K: COMP-b's.
  - **No `src/` file is edited.**
    - `src/teleop_program.cpp` ~:765-771 logs "median reads are over the PRESENT members". After
      Part B that is incomplete but not false. Leave it (merge note).
    - `src/calibrate_program.cpp` ~:855-866 prints `warning()` only beside a LEFT OUT (decoupled)
      list. On a run with no decoupled port, the no-reading warning reaches the Done screen and the
      PUSH TABLE line, but not that summary line. Log it as a follow-up; the PUSH TABLE already
      names the port.
- **Not this chunk:**
  - `ProsMotor` (`hal/pros/motor.hpp`): its screen is the robot's line of defence (HA-08), and it
    stays.
  - The host test plant (`sim/`): no change is needed (N9's `Frozen` route).
  - `MotorGroup::evaluateDisagreement()` and the monitor (D6).
  - The calibrator's other comparisons against a live reading (D9).
  - Runtime member-absence inside the library graph: a later chunk, per `motor_group.hpp` ~:89-90.
  - A new `FaultCode` or a new `Refusal` code, or a raise for an excluded member. D8 is an
    observable only, and D5's recommendation adds no code.
  - Any hardware claim.
- **Found while re-checking, for the log (not fixed here):** `finishPushCapture` reads every shaft,
  and then `pushSummary()` reads every shaft again in the same tick (~:1032 and ~:1443). On a
  `FakeMotor` the two passes agree. On a robot they can differ only by what moves between two reads,
  so the shared helper makes the two counts one PREDICATE, not one READ.

## 5. Checks

- **Baseline:** record the suite's case, assertion and skip counts on the untouched worktree first.
  Record the UNCHANGED-tree probe numbers (§0), the RED NaN run and the GREEN `±Inf` runs of N9, and
  N10's unchanged-tree outcomes (§2).
- **After the change:** the suite is green. Give the counts, and explain the difference as the new
  file's cases plus any explained per-test movement.
- **Every mutation RED on a fresh binary.** The recipe (it avoids the stale-binary trap):
  - edit, then `touch test/*.cpp` (`motor_group.hpp` and `drivetrain_calibrator.hpp` are included
    widely);
  - `cmake --build build/test --target shulib_tests/fast -j"$(nproc)"`, and **require `Linking CXX
    executable shulib_tests`** in the output;
  - run the whole suite and list the failing cases;
  - restore, `cmp` against the saved copy, and relink green.
- `python3 tools/src_build_gate.py self-test` and `check`: **every build the gate lists links** (six
  at `a01e3a2`; seven once COMP-b has merged), with 0 disallowed warnings.
- The ARM header gate: the RESUMING command, `-Werror ... -mfloat-abi=softfp`.
- The CI greps: PROS-free and sim-free.
- `api_doc_tool.py` self-test, `check-coverage`, `check-fresh`, `check-examples` and
  `check-removability`.
- `doc_staleness_audit.py`, and its self-test.
- Register reconciliation:
  - direction 1 prints nothing;
  - direction 2 finds `HA-165` in BOTH files its Source names, `motor_group.hpp` and
    `test/median_nonfinite_test.cpp`.
- New string literals are ASCII and carry no `%` (the calibrator's screen rule, header ~:122-130).
- The briefing is regenerated LAST, and `check` passes.
- **Nothing committed.** The report lists deviations, found-while-reading items and the merge notes.

## 6. Landmines

1. **Never pin the undefined behaviour.** Today's `std::sort`-with-NaN results are logged as observed
   on THIS host's libstdc++, never asserted in the suite. The legacy oracle in N3 is fed finite values
   only.
2. **Keep the finite values in order.** `std::partition` happens to leave an all-finite array in place
   but reorders a mixed one. `std::sort` is not stable, and for n ≤ 16 libstdc++'s insertion sort
   keeps equal values in input order, so the sign of a zero median depends on the order. N4 is what
   tells.
3. **`std::midpoint`, not a hand-rolled half-sum.** It is bit-identical only where `0.5 * (a + b)`
   does not overflow; `0.5*a + 0.5*b` and `a + 0.5*(b - a)` are not. N3 is the proof, so do not trim
   its subnormal and large-magnitude draws, or its fixed pairs.
4. **The gate must not move.** Do not "tidy" `drive_encoder_odometry.hpp`'s `update()`. Its
   `if (std::isfinite(pL)) baseLeft_ = pL;` (~:162-163) is what gives class C its catch-up and class
   B its re-armed baseline.
5. **The exclusion is per read.** A member excluded on one read is back on the next finite one. No
   group state other than D8's counter may remember it.
6. **`MemberFault::Frozen` holds velocity as well as position.**
   - N9 asserts the motion trace and zero `ODO_STUCK`, not the absence of `MOTOR_GROUP_DISAGREE`
     (D6).
   - N10 freezes the member at rest, so the held velocity does not block the rest windows or trip
     NonDriveMoved by itself.
7. **One "moved" predicate in the calibrator (D5).** Neither `finishPushCapture` nor `pushSummary()`
   may keep its own comparison. M12 and M13 are exactly those two regressions.
8. **Case names.** Part B's cases are `DIAG1 N1..N10`. `DIAG1 B9` is Part A's existing case; never
   reuse a Part A letter.
9. **HA-165 only.** Do not renumber or amend other entries beyond HA-08's one sentence.
10. **No robot claims.** Nothing here has run on a robot, and on robot two the exclusion is expected
    never to fire (HA-08). Say so in every document the change touches.
11. **Merge notes for the coordinator:**
    - `docs/changelog.md`: T3 Part 2 inserts a section at the top (~:19) and edits COMP-a's section
      (~:27, ~:41). COMP-b adds its own. All go at the top, newest first.
    - `docs/hardware-assumptions.md`:
      - T3 Part 2 rewrites the status paragraph (~:68-71, "T3 Part 2 ... added no entry") and edits
        HA-153, HA-154, HA-156 and HA-159.
      - COMP-b (HA-162) and 2c (HA-163, HA-164) may land first.
      - The status count becomes the combined one.
      - HA-165's index row goes **in numeric order: after the highest number below 165 present, and
        before any above it**. The coordinator's notes reserve 166-174 (SAFE-SAVE, L9-rest, H1, L7)
        in the same wave, so "after the highest present" would be wrong.
    - `docs/guide/11-reading-the-diagnostics.md`: T3 Part 2 inserts six lines after ~:196, which
      shifts this chunk's hits (~:214, ~:247-261) down by six. The edits are apart, so this is
      expected to merge clean. Check it.
    - `docs/api/*` and `docs/internal/PROJECT-BRIEFING.md`: regenerate after the merge.
    - `src/teleop_program.cpp` ~:767's log line, and `src/calibrate_program.cpp` ~:855-866's summary
      line (§4): src follow-ups, for whichever of T3 Part 2 or this chunk merges second, or for the
      next chunk that owns `src/calibrate_program.cpp`.
