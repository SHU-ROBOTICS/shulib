#!/usr/bin/env bash
# F1 mutation harness — break it, rebuild, run, OBSERVE the red, restore.
#
# A mutation that stays GREEN is a hole in the suite and the most valuable thing this
# script can find. The runner is GATED ON BUILD SUCCESS: a mutation that fails to compile
# is reported as BUILD-FAIL, never as "red", because a compile error proves nothing about
# the tests (the C4 lesson: a non-compiling mutation read off a stale binary looks green).
#
# Usage: docs/internal/verify/verify-f1.sh          (from the repo root)
set -uo pipefail
cd "$(dirname "$0")/../../.." || exit 1
BUILD=build/test
BIN=$BUILD/shulib_tests

pass=0; holes=0; buildfail=0; skipped=0

# NEVER use `git checkout` to undo a mutation: these files carry UNCOMMITTED work, and a
# checkout discards the chunk, not the mutation. Restore is always from the byte copy taken
# immediately before the edit. (E2's harness cost an hour by getting that wrong once.)
#
# Do not pipe this script into `head`: SIGPIPE kills it mid-mutation and leaves a header
# broken on disk. It prints a bounded amount; redirect to a file if you need to page it.
trap 'echo "INTERRUPTED — restoring $CURRENT"; [ -n "$CURRENT" ] && cp /tmp/f1-mut-backup "$CURRENT"; exit 130' INT TERM PIPE
CURRENT=""

# mutate NAME FILE FROM TO [FROM2 TO2]
mutate() {
  local name="$1" file="$2" from="$3" to="$4" from2="${5:-}" to2="${6:-}"
  printf '\n=== MUTATION: %s\n    %s\n' "$name" "$file"
  cp "$file" /tmp/f1-mut-backup
  CURRENT="$file"
  if ! grep -qF -- "$from" "$file"; then
    echo "    *** SKIP — PATTERN NOT FOUND. This mutation did NOT run. ***"
    skipped=$((skipped+1)); CURRENT=""; return
  fi
  if [ -n "$from2" ] && ! grep -qF -- "$from2" "$file"; then
    echo "    *** SKIP — SECOND PATTERN NOT FOUND. This mutation did NOT run. ***"
    skipped=$((skipped+1)); CURRENT=""; return
  fi
  python3 - "$file" "$from" "$to" "$from2" "$to2" <<'PY'
import sys
p, a, b, a2, b2 = sys.argv[1:6]
s = open(p).read()
s = s.replace(a, b, 1)
if a2:
    s = s.replace(a2, b2, 1)
open(p, 'w').write(s)
PY
  # THE EDIT MUST HAVE CHANGED THE FILE. `grep -F` with a MULTI-LINE pattern matches if ANY
  # single line matches, so the pre-flight check above can pass while python's exact
  # whole-string replace finds nothing — and then the mutation is reported GREEN having
  # never run (it happened twice during E3). A byte-compare makes that impossible.
  if cmp -s "$file" /tmp/f1-mut-backup; then
    echo "    *** SKIP — THE EDIT CHANGED NOTHING. This mutation did NOT run. ***"
    skipped=$((skipped+1)); CURRENT=""; return
  fi
  if ! cmake --build "$BUILD" -j"$(nproc)" >/tmp/f1-mut-build.log 2>&1; then
    echo "    BUILD-FAIL — mutation did not compile; proves nothing"
    buildfail=$((buildfail+1))
    cp /tmp/f1-mut-backup "$file"; CURRENT=""
    cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
    return
  fi
  local out
  out=$("$BIN" 2>&1 | tail -3)
  if echo "$out" | grep -q "FAILURE"; then
    echo "    RED (good) — $(echo "$out" | grep 'test cases' | tr -s ' ')"
    pass=$((pass+1))
  else
    echo "    *** GREEN — THIS IS A HOLE IN THE SUITE ***"
    holes=$((holes+1))
  fi
  cp /tmp/f1-mut-backup "$file"
  CURRENT=""
  cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
}

OP=include/shulib/manipulation/mechanism_op.hpp
SD=include/shulib/manipulation/stall_detector.hpp
MECH=include/shulib/hal/mechanism.hpp
HOSTILE=include/shulib/sim/hostile/mechanism_hostility.hpp
RTN=include/shulib/chassis/routine.hpp

# ── The eight named in the brief/prompt, first ──────────────────────────────────────

# 1. Disarm the operation watchdog. Killer: the never-confirm tick-50 case and
#    the adversarial-clock cases (op would run forever = maxTicks Running).
mutate "1 watchdog disarmed (RunUntilConfirmed never times out)" "$OP" \
"        if (watchdog_.expired()) {" \
"        if (false && watchdog_.expired()) {"

# 2. cancel() skips the safe state. Killer: cancel-contract clause 1 (device
#    still energized after cancel). python replaces the FIRST occurrence only,
#    which is RunUntilConfirmed::cancel (ActuateAndConfirm's copy is later in
#    the file and untouched).
mutate "2 cancel() skips applySafeState" "$OP" \
'        mech_->applySafeState();  // always — "make it safe NOW", idempotent' \
'        // (mutation: safe state skipped)'

# 3. cancel() overwrites a completed verdict. Killer: cancel-contract clause 2
#    (Succeeded rewritten to Cancelled).
mutate "3 cancel() overwrites a completed verdict" "$OP" \
"        mech_->applySafeState();  // always — \"make it safe NOW\", idempotent
        if (!finished_) {
            releaseClaim();
            outcome_ = MechanismOutcome::Cancelled;
            finished_ = true;
        }" \
"        mech_->applySafeState();  // always — \"make it safe NOW\", idempotent
        if (!finished_) {
            releaseClaim();
            finished_ = true;
        }
        outcome_ = MechanismOutcome::Cancelled;"

# 4a/4b. SWAP THE TWO SAFE STATES — both directions: everyone forced to Coast
#    (drops the loaded lift) and everyone forced to Hold (cooks the jammed
#    intake). The declared-per-mechanism asymmetry must catch each.
mutate "4a safe state ignores declaration — everyone COASTS (lift drops)" "$MECH" \
"            m->setBrakeMode(safe_);" \
"            m->setBrakeMode(BrakeMode::Coast);"
mutate "4b safe state ignores declaration — everyone HOLDS (intake cooks)" "$MECH" \
"            m->setBrakeMode(safe_);" \
"            m->setBrakeMode(BrakeMode::Hold);"

# 5. Stall detector always healthy. Killer: every Stalled timeline + the fault
#    latch asserts.
mutate "5 stall detector always reports healthy" "$SD" \
"        return (now - windowStart_).value() >= cfg_.persistence.value();" \
"        return false;"

# 6. An unconfirmed completion reports success. Killer: the Unconfirmed
#    timeline + the then() UNCONFIRMED-stops-the-chain case.
mutate "6 Unconfirmed reports Succeeded" "$OP" \
"            return finish(MechanismOutcome::Unconfirmed);" \
"            return finish(MechanismOutcome::Succeeded);"

# 7. The fault raise dropped, verdict kept — E1's exact hole class. Killer:
#    the cases that assert the LATCH independently of the outcome.
mutate "7 MechanismStalled raise dropped (verdict kept)" "$OP" \
"            raiseStalled();
            return finish(MechanismOutcome::Stalled);" \
"            return finish(MechanismOutcome::Stalled);"

# 8. The hostile jam injection made a no-op. Killer: the liveness pin (and
#    every jam timeline). NOTE the first form of this mutation (`return
#    false;`) BUILD-FAILED — the unused `now` tripped -Werror — and a
#    build-fail proves nothing (C4), so the no-op keeps reading `now`:
#    the clock is monotonic from 0, so `now < 0.0` is never true.
mutate "8 hostile jam injection is a no-op" "$HOSTILE" \
"        return now >= cfg_.start.value() && now < cfg_.end.value();" \
"        return now < 0.0;"

# ── Beyond the named eight ──────────────────────────────────────────────────────────

# 9. then()'s mechanism mapping inverted. Killer: both then() outcome cases
#    (success would stop the chain, failures would pass).
mutate "9 then() maps mechanism outcomes inverted" "$RTN" \
"            succeeded = (mo == manipulation::MechanismOutcome::Succeeded);" \
"            succeeded = (mo != manipulation::MechanismOutcome::Succeeded);"

# 10. The claim precondition defeated. Killer: the two-operations collision
#     case (no throw where one is required).
mutate "10 claim collision precondition defeated" "$OP" \
"            SHULIB_PRECONDITION(mech_->tryClaim(),
                                \"RunUntilConfirmed::start: mechanism already driven by \"" \
"            SHULIB_PRECONDITION(mech_->tryClaim() || true,
                                \"RunUntilConfirmed::start: mechanism already driven by \""

# 11. The pre-actuation confirm suppression removed. Killer: the eager-confirm
#     case (pred consulted during actuation → succeeds at tick 0 on the
#     pre-actuation state).
mutate "11 confirm consulted during actuation" "$OP" \
"        if (now.value() < actDeadline_.value()) {" \
"        if (false && now.value() < actDeadline_.value()) {"

# 12. Confirm/stall order swapped. Killer: the same-tick success-wins case
#     (added when planning THIS mutation exposed the hole — see the test).
mutate "12 stall checked before confirm (stall wins ties)" "$OP" \
"        if (confirm_()) {
            return finish(MechanismOutcome::Succeeded);
        }
        if (stall_.update(deps_.clock->now(), mech_->maxCurrent(), mech_->meanVelocity())) {
            raiseStalled();
            return finish(MechanismOutcome::Stalled);
        }" \
"        if (stall_.update(deps_.clock->now(), mech_->maxCurrent(), mech_->meanVelocity())) {
            raiseStalled();
            return finish(MechanismOutcome::Stalled);
        }
        if (confirm_()) {
            return finish(MechanismOutcome::Succeeded);
        }"

# 13. The discrete deadline pair disarmed (confirm window never closes) — the
#     no-hang analog for ActuateAndConfirm. Killer: the Unconfirmed timeline
#     (never exits → maxTicks Running).
mutate "13 ActuateAndConfirm confirm window never closes" "$OP" \
"        confirmDeadline_ = actDeadline_ + cfg_.confirmWindow;" \
"        confirmDeadline_ = actDeadline_ + cfg_.confirmWindow + units::Time{1.0e9};"

# 14. Safe-state ordering swapped (0 V before brake mode = a momentary coast
#     under load). Killer: the RecordingMotor event-order case.
mutate "14 safe state commands 0 V before the brake mode" "$MECH" \
"            m->setBrakeMode(safe_);
            m->setVoltage(units::Voltage{0.0});" \
"            m->setVoltage(units::Voltage{0.0});
            m->setBrakeMode(safe_);"

# 15. The T4 split regressed: ActuateAndConfirm's exit applies the safe state
#     (a successful grab un-grabs itself). Killer: the grip-kept asserts on
#     the Succeeded and Unconfirmed timelines.
mutate "15 discrete op applies safe state on completion (un-grabs)" "$OP" \
"        // path that forces the declared safe state on a discrete actuator.
        releaseClaim();" \
"        // path that forces the declared safe state on a discrete actuator.
        mech_->applySafeState();
        releaseClaim();"

# 16. The claim never released on a motor op's exit. Killer: the
#     collision-then-handoff case (opB.start() after opA's exit throws) and
#     the claimed() asserts on every exit path.
mutate "16 claim not released when a motor op finishes" "$OP" \
"    MechanismOutcome finish(MechanismOutcome o) {
        mech_->applySafeState();  // EVERY exit path lands here (banner)
        releaseClaim();" \
"    MechanismOutcome finish(MechanismOutcome o) {
        mech_->applySafeState();  // EVERY exit path lands here (banner)
        holdsClaim_ = false;  // (mutation: token left claimed on the mechanism)"

# 17. Motor-op exits skip the safe state entirely (success leaves the intake
#     spinning). Killer: the DoD timeline's exit-disposition asserts.
mutate "17 motor op exits without the safe state" "$OP" \
"    MechanismOutcome finish(MechanismOutcome o) {
        mech_->applySafeState();  // EVERY exit path lands here (banner)
        releaseClaim();
        outcome_ = o;" \
"    MechanismOutcome finish(MechanismOutcome o) {
        releaseClaim();
        outcome_ = o;"

printf '\nF1 mutations: %d RED (good), %d GREEN (HOLES), %d build-fail, %d SKIPPED\n' \
  "$pass" "$holes" "$buildfail" "$skipped"
if [ "$skipped" -gt 0 ]; then
  echo "A SKIP means a mutation DID NOT RUN — that is never allowed to pass quietly."
  exit 2
fi
if [ "$holes" -gt 0 ]; then
  exit 1
fi
