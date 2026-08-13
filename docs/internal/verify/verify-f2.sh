#!/usr/bin/env bash
# Chunk harness for F2 — the sequence engine and the guaranteed end-of-run action.
#
# ROLE, stated so the F1 mistake cannot repeat: this file is the CHUNK'S OWN
# harness (the brief's scope item 7). It is NOT the reviewer's independent
# verification — the reviewer writes their own, from scratch, and this script's
# green means only "the chunk checked itself". verify-f1.sh belongs to the
# reviewer and is untouched by this chunk.
#
# WHAT IT CHECKS
#   1. process: no commits made; the tree actually contains work
#   2. the full suite, both PROS-free guards, the ARM compile gate
#   3. THE NAME COLLISION: freeze-register rows F1–F5 (locked contracts named
#      "F1".."F5", which have NOTHING to do with chunks F1/F2) are unedited,
#      and no register row gained a LOCKED claim — F2 freezes NOTHING
#   4. scope: no season content, no field coordinate in sequence/, no invented
#      defaults in the guard's config
#   5. MUTATIONS: fourteen, each applied to the real header, REBUILT (a
#      non-compiling mutation read off a stale binary looks green — the C4
#      lesson), run, observed, restored, with restoration verified byte-1:1.
#      A mutation that stays GREEN is a hole and the most valuable output.
#
# SIGPIPE discipline (E2's lesson): trap PIPE; never pipe this script into
# `head`. Two kinds of red (verify-f1's convention): FAIL = the repo violates
# a property; HARNESS = this script could not ask its question.

set -uo pipefail
trap '' PIPE
cd /home/gonzei/projects/shulib || { echo "repo not found"; exit 1; }

BRIEF_COMMIT=9915745ea4fbde4b2c17ba0ff136a2d076de91f1
SCOPED='F2 *,C2/F2 *'
FAILS=0
HARNESS=0
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
grn()   { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()   { printf '\033[33m%s\033[0m\n' "$*"; }
hdr()   { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
fail()  { red "  FAIL: $*"; FAILS=$((FAILS+1)); }
harn()  { ylw "  HARNESS: $*"; HARNESS=$((HARNESS+1)); }
ok()    { grn "  ok: $*"; }

# ══ 1. process ═════════════════════════════════════════════════════════════════════
hdr "1. PROCESS — nothing committed, work present"
if [ "$(git rev-parse HEAD)" = "$BRIEF_COMMIT" ]; then
  ok "HEAD is still the brief commit"
else
  fail "HEAD moved — the chunk committed (forbidden)"
fi
if [ -n "$(git status --porcelain)" ]; then
  ok "working tree carries the chunk's work"
else
  fail "clean tree — the chunk produced nothing?"
fi

# ══ 2. suite + guards + ARM ════════════════════════════════════════════════════════
hdr "2. FULL SUITE"
if cmake --build build/test -j"$(nproc)" > "$T/build.log" 2>&1; then
  ./build/test/shulib_tests > "$T/suite.log" 2>&1
  if grep -q "Status: SUCCESS" "$T/suite.log"; then
    ok "$(grep 'test cases' "$T/suite.log" | tr -s ' ')"
    ok "$(grep 'assertions' "$T/suite.log" | tr -s ' ')"
  else
    fail "suite not green — see $T/suite.log"
  fi
else
  fail "build failed — see $T/build.log"
fi

hdr "3. PROS-FREE GUARDS"
if grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib >/dev/null; then
  fail "GUARD1: a pros/ include reached include/shulib"
else
  ok "GUARD1 (no pros/ includes)"
fi
if grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib >/dev/null; then
  fail "GUARD2: a sim/ include escaped sim/"
else
  ok "GUARD2 (sim/ stays in sim/)"
fi

hdr "4. ARM COMPILE GATE (all v2 headers)"
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort \
  | awk '{print "#include \""$0"\""}' > "$T/all.cpp"
echo 'int main(){return 0;}' >> "$T/all.cpp"
NHDR=$(( $(wc -l < "$T/all.cpp") - 1 ))
if arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
     -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
     -c "$T/all.cpp" -o /dev/null -Iinclude 2> "$T/arm.log"; then
  ok "ARM gate clean ($NHDR headers)"
else
  fail "ARM gate broke — see $T/arm.log"
fi

# ══ 5. the freeze register (THE NAME COLLISION) ═══════════════════════════════════
hdr "5. FREEZE REGISTER — rows F1–F5 unedited; F2 froze NOTHING"
git show "$BRIEF_COMMIT:docs/roadmap.md" > "$T/roadmap.base.md" 2>/dev/null \
  || harn "cannot read baseline roadmap from git"
python3 - "$T/roadmap.base.md" docs/roadmap.md <<'PY'
import re, sys

def rows(path):
    out = {}
    for line in open(path):
        m = re.match(r'^\|\s*(F\d+)\s*\|', line)
        if m:
            out[m.group(1)] = line.rstrip('\n')
    return out

old, new = rows(sys.argv[1]), rows(sys.argv[2])
problems = []
for key in ("F1", "F2", "F3", "F4", "F5"):
    if old.get(key) != new.get(key):
        problems.append(f"{key}: LOCKED row edited (the name-collision landmine)")
for key in sorted(set(new) - set(old), key=lambda k: int(k[1:])):
    if 'LOCKED' in new[key]:
        problems.append(f"{key}: NEW row claims LOCKED — F2 freezes nothing")
    else:
        print(f"  new row {key}: not locked (allowed)")
seq_rows = [k for k, v in new.items() if 'sequence' in v.lower() or 'end-of-run' in v.lower()]
if not any('NOT FROZEN' in new[k] or 'not frozen' in new[k] for k in seq_rows):
    problems.append("no register row states the sequence engine is NOT FROZEN "
                    "(silence in the register reads as frozen — D2's lesson)")
if problems:
    for p in problems:
        print(f"  REGISTER-FAIL: {p}")
    sys.exit(1)
print("  register: F1–F5 untouched; nothing newly locked; non-freeze stated")
PY
if [ $? -ne 0 ]; then fail "freeze register violated (see lines above)"; else ok "register clean"; fi

# ══ 6. scope gates ════════════════════════════════════════════════════════════════
hdr "6. SCOPE — no season content, no field knowledge, no invented defaults"
if grep -rnE 'buildStack|matchLoadCycle|endInMidfield|strategyMode' include/ test/ \
     --include='*.hpp' --include='*.cpp' >/dev/null; then
  fail "season-content symbol found in code (F4/F' owns those)"
else
  ok "no season-content symbols in code"
fi
# The library must never know a field position: sequence/ may not construct a
# Length or Pose2d literal (test fixtures live in test/, labelled).
if grep -nE 'Length\{|Pose2d\{' include/shulib/sequence/*.hpp >/dev/null; then
  fail "a coordinate literal lives in sequence/ (the library learned the field)"
else
  ok "sequence/ contains no coordinate literals"
fi
# Both instants must be required (0-default + validated > 0): a nonzero default
# would be an invented lead time / match length.
if grep -q 'endActionAt{0.0}' include/shulib/sequence/run_guard.hpp \
   && grep -q 'hardStopAt{0.0}' include/shulib/sequence/run_guard.hpp \
   && grep -q 'endActionAt.value() > 0.0' include/shulib/sequence/run_guard.hpp; then
  ok "no default lead time / match length (zero-defaults, validated required)"
else
  fail "the guard's instants are not required-with-no-default anymore"
fi

# ══ 7. mutations ══════════════════════════════════════════════════════════════════
hdr "7. MUTATIONS — applied, REBUILT, run ($SCOPED), restored (verified)"

RG=include/shulib/sequence/run_guard.hpp
MO=include/shulib/manipulation/mechanism_op.hpp
HM=include/shulib/hal/mechanism.hpp
MS=include/shulib/motion/motion_scheduler.hpp
cp "$RG" "$T/rg.bak"; cp "$MO" "$T/mo.bak"; cp "$HM" "$T/hm.bak"; cp "$MS" "$T/ms.bak"

RED=0; GREEN=0; BFAIL=0; SKIPPED=0

# Positive control first: the scoped suite must be green UNMUTATED, or a
# mutation "going red" would mean nothing.
./build/test/shulib_tests -tc="$SCOPED" > "$T/control.log" 2>&1
if ! grep -q "Status: SUCCESS" "$T/control.log"; then
  harn "positive control failed — scoped suite not green before any mutation"
fi

# mutate FILE OLD NEW  — python exact-count-1 replace; exits 3 if not found.
mutate() {
  python3 - "$1" <<PY
import sys
p = sys.argv[1]
s = open(p).read()
old = '''$2'''
new = '''$3'''
if s.count(old) != 1:
    sys.exit(3)
open(p, 'w').write(s.replace(old, new))
PY
}

restore_all() {
  cp "$T/rg.bak" "$RG"; cp "$T/mo.bak" "$MO"; cp "$T/hm.bak" "$HM"; cp "$T/ms.bak" "$MS"
}

run_mutation() {  # NAME FILE OLD NEW
  local name="$1" file="$2" old="$3" new="$4"
  printf '  %-58s ' "$name"
  if ! mutate "$file" "$old" "$new"; then
    ylw "SKIPPED (anchor text not found — harness problem, not a finding)"
    SKIPPED=$((SKIPPED+1)); restore_all; return
  fi
  if ! cmake --build build/test -j"$(nproc)" > "$T/mbuild.log" 2>&1; then
    ylw "BUILD-FAIL (mutation does not compile — proves nothing either way)"
    BFAIL=$((BFAIL+1)); restore_all
    cmake --build build/test -j"$(nproc)" > /dev/null 2>&1 || harn "restore rebuild failed"
    return
  fi
  ./build/test/shulib_tests -tc="$SCOPED" > "$T/mrun.log" 2>&1
  if grep -q "Status: SUCCESS" "$T/mrun.log"; then
    red "GREEN — A HOLE"
    GREEN=$((GREEN+1))
  else
    grn "RED (caught: $(grep -c 'ERROR' "$T/mrun.log" | tr -d ' ') failing assertions)"
    RED=$((RED+1))
  fi
  restore_all
  cmake --build build/test -j"$(nproc)" > /dev/null 2>&1 || harn "restore rebuild failed"
}

run_mutation "M1 latch disarmed (no post-deadline cancel)" "$RG" \
'        sched_->cancel();  // legal from pace(); pinned in C2'"'"'s re-entrancy list
        ++postExpiryCancels_;' \
'        return;  // MUTATION
        ++postExpiryCancels_;'

run_mutation "M2 deadline check moved AFTER the plant step" "$RG" \
'        if (running_) {
            ++pacesSeen_;
            const double now = clock_->now().value();
            if (now >= floorDeadline_) {
                fireFloor(now);
            } else if (now >= actDeadline_ && !inEndAction_) {
                noteExpired(now);
                cutActiveMotion();
            }
        }
        inner_->pace();  // the world advances AFTER the checks (the ordering pin)' \
'        inner_->pace();  // MUTATION: world first, checks after
        if (running_) {
            ++pacesSeen_;
            const double now = clock_->now().value();
            if (now >= floorDeadline_) {
                fireFloor(now);
            } else if (now >= actDeadline_ && !inEndAction_) {
                noteExpired(now);
                cutActiveMotion();
            }
        }'

run_mutation "M3 end action reuses the caller-side verdict" "$RG" \
'            report.endActionSucceeded = invokeEndAction(std::forward<EndAction>(endAction));' \
'            (void)invokeEndAction(std::forward<EndAction>(endAction));
            report.endActionSucceeded = !report.scoringCut;  // MUTATION'

run_mutation "M4 the guard goes silent (no verdict lines)" "$RG" \
'        logVerdict(report);' \
'        // MUTATION: silence'

run_mutation "M5 op-cancel dropped, applySafeState kept" "$RG" \
'            if (claimant != nullptr) {
                claimant->cancel();
            } else if (m->claimed()) {' \
'            if (claimant != nullptr) {
                (void)claimant;  // MUTATION: repaint only, never cancel
            } else if (m->claimed()) {'

run_mutation "M6 hard floor conditional on end action finishing" "$RG" \
'    void fireFloor(double now) {
        if (!floorFired_) {' \
'    void fireFloor(double now) {
        if (inEndAction_) {
            return;  // MUTATION
        }
        if (!floorFired_) {'

run_mutation "M7 deadline-aware wait reports Satisfied" "$RG" \
'        if (expiredNow()) {
            return GuardedWaitResult::RunExpired;  // wins the tie (measured trap)
        }' \
'        if (expiredNow()) {
            return GuardedWaitResult::Satisfied;  // MUTATION
        }'

run_mutation "M8 end action starts BEFORE cancel-all" "$RG" \
'            cancelAll();  // strictly precedes the act (banner: T2)
            logActStart();' \
'            logActStart();  // MUTATION: act first, cancel-all after'

run_mutation "M9 run() never disarms the guard" "$RG" \
'    void disarm() noexcept { running_ = false; }' \
'    void disarm() noexcept { /* MUTATION */ }'

run_mutation "M10 pred keeps being called after expiry" "$RG" \
'            [this, &pred] { return expiredNow() || pred(); }, timeout.value());' \
'            [this, &pred] { return pred() || expiredNow(); }, timeout.value());'

run_mutation "M11 mid-flight destructor cancel removed" "$MO" \
'    ~RunUntilConfirmed() override {
        if (started_ && !finished_) {
            cancel();
        }
    }' \
'    ~RunUntilConfirmed() override { /* MUTATION */ }'

run_mutation "M12 claim stops storing the claimant" "$HM" \
'        claimant_ = &claimant;
        return true;' \
'        (void)claimant;  // MUTATION
        return true;'

run_mutation "M13 cancel() tightened to forbid the pacer position" "$MS" \
'    void cancel() {
        SHULIB_PRECONDITION(!inTick_,' \
'    void cancel() {
        SHULIB_PRECONDITION(!inWait_, "MUTATION: pacer position forbidden");
        SHULIB_PRECONDITION(!inTick_,'

run_mutation "M14 wait unwind guards neutered" "$MS" \
'        FlagScope wait{inWait_};
        WaitUnwindGuard unwind{*this};  // F2: a throw must not strand an armed motion
        loopMonitor_.reset();
        stalledPaces_ = 0;
        const double deadline = schedDeps_.ctx->clock().now().value() + timeoutSeconds;' \
'        FlagScope wait{inWait_};
        WaitUnwindGuard unwind{*this};
        unwind.disarm();  // MUTATION: guard exists but never fires
        loopMonitor_.reset();
        stalledPaces_ = 0;
        const double deadline = schedDeps_.ctx->clock().now().value() + timeoutSeconds;'

# restoration must be byte-perfect — the chunk holds uncommitted work and a
# git checkout "restore" would delete it (E2 lost an hour to that).
for pair in "rg.bak:$RG" "mo.bak:$MO" "hm.bak:$HM" "ms.bak:$MS"; do
  b="${pair%%:*}"; f="${pair#*:}"
  if cmp -s "$T/$b" "$f"; then ok "restored byte-identical: $f"; else fail "RESTORE MISMATCH: $f"; fi
done

hdr "MUTATION TALLY"
echo "  RED (caught): $RED · GREEN (HOLES): $GREEN · BUILD-FAIL: $BFAIL · SKIPPED: $SKIPPED"
[ "$GREEN" -gt 0 ] && fail "$GREEN mutation(s) survived — holes to close"
[ "$SKIPPED" -gt 0 ] && harn "$SKIPPED mutation(s) never applied"

# ══ verdict ═══════════════════════════════════════════════════════════════════════
hdr "VERDICT"
if [ "$FAILS" -gt 0 ]; then red "  $FAILS FAIL(s)"; exit 1; fi
if [ "$HARNESS" -gt 0 ]; then ylw "  no FAILs, but $HARNESS harness problem(s)"; exit 2; fi
grn "  all clear"
exit 0
