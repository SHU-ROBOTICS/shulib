#!/usr/bin/env bash
# Independent verification for chunk D3 — the cookbook, the generated reference,
# the doc-coverage gate, and the Routine freeze.
#
# Everything is re-derived from scratch; nothing is taken from the chunk report.
# Read docs/internal/verify/README.md first: A RED GATE IS A QUESTION, NOT A
# VERDICT — check whether a failure predates the chunk before attributing it.

set -uo pipefail
cd /home/gonzei/projects/shulib || exit 1

fail=0
hdr()  { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }
ok()   { printf '  \033[32mPASS\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; fail=1; }
warn() { printf '  \033[33mNOTE\033[0m %s\n' "$1"; }

hdr "0. Branch / tree / nothing committed"
[ "$(git branch --show-current)" = "shulib-v2" ] && ok "on shulib-v2" || bad "wrong branch"
[ -n "$(git status --porcelain)" ] && ok "tree dirty (expected)" || bad "tree clean — chunk produced nothing?"
warn "HEAD: $(git log -1 --pretty='%h %s')"
warn "  (D3 must NOT have committed; HEAD should still be the D3 brief commit)"

hdr "1. Build + suite"
cmake --build build/test -j"$(nproc)" >/tmp/d3_build.log 2>&1 \
  && ok "build clean under -Werror (doc gates ran first — see the log head)" \
  || { bad "BUILD FAILED"; tail -40 /tmp/d3_build.log; }
head -5 /tmp/d3_build.log | sed 's/^/    /'
./build/test/shulib_tests >/tmp/d3_tests.log 2>&1
rc=$?; tail -4 /tmp/d3_tests.log | sed 's/^/    /'
[ $rc -eq 0 ] && ok "suite green" || bad "suite RED"

hdr "2. CI guards"
grep -rqnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib \
  && bad "PROS leak" || ok "GUARD1 PROS-free"
grep -rqnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib \
  && bad "core includes sim" || ok "GUARD2 core sim-free"

hdr "3. ARM gate"
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort \
  | awk '{print "#include \""$0"\""}' > /tmp/d3_all.cpp
echo 'int main(){return 0;}' >> /tmp/d3_all.cpp
n=$(grep -c '#include' /tmp/d3_all.cpp)
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/d3_all.cpp -o /dev/null -Iinclude \
  2>/tmp/d3_arm.log && ok "ARM gate clean ($n headers)" || { bad "ARM gate failed"; tail -20 /tmp/d3_arm.log; }

hdr "4. THE LIBRARY MUST NOT DEPEND ON THE GENERATOR (brief constraint 6)"
grep -rqn 'api_doc_tool\|tools/' include/shulib \
  && { bad "the library references the doc tool"; grep -rn 'api_doc_tool\|tools/' include/shulib; } \
  || ok "include/shulib never mentions the tool — it is a tool, not a build dependency"
grep -q 'python3' Makefile 2>/dev/null \
  && bad "the PROS robot Makefile now needs python3" \
  || ok "the robot build is untouched by the doc gates"

hdr "5. Removability (four-term gate must be EMPTY)"
grep -rqnE 'internal/|chunks/|RESUMING|build-order' README.md test/README.md docs/*.md \
     docs/guide/*.md docs/cookbook/*.md docs/api/*.md \
  && { bad "public doc references internal"; grep -rnE 'internal/|chunks/|RESUMING|build-order' \
       README.md test/README.md docs/*.md docs/guide/*.md docs/cookbook/*.md docs/api/*.md; } \
  || ok "four-term gate empty (now also enforced by the build: check-removability)"

hdr "6. THE DOC GATES, run standalone"
for c in self-test check-coverage check-fresh check-examples check-removability; do
  if python3 tools/api_doc_tool.py "$c" >/tmp/d3_$c.log 2>&1; then
    ok "api_doc_tool $c"
    head -2 /tmp/d3_$c.log | sed 's/^/      /'
  else
    bad "api_doc_tool $c FAILED"; head -20 /tmp/d3_$c.log
  fi
done

hdr "7. DETERMINISM — two runs, byte-identical"
rm -rf /tmp/d3_api_a /tmp/d3_api_b
cp -r docs/api /tmp/d3_api_a
python3 tools/api_doc_tool.py generate >/dev/null 2>&1
cp -r docs/api /tmp/d3_api_b
if diff -r /tmp/d3_api_a /tmp/d3_api_b >/dev/null; then
  ok "regeneration is byte-identical (B4's check is usable)"
else
  bad "generator is NON-DETERMINISTIC"; diff -r /tmp/d3_api_a /tmp/d3_api_b | head -20
fi

hdr "8. THE COVERAGE GATE MUST CATCH ITS OWN OMISSION (the brief's central proof)"
cp include/shulib/chassis/routine.hpp /tmp/d3_routine.bak
python3 - <<'PY'
p = 'include/shulib/chassis/routine.hpp'
s = open(p).read()
s = s.replace("    /// Seed the pose estimate with the measured starting pose — every auton's",
              "    /// PLANTED-BREAK", 1)
s = s.replace("    /// first line (heading stays IMU-owned, exactly Chassis::setPose).\n", "", 1)
s = s.replace("    /// PLANTED-BREAK\n", "", 1)
open(p, 'w').write(s)
PY
if python3 tools/api_doc_tool.py check-coverage >/tmp/d3_planted.log 2>&1; then
  bad "PLANTED UNDOCUMENTED MEMBER WENT UNDETECTED — the gate is decoration"
else
  grep -q 'Routine::startAt' /tmp/d3_planted.log \
    && ok "gate fires and NAMES the member: $(grep 'UNDOCUMENTED' /tmp/d3_planted.log | head -1 | xargs)" \
    || { bad "gate fired but did not name Routine::startAt"; head -12 /tmp/d3_planted.log; }
fi
cp /tmp/d3_routine.bak include/shulib/chassis/routine.hpp
python3 tools/api_doc_tool.py check-coverage >/dev/null 2>&1 \
  && ok "header restored, gate green again" || bad "RESTORE FAILED — fix routine.hpp by hand"

hdr "9. THE F10 PIN MUST CATCH A NOEXCEPT DROP (D2's hole #1, not re-opened)"
cp include/shulib/chassis/routine.hpp /tmp/d3_routine.bak
sed -i 's|\[\[nodiscard\]\] bool ok() const noexcept { return stoppedAt_ == 0; }|[[nodiscard]] bool ok() const { return stoppedAt_ == 0; }|' \
  include/shulib/chassis/routine.hpp
if g++ -std=c++20 -fsyntax-only -Iinclude -Itest -isystem test/vendor \
     test/routine_signature_pin_test.cpp 2>/tmp/d3_pin.log; then
  bad "a noexcept DROP on a non-overloaded member compiled — D2's hole #1 is back"
else
  grep -q 'F10 FREEZE VIOLATION' /tmp/d3_pin.log \
    && ok "pin fires, naming F10: $(grep -o 'frozen signature of [^ ]*[^c]*changed' /tmp/d3_pin.log | head -1 | cut -c1-70)" \
    || { bad "build failed but the F10 message never appeared"; head -12 /tmp/d3_pin.log; }
fi
cp /tmp/d3_routine.bak include/shulib/chassis/routine.hpp
g++ -std=c++20 -fsyntax-only -Iinclude -Itest -isystem test/vendor \
  test/routine_signature_pin_test.cpp 2>/dev/null \
  && ok "header restored, pin compiles clean" || bad "RESTORE FAILED"

hdr "10. THE FREEZE ITSELF — is the F10 row true?"
row=$(grep -E '^\| F10 \|' docs/roadmap.md)
if [ -z "$row" ]; then bad "no F10 row found"; else
  echo "$row" | cut -c1-160
  echo "$row" | grep -qiE 'LOCKED' && ok "F10 marked LOCKED" || bad "F10 not marked LOCKED"
  cnt=0
  for m in startAt moveTo driveTo strafeTo turnTo face followTrajectory brake hold pause \
           waitFor ok result lastTrajectory chassis RoutineResult RoutineStopCause; do
    echo "$row" | grep -q "$m" && cnt=$((cnt+1))
  done
  echo "  members named in the row: $cnt / 17"
  [ "$cnt" -ge 15 ] && ok "row enumerates the real surface ($cnt)" \
                    || bad "row narrower than the surface ($cnt named)"
  echo "$row" | grep -q 'then()' \
    && ok "row states then()'s EXCLUSION explicitly (silence reads as frozen)" \
    || bad "row silent on then() — the brief's A4"
fi
grep -q 'unfrozen until D3' docs/roadmap.md include/shulib/chassis/*.hpp docs/guide/*.md 2>/dev/null \
  && { bad "a stale 'unfrozen until D3' claim survives"; \
       grep -rn 'unfrozen until D3' docs/roadmap.md include/shulib/chassis/ docs/guide/; } \
  || ok "no stale 'unfrozen until D3' claims anywhere"

hdr "11. Deliverables"
for f in docs/internal/chunks/D3-PROGRESS.md docs/internal/chunks/D3-COMPLETED.md \
         docs/cookbook/README.md docs/api/README.md docs/api/chassis.md docs/api/routine.md \
         tools/api_doc_tool.py test/cookbook_examples_test.cpp \
         test/routine_signature_pin_test.cpp test/api_reference_fidelity_test.cpp \
         docs/internal/docs-publishing.md; do
  [ -f "$f" ] && ok "$f ($(wc -l < "$f") lines)" || bad "$f MISSING"
done
if [ -f docs/internal/chunks/D3-COMPLETED.md ]; then
  n=$(wc -l < docs/internal/chunks/D3-COMPLETED.md)
  [ "$n" -ge 570 ] && ok "record depth ${n} lines" || bad "record only ${n} lines — below the bar"
  for k in "critique" "A1" "B1" "B2" "B3" "B4" "GREEN"; do
    grep -qi "$k" docs/internal/chunks/D3-COMPLETED.md \
      && ok "record covers: $k" || bad "record never mentions $k"
  done
fi

hdr "12. Publish honesty"
grep -q 'Generated API docs' docs/roadmap.md && {
  if grep -A14 '\[~\] \*\*Generated API docs\*\*' docs/roadmap.md | grep -qi 'nothing is published\|not published\|no Pages'; then
    ok "roadmap marks generated docs [~] and says nothing is published"
  else
    bad "roadmap may be over-claiming publication"
  fi
}
ls .github/workflows/ | sed 's/^/    workflow: /'
[ -f .github/workflows/pages.yml ] && warn "a pages workflow exists — re-check the [~]" \
                                   || ok "no Pages workflow — the [~] is honest"

hdr "SUMMARY"
[ $fail -eq 0 ] && printf '\033[32mMECHANICAL GATES PASSED\033[0m\n' || printf '\033[31mGATES FAILED\033[0m\n'
cat <<'EOF'
  STILL REQUIRED BY HAND (a freeze and a cookbook are judgment, not just gates):
   - Delete a /// comment from a public member yourself; confirm the build fails NAMING it
   - Break a pinned Routine signature yourself; confirm the build fails NAMING F10
   - Read the Routine critique: does every awkwardness carry a recommendation, and is each
     one genuinely ADDITIVELY fixable (if not, the freeze foreclosed something)
   - Read a cookbook recipe cold and ask whether you could write a routine from it
     (the DoD clause D3 could NOT close — it needs a person who has not read the code)
   - Confirm no docket item (A1-A4, B1-B4) was silently skipped
EOF
exit $fail
