#!/usr/bin/env bash
# Independent verification for chunk D2 — the F6 freeze.
# Stricter than the D1 harness: a freeze that is merely "green" is not verified.
# Everything is re-derived from scratch; nothing is taken from the chunk report.

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
warn "HEAD: $(git log -1 --pretty=%s)"
warn "  (D2 was paused mid-flight and checkpointed; confirm no commit CLAIMS D2 is complete)"

hdr "1. Build + suite"
cmake --build build/test -j"$(nproc)" >/tmp/d2_build.log 2>&1 \
  && ok "build clean under -Werror" \
  || { bad "BUILD FAILED"; tail -30 /tmp/d2_build.log; }
./build/test/shulib_tests >/tmp/d2_tests.log 2>&1
rc=$?; tail -4 /tmp/d2_tests.log
[ $rc -eq 0 ] && ok "suite green" || bad "suite RED"

hdr "2. CI guards"
grep -rqnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib \
  && bad "PROS leak" || ok "GUARD1 PROS-free"
grep -rqnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib \
  && bad "core includes sim" || ok "GUARD2 core sim-free"

hdr "3. ARM gate"
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort \
  | awk '{print "#include \""$0"\""}' > /tmp/d2_all.cpp
echo 'int main(){return 0;}' >> /tmp/d2_all.cpp
n=$(grep -c '#include' /tmp/d2_all.cpp)
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/d2_all.cpp -o /dev/null -Iinclude \
  2>/tmp/d2_arm.log && ok "ARM gate clean ($n headers)" || { bad "ARM gate failed"; tail -20 /tmp/d2_arm.log; }

hdr "4. Removability (four-term gate must be EMPTY)"
grep -rqnE 'internal/|chunks/|RESUMING|build-order' README.md test/README.md docs/*.md docs/guide/*.md \
  && { bad "public doc references internal"; grep -rnE 'internal/|chunks/|RESUMING|build-order' README.md test/README.md docs/*.md docs/guide/*.md; } \
  || ok "four-term gate empty"

hdr "5. THE FREEZE ITSELF — is the F6 row true?"
row=$(grep -E '^\| F6 \|' docs/roadmap.md)
if [ -z "$row" ]; then bad "no F6 row found"; else
  echo "$row" | cut -c1-200
  echo "$row" | grep -qiE 'LOCKED' && ok "F6 marked LOCKED" || bad "F6 not marked LOCKED"
  # the row must enumerate more than the original five verbs
  cnt=0
  for m in moveTo strafeTo turnTo followTrajectory drive brake hold cancel waitUntil pose setPose \
           strafeAuthority lastExitReason lastCompleted motionConfig deps scheduler; do
    echo "$row" | grep -q "$m" && cnt=$((cnt+1))
  done
  echo "  members named in the row: $cnt / 17"
  [ "$cnt" -ge 10 ] && ok "row enumerates the real surface ($cnt members)" \
                    || bad "row still narrower than the surface ($cnt named) — brief §1"
  echo "$row" | grep -qi "Routine" \
    && ok "row states Routine's freeze status explicitly" \
    || bad "row silent on Routine — silence reads as 'frozen' (brief A2)"
fi

hdr "6. Is there a versioning mechanism now? (brief §2)"
if grep -rqniE "SHULIB_VERSION|shulib_api_version|kApiVersion|apiVersion" include/ 2>/dev/null; then
  ok "a version identifier exists in include/"
  grep -rniE "SHULIB_VERSION|shulib_api_version|kApiVersion|apiVersion" include/ | head -5
else
  warn "no version constant in include/ — acceptable ONLY if D2 wrote a documented policy instead"
fi
grep -rqiE "breaking change|version bump|migration" docs/roadmap.md docs/internal/chunks/D2-COMPLETED.md 2>/dev/null \
  && ok "a versioning policy is written down somewhere" \
  || bad "no versioning policy found — the freeze promise still has no meaning"

hdr "7. THE SIGNATURE PIN — does it exist? (brief §3)"
pin=$(ls test/*freeze* test/*signature* test/*f6* 2>/dev/null | head -3)
if [ -n "$pin" ]; then
  ok "pin test file(s): $pin"
  echo "  static_asserts: $(grep -c static_assert $pin 2>/dev/null | paste -sd+ | bc 2>/dev/null || echo '?')"
else
  bad "no signature-pin test file found — the freeze is unenforced"
fi

hdr "8. THE NOTICE SWEEP — must be COMPLETE, not partial (brief constraint 5)"
tot=0
for f in include/shulib/chassis/chassis.hpp docs/guide/09-the-recipe-api.md \
         docs/guide/10-the-api.md docs/guide/14-what-it-cannot-do-yet.md docs/guide/README.md; do
  [ -f "$f" ] || continue
  c=$(grep -ciE "not frozen|NOT FROZEN|isn't frozen|is not yet frozen|not yet frozen" "$f")
  tot=$((tot+c))
  printf "  %-46s %s\n" "$(basename "$f")" "$c"
done
[ "$tot" -eq 0 ] && ok "sweep complete — zero stale not-frozen claims" \
                 || bad "$tot stale 'not frozen' claim(s) remain — a half-swept freeze contradicts itself"

hdr "9. GUIDE VERBATIM COUPLING — all chapters, not just 09"
python3 - <<'PY'
import re, glob, sys
src = open('test/guide_examples_test.cpp').read()
srcset = {l.strip() for l in src.splitlines()}
bad_lines, total = [], 0
for path in sorted(glob.glob('docs/guide/*.md')):
    for bi, b in enumerate(re.findall(r'```cpp\n(.*?)```', open(path).read(), re.S), 1):
        for line in b.splitlines():
            s = line.strip()
            if not s:
                continue
            total += 1
            if s not in srcset:
                bad_lines.append((path, bi, s))
print(f"  checked {total} quoted lines across docs/guide/*.md")
if bad_lines:
    print(f"  *** {len(bad_lines)} NOT VERBATIM ***")
    for p, bi, s in bad_lines[:25]:
        print(f"    {p} block {bi}: {s}")
    sys.exit(1)
print("  all quoted lines appear verbatim in test/guide_examples_test.cpp")
PY
[ $? -eq 0 ] && ok "guide examples verbatim" || bad "guide has drifted from the compiled examples"

hdr "10. Deliverables"
for f in docs/internal/chunks/D2-PROGRESS.md docs/internal/chunks/D2-COMPLETED.md; do
  [ -f "$f" ] && ok "$f ($(wc -l < "$f") lines)" || bad "$f MISSING"
done
if [ -f docs/internal/chunks/D2-COMPLETED.md ]; then
  n=$(wc -l < docs/internal/chunks/D2-COMPLETED.md)
  [ "$n" -ge 500 ] && ok "record depth ${n} lines" || bad "record only ${n} lines — below the bar"
  # the 18-row ledger must actually be re-checked
  for k in "C2 §11" "C3 §11" "C1 §11"; do
    grep -q "$k" docs/internal/chunks/D2-COMPLETED.md \
      && ok "record references the $k ledger" \
      || bad "record never mentions $k — C4 §10 required re-checking every row"
  done
fi

hdr "SUMMARY"
[ $fail -eq 0 ] && printf '\033[32mMECHANICAL GATES PASSED\033[0m\n' || printf '\033[31mGATES FAILED\033[0m\n'
cat <<'EOF'
  STILL REQUIRED BY HAND (a freeze is judgment, not just gates):
   - Break a frozen signature yourself; confirm the pin fails the build naming F6
   - Read every one of the nine D1 §2 rulings; confirm each has a rejected alternative
   - If the time retype landed: confirm the accuracy numbers are IDENTICAL, not merely green
   - Confirm no docket item was silently skipped
EOF
exit $fail
