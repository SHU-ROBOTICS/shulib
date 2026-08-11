#!/usr/bin/env bash
# Independent verification for chunk D1. Run from the repo root.
# This re-runs everything from scratch rather than trusting the chunk report.
# Exit code is nonzero if any gate fails.

set -uo pipefail
cd /home/gonzei/projects/shulib || exit 1

fail=0
hdr() { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }
ok()  { printf '  \033[32mPASS\033[0m %s\n' "$1"; }
bad() { printf '  \033[31mFAIL\033[0m %s\n' "$1"; fail=1; }

hdr "0. Branch and tree state"
branch=$(git branch --show-current)
[ "$branch" = "shulib-v2" ] && ok "on shulib-v2" || bad "on '$branch', expected shulib-v2"
if git diff --quiet HEAD 2>/dev/null && [ -z "$(git status --porcelain)" ]; then
  bad "tree is CLEAN — the chunk was told not to commit, so it should be dirty"
else
  ok "tree is dirty (expected: chunk left work uncommitted)"
fi
# The chunk was told not to commit. HEAD must still be the D1 brief.
head_subject=$(git log -1 --pretty=%s)
echo "  HEAD: $head_subject"
case "$head_subject" in
  *"D1 brief"*) ok "HEAD is still the D1 brief — nothing was committed" ;;
  *) bad "HEAD moved — the chunk may have committed against instructions" ;;
esac

hdr "1. Full rebuild from scratch + suite"
cmake --build build/test -j"$(nproc)" >/tmp/d1_build.log 2>&1 \
  && ok "build clean under -Werror" \
  || { bad "build FAILED — see /tmp/d1_build.log"; tail -30 /tmp/d1_build.log; }
if [ -x ./build/test/shulib_tests ]; then
  ./build/test/shulib_tests >/tmp/d1_tests.log 2>&1
  rc=$?
  tail -5 /tmp/d1_tests.log
  [ $rc -eq 0 ] && ok "suite green" || bad "suite RED (rc=$rc)"
else
  bad "no test binary"
fi

hdr "2. CI guard 1 — library stays PROS-free"
if grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib; then
  bad "PROS include found in include/shulib"
else
  ok "PROS-free (all of include/shulib)"
fi

hdr "3. CI guard 2 — core never includes the sim plant"
if grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib; then
  bad "core includes shulib/sim — ground truth reachable from an estimator"
else
  ok "layering holds: core is sim-free"
fi

hdr "4. ARM compile gate — every v2 header as one TU"
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort \
  | awk '{print "#include \""$0"\""}' > /tmp/d1_all_headers.cpp
echo 'int main() { return 0; }' >> /tmp/d1_all_headers.cpp
n=$(grep -c '#include' /tmp/d1_all_headers.cpp)
echo "  TU includes $n headers (C8 baseline: 102)"
if arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion \
     -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
     -c /tmp/d1_all_headers.cpp -o /dev/null -Iinclude 2>/tmp/d1_arm.log; then
  ok "ARM gate CLEAN ($n headers)"
else
  bad "ARM gate FAILED"; tail -30 /tmp/d1_arm.log
fi

hdr "5. C7/C8 removability — no public doc may link into docs/internal/"
if grep -rnE 'docs/internal|internal/|chunks/|RESUMING|build-order|-COMPLETED|-PROGRESS|guide-maintenance' \
     README.md test/README.md docs/*.md docs/guide/*.md 2>/dev/null; then
  bad "a public doc references internal — the squash-merge to main would break"
else
  ok "removability holds (public docs are internal-free)"
fi

hdr "6. F6 MUST STILL BE UNFROZEN (D2 owns the freeze)"
grep -rn "F6" docs/roadmap.md | grep -iE "freeze|frozen|pending" | head -10
if grep -qiE "not frozen|NOT FROZEN" include/shulib/chassis/chassis.hpp; then
  ok "chassis.hpp still carries the not-frozen notice"
else
  bad "chassis.hpp lost its not-frozen notice — D1 must not freeze F6"
fi
for f in docs/guide/10-the-api.md docs/guide/14-what-it-cannot-do-yet.md; do
  if grep -qiE "not frozen|isn't frozen|is not yet frozen|may still change" "$f"; then
    ok "$(basename "$f") still carries the not-frozen notice"
  else
    bad "$(basename "$f") lost its not-frozen notice"
  fi
done

hdr "7. Guide chapter 09 landed and is wired in"
if ls docs/guide/09-*.md >/dev/null 2>&1; then
  ok "chapter 09 exists: $(ls docs/guide/09-*.md)"
else
  bad "no docs/guide/09-*.md — a DoD item"
fi
c=$(grep -c 'guide-09' test/guide_examples_test.cpp 2>/dev/null || echo 0)
[ "$c" -gt 0 ] && ok "guide-09 example cases present ($c refs)" || bad "no guide-09 cases in test/guide_examples_test.cpp"

hdr "8. Deliverables present"
for f in docs/internal/chunks/D1-PROGRESS.md docs/internal/chunks/D1-COMPLETED.md; do
  if [ -f "$f" ]; then ok "$f ($(wc -l < "$f") lines)"; else bad "$f MISSING"; fi
done
if [ -f docs/internal/chunks/D1-COMPLETED.md ]; then
  n=$(wc -l < docs/internal/chunks/D1-COMPLETED.md)
  [ "$n" -ge 500 ] && ok "completion record depth ${n} lines (C1-C5 band: 570-654)" \
                   || bad "completion record only ${n} lines — below the C1-C5 depth bar"
fi

hdr "9. Delegation check — recipes must add NO motion logic"
echo "  (manual review required; these are the smells)"
if ls include/shulib/chassis/*.hpp >/dev/null 2>&1; then
  grep -rn "kP\|kI\|kD\|integral\|derivative\|atan2\|std::hypot" \
    include/shulib/chassis/ --include=*.hpp | grep -v chassis.hpp | head -20 \
    && echo "  ^^ INSPECT: control math in the recipe layer would violate delegate-only" \
    || ok "no obvious control math outside chassis.hpp"
fi

hdr "SUMMARY"
[ $fail -eq 0 ] && printf '\033[32mALL MECHANICAL GATES PASSED\033[0m — manual review still required:\n' \
                || printf '\033[31mSOME GATES FAILED\033[0m — see above. Also still required:\n'
cat <<'EOF'
  - Read D1-COMPLETED.md's facade-critique section in full (it is D2's input)
  - Confirm the design fork (eager vs deferred) is analyzed with the rejected alternative
  - Confirm every claimed mutation was actually EXECUTED (PROGRESS log should show it live)
  - Spot-check that green-mutation holes, if any, got closing tests that go red alone
  - Verify DoD claims against evidence, not prose
EOF
exit $fail
