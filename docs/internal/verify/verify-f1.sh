#!/usr/bin/env bash
# Independent verification for chunk F1 — the `Mechanism` seam + fakes.
#
# Everything below is RE-DERIVED FROM SCRATCH. Nothing is taken from the chunk's
# own report, its progress log, or its completion record: the report is the thing
# being checked, not evidence.
#
# ═══ READ THIS FIRST (docs/internal/verify/README.md, "A caution learned the hard way")
#
#   A RED GATE IS A QUESTION, NOT A VERDICT.
#
# verify-d1.sh once reported a removability FAILURE that turned out to be C8's, not
# D1's — five pre-existing prose mentions, none of them links, and the harness was
# stricter than the property actually required. Before you attribute anything below
# to F1:
#   1. Ask whether it PREDATES the chunk — `git show HEAD:<file>`. Several gates here
#      do that triage for you and label a hit PRE-EXISTING; those do not fail.
#   2. Ask whether it violates the REAL property or only a proxy for it. Where this
#      script uses a proxy, the section header says so in the comment above it.
# Every FAIL in this script is an instruction to go look, not a conclusion.
#
# ═══ WHAT MAKES THIS SCRIPT DIFFERENT FROM "did the tests pass"
#
# It checks what chunk F1 is specifically FORBIDDEN to do:
#   * commit anything (HEAD must still be the F1 brief commit, 8c600bf)
#   * produce nothing (a clean tree after a chunk is a failure, not a success)
#   * move any pinned member of routine.hpp other than then() — the ONE unfrozen member
#   * edit Freeze Register rows F1–F5 (the "F1" name collision landmine)
#   * freeze anything (F1 freezes NOTHING; the register must not claim a new lock)
#   * put game semantics in hal/, or let a mechanism own a loop or a task
#   * mint a fault code without pinning it, or renumber an existing one
#   * link a public document into docs/internal/ (the C7/C8 removability property)
#   * inflate the "never driven a robot" claims
#
# ═══ IT DOES NOT MODIFY THE REPOSITORY
#
# The pin BITE checks (does the pin still fail the build when a frozen signature
# moves?) are done against a SHADOW include tree in /tmp — a byte copy of the header
# placed on an earlier -I path — never by editing include/. verify-d3.sh edited the
# real header and restored it; that is unsafe while a chunk holds UNCOMMITTED work in
# the same file, and `git checkout` as a restore would discard the chunk rather than
# the mutation (E2 lost an hour to exactly that).
#
# Usage:  /path/to/verify-f1.sh              (cd is hard-coded; run from anywhere)
#         /path/to/verify-f1.sh > f1.log 2>&1   if you want to page it — do NOT pipe
#                                               it into `head` (SIGPIPE), E2's lesson.

set -uo pipefail
trap '' PIPE
cd /home/gonzei/projects/shulib || { echo "repo not found"; exit 1; }

# The last REVIEWER commit before the chunk's work. The chunk MUST NOT ADVANCE THIS.
#
# Override when the reviewer has legitimately committed since (e.g. this harness
# itself, or a session handoff) — those are not the chunk's commits:
#     F1_VERIFY_BASE=$(git rev-parse HEAD) docs/internal/verify/verify-f1.sh
# Default is 8c600bf, the F1 brief-corrections commit.
BASE="${F1_VERIFY_BASE:-8c600bfab987ce2c538f3d5bdc49922f56f8ca21}"
BASE_SHORT="$(git rev-parse --short "$BASE" 2>/dev/null || echo "${BASE:0:7}")"

T=/tmp/f1-verify
rm -rf "$T"; mkdir -p "$T"

fail=0
hdr()  { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }
ok()   { printf '  \033[32mPASS\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; fail=1; }
warn() { printf '  \033[33mNOTE\033[0m %s\n' "$1"; }

# head_of FILE -> the file's content at the F1 brief commit, or empty if it did not exist
head_of() { git show "$BASE:$1" 2>/dev/null; }

printf '\033[1mF1 INDEPENDENT VERIFICATION\033[0m  (%s)\n' "$(date '+%F %T')"
printf 'baseline commit: %s  — every "vs HEAD" below means vs THIS commit\n' "$BASE_SHORT"

# ══════════════════════════════════════════════════════════════════════════════════
hdr "0. NOTHING WAS COMMITTED, and the tree PRODUCED something"
# ══════════════════════════════════════════════════════════════════════════════════
# Two failures that look opposite and are the same failure: a chunk that committed
# (the brief says "Do not commit. Do not push.") and a chunk that produced nothing.

branch=$(git branch --show-current)
[ "$branch" = "shulib-v2" ] && ok "on shulib-v2" || bad "wrong branch: $branch"

now=$(git rev-parse HEAD)
if [ "$now" = "$BASE" ]; then
  ok "HEAD is still the F1 brief commit ($BASE_SHORT) — nothing was committed"
else
  bad "HEAD MOVED: $now"
  echo "     commits F1 was forbidden to make:"
  git log --oneline "$BASE..HEAD" 2>/dev/null | sed 's/^/       /'
fi
warn "HEAD: $(git log -1 --pretty='%h %s')"

if [ -n "$(git status --porcelain)" ]; then
  ok "tree is dirty (expected — a clean tree after a chunk means it produced nothing)"
else
  bad "TREE IS CLEAN — F1 produced nothing, or its work was committed/stashed away"
fi
printf '  working-tree changes (%s files):\n' "$(git status --porcelain | wc -l)"
git status --porcelain | sed 's/^/    /'

# a chunk that ships a seam ships tests for it
newcases=$( { git diff "$BASE" -- test/ ; git status --porcelain test/ | awk '/^\?\?/{print $2}' \
              | xargs -r cat | sed 's/^/+/'; } 2>/dev/null | grep -c '^+.*TEST_CASE' )
if [ "$newcases" -gt 0 ]; then
  ok "$newcases new TEST_CASE line(s) in test/ vs $BASE_SHORT"
else
  bad "ZERO new TEST_CASE lines — a new seam with no new test proves nothing"
fi

# A test file not named *_test.cpp is never globbed into the binary and never runs.
# test/CMakeLists.txt:56 -> file(GLOB TEST_SOURCES ... "*_test.cpp")
badname=0
while read -r f; do
  case "$f" in
    test/*_test.cpp|test/test_main.cpp|test/vendor/*) ;;
    test/*.cpp) echo "     $f"; badname=1 ;;
  esac
done < <(git status --porcelain test/ | awk '{print $NF}'; git diff --name-only "$BASE" -- test/)
[ "$badname" -eq 0 ] && ok "every new test/*.cpp matches the *_test.cpp glob (it will actually run)" \
                     || bad "a new test/*.cpp does NOT match the glob — it is compiled by nothing"

# ══════════════════════════════════════════════════════════════════════════════════
hdr "1. Rebuild + full suite — real counts"
# ══════════════════════════════════════════════════════════════════════════════════
# The doc gates run as part of this build (test/CMakeLists.txt); they are ALSO run
# standalone in section 4 so a failure names itself instead of being "build failed".

if cmake --build build/test -j"$(nproc)" >"$T/build.log" 2>&1; then
  ok "build clean under -Werror"
else
  bad "BUILD FAILED"
  tail -40 "$T/build.log" | sed 's/^/      /'
fi
warnings=$(grep -c 'warning:' "$T/build.log")
[ "$warnings" -eq 0 ] && ok "zero compiler warnings" || warn "$warnings warning line(s) in the build log"

./build/test/shulib_tests >"$T/tests.log" 2>&1
rc=$?
cases=$(grep -m1 'test cases:' "$T/tests.log" | tr -s ' ')
asserts=$(grep -m1 'assertions:' "$T/tests.log" | tr -s ' ')
printf '    %s\n    %s\n' "${cases:-<no test-case line>}" "${asserts:-<no assertion line>}"
if [ $rc -eq 0 ]; then
  ok "suite GREEN — $(echo "$cases" | sed 's/.*test cases: //')"
else
  bad "suite RED (exit $rc)"
  grep -E 'ERROR|FAILED|FAILURE' "$T/tests.log" | head -20 | sed 's/^/      /'
fi
# Real counts, stated: a report that says "all green" without numbers is unfalsifiable.
nfailed=$(echo "$cases" | sed -n 's/.*| \([0-9]*\) failed.*/\1/p')
[ "${nfailed:-1}" = "0" ] && ok "0 failed test cases (counted, not assumed)" \
                          || bad "failed test cases: ${nfailed:-unknown}"

# ══════════════════════════════════════════════════════════════════════════════════
hdr "2. BOTH CI GUARDS — re-derived exactly as .github/workflows/ci.yml spells them"
# ══════════════════════════════════════════════════════════════════════════════════
# GUARD 1 (PROS-free): scope = ALL of include/shulib (broadened at the C7 cutover).
# GUARD 2 (sim layering, chunk A2 constraint 3): scope = all of include/shulib EXCEPT
#   sim/ itself, via --exclude-dir, so a NEW CORE DIRECTORY IS COVERED THE MOMENT IT
#   EXISTS. That matters more than usual for F1: it creates manipulation/ and/or new
#   hal/ headers, and an enumerated guard would have missed them.

if grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib >"$T/pros.log" 2>&1; then
  bad "GUARD 1 — <pros/> leaked into the library"
  sed 's/^/      /' "$T/pros.log"
else
  ok "GUARD 1 — include/shulib is PROS-free (all of it)"
fi

if grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' \
     include/shulib >"$T/simguard.log" 2>&1; then
  bad "GUARD 2 — the core includes shulib/sim (ground truth reachable from an estimator)"
  sed 's/^/      /' "$T/simguard.log"
else
  ok "GUARD 2 — layering holds: core is sim-free"
fi

# F1 adds hostile fakes. They belong under sim/ (or test/), NEVER on a path the core
# can reach — this is the same property GUARD 2 defends, checked from the other side.
host=$(git status --porcelain | awk '{print $NF}' | grep -E 'hostile' || true)
[ -n "$host" ] && { echo "    hostile-fake files touched:"; echo "$host" | sed 's/^/      /'; }

# ══════════════════════════════════════════════════════════════════════════════════
hdr "3. ARM cross-compile gate — EVERY header, generated list (never hard-coded)"
# ══════════════════════════════════════════════════════════════════════════════════
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort \
  | awk '{print "#include \""$0"\""}' > "$T/all_headers.cpp"
echo 'int main() { return 0; }' >> "$T/all_headers.cpp"
n=$(grep -c '#include' "$T/all_headers.cpp")
if arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
     -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
     -c "$T/all_headers.cpp" -o /dev/null -Iinclude 2>"$T/arm.log"; then
  ok "ARM gate CLEAN — $n headers cross-compiled for the V5 (Cortex-A9, softfp)"
else
  bad "ARM gate FAILED"
  tail -25 "$T/arm.log" | sed 's/^/      /'
fi

# Every header F1 added must be IN that TU. The find-based list makes that automatic —
# so this check is really asking: did F1 put a header somewhere find(1) does not reach?
newhdrs=$( { git status --porcelain | awk '/\?\?/{print $2}'; git diff --name-only "$BASE"; } \
           | grep -E '^include/.*\.hpp$' | sort -u )
if [ -n "$newhdrs" ]; then
  echo "    headers F1 added or touched:"
  miss=0
  while read -r h; do
    rel=${h#include/}
    if grep -qF "\"$rel\"" "$T/all_headers.cpp"; then
      printf '      \033[32min ARM TU\033[0m  %s\n' "$rel"
    else
      printf '      \033[31mNOT IN TU\033[0m %s\n' "$rel"; miss=1
    fi
  done <<< "$newhdrs"
  [ "$miss" -eq 0 ] && ok "every new/touched header is inside the ARM TU" \
                    || bad "a header escaped the ARM gate — it is outside include/shulib/**/*.hpp"
else
  warn "no new/changed headers under include/ — F1 has not created its seam yet"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "4. ALL FOUR DOC GATES, run standalone (+ the tool's own self-test)"
# ══════════════════════════════════════════════════════════════════════════════════
# coverage / freshness / verbatim-examples / removability. Run explicitly so a failure
# names itself rather than surfacing 300 lines into a compile.
for c in self-test check-coverage check-fresh check-examples check-removability; do
  if python3 tools/api_doc_tool.py "$c" >"$T/$c.log" 2>&1; then
    ok "api_doc_tool $c"
    head -2 "$T/$c.log" | sed 's/^/        /'
  else
    bad "api_doc_tool $c FAILED"
    head -20 "$T/$c.log" | sed 's/^/        /'
  fi
done

# D3's lesson: A GATE'S EXCLUSION LIST IS WHERE ITS HOLES LIVE. The coverage gate
# covers only the two frozen surfaces (tools/api_doc_tool.py TARGETS), so an
# undocumented public member in F1's NEW header fails nothing. The brief requires
# that be RULED, not defaulted.
targets=$(grep -c '"header":' tools/api_doc_tool.py)
echo "    api_doc_tool TARGETS entries: $targets"
grep -n '"header":' tools/api_doc_tool.py | sed 's/^/      /'
if [ "$targets" -gt 2 ]; then
  ok "F1 added its header to TARGETS — the new surface is coverage-gated"
else
  warn "TARGETS still lists only the two frozen surfaces — LEGAL, but the brief requires"
  warn "  the choice to be made deliberately and recorded. Check F1-COMPLETED.md says so."
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "5. REMOVABILITY — no public doc may link into docs/internal/ (C7/C8)"
# ══════════════════════════════════════════════════════════════════════════════════
# The narrow FOUR-TERM grep is the true gate. A hit that already existed at the
# baseline is NOT F1's, and the README's caution is explicit that the harness once
# got this exactly wrong. Each hit below is triaged against $BASE_SHORT.
pubfiles=""
for f in README.md test/README.md docs/*.md docs/guide/*.md docs/cookbook/*.md docs/api/*.md; do
  [ -f "$f" ] && pubfiles="$pubfiles $f"
done
# shellcheck disable=SC2086
grep -rnE 'internal/|chunks/|RESUMING|build-order' $pubfiles > "$T/remov.log" 2>/dev/null
if [ ! -s "$T/remov.log" ]; then
  ok "four-term gate EMPTY across $(echo $pubfiles | wc -w) public documents"
else
  newhit=0
  while IFS= read -r line; do
    f=${line%%:*}; rest=${line#*:}; txt=${rest#*:}
    if head_of "$f" | grep -qF -- "$txt"; then
      printf '      \033[33mPRE-EXISTING\033[0m %s\n' "$(echo "$line" | cut -c1-120)"
    else
      printf '      \033[31mNEW (F1)\033[0m     %s\n' "$(echo "$line" | cut -c1-120)"
      newhit=1
    fi
  done < "$T/remov.log"
  if [ "$newhit" -eq 1 ]; then
    bad "F1 introduced a public->internal reference; the squash-merge to main would break"
  else
    warn "all four-term hits predate F1 — a release-review nit, NOT an F1 failure (README's caution)"
    ok "F1 introduced no new public->internal reference"
  fi
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "6. THE F6 AND F10 SIGNATURE PINS"
# ══════════════════════════════════════════════════════════════════════════════════
# Compiled standalone, then BITTEN. "The pin compiles" only proves the surface did not
# move; it does not prove the pin can still see a move. Both bites use a SHADOW header
# in /tmp on an earlier -I path — include/ is never touched.
PINFLAGS=(-std=c++20 -fsyntax-only -Iinclude -Itest -isystem test/vendor)

for pin in test/f6_signature_pin_test.cpp test/routine_signature_pin_test.cpp; do
  if g++ "${PINFLAGS[@]}" "$pin" 2>"$T/$(basename "$pin").log"; then
    ok "$(basename "$pin") compiles — the frozen surface is intact"
  else
    bad "$(basename "$pin") FAILED — a frozen signature moved"
    grep -m3 -E 'F6 FREEZE VIOLATION|F10 FREEZE VIOLATION|error:' "$T/$(basename "$pin").log" \
      | cut -c1-200 | sed 's/^/        /'
  fi
done

mkshadow() {  # mkshadow REL_HEADER  -> copies the live header into $T/shadow/REL
  local rel="$1"
  mkdir -p "$T/shadow/$(dirname "$rel")"
  cp "include/$rel" "$T/shadow/$rel"
}
rm -rf "$T/shadow"

# F10 bite: drop noexcept from Routine::ok(). D2's campaign hole #1 — for a
# NON-overloaded member the compiler accepts a static_cast that ADDS noexcept, so a
# cast-only pin cannot see noexcept being DROPPED. D3 closed it with a compound
# requirement. This confirms it is still closed.
mkshadow shulib/chassis/routine.hpp
python3 - "$T/shadow/shulib/chassis/routine.hpp" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
a = "bool ok() const noexcept { return stoppedAt_ == 0; }"
b = "bool ok() const { return stoppedAt_ == 0; }"
sys.exit(0 if (s.count(a) == 1 and open(p, 'w').write(s.replace(a, b, 1))) else 3)
PY
if [ $? -ne 0 ]; then
  bad "F10 BITE not run — Routine::ok()'s spelling moved; fix the pattern in this script"
elif g++ "${PINFLAGS[@]}" -I"$T/shadow" test/routine_signature_pin_test.cpp 2>"$T/f10bite.log"; then
  bad "F10 BITE: a noexcept DROP on Routine::ok() COMPILED — D2's hole #1 is re-opened"
else
  if grep -q 'F10 FREEZE VIOLATION' "$T/f10bite.log"; then
    ok "F10 pin bites and NAMES the freeze (noexcept drop caught)"
  else
    bad "F10 pin failed the build but never said 'F10 FREEZE VIOLATION' — a nameless red"
    head -6 "$T/f10bite.log" | cut -c1-160 | sed 's/^/        /'
  fi
fi

# F6 bite: drop noexcept from Chassis::motionConfig().
rm -rf "$T/shadow"; mkshadow shulib/chassis/chassis.hpp
python3 - "$T/shadow/shulib/chassis/chassis.hpp" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
a = "const motion::MotionConfig& motionConfig() const noexcept { return cfg_; }"
b = "const motion::MotionConfig& motionConfig() const { return cfg_; }"
sys.exit(0 if (s.count(a) == 1 and open(p, 'w').write(s.replace(a, b, 1))) else 3)
PY
if [ $? -ne 0 ]; then
  bad "F6 BITE not run — Chassis::motionConfig()'s spelling moved; fix the pattern here"
elif g++ "${PINFLAGS[@]}" -I"$T/shadow" test/f6_signature_pin_test.cpp 2>"$T/f6bite.log"; then
  bad "F6 BITE: a noexcept DROP on Chassis::motionConfig() COMPILED — the F6 pin is decoration"
else
  grep -q 'F6 FREEZE VIOLATION' "$T/f6bite.log" \
    && ok "F6 pin bites and NAMES the freeze" \
    || { bad "F6 pin failed but never named F6"; head -6 "$T/f6bite.log" | cut -c1-160 | sed 's/^/        /'; }
fi
rm -rf "$T/shadow"

# ══════════════════════════════════════════════════════════════════════════════════
hdr "7. routine.hpp — ONLY then() MAY HAVE MOVED (member-by-member diff vs $BASE_SHORT)"
# ══════════════════════════════════════════════════════════════════════════════════
# then() is THE ONE MEMBER excluded from F10's lock. Everything else in that file is
# pinned by 37 compile-time pins, and the brief is explicit: "if a pin fires for
# anything except then(), stop and report it."
#
# Section 6 proves the SIGNATURES did not move. This section is stricter: it compares
# each member's WHOLE BLOCK (declaration + body + its /// docs), so a behaviour change
# that keeps the signature — which no pin can see — is reported too.
head_of include/shulib/chassis/routine.hpp > "$T/routine.head.hpp"
if [ ! -s "$T/routine.head.hpp" ]; then
  bad "could not read routine.hpp at $BASE_SHORT"
else
  python3 - "$T/routine.head.hpp" include/shulib/chassis/routine.hpp <<'PY'
import re, sys

PINNED = ["startAt", "moveTo", "driveTo", "strafeTo", "turnTo", "face",
          "followTrajectory", "brake", "hold", "pause", "waitFor",
          "ok", "result", "lastTrajectory", "chassis"]
UNFROZEN = ["then"]
ALL = PINNED + UNFROZEN

def blocks(text):
    """member name -> concatenated text of every block (all overloads), plus the
    leading /// doc comment and any template<> line."""
    lines = text.split('\n')
    out = {n: [] for n in ALL}
    # EXACTLY four spaces: class scope. `^\s{4}` would also match an 8-space body line
    # (\s{4} is "four whitespace chars", not "four and no more"), and then the brace
    # walk starts mid-body and swallows the NEXT member — which reports a member as
    # MOVED that never changed. Caught by bite-testing this parser against a planted
    # edit before the script was ever run for real.
    pat = re.compile(r'^ {4}(?:\[\[nodiscard\]\] *)?[A-Za-z_][^;=]*?\b(' +
                     '|'.join(ALL) + r')\s*\(')
    # Match against a COMMENT-STRIPPED view. RoutineStopCause's enumerator
    #     WaitTimedOut,  ///< a waitFor() deadline passed ...
    # sits at exactly four spaces and contains "waitFor(" IN ITS COMMENT, so the raw
    # line matches the declaration pattern and the enum body gets attributed to
    # waitFor — reporting a pinned member as MOVED whenever the enum is appended to,
    # which is a legal, expected F1 edit. Also caught by bite-testing this parser.
    strip = lambda s: re.sub(r'//.*$', '', s)
    i = 0
    while i < len(lines):
        code = strip(lines[i])
        m = pat.match(code)
        if not m or '(' not in code:
            i += 1
            continue
        name = m.group(1)
        # walk back over the doc comment / template<> header
        s = i
        while s > 0 and (lines[s-1].lstrip().startswith('///') or
                         lines[s-1].lstrip().startswith('template') or
                         lines[s-1].lstrip().startswith('//')):
            s -= 1
        # walk forward, brace-counting, to the end of the body
        depth, j, started = 0, i, False
        while j < len(lines):
            c = strip(lines[j])          # a brace inside a // comment is not a brace
            depth += c.count('{') - c.count('}')
            if '{' in c:
                started = True
            if started and depth <= 0:
                break
            j += 1
        out[name].append('\n'.join(lines[s:j+1]))
        i = j + 1
    return {k: '\n'.join(v) for k, v in out.items()}

def enum_values(text, name):
    m = re.search(r'enum class ' + name + r'\s*\{(.*?)\};', text, re.S)
    if not m:
        return None
    vals = []
    for line in m.group(1).split('\n'):
        line = line.split('///')[0].strip().rstrip(',')
        if line and not line.startswith('//'):
            vals.append(line)
    return vals

def struct_fields(text, name):
    m = re.search(r'struct ' + name + r'\s*\{(.*?)\n\};', text, re.S)
    if not m:
        return None
    return [l.split('///')[0].strip() for l in m.group(1).split('\n')
            if l.strip() and not l.strip().startswith('//') and ';' in l]

old_t, new_t = open(sys.argv[1]).read(), open(sys.argv[2]).read()
old, new = blocks(old_t), blocks(new_t)

moved, missing = [], []
for name in ALL:
    o, n = old.get(name, ''), new.get(name, '')
    if not o:
        missing.append(name)
    elif o != n:
        moved.append(name)

print("  member-block comparison (declaration + body + /// docs):")
for name in ALL:
    tag = "then()  [UNFROZEN — the one member F1 may change]" if name == "then" else ""
    state = "MOVED " if name in moved else "same  "
    color = "\033[33m" if (name in moved and name == "then") else \
            ("\033[31m" if name in moved else "\033[32m")
    print(f"    {color}{state}\033[0m {name:<18} {tag}")

if missing:
    print(f"    *** could not locate at baseline: {', '.join(missing)} — fix this script's parser")

bad_moves = [m for m in moved if m in PINNED]

# RoutineStopCause is append-only: the baseline list must be a PREFIX of the new one.
oc, nc = enum_values(old_t, 'RoutineStopCause'), enum_values(new_t, 'RoutineStopCause')
if oc is None or nc is None:
    print("    *** RoutineStopCause not parseable — check by hand")
    bad_moves.append('RoutineStopCause(parse)')
elif nc[:len(oc)] != oc:
    print(f"    *** RoutineStopCause IS NOT APPEND-ONLY\n        was: {oc}\n        now: {nc}")
    bad_moves.append('RoutineStopCause')
elif len(nc) > len(oc):
    print(f"    RoutineStopCause APPENDED (legal, anticipated by routine.hpp:145-146): "
          f"{nc[len(oc):]}")
else:
    print("    RoutineStopCause unchanged")

of, nf = struct_fields(old_t, 'RoutineResult'), struct_fields(new_t, 'RoutineResult')
if of is None or nf is None:
    print("    *** RoutineResult not parseable — check by hand")
elif nf[:len(of)] != of:
    print(f"    *** RoutineResult's EIGHT FROZEN FIELDS CHANGED\n        was: {of}\n        now: {nf}")
    bad_moves.append('RoutineResult')
elif len(nf) > len(of):
    print(f"    RoutineResult gained field(s) (additive, legal): {nf[len(of):]}")
else:
    print("    RoutineResult unchanged")

if bad_moves:
    print(f"  *** PINNED MEMBERS MOVED: {', '.join(bad_moves)}")
    sys.exit(1)
sys.exit(0)
PY
  if [ $? -eq 0 ]; then
    ok "no pinned member of routine.hpp moved — only then() (if anything) did"
  else
    bad "a PINNED member of routine.hpp changed — stop and report it (brief T5)"
    echo "      full diff of the file:"
    git diff "$BASE" -- include/shulib/chassis/routine.hpp | sed 's/^/        /' | head -80
  fi
  if git diff --quiet "$BASE" -- include/shulib/chassis/routine.hpp; then
    warn "routine.hpp is byte-identical to $BASE_SHORT — T5 ruled 'fix the prose', or is unruled"
  else
    warn "routine.hpp changed; the full diff (read it — 'unchanged in meaning' is a human call):"
    git diff --stat "$BASE" -- include/shulib/chassis/routine.hpp | sed 's/^/        /'
  fi
fi

# chassis.hpp is fully frozen (F6). F1 has NO licence to touch it at all.
if git diff --quiet "$BASE" -- include/shulib/chassis/chassis.hpp; then
  ok "chassis.hpp untouched (F6 is fully frozen — F1 has no unfrozen member there)"
else
  bad "chassis.hpp CHANGED — every member is pinned by F6; this is a breaking change to argue"
  git diff "$BASE" -- include/shulib/chassis/chassis.hpp | sed 's/^/        /' | head -60
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "8. FREEZE REGISTER — rows F1–F5 unedited, and F1 FROZE NOTHING"
# ══════════════════════════════════════════════════════════════════════════════════
# THE NAME COLLISION IS LIVE: register row F1 is the COORDINATE FRAME (locked
# 2026-06-08), not this chunk. Rows F1-F5 are locked contracts about frames, units,
# accuracy, HAL signatures and kinematics and have NOTHING to do with the mechanism
# seam. Row F4 is the one exception the brief carves out (T7's freeze note) and it is
# an ADDITIVE one: F1 may state in F4 that a mechanism seam exists and is OUTSIDE F4 —
# it may not amend what F4 freezes, its date, or its status.
head_of docs/roadmap.md > "$T/roadmap.head.md"
python3 - "$T/roadmap.head.md" docs/roadmap.md <<'PY'
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

def cells(row):
    c = [x.strip() for x in row.split('|')]
    return c[1:-1] if len(c) > 2 else c

print("  register rows, baseline -> now:")
for key in sorted(set(old) | set(new), key=lambda k: int(k[1:])):
    o, n = old.get(key), new.get(key)
    if o is None:
        state, note = "ADDED", ""
        if 'LOCKED' in n:
            note = "  *** A NEW ROW CLAIMING LOCKED — F1 FREEZES NOTHING"
            problems.append(f"{key}: new row claims LOCKED")
        print(f"    \033[33m{state:<9}\033[0m {key}{note}")
        print(f"              {n[:150]}")
        continue
    if n is None:
        print(f"    \033[31m{'DELETED':<9}\033[0m {key}")
        problems.append(f"{key}: row deleted")
        continue
    if o == n:
        print(f"    \033[32m{'same':<9}\033[0m {key}")
        continue

    oc, nc = cells(o), cells(n)
    if key == 'F4':
        # the one row T7 licenses — additive only
        addl = len(oc) == len(nc)
        keeps = addl and oc[2:] == nc[2:]          # depends-on / frozen-at / status
        grows = addl and nc[1].startswith(oc[1])   # contract text only APPENDED to
        if keeps and grows:
            print(f"    \033[33m{'APPENDED':<9}\033[0m F4 (T7's freeze note — legal)")
            print(f"              + {nc[1][len(oc[1]):][:150]}")
        else:
            print(f"    \033[31m{'REWRITTEN':<9}\033[0m F4 — NOT a pure append")
            print(f"              was: {o[:150]}\n              now: {n[:150]}")
            problems.append("F4: rewritten, not appended to (the HAL freeze was amended)")
    elif key in ('F1', 'F2', 'F3', 'F5'):
        print(f"    \033[31m{'EDITED':<9}\033[0m {key} — FORBIDDEN (locked contract, "
              f"and 'F1' here is the COORDINATE FRAME, not the chunk)")
        print(f"              was: {o[:150]}\n              now: {n[:150]}")
        problems.append(f"{key}: locked row edited")
    else:
        print(f"    \033[33m{'EDITED':<9}\033[0m {key}")
        print(f"              was: {o[:150]}\n              now: {n[:150]}")

# F1 FROZE NOTHING: the set of rows claiming LOCKED must be identical.
ol = {k for k, v in old.items() if 'LOCKED' in v}
nl = {k for k, v in new.items() if 'LOCKED' in v}
print(f"\n  rows marked LOCKED: was {len(ol)} {sorted(ol)}  ->  now {len(nl)} {sorted(nl)}")
if nl != ol:
    print(f"    *** THE LOCK SET CHANGED: gained {sorted(nl-ol)}, lost {sorted(ol-nl)}")
    print("        F1 FREEZES NOTHING (brief, Freeze note). The seam gets its second")
    print("        consumer at F3, on hardware; build -> second consumer -> freeze.")
    problems.append("the LOCKED set changed")
else:
    print("    unchanged — consistent with 'F1 freezes nothing'")

# ...but silence in a freeze register reads as "frozen" (D2's lesson, and the brief
# calls this note MANDATORY). Something must SAY the seam exists and is outside F4.
touched = [k for k in new if old.get(k) != new.get(k)]
said = any(re.search(r'mechanism seam|Mechanism seam|IMechanism|chunk F1|outside F4|not part of F4',
                     new[k]) for k in touched) if touched else False
if said:
    print("  the register STATES the seam's existence + non-freeze (T7 satisfied)")
else:
    print("  *** NO REGISTER ROW STATES THAT A MECHANISM SEAM NOW EXISTS AND IS UNFROZEN.")
    print("      T7's freeze note is MANDATORY: 'silence in a freeze register reads as")
    print("      frozen' — exactly the omission D2 found and fixed for Routine.")
    problems.append("no freeze note for the mechanism seam")

sys.exit(1 if problems else 0)
PY
[ $? -eq 0 ] && ok "register: F1-F5 intact, nothing newly locked, the freeze note is present" \
             || bad "Freeze Register problem — see the rows above (and re-read the brief's §'F1 means two things')"

# a lock claimed anywhere else is still a lock claimed
if git diff "$BASE" -- docs/roadmap.md | grep -E '^\+' | grep -qiE 'LOCKED 2026-08-13|frozen at F1|locked at F1'; then
  bad "the roadmap diff claims a NEW lock dated today — F1 freezes nothing"
  git diff "$BASE" -- docs/roadmap.md | grep -E '^\+' | grep -iE 'LOCKED|frozen' | sed 's/^/        /'
else
  ok "no new 'LOCKED' claim in the roadmap diff"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "9. THE 'NEVER DRIVEN A ROBOT' CLAIMS MUST BE INTACT"
# ══════════════════════════════════════════════════════════════════════════════════
# "Do not let the seam's arrival inflate any claim. Nothing here touches accuracy, and
# nothing here has met a robot." A seam that can command a FAKE intake on a HOST is
# not a robot that can score.
claim() {  # claim FILE "LITERAL"
  if grep -qF -- "$2" "$1"; then
    ok "$(basename "$1"): $(echo "$2" | cut -c1-72)"
  else
    bad "$(basename "$1"): MISSING/WEAKENED -> $(echo "$2" | cut -c1-72)"
    # triage: was it ever there?
    head_of "$1" | grep -qF -- "$2" \
      && warn "  ...it WAS present at $BASE_SHORT — F1 removed or reworded it" \
      || warn "  ...it was NOT present at $BASE_SHORT either — this script's literal is stale"
  fi
}
claim README.md "**This library has never driven a robot.**"
claim README.md "It drove nothing, because it cannot."
grep -qi "hardware adapters" README.md && ok "README.md: the 'hardware adapters do not exist' bullet survives" \
                                       || bad "README.md: the 'hardware adapters' bullet is gone"
claim docs/guide/14-what-it-cannot-do-yet.md "## It has never driven a robot"
claim docs/guide/14-what-it-cannot-do-yet.md \
  "**No shulib code has ever controlled a motor or read a real sensor.**"
claim docs/guide/14-what-it-cannot-do-yet.md "Booting is not driving"
claim docs/guide/14-what-it-cannot-do-yet.md "**Every physical constant is a labeled guess.**"
# The < 1 degree sentence stays exactly as it is (brief, Documentation section).
grep -qF 'does **not** claim the `< 1°` requirement is met' docs/guide/14-what-it-cannot-do-yet.md \
  && ok "ch14 still declines to claim the < 1° requirement is met" \
  || bad "ch14's '< 1°' disclaimer moved — the brief says it stays EXACTLY as it is"

# Chapter 14 MUST change (two sections in it are now wrong) — but only in the places
# the brief names. Report the diff so a person can read it.
if git diff --quiet "$BASE" -- docs/guide/14-what-it-cannot-do-yet.md; then
  bad "ch14 is UNCHANGED — 'The mechanism seam is a placeholder' (l.167-184) and 'No mechanisms"
  bad "  for recipes to command' (l.191+) are now wrong or partly wrong; the brief requires a rewrite"
else
  ok "ch14 was rewritten (required)"
  git diff --stat "$BASE" -- docs/guide/14-what-it-cannot-do-yet.md | sed 's/^/        /'
fi
# and it must not have grown a claim it cannot support
if git diff "$BASE" -- README.md docs/guide/14-what-it-cannot-do-yet.md docs/guide/09-the-recipe-api.md \
     | grep -E '^\+' \
     | grep -inE 'can now score|now scores|proven on a robot|tested on hardware|verified on hardware|drives a real|a real intake' \
     >"$T/inflate.log" 2>&1; then
  bad "a claim looks INFLATED in the public docs (a PROXY — read each hit, then judge):"
  sed 's/^/        /' "$T/inflate.log"
else
  ok "no scoring/hardware claim added to README, ch09 or ch14"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr '10. T5 — the `intake.in` claim must be TRUE-AND-COMPILED or GONE EVERYWHERE'
# ══════════════════════════════════════════════════════════════════════════════════
# The brief: "Do not leave it half-done." The check-examples gate scans only fenced
# ```cpp blocks (api_doc_tool.py:780) — every occurrence of this claim today is
# inline-backtick prose, WHICH NO COMPILER HAS EVER SEEN. So: either the bare-member
# spelling is gone from public prose, or it exists as a FENCED example (and is then
# machine-checked forever).
#
# NOTE THE PROXY: this grep cannot tell a corrected sentence from a deleted one. It
# tells you where the claim still lives; you decide whether each survivor is honest.
pub_claims=$(grep -rn 'then(intake\.in)\|then(intake\.in[^(]\|`intake\.in`\|intake\.release[^(]' \
  README.md docs/*.md docs/guide/*.md docs/cookbook/*.md docs/api/*.md \
  include/shulib/chassis/routine.hpp 2>/dev/null | grep -v 'intake\.in()' || true)
fenced=$(python3 - <<'PY'
import glob, re
hit = []
for p in glob.glob('docs/**/*.md', recursive=True) + ['README.md']:
    if '/internal/' in p:
        continue
    try:
        t = open(p).read()
    except OSError:
        continue
    for b in re.findall(r'```cpp\n(.*?)```', t, re.S):
        if re.search(r'\.then\(\s*[A-Za-z_][A-Za-z_0-9]*\.[A-Za-z_][A-Za-z_0-9]*\s*[,)]', b):
            hit.append(p)
print('\n'.join(sorted(set(hit))))
PY
)
if [ -n "$fenced" ]; then
  ok "the member-callable spelling now appears in a FENCED cpp block (compiled by check-examples):"
  echo "$fenced" | sed 's/^/        /'
elif [ -z "$pub_claims" ]; then
  ok 'no bare `intake.in` claim survives in public prose — T5 ruled "fix the prose everywhere"'
else
  bad "the flagship claim still lives as UNCOMPILED prose in $(echo "$pub_claims" | wc -l) place(s)"
  bad "  and no fenced cpp example proves it. That is 'half-done' — the exact state T5 forbids."
  echo "$pub_claims" | cut -c1-150 | sed 's/^/        /'
fi

# The cookbook's hand-written Intake struct is quoted VERBATIM from the test; the two
# are bound by check-examples. If the seam contradicts "exactly the shape a real
# mechanism will" take, one of the two must have changed.
if grep -q 'exactly the shape a real mechanism will' docs/cookbook/README.md docs/cookbook/01-getting-there.md 2>/dev/null; then
  warn "the cookbook still claims the hand-written Intake is 'exactly the shape a real mechanism"
  warn "  will' take. With a seam now in the tree that is a FALSIFIABLE claim — read the struct"
  warn "  (test/cookbook_examples_test.cpp:84-97) against hal::IMechanism and judge it."
else
  ok "the cookbook's 'exactly the shape' claim was revised alongside the seam"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "11. FAULT CODES — append-only, individually pinned, and not minted for verdicts"
# ══════════════════════════════════════════════════════════════════════════════════
# T6: a FAULT says the robot is unwell; a VERDICT says the task did not happen. Minting
# codes for verdicts floods the latch and destroys first-fault triage — the entire
# reason the latch exists. Mechanically checkable: the numbers, the pins, the width.
python3 - <<'PY'
import re, subprocess, sys
BASE = "8c600bfab987ce2c538f3d5bdc49922f56f8ca21"
def read(p, rev=None):
    if rev:
        return subprocess.run(['git', 'show', f'{rev}:{p}'], capture_output=True,
                              text=True).stdout
    return open(p).read()

def codes(t):
    m = re.search(r'enum class FaultCode\s*:\s*std::uint16_t\s*\{(.*?)\n\};', t, re.S)
    out = {}
    if not m:
        return out
    for line in m.group(1).split('\n'):
        mm = re.match(r'\s*([A-Za-z_]\w*)\s*=\s*(\d+)', line)
        if mm:
            out[mm.group(1)] = int(mm.group(2))
    return out

P = 'include/shulib/diag/fault.hpp'
old, new = codes(read(P, BASE)), codes(read(P))
problems = []
for k, v in old.items():
    if k not in new:
        print(f"    *** {k} = {v} WAS DELETED — the enum is wire-stable/append-only"); problems.append(k)
    elif new[k] != v:
        print(f"    *** {k} RENUMBERED {v} -> {new[k]} — E1 blackbox files already carry {v}")
        problems.append(k)
added = {k: v for k, v in new.items() if k not in old}
print(f"    baseline codes: {len(old)} (last value {max(old.values()) if old else '-'})")
if not added:
    print("    F1 minted NO new fault code.")
    print("      That is a DEFENSIBLE ruling (T6: jam/stall may be verdicts, not pathologies)")
    print("      but the DoD says 'failure modes surface as fault codes, not hangs' — so the")
    print("      record must argue it. NOT a mechanical failure; a question for the reader.")
else:
    tst = read('test/fault_test.cpp')
    names = read(P)
    for k, v in sorted(added.items(), key=lambda kv: kv[1]):
        pinned = re.search(r'FaultCode::' + k + r'\)\s*==\s*' + str(v), tst) is not None
        spelled = re.search(r'case FaultCode::' + k + r'\s*:', names) is not None
        print(f"    APPENDED {k} = {v}   pin:{'yes' if pinned else '*** NO ***'}   "
              f"faultCodeName:{'yes' if spelled else '*** NO ***'}")
        if not pinned:
            problems.append(f'{k} unpinned'); print(f"        a code with no individual pin in "
                  f"test/fault_test.cpp is a code nothing keeps stable")
        if not spelled:
            problems.append(f'{k} unspelled'); print("        faultCodeName would render it UNKNOWN")
        if v > 31:
            problems.append(f'{k} out of mask range')
            print(f"        *** {v} > 31: abortFaultMask is a uint32_t BIT-INDEXED BY VALUE "
                  f"(motion_scheduler.hpp:945-959) and the latch has 32 tally slots")
        else:
            print(f"        mask bit {v} fits; default mask is ODO_STUCK only, so this code lands")
            print( "        on the CONTINUE-DEGRADED side (a jammed intake must not abort a drive)")
sys.exit(1 if problems else 0)
PY
[ $? -eq 0 ] && ok "FaultCode is append-only and every new code is pinned + spelled" \
             || bad "FaultCode problem — see above (E1's exact hole class: a code nothing keeps stable)"

# TickPhase::User is RESERVED FOR MECHANISMS and has no producer. E1's lesson: a field
# nothing fills is worse than a field that does not exist, because nothing looks wrong.
if git diff "$BASE" -- include/shulib/diag/debug_record.hpp | grep -qE '^[-+].*User\s*='; then
  bad "TickPhase::User's numeric value moved — it is in the F9 wire schema"
else
  ok "TickPhase::User = 5 unchanged (wire schema intact)"
fi
producers=$(grep -rln 'TickPhase::User' include/shulib src 2>/dev/null | grep -v debug_record.hpp || true)
if [ -n "$producers" ]; then
  ok "TickPhase::User now has a producer:"; echo "$producers" | sed 's/^/        /'
else
  warn "TickPhase::User still has NO producer. Legal only if F1-COMPLETED.md records WHY —"
  warn "  E1's lesson (DebugRecord::fault rendered by TermSink and documented in ch.11 while"
  warn "  nothing ever filled it) applies directly. Check the record says so."
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "12. THINGS F1 IS FORBIDDEN TO BUILD (scope creep + the standing decisions)"
# ══════════════════════════════════════════════════════════════════════════════════
newsrc=$( { git status --porcelain | awk '/\?\?/{print $2}'; git diff --name-only "$BASE"; } \
          | grep -E '^(include|src)/.*\.(hpp|cpp)$' | sort -u )

# (a) No game semantics in hal/. The house rule is hal/vision.hpp:40 — classId is an
#     OPAQUE INTEGER, deliberately not `Cup` or `Yellow`. The HAL reports what the
#     device saw; meaning is assigned above it.
sem=$(grep -rnwiE 'cup|ring|donut|mogo|goal|stake|yellow|blue|red|hue|capture|score|toggle|quadrant' \
        include/shulib/hal 2>/dev/null | grep -viE '^\S+: *(//|///|\*)' || true)
if [ -n "$sem" ]; then
  bad "possible GAME SEMANTICS in hal/ — this season would be baked into the HAL:"
  echo "$sem" | cut -c1-140 | sed 's/^/        /'
  warn "  A PROXY, not the property: 'red'/'blue' can appear innocently (alliance-free words,"
  warn "  a comment, a colour-agnostic id). Read each hit before calling it a violation."
else
  ok "no game-semantic vocabulary in include/shulib/hal/"
fi

# (b) No mechanism owns a loop, a task, or a clock. T3, and the standing
#     no-background-task decision (E1 T1). Every deterministic test in this project
#     depends on it.
if [ -n "$newsrc" ]; then
  loops=$(grep -nE 'std::thread|pros::Task|pthread_|while *\( *true *\)|for *\( *; *; *\)|std::this_thread|::delay\(|sleep\(' \
            $newsrc 2>/dev/null || true)
  if [ -n "$loops" ]; then
    bad "a new/changed source file owns a LOOP, TASK or SLEEP:"
    echo "$loops" | cut -c1-140 | sed 's/^/        /'
  else
    ok "no thread, task, spin-loop or sleep in the files F1 touched"
  fi

  # (c) F2's work is F2's. "If you find yourself writing Race or a match timer, you
  #     have crossed it" — scope creep into F2 is the single largest risk to F1.
  creep=$(grep -nwE 'Race|Deadline|Sequence|Parallel|parkGuard|ParkGuard|matchTimer|MatchTimer' \
            $newsrc 2>/dev/null || true)
  if [ -n "$creep" ]; then
    bad "F2 vocabulary appears in F1's files — combinators/park/match-timer are chunk F2:"
    echo "$creep" | cut -c1-140 | sed 's/^/        /'
  else
    ok "no combinator / park-guard / match-timer vocabulary — the F1|F2 line held"
  fi

  # (d) R1's work is R1's: "Author nothing that needs a brain."
  grep -qE 'pros::|<pros/' $newsrc 2>/dev/null \
    && bad "a new file references PROS — hal/pros implementations are R1" \
    || ok "nothing F1 wrote needs a brain (no PROS)"
else
  warn "no new/changed source files under include/ or src/ — nothing to scan"
fi

# (e) HA register: invented numbers start at HA-92 and are LABELED as invented.
if git diff --quiet "$BASE" -- docs/hardware-assumptions.md; then
  warn "hardware-assumptions.md UNCHANGED. T4 registers physical claims (does a Hold actually"
  warn "  hold a loaded cascade lift? what current does a jam draw?) as HA-nn from HA-92."
  warn "  Unchanged is only honest if F1 invented no threshold at all — verify by reading."
else
  ok "hardware-assumptions.md updated"
  added_ha=$(git diff "$BASE" -- docs/hardware-assumptions.md | grep -oE '^\+.*HA-[0-9]+' \
             | grep -oE 'HA-[0-9]+' | sort -u | tr '\n' ' ')
  echo "        HA ids added: ${added_ha:-none}"
  echo "$added_ha" | grep -qE 'HA-(9[2-9]|[1-9][0-9]{2})' \
    && ok "numbering starts at or after HA-92 (next free per the brief)" \
    || bad "no HA id >= 92 was added — a re-used number silently overwrites an existing claim"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "13. GUIDE VERBATIM COUPLING — re-derived, not trusted to check-examples"
# ══════════════════════════════════════════════════════════════════════════════════
# Every non-blank line inside a ```cpp block in docs/guide/*.md must appear verbatim in
# test/guide_examples_test.cpp. This is the anti-rot coupling the guide depends on and
# the check most likely to catch real drift: a rename in the test file silently
# invalidates every chapter quoting it. Chapter 13 (extending the library) is where F1
# writes "how to write a mechanism" — new prose is exactly where drift enters.
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
print(f"    checked {total} quoted lines across docs/guide/*.md")
if bad_lines:
    print(f"    *** {len(bad_lines)} NOT VERBATIM ***")
    for p, bi, s in bad_lines[:25]:
        print(f"      {p} block {bi}: {s[:110]}")
    sys.exit(1)
print("    all quoted lines appear verbatim in test/guide_examples_test.cpp")
PY
[ $? -eq 0 ] && ok "guide examples verbatim" || bad "the guide has drifted from the compiled examples"

# The cookbook has the same coupling against test/cookbook_examples_test.cpp.
python3 - <<'PY'
import re, glob, sys
src = open('test/cookbook_examples_test.cpp').read()
srcset = {l.strip() for l in src.splitlines()}
bad_lines, total = [], 0
for path in sorted(glob.glob('docs/cookbook/*.md')):
    for bi, b in enumerate(re.findall(r'```cpp\n(.*?)```', open(path).read(), re.S), 1):
        for line in b.splitlines():
            s = line.strip()
            if not s:
                continue
            total += 1
            if s not in srcset:
                bad_lines.append((path, bi, s))
print(f"    checked {total} quoted lines across docs/cookbook/*.md")
if bad_lines:
    print(f"    *** {len(bad_lines)} NOT VERBATIM ***")
    for p, bi, s in bad_lines[:25]:
        print(f"      {p} block {bi}: {s[:110]}")
    sys.exit(1)
print("    all quoted lines appear verbatim in test/cookbook_examples_test.cpp")
PY
[ $? -eq 0 ] && ok "cookbook examples verbatim" || bad "the cookbook has drifted from its compiled test"

# "Nothing above the seam changed": the existing recipe/cookbook/guide cases stay green
# AND UNEDITED. A DELETED line in one of these is where "unchanged in meaning" goes to
# die — it is reported, never auto-judged.
for f in test/chassis_recipe_test.cpp test/chassis_routine_test.cpp \
         test/cookbook_examples_test.cpp test/guide_examples_test.cpp; do
  [ -f "$f" ] || continue
  del=$(git diff "$BASE" --numstat -- "$f" | awk '{print $2}')
  add=$(git diff "$BASE" --numstat -- "$f" | awk '{print $1}')
  if [ -z "${del:-}" ] || [ "${del:-0}" = "0" ]; then
    ok "$(basename "$f"): +${add:-0} / -0 — additive only"
  else
    warn "$(basename "$f"): +${add} / -${del} — LINES WERE REMOVED. Each deletion is a"
    warn "  claim that an existing use still 'means exactly what it means today'. Read them:"
    git diff "$BASE" -- "$f" | grep -E '^-[^-]' | head -15 | cut -c1-130 | sed 's/^/        /'
  fi
done

# ══════════════════════════════════════════════════════════════════════════════════
hdr "14. DELIVERABLES"
# ══════════════════════════════════════════════════════════════════════════════════
for f in docs/internal/chunks/F1-PROGRESS.md docs/internal/chunks/F1-COMPLETED.md; do
  [ -f "$f" ] && ok "$f ($(wc -l < "$f") lines)" || bad "$f MISSING"
done
if [ -f docs/internal/chunks/F1-COMPLETED.md ]; then
  R=docs/internal/chunks/F1-COMPLETED.md
  n=$(wc -l < "$R")
  [ "$n" -ge 400 ] && ok "record depth ${n} lines" \
                   || bad "record only ${n} lines — below the bar this project holds"
  # T1, T2 and T4 must have their OWN sections (documentation contract).
  for t in T1 T2 T4; do
    grep -qE "^#{2,4} .*\b$t\b" "$R" && ok "record gives $t its own section" \
                                     || bad "record has no dedicated $t section (required)"
  done
  for t in T3 T5 T6 T7; do
    grep -qw "$t" "$R" && ok "record rules $t" || bad "record never mentions $t"
  done
  # every ruling needs a NAMED REJECTED ALTERNATIVE
  rej=$(grep -ciE 'rejected|alternative considered|the alternative' "$R")
  [ "$rej" -ge 7 ] && ok "$rej rejected-alternative mentions (>= 7 rulings)" \
                   || bad "only $rej rejected-alternative mentions — seven rulings each need one"
  # mutations: every GREEN must be logged, not quietly dropped
  grep -qi 'GREEN' "$R" && ok "record discusses mutation GREENs (holes)" \
                        || bad "record never says GREEN — D3 found 4, E1 2, E2 1, E3 3, E4 2"
  grep -qi 'freezes nothing\|froze nothing\|nothing is frozen' "$R" \
    && ok "record states plainly that F1 freezes nothing" \
    || bad "record does not say F1 freezes nothing (the mandatory freeze note, T7)"
  grep -qiE 'HA-9[2-9]' "$R" && ok "record cites HA-92+" \
    || warn "record cites no HA-92+ claim — only honest if T4 invented no physical number"
fi

# the mutation runner itself must exist and be gated on build success
if ls docs/internal/verify/verify-f1*.sh >/dev/null 2>&1; then
  ok "an F1 mutation harness exists: $(ls docs/internal/verify/verify-f1*.sh | tr '\n' ' ')"
  grep -ql 'BUILD-FAIL' docs/internal/verify/verify-f1*.sh \
    && ok "  it is gated on build success (C4's lesson)" \
    || bad "  it is NOT gated on build success — a non-compiling mutation reads green off a stale binary"
  grep -ql 'PIPE' docs/internal/verify/verify-f1*.sh \
    && ok "  it traps PIPE (E2 lost a header to SIGPIPE)" \
    || bad "  it does not trap PIPE"
else
  bad "no docs/internal/verify/verify-f1*.sh — the brief REQUIRES a mutation runner"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "SUMMARY"
# ══════════════════════════════════════════════════════════════════════════════════
if [ $fail -eq 0 ]; then
  printf '\033[32mMECHANICAL GATES PASSED\033[0m — and that is the smaller half.\n'
else
  printf '\033[31mGATES FAILED\033[0m — and remember: a red gate is a QUESTION. For each one, ask\n'
  printf '  (a) does it predate %s?  (b) does it violate the real property or a proxy?\n' "$BASE_SHORT"
fi

cat <<'EOF'

╔══════════════════════════════════════════════════════════════════════════════════╗
║  WHAT THIS SCRIPT CANNOT MECHANIZE — a person must read for all of this          ║
╚══════════════════════════════════════════════════════════════════════════════════╝

F1 defines a SEAM. No gate has an opinion about whether a seam is the right seam, and
this is the first chunk since C1 to define one rather than fill one in — six later
chunks (F2, F3, G1, G2, H2, R1) inherit whatever it decided. The mechanizable part
above is the smaller half.

  THE RULINGS — each needs a real decision AND a named rejected alternative
   1. T1: does `IMechanism` EARN ITS EXISTENCE, or is it a std::span<IMotor*> with a
      nicer name? Weigh the four cases in the brief ITEM BY ITEM (R1 over pros::Motor;
      H2 over VexBuilder joints where there may be no motor at all; A3-style hostility
      injected honestly at one seam vs. faked currents on three FakeMotors; and AIR —
      the H-drive's primary mechanism is a PNEUMATIC CLAMP and there is no digital-out
      seam in the tree). A seam shaped only around motors was shaped by half the
      hardware. "We shipped a concrete MotorGroup + IDigitalOut instead" is a
      DEFENSIBLE answer; "the roadmap used the word abstraction" is not.
   2. T1's scope line: how much of the operation layer is F1's and how much is F2's?
      Read the new code and ask whether it crossed. Scope creep into F2 is the single
      largest risk to this chunk, and §12 above only greps for the OBVIOUS words.
   3. T2: is an unconfirmed operation IMPOSSIBLE to mistake for a successful one at
      the Routine layer? Not "documented as different" — impossible. Read D1 §2.7
      (three result vocabularies) and judge whether F1 made F2's job easier or harder.
   4. T4: does a `Hold` actually hold a loaded cascade lift? NOBODY CAN CHECK THIS
      UNTIL THERE IS A ROBOT. That is why the reasoning has to survive in writing —
      read the HA-92+ entries and ask whether each is falsifiable and blast-radiused.
   5. T6: is the FAULT vs VERDICT line drawn in the right place? A jam with a healthy
      mechanism might be either. Getting it backwards floods the latch with normal
      outcomes and destroys first-fault triage — which is the whole reason it exists.
   6. T7: can G1's `RobotBuilder.from(profile)` still wire mechanisms from
      `{name, motors:[...], pneumatics:[port]}`? Try to sketch it. If you cannot, F1
      moved a problem into a chunk that cannot argue back.

  THE TESTS — the recurring trap in its F1 form
   7. DO THE FAKE AND THE OPERATION SHARE A NOTION OF "DONE"? This has bitten five
      chunks (C1, C3, C4, E2, E3). If the fake reports completion by the same rule the
      operation uses to decide completion, a broken completion check passes green and
      proves nothing. Read the fake and the operation side by side.
   8. WERE THE EXPECTED TIMELINES WRITTEN BY HAND, FROM THE CONTRACT, BEFORE RUNNING?
      "tick N: commanded; tick N+k: confirmed; verdict at N+k" — asserted against
      literals, not against the fake's opinion. Only the git history and the progress
      log can tell you; a timeline back-filled from a passing run looks identical.
   9. CAN THE FAKE LIE? Report confirmed when it is not; report motion when stalled.
      An operation that trusts its device has no failure detection, and a test that
      never lies to it cannot tell. Check the hostile fakes actually BITE (A3's
      precedent: a hostility that does not bite is worse than none).
  10. Every test names, in a comment, the bug it would catch. Read the comments and
      ask whether the named bug is real and whether the test would actually catch it.
  11. Run the mutation harness separately and read EVERY GREEN. This script does not
      run mutations — it only checks that a gated runner exists.

  THE CONCURRENCY PROOF
  12. Does a mechanism ticking inside `scheduler.waitUntil` while a motion runs trip
      NOTHING (inTick_/inWait_/inBoundary_), double-tick NOTHING, and leave motion
      accuracy UNCHANGED? "Both progress" is easy to assert weakly. Check the test
      would fail if the mechanism silently stopped ticking.
  13. Is there any blocking convenience? If so, prove it cannot deadlock, cannot
      re-enter a wait that already owns the loop, and is watchdog-armed in start()
      with NO path that disarms it. If that proof is hard, the answer was "ship only
      the tick form", and the record should say why.

  THE PROSE — four stale-prose defects were found by READING in one session at Phase E
  14. Chapter 13: could someone who is NOT a robotics expert write a mechanism against
      this seam from that chapter alone? That is a cold-read test with a person who has
      not seen the code — the same DoD clause D3 could not close.
  15. Chapter 14, rewritten in E3's order: what is MEASURED, what it was measured
      AGAINST, what remains UNMEASURED. Does it still say plainly that no concrete
      scoring primitive exists, nothing has run on hardware, and there is no sequencer?
  16. Every sentence this script proved still EXISTS — reread it for whether it is
      still TRUE. §9 checks literals; no grep can check truth.
  17. The master plan §17 tier table, §16.3 registry example, §6 module map, and the
      F4 scope note (lines 293-297) which lists this exact work as DEFERRED — it is not
      deferred any more.
  18. The roadmap M4 WS7 checkbox and M7's two `[~]` Tier-2 items: they are partial
      SPECIFICALLY because `.then(intake.in)` could not exist. Did they move, and is
      the evidence cited real?

  AND THE ONE THAT MATTERS MOST
  19. The library can command a FAKE mechanism on a HOST. No robot has moved, nothing
      has been scored, and the seam's arrival must not have inflated a single claim
      anywhere. If any sentence in this chunk reads better than that, it is wrong.
EOF

exit $fail
