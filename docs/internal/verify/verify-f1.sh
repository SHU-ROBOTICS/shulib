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
# moves?) are done against an ISOLATED COPY of include/ in /tmp — never by editing
# include/. verify-d3.sh edited the real header and restored it; that is unsafe while a
# chunk holds UNCOMMITTED work in the same file, and `git checkout` as a restore would
# discard the chunk rather than the mutation (E2 lost an hour to exactly that).
#
# ISOLATED, NOT SHADOWED. Until 2026-08-13 this section used a "shadow" header on what
# the comment CLAIMED was an earlier -I path — and the implementation appended it AFTER
# -Iinclude. GCC searches -I left to right, so the real header always won, the mutated
# copy was never opened, both bites compiled green, and the script reported "the pin is
# decoration" twice. Two false five-alarm fires from one token of ordering. The repair
# is not to fix the order (that is a thing you have to keep getting right) but to make
# the failure IMPOSSIBLE: the isolated tree is the ONLY -I that can serve a shulib
# header, so there is no second copy left to win a search. See section 6.
#
# ═══ A GREEN FROM A MUTATION RUNNER MEANS NOTHING UNTIL A CONTROL HAS GONE RED
#
# The same episode taught the deeper lesson. That bite guarded the mutation's
# APPLICATION (`count == 1`) and never guarded its ARRIVAL at the compiler, so every
# possible breakdown in delivery — wrong -I order, wrong path, a different include
# spelling — read out as the one alarming conclusion "the pin does not bite". A tool
# that cannot tell "the pin is blind" from "my mutation never arrived" is worse than no
# tool. Section 6 therefore (a) runs POSITIVE CONTROLS first and refuses to interpret
# anything downstream if one fails to fire, and (b) proves delivery with -MM before
# every probe. Both report HARNESS, not FAIL — see "TWO KINDS OF RED" below.
#
# ═══ TWO KINDS OF RED
#
#   FAIL    — the repository violates a property. That is the chunk's problem.
#   HARNESS — this script could not ask the question: a literal it greps for was
#             reworded, a mutation would not apply, a control did not fire. That is
#             THIS FILE's problem and it must never be read as a finding about F1.
# Exit status: 0 all clear · 1 at least one FAIL · 2 no FAIL but the harness is broken.
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

# ─── THE SECOND BASELINE, AND WHY THERE ARE TWO ───────────────────────────────────
# $BASE answers "did the chunk commit?" — a POSITION check, and the override above
# moves it forward every time the reviewer commits.
# It cannot also answer "what did the chunk CHANGE": once F1's work is itself a commit
# and the reviewer runs with F1_VERIFY_BASE=$(git rev-parse HEAD), every content diff
# against $BASE is empty and the gates that require a change ("ch14 MUST be rewritten",
# "the register must state the freeze note") go red for a chunk that did all of it.
# That is stale-by-construction: the harness would fail F1 for having succeeded.
#
# So content is compared against $WORKBASE — the tree immediately BEFORE F1's work —
# DERIVED FROM THE REPO, not hardcoded: this project commits a chunk as "<CHUNK> <topic>
# (WS<n>): ...", so the newest commit whose subject starts with "F1 " is F1's work and
# its parent is the tree before it. If no such commit exists the chunk is still
# uncommitted, and $WORKBASE falls back to $BASE — the original behaviour.
f1commit=$(git log --format='%H %s' -n 200 | sed -n 's/^\([0-9a-f]\{40\}\) F1 .*/\1/p' | head -1)
if [ -n "$f1commit" ] && git rev-parse -q --verify "$f1commit^" >/dev/null 2>&1; then
  WORKBASE=$(git rev-parse "$f1commit^")
else
  WORKBASE="$BASE"
fi
WORKBASE_SHORT="$(git rev-parse --short "$WORKBASE" 2>/dev/null || echo "${WORKBASE:0:7}")"

T=/tmp/f1-verify
rm -rf "$T"; mkdir -p "$T"

fail=0
hfail=0
hdr()  { printf '\n\033[1m=== %s ===\033[0m\n' "$1"; }
ok()   { printf '  \033[32mPASS\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; fail=1; }
warn() { printf '  \033[33mNOTE\033[0m %s\n' "$1"; }
# harn(): THIS SCRIPT could not ask the question. Never a finding about F1. Kept
# separate from bad() because conflating the two is how a harness produces confident
# false alarms — the exact defect that put five reds on F1's first run.
harn() { printf '  \033[35mHARNESS\033[0m %s\n' "$1"; hfail=1; }

# head_of FILE -> the file's content BEFORE F1's work, or empty if it did not exist
head_of() { git show "$WORKBASE:$1" 2>/dev/null; }

printf '\033[1mF1 INDEPENDENT VERIFICATION\033[0m  (%s)\n' "$(date '+%F %T')"
printf 'position baseline: %s  — "the chunk must not have committed past this"\n' "$BASE_SHORT"
printf 'content  baseline: %s  — "what F1 changed" is measured from here%s\n' "$WORKBASE_SHORT" \
       "$([ "$WORKBASE" = "$BASE" ] && echo ' (same commit: F1 is uncommitted)' || echo " (derived: parent of ${f1commit:0:7})")"

# ══════════════════════════════════════════════════════════════════════════════════
hdr "0. NOTHING WAS COMMITTED, and the tree PRODUCED something"
# ══════════════════════════════════════════════════════════════════════════════════
# Two failures that look opposite and are the same failure: a chunk that committed
# (the brief says "Do not commit. Do not push.") and a chunk that produced nothing.

branch=$(git branch --show-current)
[ "$branch" = "shulib-v2" ] && ok "on shulib-v2" || bad "wrong branch: $branch"

now=$(git rev-parse HEAD)
if [ "$now" = "$BASE" ]; then
  ok "HEAD is still the position baseline ($BASE_SHORT) — the chunk committed nothing past it"
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
newcases=$( { git diff "$WORKBASE" -- test/ ; git status --porcelain test/ | awk '/^\?\?/{print $2}' \
              | xargs -r cat | sed 's/^/+/'; } 2>/dev/null | grep -c '^+.*TEST_CASE' )
if [ "$newcases" -gt 0 ]; then
  ok "$newcases new TEST_CASE line(s) in test/ vs $WORKBASE_SHORT"
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
done < <(git status --porcelain test/ | awk '{print $NF}'; git diff --name-only "$WORKBASE" -- test/)
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
newhdrs=$( { git status --porcelain | awk '/\?\?/{print $2}'; git diff --name-only "$WORKBASE"; } \
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
# covers only the surfaces listed in tools/api_doc_tool.py TARGETS, so an undocumented
# public member in F1's NEW header fails nothing. The brief requires that be RULED, not
# defaulted.
#
# The old test was `targets > 2`, with 2 meaning "the two frozen surfaces" — a fact about
# the tool on the day this was written, not a property. Derived instead: did the list
# GROW since the content baseline? That keeps meaning the same thing after the next chunk
# adds a surface, and it stops being a check that silently passes forever once someone
# else adds a third entry.
targets=$(grep -c '"header":' tools/api_doc_tool.py)
targets_base=$(head_of tools/api_doc_tool.py | grep -c '"header":')
echo "    api_doc_tool TARGETS entries: $targets_base at $WORKBASE_SHORT -> $targets now"
grep -n '"header":' tools/api_doc_tool.py | sed 's/^/      /'
if [ "$targets" -gt "$targets_base" ]; then
  ok "F1 added its header to TARGETS — the new surface is coverage-gated"
else
  warn "TARGETS did not grow — F1's new header is NOT coverage-gated. LEGAL, but the brief"
  warn "  requires the choice to be made deliberately and recorded. Check F1-COMPLETED.md."
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "5. REMOVABILITY — no public doc may link into docs/internal/ (C7/C8)"
# ══════════════════════════════════════════════════════════════════════════════════
# The narrow FOUR-TERM grep is the true gate. A hit that already existed at the
# baseline is NOT F1's, and the README's caution is explicit that the harness once
# got this exactly wrong. Each hit below is triaged against $WORKBASE_SHORT.
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
# move; it does not prove the pin can still see a move.
#
# ═══ THE FIVE-ALARM FIRE THIS SECTION WAS REBUILT AFTER (2026-08-13) ══════════════
#
# The first run of this script reported that BOTH noexcept bites compiled green — that
# is, that the F6 and F10 pins were "decoration". They were not. The bites did this:
#
#     PINFLAGS=(... -Iinclude ...)
#     g++ "${PINFLAGS[@]}" -I"$T/shadow" <pin.cpp>          # shadow LAST
#
# GCC searches -I directories LEFT TO RIGHT, so include/shulib/chassis/routine.hpp won
# every time and the mutated shadow copy WAS NEVER OPENED (`g++ -H` prints the real path).
# The mutation applied perfectly to a file the compiler then ignored. Two false alarms
# from one token of ordering — and the comment three lines above the bug claimed the
# shadow was "on an earlier -I path", so the intent was right and nobody ever checked
# that the comment was true.
#
# The repair is NOT "put -I$T/shadow first". That leaves a second, unmutated copy of
# every header on the search path, still one edit away from winning again. Instead:
#
#   1. ISOLATE, DO NOT SHADOW. include/ is copied WHOLESALE to $T/iso, and $T/iso is the
#      ONLY -I that can serve a shulib header. There is no second copy left to win a
#      search, so this entire class of bug is structurally impossible rather than
#      something the next editor has to keep getting right.
#   2. EVERY PROBE PROVES DELIVERY. `g++ -MM -MG` must list the mutated header BEFORE the
#      compile is believed, so a green can never silently mean "the mutation never
#      arrived". A delivery miss reports HARNESS, never FAIL.
#   3. CONTROLS FIRST, AND THEY GATE EVERYTHING AFTER THEM. Before any subtle mutation,
#      each pin file is shown a change it MUST catch (a frozen member removed; a frozen
#      return type changed) and one it must NOT catch (nothing changed at all). If a
#      control misbehaves the bites are SKIPPED and the section reports "method broken",
#      because a green from a runner whose controls have not fired means nothing.
#   4. THE VERDICT IS "RED AND NAMED". A red that never prints "F6/F10 FREEZE VIOLATION"
#      is not a pin biting — it is some other compile error, and it is reported as one.
#
# include/ is never written to. The isolated tree is restored from include/ before every
# probe and diffed against it at the end of the section to prove it.

# The clean compile is the real gate, so it runs under the REAL build's flags
# (test/CMakeLists.txt:20-22), -Werror included.
CMAKEWARN=(-Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion
           -Wdouble-promotion)
for pin in test/f6_signature_pin_test.cpp test/routine_signature_pin_test.cpp; do
  if g++ -std=c++20 -fsyntax-only "${CMAKEWARN[@]}" -Iinclude -Itest -isystem test/vendor \
       "$pin" 2>"$T/$(basename "$pin").log"; then
    ok "$(basename "$pin") compiles — the frozen surface is intact"
  else
    bad "$(basename "$pin") FAILED — a frozen signature moved"
    grep -m3 -E 'F6 FREEZE VIOLATION|F10 FREEZE VIOLATION|error:' "$T/$(basename "$pin").log" \
      | cut -c1-200 | sed 's/^/        /'
  fi
done

# ─── the isolated tree ────────────────────────────────────────────────────────────
ISO="$T/iso"
rm -rf "$ISO"
cp -a include "$ISO"          # $ISO/shulib/... — a full copy, not a single header
# NO -Iinclude HERE. That omission is the whole point; if you add it back, the bites go
# blind again exactly as they did on 2026-08-13.  -Werror is dropped for the probes so a
# red can only come from a hard error (the static_assert), never from a stray warning a
# mutation happened to provoke.
PINFLAGS=(-std=c++20 -fsyntax-only -Wall -Wextra -Wpedantic -Wshadow -Wconversion
          -Wsign-conversion -Wdouble-promotion -I"$ISO" -Itest -isystem test/vendor)

# decl_line REL 'ERE' -> the ONE non-comment line of include/REL matching the pattern.
# Derived from the repo instead of hardcoded, so a reformat or a new attribute does not
# turn into "the mutation would not apply".
decl_line() {
  local out
  out=$(grep -nE "$2" "include/$1" | grep -vE '^[0-9]+: *(//|\*)')
  [ "$(printf '%s' "$out" | grep -c .)" -eq 1 ] || return 1
  printf '%s' "${out#*:}"
}

# probe KIND LABEL PIN REL TAG FROM TO
#   KIND=neg     nothing is changed; the probe MUST compile green (the copy is faithful)
#   KIND=control a change the pin MUST catch; a green here means THE METHOD IS BROKEN
#   KIND=bite    the real question; a green here is a HOLE IN THE PIN and a real FAIL
# Sets probe_ok=1 when the probe behaved as required.
probe() {
  local kind="$1" label="$2" pin="$3" rel="$4" tag="$5" from="$6" to="$7"
  local log="$T/probe.$(basename "$pin").$$.log"
  probe_ok=0
  cp "include/$rel" "$ISO/$rel"                      # restore before every probe
  if [ "$kind" != "neg" ]; then
    if [ -z "$from" ]; then
      harn "$label: could not DERIVE the line to mutate from include/$rel — the"
      warn "  declaration was reworded or now matches more than once. Nothing was asked."
      return
    fi
    python3 - "$ISO/$rel" "$from" "$to" <<'PY' || { harn "$label: the mutation would not apply (see above) — nothing was asked"; return; }
import sys
p, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(p, encoding='utf-8').read()
n = s.count(a)
if n != 1:
    sys.stderr.write(f"        mutation text occurs {n} times, need exactly 1\n")
    sys.exit(3)
open(p, 'w', encoding='utf-8').write(s.replace(a, b, 1))
PY
    # DELIVERY GUARD: prove the compiler will open the MUTATED file. Without this, every
    # delivery failure reads out as "the pin does not bite" — the 2026-08-13 defect.
    if ! g++ "${PINFLAGS[@]}" -MM -MG "$pin" 2>/dev/null | tr ' ' '\n' | grep -qF "$ISO/$rel"; then
      harn "$label: DELIVERY-GUARD FAILED — $ISO/$rel is not in the TU's dependency list,"
      warn "  so whatever the compile says next is about some OTHER copy of the header."
      warn "  This is a fault in THIS SCRIPT, not evidence about the pin."
      return
    fi
  fi
  if g++ "${PINFLAGS[@]}" "$pin" 2>"$log"; then
    case "$kind" in
      neg)     ok    "$label: green, as required — the isolated copy is faithful"; probe_ok=1 ;;
      control) harn  "$label: POSITIVE CONTROL DID NOT FIRE. A change the pin must catch"
               warn  "  compiled green, so the METHOD IS BROKEN and no bite below means"
               warn  "  anything. This is NOT a finding that the pin is decoration." ;;
      bite)    bad   "$label: COMPILED GREEN — the pin cannot see this change. A HOLE."
               warn  "  (the controls above fired, so the mutation did arrive)" ;;
    esac
  else
    if [ "$kind" = "neg" ]; then
      harn "$label: the UNMUTATED isolated copy does not compile — the copy or the flags"
      warn "  are wrong, and every probe below would be a red for the wrong reason."
      grep -m3 'error:' "$log" | cut -c1-160 | sed 's/^/        /'
    elif grep -q "$tag" "$log"; then
      ok "$label: red, and the message NAMES the freeze ($tag)"; probe_ok=1
    else
      bad "$label: red, but the message never says '$tag' — a nameless red is not a pin"
      grep -m4 'error:' "$log" | cut -c1-160 | sed 's/^/        /'
    fi
  fi
}

# ─── F10 / Routine ────────────────────────────────────────────────────────────────
R=shulib/chassis/routine.hpp
r_ok=$(decl_line "$R" 'bool ok\(\) const noexcept')                        || r_ok=""
r_traj=$(decl_line "$R" 'TrajectoryResult& lastTrajectory\(\) const noexcept') || r_traj=""

probe neg "F10 negative control (nothing changed)" \
      test/routine_signature_pin_test.cpp "$R" 'F10 FREEZE VIOLATION' "" ""
neg10=$probe_ok
# CONTROL 1 — a frozen member REMOVED. Renaming it out from under the pin is the same
# thing to the pin and does not break routine.hpp's own callers (the accessor has none;
# ok() does — result() calls it — which is why the removal control uses lastTrajectory).
probe control "F10 control A: frozen member lastTrajectory() removed" \
      test/routine_signature_pin_test.cpp "$R" 'F10 FREEZE VIOLATION' \
      "$r_traj" "${r_traj/lastTrajectory(/lastTrajectoryGONE(}"
ctl10a=$probe_ok
# CONTROL 2 — a frozen RETURN TYPE changed.
probe control "F10 control B: frozen return type of ok() bool -> int" \
      test/routine_signature_pin_test.cpp "$R" 'F10 FREEZE VIOLATION' \
      "$r_ok" "${r_ok/bool ok(/int ok(}"
ctl10b=$probe_ok

# THE BITE — drop noexcept from Routine::ok(). D2's campaign hole #1: for a NON-overloaded
# member the compiler accepts a static_cast that ADDS noexcept, so a cast-only pin cannot
# see noexcept being DROPPED. D3 closed it with a compound requirement. Still closed?
if [ "$neg10" = 1 ] && [ "$ctl10a" = 1 ] && [ "$ctl10b" = 1 ]; then
  probe bite "F10 BITE: noexcept DROPPED from Routine::ok()" \
        test/routine_signature_pin_test.cpp "$R" 'F10 FREEZE VIOLATION' \
        "$r_ok" "${r_ok/ const noexcept/ const}"
else
  harn "F10 BITE SKIPPED — a control misbehaved above. A green from a mutation runner"
  warn "  whose controls have not fired is not evidence of anything, and reporting one"
  warn "  as 'the pin is decoration' is exactly how this section produced a false alarm."
fi

# ─── F6 / Chassis ─────────────────────────────────────────────────────────────────
C=shulib/chassis/chassis.hpp
c_cfg=$(decl_line "$C" 'MotionConfig& motionConfig\(\) const noexcept')  || c_cfg=""
c_sa=$(decl_line "$C" 'double strafeAuthority\(\) const \{')             || c_sa=""

probe neg "F6 negative control (nothing changed)" \
      test/f6_signature_pin_test.cpp "$C" 'F6 FREEZE VIOLATION' "" ""
neg6=$probe_ok
probe control "F6 control A: frozen member strafeAuthority() removed" \
      test/f6_signature_pin_test.cpp "$C" 'F6 FREEZE VIOLATION' \
      "$c_sa" "${c_sa/double strafeAuthority(/double strafeAuthorityGONE(}"
ctl6a=$probe_ok
probe control "F6 control B: frozen return type of motionConfig() const& -> by value" \
      test/f6_signature_pin_test.cpp "$C" 'F6 FREEZE VIOLATION' \
      "$c_cfg" "${c_cfg/const motion::MotionConfig\& motionConfig(/motion::MotionConfig motionConfig(}"
ctl6b=$probe_ok

if [ "$neg6" = 1 ] && [ "$ctl6a" = 1 ] && [ "$ctl6b" = 1 ]; then
  probe bite "F6 BITE: noexcept DROPPED from Chassis::motionConfig()" \
        test/f6_signature_pin_test.cpp "$C" 'F6 FREEZE VIOLATION' \
        "$c_cfg" "${c_cfg/ const noexcept/ const}"
else
  harn "F6 BITE SKIPPED — a control misbehaved above; see the F10 note."
fi

# Restore and PROVE the restore: the isolated tree must be byte-identical to include/,
# and include/ must be untouched (it is never opened for writing anywhere above).
cp "include/$R" "$ISO/$R"; cp "include/$C" "$ISO/$C"
if diff -r -q include "$ISO" >/dev/null 2>&1; then
  ok "every mutation was restored — the isolated tree is byte-identical to include/"
else
  harn "the isolated tree did not restore cleanly (include/ is still untouched — it is"
  warn "  never written by this script — but a later probe would have run on a dirty copy)"
  diff -r -q include "$ISO" 2>&1 | head -5 | sed 's/^/        /'
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "7. routine.hpp — ONLY then() MAY HAVE MOVED (member-by-member diff vs $WORKBASE_SHORT)"
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
  bad "could not read routine.hpp at $WORKBASE_SHORT"
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
    git diff "$WORKBASE" -- include/shulib/chassis/routine.hpp | sed 's/^/        /' | head -80
  fi
  if git diff --quiet "$WORKBASE" -- include/shulib/chassis/routine.hpp; then
    warn "routine.hpp is byte-identical to $WORKBASE_SHORT — T5 ruled 'fix the prose', or is unruled"
  else
    warn "routine.hpp changed; the full diff (read it — 'unchanged in meaning' is a human call):"
    git diff --stat "$WORKBASE" -- include/shulib/chassis/routine.hpp | sed 's/^/        /'
  fi
fi

# chassis.hpp is fully frozen (F6). F1 has NO licence to touch it at all.
if git diff --quiet "$WORKBASE" -- include/shulib/chassis/chassis.hpp; then
  ok "chassis.hpp untouched (F6 is fully frozen — F1 has no unfrozen member there)"
else
  bad "chassis.hpp CHANGED — every member is pinned by F6; this is a breaking change to argue"
  git diff "$WORKBASE" -- include/shulib/chassis/chassis.hpp | sed 's/^/        /' | head -60
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
if git diff "$WORKBASE" -- docs/roadmap.md | grep -E '^\+' | grep -qiE 'LOCKED 2026-08-13|frozen at F1|locked at F1'; then
  bad "the roadmap diff claims a NEW lock dated today — F1 freezes nothing"
  git diff "$WORKBASE" -- docs/roadmap.md | grep -E '^\+' | grep -iE 'LOCKED|frozen' | sed 's/^/        /'
else
  ok "no new 'LOCKED' claim in the roadmap diff"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "9. THE 'NEVER DRIVEN A ROBOT' CLAIMS MUST BE INTACT"
# ══════════════════════════════════════════════════════════════════════════════════
# "Do not let the seam's arrival inflate any claim. Nothing here touches accuracy, and
# nothing here has met a robot." A seam that can command a FAKE intake on a HOST is
# not a robot that can score.
# ─── HOW A CLAIM IS CHECKED, AND THE DEFECT THAT CHANGED IT ───────────────────────
# This gate protects a PROPERTY ("the docs still say the library has never driven a
# robot"), and it used to test that property with `grep -qF` on one exact sentence.
# Two ways that lies, both of them observed on F1's first run:
#
#   1. LINE WRAPPING. Chapter 14 says "Booting is not driving" — with the newline of a
#      wrapped markdown paragraph between "not" and "driving". grep -F never matches
#      across a newline, so the harness reported the claim MISSING/WEAKENED while it sat
#      there in the file. Every literal is therefore matched against a WHITESPACE-
#      NORMALISED view of the document: a reflow is not a retraction.
#   2. REWORDING. A claim can survive in different words. So a claim takes ALTERNATIVE
#      spellings and passes if ANY of them is present. That is not a weakening as long
#      as every alternative is itself a full statement of the property — a claim list
#      whose alternatives are vaguer than the property is how this gate would go vacuous,
#      so each list below is deliberately narrow.
#
# And the triage is now honest about whose problem it is: a literal that was NOT present
# at the content baseline either is THIS SCRIPT being stale (HARNESS), not F1 weakening
# anything. Reporting that as FAIL is how a harness manufactures findings.
cat > "$T/claimcheck.py" <<'PY'
import re, sys
norm = lambda s: re.sub(r'\s+', ' ', s)
hay = norm(open(sys.argv[1], encoding='utf-8').read())
sys.exit(0 if any(norm(a) in hay for a in sys.argv[2:]) else 1)
PY
claim() {  # claim FILE "DESC" "LITERAL" ["ALTERNATIVE" ...]
  local f="$1" desc="$2"; shift 2
  if python3 "$T/claimcheck.py" "$f" "$@"; then
    ok "$(basename "$f"): $desc"
  else
    head_of "$f" > "$T/claim.base" 2>/dev/null
    if [ -s "$T/claim.base" ] && python3 "$T/claimcheck.py" "$T/claim.base" "$@"; then
      bad "$(basename "$f"): MISSING/WEAKENED -> $desc"
      warn "  ...it WAS present at $WORKBASE_SHORT — F1 removed or reworded it. Spellings sought:"
      printf '        %s\n' "$@" | cut -c1-110
    else
      harn "$(basename "$f"): none of this script's spellings for [$desc] were present at"
      warn "  $WORKBASE_SHORT either, so the literal is STALE, not the claim weakened. Read the"
      warn "  file, then fix the spellings here — do NOT read this as a finding about F1."
      printf '        %s\n' "$@" | cut -c1-110
    fi
  fi
}
claim README.md "the never-driven headline" \
  "**This library has never driven a robot.**"
claim README.md "the on-brain boot claims nothing about driving" \
  "It drove nothing, because it cannot."
grep -qi "hardware adapters" README.md && ok "README.md: the 'hardware adapters do not exist' bullet survives" \
                                       || bad "README.md: the 'hardware adapters' bullet is gone"
claim docs/guide/14-what-it-cannot-do-yet.md "the never-driven heading" \
  "## It has never driven a robot"
claim docs/guide/14-what-it-cannot-do-yet.md "no motor was ever controlled" \
  "**No shulib code has ever controlled a motor or read a real sensor.**"
# THE BOOT-vs-DRIVE DISTINCTION. The property: chapter 14 states that booting on a brain
# is not evidence of driving. Any ONE of these says exactly that; each is a complete
# statement of it, so the list is robust to rewording without being satisfiable by a
# weaker sentence. (Alternative 1 is the one that wraps.)
claim docs/guide/14-what-it-cannot-do-yet.md "booting is not driving" \
  "Booting is not driving" \
  "It has booted on a brain, and that proves less than it sounds like" \
  "boots, prints a banner — and drives nothing"
claim docs/guide/14-what-it-cannot-do-yet.md "every physical constant is a guess" \
  "**Every physical constant is a labeled guess.**"
# The < 1 degree sentence stays exactly as it is (brief, Documentation section). ONE
# spelling only, on purpose: the brief pins this sentence's wording, so unlike the claims
# above it has no legitimate alternative. Normalised for wrapping, not for wording.
claim docs/guide/14-what-it-cannot-do-yet.md "the < 1° requirement is still NOT claimed met" \
  'does **not** claim the `< 1°` requirement is met'

# Chapter 14 MUST change (two sections in it are now wrong) — but only in the places
# the brief names. Report the diff so a person can read it.
if git diff --quiet "$WORKBASE" -- docs/guide/14-what-it-cannot-do-yet.md; then
  bad "ch14 is UNCHANGED — 'The mechanism seam is a placeholder' (l.167-184) and 'No mechanisms"
  bad "  for recipes to command' (l.191+) are now wrong or partly wrong; the brief requires a rewrite"
else
  ok "ch14 was rewritten (required)"
  git diff --stat "$WORKBASE" -- docs/guide/14-what-it-cannot-do-yet.md | sed 's/^/        /'
fi
# and it must not have grown a claim it cannot support
if git diff "$WORKBASE" -- README.md docs/guide/14-what-it-cannot-do-yet.md docs/guide/09-the-recipe-api.md \
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
# ```cpp blocks (api_doc_tool.py) — inline-backtick prose is text NO COMPILER HAS EVER
# SEEN. So the property has two halves, and BOTH are checked below:
#   A. no public document still ASSERTS `.then(intake.in)` as a thing you can write, and
#   B. the corrected spelling exists as a COMPILED, quoted example, so it can never rot
#      back into prose.
#
# ─── THE DEFECT THIS REPLACES ─────────────────────────────────────────────────────
# This gate used to COUNT occurrences of the string and fail on any: it reported "the
# flagship claim still lives as UNCOMPILED prose in 8 places" when all eight survivors
# were RETRACTIONS — sentences whose whole job is to say the spelling was never valid
# C++ and to give the corrected form. A checker that cannot tell a claim from its
# correction demands the documentation delete its own errata, which is the opposite of
# what T5 asked for. The old comment even admitted the proxy ("you decide whether each
# survivor is honest") and then failed the build anyway; a check that needs a human to
# decide must not be wired to `bad`.
#
# What is checked now: each surviving occurrence must sit inside a RETRACTING CONTEXT —
# its markdown paragraph (or, in a header, its comment block) must also say the spelling
# was wrong / corrected / conditional. An occurrence with no retraction near it is a bare
# assertion, and that IS the half-done state T5 forbids. This is still a proxy — prose
# cannot be parsed — but it is a proxy for the right property, and it fails only on the
# shape that is actually wrong.
#
# ONE BRANCH WAS DELETED OUTRIGHT: "or the bare-member spelling appears in a FENCED cpp
# block, and is therefore compiled" used to be an acceptable outcome. It is not one any
# more, and it never really was: T5 established that `then(intake.in)` is not valid C++
# here at all, so a fenced block containing it could not compile and check-examples would
# fail on it first. Keeping that branch would have offered a PASS for a state that cannot
# exist. Part B below replaces it with the demand that the CORRECTED spelling be compiled.
python3 - <<'PY'
import glob, re, sys

FILES = (['README.md', 'include/shulib/chassis/routine.hpp']
         + [p for p in glob.glob('docs/**/*.md', recursive=True) if '/internal/' not in p])
# The bad spelling: then(<obj>.<member>) with no call parens, in any of its quotings.
IDIOM = re.compile(r'then\(\s*[A-Za-z_]\w*\.[A-Za-z_]\w*\s*\)|`[A-Za-z_]\w*\.(in|release)`')
# A retraction: the sentence around it disowns the spelling. Deliberately specific —
# "old"/"wrong" alone would let a vague sentence launder a live claim.
RETRACT = re.compile(
    r'never (?:valid|real|been valid) c\+\+|was never valid|not valid c\+\+|'
    r'valid c\+\+ only|corrected|correction|errata|'
    r'names? a (?:member )?function without calling it|only names a function|'
    r'does not call one|used to be quoted|older drafts|no longer|was wrong', re.I)

def paragraphs(text, is_md):
    """(start_line, end_line, text) for each blank-line-separated block. For a header,
    a run of consecutive comment lines is the block — that is its paragraph."""
    lines = text.split('\n')
    out, cur, start = [], [], 1
    for i, l in enumerate(lines, 1):
        blank = (not l.strip()) if is_md else (not l.strip().startswith('//'))
        if blank:
            if cur:
                out.append((start, i - 1, '\n'.join(cur)))
            cur, start = [], i + 1
        else:
            if not cur:
                start = i
            cur.append(l)
    if cur:
        out.append((start, len(lines), '\n'.join(cur)))
    return out

total, bare = 0, []
for p in sorted(set(FILES)):
    try:
        t = open(p, encoding='utf-8').read()
    except OSError:
        continue
    for s, e, block in paragraphs(t, p.endswith('.md')):
        hits = IDIOM.findall(block)
        if not hits:
            continue
        total += len(hits)
        if not RETRACT.search(block):
            bare.append((p, s, e, block.strip().split('\n')[0][:120]))

print(f"    {total} surviving mention(s) of the bare-member spelling across "
      f"{len(FILES)} public files; {len(bare)} paragraph(s) mention it WITHOUT retracting it")
if bare:
    print("    *** BARE ASSERTION(S) — a mention with no correction anywhere in its paragraph:")
    for p, s, e, first in bare:
        print(f"      {p}:{s}-{e}: {first}")
    sys.exit(1)
sys.exit(0)
PY
[ $? -eq 0 ] && ok 'every surviving `intake.in` mention is a RETRACTION, not a claim (T5)' \
             || bad 'a public document still ASSERTS `.then(intake.in)` — the half-done state T5 forbids'

# B. THE CORRECTED FORM MUST BE COMPILED, not merely written. Derived, not hardcoded:
# find the guide chapter that quotes a fenced `.then(` chain, and require that the same
# lines live in test/guide_examples_test.cpp — §13's verbatim gate then keeps them
# identical forever, and check-examples compiles them. Without this half, "delete every
# mention" would satisfy part A while leaving the flagship idiom unproven.
python3 - <<'PY'
import glob, re, sys
# Each doc family is coupled to ITS OWN companion test (the same pairing section 13
# enforces): the guide to guide_examples_test.cpp, the cookbook to
# cookbook_examples_test.cpp. Checking a cookbook line against the guide's test reports
# every cookbook example as uncompiled, which is a harness defect, not a finding.
COUPLING = {'docs/guide': 'test/guide_examples_test.cpp',
            'docs/cookbook': 'test/cookbook_examples_test.cpp'}
src = {d: {l.strip() for l in open(t).read().splitlines()} for d, t in COUPLING.items()}
found = []
for d in COUPLING:
    for p in sorted(glob.glob(d + '/*.md')):
        for b in re.findall(r'```cpp\n(.*?)```', open(p).read(), re.S):
            for line in b.splitlines():
                s = line.strip()
                if '.then(' in s:
                    found.append((p, s, s in src[d]))
if not found:
    print("    *** no fenced cpp block in the guide or cookbook quotes a `.then(` chain")
    sys.exit(1)
for p, s, compiled in found:
    print(f"    {'compiled' if compiled else '*** NOT IN THE TEST'}  {p}: {s[:90]}")
sys.exit(0 if all(c for _, _, c in found) else 1)
PY
[ $? -eq 0 ] && ok "the corrected then() spelling exists as a COMPILED, quoted example" \
             || bad "the corrected then() spelling is prose only — no compiler has seen it (T5's other half)"

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
python3 - "$WORKBASE" <<'PY'
import re, subprocess, sys
BASE = sys.argv[1]      # the content baseline, passed in — NOT a second hardcoded sha
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

# TickPhase is part of the F9 WIRE SCHEMA: a blackbox file already on disk carries the
# numbers, so an existing enumerator's VALUE may never move and a name may never be
# recycled. Appending is legal.
#
# THIS CHECK USED TO DIFF WHOLE LINES (`grep '^[-+].*User\s*='`) and called any edit to
# the line a wire break. F1 changed the trailing COMMENT on that line
# ("RESERVED" -> "RESERVED, still"); the value 5 never moved, and the harness reported a
# schema violation for a comment. Compare the VALUES — and compare all of them, not just
# User's, which the line-diff never did either.
python3 - "$WORKBASE" include/shulib/diag/debug_record.hpp <<'PY'
import re, subprocess, sys
base, path = sys.argv[1], sys.argv[2]

def enum_map(text, name):
    """enumerator -> value, resolving implicit numbering the way C++ does. None if the
    enum is not there at all (which is itself the answer, not a crash)."""
    m = re.search(r'enum class ' + name + r'[^{]*\{(.*?)\n\};', text, re.S)
    if not m:
        return None
    out, nxt = {}, 0
    for line in m.group(1).split('\n'):
        line = re.sub(r'//.*$', '', line).strip().rstrip(',').strip()
        if not line:
            continue
        mm = re.match(r'([A-Za-z_]\w*)\s*(?:=\s*(-?\d+))?$', line)
        if not mm:
            continue
        val = int(mm.group(2)) if mm.group(2) is not None else nxt
        out[mm.group(1)] = val
        nxt = val + 1
    return out

old_t = subprocess.run(['git', 'show', f'{base}:{path}'], capture_output=True,
                       text=True).stdout
old, new = enum_map(old_t, 'TickPhase'), enum_map(open(path).read(), 'TickPhase')
if new is None:
    print("    *** TickPhase not parseable in the working tree"); sys.exit(2)
if old is None:
    print(f"    *** TickPhase not parseable at {base[:7]}"); sys.exit(2)
moved = {k: (v, new.get(k)) for k, v in old.items() if new.get(k) != v}
added = {k: v for k, v in new.items() if k not in old}
print(f"    TickPhase enumerators: {len(old)} at baseline -> {len(new)} now; "
      f"User = {new.get('User')}")
if moved:
    for k, (o, n) in moved.items():
        print(f"    *** {k} MOVED {o} -> {n} — E1 blackbox files on disk already carry {o}")
    sys.exit(1)
if added:
    print(f"    appended (legal, additive): {added}")
sys.exit(0)
PY
case $? in
  0) ok "TickPhase values are stable — no enumerator moved (F9 wire schema intact)" ;;
  1) bad "a TickPhase enumerator's VALUE moved — it is in the F9 wire schema" ;;
  *) harn "TickPhase could not be parsed on one side — this check asked nothing; fix the parser" ;;
esac
# A PRODUCER WRITES THE VALUE. A renderer only reads it.
#
# This check used to be `grep -rln 'TickPhase::User'` and it PASSED — on
# diag/tick_attribution.hpp:162, `case TickPhase::User: return "usr";`. That is a name
# renderer, added by C5, that fires only when something has ALREADY set the phase. So the
# check reported "TickPhase::User now has a producer" on the strength of a switch label,
# for a file F1 never touched — and the very lesson it cites (E1: DebugRecord::fault was
# RENDERED by TermSink and documented in chapter 11 while nothing ever filled it) is
# precisely the shape it was accepting as proof. A mention is not a producer.
prod=$(python3 - <<'PY'
import glob, re
def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
    return '\n'.join(re.sub(r'//.*$', '', l) for l in t.split('\n'))
files = glob.glob('include/shulib/**/*.hpp', recursive=True) + \
        glob.glob('src/**/*.cpp', recursive=True) + glob.glob('src/**/*.hpp', recursive=True)
for p in sorted(set(files)):
    if p.endswith('debug_record.hpp'):        # the declaration is not a use
        continue
    try:
        t = strip_comments(open(p, encoding='utf-8').read())
    except OSError:
        continue
    for i, l in enumerate(t.split('\n'), 1):
        if 'TickPhase::User' not in l:
            continue
        reads = re.search(r'case\s+TickPhase::User|[=!]=\s*TickPhase::User', l)
        print(f"{'READS ' if reads else 'WRITES'} {p}:{i}: {l.strip()[:96]}")
PY
)
[ -n "$prod" ] && echo "$prod" | sed 's/^/        /'
if echo "$prod" | grep -q '^WRITES'; then
  ok "TickPhase::User now has a real PRODUCER (a site that WRITES the phase)"
else
  warn "TickPhase::User still has NO PRODUCER — every mention above only reads or renders"
  warn "  it. Legal only if F1-COMPLETED.md records WHY. E1's lesson (DebugRecord::fault"
  warn "  rendered by TermSink and documented in ch.11 while nothing ever filled it)"
  warn "  applies directly, and a renderer is exactly what E1 mistook for a producer."
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "12. THINGS F1 IS FORBIDDEN TO BUILD (scope creep + the standing decisions)"
# ══════════════════════════════════════════════════════════════════════════════════
newsrc=$( { git status --porcelain | awk '/\?\?/{print $2}'; git diff --name-only "$WORKBASE"; } \
          | grep -E '^(include|src)/.*\.(hpp|cpp)$' | sort -u )

# (a) No game semantics in hal/. The house rule is hal/vision.hpp:40 — classId is an
#     OPAQUE INTEGER, deliberately not `Cup` or `Yellow`. The HAL reports what the
#     device saw; meaning is assigned above it.
#
#     THE PROPERTY IS ABOUT THE API SURFACE, NOT THE FILE. Two things were wrong here:
#       * SCOPE. The old grep ran over raw lines and tried to drop comments with
#         `grep -v '^\S+: *(//|///|\*)'`, which only skips a line whose CONTENT STARTS
#         with a comment marker — a trailing `// ... the ring ...` on a code line still
#         matched. Comments are now stripped (line and block) before the scan, so what is
#         searched is the API surface: the names a caller can type. The reviewer's rule
#         is explicit that a comment may mention a mechanism or a game piece while
#         explaining WHY the HAL refuses to name one; that comment is the property being
#         upheld, and flagging it inverted the check.
#       * VOCABULARY. The old list carried `hue`, and hal/optical.hpp has always declared
#         `virtual double hue() const` — a colour-space reading the V5 optical sensor
#         physically reports, in degrees. That is device vocabulary, not game vocabulary:
#         a hue is what the sensor measured, and which game object it means is exactly the
#         judgement the HAL refuses to make. `capture` (capture a value) and `toggle` (a
#         solenoid's two states) are device words too, and are gone for the same reason.
#         What remains is game-OBJECT vocabulary plus alliance colours, which stay because
#         `red`/`blue`/`yellow` in a HAL identifier really would be this season leaking in
#         — and a hit is now triaged against the content baseline before it is called F1's.
#     And it is STRICTER than the grep it replaces, not looser: `grep -wi ring` cannot
#     see `sawRing()` or `ring_count` — the two spellings a game word would actually
#     arrive in. Identifiers are split on case and underscores before matching, so
#     `IRingSensor` is a hit and `hue()` is not. Bite-tested against a planted
#     `class IRingSensor { bool sawRing(); }` and `int score_count;` before this ran.
sem=$(python3 - <<'PY'
import glob, re
GAME = {'cup', 'cups', 'ring', 'rings', 'donut', 'donuts', 'mogo', 'mogos',
        'stake', 'stakes', 'goal', 'goals', 'alliance', 'preload', 'preloads',
        'matchload', 'quadrant', 'score', 'scores', 'scored', 'scoring',
        'ladder', 'climb', 'hang', 'yellow', 'blue', 'red'}
def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)     # block comments, incl. /** ... */
    return '\n'.join(re.sub(r'//.*$', '', l) for l in t.split('\n'))
def words(line):
    out = []
    for ident in re.findall(r'[A-Za-z_]\w*', line):
        out += [w.lower() for w in re.findall(r'[A-Z]+(?![a-z])|[A-Z][a-z]*|[a-z]+', ident)]
    return out
for p in sorted(glob.glob('include/shulib/hal/**/*.hpp', recursive=True)):
    for i, l in enumerate(strip_comments(open(p, encoding='utf-8').read()).split('\n'), 1):
        hit = sorted(set(words(l)) & GAME)
        if hit:
            print(f"{p}:{i}:{l.strip()}   <- {hit}")
PY
)
if [ -n "$sem" ]; then
  # Triage before blame — the D1 lesson the README's caution is written about.
  newsem=0
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    f=${line%%:*}; txt=${line#*:}; txt=${txt#*:}
    txt=${txt%%   <- *}          # drop the "<- [word]" annotation before matching
    if head_of "$f" | grep -qF -- "$txt"; then
      printf '      \033[33mPRE-EXISTING\033[0m %s\n' "$(echo "$line" | cut -c1-120)"
    else
      printf '      \033[31mNEW (F1)\033[0m     %s\n' "$(echo "$line" | cut -c1-120)"
      newsem=1
    fi
  done <<< "$sem"
  if [ "$newsem" -eq 1 ]; then
    bad "GAME-OBJECT vocabulary entered the hal/ API surface — this season would be baked in"
    warn "  Still a PROXY: read each NEW hit. A colour word can be innocent in a device API."
  else
    warn "every game-word hit in hal/ predates $WORKBASE_SHORT — not F1's, and each one has"
    warn "  already been judged; re-read them only at a release review."
    ok "F1 added no game-object vocabulary to the hal/ API surface"
  fi
else
  ok "no game-object vocabulary in the hal/ API surface (comments stripped; identifiers only)"
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

# (e) HA register: a new physical claim must take a FREE number. The brief says "from
#     HA-92" — but 92 is a fact about the register on the day the brief was written, and
#     hardcoding it here would silently rot the moment another chunk registers a claim.
#     THE REAL PROPERTY is collision: an id that already existed must not be re-used,
#     because re-using it overwrites someone else's falsifiable claim in place. So the
#     next-free number is DERIVED from the register at the content baseline, and the two
#     old failure modes (an id below the watermark; the wrong count) both fall out of it.
if git diff --quiet "$WORKBASE" -- docs/hardware-assumptions.md; then
  warn "hardware-assumptions.md UNCHANGED. T4 registers physical claims (does a Hold actually"
  warn "  hold a loaded cascade lift? what current does a jam draw?) as the next free HA-nn."
  warn "  Unchanged is only honest if F1 invented no threshold at all — verify by reading."
else
  ok "hardware-assumptions.md updated"
  head_of docs/hardware-assumptions.md > "$T/ha.base.md"
  python3 - "$T/ha.base.md" docs/hardware-assumptions.md <<'PY'
import re, sys
ids = lambda p: {int(m) for m in re.findall(r'\bHA-0*(\d+)\b', open(p, encoding='utf-8').read())}
old, new = ids(sys.argv[1]), ids(sys.argv[2])
nextfree = (max(old) + 1) if old else 1
added = sorted(new - old)
print(f"    register held {len(old)} ids at the baseline (highest HA-{max(old) if old else 0}); "
      f"next free was HA-{nextfree}")
print(f"    ids F1 added: {', '.join('HA-%d' % i for i in added) or 'none'}")
clash = [i for i in added if i < nextfree]        # cannot happen by construction, but
missing = sorted(old - new)                        # a DELETED id can, and it is worse
if missing:
    print(f"    *** ids that VANISHED from the register: {['HA-%d' % i for i in missing]} —")
    print("        a falsifiable claim was deleted rather than settled")
if not added:
    print("    *** the file changed but registered NO new id — if F1 invented a physical")
    print("        number, editing an existing entry in place overwrites another chunk's claim")
sys.exit(1 if (missing or clash or not added) else 0)
PY
  [ $? -eq 0 ] && ok "every new HA id is above the baseline watermark, and none was deleted" \
               || bad "HA register problem — see above (a re-used or deleted id destroys a claim)"
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
  del=$(git diff "$WORKBASE" --numstat -- "$f" | awk '{print $2}')
  add=$(git diff "$WORKBASE" --numstat -- "$f" | awk '{print $1}')
  if [ -z "${del:-}" ] || [ "${del:-0}" = "0" ]; then
    ok "$(basename "$f"): +${add:-0} / -0 — additive only"
  else
    warn "$(basename "$f"): +${add} / -${del} — LINES WERE REMOVED. Each deletion is a"
    warn "  claim that an existing use still 'means exactly what it means today'. Read them:"
    git diff "$WORKBASE" -- "$f" | grep -E '^-[^-]' | head -15 | cut -c1-130 | sed 's/^/        /'
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

# The CHUNK's mutation runner must exist and be gated on build success.
#
# THIS SCRIPT IS EXCLUDED FROM THE SEARCH, and that is the point: the old glob was
# `verify-f1*.sh`, which matches THIS FILE, and `grep -l` over several files succeeds if
# ANY of them matches. This harness traps PIPE and mentions BUILD-FAIL, so the gate would
# have gone green on the reviewer's own script even if the chunk had shipped no runner at
# all. A check that can be satisfied by the checker is not a check.
self=$(basename "$0")
runners=$(ls docs/internal/verify/verify-f1*.sh 2>/dev/null | grep -v "/$self\$" || true)
if [ -n "$runners" ]; then
  ok "a CHUNK-authored F1 mutation harness exists: $(echo "$runners" | tr '\n' ' ')"
  # shellcheck disable=SC2086
  grep -ql 'BUILD-FAIL' $runners \
    && ok "  it is gated on build success (C4's lesson)" \
    || bad "  it is NOT gated on build success — a non-compiling mutation reads green off a stale binary"
  # shellcheck disable=SC2086
  grep -ql 'PIPE' $runners \
    && ok "  it traps PIPE (E2 lost a header to SIGPIPE)" \
    || bad "  it does not trap PIPE"
else
  bad "no chunk-authored docs/internal/verify/verify-f1*.sh — the brief REQUIRES a mutation"
  bad "  runner, and this reviewer harness does not count as one"
fi

# ══════════════════════════════════════════════════════════════════════════════════
hdr "SUMMARY"
# ══════════════════════════════════════════════════════════════════════════════════
if [ $fail -eq 0 ]; then
  printf '\033[32mMECHANICAL GATES PASSED\033[0m — and that is the smaller half.\n'
else
  printf '\033[31mGATES FAILED\033[0m — and remember: a red gate is a QUESTION. For each one, ask\n'
  printf '  (a) does it predate %s?  (b) does it violate the real property or a proxy?\n' "$WORKBASE_SHORT"
fi
if [ $hfail -ne 0 ]; then
  printf '\033[35mHARNESS ERRORS were reported above\033[0m — a literal this script greps for was\n'
  printf '  reworded, a mutation would not apply, or a control did not fire. Those lines are\n'
  printf '  defects in THIS FILE and say nothing about F1. Fix them here, then re-run;\n'
  printf '  do not carry them into a review as findings.\n'
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

# 0 = clean · 1 = at least one real property FAILED · 2 = no FAIL, but this harness could
# not ask one or more of its questions. 2 is not a pass and not a finding: it is a bug
# report against this file.
if [ $fail -ne 0 ]; then exit 1; fi
if [ $hfail -ne 0 ]; then exit 2; fi
exit 0
