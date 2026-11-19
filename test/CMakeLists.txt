# Host-side unit tests for shulib core (pure C++20, ZERO pros/ headers).
#
#   Configure:  cmake -S test -B build/test
#   Build:      cmake --build build/test
#   Run:        ./build/test/shulib_tests        (or: ctest --test-dir build/test)
#
# This build is completely separate from the PROS ARM Makefile. It compiles only
# the dependency-free core so we can catch bugs off-robot, fast — the library's
# headers, the host tests, and (since COMP-t) ONE .cpp from src/: the team's
# PROS-free auton file, src/team/autons.cpp (see target_sources below).
cmake_minimum_required(VERSION 3.20)
project(shulib_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # emit compile_commands.json for clangd / editor tooling

# Strict warnings are themselves a bug-catching tool — errors on OUR code.
# (doctest.h is included as a SYSTEM header below so the framework's own
# internals never trip these.)
add_compile_options(
  -Wall -Wextra -Wpedantic -Werror
  -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion)

# ── The §18.5 build hash (chunk C5) ─────────────────────────────────────────────────
# The session header must say WHICH BINARY ran; code cannot know its own commit,
# so the BUILD defines it (diag/build_info.hpp carries the contract). `git
# describe --always --dirty`: --dirty is load-bearing — a clean-looking hash from
# a modified tree is a WRONG hash, and a wrong hash is worse than an absent one.
# If git is unavailable the macro is NOT defined and the loud MISSING path runs
# (test/session_header_test.cpp turns that into a visible failure here, where git
# does exist — never a silently plausible placeholder). Evaluated at CONFIGURE
# time; the CONFIGURE_DEPENDS below re-run it when the checked-out commit moves
# (a dirty-flag change without a commit can lag until the next reconfigure — an
# accepted, documented staleness for the HOST suite; the R1 robot Makefile
# evaluates per build).
#
# WHICH FILE MOVES WHEN THE COMMIT MOVES (fixed 2026-09-24, audit coord#0): this
# watched `.git/HEAD` alone until then, and on a branch that file holds
# `ref: refs/heads/<branch>` and does NOT change when a commit lands — so the
# configure never re-ran, and build/test carried `d9d0ee3-dirty` five commits
# after d9d0ee3. The reflog (`logs/HEAD`) is appended on EVERY move of HEAD —
# commit, checkout, reset, rebase, pull — so it is watched too. Both paths come
# from `git rev-parse --git-path`, which is also right inside a linked worktree.
execute_process(
  COMMAND git describe --always --dirty --abbrev=7
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
  OUTPUT_VARIABLE SHULIB_GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT SHULIB_GIT_HASH STREQUAL "")
  add_compile_definitions(SHULIB_BUILD_HASH="${SHULIB_GIT_HASH}")
  message(STATUS "shulib build hash: ${SHULIB_GIT_HASH}")
  foreach(_shulib_git_file HEAD logs/HEAD)
    execute_process(
      COMMAND git rev-parse --git-path ${_shulib_git_file}
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
      OUTPUT_VARIABLE _shulib_git_path
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    get_filename_component(_shulib_git_path "${_shulib_git_path}" ABSOLUTE
                           BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
    # A missing file (reflog disabled) is skipped rather than watched: a
    # configure dependency that does not exist breaks the generated build.
    if(EXISTS "${_shulib_git_path}")
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_shulib_git_path}")
    endif()
  endforeach()
else()
  message(WARNING "shulib: git hash unavailable — SHULIB_BUILD_HASH not defined; "
                  "the session header will be LOUDLY missing (by design)")
endif()

enable_testing()

# Convention: every "*_test.cpp" in this directory is a test translation unit.
# test_main.cpp provides doctest's main() exactly once.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")

add_executable(shulib_tests test_main.cpp ${TEST_SOURCES})

# THE src/ .cpp FILES THE HOST BUILDS (chunk COMP-t, 2026-09-28; two since ROB1-A): the team's autons,
# src/team/autons.cpp. It is PROS-free by design, and it is exactly what the robot
# runs — so the host suite runs every auton in it on the host test plant through
# the Auton Tester's own run sequence (test/autons_test.cpp), and a change to an
# auton is tested before it is uploaded. Every other src/ file stays robot-only
# (the programs include PROS); a PROS-free src/ header the host needs is included
# from src/ through the root below, never compiled here.
target_sources(shulib_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/autons.cpp")
# ...and, since chunk ROB1-A (2026-11-11), robot one's: src/team/robot_one_autons.cpp, PROS-free the
# same way (test/robot_one_autons_test.cpp runs it on the host test plant).
target_sources(shulib_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/robot_one_autons.cpp")
# ...and, since chunk ROB1-BT (2026-11-16), the skills route BOTH robots register: src/team/skills_route.cpp.
target_sources(shulib_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/skills_route.cpp")

# ── THE SUITE'S OWN FLAGS, as ONE INTERFACE target (chunk MECH-A1, 2026-10-08) ──────────────
# Every include root, option and define below used to be PRIVATE to shulib_tests. The mechanism
# core's compile-refusal harness (compile_refusals/, below) must compile its translation units
# with EXACTLY the suite's flags — a refusal proven under different flags proves nothing about the
# suite's — so the flags now live on `shulib_test_flags`, which both link: one definition, no copy
# to drift. (The warning set is the directory's add_compile_options above, which every target here
# already shares.) The suite's compile command is byte-identical to what it was (MECH-PROGRESS.md
# records the before/after diff of one TU's command).
add_library(shulib_test_flags INTERFACE)
target_link_libraries(shulib_tests PRIVATE shulib_test_flags)

# shulib headers (added as we build them); doctest as a system header.
target_include_directories(shulib_test_flags INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(shulib_test_flags SYSTEM INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

# src/'s own include root (chunk SRC1, 2026-09-27) — the same one the PROS Makefile gives the
# robot build (its EXTRA_INCDIR comment carries the whole reasoning). The tests that read a
# robot's tables or a program's pure helpers include them as src/ does, written FROM src/:
# `#include "robots/chassis_table.hpp"`, `#include "common/boot_source.hpp"`, never `../src/…`.
# -iquote (quoted includes only), not -I; the joined spelling keeps it ONE option, which CMake's
# option de-duplication cannot split. tools/src_build_gate.py's step 0 [include-root] fails any
# path two roots could both answer, so the search order never decides which file is meant.
target_compile_options(shulib_test_flags INTERFACE "-iquote${CMAKE_CURRENT_SOURCE_DIR}/../src")

# ── The host PROS shim (chunk R1a) ──────────────────────────────────────────────────
# hal/pros adapters #include <pros/*.hpp>. On the HOST they must resolve to
# test/pros_shim/'s hand-written programmable stand-ins, NOT the real vendored
# SDK headers under ../include/pros/ — BEFORE wins that race, in this build
# only. The PROS Makefile never sees this directory, and every shim header
# #errors unless SHULIB_HOST_PROS_SHIM is defined, which happens HERE and
# nowhere else: a shim that leaked into the robot include path would produce a
# binary that uploads cleanly, boots cleanly, prints a healthy banner, and
# drives NOTHING — the silent catastrophic failure T3 exists to prevent.
# (The shim tests the adapter against our BELIEF about PROS; it cannot test
# the belief — hardware tests the belief. pros_shim/pros/shim_control.hpp
# carries the full statement.)
target_include_directories(shulib_test_flags BEFORE INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/pros_shim")
target_compile_definitions(shulib_test_flags INTERFACE SHULIB_HOST_PROS_SHIM)

# The API-reference fidelity test reads the COMMITTED docs/api/ markdown and
# compares its rendered signatures against the headers, so it needs to know
# where the source tree is. (It fails loudly if it cannot open the files —
# a fidelity test that skips is decoration.)
target_compile_definitions(shulib_test_flags INTERFACE
  SHULIB_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")

# ── The documentation gates (chunk D3) ──────────────────────────────────────────────
# Two build-time checks, both run by tools/api_doc_tool.py:
#
#   check-coverage  a PUBLIC member of the documented headers with no ///
#                   comment fails the build, NAMING the member. This is the
#                   piece that makes "generated docs" mean "complete docs":
#                   a generator faithfully extracts whatever comments exist,
#                   so an undocumented member is silently absent and the
#                   reference still looks finished. Nothing looks wrong, which
#                   is worse than a stale document.
#   check-fresh     docs/api/ must be byte-identical to a fresh generation, so
#                   committed output cannot fall behind the headers between
#                   runs of the tool. (Also catches a hand-written file dropped
#                   into docs/api/, which is generated in full.)
#   check-examples  every ```cpp line in the public documentation must appear
#                   VERBATIM in a compiled example test — or, since T4 Part 1a,
#                   in a team file a chapter quotes from (a robot's driver file,
#                   src/team/*_driver.hpp, and since COMP-t the team's auton
#                   file, src/team/autons.cpp; the tool's EXAMPLE_SOURCE_GLOBS
#                   lists the kinds — and each listing of either must also be
#                   an unbroken run of it, WHOLE_RUN_SOURCE_GLOBS, so a line
#                   added to the file inside a listed part fails too). That rule has existed
#                   since C8 — but it lived only in an internal verify script,
#                   so until D3 a rotted example passed the build AND passed
#                   CI. Found by mutation; a rule nothing runs is a convention.
#   briefing-status the PROJECT-BRIEFING's generated block must match the repo.
#                   A handoff nobody can trust is worse than none: the next
#                   session believes it. It used to ALSO require a
#                   DURABLE-REVIEWED-AT stamp asserting a human had re-read the
#                   hand-written half; that was removed at R1a, because the chunk
#                   that trips the gate is the same actor that can edit the stamp
#                   silencing it, and a machine duly did. The narrative half is
#                   now honestly ungated — see doc_staleness_audit.py, which
#                   checks the mechanizable subset and says so.
#
#   check-removability  no public document may reference docs/internal/, which
#                   is dropped when this branch squash-merges to main. Same
#                   story: the property was real, the enforcement was manual.
#
# Plus self-test, which exercises the tool's own parser against fixtures —
# including the case where a member would otherwise inherit its neighbour's
# comment, the hole the coverage rule exists to close.
#
# WHY THIS IS A BUILD STEP AND NOT A doctest CASE: a /// comment is invisible
# to the compiler, so the check has to read the source text. Re-implementing
# the parser in C++ would create a SECOND definition of "documented", and the
# two would disagree eventually, silently. One parser, wired where it cannot
# be forgotten. (The library itself is header-only and does not depend on this
# tool in any way; only the host TEST build runs it.)
#
# Python is REQUIRED, deliberately: a gate that silently disables itself when
# an interpreter is missing is exactly the failure mode this chunk exists to
# remove. There is no opt-out flag for the same reason.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(NOT Python3_Interpreter_FOUND)
  message(FATAL_ERROR
    "python3 was not found, and the host test build needs it for the "
    "documentation gates (tools/api_doc_tool.py): a public member with no "
    "documentation must fail the build, and the generated API reference in "
    "docs/api/ must be verified up to date. Install python3 (Ubuntu: "
    "sudo apt-get install -y python3) and re-run cmake. There is no opt-out: "
    "a gate you can switch off is not a gate.")
endif()

set(SHULIB_DOC_TOOL "${CMAKE_CURRENT_SOURCE_DIR}/../tools/api_doc_tool.py")
set(SHULIB_DOC_STAMP "${CMAKE_CURRENT_BINARY_DIR}/doc_gates.stamp")
set(SHULIB_READ_TOOL "${CMAKE_CURRENT_SOURCE_DIR}/../tools/doc_readability.py")

# Every public document and every example source is an INPUT: editing a chapter
# must re-run the gates, or a drift introduced after the last build sits green
# until something unrelated forces a rebuild. CONFIGURE_DEPENDS so a new
# chapter or a new examples file is picked up without a manual cmake re-run.
# The example sources are the tool's EXAMPLE_SOURCE_GLOBS, each kind here too:
# the example tests, (since T4 Part 1a) a robot's driver file, which the
# cookbook quotes part by part, and (since COMP-t) the team's auton file, which
# guide chapter 16 quotes — an edit to either must re-run the verbatim scan.
file(GLOB SHULIB_DOC_INPUTS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../README.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/*example*_test.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/*_driver.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/autons.cpp")
# Every public page, in ANY folder under docs/, as check-nav and check-tone read them (DOCS3-N1's
# review: a page in a new folder, missing from a hand-kept list, did not re-run the gate when edited).
file(GLOB_RECURSE SHULIB_DOC_PUBLIC_PAGES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../docs/*.md")
list(FILTER SHULIB_DOC_PUBLIC_PAGES EXCLUDE REGEX "/docs/internal/")
list(APPEND SHULIB_DOC_INPUTS ${SHULIB_DOC_PUBLIC_PAGES})

# ── EVERY header and EVERY generated page is an input (chunk DOCS2) ──────────
# Until DOCS2 the gate depended on two headers and three pages, because those
# were the whole reference. It now covers 115 headers and 117 pages, and a
# DEPENDS list naming two of them would mean editing any OTHER header left the
# coverage and freshness gates un-run until something unrelated forced a
# rebuild — a member added with no `///` sitting green for a whole session.
# That is this very file's own lesson, three comments down: "a gate that does
# not re-run when the thing it checks changes is not a gate."
#
# Globbed with CONFIGURE_DEPENDS rather than listed, for the same reason the
# tool's own target list is a glob: a header added later must be covered
# because it exists, not because someone remembered.
file(GLOB_RECURSE SHULIB_DOC_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../include/shulib/*.hpp")
file(GLOB SHULIB_DOC_PAGES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../docs/api/*.md")
# ── docs/internal/ is DROPPED on the published branch, and this file used to
# forget that ────────────────────────────────────────────────────────────────
# DOCS1 found `main` unable to build its own test suite:
#
#   No rule to make target '.../docs/internal/PROJECT-BRIEFING.md',
#   needed by 'doc_gates.stamp'.  Stop.
#
# The briefing was listed as a hard DEPENDS so the gate would re-run when its
# own subject changed — correct on the working branch, fatal on the released
# one, where `docs/internal/` does not exist at all. C7's removability rule says
# nothing outside that directory may reference into it; `check-removability`
# never caught this because it scans public *.md and this is a build file.
#
# So the dependency is now conditional, and the CHECK is too. `briefing_status.py
# check` already exits 0 when the briefing is absent, but relying on that would
# leave the build silently running a gate against nothing; skipping it out loud
# is the honest form. On the working branch nothing changes — the briefing is
# there, the dependency is real, and the gate still re-runs when it is edited.
set(SHULIB_BRIEFING "${CMAKE_CURRENT_SOURCE_DIR}/../docs/internal/PROJECT-BRIEFING.md")
if(EXISTS "${SHULIB_BRIEFING}")
  set(SHULIB_BRIEFING_DEP "${SHULIB_BRIEFING}")
  set(SHULIB_BRIEFING_CMD COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/../tools/briefing_status.py" check)
else()
  set(SHULIB_BRIEFING_DEP "")
  set(SHULIB_BRIEFING_CMD COMMAND "${CMAKE_COMMAND}" -E echo
      "briefing gate: SKIPPED — docs/internal/ is absent (this is the published tree)")
endif()

add_custom_command(
  OUTPUT "${SHULIB_DOC_STAMP}"
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" self-test
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-coverage
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-fresh
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-examples
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-removability
  # The briefing tool's chunk/part rule (COMP-t's merge, 2026-09-29) decides its position line;
  # its self-test needs no briefing, so it runs on the published tree too.
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/briefing_status.py" self-test
  ${SHULIB_BRIEFING_CMD}
  # staleness  the four gates above check that documentation EXISTS and is
  #            well-formed; none has an opinion about whether a sentence is
  #            still TRUE. This one checks the mechanizable subset: a figure a
  #            document states versus what the repo says, two documents
  #            disagreeing, and a named file that does not exist. Added at R1a,
  #            replacing the DURABLE-REVIEWED-AT stamp — which claimed to force a
  #            human re-read and could not, because the chunk that trips the gate
  #            is the same actor that can silence it. This checks less and means
  #            it. Its self-test runs first, because a gate nobody has watched go
  #            red is a rumour.
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/doc_staleness_audit.py" self-test
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/doc_staleness_audit.py"
  # reading (DOCS3-N1): every public page in the site's menu exactly once; no page says
  # "honest" or opens with a status disclaimer; the front page's first-week picture still
  # matches guide 21's order; the site copy anchors every register row.
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_READ_TOOL}" self-test
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_READ_TOOL}" check-nav
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_READ_TOOL}" check-tone
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/docs_figures/first_week.py" self-test
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/docs_figures/first_week.py" check
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/docs_figures/build_to_program.py" self-test
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/docs_figures/build_to_program.py" check
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/prepare_site.py" self-test
  COMMAND "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/../tools/prepare_site.py" check-register
  COMMAND "${CMAKE_COMMAND}" -E touch "${SHULIB_DOC_STAMP}"
  DEPENDS
    "${SHULIB_DOC_TOOL}"
    ${SHULIB_DOC_HEADERS}
    ${SHULIB_DOC_PAGES}
    "${CMAKE_CURRENT_SOURCE_DIR}/../mkdocs.yml"
    # The briefing gate's own subject and tool. Without these two the gate
    # re-ran ONLY when some unrelated document changed — so the briefing could
    # be edited into a lie, or drift as chunks completed, and the build stayed
    # green until something else happened to touch a chapter. A gate that does
    # not re-run when the thing it checks changes is not a gate. (Found while
    # briefing R1a: `briefing_status.py check` was failing at HEAD and the
    # build did not know.)
    ${SHULIB_BRIEFING_DEP}
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/briefing_status.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/doc_staleness_audit.py"
    # The bounded suite run both of them use (2026-10-22): a change to its bounds re-runs the gate.
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/suite_run.py"
    # The reading checks and the files they read beyond the pages (DOCS3-N1).
    "${SHULIB_READ_TOOL}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/docs_figures/first_week.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/assets/first-week.svg"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/docs_figures/build_to_program.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/assets/build-to-program.svg"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/prepare_site.py"
    ${SHULIB_DOC_INPUTS}
  COMMENT "shulib doc gates: coverage, freshness, verbatim examples, removability, staleness, reading"
  VERBATIM)
add_custom_target(shulib_doc_gates DEPENDS "${SHULIB_DOC_STAMP}")
add_dependencies(shulib_tests shulib_doc_gates)

add_test(NAME shulib_tests COMMAND shulib_tests)

# ── THE ROUTE PREVIEW (chunk RPV1-A, 2026-11-12) ────────────────────────────────────────────────────
# One named auton from the team's REAL autons file through the REAL library on the host test plant,
# drawn as one offline page (test/route_preview_run.hpp; run it with tools/route_preview.py, which
# builds this target and runs it under the suite's own bounds). Built by the default target, so CI
# compiles it; it does NOT wait for the doc gates. route_preview_main.cpp is not a *_test.cpp, so the
# suite's glob above never compiles it — the suite calls runPreview() itself (route_preview_test.cpp).
add_executable(shulib_route_preview route_preview_main.cpp "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/autons.cpp"
                                    "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/robot_one_autons.cpp"
                                    "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/skills_route.cpp")  # ROB1-BT: robot two registers the shared skills route
target_link_libraries(shulib_route_preview PRIVATE shulib_test_flags)

# ── THE COMPILE-REFUSAL HARNESS (chunk MECH-A1, 2026-10-08; the MECH brief's §3.0) ──────────
# The mechanism core refuses a robot file's mistakes AT BUILD, each by name (a consteval check calls
# a function whose NAME is the reason). A refusal no test can see is one a mutation can delete, so
# every refusal a robot file can reach has a translation unit in compile_refusals/ that must FAIL
# to compile, naming its reason (all but the two unreachable "used twice" fallbacks in
# mechanism/wiring.hpp — test/README.md says which); each is a CTest (`ctest -R compile_refusal`), run by CI beside the suite. The TUs are NOT
# part of shulib_tests (they are EXCLUDE_FROM_ALL object targets): compile_refusals/CMakeLists.txt
# and run_refusal.py say how a TU is judged.
add_subdirectory(compile_refusals)

# ── THE ROUTE BUILDER'S NATIVE PROGRAM (chunk RMP1-A1, 2026-11-14; the RMP1 brief A.3.3 [W9]) ─────────────
# `shulib_route_maker describe --robot one|two|x|y|z` writes a robot's description (its REAL tables, as the
# route builder's page reads them) and `curves` the CV1 curve golden; CI's route-builder job builds this ONE
# target and bakes robots one and two into the page (tools/route_maker/build.py --describe). Host only, the
# suite's own flags (shulib_test_flags), NOT part of shulib_tests and not needed by it (the suite includes
# the same headers). Since RMP1-A2e, `preview` too: the runner (route_maker_runner.hpp) — the SAME main the
# WebAssembly module is built from (tools/route_maker/wasm/), so WG2 compares the two builds' records. Both
# drop the repository's path from every file name the code keeps (W10: -ffile-prefix-map, the module's flag).
# Neither compiles src/team/: the engine runs the builder's route, never the team's autons
# (route_maker_no_team_autons.cpp).
add_executable(shulib_route_maker route_maker_main.cpp route_maker_no_team_autons.cpp)
target_link_libraries(shulib_route_maker PRIVATE shulib_test_flags)
get_filename_component(SHULIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_compile_options(shulib_route_maker PRIVATE "-ffile-prefix-map=${SHULIB_ROOT}/=")
