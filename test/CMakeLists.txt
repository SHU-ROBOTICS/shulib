# Host-side unit tests for shulib core (pure C++20, ZERO pros/ headers).
#
#   Configure:  cmake -S test -B build/test
#   Build:      cmake --build build/test
#   Run:        ./build/test/shulib_tests        (or: ctest --test-dir build/test)
#
# This build is completely separate from the PROS ARM Makefile. It compiles only
# the dependency-free core so we can catch bugs off-robot, fast.
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
# time; the CONFIGURE_DEPENDS on .git/HEAD re-runs it when the checked-out commit
# moves (a dirty-flag change without a commit can lag until the next reconfigure
# — an accepted, documented staleness for the HOST suite; the R1 robot Makefile
# evaluates per build).
execute_process(
  COMMAND git describe --always --dirty --abbrev=7
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
  OUTPUT_VARIABLE SHULIB_GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT SHULIB_GIT_HASH STREQUAL "")
  add_compile_definitions(SHULIB_BUILD_HASH="${SHULIB_GIT_HASH}")
  message(STATUS "shulib build hash: ${SHULIB_GIT_HASH}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/../.git/HEAD")
else()
  message(WARNING "shulib: git hash unavailable — SHULIB_BUILD_HASH not defined; "
                  "the session header will be LOUDLY missing (by design)")
endif()

enable_testing()

# Convention: every "*_test.cpp" in this directory is a test translation unit.
# test_main.cpp provides doctest's main() exactly once.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")

add_executable(shulib_tests test_main.cpp ${TEST_SOURCES})

# shulib headers (added as we build them); doctest as a system header.
target_include_directories(shulib_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(shulib_tests SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

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
target_include_directories(shulib_tests BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pros_shim")
target_compile_definitions(shulib_tests PRIVATE SHULIB_HOST_PROS_SHIM)

# The API-reference fidelity test reads the COMMITTED docs/api/ markdown and
# compares its rendered signatures against the headers, so it needs to know
# where the source tree is. (It fails loudly if it cannot open the files —
# a fidelity test that skips is decoration.)
target_compile_definitions(shulib_tests PRIVATE
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
#                   VERBATIM in a compiled example test. That rule has existed
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

# Every public document and every example source is an INPUT: editing a chapter
# must re-run the gates, or a drift introduced after the last build sits green
# until something unrelated forces a rebuild. CONFIGURE_DEPENDS so a new
# chapter or a new examples file is picked up without a manual cmake re-run.
file(GLOB SHULIB_DOC_INPUTS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../README.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/../docs/*.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/../docs/guide/*.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/../docs/cookbook/*.md"
  "${CMAKE_CURRENT_SOURCE_DIR}/*example*_test.cpp")
add_custom_command(
  OUTPUT "${SHULIB_DOC_STAMP}"
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" self-test
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-coverage
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-fresh
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-examples
  COMMAND "${Python3_EXECUTABLE}" "${SHULIB_DOC_TOOL}" check-removability
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../tools/briefing_status.py" check
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
  COMMAND "${CMAKE_COMMAND}" -E touch "${SHULIB_DOC_STAMP}"
  DEPENDS
    "${SHULIB_DOC_TOOL}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/shulib/chassis/chassis.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include/shulib/chassis/routine.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/api/chassis.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/api/routine.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/api/README.md"
    # The briefing gate's own subject and tool. Without these two the gate
    # re-ran ONLY when some unrelated document changed — so the briefing could
    # be edited into a lie, or drift as chunks completed, and the build stayed
    # green until something else happened to touch a chapter. A gate that does
    # not re-run when the thing it checks changes is not a gate. (Found while
    # briefing R1a: `briefing_status.py check` was failing at HEAD and the
    # build did not know.)
    "${CMAKE_CURRENT_SOURCE_DIR}/../docs/internal/PROJECT-BRIEFING.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/briefing_status.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/doc_staleness_audit.py"
    ${SHULIB_DOC_INPUTS}
  COMMENT "shulib doc gates: coverage, freshness, verbatim examples, removability"
  VERBATIM)
add_custom_target(shulib_doc_gates DEPENDS "${SHULIB_DOC_STAMP}")
add_dependencies(shulib_tests shulib_doc_gates)

add_test(NAME shulib_tests COMMAND shulib_tests)
