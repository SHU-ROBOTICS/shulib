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

add_test(NAME shulib_tests COMMAND shulib_tests)
