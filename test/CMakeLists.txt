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

enable_testing()

# Convention: every "*_test.cpp" in this directory is a test translation unit.
# test_main.cpp provides doctest's main() exactly once.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")

add_executable(shulib_tests test_main.cpp ${TEST_SOURCES})

# shulib headers (added as we build them); doctest as a system header.
target_include_directories(shulib_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(shulib_tests SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

add_test(NAME shulib_tests COMMAND shulib_tests)
