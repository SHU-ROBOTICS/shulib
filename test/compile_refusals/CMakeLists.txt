# THE COMPILE-REFUSAL HARNESS (chunk MECH-A1, 2026-10-08; the MECH brief's §3.0).
#
# One translation unit per BUILD-TIME refusal of the mechanism core a robot file can reach (the two
# "used twice" fallbacks in mechanism/wiring.hpp, which no list can reach, have none): each must
# FAIL to compile, and GCC's diagnostics must NAME its reason — the text of its `// expect:`
# line(s), matched as a fixed string in a diagnostic's MESSAGE (never its path), and, where it
# carries `// expect-at: <line>`, GCC must cite that line of the TU (how a refusal's LOCATION is
# pinned, not only its words).
# run_refusal.py is the judge; its docstring states the rules.
#
# Every *.cpp here IS a test: a TU dropped into this directory is picked up at the next configure
# (CONFIGURE_DEPENDS) — a refusal cannot be added and then forgotten by the harness. Each TU is its
# own EXCLUDE_FROM_ALL object target (never part of `all`, never of shulib_tests), linked to
# shulib_test_flags so it compiles with EXACTLY the suite's include roots, defines and warnings; its
# CTest builds that one target (`cmake --build <dir> --target cr_<stem>`). The tests share a
# RESOURCE_LOCK on the build tree: two `make` runs in one tree at once can race on its bookkeeping,
# so `ctest -j` still runs these one at a time.
#
# THE HARNESS PROVES ITSELF: the TUs named harness_*.cpp are WILL_FAIL — each is a way the judge could
# be fooled (a TU that compiles; a TU refused for a DIFFERENT reason; a TU whose reason text appears
# only in its own FILE NAME, which every make and GCC line carries; a TU refused for its reason but at
# a line GCC does not cite — the `// expect-at:` rule's own test, MECH-A1's review), and the judge must
# say FAIL for each, or the CTest is red. Run them:  ctest --test-dir build/test -R compile_refusal
#
# WHERE A REFUSAL LANDS is pinned (`// expect-at:`) wherever two layers could raise the same name:
# mechanisms() re-checks every entry on its own (its spec, its wiring — so a hand-built entry is held
# to wire()'s rules), so a wire()-level refusal that stopped firing would still be refused, by the
# same name, at the list's FIRST line. Those TUs put one wire(...) per line and pin the wire line.
#
# NAMING: <neg5 id or kind>_<what>.cpp — the brief's neg5 numbers (n01..n63, m1..m3) where the probe
# set had the case. The three refusals that are plain C++ errors, not named ones, are said so in
# their TUs (n01: a misspelt mechanism; n07: `90_deg`; n09: a run-time value where a constant is
# checked) and match GCC's own words.
#
# GROWN BY EVERY RUN (the brief's §3.0): MECH-A2 adds until.hpp's, MECH-A4 driver_inputs.hpp's and the
# Routine's, MECH-A5 DriverFile's and the robot table file's.

function(shulib_compile_refusal stem)
  set(target "cr_${stem}")
  add_library(${target} OBJECT EXCLUDE_FROM_ALL "${CMAKE_CURRENT_SOURCE_DIR}/${stem}.cpp")
  target_link_libraries(${target} PRIVATE shulib_test_flags)
  add_test(NAME "compile_refusal_${stem}"
           COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/run_refusal.py"
                   --build-dir "${CMAKE_BINARY_DIR}" --target "${target}"
                   --source "${CMAKE_CURRENT_SOURCE_DIR}/${stem}.cpp")
  set_tests_properties("compile_refusal_${stem}" PROPERTIES RESOURCE_LOCK shulib_build_tree)
  if(stem MATCHES "^harness_")
    set_tests_properties("compile_refusal_${stem}" PROPERTIES WILL_FAIL TRUE)
  endif()
endfunction()

file(GLOB SHULIB_REFUSAL_TUS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(SORT SHULIB_REFUSAL_TUS)
foreach(tu IN LISTS SHULIB_REFUSAL_TUS)
  get_filename_component(stem "${tu}" NAME_WE)
  shulib_compile_refusal("${stem}")
endforeach()
