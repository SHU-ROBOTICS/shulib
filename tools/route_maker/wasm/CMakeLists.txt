# THE PREVIEW ENGINE (chunk RMP1-A2e; the RMP1 brief A.3.3): the REAL library, the host test plant, RPV1's engine
# and the runner (test/route_maker_runner.hpp), compiled to WebAssembly from the ONE main the native
# `shulib_route_maker` is built from (test/route_maker_main.cpp, W9). Emscripten 6.0.13, pinned (README.md):
#
#   emcmake cmake -S tools/route_maker/wasm -B build/wasm && cmake --build build/wasm
#
# build/wasm/ then holds (never committed — CI builds them on every push):
#   preview.mjs + preview.wasm  the SITE form (an ES module and its .wasm)
#   preview_single.mjs          the ONE-FILE form (the same .wasm inside, base64)
#   preview_test.mjs + .wasm    the TEST module: also every golden's exported C++ compiled in (WG1) and the
#                               runner's test-only looping step (WG7)
# The suite's warning flags with -Werror (Emscripten's Clang is a second compiler gate on the library, W14), the
# legacy Wasm exceptions the probes proved (P-W1b; W6), and the repository's path dropped from every file name the
# code keeps (W10). The include roots are spelled as test/CMakeLists.txt spells them, so a kept file name is the
# same text natively and here (WG2).
cmake_minimum_required(VERSION 3.20)
project(shulib_route_maker_wasm CXX)
if(NOT EMSCRIPTEN)
  message(FATAL_ERROR "the preview engine is built with Emscripten: emcmake cmake -S tools/route_maker/wasm -B build/wasm")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-Os")  # no NDEBUG: the native program defines none either

get_filename_component(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(T "${ROOT}/test")

add_library(rm_flags INTERFACE)
target_compile_options(rm_flags INTERFACE
  -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion
  -fwasm-exceptions "-ffile-prefix-map=${ROOT}/=" "-iquote${T}/../src")
target_include_directories(rm_flags BEFORE INTERFACE "${T}/pros_shim")
target_include_directories(rm_flags INTERFACE "${T}/../include")
target_include_directories(rm_flags SYSTEM INTERFACE "${T}/vendor")
target_compile_definitions(rm_flags INTERFACE SHULIB_HOST_PROS_SHIM)
# callMain leaves its arguments on the C stack, so the page restores the stack after each call
# (src/host/engine_module.mjs: stackSave/stackRestore). A run needs 16-32 KB of stack (measured over every golden and
# 300 sweep routes): 1 MB, and an overflow ABORTS rather than silently corrupting a record (~5 % slower).
target_link_options(rm_flags INTERFACE
  -Os -fwasm-exceptions -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createPreview -sINVOKE_RUN=0
  -sEXPORTED_RUNTIME_METHODS=callMain,stackSave,stackRestore -sALLOW_MEMORY_GROWTH=1 -sMAXIMUM_MEMORY=256MB
  -sSTACK_SIZE=1MB -sSTACK_OVERFLOW_CHECK=2 -sENVIRONMENT=web,worker,node)

# Not src/team/: the engine runs the builder's route, never the team's autons (that file says why).
add_library(rm_no_team OBJECT "${T}/route_maker_no_team_autons.cpp")
target_link_libraries(rm_no_team PRIVATE rm_flags)
add_library(rm_main OBJECT "${T}/route_maker_main.cpp")
target_link_libraries(rm_main PRIVATE rm_flags)

add_executable(preview $<TARGET_OBJECTS:rm_main> $<TARGET_OBJECTS:rm_no_team>)
add_executable(preview_single $<TARGET_OBJECTS:rm_main> $<TARGET_OBJECTS:rm_no_team>)
target_link_options(preview_single PRIVATE -sSINGLE_FILE=1 -sSINGLE_FILE_BINARY_ENCODE=0)  # base64 (P-W2)
add_executable(preview_test "${T}/route_maker_main.cpp" "${T}/route_maker_goldens.cpp" $<TARGET_OBJECTS:rm_no_team>)
target_compile_definitions(preview_test PRIVATE SHULIB_ROUTE_MAKER_TEST_STEPS)
foreach(t preview preview_single preview_test)
  target_link_libraries(${t} PRIVATE rm_flags)
  set_target_properties(${t} PROPERTIES SUFFIX ".mjs")
endforeach()
