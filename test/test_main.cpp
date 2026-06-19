// Provides doctest's main() for the whole test binary — exactly once.
// All actual tests live in *_test.cpp files; this TU stays empty on purpose
// so the framework is compiled a single time.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
