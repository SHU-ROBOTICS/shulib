// printText() — the robot programs' one path to the brain's screen (src/common/screen_text.hpp) —
// RUN through the host PROS shim's recording screen (test/pros_shim/pros/screen.h), chunk CAL1
// Part B (2026-11-05; the CAL1 brief's §2.B test 6, Re-check R7). Since Part B, printText()'s body is
// the LIBRARY's sanitizer, hal::screenSafeLine(), and nothing of its own: these cases pin what
// reaches PROS — the exact bytes, the order of the three C calls, the literal "%s" format, the
// print counter — so a printText() that went back to a sanitizer of its own (one that let a '%'
// through, or cut at the wrong budget) is red here as well as in test/brain_screen_test.cpp's S2.
// NOTHING HERE HAS RUN ON A ROBOT: the shim tests the program against OUR BELIEF about PROS
// (vsnprintf reads a '%' that reaches print_at as a conversion, as we believe VEXos does).
//
// Each case names the bug it catches:
//   X1  the sanitizer's cases through printText: '%' and a control byte become '?', an em dash
//       and an en dash '-', a 3-byte character and a 2-byte one ONE '?', the cut at each size's
//       budget, the pad never past it          (printText's own copy of the rule, '%' let through)
//   X2  the C calls, in order: the eraser, then the pen, then print_at through a LITERAL "%s" —
//       never the text as a format — at the size's PROS format and the given place
//   X3  the counter: g_screenPrints counts every print, once each; the shim's own count agrees

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "common/screen_text.hpp"
#include "pros/screen.h"
#include "shulib/calibration/calibrate_panel.hpp"

using shulib::bench::g_screenPrints;
using shulib::bench::printText;
using shulib::calibration::budgetFor;
using shulib::calibration::TextSize;

namespace {

/// The text of the shim's LAST draw, which must be one print, or "<no print>".
std::string lastPrinted() {
    const auto& log = pros::shim::screenState().log;
    if (log.empty()) {
        return "<no print>";
    }
    const std::string& l = log.back();
    const auto at = l.find(" '");
    if (l.rfind("print ", 0) != 0 || at == std::string::npos || l.back() != '\'') {
        return "<no print>";
    }
    return l.substr(at + 2, l.size() - at - 3);
}

/// printText at SMALL with no pad, on a clean screen; what reached the screen.
std::string through(const char* text, TextSize size = TextSize::Small, std::size_t pad = 0) {
    pros::shim::resetScreen();
    printText(size, 8, 30, 0xF0F0F0, 0x101418, text, pad);
    return lastPrinted();
}

}  // namespace

TEST_CASE("printText X1: the library's sanitizer, through printText -- every rule, each size's cut and "
          "the pad") {
    // '%' — the one that matters: a '%' at print_at would be read as a conversion.
    CHECK(through("50% done") == "50? done");
    CHECK(through("%s%d%%") == "?s?d??");
    // control bytes: a tab, a newline, DEL
    CHECK(through("a\tb\nc\x7F" "d") == "a?b?c?d");
    // an em dash and an en dash become '-' (the adapters' precondition messages carry them)
    CHECK(through("a \xE2\x80\x94 b \xE2\x80\x93 c") == "a - b - c");
    // any other non-ASCII character becomes ONE '?': a 3-byte euro sign, a 2-byte e-acute, a 4-byte
    // emoji — never one '?' per byte
    CHECK(through("\xE2\x82\xAC" "1") == "?1");
    CHECK(through("caf\xC3\xA9") == "caf?");
    CHECK(through("\xF0\x9F\x98\x80!") == "?!");
    // a truncated sequence at the very end reads nothing past the terminator
    CHECK(through("x\xE2") == "x?");
    // a null text prints ""
    CHECK(through(nullptr) == "");
    // the cut at each size's budget, exactly at the edge (50 SMALL, 36 MEDIUM, 18 LARGE)
    for (const TextSize size : {TextSize::Small, TextSize::Medium, TextSize::Large}) {
        CAPTURE(static_cast<int>(size));
        const std::size_t b = budgetFor(size);
        const std::string exact(b, 'x');
        const std::string over(b + 1, 'y');
        CHECK(through(exact.c_str(), size) == exact);
        CHECK(through(over.c_str(), size) == std::string(b, 'y'));
        // the pad: to padTo characters, never past the budget
        CHECK(through("ab", size, 5) == "ab   ");
        CHECK(through("ab", size, b + 10) == "ab" + std::string(b - 2, ' '));
        CHECK(through(exact.c_str(), size, b + 10) == exact);
    }
    CHECK(budgetFor(TextSize::Small) == 50);
    CHECK(budgetFor(TextSize::Medium) == 36);
    CHECK(budgetFor(TextSize::Large) == 18);
    // the rule is the library's, byte for byte: the same text through screenSafeLine
    char lib[shulib::hal::kScreenLineBytes];
    const char* const hard = "50% \xE2\x80\x94 \xE2\x82\xAC\t ok";
    (void)shulib::hal::screenSafeLine(lib, sizeof lib, hard, budgetFor(TextSize::Medium), 30);
    CHECK(through(hard, TextSize::Medium, 30) == std::string{lib});
}

TEST_CASE("printText X2: the eraser, then the pen, then print_at through a LITERAL \"%s\" -- the text is "
          "never the format") {
    pros::shim::resetScreen();
    printText(TextSize::Large, 12, 40, 0x39C36E, 0x101418, "100% READY");
    const auto& calls = pros::shim::screenState().calls;
    REQUIRE(calls.size() == 4);  // set_eraser, set_pen, print_at's format, its draw
    CHECK(calls[0] == "set_eraser 101418");
    CHECK(calls[1] == "set_pen 39c36e");
    CHECK(calls[2] == "print_at format '%s'");
    CHECK(calls[3] == "print L 12 40 pen=39c36e eraser=101418 '100? READY'");
    // each size at its PROS format letter (S, M, L — never a centred format)
    for (const auto& [size, letter] : std::vector<std::pair<TextSize, std::string>>{
             {TextSize::Small, "S"}, {TextSize::Medium, "M"}, {TextSize::Large, "L"}}) {
        CAPTURE(letter);
        pros::shim::resetScreen();
        printText(size, 3, 4, 0xFFFFFF, 0x000000, "x");
        REQUIRE(pros::shim::screenState().log.size() == 1);
        CHECK(pros::shim::screenState().log[0] == "print " + letter + " 3 4 pen=ffffff eraser=000000 'x'");
    }
}

TEST_CASE("printText X3: g_screenPrints counts every print, once each, and agrees with the screen's own "
          "count") {
    pros::shim::resetScreen();
    const std::uint32_t before = g_screenPrints;
    for (int i = 0; i < 7; ++i) {
        printText(TextSize::Medium, 8, 30, 0xFFFFFF, 0x000000, i % 2 == 0 ? "%" : "");
    }
    CHECK(g_screenPrints - before == 7);
    CHECK(pros::shim::screenState().prints == 7);  // an empty text is still a print
}
