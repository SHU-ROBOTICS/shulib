// The brain screen seam and the standard Calibrate's painter (chunk CAL1 Part A1, the brief's §2.A
// test 7): hal::screenSafeLine (the screen's sanitizer), hal::fake::FakeBrainScreen, and
// calibration::CalibratePainter with the program's own screens, all on the fake.
// NOTHING HERE HAS RUN ON A ROBOT, and no pixel here has been seen on a V5: the budgets and places
// are ESTIMATES (A4 register HA-136). What these cases prove is the painter's LOGIC — what it
// draws, in which colour, padded how far, and when it draws nothing.
//
// Each case names the bug it catches; the mutations that must turn it red are in the brief
// (§2.A mutations 16, 17) and in CAL1-PROGRESS.md (Part A1), each observed red on a fresh binary:
//   S1  the sanitizer's cases: '%', control bytes and DEL, both dashes, other 2-, 3- and 4-byte
//       characters (one '?' each), a stray continuation byte, a sequence cut by the terminator,
//       the cut at `cols` and at the buffer, the pad (never past the cut), null text, cap 0 and 1
//                                                        ('%' passed through)
//   S2  the sanitizer and the robot programs' printText() (src/common/screen_text.hpp, through the
//       host PROS shim) give the SAME bytes for the same text, size and pad — a fixed corpus and
//       5,000 random byte strings: two copies of one rule until CAL1 Part B, which made printText()
//       CALL the sanitizer (test/screen_text_test.cpp pins that path); this case now holds a copy
//       from coming back
//   S3  FakeBrainScreen records each call in the shim's words, sanitizes, counts prints; its touch:
//       a press moves the place and not the release count, a release the count; an error read is
//       PROS's (every field -1), never the last tap
//   P1  the painter's first render, call for call, written out by hand from today's program's
//       painter: the content erased, the header bar and its two texts, each line in its tone's
//       colour, each button's fill, double edge and centred label
//   P2  nothing changed → nothing drawn; a change of screen, of a line's place (down or sideways)
//       or size, or of the line count → a full repaint
//   P3  a line's COLOUR change is painted at once, alone, padded to the old text — and recorded:
//       the same view for the next second draws nothing, and a later text pads to the new one
//   P4  a live number is repainted at most every kLiveRepaintSeconds — 1 s of a number changing
//       every 10 ms tick paints that line at most 6 times              (the throttle removed)
//   P5  the header is repainted on a tone or word change, and only the header
//   P6  a vanished button is erased; a re-armed, moved or re-labelled one is painted whole; a kept
//       one is not touched; each kind's colours
//   P7  the program's own screens: starting, cannot start (complete, cut, and each of its five
//       texts over budget), not driver control, the competition-state screen (its three states'
//       words, and a cut state, reason or status said) — each line, place and colour; every text
//       a caller passes that a screen checks against a budget, at EXACTLY that budget in characters
//       drawn (more in bytes) drawn whole and NOT said, and one character over said; each painted
//       over a RENDERED view leaves the painter's record stale, so the same view re-rendered
//       repaints in full
//   P8  the header's build time is cut at kBuildTimeCols and its word at kHeaderCols; a string_view
//       that is not NUL-terminated is painted to its end and no further, and one longer than the
//       copy buffer is copied a line's worth only; render() cuts a line and a button label at
//       their size's budget WITHOUT a word on screen; a '%' in a view's line reaches the screen
//       as '?'
//   P9  every literal of calibration::screen_text is in its kAll list (read from the header's
//       source) with the budget of the size its comment states — and of the size the painter
//       draws it at — and fits it; and the compile-time check itself refuses what it must (at
//       compile time: a word one character over, a '%', a non-ASCII byte, a reason that does not
//       wrap onto the state screen's two lines, a wrapped shape no screen draws, a list with one
//       bad entry anywhere), and the header runs it over kAll in a static_assert

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "common/screen_text.hpp"  // printText(): the robot programs' copy of the rule (S2)
#include "pros/screen.h"           // the host PROS shim's screen (S2 reads what printText printed)
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/calibration/calibrate_screen.hpp"
#include "shulib/hal/brain_screen.hpp"
#include "shulib/hal/fake/fake_brain_screen.hpp"

namespace {

using shulib::calibration::budgetFor;
using shulib::calibration::CalibratePainter;
using shulib::calibration::kBuildTimeCols;
using shulib::calibration::kColAbortFill;
using shulib::calibration::kColAmber;
using shulib::calibration::kColBg;
using shulib::calibration::kColBlue;
using shulib::calibration::kColEdge;
using shulib::calibration::kColGreen;
using shulib::calibration::kColGrey;
using shulib::calibration::kColGreyFill;
using shulib::calibration::kColRed;
using shulib::calibration::kColText;
using shulib::calibration::kLiveRepaintSeconds;
using shulib::calibration::PanelButton;
using shulib::calibration::PanelButtonId;
using shulib::calibration::PanelLine;
using shulib::calibration::PanelRect;
using shulib::calibration::PanelScreen;
using shulib::calibration::PanelTone;
using shulib::calibration::PanelView;
using shulib::calibration::TextSize;
using shulib::hal::screenSafeLine;
using shulib::hal::ScreenTextSize;
using shulib::hal::fake::FakeBrainScreen;
namespace screen_text = shulib::calibration::screen_text;

/// screenSafeLine into a fresh buffer, as a string.
std::string safe(const char* text, std::size_t cols, std::size_t padTo = 0,
                 std::size_t cap = shulib::hal::kScreenLineBytes) {
    std::vector<char> out(cap + 8, '#');
    const std::size_t n = screenSafeLine(out.data(), cap, text, cols, padTo);
    const std::string s{out.data()};
    CHECK(n == s.size());
    return s;
}

/// The log lines from index `from` on.
std::vector<std::string> since(const FakeBrainScreen& s, std::size_t from) {
    return {s.log().begin() + static_cast<std::ptrdiff_t>(from), s.log().end()};
}

/// A line of a view.
PanelLine line(std::int16_t x, std::int16_t y, TextSize size, PanelTone tone, const char* text) {
    PanelLine l;
    l.x = x;
    l.y = y;
    l.size = size;
    l.tone = tone;
    std::snprintf(l.text, sizeof l.text, "%s", text);
    return l;
}

/// A view: a screen, a header, lines and buttons.
PanelView view(PanelScreen screen, PanelTone tone, const char* word, std::vector<PanelLine> lines,
               std::vector<PanelButton> buttons) {
    PanelView v;
    v.screen = screen;
    v.headerTone = tone;
    v.headerWord = word;
    REQUIRE(lines.size() <= shulib::calibration::kPanelMaxLines);
    REQUIRE(buttons.size() <= shulib::calibration::kPanelMaxButtons);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        v.lines[i] = lines[i];
    }
    v.lineCount = static_cast<std::uint8_t>(lines.size());
    for (std::size_t i = 0; i < buttons.size(); ++i) {
        v.buttons[i] = buttons[i];
    }
    v.buttonCount = static_cast<std::uint8_t>(buttons.size());
    return v;
}

PanelButton button(PanelButtonId id, PanelRect r, const char* label, bool armed = true) {
    PanelButton b;
    b.id = id;
    b.rect = r;
    b.label = label;
    b.armed = armed;
    return b;
}

constexpr PanelRect kDoneRect{6, 180, 300, 234};    // kPrimarySlot
constexpr PanelRect kAbortRect{330, 180, 474, 234};  // kAbortSlot

/// The push screen used through P1-P6: two lines (an instruction and a live number), DONE, ABORT.
PanelView pushView(const char* number = "12.0 in", PanelTone numberTone = PanelTone::Green,
                   bool doneArmed = true) {
    return view(PanelScreen::Push, PanelTone::Blue, "2/5 PUSH",
                {line(8, 30, TextSize::Large, PanelTone::White, "PUSH IT"),
                 line(8, 102, TextSize::Large, numberTone, number)},
                {button(PanelButtonId::Done, kDoneRect, "DONE", doneArmed),
                 button(PanelButtonId::Abort, kAbortRect, "ABORT")});
}

std::string hex(std::uint32_t c) {
    char b[16];
    std::snprintf(b, sizeof b, "%06x", static_cast<unsigned>(c));
    return b;
}

/// The text printText() handed to the shim's screen_print_at, from its last recorded draw — or
/// "<no print>" when the last draw is not a print of the expected shape (no assertion here: S2
/// calls it 75,000 times and counts the mismatches itself).
std::string lastShimPrintText() {
    const auto& log = pros::shim::screenState().log;
    if (log.empty()) {
        return "<no print>";
    }
    const std::string& l = log.back();
    const auto at = l.find(" '");
    if (l.rfind("print ", 0) != 0 || at == std::string::npos || l.size() < at + 3
        || l.back() != '\'') {
        return "<no print>";
    }
    return l.substr(at + 2, l.size() - at - 3);
}

/// screenSafeLine into a fresh buffer, as a string — the S2 loop's form, with no assertion.
std::string safeQuiet(const char* text, std::size_t cols, std::size_t padTo) {
    char out[shulib::hal::kScreenLineBytes];
    screenSafeLine(out, sizeof out, text, cols, padTo);
    return out;
}

std::string readSource(const char* rel) {
    std::ifstream in(std::filesystem::path(SHULIB_SOURCE_DIR) / rel);
    REQUIRE_MESSAGE(in.good(), "cannot open ", rel);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

TEST_CASE("BrainScreen S1: the sanitizer -- format characters, control bytes, non-ASCII, the cut "
          "and the pad") {
    // BUG CAUGHT: a '%' passed to VEXos (BELIEVED read as a conversion with nothing behind it —
    // hal/brain_screen.hpp's header; never measured); a control byte or a non-ASCII byte drawn as
    // garbage; a multi-byte character turned into several '?' (it would push the line past its
    // budget); a dash turned into '?'; a pad that runs past the budget (an in-place update would
    // overwrite the next field); a read past the terminator.
    CHECK(safe("50% done", 50) == "50? done");
    CHECK(safe("%s%d%%", 50) == "?s?d??");
    CHECK(safe("a\tb\nc\x01" "d\x1F" "e\x7F" "f", 50) == "a?b?c?d?e?f");
    CHECK(safe("a \xE2\x80\x94 b", 50) == "a - b");      // em dash
    CHECK(safe("1\xE2\x80\x93" "2", 50) == "1-2");       // en dash
    CHECK(safe("wait\xE2\x80\xA6", 50) == "wait?");      // an ellipsis: not a dash, ONE '?'
    CHECK(safe("caf\xC3\xA9!", 50) == "caf?!");          // 2-byte
    CHECK(safe("5 \xE2\x82\xAC each", 50) == "5 ? each");  // 3-byte (the euro sign)
    CHECK(safe("ok\xF0\x9F\x98\x80ok", 50) == "ok?ok");  // 4-byte
    CHECK(safe("a\x80\xBF" "b", 50) == "ab");            // stray continuation bytes: dropped
    CHECK(safe("a\xE2", 50) == "a?");                    // cut by the terminator after the lead
    CHECK(safe("a\xE2\x80", 50) == "a?");                // ... and after one continuation byte
    // A dash or a '?' is ONE column: the cut counts characters drawn, not bytes read.
    CHECK(safe("a\xE2\x80\x94" "bcdef", 3) == "a-b");
    CHECK(safe("\xC3\xA9\xC3\xA9\xC3\xA9\xC3\xA9", 2) == "??");
    // The cut, the pad, and the pad never past the cut.
    CHECK(safe("ABCDEFGHIJ", 4) == "ABCD");
    CHECK(safe("AB", 10, 5) == "AB   ");
    CHECK(safe("AB", 4, 9) == "AB  ");
    CHECK(safe("", 3, 3) == "   ");
    CHECK(safe("ABCDEF", 0, 5).empty());
    // The buffer cuts too: cap 6 holds 5 characters and the terminator, whatever `cols` says.
    CHECK(safe("ABCDEFGHIJ", 50, 0, 6) == "ABCDE");
    CHECK(safe("AB", 50, 40, 6) == "AB   ");
    // A null text writes ""; a cap of 1 writes only the terminator.
    CHECK(safe(nullptr, 10).empty());
    CHECK(safe(nullptr, 10, 3) == "   ");
    CHECK(safe("ABC", 10, 0, 1).empty());
    // Cap 0 and a null buffer write nothing at all.
    char guard[4] = {'x', 'y', 'z', '\0'};
    CHECK(screenSafeLine(guard, 0, "ABC", 10) == 0);
    CHECK(std::string{guard} == "xyz");
    CHECK(screenSafeLine(nullptr, 10, "ABC", 10) == 0);
}

TEST_CASE("BrainScreen S2: the sanitizer and the robot programs' printText() give the same bytes "
          "for the same text, size and pad") {
    // BUG CAUGHT: two copies of the rule drifting apart — until CAL1 Part B printText() carried its
    // own; since Part B it calls screenSafeLine(), and this holds a second copy from coming back
    // unseen — a library screen and a program screen would then draw the same text differently, and a
    // host test of one would say nothing about the other.
    using shulib::bench::printText;
    std::vector<std::string> corpus = {
        "",
        "plain text",
        "50% done -- \xE2\x80\x94 \xE2\x80\x93 a\tb \xE2\x82\xAC end",
        "\x01\x02\x7F%%%",
        "\xF0\x9F\x98\x80 \xC3\xA9 \x80 \xE2",
        "a line much longer than any budget the brain has, so every size cuts it somewhere: "
        "LARGE, MEDIUM and SMALL",
        std::string(60, '%'),
        std::string(60, '\xE2'),
    };
    // 5,000 random byte strings (a fixed seed, so a failure reproduces), weighted toward the bytes
    // the rule treats specially.
    std::uint32_t x = 0xC0FFEEu;
    const auto next = [&x]() {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return x;
    };
    const unsigned char special[] = {'%', 0x01, 0x1F, 0x7F, 0x80, 0x93, 0x94, 0xBF,
                                     0xC0, 0xC3, 0xE2, 0xF0, 0xFF, ' ', 'a', 'Z'};
    for (int n = 0; n < 5000; ++n) {
        std::string s;
        const std::uint32_t len = next() % 70;
        for (std::uint32_t i = 0; i < len; ++i) {
            const std::uint32_t r = next();
            unsigned char c = (r & 1u) != 0 ? special[(r >> 1) % sizeof special]
                                            : static_cast<unsigned char>(1 + (r >> 8) % 255);
            s.push_back(static_cast<char>(c));
        }
        corpus.push_back(s);
    }
    const TextSize sizes[] = {TextSize::Small, TextSize::Medium, TextSize::Large};
    const std::size_t pads[] = {0, 5, 18, 36, 60};
    pros::shim::resetScreen();
    std::size_t compared = 0;
    std::size_t differ = 0;
    for (const std::string& s : corpus) {
        for (const TextSize size : sizes) {
            for (const std::size_t pad : pads) {
                pros::shim::screenState().log.clear();
                printText(size, 8, 30, 0xFFFFFF, 0x000000, s.c_str(), pad);
                const std::string program = lastShimPrintText();
                const std::string library = safeQuiet(s.c_str(), budgetFor(size), pad);
                if (program != library && ++differ <= 5) {
                    FAIL_CHECK("the two sanitizers differ for size ", static_cast<int>(size),
                               " pad ", pad, ": printText '", program, "', screenSafeLine '",
                               library, "'");
                }
                ++compared;
            }
        }
    }
    CHECK(differ == 0);
    CHECK(compared == corpus.size() * 15);
    CHECK(compared > 75000);
    // The comparison can fail: a text the two must draw differently from the raw bytes.
    printText(TextSize::Small, 8, 30, 0xFFFFFF, 0x000000, "50%");
    CHECK(lastShimPrintText() == "50?");
    CHECK(lastShimPrintText() != std::string{"50%"});
}

TEST_CASE("BrainScreen S3: FakeBrainScreen records every call in the shim's words, sanitizes, "
          "counts its prints, and reads the touch as PROS does") {
    // BUG CAUGHT: a fake that records the text BEFORE the sanitizer (a host test would then pass a
    // '%' the robot is believed to choke on); colours recorded in another format than the shim's
    // (the parity test against the PROS adapter could not compare); a tap that does not move the
    // release count (a release-edge reader would never see it); a press that moves it (a reader
    // acting on finger-DOWN would pass); an error read that keeps the last tap where PROS answers
    // -1 in every field (a reader that takes its touch baseline from an error read sees a phantom
    // tap on the robot — the next good read is a "new" release at the last place — and nothing on
    // the fake; the adapter test's A5b holds the two equal read for read).
    FakeBrainScreen s;
    s.fillRect(1, 2, 3, 4, 0x39C36E);
    s.outlineRect(5, 6, 7, 8, 0x0000FF);
    s.eraseRect(9, 10, 11, 12, 0x101418);
    s.eraseAll(0x000000);
    s.print(ScreenTextSize::Small, 10, 20, 0xFFFFFF, 0x000000, "50% \xE2\x80\x94 ok", 50);
    s.print(ScreenTextSize::Medium, -1, 300, 0xABCDEF, 0x123456, "padded", 36, 9);
    s.print(ScreenTextSize::Large, 0, 0, 0x1, 0x2, "CUT AT FOUR", 4);
    const std::vector<std::string> want = {
        "fill 1 2 3 4 pen=39c36e",
        "rect 5 6 7 8 pen=0000ff",
        "erase 9 10 11 12 eraser=101418",
        "erase-all eraser=000000",
        "print S 10 20 pen=ffffff eraser=000000 '50? - ok'",
        "print M -1 300 pen=abcdef eraser=123456 'padded   '",
        "print L 0 0 pen=000001 eraser=000002 'CUT '",
    };
    CHECK(s.log() == want);
    CHECK(s.prints() == 3);

    // The touch: nothing yet; a press moves the place and NOT the release count; the release
    // moves the count; a tap is both.
    CHECK_FALSE(s.touch().error);
    CHECK(s.touch().releaseCount == 0);
    s.press(30, 40);
    CHECK_FALSE(s.touch().error);
    CHECK(s.touch().releaseCount == 0);
    CHECK(s.touch().x == 30);
    CHECK(s.touch().y == 40);
    s.release();
    CHECK(s.touch().releaseCount == 1);
    s.tap(240, 120);
    CHECK(s.touch().releaseCount == 2);
    CHECK(s.touch().x == 240);
    CHECK(s.touch().y == 120);
    // An error read is PROS's: the flag, and every other field -1 — never the last tap.
    s.setTouchError(true);
    const shulib::hal::ScreenTouch failed = s.touch();
    CHECK(failed.error);
    CHECK(failed.releaseCount == -1);
    CHECK(failed.x == -1);
    CHECK(failed.y == -1);
    s.tap(10, 20);  // a tap while the reads fail still counts ...
    CHECK(s.touch().releaseCount == -1);
    s.setTouchError(false);  // ... and the first good read shows it, and nothing else
    CHECK_FALSE(s.touch().error);
    CHECK(s.touch().releaseCount == 3);
    CHECK(s.touch().x == 10);
    CHECK(s.touch().y == 20);

    // The observer sees every recorded line as it is recorded.
    std::vector<std::string> seen;
    s.observe(
        [](const std::string& l, void* ctx) {
            static_cast<std::vector<std::string>*>(ctx)->push_back(l);
        },
        &seen);
    s.eraseAll(0x111111);
    s.print(ScreenTextSize::Small, 1, 1, 0, 0, "x", 50);
    CHECK(seen == std::vector<std::string>{"erase-all eraser=111111",
                                           "print S 1 1 pen=000000 eraser=000000 'x'"});
    s.observe(nullptr, nullptr);
    s.eraseAll(0x222222);
    CHECK(seen.size() == 2);
}

TEST_CASE("CalibratePainter P1: the first render, call for call -- the content erased, the header, "
          "each line in its tone's colour, each button's fill, double edge and centred label") {
    // BUG CAUGHT: a palette entry changed (a tone drawn in another colour); the header's word or
    // build time moved or drawn in a colour that does not read on the bar; a button's edge drawn
    // once; a label not centred at the MEDIUM budget's width; the content erased in the wrong
    // colour or not below the header bar; the order of the draws changed (the PROS calls would
    // then differ from R3d's program's). Written out by hand from R3d's program's painter
    // (src/programs/calibrate_program.cpp until CAL1 Part B, paintHeader / eraseContent / paintButton /
    // render).
    FakeBrainScreen s;
    CalibratePainter p{s, "BUILD 12:34:56"};
    p.render(pushView(), 0.0);
    const std::vector<std::string> want = {
        "erase 0 27 479 235 eraser=101418",
        "fill 0 0 479 26 pen=1e5aa8",
        "print M 8 4 pen=f0f0f0 eraser=1e5aa8 '2/5 PUSH'",
        "print S 340 7 pen=f0f0f0 eraser=1e5aa8 'BUILD 12:34:56'",
        "print L 8 30 pen=f0f0f0 eraser=101418 'PUSH IT'",
        "print L 8 102 pen=39c36e eraser=101418 '12.0 in'",
        // DONE (kPrimarySlot 6,180-300,234): green fill, light edge twice, dark label centred:
        // width 295, 4 x 13 px = 52 → x 6 + (295 - 52) / 2 = 127; y 180 + (55 - 20) / 2 = 197.
        "fill 6 180 300 234 pen=39c36e",
        "rect 6 180 300 234 pen=c8d8f0",
        "rect 7 181 299 233 pen=c8d8f0",
        "print M 127 197 pen=101418 eraser=39c36e 'DONE'",
        // ABORT (kAbortSlot 330,180-474,234): dark red fill, red edge, white label:
        // width 145, 5 x 13 = 65 → x 330 + 40 = 370.
        "fill 330 180 474 234 pen=b0302a",
        "rect 330 180 474 234 pen=e2564a",
        "rect 331 181 473 233 pen=e2564a",
        "print M 370 197 pen=f0f0f0 eraser=b0302a 'ABORT'",
    };
    CHECK(s.log() == want);
    CHECK(s.prints() == 6);

    // Every tone's text colour and every header tone's bar and ink, one line / one header each.
    struct ToneCase {
        PanelTone tone;
        std::uint32_t text, bar, ink;
    };
    const ToneCase tones[] = {
        {PanelTone::Amber, kColAmber, kColAmber, kColBg}, {PanelTone::Green, kColGreen, kColGreen, kColBg},
        {PanelTone::Red, kColRed, kColRed, kColBg},       {PanelTone::Grey, kColGrey, kColGreyFill, kColText},
        {PanelTone::Blue, kColEdge, kColBlue, kColText},  {PanelTone::White, kColText, kColBlue, kColText},
    };
    for (const ToneCase& t : tones) {
        FakeBrainScreen one;
        CalibratePainter q{one, "B"};
        q.render(view(PanelScreen::Saved, t.tone, "W", {line(8, 70, TextSize::Small, t.tone, "x")}, {}),
                 0.0);
        REQUIRE(one.log().size() == 5);
        CHECK(one.log()[1] == "fill 0 0 479 26 pen=" + hex(t.bar));
        CHECK(one.log()[2] == "print M 8 4 pen=" + hex(t.ink) + " eraser=" + hex(t.bar) + " 'W'");
        CHECK(one.log()[4] == "print S 8 70 pen=" + hex(t.text) + " eraser=101418 'x'");
    }
}

TEST_CASE("CalibratePainter P2: nothing changed draws nothing; a change of screen, of a line's "
          "place or size, or of the line count repaints everything") {
    // BUG CAUGHT: a repaint every tick (each print is BELIEVED to leak — calibrate_screen.hpp's
    // header; never measured); a screen change painted by difference (the old screen's lines and
    // buttons would stay where the new one has none); a moved line printed at its new place with
    // the old one left behind.
    FakeBrainScreen s;
    CalibratePainter p{s, "BUILD 12:34:56"};
    p.render(pushView(), 0.0);
    const std::size_t full = s.log().size();
    REQUIRE(full == 14);
    for (int k = 1; k <= 100; ++k) {
        p.render(pushView(), k / 100.0);
    }
    CHECK(s.log().size() == full);  // 100 identical ticks: nothing

    // The same content on another screen: a full repaint.
    PanelView other = pushView();
    other.screen = PanelScreen::Spin;
    std::size_t at = s.log().size();
    p.render(other, 2.0);
    REQUIRE(since(s, at).size() == full);
    CHECK(since(s, at).front() == "erase 0 27 479 235 eraser=101418");

    // A line moved down 2 px: a full repaint.
    PanelView moved = other;
    moved.lines[1].y = 104;
    at = s.log().size();
    p.render(moved, 3.0);
    REQUIRE(since(s, at).size() == full);
    CHECK(since(s, at).front() == "erase 0 27 479 235 eraser=101418");

    // A line moved 2 px SIDEWAYS (only x): a full repaint too — not the line reprinted at its new
    // place with the old text left beside it.
    PanelView sideways = moved;
    sideways.lines[1].x = 10;
    at = s.log().size();
    p.render(sideways, 3.5);
    REQUIRE(since(s, at).size() == full);
    CHECK(since(s, at).front() == "erase 0 27 479 235 eraser=101418");

    // A line's size changed: a full repaint.
    PanelView resized = sideways;
    resized.lines[1].size = TextSize::Medium;
    at = s.log().size();
    p.render(resized, 4.0);
    CHECK(since(s, at).size() == full);

    // One line fewer: a full repaint (and the dropped line is not drawn).
    PanelView fewer = resized;
    fewer.lineCount = 1;
    at = s.log().size();
    p.render(fewer, 5.0);
    REQUIRE(since(s, at).size() == full - 1);
    CHECK(since(s, at).front() == "erase 0 27 479 235 eraser=101418");

    // invalidate(): the same view again repaints everything.
    p.invalidate();
    at = s.log().size();
    p.render(fewer, 6.0);
    CHECK(since(s, at).size() == full - 1);
}

TEST_CASE("CalibratePainter P3: a line's COLOUR change is painted at once, alone, padded to the old "
          "text -- inside the live-number throttle") {
    // BUG CAUGHT: a colour change waiting for the throttle (the amber "not yet" turning green would
    // lag the push by up to 0.2 s); a colour change repainting the whole screen; an in-place
    // update that erases first (a flicker) or does not cover the old, longer text.
    FakeBrainScreen s;
    CalibratePainter p{s, "B"};
    p.render(pushView("12.0 in", PanelTone::Amber), 0.0);
    const std::size_t at = s.log().size();
    p.render(pushView("9 in", PanelTone::Green), 0.01);  // 10 ms later: the colour changed
    const std::vector<std::string> want = {"print L 8 102 pen=39c36e eraser=101418 '9 in   '"};
    CHECK(since(s, at) == want);
    // ... and RECORDED as painted: the same view for the next second of 10 ms ticks draws nothing.
    // (A record left on the old colour would reprint the line every tick, unthrottled — the
    // believed leak the throttle exists to bound, reached by another path.)
    const std::size_t mark = s.log().size();
    for (int k = 2; k <= 101; ++k) {
        p.render(pushView("9 in", PanelTone::Green), k / 100.0);
    }
    CHECK(s.log().size() == mark);
    // Its TEXT is recorded too: a shorter number later is padded to "9 in" (4 characters), not to
    // the "12.0 in" (7) the colour change painted over.
    p.render(pushView("8", PanelTone::Green), 1.5);
    REQUIRE(s.log().size() == mark + 1);
    CHECK(s.log().back() == "print L 8 102 pen=39c36e eraser=101418 '8   '");
}

TEST_CASE("CalibratePainter P4: a live number is repainted at most every kLiveRepaintSeconds, "
          "padded to the old text") {
    // BUG CAUGHT (mutation 16): the throttle removed — a number reprinted every 10 ms tick, and
    // PROS 4.2.2's print is BELIEVED to leak its buffer on every call (calibrate_screen.hpp's
    // header; read, never measured): megabytes over a few runs, if so.
    REQUIRE(kLiveRepaintSeconds == doctest::Approx(0.2));
    FakeBrainScreen s;
    CalibratePainter p{s, "B"};
    p.render(pushView("0.0 in"), 0.0);
    const std::size_t at = s.log().size();
    char number[32];
    for (int k = 1; k <= 100; ++k) {  // 1 s, the number changing every 10 ms tick
        std::snprintf(number, sizeof number, "%d.%d in", k / 10, k % 10);
        p.render(pushView(number), k / 100.0);
    }
    const auto drawn = since(s, at);
    // At most one paint per 0.2 s after the first: 5 in 1 s (0.2, 0.4, 0.6, 0.8, 1.0), allowing one
    // for the tick grid's rounding; and at least 4 (it does repaint).
    CHECK(drawn.size() <= 6);
    CHECK(drawn.size() >= 4);
    for (const std::string& l : drawn) {
        CHECK(l.rfind("print L 8 102 pen=39c36e eraser=101418 '", 0) == 0);  // only that line
    }
    // Times exactly representable: 0.125 s after a paint is too soon; 0.25 s is not.
    FakeBrainScreen t;
    CalibratePainter q{t, "B"};
    q.render(pushView("1234 in"), 0.5);
    std::size_t mark = t.log().size();
    q.render(pushView("5 in"), 0.625);
    CHECK(t.log().size() == mark);  // throttled
    q.render(pushView("5 in"), 0.75);
    REQUIRE(t.log().size() == mark + 1);
    CHECK(t.log().back() == "print L 8 102 pen=39c36e eraser=101418 '5 in   '");  // padded to 7
    // The throttle is per line: the next change waits 0.2 s from THIS paint, not the first.
    mark = t.log().size();
    q.render(pushView("6 in"), 0.875);
    CHECK(t.log().size() == mark);
    q.render(pushView("6 in"), 1.0);
    CHECK(t.log().size() == mark + 1);
    // The edge itself (the coordinator's CM13, 2026-10-15): a change EXACTLY kLiveRepaintSeconds
    // after a paint is repainted — "at most every", so never sooner, but not later either. A paint
    // at 0.0 makes the difference exactly kLiveRepaintSeconds, so the edge is really reached.
    FakeBrainScreen e;
    CalibratePainter r{e, "B"};
    r.render(pushView("1 in"), 0.0);
    mark = e.log().size();
    r.render(pushView("2 in"), kLiveRepaintSeconds);
    CHECK(e.log().size() == mark + 1);
}

TEST_CASE("CalibratePainter P5: the header is repainted on a tone or word change, and only the "
          "header") {
    // BUG CAUGHT: a header that keeps the old colour when the state changes (amber "powered" is
    // the stand-clear warning); a header change that repaints the lines (wasted prints, each
    // believed to leak) or that does not repaint the build time over the new bar colour.
    FakeBrainScreen s;
    CalibratePainter p{s, "BUILD 12:34:56"};
    PanelView v = pushView();
    p.render(v, 0.0);
    std::size_t at = s.log().size();
    v.headerTone = PanelTone::Amber;
    p.render(v, 0.01);
    CHECK(since(s, at) == std::vector<std::string>{
                              "fill 0 0 479 26 pen=e8b23a",
                              "print M 8 4 pen=101418 eraser=e8b23a '2/5 PUSH'",
                              "print S 340 7 pen=101418 eraser=e8b23a 'BUILD 12:34:56'"});
    at = s.log().size();
    v.headerWord = "3/5 PULSES";
    p.render(v, 0.02);
    CHECK(since(s, at) == std::vector<std::string>{
                              "fill 0 0 479 26 pen=e8b23a",
                              "print M 8 4 pen=101418 eraser=e8b23a '3/5 PULSES'",
                              "print S 340 7 pen=101418 eraser=e8b23a 'BUILD 12:34:56'"});
    // The same word in ANOTHER buffer (compared by text, not by pointer): nothing.
    static const char copy[] = "3/5 PULSES";
    v.headerWord = copy;
    at = s.log().size();
    p.render(v, 0.03);
    CHECK(s.log().size() == at);
}

TEST_CASE("CalibratePainter P6: buttons -- a vanished one is erased, a changed one painted whole, a "
          "kept one untouched; each kind's colours") {
    // BUG CAUGHT: a vanished button left on the screen (a tap there would look live and do
    // nothing); a greyed button that stays grey once armed; a kept button repainted every tick;
    // ABORT, DONE or a greyed button in the wrong colours.
    FakeBrainScreen s;
    CalibratePainter p{s, "B"};
    p.render(pushView("12.0 in", PanelTone::Green, false), 0.0);  // DONE greyed (arming)
    // The greyed DONE: grey fill, grey edge, grey label.
    CHECK(s.log()[6] == "fill 6 180 300 234 pen=" + hex(kColGreyFill));
    CHECK(s.log()[7] == "rect 6 180 300 234 pen=" + hex(kColGrey));
    CHECK(s.log()[9] == "print M 127 197 pen=" + hex(kColGrey) + " eraser=" + hex(kColGreyFill)
                            + " 'DONE'");
    // DONE arms: DONE painted whole, ABORT untouched.
    std::size_t at = s.log().size();
    p.render(pushView("12.0 in", PanelTone::Green, true), 0.5);
    CHECK(since(s, at) == std::vector<std::string>{
                              "fill 6 180 300 234 pen=39c36e", "rect 6 180 300 234 pen=c8d8f0",
                              "rect 7 181 299 233 pen=c8d8f0",
                              "print M 127 197 pen=101418 eraser=39c36e 'DONE'"});
    // DONE vanishes: its place erased to the ground, ABORT untouched.
    PanelView noDone = pushView();
    noDone.buttons[0] = noDone.buttons[1];
    noDone.buttonCount = 1;
    at = s.log().size();
    p.render(noDone, 0.6);
    CHECK(since(s, at) == std::vector<std::string>{"erase 6 180 300 234 eraser=101418"});
    // ABORT moves (the powered phases widen it to the whole row): old place erased, new painted.
    PanelView wide = noDone;
    wide.buttons[0].rect = PanelRect{6, 180, 474, 234};
    at = s.log().size();
    p.render(wide, 0.7);
    const auto moved = since(s, at);
    REQUIRE(moved.size() == 5);
    CHECK(moved[0] == "erase 330 180 474 234 eraser=101418");
    CHECK(moved[1] == "fill 6 180 474 234 pen=b0302a");
    // A RUN AGAIN box: blue fill, light edge, white label, centred in kRunAgainBox:
    // width 229, 9 x 13 = 117 → x 246 + 56 = 302; height 47 → y 112 + 13 = 125.
    PanelView done = view(PanelScreen::Saved, PanelTone::Green, "SAVED", {},
                          {button(PanelButtonId::RunAgain, PanelRect{246, 112, 474, 158},
                                  "RUN AGAIN")});
    at = s.log().size();
    p.render(done, 1.0);
    const auto again = since(s, at);
    REQUIRE(again.size() == 8);
    CHECK(again[4] == "fill 246 112 474 158 pen=" + hex(kColBlue));
    CHECK(again[5] == "rect 246 112 474 158 pen=" + hex(kColEdge));
    CHECK(again[7] == "print M 302 125 pen=f0f0f0 eraser=1e5aa8 'RUN AGAIN'");
    // A label too wide to centre sits 4 px in.
    PanelView narrow = view(PanelScreen::Saved, PanelTone::Green, "SAVED", {},
                            {button(PanelButtonId::RunAgain, PanelRect{100, 100, 120, 140},
                                    "RUN AGAIN")});
    at = s.log().size();
    p.render(narrow, 2.0);
    CHECK(since(s, at).back().rfind("print M 104 110 ", 0) == 0);
    // A re-labelled button in the same place is painted whole.
    PanelView relabelled = narrow;
    relabelled.buttons[0].label = "AGAIN";
    at = s.log().size();
    p.render(relabelled, 3.0);
    REQUIRE(since(s, at).size() == 4);
    CHECK(since(s, at).back().find("'AGAIN'") != std::string::npos);
}

TEST_CASE("CalibratePainter P7: the program's own screens, line by line; each, painted over a "
          "rendered view, leaves that view's next render() a full repaint") {
    // BUG CAUGHT: a refusal screen whose words overrun the screen (a why of three MEDIUM lines
    // and a fix of two SMALL ones, at their places); a cut text that does not say the rest is on
    // the serial log — the why or the fix, and the header, headline or footer a caller passed, or
    // the not-driver-control screen's status bits; a state screen drawn on the calibration ground
    // instead of black, or whose state name, reason or status is cut without a word; any of the
    // six budget checks (paintCannot's header, headline and footer, the not-driver-control
    // screen's bits, the state screen's name and bits) off by one either way — a text of EXACTLY
    // its budget said to be cut (the operator sent to the serial log for text that was never cut,
    // and "the census is on the serial log" lost), or one character over it cut without a word —
    // or counting BYTES, not characters drawn; any own screen that leaves the painter's record VALID —
    // the next render() of the view it painted over would then draw nothing over a screen it no
    // longer matches. Each own screen is painted over a RENDERED view for that reason: on a
    // painter that never rendered, the record is stale already and the bug cannot show.
    FakeBrainScreen s;
    CalibratePainter p{s, "BUILD 12:34:56"};
    // Render the view, then the same view again after `paint`: a full repaint (14 draws).
    const auto repaintsInFullAfter = [&](const char* what, double now) {
        INFO(what);
        const std::size_t at = s.log().size();
        p.render(pushView(), now);
        CHECK(since(s, at).size() == 14);  // full, not "nothing changed"
    };

    p.render(pushView(), 0.0);
    std::size_t at = s.log().size();
    p.paintStarting();
    CHECK(since(s, at) == std::vector<std::string>{
                              "erase 0 27 479 235 eraser=101418", "fill 0 0 479 26 pen=1e5aa8",
                              "print M 8 4 pen=f0f0f0 eraser=1e5aa8 'CALIBRATE'",
                              "print S 340 7 pen=f0f0f0 eraser=1e5aa8 'BUILD 12:34:56'",
                              "print L 8 30 pen=f0f0f0 eraser=101418 'HANDS OFF'",
                              "print L 8 66 pen=f0f0f0 eraser=101418 'STARTING UP'",
                              "print M 8 110 pen=9aa4ae eraser=101418 'finding the motors and the IMU'"});
    repaintsInFullAfter("paintStarting", 1.0);

    // A refusal whose texts fit: three MEDIUM lines at most, two SMALL (both used here, at 146
    // and 162), the footer, then where to look ("the census is on the serial log").
    at = s.log().size();
    p.paintCannot(screen_text::kHeaderCannot, PanelTone::Red, screen_text::kTooFewMotors,
                  "the census found 1 motor; a tank needs at least two drive motors",
                  "re-seat the drive cables, check that every motor's light is green, and run again",
                  screen_text::kNothingPowered);
    CHECK(since(s, at) == std::vector<std::string>{
                              "erase 0 27 479 235 eraser=101418", "fill 0 0 479 26 pen=e2564a",
                              "print M 8 4 pen=101418 eraser=e2564a 'CANNOT START'",
                              "print S 340 7 pen=101418 eraser=e2564a 'BUILD 12:34:56'",
                              "print L 8 34 pen=e2564a eraser=101418 'TOO FEW MOTORS'",
                              "print M 8 76 pen=f0f0f0 eraser=101418 'the census found 1 motor; a tank'",
                              "print M 8 98 pen=f0f0f0 eraser=101418 'needs at least two drive motors'",
                              "print S 8 146 pen=e8b23a eraser=101418 're-seat the drive cables, check that every motor's'",
                              "print S 8 162 pen=e8b23a eraser=101418 'light is green, and run again'",
                              "print S 8 186 pen=9aa4ae eraser=101418 'nothing is powered -- fix it, then restart it'",
                              "print S 8 202 pen=9aa4ae eraser=101418 'the census is on the serial log'"});
    repaintsInFullAfter("paintCannot", 2.0);
    // A why too long for three lines: what fitted, then "the full text is on the serial log".
    at = s.log().size();
    p.paintCannot(screen_text::kHeaderCannot, PanelTone::Red, screen_text::kFactsBad,
                  "one two three four five six seven eight nine ten eleven twelve thirteen "
                  "fourteen fifteen sixteen seventeen eighteen nineteen twenty",
                  nullptr, screen_text::kNothingPowered);
    const auto cut = since(s, at);
    CHECK(cut.back() == "print S 8 202 pen=9aa4ae eraser=101418 'the full text is on the serial log'");
    int whyLines = 0;
    for (const std::string& l : cut) {
        whyLines += std::regex_search(l, std::regex{"^print M 8 (76|98|120) "}) ? 1 : 0;
    }
    CHECK(whyLines == 3);
    // A word longer than a SMALL line in the fix is never split: cut, and said.
    at = s.log().size();
    p.paintCannot(screen_text::kHeaderCannot, PanelTone::Red, screen_text::kImuCount, "short",
                  "x23456789012345678901234567890123456789012345678901234567890",
                  screen_text::kNothingPowered);
    CHECK(since(s, at).back()
          == "print S 8 202 pen=9aa4ae eraser=101418 'the full text is on the serial log'");
    // A header, headline or footer a caller passed that is longer than its budget: cut, and said
    // — each alone, with a why and a fix that fit.
    const std::string_view overBudget[3][3] = {
        {"A HEADER WORD MUCH TOO LONG", screen_text::kTooFewMotors, screen_text::kNothingPowered},
        {screen_text::kHeaderCannot, "A HEADLINE TOO LONG FOR LARGE", screen_text::kNothingPowered},
        {screen_text::kHeaderCannot, screen_text::kTooFewMotors,
         "a footer of more than fifty characters, which SMALL cannot hold"},
    };
    for (const auto& o : overBudget) {
        FakeBrainScreen one;
        CalibratePainter q{one, "B"};
        q.paintCannot(o[0], PanelTone::Red, o[1], "short", "short", o[2]);
        CHECK_MESSAGE(one.log().back()
                          == "print S 8 202 pen=9aa4ae eraser=101418 'the full text is on the serial log'",
                      o[0], " | ", o[1], " | ", o[2]);
    }
    // Each text a caller passes, at EXACTLY its budget in characters drawn, and one character over
    // it: at the budget it is drawn whole and NOT said to be cut; one over, it is cut back to the
    // same characters and said. Each exact text carries an em dash (one character drawn, three
    // bytes), so it is over its budget in BYTES. The header word's budget is kHeaderCols (24), the
    // headline's LARGE's (18), the footer's SMALL's (50).
    const auto drawn = [](std::string t) {  // the em dash drawn as '-'
        return t.replace(t.find("\xE2\x80\x94"), 3, "-");
    };
    const std::string exactHeader = "STEP 9 OF 9 \xE2\x80\x94 CHECK DONE";  // 24 characters, 26 bytes
    const std::string exactHeadline = "FRONT \xE2\x80\x94 UNCHECKED!";      // 18 characters, 20 bytes
    const std::string exactFooter =
        "nothing is powered \xE2\x80\x94 fix the cables, then restart.";    // 50 characters, 52 bytes
    REQUIRE(drawn(exactHeader).size() == shulib::calibration::kHeaderCols);
    REQUIRE(drawn(exactHeadline).size() == shulib::calibration::kLargeCols);
    REQUIRE(drawn(exactFooter).size() == shulib::calibration::kSmallCols);
    for (std::size_t which = 0; which < 3; ++which) {
        for (const bool over : {false, true}) {
            std::string text[3] = {std::string{screen_text::kHeaderCannot},
                                   std::string{screen_text::kTooFewMotors},
                                   std::string{screen_text::kNothingPowered}};
            const std::string exact[3] = {exactHeader, exactHeadline, exactFooter};
            text[which] = exact[which] + (over ? "!" : "");
            FakeBrainScreen one;
            CalibratePainter q{one, "B"};
            q.paintCannot(text[0], PanelTone::Red, text[1], "short", "short", text[2]);
            INFO("text ", which, (over ? ", one character over" : ", exactly the budget"));
            REQUIRE(one.log().size() == 9);  // erase, bar, word, build, headline, why, fix, footer, where
            const std::string place[3] = {"print M 8 4 pen=101418 eraser=e2564a '",
                                          "print L 8 34 pen=e2564a eraser=101418 '",
                                          "print S 8 186 pen=9aa4ae eraser=101418 '"};
            const std::size_t entry[3] = {2, 4, 7};
            CHECK(one.log()[entry[which]] == place[which] + drawn(exact[which]) + "'");
            CHECK(one.log()[8] == std::string{"print S 8 202 pen=9aa4ae eraser=101418 '"}
                                      + (over ? "the full text is on the serial log"
                                              : "the census is on the serial log")
                                      + "'");
        }
    }

    p.render(pushView(), 10.0);
    at = s.log().size();
    p.paintNotDriverControl("status bits 0x05 field=yes disabled=yes auton=no");
    CHECK(since(s, at) == std::vector<std::string>{
                              "erase 0 27 479 235 eraser=101418", "fill 0 0 479 26 pen=e2564a",
                              "print M 8 4 pen=101418 eraser=e2564a 'STOPPED -- MOTORS OFF'",
                              "print S 340 7 pen=101418 eraser=e2564a 'BUILD 12:34:56'",
                              "print L 8 34 pen=e8b23a eraser=101418 'NOT DRIVER CONTROL'",
                              "print S 8 80 pen=f0f0f0 eraser=101418 'It powers the motors, so it runs ONLY in'",
                              "print S 8 96 pen=f0f0f0 eraser=101418 'DRIVER CONTROL, enabled. Motors: 0 V, coasting.'",
                              "print S 8 120 pen=9aa4ae eraser=101418 'status bits 0x05 field=yes disabled=yes auton=no'",
                              "print S 8 146 pen=f0f0f0 eraser=101418 'Unplug any competition switch or field cable,'",
                              "print S 8 162 pen=f0f0f0 eraser=101418 'and NEVER start it from the CONTROLLER's menu.'",
                              "print S 8 188 pen=9aa4ae eraser=101418 'Any result is on the serial log too ([R3D]).'"});
    repaintsInFullAfter("paintNotDriverControl", 11.0);
    // Status bits a caller passed: exactly the SMALL budget (50) is drawn whole and NOT said; one
    // character more is cut, and the screen's last line says the full text is on the serial log.
    {
        const std::string fifty = "status bits 0x07 field=yes disabled=yes auton=yes.";
        REQUIRE(fifty.size() == shulib::calibration::kSmallCols);
        FakeBrainScreen one;
        CalibratePainter q{one, "B"};
        q.paintNotDriverControl(fifty.c_str());
        REQUIRE(one.log().size() == 11);  // no "full text" line
        CHECK(one.log()[7] == "print S 8 120 pen=9aa4ae eraser=101418 '" + fifty + "'");
        FakeBrainScreen two;
        CalibratePainter r{two, "B"};
        r.paintNotDriverControl((fifty + "!").c_str());
        REQUIRE(two.log().size() == 12);
        CHECK(two.log()[7] == "print S 8 120 pen=9aa4ae eraser=101418 '" + fifty + "'");
        CHECK(two.log()[11]
              == "print S 8 204 pen=9aa4ae eraser=101418 'the full text is on the serial log'");
    }

    // The competition-state screen, with screen_text's words for disabled(): black, the state LARGE
    // in amber, the rule, why on two SMALL lines, the bits, what to do — every line in budget, so
    // no "full text" line. No header.
    at = s.log().size();
    p.paintStateScreen(screen_text::kStateNameDisabled.data(), screen_text::kStateWhyDisabled.data(),
                       "status bits 0x05 field=yes disabled=yes auton=no");
    CHECK(since(s, at) == std::vector<std::string>{
                              "erase-all eraser=000000",
                              "print L 10 30 pen=f0c000 eraser=000000 'DISABLED'",
                              "print S 10 80 pen=ffffff eraser=000000 'This program runs ONLY in DRIVER CONTROL, enabled.'",
                              "print S 10 100 pen=ffffff eraser=000000 'VEXos reports the robot DISABLED, so the driver'",
                              "print S 10 116 pen=ffffff eraser=000000 'task is not running.'",
                              "print S 10 136 pen=ffffff eraser=000000 'status bits 0x05 field=yes disabled=yes auton=no'",
                              "print S 10 160 pen=ffffff eraser=000000 'Look at the CONTROLLER: it names this state.'",
                              "print S 10 176 pen=ffffff eraser=000000 'Unplug any competition switch or field cable'",
                              "print S 10 192 pen=ffffff eraser=000000 'from the controller. This clears by itself.'"});
    repaintsInFullAfter("paintStateScreen", 12.0);
    // The other two states' words, each whole on the screen (name, then its reason's two lines).
    const char* const states[2][4] = {
        {screen_text::kStateNameConnected.data(), screen_text::kStateWhyConnected.data(),
         "A competition switch or field is connected through", "the controller."},
        {screen_text::kStateNameAutonomous.data(), screen_text::kStateWhyAutonomous.data(),
         "No motion here, ever: calibration powers the",
         "motors only while a person stands by the battery."},
    };
    for (const auto& st : states) {
        FakeBrainScreen one;
        CalibratePainter q{one, "B"};
        q.paintStateScreen(st[0], st[1], "status bits 0x02 field=yes disabled=no auton=yes");
        REQUIRE(one.log().size() == 9);  // no "full text" line
        CHECK(one.log()[1] == std::string{"print L 10 30 pen=f0c000 eraser=000000 '"} + st[0] + "'");
        CHECK(one.log()[3] == std::string{"print S 10 100 pen=ffffff eraser=000000 '"} + st[2] + "'");
        CHECK(one.log()[4] == std::string{"print S 10 116 pen=ffffff eraser=000000 '"} + st[3] + "'");
    }
    // Words that do NOT fit — robot two's other programs' 23-character name and a reason of three
    // lines — are cut, and the screen says the full text is on the serial log (its last line).
    {
        FakeBrainScreen one;
        CalibratePainter q{one, "B"};
        q.paintStateScreen("FIELD CONTROL CONNECTED",
                           "No motion here, ever: calibration powers the motors and lives under "
                           "driver control, where a person is standing by the battery.",
                           "status bits 0x04 field=yes disabled=no auton=no");
        REQUIRE(one.log().size() == 10);
        CHECK(one.log()[1] == "print L 10 30 pen=f0c000 eraser=000000 'FIELD CONTROL CONN'");
        CHECK(one.log()[9] == "print S 10 210 pen=9aa4ae eraser=000000 'the full text is on the serial log'");
    }
    // Each of the three alone is enough to say it: the name, the reason, the status.
    const char* const alone[3][3] = {
        {"FIELD CONTROL CONNECTED", "short", "status bits 0x04"},
        {"DISABLED", "one two three four five six seven eight nine ten eleven twelve thirteen "
                     "fourteen fifteen sixteen seventeen eighteen nineteen twenty", "status bits 0x01"},
        {"DISABLED", "short", "status bits 0x01 and then far more than fifty characters of it"},
    };
    for (const auto& a : alone) {
        FakeBrainScreen one;
        CalibratePainter q{one, "B"};
        q.paintStateScreen(a[0], a[1], a[2]);
        CHECK_MESSAGE(one.log().back()
                          == "print S 10 210 pen=9aa4ae eraser=000000 'the full text is on the serial log'",
                      a[0], " | ", a[1], " | ", a[2]);
    }
    // The state name and the status bits at EXACTLY their budgets in characters drawn (LARGE's 18,
    // SMALL's 50; each with an em dash, so over them in bytes): drawn whole, NO "full text" line.
    // One character over: cut back to the same characters, and the last line says it.
    const std::string exactState = "STATE \xE2\x80\x94 EIGHTEEN!!";  // 18 characters, 20 bytes
    const std::string exactBits =
        "status bits 0x07 \xE2\x80\x94 field=yes disabled=yes auton=no";  // 50 characters, 52 bytes
    REQUIRE(drawn(exactState).size() == shulib::calibration::kLargeCols);
    REQUIRE(drawn(exactBits).size() == shulib::calibration::kSmallCols);
    for (std::size_t which = 0; which < 2; ++which) {
        for (const bool over : {false, true}) {
            const std::string name = which == 0 ? exactState + (over ? "!" : "")
                                                : std::string{screen_text::kStateNameDisabled};
            const std::string bits = which == 1 ? exactBits + (over ? "!" : "") : "status bits 0x01";
            FakeBrainScreen one;
            CalibratePainter q{one, "B"};
            q.paintStateScreen(name.c_str(), screen_text::kStateWhyDisabled.data(), bits.c_str());
            INFO((which == 0 ? "the state name" : "the status bits"),
                 (over ? ", one character over" : ", exactly the budget"));
            REQUIRE(one.log().size() == (over ? 10U : 9U));
            CHECK(one.log()[1] == "print L 10 30 pen=f0c000 eraser=000000 '"
                                      + (which == 0 ? drawn(exactState) : std::string{"DISABLED"})
                                      + "'");
            CHECK(one.log()[5] == "print S 10 136 pen=ffffff eraser=000000 '"
                                      + (which == 1 ? drawn(exactBits) : std::string{"status bits 0x01"})
                                      + "'");
            if (over) {
                CHECK(one.log()[9]
                      == "print S 10 210 pen=9aa4ae eraser=000000 'the full text is on the serial log'");
            }
        }
    }
}

TEST_CASE("CalibratePainter P8: the build time is cut at kBuildTimeCols and the header word at "
          "kHeaderCols; a view is painted to its end only, and a long one copied a line's worth; a "
          "'%' in a line reaches the screen as '?'") {
    // BUG CAUGHT: a long build text running into the right edge, or a long header word into the
    // build time (the header's layout is checked at compile time for kBuildTimeCols and
    // kHeaderCols characters, and only the cuts keep that true — MEDIUM's own budget, 36, would
    // not); a string_view's .data() handed on as a C string (it reads past the view, into
    // whatever follows); a view longer than the copy buffer copied whole (a stack overflow); a
    // line printed without the sanitizer.
    REQUIRE(kBuildTimeCols == 14);
    FakeBrainScreen s;
    CalibratePainter p{s, "BUILD 12:34:56 AND MORE"};
    p.paintStarting();
    REQUIRE(s.log().size() == 7);
    CHECK(s.log()[3] == "print S 340 7 pen=f0f0f0 eraser=1e5aa8 'BUILD 12:34:56'");
    FakeBrainScreen n;
    CalibratePainter q{n, nullptr};
    q.paintStarting();
    REQUIRE(n.log().size() == 7);
    CHECK(n.log()[3] == "print S 340 7 pen=f0f0f0 eraser=1e5aa8 ''");

    static const char buffer[] = "HEADLINE-and-what-follows-it";
    const std::string_view headline{buffer, 8};
    static const char footer[] = "FOOTER, then more";
    FakeBrainScreen c;
    CalibratePainter r{c, "B"};
    r.paintCannot(std::string_view{buffer + 9, 3}, PanelTone::Red, headline, "", "",
                  std::string_view{footer, 6});
    REQUIRE(c.log().size() == 7);  // erase, bar, word, build, headline, footer, where (no why, no fix)
    CHECK(c.log()[2] == "print M 8 4 pen=101418 eraser=e2564a 'and'");
    CHECK(c.log()[4] == "print L 8 34 pen=e2564a eraser=101418 'HEADLINE'");
    CHECK(c.log()[5] == "print S 8 186 pen=9aa4ae eraser=101418 'FOOTER'");
    // ... and MEASURED to its end only: every view fits its line, so nothing is said to be cut.
    CHECK(c.log()[6] == "print S 8 202 pen=9aa4ae eraser=101418 'the census is on the serial log'");

    // A header word longer than kHeaderCols — through a view and through an own screen — is cut
    // at 24 characters, not at MEDIUM's 36.
    FakeBrainScreen hw;
    CalibratePainter h{hw, "B"};
    h.render(view(PanelScreen::Saved, PanelTone::Green, "A HEADER WORD MUCH TOO LONG FOR THE BAR", {}, {}),
             0.0);
    REQUIRE(hw.log().size() == 4);  // erase, bar, word, build
    CHECK(hw.log()[2] == "print M 8 4 pen=101418 eraser=39c36e 'A HEADER WORD MUCH TOO L'");
    h.paintCannot("A HEADER WORD MUCH TOO LONG FOR THE BAR", PanelTone::Red, "X", "", "", "");
    REQUIRE(hw.log().size() >= 7);  // erase, bar, word, build, headline, footer, where to look
    CHECK(hw.log()[hw.log().size() - 5] == "print M 8 4 pen=101418 eraser=e2564a 'A HEADER WORD MUCH TOO L'");

    // A view of 200 characters with no terminator after them, longer than the 128-byte copy
    // buffer: a line's worth is copied (and counted), the size's budget of it drawn.
    std::string longText(260, '#');
    for (std::size_t i = 0; i < 200; ++i) {
        longText[i] = static_cast<char>('A' + static_cast<int>(i % 26));
    }
    FakeBrainScreen lv;
    CalibratePainter l{lv, "B"};
    l.paintCannot(std::string_view{longText.data(), 200}, PanelTone::Red,
                  std::string_view{longText.data(), 200}, "", "",
                  std::string_view{longText.data(), 200});
    REQUIRE(lv.log().size() == 7);
    CHECK(lv.log()[2] == "print M 8 4 pen=101418 eraser=e2564a 'ABCDEFGHIJKLMNOPQRSTUVWX'");
    CHECK(lv.log()[4] == "print L 8 34 pen=e2564a eraser=101418 'ABCDEFGHIJKLMNOPQR'");
    CHECK(lv.log()[5]
          == "print S 8 186 pen=9aa4ae eraser=101418 'ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX'");
    CHECK(lv.log()[6] == "print S 8 202 pen=9aa4ae eraser=101418 'the full text is on the serial log'");

    // render() cuts WITHOUT a word on screen, as the header now says (the verification round: its
    // "Nothing is cut silently" was false): a LARGE line past 18, a MEDIUM line past 36 and a
    // button label past MEDIUM's 36 are each drawn as their budget's worth, and nothing else is
    // drawn — no "full text" line. (A SMALL line cannot pass its 50: a PanelLine holds 50.)
    // BUG CAUGHT: a view's line cut at a budget other than its size's (a LARGE line drawn 30
    // wide runs off its place), a label cut at another width, and a render() that grew a "full
    // text" line its documentation does not promise.
    FakeBrainScreen cv;
    CalibratePainter u{cv, "B"};
    u.render(view(PanelScreen::Saved, PanelTone::Green, "W",
                  {line(8, 30, TextSize::Large, PanelTone::White, "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123"),
                   line(8, 70, TextSize::Medium, PanelTone::White,
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghi")},
                  {button(PanelButtonId::RunAgain, kDoneRect, "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcd")}),
             0.0);
    // erase, bar, word, build, the two lines, the button's fill and two edges, its label.
    REQUIRE(cv.log().size() == 10);
    CHECK(cv.log()[4] == "print L 8 30 pen=f0f0f0 eraser=101418 'ABCDEFGHIJKLMNOPQR'");
    CHECK(cv.log()[5] == "print M 8 70 pen=f0f0f0 eraser=101418 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'");
    // 40 x 13 px is wider than the 295 px box, so the label starts 4 px in: x 6 + 4 = 10.
    CHECK(cv.log()[9] == "print M 10 197 pen=f0f0f0 eraser=1e5aa8 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'");

    FakeBrainScreen pct;
    CalibratePainter t{pct, "B"};
    t.render(view(PanelScreen::Saved, PanelTone::Green, "100% SAVED",
                  {line(8, 70, TextSize::Medium, PanelTone::White, "50% \xE2\x80\x94 done")}, {}),
             0.0);
    REQUIRE(pct.log().size() == 5);
    CHECK(pct.log()[2] == "print M 8 4 pen=101418 eraser=39c36e '100? SAVED'");
    CHECK(pct.log()[4] == "print M 8 70 pen=f0f0f0 eraser=101418 '50? - done'");
}

TEST_CASE("CalibratePainter P9: every literal of calibration::screen_text is in kAll with the budget "
          "of the size it is drawn at, and every one fits it") {
    // BUG CAUGHT: a literal added to the header without joining kAll — the compile-time budget
    // check would never see it, and a word too long for its line would reach a robot unchecked;
    // a literal listed in kAll against the WRONG budget — a MEDIUM line checked at SMALL's 50
    // passes the compile-time check and is then cut at 36 on the screen. The size is read two
    // ways: the one each literal's comment states (every literal), and the one the painter draws
    // each literal it names at (printPanelView's TextSize, or the header's for paintHeader).
    // ALSO CAUGHT: the compile-time check itself weakened so that it passes everything — a
    // one-line literal measured against its own length, the wrapped branch or its ASCII check
    // dropped, a wrap that is never looked at, any width or line count taken as the state
    // screen's shape, a list walked without looking, the static_assert removed. Every literal
    // TODAY fits, so none of those would turn the build red; the next word too long for its
    // line would then fail only here, at test time, not at the build the header promises.
    const std::string src = readSource("include/shulib/calibration/calibrate_screen.hpp");
    const auto open = src.find("namespace screen_text {");
    const auto table = src.find("inline constexpr Checked kAll[]");
    const auto close = src.find("};", table);
    REQUIRE(open != std::string::npos);
    REQUIRE(table != std::string::npos);
    REQUIRE(close != std::string::npos);
    const std::string literals = src.substr(open, table - open);
    const std::string all = src.substr(table, close - table);

    // Each literal, and the budget its comment block's FIRST line states.
    std::map<std::string, std::string> budgetOf;
    {
        std::istringstream in(literals);
        std::string line;
        std::string blockFirst;
        bool inBlock = false;
        const std::regex decl{R"(^inline constexpr std::string_view (k[A-Za-z0-9]+) =)"};
        while (std::getline(in, line)) {
            if (line.rfind("///", 0) == 0) {
                if (!inBlock) {
                    blockFirst = line;
                    inBlock = true;
                }
                continue;
            }
            std::smatch m;
            if (std::regex_search(line, m, decl)) {
                std::string want;
                if (blockFirst.rfind("/// Header word", 0) == 0) {
                    want = "kHeaderCols";
                } else if (blockFirst.rfind("/// LARGE", 0) == 0) {
                    want = "kLargeCols";
                } else if (blockFirst.rfind("/// MEDIUM", 0) == 0) {
                    want = "kMediumCols";
                } else if (blockFirst.rfind("/// SMALL, wrapped onto two lines", 0) == 0) {
                    want = "kSmallCols, detail::kStateWhyLines";
                } else if (blockFirst.rfind("/// SMALL", 0) == 0) {
                    want = "kSmallCols";
                }
                CHECK_MESSAGE(!want.empty(), m[1].str(),
                              "'s comment does not begin with the size it is drawn at");
                budgetOf[m[1].str()] = want;
            }
            inBlock = false;
        }
    }
    CHECK(budgetOf.size() == 35);
    CHECK(std::size(screen_text::kAll) == budgetOf.size());
    for (const auto& [name, want] : budgetOf) {
        CHECK_MESSAGE(all.find("{" + name + ", " + want + "}") != std::string::npos, name,
                      " is not in kAll with the budget its comment states (", want, ")");
    }

    // The size the painter DRAWS each literal it names at, from its calls (whitespace collapsed).
    std::string flat;
    for (const char c : src) {
        const bool space = c == ' ' || c == '\n' || c == '\t';
        if (!space || (!flat.empty() && flat.back() != ' ')) {
            flat.push_back(space ? ' ' : c);
        }
    }
    const std::regex call{R"((printPanelView|paintHeader)\(([^;]*)\);)"};
    const std::regex sizeRe{R"(TextSize::(Large|Medium|Small))"};
    const std::regex nameRe{R"(screen_text::(k[A-Za-z0-9]+))"};
    int drawnByName = 0;
    for (auto it = std::sregex_iterator(flat.begin(), flat.end(), call);
         it != std::sregex_iterator(); ++it) {
        const std::string args = (*it)[2].str();
        std::string want = "kHeaderCols";
        if ((*it)[1].str() == "printPanelView") {
            std::smatch sz;
            if (!std::regex_search(args, sz, sizeRe)) {
                continue;  // a helper's own definition, not a draw
            }
            want = sz[1].str() == "Large" ? "kLargeCols"
                   : sz[1].str() == "Medium" ? "kMediumCols"
                                             : "kSmallCols";
        }
        for (auto n = std::sregex_iterator(args.begin(), args.end(), nameRe);
             n != std::sregex_iterator(); ++n) {
            const std::string name = (*n)[1].str();
            REQUIRE_MESSAGE(budgetOf.count(name) == 1, name, " is drawn but is not a literal");
            CHECK_MESSAGE(budgetOf[name].rfind(want, 0) == 0, name, " is drawn at ", want,
                          " but checked against ", budgetOf[name]);
            ++drawnByName;
        }
    }
    // The own screens draw 19 by name: starting 4 (its header word among them), cannot start
    // 2, not driver control 8 (its "full text" line among them), the state screen 5.
    CHECK(drawnByName == 19);

    // The compile-time check refuses what it must — each of these is evaluated by the compiler, so
    // a check weakened to pass everything fails to BUILD this test.
    {
        using screen_text::allFit;
        using screen_text::Checked;
        using screen_text::fits;
        using shulib::calibration::detail::wrapsCompletely;
        constexpr std::size_t kLarge = shulib::calibration::kLargeCols;
        constexpr std::size_t kMedium = shulib::calibration::kMediumCols;
        constexpr std::size_t kSmall = shulib::calibration::kSmallCols;
        // One line: exactly the budget fits; one character over, a '%' or a non-ASCII byte does not.
        static_assert(fits(Checked{"EIGHTEEN CHARACTER", kLarge}));
        static_assert(!fits(Checked{"NINETEEN CHARACTERS", kLarge}));
        static_assert(!fits(Checked{"50% done", kSmall}));
        static_assert(!fits(Checked{"FRONT \xE2\x80\x94 X", kLarge}));
        // Wrapped onto the state screen's two SMALL lines: a reason that wraps fits; the draft's
        // 126-character reason (three lines' worth) does not, nor one with a '%', nor any other
        // shape — MEDIUM's width, or three lines.
        static_assert(fits(Checked{screen_text::kStateWhyDisabled, kSmall, 2}));
        static_assert(!fits(Checked{"No motion here, ever: calibration powers the motors and lives "
                                    "under driver control, where a person is standing by the "
                                    "battery.",
                                    kSmall, 2}));
        static_assert(!fits(Checked{"a reason, and 50% of it", kSmall, 2}));
        static_assert(!fits(Checked{"short", kMedium, 2}));
        static_assert(!fits(Checked{"short", kSmall, 3}));
        static_assert(wrapsCompletely("short", kSmall, 2));
        static_assert(!wrapsCompletely("short", kMedium, 2));
        static_assert(!wrapsCompletely("short", kSmall, 3));
        // A list: one bad entry anywhere refuses it.
        static constexpr Checked good[] = {{"FINE", kLarge}, {"FINE TOO", kLarge}};
        static constexpr Checked badLast[] = {{"FINE", kLarge}, {"NINETEEN CHARACTERS", kLarge}};
        static constexpr Checked badFirst[] = {{"NINETEEN CHARACTERS", kLarge}, {"FINE", kLarge}};
        static_assert(allFit(good));
        static_assert(!allFit(badLast));
        static_assert(!allFit(badFirst));
    }
    // ... and the header runs it over kAll at compile time (without that static_assert every
    // literal would be checked only here, at test time).
    CHECK(src.find("\nstatic_assert(allFit(kAll), \"calibrate_screen: ") != std::string::npos);

    // And each fits, as the compile-time check says.
    CHECK(screen_text::allFit(screen_text::kAll));
    for (const auto& c : screen_text::kAll) {
        if (c.lines <= 1) {
            CHECK_MESSAGE(shulib::calibration::screenText(c.text, c.cols), c.text);
        } else {
            CHECK(c.cols == shulib::calibration::kSmallCols);
            CHECK(c.lines == 2);
            const bool wraps =
                shulib::calibration::wrapWords<shulib::calibration::kSmallCols, 2>(c.text).complete;
            CHECK_MESSAGE(wraps, c.text);
            CHECK_MESSAGE(shulib::calibration::screenText(c.text, c.text.size()), c.text);
        }
    }
}
