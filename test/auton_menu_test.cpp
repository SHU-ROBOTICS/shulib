// competition::AutonMenu — the auton selector as a pure model (chunk COMP-a; the COMP brief's
// ruling C4). Every case names the bug it would catch.
//
// The properties are checked over EVERY reachable view (every page, locked and unlocked, armed
// and greyed) rather than over one hand-picked screen — the calibrate panel's lesson: the frames
// nobody looked at are the ones with the text cut in half.

#include "doctest.h"

#include <cstdio>
#include <limits>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonMenu;
using shulib::competition::AutonPeriods;
using shulib::competition::AutonRegistry;
using shulib::competition::kMaxAutonNameChars;
using shulib::competition::kMenuNoneSlot;
using shulib::competition::MenuButton;
using shulib::competition::MenuButtonId;
using shulib::competition::MenuTouch;
using shulib::competition::MenuTouchAction;
using shulib::competition::MenuView;
using shulib::competition::StartPose;
using shulib::units::Length;
using shulib::units::Time;
namespace ui = shulib::ui;

namespace {

RoutineResult noop(AutonContext& /*ctx*/) { return RoutineResult{}; }

/// Names chosen to stress the budgets: the longest legal name, one ending in the kind's own word,
/// punctuation the screen can show.
const char* const kNames[] = {"aaaaaaaaaaaaaaaaaaaaaaaaaaaaa",  // 29 characters, the budget
                              "left rush", "right: 3 ring + climb", "skills 1", "x"};
constexpr int kCount = 5;

AutonRegistry makeRegistry(int n) {
    AutonRegistry reg{AutonPeriods{.match = Time{30.0}, .skills = Time{60.0}}};
    for (int i = 0; i < n; ++i) {
        AutonEntry e{};
        e.name = kNames[i];
        e.kind = (i % 2 == 0) ? AutonKind::Match : AutonKind::Skills;
        e.routine = &noop;
        e.start = (i % 3 == 0) ? StartPose::relative()
                               : StartPose::field(Length{-48.0}, Length{-24.0},
                                                  shulib::math::Angle::degrees(90.0));
        e.endActionAt = Time{25.5};
        e.hardStopAt = Time{29.25};
        reg.add(e);
    }
    return reg;
}

/// Taps at the centre of `r`, with a fresh release each time.
struct Finger {
    std::int32_t releases = 100;
    ui::TouchSample at(ui::Rect r) {
        return ui::TouchSample{.error = false,
                               .releaseCount = ++releases,
                               .x = static_cast<std::int16_t>((r.x0 + r.x1) / 2),
                               .y = static_cast<std::int16_t>((r.y0 + r.y1) / 2)};
    }
};

/// A view whose layout has been up long enough to be armed.
MenuView armedView(AutonMenu& m, double& t) {
    (void)m.view(t);
    t += ui::kArmingDelaySeconds + 0.01;
    return m.view(t);
}

const MenuButton* find(const MenuView& v, MenuButtonId id, int entry = -1) {
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        if (v.buttons[i].id == id && (id != MenuButtonId::Entry || v.buttons[i].entry == entry)) {
            return &v.buttons[i];
        }
    }
    return nullptr;
}

/// Every string in `v` passes the screen test at its budget; every rectangle is on the screen; no
/// two buttons' HIT BOXES overlap (a tap between two buttons must not be able to hit either one
/// by accident of order).
void checkScreenSafe(const MenuView& v) {
    CHECK(ui::screenText(v.header.text, ui::budgetFor(v.header.size)));
    CHECK(ui::screenText(v.info.text, ui::budgetFor(v.info.size)));
    CHECK(ui::screenText(v.status.text, ui::budgetFor(v.status.size)));
    CHECK(v.header.size == ui::TextSize::Medium);
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        CAPTURE(v.buttons[i].label);
        CHECK(ui::screenText(v.buttons[i].label, ui::kMediumCols));
        CHECK(ui::onScreen(v.buttons[i].rect));
        for (std::size_t j = i + 1; j < v.buttonCount; ++j) {
            CHECK_FALSE(ui::overlaps(ui::hitBox(v.buttons[i].rect), ui::hitBox(v.buttons[j].rect)));
        }
    }
}

}  // namespace

// Bug caught: a selector that boots with an auton chosen — a robot nobody touched drives off the
// line. NONE is the default, it is drawn, it is highlighted, and the header says so.
TEST_CASE("C4 NONE is the default choice, always drawn, in the same place") {
    const AutonRegistry reg = makeRegistry(kCount);
    AutonMenu menu{reg, 7};
    CHECK(menu.choice() == -1);
    CHECK(menu.chosenEntry() == nullptr);
    double t = 0.0;
    const MenuView v = menu.view(t);
    CHECK(std::string_view{v.header.text} == "AUTON: NONE");
    const MenuButton* none = find(v, MenuButtonId::None);
    REQUIRE(none != nullptr);
    CHECK(none->chosen);
    CHECK(none->rect == kMenuNoneSlot);
    CHECK(none->armed);  // NONE is never delayed
    // …and on every page.
    Finger f;
    for (int page = 0; page < menu.pageCount(); ++page) {
        const MenuView pv = armedView(menu, t);
        const MenuButton* n = find(pv, MenuButtonId::None);
        REQUIRE(n != nullptr);
        CHECK(n->rect == kMenuNoneSlot);
        if (const MenuButton* next = find(pv, MenuButtonId::Next)) {
            REQUIRE(menu.touch(f.at(next->rect), pv).action == MenuTouchAction::Paged);
        }
    }
}

// Bug caught: an auton the selector cannot reach (paging that drops the last one, an off-by-one
// on a page boundary) or a button that chooses a DIFFERENT auton than its label says.
TEST_CASE("C4 every drawn button maps to its entry, and paging hides none") {
    for (int n = 0; n <= kCount; ++n) {
        CAPTURE(n);
        const AutonRegistry reg = makeRegistry(n);
        AutonMenu menu{reg, 0};
        Finger f;
        double t = 0.0;
        std::multiset<int> seen;
        for (int page = 0; page < menu.pageCount(); ++page) {
            const MenuView v = armedView(menu, t);
            CHECK(v.page == page);
            for (std::size_t i = 0; i < v.buttonCount; ++i) {
                const MenuButton& b = v.buttons[i];
                if (b.id != MenuButtonId::Entry) {
                    continue;
                }
                REQUIRE(b.entry >= 0);
                REQUIRE(static_cast<std::size_t>(b.entry) < reg.size());
                seen.insert(b.entry);
                const AutonEntry& e = reg.at(static_cast<std::size_t>(b.entry));
                char expect[64];
                std::snprintf(expect, sizeof expect, "%s %s", e.name,
                              shulib::competition::autonKindWord(e.kind));
                CHECK(std::string_view{b.label} == expect);
                // Touching it chooses exactly that entry.
                const MenuTouch r = menu.touch(f.at(b.rect), v);
                CHECK(r.action == MenuTouchAction::Chose);
                CHECK(menu.choice() == b.entry);
                CHECK(menu.chosenEntry() == &e);
            }
            if (const MenuButton* next = find(v, MenuButtonId::Next)) {
                REQUIRE(menu.touch(f.at(next->rect), v).action == MenuTouchAction::Paged);
            }
        }
        CHECK(seen.size() == static_cast<std::size_t>(n));  // none twice…
        for (int i = 0; i < n; ++i) {
            CHECK(seen.count(i) == 1);  // …and none missing
        }
    }
}

// Bug caught: the double-tap hazard (the calibrate panel's first night: FRONT FIRST appeared exactly
// where DONE had been). For every button of every reachable view: after pressing it, any button
// of the NEW view whose hit box overlaps the pressed one's must be the SAME button.
TEST_CASE("C4 no new button appears under the one just pressed") {
    const AutonRegistry reg = makeRegistry(kCount);
    AutonMenu probe{reg, 0};
    const int pages = probe.pageCount();
    REQUIRE(pages == 3);
    for (int page = 0; page < pages; ++page) {
        for (std::size_t which = 0; which < shulib::competition::kMenuMaxButtons; ++which) {
            AutonMenu menu{reg, 0};
            Finger f;
            double t = 0.0;
            MenuView v = armedView(menu, t);
            for (int p = 0; p < page; ++p) {  // walk to the page
                REQUIRE(menu.touch(f.at(find(v, MenuButtonId::Next)->rect), v).action
                        == MenuTouchAction::Paged);
                v = armedView(menu, t);
            }
            if (which >= v.buttonCount) {
                continue;
            }
            const MenuButton pressed = v.buttons[which];
            CAPTURE(page);
            CAPTURE(pressed.label);
            (void)menu.touch(f.at(pressed.rect), v);
            const MenuView after = menu.view(t);
            for (std::size_t i = 0; i < after.buttonCount; ++i) {
                const MenuButton& b = after.buttons[i];
                if (ui::overlaps(ui::hitBox(b.rect), ui::hitBox(pressed.rect))) {
                    CHECK(b.id == pressed.id);
                    CHECK(b.entry == pressed.entry);
                }
            }
        }
    }
}

// Bug caught: a choice that can change after the auton started — a touch racing the start would
// run one auton and report another, or change the auton mid-match. lockChoice() returns the choice
// in the same atomic operation that locks it; afterwards every button is dead, and even a view
// drawn BEFORE the lock (its buttons still armed) cannot change it.
TEST_CASE("C4 no touch changes the choice once the auton has started") {
    const AutonRegistry reg = makeRegistry(kCount);
    AutonMenu menu{reg, 0};
    Finger f;
    double t = 0.0;
    const MenuView before = armedView(menu, t);
    const MenuButton* first = find(before, MenuButtonId::Entry, 0);
    const MenuButton* second = find(before, MenuButtonId::Entry, 1);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(menu.touch(f.at(first->rect), before).action == MenuTouchAction::Chose);

    CHECK(menu.lockChoice() == 0);  // the auton that runs is entry 0
    CHECK(menu.locked());
    // A stale, armed view: the finger lands on entry 1 and on NONE — both refused.
    CHECK(menu.touch(f.at(second->rect), before).action == MenuTouchAction::RefusedLocked);
    CHECK(menu.touch(f.at(kMenuNoneSlot), before).action == MenuTouchAction::RefusedLocked);
    CHECK(menu.choice() == 0);
    // The current view: locked, every button dead, the status line says so.
    const MenuView locked = armedView(menu, t);
    CHECK(locked.locked);
    for (std::size_t i = 0; i < locked.buttonCount; ++i) {
        CHECK_FALSE(locked.buttons[i].armed);
    }
    CHECK(std::string_view{locked.status.text} == shulib::competition::kMenuLockedText);
    CHECK(menu.touch(f.at(second->rect), locked).action == MenuTouchAction::None);
    CHECK(menu.choice() == 0);
    checkScreenSafe(locked);

    // After the program unlocks (its auton task FINISHED), a new choice is taken again.
    menu.unlock();
    const MenuView again = armedView(menu, t);
    CHECK(menu.touch(f.at(kMenuNoneSlot), again).action == MenuTouchAction::Chose);
    CHECK(menu.choice() == -1);
}

// Bug caught: a new layout's buttons live on the frame they first appear (a reflexive second tap
// answers the new screen), or NONE delayed with them (the safe choice must never be greyed).
TEST_CASE("C4 the arming delay: new layouts come up greyed, except NONE") {
    const AutonRegistry reg = makeRegistry(kCount);
    AutonMenu menu{reg, 0};
    Finger f;
    double t = 10.0;
    const MenuView first = menu.view(t);
    for (std::size_t i = 0; i < first.buttonCount; ++i) {
        CHECK(first.buttons[i].armed == (first.buttons[i].id == MenuButtonId::None));
    }
    const MenuButton* entry0 = find(first, MenuButtonId::Entry, 0);
    REQUIRE(entry0 != nullptr);
    CHECK(menu.touch(f.at(entry0->rect), first).action == MenuTouchAction::None);  // greyed
    CHECK(menu.choice() == -1);
    t += ui::kArmingDelaySeconds - 0.01;
    const MenuView early = menu.view(t);
    REQUIRE(find(early, MenuButtonId::Entry, 0) != nullptr);
    CHECK_FALSE(find(early, MenuButtonId::Entry, 0)->armed);  // not yet
    t += 0.02;
    const MenuView armed = menu.view(t);
    REQUIRE(find(armed, MenuButtonId::Entry, 0) != nullptr);
    CHECK(find(armed, MenuButtonId::Entry, 0)->armed);
    // A CHOICE does not change the layout (no new delay)...
    REQUIRE(menu.touch(f.at(entry0->rect), armed).action == MenuTouchAction::Chose);
    const MenuView chosen = menu.view(t);
    REQUIRE(find(chosen, MenuButtonId::Entry, 1) != nullptr);
    CHECK(find(chosen, MenuButtonId::Entry, 1)->armed);
    // ...a PAGE FLIP does.
    REQUIRE(find(armed, MenuButtonId::Next) != nullptr);
    REQUIRE(menu.touch(f.at(find(armed, MenuButtonId::Next)->rect), armed).action
            == MenuTouchAction::Paged);
    const MenuView flipped = menu.view(t);
    REQUIRE(find(flipped, MenuButtonId::Entry, 2) != nullptr);
    CHECK_FALSE(find(flipped, MenuButtonId::Entry, 2)->armed);
    REQUIRE(find(flipped, MenuButtonId::None) != nullptr);
    CHECK(find(flipped, MenuButtonId::None)->armed);
}

// Bug caught: the touch discipline — an error read acted on (PROS answers every field -1), a stale
// sample replayed as a second tap, or a tap made before the menu existed replayed as a choice.
TEST_CASE("C4 the release edge: error reads, stale samples and old taps never choose") {
    const AutonRegistry reg = makeRegistry(kCount);
    AutonMenu menu{reg, 50};
    double t = 0.0;
    const MenuView v = armedView(menu, t);
    const ui::Rect r = find(v, MenuButtonId::Entry, 1)->rect;
    const auto cx = static_cast<std::int16_t>((r.x0 + r.x1) / 2);
    const auto cy = static_cast<std::int16_t>((r.y0 + r.y1) / 2);
    // the count the menu started with: a tap from before it existed
    CHECK(menu.touch({.error = false, .releaseCount = 50, .x = cx, .y = cy}, v).action
          == MenuTouchAction::None);
    // an error read — and it must not consume the next release
    CHECK_FALSE(menu.touch({.error = true, .releaseCount = 51, .x = cx, .y = cy}, v).newRelease);
    const MenuTouch real = menu.touch({.error = false, .releaseCount = 51, .x = cx, .y = cy}, v);
    CHECK(real.action == MenuTouchAction::Chose);
    CHECK(menu.choice() == 1);
    REQUIRE(menu.touch({.error = false, .releaseCount = 52, .x = static_cast<std::int16_t>(
                            (kMenuNoneSlot.x0 + kMenuNoneSlot.x1) / 2),
                        .y = 200},
                       v)
                .action
            == MenuTouchAction::Chose);
    // the same release again (a stale sample): nothing
    CHECK(menu.touch({.error = false, .releaseCount = 52, .x = cx, .y = cy}, v).action
          == MenuTouchAction::None);
    CHECK(menu.choice() == -1);
}

// Bug caught: text the V5 cannot draw — a line past its budget, a non-ASCII byte, a '%' that PROS
// would re-read as a format — on ANY reachable view: every page, every choice, locked and not.
TEST_CASE("C4 every reachable view is screen-safe: ASCII, no percent, within budget") {
    const AutonRegistry reg = makeRegistry(kCount);
    AutonMenu menu{reg, 0};
    Finger f;
    double t = 0.0;
    for (int choose = -1; choose < kCount; ++choose) {
        for (int page = 0; page < menu.pageCount(); ++page) {
            MenuView v = armedView(menu, t);
            checkScreenSafe(v);
            if (const MenuButton* b = find(v, MenuButtonId::Entry, choose)) {
                (void)menu.touch(f.at(b->rect), v);
                checkScreenSafe(menu.view(t));
            }
            if (const MenuButton* next = find(v, MenuButtonId::Next)) {
                (void)menu.touch(f.at(next->rect), v);
            } else {
                while (menu.page() > 0) {  // back to page 0 for the next choice
                    v = armedView(menu, t);
                    (void)menu.touch(f.at(find(v, MenuButtonId::Prev)->rect), v);
                }
            }
        }
    }
    (void)menu.lockChoice();
    checkScreenSafe(armedView(menu, t));
    // The longest legal name is shown WHOLE in the header, not cut.
    AutonMenu fresh{reg, 0};
    MenuView v = armedView(fresh, t);
    (void)fresh.touch(f.at(find(v, MenuButtonId::Entry, 0)->rect), v);
    v = fresh.view(t);
    CHECK(std::string_view{v.header.text} == std::string{"AUTON: "} + kNames[0]);
    CHECK(std::string_view{v.header.text}.size() == 7 + kMaxAutonNameChars);
}

// Bug caught (T4 Part 3; the team lead's ruling of 2026-10-02 on COMP-b's C5, "drop just that
// auton"): an entry the registry DROPPED leaving no trace on the selector — the person choosing an
// auton sees theirs missing and cannot tell why — or a dropped entry still drawn as a button (it was
// never registered: choosing it would run nothing). The status line names each dropped entry in
// RED, in turn (kMenuRefusalTurnSeconds apiece, back to the first), while the choice is open; once
// the auton has started it says LOCKED as before; a time that is not a number shows the first; and
// a registry that dropped nothing shows no REFUSED word at all.
TEST_CASE("C5 the selector names each DROPPED entry in red, in turn; a dropped entry is no button") {
    using shulib::competition::kMenuRefusalTurnSeconds;
    using shulib::competition::OnRefusal;
    AutonRegistry reg{AutonPeriods{.match = Time{30.0}, .skills = Time{60.0}},
                      OnRefusal::DropEntry};
    const auto entry = [](const char* name, double hardStop) {
        AutonEntry e{};
        e.name = name;
        e.kind = AutonKind::Match;
        e.routine = &noop;
        e.start = StartPose::relative();
        e.endActionAt = Time{25.0};
        e.hardStopAt = Time{hardStop};
        return e;
    };
    reg.add(entry("too late", 31.0));   // dropped: past the period
    reg.add(entry("left rush", 29.0));
    reg.add(entry("also late", 45.0));  // dropped
    reg.add(entry("right rush", 29.0));
    REQUIRE(reg.size() == 2);
    REQUIRE(reg.refusedCount() == 2);
    AutonMenu menu{reg, 0};
    const auto statusAt = [&menu](double t) { return std::string{menu.view(t).status.text}; };
    CHECK(statusAt(0.0) == "REFUSED " + std::string{reg.refusal(0)}.substr(0, 42));
    CHECK(statusAt(0.0).find("hardStopAt 31.00 s") != std::string::npos);
    CHECK(menu.view(0.0).status.tone == ui::Tone::Red);
    CHECK(statusAt(kMenuRefusalTurnSeconds * 0.99).rfind("REFUSED auton 'too late'", 0) == 0);
    CHECK(statusAt(kMenuRefusalTurnSeconds).rfind("REFUSED auton 'also late'", 0) == 0);
    CHECK(statusAt(2 * kMenuRefusalTurnSeconds + 0.1).rfind("REFUSED auton 'too late'", 0) == 0);
    CHECK(statusAt(-1.0).rfind("REFUSED auton 'too late'", 0) == 0);
    CHECK(statusAt(std::numeric_limits<double>::quiet_NaN()).rfind("REFUSED auton 'too late'", 0)
          == 0);
    CHECK(statusAt(1e300).rfind("REFUSED auton '", 0) == 0);
    for (double t = 0.0; t < 6.0; t += 0.5) {
        const MenuView v = menu.view(t);
        checkScreenSafe(v);
        CHECK(std::string_view{v.status.text}.size() <= ui::kSmallCols);
        for (std::size_t i = 0; i < v.buttonCount; ++i) {
            CHECK(std::string_view{v.buttons[i].label}.find("late") == std::string_view::npos);
        }
    }
    // Each dropped entry is named for 2 s: the number the changelog and chapter 17 state ("(2 s)
    // each"), pinned so a change of the constant cannot leave the documents saying the old one
    // (the T4-3 review's RV5).
    static_assert(kMenuRefusalTurnSeconds == 2.0,
                  "the documents say each dropped entry is named for 2 s -- reword them with it");
    (void)menu.lockChoice();
    CHECK(statusAt(0.0) == std::string{shulib::competition::kMenuLockedText});
    // Nothing dropped: no REFUSED word, whatever the time.
    const AutonRegistry clean = makeRegistry(2);
    AutonMenu plain{clean, 0};
    for (double t = 0.0; t < 6.0; t += 0.5) {
        CHECK(std::string{plain.view(t).status.text}.find("REFUSED") == std::string::npos);
    }
}

// Bug caught (the T4-3 review's RV2): MORE entries dropped than the registry keeps sentences for
// (kMaxAutonRefusals; kMaxAutons is 16, so 9 or more can be dropped), with the turn not clamped to
// the KEPT sentences -- every ninth turn would ask for a sentence that was never kept and the red
// line would read a bare `REFUSED `. Every turn of a whole cycle names a kept entry.
TEST_CASE("C5 the selector's turn walks only the KEPT sentences when more were dropped than kept") {
    using shulib::competition::kMenuRefusalTurnSeconds;
    using shulib::competition::OnRefusal;
    AutonRegistry reg{AutonPeriods{.match = Time{30.0}, .skills = Time{60.0}},
                      OnRefusal::DropEntry};
    char names[shulib::competition::kMaxAutonRefusals + 1][12]{};
    for (std::size_t i = 0; i < shulib::competition::kMaxAutonRefusals + 1; ++i) {
        std::snprintf(names[i], sizeof names[i], "late %d", static_cast<int>(i));
        AutonEntry e{};
        e.name = names[i];
        e.kind = AutonKind::Match;
        e.routine = &noop;
        e.start = StartPose::relative();
        e.endActionAt = Time{25.0};
        e.hardStopAt = Time{31.0};  // past the 30 s match period: dropped
        reg.add(e);
    }
    REQUIRE(reg.refusedCount() == shulib::competition::kMaxAutonRefusals + 1);
    AutonMenu menu{reg, 0};
    const double cycle =
        static_cast<double>(shulib::competition::kMaxAutonRefusals + 1) * kMenuRefusalTurnSeconds;
    std::set<std::string> named;
    for (double t = 0.0; t < 2.0 * cycle; t += kMenuRefusalTurnSeconds / 2.0) {
        const std::string status{menu.view(t).status.text};
        CAPTURE(t);
        CAPTURE(status);
        REQUIRE(status.rfind("REFUSED auton 'late ", 0) == 0);  // never the bare prefix
        named.insert(status.substr(0, std::string{"REFUSED auton 'late 0'"}.size()));
    }
    CHECK(named.size() == shulib::competition::kMaxAutonRefusals);  // the kept ones, each in turn
    CHECK(named.count("REFUSED auton 'late 8'") == 0U);             // the ninth was never kept
}
