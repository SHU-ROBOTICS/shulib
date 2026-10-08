#pragma once
//
// competition::AutonMenu — the brain-screen auton selector as a PURE MODEL (chunk COMP-a,
// 2026-09-26; the COMP brief's ruling C4).
//
// ── What it decides, and why it is a model ──────────────────────────────────────────
// The selector is the last thing a person touches before a match, usually in a hurry, often with
// gloves, on a resistive screen whose release point wanders. Everything that decides WHAT it says
// and WHICH touches mean something lives here, where the host suite can walk every reachable view
// — the calibrate panel's lesson (calibration/calibrate_panel.hpp), built on the same generic
// pieces (ui/touch_panel.hpp). The program only RENDERS a MenuView and hands in touch samples.
//
// The rules, each pinned by a test:
//   * NONE is ALWAYS on the screen, in the SAME place, and it is the DEFAULT: a program that boots
//     and is never touched runs no auton. (A robot that drives off the line because nobody chose
//     is the failure; a robot that sits still is the safe one.)
//   * One button per registered auton, labelled "<name> <KIND>", PAGED when more than
//     kMenuEntriesPerPage are registered; paging hides none (every entry is on exactly one page).
//   * No new button appears under the one just pressed: entries live in the middle of the screen,
//     NONE / PREV / NEXT in the bottom row, and a page flip either keeps a button where it was (the
//     same id) or leaves that place empty — so a reflexive double tap cannot answer a question the
//     first tap asked.
//   * After every layout change the new buttons are greyed and dead for ui::kArmingDelaySeconds,
//     except NONE, which is never delayed (choosing no auton is always safe).
//   * The CHOICE is one atomic value, readable from any task; lockChoice() freezes it and returns
//     it in the same atomic operation, so a touch racing the auton's start can never change the
//     auton that runs — and, once locked, no touch changes it at all (every button is dead).
//   * Every string it draws passes ui::screenText(): printable ASCII, no '%', within its budget —
//     literals by static_assert, names because the registry refused anything else.
//   * An entry the registry DROPPED (OnRefusal::DropEntry, the team lead's ruling of 2026-10-02)
//     is never a button — it was never registered — and the status line NAMES it, in red, while
//     the choice is not locked: `REFUSED auton '<name>': <why>`, cut at the line's budget (the
//     program's log has the whole sentence), each dropped entry in turn, kMenuRefusalTurnSeconds
//     apiece. A person choosing an auton must not be left to wonder why theirs is missing.
//
// ── What it does NOT decide ─────────────────────────────────────────────────────────
// Which task polls the screen, when to lock and unlock (the program: lock when the auton task
// starts the auton, unlock — if it wants a second autonomous() to take a new choice — only after
// that task has finished), pixels, fonts, RGB, and repainting only what changed. NOTHING HERE HAS
// RUN ON A ROBOT.
//
// PROS-free, allocation-free, header-only.

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "shulib/competition/auton_registry.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::competition {

/// How many auton buttons one page shows. Two full-width rows fit between the two status lines and
/// the bottom row with every vertical gap above 2 * ui::kHitPadY (the layout rule that keeps two
/// padded hit boxes from overlapping). The layout is an ESTIMATE like every screen number (HA-136).
inline constexpr std::size_t kMenuEntriesPerPage = 2;
/// The most buttons one view carries: the page's autons plus PREV, NONE and NEXT.
inline constexpr std::size_t kMenuMaxButtons = kMenuEntriesPerPage + 3;

/// The header bar (the current choice).
inline constexpr ui::Rect kMenuHeaderBar{0, 0, 479, 26};
/// The first auton row: full width, so a 29-character name and its kind fit one medium line.
inline constexpr ui::Rect kMenuEntryRow0{6, 66, 474, 100};
/// The second auton row.
inline constexpr ui::Rect kMenuEntryRow1{6, 122, 474, 156};
/// PREV, bottom left — drawn only when there is a previous page.
inline constexpr ui::Rect kMenuPrevSlot{6, 180, 154, 234};
/// NONE, bottom centre — drawn on EVERY view, always in this place.
inline constexpr ui::Rect kMenuNoneSlot{166, 180, 314, 234};
/// NEXT, bottom right — drawn only when there is a next page.
inline constexpr ui::Rect kMenuNextSlot{326, 180, 474, 234};

/// The auton rows, top to bottom.
inline constexpr ui::Rect kMenuEntryRows[kMenuEntriesPerPage] = {kMenuEntryRow0, kMenuEntryRow1};

/// The header's prefix; "AUTON: " + a kMaxAutonNameChars name is one medium line (B25).
inline constexpr std::string_view kMenuHeaderPrefix = "AUTON: ";
/// NONE's label.
inline constexpr std::string_view kMenuNoneLabel = "NONE";
/// PREV's label.
inline constexpr std::string_view kMenuPrevLabel = "< PREV";
/// NEXT's label.
inline constexpr std::string_view kMenuNextLabel = "NEXT >";
/// The status line while nothing is locked.
inline constexpr std::string_view kMenuHintText = "touch a name to choose it; NONE runs nothing";
/// The status line once the auton has started.
inline constexpr std::string_view kMenuLockedText = "LOCKED -- the auton has started";
/// The info line for NONE.
inline constexpr std::string_view kMenuNoneInfo = "NONE: no auton runs (the default)";
/// The status line's prefix before a dropped entry's sentence (file banner).
inline constexpr std::string_view kMenuRefusedPrefix = "REFUSED ";
/// How long the status line names each dropped entry before the next (file banner).
inline constexpr double kMenuRefusalTurnSeconds = 2.0;

// The layout rules, checked where they are written (the calibrate panel's discipline).
static_assert(kMenuHeaderPrefix.size() + kMaxAutonNameChars <= ui::kMediumCols,
              "AUTON: <name> must fit one medium line");
static_assert(kMaxAutonNameChars + 1 + std::string_view{"SKILLS"}.size() <= ui::kMediumCols,
              "an auton button's label '<name> SKILLS' must fit one medium line");
static_assert(ui::screenText(kMenuHeaderPrefix, ui::kMediumCols));
static_assert(ui::screenText(kMenuNoneLabel, ui::kMediumCols));
static_assert(ui::screenText(kMenuPrevLabel, ui::kMediumCols));
static_assert(ui::screenText(kMenuNextLabel, ui::kMediumCols));
static_assert(ui::screenText(kMenuHintText, ui::kSmallCols));
static_assert(ui::screenText(kMenuLockedText, ui::kSmallCols));
static_assert(ui::screenText(kMenuNoneInfo, ui::kSmallCols));
static_assert(ui::screenText(kMenuRefusedPrefix, ui::kSmallCols));
static_assert(ui::onScreen(kMenuHeaderBar) && ui::onScreen(kMenuEntryRow0)
              && ui::onScreen(kMenuEntryRow1) && ui::onScreen(kMenuPrevSlot)
              && ui::onScreen(kMenuNoneSlot) && ui::onScreen(kMenuNextSlot));
static_assert(kMenuEntryRow1.y0 - kMenuEntryRow0.y1 > 2 * ui::kHitPadY,
              "the two auton rows' padded hit boxes must not overlap");
static_assert(kMenuNoneSlot.y0 - kMenuEntryRow1.y1 > 2 * ui::kHitPadY,
              "the bottom row's padded hit boxes must not reach an auton row");
static_assert(kMenuNoneSlot.x0 - kMenuPrevSlot.x1 > 2 * ui::kHitPadX
                  && kMenuNextSlot.x0 - kMenuNoneSlot.x1 > 2 * ui::kHitPadX,
              "the bottom row's padded hit boxes must not overlap");

/// Which button.
enum class MenuButtonId : std::uint8_t {
    Entry,  ///< a registered auton (MenuButton::entry says which)
    None,   ///< NONE — always present, never delayed
    Prev,   ///< the previous page
    Next,   ///< the next page
};

/// One button of a view.
struct MenuButton {
    /// Which button.
    MenuButtonId id = MenuButtonId::None;
    /// For an Entry: the registry index it chooses. -1 otherwise.
    int entry = -1;
    /// Where it is drawn (its hit box is ui::hitBox(rect)).
    ui::Rect rect{};
    /// Its label, MEDIUM, screen-safe.
    char label[ui::kMediumCols + 1]{};
    /// False while the arming delay runs or once the choice is locked: drawn GREYED, never hit.
    bool armed = true;
    /// True for the button whose choice is the current one (drawn highlighted).
    bool chosen = false;
};

/// One line of text at a fixed place and size.
struct MenuLine {
    /// Left edge, pixels.
    std::int16_t x = 0;
    /// Top edge, pixels.
    std::int16_t y = 0;
    /// Its size; the text fits ui::budgetFor(size).
    ui::TextSize size = ui::TextSize::Medium;
    /// Its colour, by meaning.
    ui::Tone tone = ui::Tone::White;
    /// The text, NUL-terminated, screen-safe.
    char text[ui::kSmallCols + 1]{};
};

/// Everything one frame of the selector says, and which touches mean something.
struct MenuView {
    /// "AUTON: <name>" — the current choice — MEDIUM, in the header bar.
    MenuLine header{};
    /// The choice's kind, schedule and start mode (or what NONE means), SMALL.
    MenuLine info{};
    /// The hint, or LOCKED, and the page — SMALL.
    MenuLine status{};
    /// The buttons, in drawing order: the page's autons, then PREV (if any), NONE, NEXT (if any).
    MenuButton buttons[kMenuMaxButtons]{};
    /// How many entries of `buttons` are real.
    std::uint8_t buttonCount = 0;
    /// The page shown, 0-based.
    int page = 0;
    /// How many pages there are (at least 1 — the page with only NONE when nothing is registered).
    int pageCount = 1;
    /// The choice is locked: every button is dead.
    bool locked = false;
};

/// What a touch did.
enum class MenuTouchAction : std::uint8_t {
    None,           ///< nothing — no new release, a miss, or a greyed button
    Chose,          ///< the choice changed (or was re-chosen) — MenuTouch::choice holds it
    Paged,          ///< the page changed
    RefusedLocked,  ///< a live-looking button was hit after the choice locked — nothing changed
};

/// The outcome of one touch read.
struct MenuTouch {
    /// What happened.
    MenuTouchAction action = MenuTouchAction::None;
    /// The button index hit in the DRAWN view (-1 for none) — for the log.
    int hit = -1;
    /// True when this read carried a NEW release (logged even when it hits nothing).
    bool newRelease = false;
    /// The choice after this touch: a registry index, or -1 for NONE.
    int choice = -1;
};

/// The selector (file banner). Borrows the registry, which must be COMPLETE before the first
/// view() and outlive the menu. view() and touch() belong to ONE task (the one that paints);
/// choice(), chosenEntry(), lockChoice(), unlock() and locked() may be called from any task.
class AutonMenu {
public:
    /// `releaseCountAtStart`: PROS's release_count when the menu first appears, so a tap made
    /// before it existed is never replayed as a choice. The choice starts at NONE.
    AutonMenu(const AutonRegistry& registry, std::int32_t releaseCountAtStart) noexcept
        : registry_{&registry}, edge_{releaseCountAtStart} {}

    /// Not copyable or movable: the choice is an atomic other tasks read by address.
    AutonMenu(const AutonMenu&) = delete;
    /// See the copy constructor.
    AutonMenu& operator=(const AutonMenu&) = delete;
    /// See the copy constructor.
    AutonMenu(AutonMenu&&) = delete;
    /// See the copy constructor.
    AutonMenu& operator=(AutonMenu&&) = delete;
    /// Owns nothing that needs releasing.
    ~AutonMenu() = default;

    /// The view to paint at `nowSeconds`. Restarts the arming clock whenever the layout (page and
    /// buttons) differs from the last view this returned, so a new layout's buttons come up greyed
    /// on the very first frame they appear.
    [[nodiscard]] MenuView view(double nowSeconds) {
        MenuView v = build(0.0, nowSeconds);
        const double since = clock_.observe(layoutKey(v), nowSeconds);
        return build(since, nowSeconds);
    }

    /// One touch read against the view that is ON THE SCREEN (`drawn`). A new release on an armed
    /// button chooses its auton (or NONE) or flips the page; once locked, nothing changes.
    [[nodiscard]] MenuTouch touch(const ui::TouchSample& s, const MenuView& drawn) {
        MenuTouch r{};
        r.choice = choice();
        if (!edge_.take(s)) {
            return r;  // an error read, or no new release: x/y are stale — never act on them
        }
        r.newRelease = true;
        r.hit = ui::hitTest(drawn.buttons, drawn.buttonCount, s.x, s.y);
        if (r.hit < 0) {
            return r;
        }
        const MenuButton& b = drawn.buttons[static_cast<std::size_t>(r.hit)];
        switch (b.id) {
            case MenuButtonId::Prev:
                if (page_ > 0) {
                    --page_;
                    r.action = MenuTouchAction::Paged;
                }
                return r;
            case MenuButtonId::Next:
                if (page_ + 1 < pageCount()) {
                    ++page_;
                    r.action = MenuTouchAction::Paged;
                }
                return r;
            case MenuButtonId::None:
                r.action = choose(-1) ? MenuTouchAction::Chose : MenuTouchAction::RefusedLocked;
                r.choice = choice();
                return r;
            case MenuButtonId::Entry:
                if (b.entry < 0 || static_cast<std::size_t>(b.entry) >= registry_->size()) {
                    return r;  // a view from another registry: never act on it
                }
                r.action = choose(b.entry) ? MenuTouchAction::Chose : MenuTouchAction::RefusedLocked;
                r.choice = choice();
                return r;
        }
        return r;
    }

    /// The current choice: a registry index, or -1 for NONE (the default). Any task.
    [[nodiscard]] int choice() const noexcept { return indexOf(state_.load()); }

    /// The chosen entry, or nullptr for NONE. Any task.
    [[nodiscard]] const AutonEntry* chosenEntry() const noexcept {
        const int c = choice();
        return c < 0 ? nullptr : &registry_->at(static_cast<std::size_t>(c));
    }

    /// LOCK the choice and return it, in ONE atomic operation — the auton that runs is the choice
    /// at this instant, and no touch after it changes the choice (file banner). Any task.
    int lockChoice() noexcept { return indexOf(state_.fetch_or(kLockedBit)); }

    /// Unlock: touches may change the choice again. For the program to call only after the auton
    /// it locked for has FINISHED (a second autonomous() on the same boot then takes the choice
    /// current at ITS start). Any task.
    void unlock() noexcept { state_.fetch_and(~kLockedBit); }

    /// True while locked. Any task.
    [[nodiscard]] bool locked() const noexcept { return (state_.load() & kLockedBit) != 0; }

    /// The page shown, 0-based.
    [[nodiscard]] int page() const noexcept { return page_; }

    /// How many pages the registry needs (at least 1).
    [[nodiscard]] int pageCount() const noexcept {
        const std::size_t n = registry_->size();
        const std::size_t pages = (n + kMenuEntriesPerPage - 1) / kMenuEntriesPerPage;
        return pages == 0 ? 1 : static_cast<int>(pages);
    }

private:
    /// The state word: bit 30 = locked; the low bits = choice + 1 (0 = NONE). One atomic, so a
    /// lock and the choice it freezes can never disagree.
    static constexpr std::int32_t kLockedBit = 0x40000000;  // bit 30
    static constexpr std::int32_t kChoiceMask = kLockedBit - 1;

    [[nodiscard]] static int indexOf(std::int32_t state) noexcept {
        return static_cast<int>(state & kChoiceMask) - 1;
    }

    /// Set the choice to `index` (-1 = NONE) unless locked — compare-and-swap, so a lock that lands
    /// between the read and the write wins. Returns false when refused because locked.
    bool choose(int index) noexcept {
        std::int32_t s = state_.load();
        for (;;) {
            if ((s & kLockedBit) != 0) {
                return false;
            }
            if (state_.compare_exchange_weak(s, static_cast<std::int32_t>(index + 1))) {
                return true;
            }
        }
    }

    [[nodiscard]] static std::uint32_t layoutKey(const MenuView& v) noexcept {
        ui::LayoutKey k;
        k.mix(static_cast<std::uint32_t>(v.page));
        k.mix(v.buttonCount);
        for (std::size_t i = 0; i < v.buttonCount; ++i) {
            const MenuButton& b = v.buttons[i];
            k.mix(static_cast<std::uint32_t>(b.id));
            k.mix(static_cast<std::uint32_t>(b.entry));
            k.mix(b.rect);
        }
        return k.value();
    }

    static void setLine(MenuLine& line, std::int16_t x, std::int16_t y, ui::TextSize size,
                        ui::Tone tone, std::string_view text) noexcept {
        line.x = x;
        line.y = y;
        line.size = size;
        line.tone = tone;
        ui::copyScreenText(line.text, sizeof line.text, text, ui::budgetFor(size));
    }

    static void addButton(MenuView& v, MenuButtonId id, int entry, ui::Rect rect,
                          std::string_view label, bool armed, bool chosen) noexcept {
        MenuButton& b = v.buttons[v.buttonCount++];
        b.id = id;
        b.entry = entry;
        b.rect = rect;
        ui::copyScreenText(b.label, sizeof b.label, label, ui::kMediumCols);
        b.armed = armed;
        b.chosen = chosen;
    }

    /// Which dropped entry the status line names at `nowSeconds` (file banner): each in turn,
    /// kMenuRefusalTurnSeconds apiece; the first for a time that is not a number or is negative.
    [[nodiscard]] std::size_t refusalTurn(double nowSeconds) const noexcept {
        const std::size_t kept = registry_->refusedCount() < kMaxAutonRefusals
                                     ? registry_->refusedCount()
                                     : kMaxAutonRefusals;
        if (kept == 0 || !std::isfinite(nowSeconds) || nowSeconds < 0.0) {
            return 0;
        }
        const double turns = std::floor(nowSeconds / kMenuRefusalTurnSeconds);
        return turns < 1e9 ? static_cast<std::size_t>(turns) % kept : 0;
    }

    /// The whole view at `nowSeconds`, for a layout that has been up `sinceChange` seconds.
    [[nodiscard]] MenuView build(double sinceChange, double nowSeconds) const {
        MenuView v{};
        const std::int32_t state = state_.load();
        const int chosen = indexOf(state);
        v.locked = (state & kLockedBit) != 0;
        v.pageCount = pageCount();
        v.page = page_ < v.pageCount ? page_ : v.pageCount - 1;
        const bool armedNow = !v.locked && ui::ArmingClock::armedAfter(sinceChange);

        char text[ui::kSmallCols + 1] = "";
        const AutonEntry* e =
            chosen < 0 ? nullptr : &registry_->at(static_cast<std::size_t>(chosen));
        std::snprintf(text, sizeof text, "%s%s", kMenuHeaderPrefix.data(),
                      e == nullptr ? kMenuNoneLabel.data() : e->name);
        setLine(v.header, 6, 4, ui::TextSize::Medium, e == nullptr ? ui::Tone::Blue : ui::Tone::Green,
                text);
        if (e == nullptr) {
            setLine(v.info, 6, 30, ui::TextSize::Small, ui::Tone::White, kMenuNoneInfo);
        } else {
            std::snprintf(text, sizeof text, "%s park T+%.1fs stop T+%.1fs %s",
                          autonKindWord(e->kind), e->endActionAt.value(), e->hardStopAt.value(),
                          startModeWord(e->effectiveStart().mode()));
            setLine(v.info, 6, 30, ui::TextSize::Small, ui::Tone::White, text);
        }
        if (v.locked) {
            setLine(v.status, 6, 46, ui::TextSize::Small, ui::Tone::Amber, kMenuLockedText);
        } else if (registry_->refusedCount() > 0) {
            std::snprintf(text, sizeof text, "%s%.42s", kMenuRefusedPrefix.data(),
                          registry_->refusal(refusalTurn(nowSeconds)));
            setLine(v.status, 6, 46, ui::TextSize::Small, ui::Tone::Red, text);
        } else if (v.pageCount > 1) {
            std::snprintf(text, sizeof text, "PAGE %d/%d -- %s", v.page + 1, v.pageCount,
                          "touch a name");
            setLine(v.status, 6, 46, ui::TextSize::Small, ui::Tone::Grey, text);
        } else {
            setLine(v.status, 6, 46, ui::TextSize::Small, ui::Tone::Grey, kMenuHintText);
        }

        const std::size_t first = static_cast<std::size_t>(v.page) * kMenuEntriesPerPage;
        for (std::size_t row = 0; row < kMenuEntriesPerPage; ++row) {
            const std::size_t index = first + row;
            if (index >= registry_->size()) {
                break;
            }
            const AutonEntry& entry = registry_->at(index);
            std::snprintf(text, sizeof text, "%s %s", entry.name, autonKindWord(entry.kind));
            addButton(v, MenuButtonId::Entry, static_cast<int>(index), kMenuEntryRows[row], text,
                      armedNow, static_cast<int>(index) == chosen);
        }
        if (v.page > 0) {
            addButton(v, MenuButtonId::Prev, -1, kMenuPrevSlot, kMenuPrevLabel, armedNow, false);
        }
        // NONE: always present, always in the same place, never delayed — dead only when locked.
        addButton(v, MenuButtonId::None, -1, kMenuNoneSlot, kMenuNoneLabel, !v.locked, chosen < 0);
        if (v.page + 1 < v.pageCount) {
            addButton(v, MenuButtonId::Next, -1, kMenuNextSlot, kMenuNextLabel, armedNow, false);
        }
        return v;
    }

    const AutonRegistry* registry_;
    ui::ReleaseEdge edge_;
    ui::ArmingClock clock_{};
    int page_ = 0;
    std::atomic<std::int32_t> state_{0};  // NONE, unlocked — the default (file banner)
};

}  // namespace shulib::competition
