#pragma once
//
// dropped_autons.hpp — THE WORDS for an auton the registry DROPPED at registration, said at boot by
// both programs that register the team's autons: "shulib Competition" and "shulib Auton Tester"
// (chunk T4 Part 3, 2026-10-04; the team lead's ruling of 2026-10-02 on COMP-b's C5, "drop just that
// auton").
//
// ── Why a drop, not a refusal ───────────────────────────────────────────────────────
// Until that ruling one auton entry the registry refused (a hard stop past its period, a name too
// long, a route that does not validate) refused the whole BOOT — driver control included: an auton
// bug cost the match's driver period. Both programs now build their registry with
// competition::OnRefusal::DropEntry (include/shulib/competition/auton_registry.hpp): a refused entry
// is registered as if it had never been written, its sentence is kept, and every other auton still
// runs — and, in the competition program, driver control (the Auton Tester's opcontrol() does
// nothing by design, so its line never claims it). The selector names each dropped entry in red
// while the choice is open (competition::AutonMenu's status line), and the boot says each here —
// on serial and the SD run log — so the student who wrote it is told by name, not left to wonder
// why it is missing. (The team lead's ruling was given for the competition program; the Auton
// Tester's drop is the coordinator's extension of it, the T4 Part 3 late step's instruction "the
// same for the Auton Tester if it shares the registration path" — which the team lead confirmed
// on 2026-10-08.)
//
// Two programs typing these sentences would drift, so they are made here, PROS-free and host-tested
// (test/dropped_autons_test.cpp). Every line is ASCII, within kDroppedAutonLineBytes (TermSink's
// 200 bytes: never cut), with a precision on every %s. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdio>

#include "common/robot_words.hpp"
#include "shulib/competition/auton_registry.hpp"

namespace shulib::bench {

/// The longest line sayDroppedAutons() hands out, terminator included (TermSink's 200 bytes).
inline constexpr std::size_t kDroppedAutonLineBytes = 200;

/// Hand `emit(const char*)` the boot's lines about the autons `registry` DROPPED, in order: none
/// when it dropped none; else one line saying what a drop means — and, when `driverControlRuns`
/// (the competition program; never the Auton Tester, whose driver control does nothing), that
/// driver control still runs — then one per dropped entry, `DROPPED: <its sentence>`, the
/// registry's own (it names the auton first), and, when more were dropped than the registry keeps
/// sentences for (competition::kMaxAutonRefusals), one line counting the rest. The first line says
/// which: `(each named below)`, or past the cap `(the first <cap> named below)`, the last line
/// counting the rest (the T4-3 verification: "each" was false past the cap). Each line is a buffer
/// valid only during its `emit` call. The first line names the robot's OWN autons file — robot two's
/// `src/team/autons.cpp` verbatim, robot one's `src/team/robot_one_autons.cpp` (chunk ROB1-A; `Words`,
/// src/common/robot_words.hpp — a choice between two literals, decided at compile time).
template <class Emit, class Words = ThisRobotWords>
void sayDroppedAutons(const competition::AutonRegistry& registry, bool driverControlRuns,
                      Emit&& emit, Words = {}) {
    const std::size_t dropped = registry.refusedCount();
    if (dropped == 0) {
        return;
    }
    const std::size_t kept =
        dropped < competition::kMaxAutonRefusals ? dropped : competition::kMaxAutonRefusals;
    // "the first <cap> named below" is at most 23 characters while the cap is one digit — the
    // precision below (%.23s) keeps the line within 200 bytes at a 10-digit count, and a cap of
    // ten or more would be cut there, so it is refused here instead.
    static_assert(competition::kMaxAutonRefusals < 10,
                  "dropped_autons.hpp: '(the first N named below)' fits %.23s only for a one-digit "
                  "cap -- widen the precision and re-check the 200-byte line (DA2)");
    char named[48];
    if (dropped > kept) {
        std::snprintf(named, sizeof named, "the first %u named below",
                      static_cast<unsigned>(kept));
    } else {
        std::snprintf(named, sizeof named, "each named below");
    }
    char line[kDroppedAutonLineBytes];
    std::snprintf(line, sizeof line,
                  Words::value.claw
                      ? "%u auton(s) REFUSED at registration and DROPPED -- the dropped ones are NOT on "
                        "the menu (%.23s); every other auton%s -- fix src/team/autons.cpp"
                      : "%u auton(s) REFUSED at registration and DROPPED -- NOT on the menu (%.23s); "
                        "every other auton%s -- fix src/team/robot_one_autons.cpp",
                  static_cast<unsigned>(dropped), static_cast<const char*>(named),
                  driverControlRuns ? " and driver control still run" : " still runs");
    emit(static_cast<const char*>(line));
    for (std::size_t i = 0; i < kept; ++i) {
        std::snprintf(line, sizeof line, "DROPPED: %.190s", registry.refusal(i));
        emit(static_cast<const char*>(line));
    }
    if (dropped > kept) {
        std::snprintf(line, sizeof line,
                      "DROPPED: %u more, whose sentences were not kept (the registry keeps %u)",
                      static_cast<unsigned>(dropped - kept), static_cast<unsigned>(kept));
        emit(static_cast<const char*>(line));
    }
}

}  // namespace shulib::bench
