#pragma once
//
// DriveProtection — the old "shulib Drive"'s drive protections, in the library's driver loop,
// for every program that drives through it (chunk T4 Part 2, 2026-09-29).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// The team lead's ruling R4 (2026-09-27): "Moved into the library, so the competition program's
// opcontrol() and Teleop get them too, not just this program: the mid-match absence rule and the
// two re-arming cuts." Until this file only the old "shulib Drive" (src/programs/drive_program.cpp)
// had them, and it drives through bare adapters, never through the library. Now
// teleop::runDriverControl (teleop_loop.hpp) builds ONE DriveProtection per call and runs it on
// every tick, so every program that drives through the library's loop is protected with no wiring
// of its own — and no program can forget to be. The numbers are DriverLoopConfig::protection, a
// field whose defaults are the old program's, so a program that states nothing gets them.
//
// ── ONE TICK, in the loop's order ───────────────────────────────────────────────────
//   beforeDrive()  THE FROZEN RULE, first. Every group's FrozenReadingDetector
//                  (coupled_side_monitor.hpp) is handed each PRESENT member's fresh position()
//                  and velocity(). A member whose readings are FROZEN — repeated bit for bit for
//                  5 reads while it, or a side-mate whose readings still change, says the train
//                  turns, with at least as many of its side answering as not — is kept out of its
//                  group's fight monitor for this tick (MotorGroup::setMonitorExcluded) and out
//                  of the over-current watch. One FROZEN for the monitor's 250 ms is marked ABSENT
//                  (MotorGroup::markAbsent: out of every read, still commanded), logged once, and
//                  the degradation re-judged by the MID-RUN rule, evaluateRuntimeDegradation:
//                  more than two drive motors dead in total, anywhere, REFUSES — 0 V for the
//                  rest of the run (the team lead's N7 (c), "Only when 3+ are dead in total").
//                  It runs BEFORE drive() because drive() is where the group monitors are judged:
//                  a member found FROZEN here is already out of the monitor THIS tick, so a dead
//                  member's stale velocity can never start the fight streak that would cut
//                  (the 2026-09-27 re-check's race; the brief's landmine 16).
//   drive()        the loop's own call — the stick's command, or, while a cut or a refusal is in
//                  force (commandAllowed() false), a ZERO command, so the Localizer and the health
//                  tick keep running. The cruise feedforward makes a zero command exactly 0 V.
//   afterDrive()   a cut in force counts down and, at zero, RE-ARMS: every over-current streak
//                  cleared, every group's monitor reset, one line. Otherwise two checks, each of
//                  which CUTS:
//                  * the FIGHT: a group's own monitor, judged once in this tick's drive() with its
//                    ABSENT and FROZEN members already out, persisted — a member opposite its
//                    side's command above 0.5 rad/s, or under 25 % of the side's fastest, while the
//                    side is commanded above 1 V and turning above 1 rad/s, for 250 ms
//                    (HA-132/133). There is no second monitor.
//                  * the OVER-CURRENT: one member above `overCurrentAmps` (2.4 A) for
//                    `overCurrentTicks` (250 ms) CONSECUTIVE ticks, counted only while its side
//                    is commanded above the monitor's 1 V floor and never for an ABSENT or FROZEN
//                    member (a dead port's current is its last good reading, and one that died at
//                    the current limit would otherwise re-cut a standing robot every 1.25 s),
//                    judged per member, never on the side's total (HA-176). A full-stick
//                    REVERSAL at speed, or a full-stick launch from rest, is expected to hold
//                    every member near its current limit (believed to be the PROS default
//                    2.5 A, HA-154 — unmeasured) while the train turns round or spins up; whether
//                    that lasts under 250 ms on robot two is unmeasured too, so a healthy drive
//                    may cut mid-manoeuvre (HA-176's settle has the robot check; the host test
//                    plant synthesizes no current and cannot show it).
//                  A cut writes 0 V to every drive motor IN THIS TICK — drive() has already sent
//                  the stick's volts, and a zero command alone would start the cut a tick late —
//                  then holds the drive at 0 V for `cutTicks` (1 s), counts the cut and logs it
//                  with one `at the cut:` line per member, and re-arms. With `overCurrentCuts`
//                  false an over-current is logged and counted as a WARNING and nothing is cut
//                  (the team lead's N1 (a): the cut is ON, with one switch to turn it off, and a
//                  push check on the robot is required before any match).
//                  THE PEAK LINE: the tick a side's command falls back to 1 V or less, one line
//                  if that driven stretch's highest member current is a new high for this entry
//                  — the record a push check reads, which does not depend on the loop ever
//                  seeing the disable (HA-161; the team has no competition switch).
//   onDisable()    the loop's step 1, after cancel(): one summary line per group — each member's
//                  peak current and temperature this entry, and the cut count. A bonus: it prints
//                  only if the loop polls disabled() before PROS deletes the task. (A drive with
//                  no groups prints none.)
// Every line goes through the chassis's own telemetry sink with the loop's tag, `TEL`, in the old
// program's words, re-pointed from a bare port to the group's label ("port 13") and its side.
//
// ── What persists, and what does not ────────────────────────────────────────────────
// The ABSENT marks live in the GROUPS, which live in the program's static storage: they persist
// for the rest of the run — into the next driver-control entry, which re-judges them at entry (so
// a robot with 3+ dead is REFUSED from tick 1, every later start included, until the program is
// started again), and into a later autonomous (not acted on there: the scheduler's fault policy
// governs a motion). The cut, its count and the FROZEN exclusion do NOT persist: this object is
// built per call, and both enter() and its destructor clear every group's exclusion mask, so the
// disable return AND a throw out of the loop leave the groups as motion code expects
// (MOTOR_GROUP_DISAGREE in an autonomous motion is unchanged). If PROS deletes the task before the
// loop polls the disable, no destructor runs: the mask then holds the last tick's FROZEN members
// until the next driver-control entry clears it — stated, not solved.
//
// ── A drive motor that is not in a group ────────────────────────────────────────────
// (the invented X-drive's four wheels; a hand-built chassis) gets the over-current cut only,
// named by its wheel index and counted while that motor is commanded above 1 V. It has no side to
// be compared against, so there is no FROZEN rule and no fight for it — and so a bare motor's
// frozen current is NOT screened out of its over-current streak: stated, not solved. The X-drive
// refuses at boot on every robot that exists.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// Host-tested on the host test plant's member faults and on hand-stepped readings
// (test/drive_protection_test.cpp) — a train with inertia, a wall, a coast-down and a slipping
// member, which the plant cannot shape. NOTHING here has run on a robot. Every number is invented
// or reasoned: the 2.4 A over 250 ms (HA-176), the 5 reads and the 250 ms latch (HA-175), the
// fight's floors (HA-132/133). The per-tick reads — each present member's position, velocity,
// current and temperature, about 32 adapter reads per 10 ms tick on robot two — are what the old
// program read, and their cost on the brain is unmeasured (HA-32). So is this object's size on
// the driver-control task's stack (about 5 KB at kMaxProtectedGroups groups).
//
// PROS-free, sim-free, allocation-free.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <type_traits>

#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::teleop {

/// The drive protections' numbers — the old "shulib Drive"'s (src/programs/drive_program.cpp),
/// each with its source, and ONE new one, `frozenSuspectReads`. A program leaves this at its
/// defaults by naming no `protection` in its DriverLoopConfig. The fight's floors are NOT here:
/// they are each group's SideMonitorConfig (HA-132/133), one set of numbers as before.
/// runDriverControl checks it before anything moves (validProtectionConfig).
struct DriveProtectionConfig {
    /// A drive motor drawing MORE than this, in amperes, is over-current — the old program's
    /// `kCurrentLimitA`, INVENTED for a first run (A4 register HA-176). Finite and > 0.
    double overCurrentAmps = 2.4;
    /// Consecutive ticks over `overCurrentAmps` before an over-current counts (25 = 250 ms at
    /// 10 ms) — EQUAL to the fight monitor's `persistTicks` by choice, as the old program pinned it
    /// (HA-132). > 0.
    int overCurrentTicks = 25;
    /// How long a cut holds the drive at 0 V, in ticks (100 = the old program's 1 s). > 0.
    int cutTicks = 100;
    /// Cuts logged in full — the cut line and one `at the cut:` line per member; later cuts are
    /// counted but no longer logged per port, so a permanent fight cannot fill a log (the old
    /// program's 50). Over-current WARNINGS are logged within the same number, counted apart. > 0.
    int maxCutsLogged = 50;
    /// true (the default, the team lead's N1 (a)): an over-current CUTS the drive. false: it is
    /// logged and counted as a WARNING (DriveProtectionState::overCurrentWarnings) and nothing is
    /// cut — the one switch for a robot that must push. The fight cut is not switched by this.
    bool overCurrentCuts = true;
    /// The consecutive repeated reads — position AND velocity bit-identical — that make a member
    /// SUSPECT (FrozenReadingDetector; A4 register HA-175, invented). > 0 and below EVERY group's
    /// `persistTicks`: the frozen-reading rule must decide before the fight monitor can cut.
    int frozenSuspectReads = 5;
};

static_assert(DriveProtectionConfig{}.overCurrentTicks == SideMonitorConfig{}.persistTicks,
              "the 250 ms over-current window equals the fight monitor's window by choice "
              "(HA-132): change both, with a reason");
static_assert(DriveProtectionConfig{}.frozenSuspectReads < SideMonitorConfig{}.persistTicks,
              "the frozen-reading rule must decide before the fight monitor can cut on a dead "
              "member's stale velocity");

/// What the drive protections are doing to the drive right now.
enum class ProtectionMode {
    Driving,   ///< Every drive motor answers; the stick drives.
    Degraded,  ///< Some are dead (at boot, or marked mid-run) but no more than two: the stick drives, and the driver is told.
    Cut,       ///< A 1 s cut is in force (a fight, or an over-current): the drive is at 0 V, and re-arms by itself.
    Refused,   ///< More than two drive motors are dead in total: 0 V for the rest of the run.
};

/// The most motor groups one DriveProtection covers — a tank's two, or a four-wheel holonomic
/// with a group per wheel. runDriverControl refuses a chassis with more before anything moves.
inline constexpr std::size_t kMaxProtectedGroups = 4;

/// The size of each text field of DriveProtectionState, terminating NUL included.
inline constexpr std::size_t kProtectionTextBytes = 96;

/// What the protections are doing, for a panel — handed to a `perTick` that takes a fourth
/// argument (runDriverControl). Trivially copyable; every text is NUL-terminated, cut to fit.
struct DriveProtectionState {
    /// Driving, degraded, cut, or refused.
    ProtectionMode mode = ProtectionMode::Driving;
    /// Ticks of the cut in force still to run (0 when no cut is in force).
    int cutTicksLeft = 0;
    /// Cuts this driver-control entry — logged ones and unlogged ones alike.
    int cutCount = 0;
    /// Over-current WARNINGS this entry (DriveProtectionConfig::overCurrentCuts false).
    int overCurrentWarnings = 0;
    /// The last cut's words, as its log line gives them ("CUT: FIGHT on LEFT -- port(s) 18"); empty
    /// before the first cut.
    char lastCut[kProtectionTextBytes]{};
    /// Every DEAD drive port — a boot hole or a runtime mark — by label, in group order (LEFT's,
    /// then RIGHT's: "18 13"); empty when all answer. An unlabelled member is "m<index>".
    char dead[kProtectionTextBytes]{};
    /// The members FROZEN this tick, the same way ("13"). Empty almost always: a dead member
    /// passes through it for 250 ms on its way to `dead`, and a LIVE member may show for a few
    /// ticks as its train stops (the probes: at most 3 at a coast-down; at a wall, one member
    /// whose report settles sooner than its mates' shows for as long as their lagging reports
    /// still read above 1 rad/s — 9 ticks at 30 ms beside 80 ms). The exception is the rule's
    /// known gap (A4 register HA-175): a stalled LIVE member can stay here the full 250 ms and
    /// then be marked dead — at any wall when its report settles enough sooner (30 ms beside
    /// 120 ms in the host model), and beside a slipping side-mate at smaller differences.
    char frozen[kProtectionTextBytes]{};
    /// The degradation verdict in force, by the MID-RUN rule (evaluateRuntimeDegradation).
    DegradationVerdict verdict{.verdict = DriveVerdict::Drive};
    /// How many groups (sides) `peakAmps` holds.
    std::size_t groupCount = 0;
    /// Each group's highest member current this entry, amperes, over its present members that
    /// were not FROZEN (a frozen current is a stale one). Index = the chassis's motorGroups order
    /// (a tank: 0 LEFT, 1 RIGHT).
    double peakAmps[kMaxProtectedGroups]{};
};

static_assert(std::is_trivially_copyable_v<DriveProtectionState>,
              "a panel copies the state; keep it plain data");

/// The entry check runDriverControl makes before anything moves (the brief's P2-20): every number
/// of `cfg` finite and > 0, no more than kMaxProtectedGroups groups, and `frozenSuspectReads`
/// below every group's `persistTicks`. False, with the reason in `why` (cut to `cap`, always
/// NUL-terminated when cap > 0; `why` may be null), when one is not.
[[nodiscard]] inline bool validProtectionConfig(const DriveProtectionConfig& cfg,
                                                std::span<hal::MotorGroup* const> groups,
                                                char* why, std::size_t cap) noexcept {
    const auto refuse = [why, cap](const char* text) noexcept {
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap, "%s", text);
        }
        return false;
    };
    if (!(std::isfinite(cfg.overCurrentAmps) && cfg.overCurrentAmps > 0.0)) {
        return refuse("DriveProtectionConfig: overCurrentAmps must be finite and > 0");
    }
    if (cfg.overCurrentTicks <= 0) {
        return refuse("DriveProtectionConfig: overCurrentTicks must be > 0");
    }
    if (cfg.cutTicks <= 0) {
        return refuse("DriveProtectionConfig: cutTicks must be > 0");
    }
    if (cfg.maxCutsLogged <= 0) {
        return refuse("DriveProtectionConfig: maxCutsLogged must be > 0");
    }
    if (cfg.frozenSuspectReads <= 0) {
        return refuse("DriveProtectionConfig: frozenSuspectReads must be > 0");
    }
    if (groups.size() > kMaxProtectedGroups) {
        return refuse("DriveProtection: the chassis has more motor groups than the protections "
                      "cover (kMaxProtectedGroups)");
    }
    for (const hal::MotorGroup* g : groups) {
        if (g == nullptr) {
            return refuse("DriveProtection: a motorGroups entry is null");
        }
        if (cfg.frozenSuspectReads >= g->thresholds().persistTicks) {
            return refuse("DriveProtectionConfig: frozenSuspectReads must be below every group's "
                          "persistTicks -- a dead member must leave the fight monitor before the "
                          "monitor can cut on it");
        }
    }
    return true;
}

/// The drive protections for ONE driver-control entry (the file header): the frozen-reading rule
/// and runtime absence, the fight and over-current cuts, the mid-run refusal, and their log lines.
/// runDriverControl builds one per call and drives it — enter() before tick 1, then
/// beforeDrive() / commandAllowed() / afterDrive() every tick, onDisable() on the disable — so a
/// program never calls it; it is public so a test, or a program with its own loop, can.
/// Non-copyable, non-movable. The chassis, its groups, its drive motors and its sink must outlive
/// it; its destructor clears every group's exclusion mask.
class DriveProtection {
public:
    /// Over `chassis`'s drive motors, motor groups and telemetry sink, with `cfg` and the loop's
    /// tick period (for the durations its lines print). PRECONDITIONS: validProtectionConfig(cfg,
    /// the chassis's motorGroups) holds, and `tickPeriod` is finite and > 0 — exactly the two
    /// runDriverControl checks before it commands anything, so the loop's DriveProtection can
    /// never refuse after the loop's entry has moved something (any tick the loop accepts prints:
    /// "12.5 ms" at a 0.5 ms tick).
    DriveProtection(chassis::Chassis& chassis, const DriveProtectionConfig& cfg,
                    units::Time tickPeriod)
        : groups_{chassis.deps().motorGroups},
          drive_{chassis.deps().ctx->driveMotors()},
          log_{&chassis.deps().ctx->telemetry()},
          cfg_{cfg},
          tickSeconds_{tickPeriod.value()} {
        char why[kLineBytes] = "";
        SHULIB_PRECONDITION(validProtectionConfig(cfg_, groups_, why, sizeof why), why);
        SHULIB_PRECONDITION(std::isfinite(tickSeconds_) && tickSeconds_ > 0.0,
                            "DriveProtection: the tick period must be finite and > 0");
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            detectors_[g].emplace(groups_[g]->thresholds(), cfg_.frozenSuspectReads);
        }
        for (std::size_t w = 0; w < drive_.size() && bareCount_ < kMaxBare; ++w) {
            if (!isGroup(drive_[w])) {
                bare_[bareCount_++] = w;
            }
        }
        state_.groupCount = groups_.size();
    }

    /// Clears every group's exclusion mask, so the disable return and a throw out of the loop both
    /// leave the groups as motion code expects (the file header). The ABSENT marks stay.
    ~DriveProtection() { clearMasks(); }

    /// Non-copyable: it holds per-member state for groups it does not own.
    DriveProtection(const DriveProtection&) = delete;
    /// Non-movable, same reasoning.
    DriveProtection(DriveProtection&&) = delete;
    /// Non-copy-assignable, same reasoning.
    DriveProtection& operator=(const DriveProtection&) = delete;
    /// Non-move-assignable, same reasoning.
    DriveProtection& operator=(DriveProtection&&) = delete;

    /// Before tick 1: clear every group's exclusion mask and judge the drive by the MID-RUN rule
    /// from the groups as they stand — runtime marks from an earlier entry included. More than two
    /// dead in total REFUSES here, with one line, and the drive gets 0 V from tick 1; anything dead
    /// is said in one DEGRADED line; a complete drive says nothing.
    void enter() {
        clearMasks();
        rejudge(true);
    }

    /// Every tick, BEFORE drive(): the frozen-reading rule over every present member, the FROZEN
    /// members kept out of their group's monitor for this tick, and a member FROZEN for the
    /// monitor's window marked ABSENT (one line, and the degradation re-judged — which may REFUSE).
    void beforeDrive() {
        state_.frozen[0] = '\0';
        bool anyMarked = false;
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            hal::MotorGroup& grp = *groups_[g];
            const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
            MemberReading readings[kMaxSideMembers];
            for (std::size_t i = 0; i < n; ++i) {
                if (grp.isPresent(i)) {
                    const hal::IMotor& m = grp.member(i);
                    readings[i] = MemberReading{m.position().value(), m.velocity().value(), true};
                } else {
                    readings[i] = MemberReading{0.0, 0.0, false};  // never read (the header)
                }
            }
            const FrozenVerdict v =
                detectors_[g]->update(std::span<const MemberReading>{readings, n});
            frozen_[g] = v.frozenMask;
            grp.setMonitorExcluded(v.frozenMask);
            for (std::size_t i = 0; i < n; ++i) {
                if ((v.frozenMask & bit(i)) != 0) {
                    appendToken(state_.frozen, g, i);
                }
            }
            for (std::size_t i = 0; i < n; ++i) {
                if ((v.latchedMask & bit(i)) == 0 || !grp.markAbsent(i)) {
                    continue;  // a refused mark (the last present member) is left present
                }
                anyMarked = true;
                frozen_[g] &= ~bit(i);
                overTicks_[g][i] = 0;
                char who[kNameBytes];
                memberName(g, i, who, sizeof who);
                char window[kNameBytes];
                durationText(grp.thresholds().persistTicks, window, sizeof window);
                char line[kLineBytes];
                std::snprintf(line, sizeof line,
                              "WARNING: %s (%s) went ABSENT at runtime: reads frozen for %s while "
                              "its side answered -- dropped from the fight detector, still "
                              "commanded (harmless)",
                              who, groupName(g), window);
                say(hal::LogLevel::Warn, line);
            }
        }
        if (anyMarked) {
            rejudge(false);
        }
    }

    /// Whether this tick's drive() may take the stick's command: false while a cut or a refusal is
    /// in force, when the loop hands drive() a zero command instead.
    [[nodiscard]] bool commandAllowed() const noexcept { return !refused_ && cutTicksLeft_ == 0; }

    /// Every tick, AFTER drive(): this tick's currents and peaks, the driven stretches (the peak
    /// line), then a cut in force counting down (and re-arming at zero) — or, with none in force
    /// and no refusal, the fight and over-current checks, either of which may CUT now.
    void afterDrive() {
        readCurrents();
        trackStretches();
        if (refused_) {
            publish();
            return;
        }
        if (cutTicksLeft_ > 0) {
            --cutTicksLeft_;
            if (cutTicksLeft_ == 0) {
                rearm();
            }
            publish();
            return;
        }
        // The FIGHT: the first group whose own monitor persisted in this tick's drive().
        std::size_t fightGroup = kNone;
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            if (groups_[g]->lastVerdict().persistedCount > 0) {
                fightGroup = g;
                break;
            }
        }
        // The OVER-CURRENT: every streak advances every tick; the first to reach the window wins.
        std::size_t overGroup = kNone;
        std::size_t overMember = kNone;
        advanceGroupStreaks(overGroup, overMember);
        std::size_t overBare = kNone;
        advanceBareStreaks(overBare);
        char words[kProtectionTextBytes];
        if (fightGroup != kNone) {
            fightWords(fightGroup, words, sizeof words);
            cut(words);
        } else if (overGroup != kNone || overBare != kNone) {
            if (cfg_.overCurrentCuts) {
                overCurrentWords(overGroup, overMember, overBare, words, sizeof words);
                cut(words);
            } else {
                warnOverCurrent();
            }
        }
        publish();
    }

    /// The loop's step 1, AFTER cancel(): one summary line per group (each member's peak current
    /// and temperature this entry, and the cut count). Bare drive motors (no group) get none —
    /// they have no stretch or peak record either, only the over-current cut.
    void onDisable() {
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const hal::MotorGroup& grp = *groups_[g];
            char line[kLineBytes];
            int used = std::snprintf(line, sizeof line, "drive at the disable: %s, cuts %d",
                                     groupName(g), cutCount_);
            const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
            for (std::size_t i = 0; i < n && used > 0 && static_cast<std::size_t>(used) < sizeof line;
                 ++i) {
                char who[kNameBytes];
                memberName(g, i, who, sizeof who);
                char* at = line + used;
                const std::size_t room = sizeof line - static_cast<std::size_t>(used);
                int wrote = 0;
                if ((grp.absentMask() & bit(i)) != 0) {
                    wrote = std::snprintf(at, room, " | %s dead at boot", who);
                } else if ((grp.runtimeAbsentMask() & bit(i)) != 0) {
                    wrote = std::snprintf(at, room, " | %s DEAD (to %.2f A %.0f C)", who,
                                          peakAmps_[g][i], peakTempC_[g][i]);
                } else {
                    wrote = std::snprintf(at, room, " | %s %.2f A %.0f C", who, peakAmps_[g][i],
                                          peakTempC_[g][i]);
                }
                used = wrote < 0 ? -1 : used + wrote;
            }
            say(hal::LogLevel::Info, line);
        }
    }

    /// What the protections are doing right now (for a panel).
    [[nodiscard]] const DriveProtectionState& state() const noexcept { return state_; }

    /// The numbers this entry runs under.
    [[nodiscard]] const DriveProtectionConfig& config() const noexcept { return cfg_; }

private:
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);
    static constexpr std::size_t kMaxBare =
        static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels);
    static constexpr std::size_t kLineBytes = 200;  // TermSink's message budget
    static constexpr std::size_t kNameBytes = 24;
    static constexpr int kCountCap = 0x40000000;  // 2^30

    /// One side's driven stretch, for the peak line.
    struct Stretch {
        bool driven = false;
        int ticks = 0;
        double peak = 0.0;
        std::size_t peakMember = kNone;
        int startCutCount = 0;
        double entryHigh = 0.0;
    };

    [[nodiscard]] static constexpr std::uint32_t bit(std::size_t i) noexcept {
        return std::uint32_t{1} << i;
    }

    [[nodiscard]] bool isGroup(const hal::IMotor* m) const noexcept {
        for (const hal::MotorGroup* g : groups_) {
            if (static_cast<const hal::IMotor*>(g) == m) {
                return true;
            }
        }
        return false;
    }

    /// "LEFT" / "RIGHT" on a two-group chassis (the Robot's order), else "group <g>".
    [[nodiscard]] const char* groupName(std::size_t g) const noexcept {
        static constexpr const char* kSides[2] = {"LEFT", "RIGHT"};
        static constexpr const char* kGroups[kMaxProtectedGroups] = {"group 0", "group 1",
                                                                     "group 2", "group 3"};
        if (groups_.size() == 2 && g < 2) {
            return kSides[g];
        }
        return g < kMaxProtectedGroups ? kGroups[g] : "group ?";
    }

    /// "port 13" (the group's label) or "member 2" (no label).
    void memberName(std::size_t g, std::size_t i, char* buf, std::size_t cap) const noexcept {
        const int label = groups_[g]->label(i);
        if (label > 0) {
            std::snprintf(buf, cap, "port %d", label);
        } else {
            std::snprintf(buf, cap, "member %u", static_cast<unsigned>(i));
        }
    }

    /// Append member `i` of group `g` to a space-separated list: "13", or "m2" unlabelled.
    void appendToken(char* list, std::size_t g, std::size_t i) const noexcept {
        const std::size_t len = std::strlen(list);
        if (len + 1 >= kProtectionTextBytes) {
            return;
        }
        const int label = groups_[g]->label(i);
        char* at = list + len;
        const std::size_t room = kProtectionTextBytes - len;
        if (label > 0) {
            std::snprintf(at, room, "%s%d", len == 0 ? "" : " ", label);
        } else {
            std::snprintf(at, room, "%sm%u", len == 0 ? "" : " ", static_cast<unsigned>(i));
        }
    }

    /// `ticks` of this loop as words: "250 ms", "1 s" (a whole number of seconds, from 1 s up),
    /// "12.5 ms" at a 0.5 ms tick. Computed in double from the tick the loop accepted, so no tick
    /// that runDriverControl lets run can make it refuse or print a false duration.
    void durationText(int ticks, char* buf, std::size_t cap) const noexcept {
        const double seconds = static_cast<double>(ticks) * tickSeconds_;
        const double whole = std::round(seconds);
        if (seconds >= 1.0 && std::abs(seconds - whole) <= 1e-9 * seconds) {
            std::snprintf(buf, cap, "%.6g s", whole);
        } else {
            std::snprintf(buf, cap, "%.6g ms", seconds * 1000.0);
        }
    }

    void say(hal::LogLevel level, const char* line) { log_->log(level, "TEL", line); }

    void clearMasks() noexcept {
        for (hal::MotorGroup* g : groups_) {
            g->setMonitorExcluded(0);
        }
    }

    /// 0 V to every drive motor, now (a group fans it to every member it commands).
    void zeroDrive() {
        for (hal::IMotor* m : drive_) {
            m->setVoltage(units::Voltage{0.0});
        }
    }

    /// The census, by the MID-RUN rule: group 0 as one side, every other group as the other.
    [[nodiscard]] DegradationVerdict census() const noexcept {
        if (groups_.empty()) {
            return DegradationVerdict{.verdict = DriveVerdict::Drive};
        }
        SideCount left{static_cast<int>(groups_[0]->memberCount()),
                       static_cast<int>(groups_[0]->presentCount())};
        SideCount right{};
        for (std::size_t g = 1; g < groups_.size(); ++g) {
            right.expected += static_cast<int>(groups_[g]->memberCount());
            right.answering += static_cast<int>(groups_[g]->presentCount());
        }
        return evaluateRuntimeDegradation(left, right);
    }

    [[nodiscard]] static const char* verdictWord(DriveVerdict v) noexcept {
        return v == DriveVerdict::Drive           ? "DRIVE"
               : v == DriveVerdict::DriveDegraded ? "DRIVE_DEGRADED"
                                                  : "REFUSE";
    }

    /// The reason without its leading "REFUSE: " (the line already says REFUSED).
    [[nodiscard]] static const char* refusalReason(const char* reason) noexcept {
        return std::strncmp(reason, "REFUSE: ", 8) == 0 ? reason + 8 : reason;
    }

    /// Re-judge the degradation from the groups (at entry, or after a mark), say it, and REFUSE
    /// when the rule says so.
    void rejudge(bool atEntry) {
        const DegradationVerdict v = census();
        state_.verdict = v;
        state_.dead[0] = '\0';
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const std::size_t n = std::min(groups_[g]->memberCount(), kMaxSideMembers);
            for (std::size_t i = 0; i < n; ++i) {
                if (!groups_[g]->isPresent(i)) {
                    appendToken(state_.dead, g, i);
                }
            }
        }
        char line[kLineBytes];
        if (!atEntry) {
            std::snprintf(line, sizeof line,
                          "degradation re-evaluated: %s (left dead %d, right dead %d, total %d) -- "
                          "dead: %.60s",
                          verdictWord(v.verdict), v.leftDead, v.rightDead, v.totalDead,
                          state_.dead);
            say(hal::LogLevel::Warn, line);
        }
        if (v.verdict == DriveVerdict::Refuse) {
            if (!refused_) {
                refused_ = true;
                cutTicksLeft_ = 0;
                zeroDrive();
                if (atEntry) {
                    std::snprintf(line, sizeof line,
                                  "REFUSED at the start of driver control: %.40s (left dead %d, "
                                  "right dead %d, total %d) -- 0 V for the rest of the run",
                                  refusalReason(v.reason), v.leftDead, v.rightDead, v.totalDead);
                    say(hal::LogLevel::Error, line);
                    std::snprintf(line, sizeof line,
                                  "  dead: %.60s -- restart the program once the robot is fixed",
                                  state_.dead);
                } else {
                    // The reason bounded at 68, the longest the mid-run rule gives ("motor counts
                    // make no sense (negative, or more answering than listed)"): that line is then
                    // exactly 199 characters, and GCC 13 at -O3 (host) stops seeing 200+ (the
                    // review of 2026-09-30's "check the other snprintf sites" — found here).
                    std::snprintf(line, sizeof line,
                                  "REFUSED: %.68s -- 0 V for the rest of the run, every later "
                                  "start of driver control included; restart the program once "
                                  "the robot is fixed",
                                  refusalReason(v.reason));
                }
                say(hal::LogLevel::Error, line);
            }
        } else if (atEntry && v.verdict == DriveVerdict::DriveDegraded) {
            std::snprintf(line, sizeof line,
                          "driver control starts DEGRADED (left dead %d, right dead %d, total %d) "
                          "-- dead: %.60s",
                          v.leftDead, v.rightDead, v.totalDead, state_.dead);
            say(hal::LogLevel::Warn, line);
        }
        publish();
    }

    /// This tick's currents and temperatures, read ONLY from present members (a runtime-ABSENT
    /// member is never read), and the entry's peaks.
    void readCurrents() {
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const hal::MotorGroup& grp = *groups_[g];
            const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
            for (std::size_t i = 0; i < n; ++i) {
                if (!grp.isPresent(i)) {
                    ampsNow_[g][i] = 0.0;
                    continue;
                }
                const hal::IMotor& m = grp.member(i);
                const double amps = m.current().value();
                const double temp = m.temperature();
                ampsNow_[g][i] = amps;
                peakAmps_[g][i] = std::max(peakAmps_[g][i], amps);
                peakTempC_[g][i] = std::max(peakTempC_[g][i], temp);
                if ((frozen_[g] & bit(i)) == 0) {
                    state_.peakAmps[g] = std::max(state_.peakAmps[g], amps);
                }
            }
        }
        for (std::size_t b = 0; b < bareCount_; ++b) {
            bareAmpsNow_[b] = drive_[bare_[b]]->current().value();
        }
    }

    /// Each group's driven stretch, on the command drive() gave this tick: its peak member current
    /// while driven, and the peak line the tick the command falls back to the floor or below.
    void trackStretches() {
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const hal::MotorGroup& grp = *groups_[g];
            Stretch& st = stretch_[g];
            const bool driven =
                std::abs(grp.commandedVoltage().value()) > grp.thresholds().commandFloorV;
            if (driven) {
                if (!st.driven) {
                    st.driven = true;
                    st.ticks = 0;
                    st.peak = 0.0;
                    st.peakMember = kNone;
                    st.startCutCount = cutCount_;
                }
                st.ticks = std::min(st.ticks + 1, kCountCap);
                const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
                for (std::size_t i = 0; i < n; ++i) {
                    if (grp.isPresent(i) && (frozen_[g] & bit(i)) == 0 && ampsNow_[g][i] > st.peak) {
                        st.peak = ampsNow_[g][i];
                        st.peakMember = i;
                    }
                }
                continue;
            }
            if (!st.driven) {
                continue;
            }
            st.driven = false;
            if (st.peakMember == kNone || !(st.peak > st.entryHigh)) {
                continue;
            }
            st.entryHigh = st.peak;
            char who[kNameBytes];
            memberName(g, st.peakMember, who, sizeof who);
            char tail[kNameBytes] = "no cut";
            if (cutCount_ > st.startCutCount) {
                std::snprintf(tail, sizeof tail, "cut #%d", cutCount_);
            }
            char line[kLineBytes];
            std::snprintf(line, sizeof line, "drive: %s peak %.2f A (%s) over a %.1f s drive, %s",
                          groupName(g), st.peak, who, static_cast<double>(st.ticks) * tickSeconds_,
                          tail);
            say(hal::LogLevel::Info, line);
        }
    }

    /// Advance every group member's over-current streak; the first at or past the window is the
    /// offender (kNone when none is).
    void advanceGroupStreaks(std::size_t& overGroup, std::size_t& overMember) noexcept {
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const hal::MotorGroup& grp = *groups_[g];
            const bool driven =
                std::abs(grp.commandedVoltage().value()) > grp.thresholds().commandFloorV;
            const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
            for (std::size_t i = 0; i < n; ++i) {
                const bool counted = driven && grp.isPresent(i) && (frozen_[g] & bit(i)) == 0
                                     && ampsNow_[g][i] > cfg_.overCurrentAmps;
                overTicks_[g][i] = counted ? std::min(overTicks_[g][i] + 1, kCountCap) : 0;
                if (overGroup == kNone && overTicks_[g][i] >= cfg_.overCurrentTicks) {
                    overGroup = g;
                    overMember = i;
                }
            }
        }
    }

    /// The same for the drive motors that are in no group, each against its own command.
    void advanceBareStreaks(std::size_t& overBare) noexcept {
        for (std::size_t b = 0; b < bareCount_; ++b) {
            const hal::IMotor& m = *drive_[bare_[b]];
            const bool counted =
                std::abs(m.commandedVoltage().value()) > kBareCommandFloorV
                && bareAmpsNow_[b] > cfg_.overCurrentAmps;
            bareOverTicks_[b] = counted ? std::min(bareOverTicks_[b] + 1, kCountCap) : 0;
            if (overBare == kNone && bareOverTicks_[b] >= cfg_.overCurrentTicks) {
                overBare = b;
            }
        }
    }

    /// The fight's words, the old program's: the whole side, or the fighting ports.
    void fightWords(std::size_t g, char* buf, std::size_t cap) const noexcept {
        const hal::MotorGroup& grp = *groups_[g];
        const SideVerdict& v = grp.lastVerdict();
        char names[kProtectionTextBytes] = "";
        bool labelled = true;
        const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
        for (std::size_t i = 0; i < n; ++i) {
            labelled = labelled && ((v.persistedMask & bit(i)) == 0 || grp.label(i) > 0);
        }
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t len = std::strlen(names);
            if ((v.persistedMask & bit(i)) == 0 || len + 1 >= sizeof names) {
                continue;
            }
            // The ports when every one named has a label; otherwise the members' indexes.
            std::snprintf(names + len, sizeof names - len, "%s%d", len == 0 ? "" : " ",
                          labelled ? grp.label(i) : static_cast<int>(i));
        }
        if (v.persistedCount == v.presentCount) {
            std::snprintf(buf, cap, "CUT: whole %s side OPPOSITE/stalled vs command (%s %s)",
                          groupName(g), labelled ? "ports" : "members", names);
        } else {
            std::snprintf(buf, cap, "CUT: FIGHT on %s -- %s %s", groupName(g),
                          labelled ? "port(s)" : "member(s)", names);
        }
    }

    /// The over-current's words: "CUT: port 17 (LEFT) over 2.4 A for 250 ms".
    void overCurrentWords(std::size_t g, std::size_t i, std::size_t b, char* buf,
                          std::size_t cap) const noexcept {
        char who[kNameBytes];
        char where[kNameBytes];
        if (g != kNone) {
            memberName(g, i, who, sizeof who);
            std::snprintf(where, sizeof where, " (%s)", groupName(g));
        } else {
            std::snprintf(who, sizeof who, "wheel %u", static_cast<unsigned>(bare_[b]));
            where[0] = '\0';
        }
        char window[kNameBytes];
        durationText(cfg_.overCurrentTicks, window, sizeof window);
        // Each %s bounded (the ARM build's -Wformat-truncation saw 98 bytes worst case into the
        // 96 of DriveProtectionState::lastCut): "port 20" / "member 20" / "wheel 3" and " (RIGHT)"
        // / " (group 3)" are far inside 16 and 12, and a window prints whole (23 is its buffer's).
        std::snprintf(buf, cap, "CUT: %.16s%.12s over %.3g A for %.23s", who, where,
                      cfg_.overCurrentAmps, window);
    }

    /// CUT NOW: 0 V to every drive motor in this tick, the cut held for cutTicks, counted, and
    /// logged within the bound (the cut line, then one `at the cut:` line per member).
    void cut(const char* words) {
        zeroDrive();
        cutTicksLeft_ = cfg_.cutTicks;
        cutCount_ = std::min(cutCount_ + 1, kCountCap);
        std::snprintf(state_.lastCut, sizeof state_.lastCut, "%s", words);
        if (cutCount_ > cfg_.maxCutsLogged) {
            return;
        }
        char lasting[kNameBytes];
        durationText(cfg_.cutTicks, lasting, sizeof lasting);
        char line[kLineBytes];
        std::snprintf(line, sizeof line, "%s -- %s 0 V for %s, cut #%d, then re-arm", words,
                      groups_.size() == 2 ? "both sides" : "every drive motor", lasting, cutCount_);
        say(hal::LogLevel::Warn, line);
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const hal::MotorGroup& grp = *groups_[g];
            const std::size_t n = std::min(grp.memberCount(), kMaxSideMembers);
            for (std::size_t i = 0; i < n; ++i) {
                char who[kNameBytes];
                memberName(g, i, who, sizeof who);
                if ((grp.absentMask() & bit(i)) != 0) {
                    std::snprintf(line, sizeof line, "  at the cut: %s %s ABSENT (dead at boot)",
                                  groupName(g), who);
                } else if ((grp.runtimeAbsentMask() & bit(i)) != 0) {
                    std::snprintf(line, sizeof line, "  at the cut: %s %s ABSENT (not read)",
                                  groupName(g), who);
                } else {
                    const hal::IMotor& m = grp.member(i);
                    std::snprintf(line, sizeof line,
                                  "  at the cut: %s %s v=%+7.2f rad/s %5.2f A %3.0f C%s",
                                  groupName(g), who, m.velocity().value(), ampsNow_[g][i],
                                  m.temperature(), (frozen_[g] & bit(i)) != 0 ? " FROZEN" : "");
                }
                say(hal::LogLevel::Info, line);
            }
        }
        for (std::size_t b = 0; b < bareCount_; ++b) {
            const hal::IMotor& m = *drive_[bare_[b]];
            std::snprintf(line, sizeof line, "  at the cut: wheel %u v=%+7.2f rad/s %5.2f A %3.0f C",
                          static_cast<unsigned>(bare_[b]), m.velocity().value(), bareAmpsNow_[b],
                          m.temperature());
            say(hal::LogLevel::Info, line);
        }
        if (cutCount_ == cfg_.maxCutsLogged) {
            say(hal::LogLevel::Info, "  (further cuts are counted but no longer logged per port)");
        }
    }

    /// With over-current cuts OFF: one WARNING per member per episode, the tick its streak first
    /// reaches the window, counted, and logged within the same bound as cuts.
    void warnOverCurrent() {
        char window[kNameBytes];
        durationText(cfg_.overCurrentTicks, window, sizeof window);
        const auto warn = [&](const char* who, const char* where) {
            warnings_ = std::min(warnings_ + 1, kCountCap);
            if (warnings_ > cfg_.maxCutsLogged) {
                return;
            }
            // Each %s bounded, as in overCurrentWords(): GCC 13 sizes this call at -O1 and above
            // (not at the brain's -Os, so far) and saw 203 bytes worst case into the 200-byte line.
            // Bounded, the worst case is 184 characters; a real line is about 150.
            char line[kLineBytes];
            std::snprintf(line, sizeof line,
                          "WARNING: %.16s%.12s over %.3g A for %.23s -- over-current cuts are OFF "
                          "(DriveProtectionConfig::overCurrentCuts): not cut, warning #%d",
                          who, where, cfg_.overCurrentAmps, window, warnings_);
            say(hal::LogLevel::Warn, line);
        };
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            const std::size_t n = std::min(groups_[g]->memberCount(), kMaxSideMembers);
            for (std::size_t i = 0; i < n; ++i) {
                if (overTicks_[g][i] == cfg_.overCurrentTicks) {
                    char who[kNameBytes];
                    memberName(g, i, who, sizeof who);
                    char where[kNameBytes];
                    std::snprintf(where, sizeof where, " (%s)", groupName(g));
                    warn(who, where);
                }
            }
        }
        for (std::size_t b = 0; b < bareCount_; ++b) {
            if (bareOverTicks_[b] == cfg_.overCurrentTicks) {
                char who[kNameBytes];
                std::snprintf(who, sizeof who, "wheel %u", static_cast<unsigned>(bare_[b]));
                warn(who, "");
            }
        }
    }

    /// RE-ARM after a cut: every over-current streak cleared, every group's monitor reset, one
    /// line. The frozen-reading detectors are NOT reset: a dead member keeps counting toward
    /// ABSENT through a cut.
    void rearm() {
        for (std::size_t g = 0; g < groups_.size(); ++g) {
            for (int& t : overTicks_[g]) {
                t = 0;
            }
            groups_[g]->resetDisagreement();
        }
        for (int& t : bareOverTicks_) {
            t = 0;
        }
        char lasting[kNameBytes];
        durationText(cfg_.cutTicks, lasting, sizeof lasting);
        char line[kLineBytes];
        std::snprintf(line, sizeof line, "re-armed after cut #%d (%s at 0 V)", cutCount_, lasting);
        say(hal::LogLevel::Info, line);
    }

    /// The state a panel reads, brought up to date.
    void publish() noexcept {
        state_.mode = refused_               ? ProtectionMode::Refused
                      : cutTicksLeft_ > 0    ? ProtectionMode::Cut
                      : state_.verdict.verdict == DriveVerdict::DriveDegraded
                          ? ProtectionMode::Degraded
                          : ProtectionMode::Driving;
        state_.cutTicksLeft = cutTicksLeft_;
        state_.cutCount = cutCount_;
        state_.overCurrentWarnings = warnings_;
    }

    /// A bare drive motor's command floor: the fight monitor's default (1 V), since a bare motor
    /// has no group thresholds of its own.
    static constexpr double kBareCommandFloorV = SideMonitorConfig{}.commandFloorV;

    std::span<hal::MotorGroup* const> groups_;
    std::span<hal::IMotor* const> drive_;
    hal::ITelemetrySink* log_;
    DriveProtectionConfig cfg_;
    double tickSeconds_;
    std::optional<FrozenReadingDetector> detectors_[kMaxProtectedGroups];
    std::uint32_t frozen_[kMaxProtectedGroups] = {};
    int overTicks_[kMaxProtectedGroups][kMaxSideMembers] = {};
    double ampsNow_[kMaxProtectedGroups][kMaxSideMembers] = {};
    double peakAmps_[kMaxProtectedGroups][kMaxSideMembers] = {};
    double peakTempC_[kMaxProtectedGroups][kMaxSideMembers] = {};
    Stretch stretch_[kMaxProtectedGroups] = {};
    std::size_t bare_[kMaxBare] = {};
    std::size_t bareCount_ = 0;
    int bareOverTicks_[kMaxBare] = {};
    double bareAmpsNow_[kMaxBare] = {};
    bool refused_ = false;
    int cutTicksLeft_ = 0;
    int cutCount_ = 0;
    int warnings_ = 0;
    DriveProtectionState state_{};
};

}  // namespace shulib::teleop
