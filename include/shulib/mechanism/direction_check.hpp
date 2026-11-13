#pragma once
//
// mechanism/direction_check.hpp — THE DIRECTION CHECK's judgement (the mechanism core, chunk MECH-A4,
// 2026-10-22; its bench page is MECH-A6's station 13). Pure, host-tested, PROS-free.
//
// ── What it is for ─────────────────────────────────────────────────────────────────────
// The boot names every role's port and sees a cable on the WRONG KIND of device, but two cables of
// the SAME kind swapped between mechanisms, a motor declared the wrong way, or an angle sensor
// mounted backwards all look fine at boot. The direction check jogs ONE mechanism a little (the page
// commands it, under its own gates), records what the encoders saw, and asks the person ONE question
// — "ONLY '<name>' should have moved: did it go its + way, the OTHER way, or did it NOT move (or
// something ELSE moved)?" — and this file turns the two halves into a finding and the exact place to
// change: the declaration's line for a direction, the `wire(...)` line for a cable on the wrong
// mechanism. It REPORTS; it cannot save a flip (the team lead's Q1 (a): the robot file is the only
// place a direction lives — edit it, rebuild, upload).
//
// ── What the encoders can see, and what only a person can ──────────────────────────
// A V5 motor reverses its command AND its encoder together, so a motor's own encoder cannot see that
// motor declared the wrong way (the brief's landmine 17). What the encoders CAN see:
//   * a motor whose encoder moves AGAINST its command WHILE A MATE MOVES WITH ITS OWN — the mate
//     drags it: a FIGHT, and THAT motor's Forward/Reversed is the one to flip;
//   * every motor that moved went AGAINST its command (one motor, or all of them alike) — no mate
//     dragged it: the mechanism was BACK-DRIVEN by something stronger than the jog (gravity, a
//     load). No declaration is wrong, and none is named (the review's finding: "make motor 1
//     Reversed" for a sagging one-motor arm would break a correct declaration);
//   * every motor still while every one draws current — two motors of equals declared against each
//     other stall TOGETHER, or the mechanism is JAMMED / at a stop (never "check motor 1's cable");
//   * a motor that neither moved nor drew — its cable, or its port in the wiring;
//   * an angle sensor reading against the motors — the sensor's mounting is declared backwards.
// What only the person can say: which way the MECHANISM moved, and whether it was THIS mechanism at
// all — a same-kind cable swap moves ANOTHER mechanism, whose encoder agrees with its command (the
// re-check's D6: the THIRD answer names the wiring line, not the declaration).
//
// ── "Change it HERE" ─────────────────────────────────────────────────────────────────
// A role's place, in order: its stated `changeAt` (a GENERATED robot file's JSON path — G3a), else
// the file and line the declaration was written on (captured by the `.name` word — no string typed
// by hand, so it cannot go stale when a line moves), else the declaration's own line. A cable on the
// wrong mechanism is fixed at its `wire(...)` line (captured where motors(...) / lines(...) is
// written). A captured path is the one the compiler opened, so host and brain differ: a test pins
// its SUFFIX.
//
// ── The thresholds are the PAGE's ───────────────────────────────────────────────────
// `minMoveRad` (an encoder change below it is "did not move") and `drawAmps` (a still motor drawing
// at least it is pushing on something) are the bench page's STATED numbers (HA-182's page entries,
// MECH-A6) — never invented here.
//
// Allocation-free; every sentence bounded and NUL-terminated in the verdict's own buffer.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "shulib/core/check.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace shulib::mechanism {

/// What the person answered after the jog — the page's THREE buttons, plus "not asked yet".
enum class PersonSaw : std::uint8_t {
    NotAsked = 0,                 ///< the question has not been answered yet
    ItsPlusWay,                   ///< THIS mechanism moved, the way its "+" should
    TheOtherWay,                  ///< THIS mechanism moved, the OTHER way
    NothingOrSomethingElseMoved,  ///< it did NOT move — or ANOTHER mechanism moved
};

/// What the page recorded over one jog of ONE motor mechanism.
struct DirectionObservation {
    /// The jog's signed command, volts — the robot file's "+" way when positive. Finite, not 0.
    double commandVolts = 0.0;
    /// Each motor role's encoder change over the jog, radians (role order: the declaration's).
    double motorDeltaRad[kMaxMotorRoles]{};
    /// Each motor role's highest current during the jog, amps.
    double motorPeakAmps[kMaxMotorRoles]{};
    /// The mechanism has an angle sensor and the page read it.
    bool hasAngleSensor = false;
    /// The angle sensor's change over the jog, radians (as the declaration's mounting reads it).
    double sensorDeltaRad = 0.0;
    /// The person's answer.
    PersonSaw person = PersonSaw::NotAsked;
};

/// The judgement's finding.
enum class DirectionFinding : std::uint8_t {
    Ok,                       ///< it moved its + way (or a pneumatic went where it was told)
    DidNotMove,               ///< a motor neither moved nor drew current: its cable, or its port
    MotorFights,              ///< a motor turned AGAINST its command while a mate turned WITH its own: the mate drags it
    FightOrJam,               ///< every motor still, every one drawing: they fight, or it is jammed
    SensorReversed,           ///< the angle sensor reads against the motors
    MechanismReversed,        ///< it moved the OTHER way (or a pneumatic went to another position)
    CableOnAnotherMechanism,  ///< it did not move although commanded: its cable or line is elsewhere
    AwaitingPerson,           ///< the encoders found nothing: ask "ONLY '<name>' should move: which way?"
    BackDriven,               ///< every motor that moved went AGAINST its command: gravity or a load beat the jog
};

/// "Change it HERE" (file header): `source` when stated (a generated robot file's JSON path), else
/// `file`:`line` (where the team wrote it), else the declaration's own line, else the robot file by
/// name. Writes at most `cap` bytes, NUL-terminated.
inline void changeHere(char* out, std::size_t cap, const char* source, const char* file, std::uint32_t line,
                       const MechanismSpec& s) {
    if (source != nullptr) {
        std::snprintf(out, cap, "%.200s", source);
    } else if (file != nullptr && line > 0) {
        std::snprintf(out, cap, "%.200s:%u", file, static_cast<unsigned>(line));
    } else if (s.declaredFile != nullptr && s.declaredLine > 0) {
        std::snprintf(out, cap, "%.180s:%u (its declaration)", s.declaredFile, static_cast<unsigned>(s.declaredLine));
    } else {
        std::snprintf(out, cap, "the robot file (its '%.23s' declaration)", s.name);
    }
}
/// The mechanism's WIRING line (its `wire(...)`), where a cable on the wrong port is fixed; the
/// wiring list by name when the line was not captured. Writes at most `cap` bytes, NUL-terminated.
inline void wireHere(char* out, std::size_t cap, const MechanismWiring& w, const MechanismSpec& s) {
    if (w.file != nullptr && w.line > 0) {
        std::snprintf(out, cap, "%.200s:%u", w.file, static_cast<unsigned>(w.line));
    } else {
        std::snprintf(out, cap, "the wiring list (its '%.23s' line)", s.name);
    }
}

/// The judgement: a finding, the motor role it names (where it names one), and the sentence — the
/// robot-file line to change, or what to check.
struct DirectionVerdict {
    /// What was found.
    DirectionFinding finding = DirectionFinding::AwaitingPerson;
    /// The motor role the finding names (0-based; 0 where it names none).
    std::uint8_t role = 0;
    /// The sentence (empty while awaiting the person).
    char fix[384]{};
};

/// THE JUDGEMENT for a MOTOR mechanism (file header), in this order: every motor still and every one
/// drawing → FightOrJam; a motor that neither moved nor drew → DidNotMove (its cable, its wire line);
/// a motor that moved AGAINST the command while another moved WITH it → MotorFights (that role's
/// Forward/Reversed); motors that moved, every one AGAINST the command → BackDriven (no line named:
/// jog harder or from another pose); the angle sensor against the command → SensorReversed; then the
/// person's answer — NotAsked → AwaitingPerson,
/// TheOtherWay → MechanismReversed (swap EVERY Forward and Reversed), NothingOrSomethingElseMoved →
/// CableOnAnotherMechanism (its wire line), ItsPlusWay → Ok. PRECONDITIONS: `s` is a motor kind (one
/// to four motor roles); `o.commandVolts` finite and not 0; `minMoveRad` and `drawAmps` finite and
/// above 0 (the page's stated numbers).
[[nodiscard]] inline DirectionVerdict judgeDirection(const MechanismSpec& s, const MechanismWiring& w,
                                                     const DirectionObservation& o, double minMoveRad,
                                                     double drawAmps) {
    SHULIB_PRECONDITION(s.kind != Kind::Pneumatic && s.motorCount >= 1 && s.motorCount <= kMaxMotorRoles,
                        "judgeDirection: a MOTOR mechanism (a pneumatic is judged by judgePneumatic)");
    SHULIB_PRECONDITION(std::isfinite(o.commandVolts) && o.commandVolts != 0.0,
                        "judgeDirection: the jog's command must be a finite, non-zero number of volts");
    SHULIB_PRECONDITION(std::isfinite(minMoveRad) && minMoveRad > 0.0 && std::isfinite(drawAmps) && drawAmps > 0.0,
                        "judgeDirection: the page's two thresholds must be finite and above 0");
    DirectionVerdict v;
    const double cmd = o.commandVolts > 0.0 ? 1.0 : -1.0;
    char decl[224];
    changeHere(decl, sizeof decl, s.motors[0].source, s.motors[0].file, s.motors[0].line, s);
    bool anyMoved = false;
    bool allStillDrawing = true;
    for (std::uint8_t m = 0; m < s.motorCount; ++m) {
        const bool still = std::abs(o.motorDeltaRad[m]) < minMoveRad;
        anyMoved = anyMoved || !still;
        allStillDrawing = allStillDrawing && still && o.motorPeakAmps[m] >= drawAmps;
    }
    if (!anyMoved && allStillDrawing) {  // every motor still, every one pushing: a fight of equals, or a jam
        v.finding = DirectionFinding::FightOrJam;
        if (s.motorCount > 1 && s.motors[0].source != nullptr) {  // a GENERATED robot file: its JSON paths
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' did not move but its motors drew current: they FIGHT (one is declared the wrong way "
                          "-- check each motor role's reversed in the generated robot file, motor 1's at %.160s) or it "
                          "is JAMMED / at a stop",
                          s.name, decl);
        } else if (s.motorCount > 1) {
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' did not move but its motors drew current: they FIGHT (one is declared the wrong way "
                          "-- check each Forward/Reversed at %.160s) or it is JAMMED / at a stop",
                          s.name, decl);
        } else {
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' did not move but its motor drew current: it is JAMMED or at its stop -- clear it "
                          "and jog the other way",
                          s.name);
        }
        return v;
    }
    for (std::uint8_t m = 0; m < s.motorCount; ++m) {
        if (std::abs(o.motorDeltaRad[m]) < minMoveRad && o.motorPeakAmps[m] < drawAmps) {
            v.finding = DirectionFinding::DidNotMove;
            v.role = m;
            char wl[224];
            wireHere(wl, sizeof wl, w, s);
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' motor %d neither moved nor drew current: check its cable, and its port in the "
                          "wire(...) line at %.160s",
                          s.name, m + 1, wl);
            return v;
        }
    }
    // A motor AGAINST its command is a FIGHT only when a mate moved WITH its own (the mate drags it);
    // when every motor that moved went against it, the mechanism was BACK-DRIVEN (file header).
    bool anyWith = false;
    bool anyAgainst = false;
    for (std::uint8_t m = 0; m < s.motorCount; ++m) {
        if (std::abs(o.motorDeltaRad[m]) >= minMoveRad) {
            anyWith = anyWith || o.motorDeltaRad[m] * cmd > 0.0;
            anyAgainst = anyAgainst || o.motorDeltaRad[m] * cmd < 0.0;
        }
    }
    if (anyAgainst && !anyWith) {
        v.finding = DirectionFinding::BackDriven;
        std::snprintf(v.fix, sizeof v.fix,
                      "'%.23s' was turned AGAINST its command by something stronger than the jog (gravity or a load): "
                      "every motor that moved went the same wrong way, so no declaration is wrong -- jog it harder, "
                      "or from another pose, and check again",
                      s.name);
        return v;
    }
    for (std::uint8_t m = 0; m < s.motorCount; ++m) {
        if (std::abs(o.motorDeltaRad[m]) >= minMoveRad && o.motorDeltaRad[m] * cmd < 0.0) {
            v.finding = DirectionFinding::MotorFights;
            v.role = m;
            char here[224];
            changeHere(here, sizeof here, s.motors[m].source, s.motors[m].file, s.motors[m].line, s);
            if (s.motors[m].source != nullptr) {  // a GENERATED robot file: its JSON path
                std::snprintf(v.fix, sizeof v.fix,
                              "'%.23s' motor %d turns AGAINST its command (its mate drags it): at %.160s make it "
                              "%s, then regenerate, rebuild and upload",
                              s.name, m + 1, here, s.motors[m].reversed ? "false" : "true");
            } else {
                std::snprintf(v.fix, sizeof v.fix,
                              "'%.23s' motor %d turns AGAINST its command (its mate drags it): in the declaration at "
                              "%.160s, .motors: make motor %d %s, then rebuild and upload",
                              s.name, m + 1, here, m + 1, s.motors[m].reversed ? "Forward" : "Reversed");
            }
            return v;
        }
    }
    if (o.hasAngleSensor && std::abs(o.sensorDeltaRad) >= minMoveRad && o.sensorDeltaRad * cmd < 0.0) {
        v.finding = DirectionFinding::SensorReversed;
        char here[224];
        changeHere(here, sizeof here, s.angleSensor.source, s.angleSensor.file, s.angleSensor.line, s);
        if (s.angleSensor.source != nullptr) {  // a GENERATED robot file: its JSON path
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' angle sensor reads against its motors: at %.160s make it %s, then regenerate, "
                          "rebuild and upload",
                          s.name, here, s.angleSensor.reversed ? "false" : "true");
        } else {
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' angle sensor reads against its motors: at %.160s make it rotationSensor(%s), then "
                          "rebuild and upload",
                          s.name, here, s.angleSensor.reversed ? "Forward" : "Reversed");
        }
        return v;
    }
    switch (o.person) {
        case PersonSaw::NotAsked:
            return v;  // AwaitingPerson: "ONLY '<name>' should move: which way did it go?"
        case PersonSaw::NothingOrSomethingElseMoved: {
            v.finding = DirectionFinding::CableOnAnotherMechanism;
            char wl[224];
            wireHere(wl, sizeof wl, w, s);
            std::snprintf(v.fix, sizeof v.fix,
                          "'%.23s' did not move although its motors turned: a cable of '%.23s' is on ANOTHER "
                          "mechanism's port (a swap the boot cannot see) -- check the wire(...) line at %.150s and the "
                          "wire line of what moved",
                          s.name, s.name, wl);
            return v;
        }
        case PersonSaw::TheOtherWay:
            v.finding = DirectionFinding::MechanismReversed;
            if (s.motors[0].source != nullptr) {  // a GENERATED robot file: its JSON paths
                std::snprintf(v.fix, sizeof v.fix,
                              "'%.23s' moved the OTHER way: in the generated robot file, flip EVERY motor role's "
                              "reversed (and the angle sensor's) -- motor 1's is at %.160s -- then regenerate, rebuild "
                              "and upload",
                              s.name, decl);
            } else {
                std::snprintf(v.fix, sizeof v.fix,
                              "'%.23s' moved the OTHER way: in the declaration at %.160s, .motors: swap EVERY Forward "
                              "and Reversed (and the angle sensor's), then rebuild and upload",
                              s.name, decl);
            }
            return v;
        case PersonSaw::ItsPlusWay:
            break;
    }
    v.finding = DirectionFinding::Ok;
    std::snprintf(v.fix, sizeof v.fix, "'%.23s': direction confirmed", s.name);
    return v;
}

/// A pneumatic's answer "it did NOT move, or ANOTHER mechanism moved" (judgePneumatic's `sawPosition`).
inline constexpr std::uint8_t kNothingOrSomethingElseMoved = 0xFF;

/// THE JUDGEMENT for a PNEUMATIC mechanism: the page commanded position `commanded` (after the person
/// confirmed the robot was safe) and the person says which named position it went to — or
/// kNothingOrSomethingElseMoved. The commanded one → Ok; another → MechanismReversed (its line levels
/// are swapped, named where they are stated); nothing or something else → CableOnAnotherMechanism
/// (its valve is not on the line the wiring says, or is unplugged, or has no air — its wire line).
/// PRECONDITIONS: `s` is a pneumatic; `commanded` one of its positions; `sawPosition` one of its
/// positions or kNothingOrSomethingElseMoved.
[[nodiscard]] inline DirectionVerdict judgePneumatic(const MechanismSpec& s, const MechanismWiring& w,
                                                     std::uint8_t commanded, std::uint8_t sawPosition) {
    SHULIB_PRECONDITION(s.kind == Kind::Pneumatic && s.pneumatic.positionCount <= kMaxPneumaticPositions,
                        "judgePneumatic: a PNEUMATIC mechanism (a motor one is judged by judgeDirection)");
    SHULIB_PRECONDITION(commanded < s.pneumatic.positionCount, "judgePneumatic: the commanded position is not one of its");
    SHULIB_PRECONDITION(sawPosition < s.pneumatic.positionCount || sawPosition == kNothingOrSomethingElseMoved,
                        "judgePneumatic: the answer is not one of its positions, nor 'nothing / something else'");
    DirectionVerdict v;
    if (sawPosition == kNothingOrSomethingElseMoved) {
        v.finding = DirectionFinding::CableOnAnotherMechanism;
        char wl[224];
        wireHere(wl, sizeof wl, w, s);
        std::snprintf(v.fix, sizeof v.fix,
                      "'%.23s' did not move when told '%.15s': its valve is not on the line the wiring says (a 3-wire "
                      "swap, or an unplugged or empty valve -- the boot cannot see either): check the wire(...) line "
                      "at %.130s, the tubing and the air",
                      s.name, s.pneumatic.positions[commanded].name, wl);
        return v;
    }
    if (commanded == sawPosition) {
        v.finding = DirectionFinding::Ok;
        std::snprintf(v.fix, sizeof v.fix, "'%.23s': '%.15s' confirmed", s.name, s.pneumatic.positions[commanded].name);
        return v;
    }
    char levelsAt[224];
    changeHere(levelsAt, sizeof levelsAt, s.pneumatic.levelsSource, s.pneumatic.levelsFile, s.pneumatic.levelsLine, s);
    v.finding = DirectionFinding::MechanismReversed;
    if (s.pneumatic.levelsSource != nullptr) {  // a GENERATED robot file: its JSON path
        std::snprintf(v.fix, sizeof v.fix,
                      "'%.23s' went '%.15s' when told '%.15s': its line levels are swapped -- at %.160s swap each "
                      "position's levels, then regenerate, rebuild and upload",
                      s.name, s.pneumatic.positions[sawPosition].name, s.pneumatic.positions[commanded].name, levelsAt);
    } else {
        std::snprintf(v.fix, sizeof v.fix,
                      "'%.23s' went '%.15s' when told '%.15s': its line levels are swapped -- in the declaration at "
                      "%.160s, .positions: swap each position's levels, then rebuild and upload",
                      s.name, s.pneumatic.positions[sawPosition].name, s.pneumatic.positions[commanded].name, levelsAt);
    }
    return v;
}

}  // namespace shulib::mechanism
