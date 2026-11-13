// MECH-A4 — THE DIRECTION CHECK's judgement (mechanism/direction_check.hpp; the brief's §1.11, P7 and
// the re-check's D6): every finding with its sentence VERBATIM, "change it HERE"'s sources, the THREE
// answers (its + way / the other way / it did NOT move or something ELSE moved), a same-kind cable swap
// named at its wire(...) line, two equal motors stalled against each other, the command's sign, and
// the preconditions. Every case names the bug it would catch. The judgement is pure: nothing here
// commands a device (its bench page is MECH-A6's station 13).
//
// The declarations are test/mechanism_fixtures.hpp's: robot x's arm (`.name` on line 38, its
// wire(...) on line 93), its lift (line 51, wire line 94); robot two's claw (line 115, wire line 127);
// robot y's wrist (its rotationSensor(Reversed) on line 134). A captured path is the one the compiler
// opened, so each sentence is built from the captured path and pinned by its SUFFIX and line.

#include "doctest.h"

#include <cmath>
#include <limits>
#include <string>
#include <string_view>

#include "mechanism_fixtures.hpp"
#include "shulib/core/check.hpp"
#include "shulib/mechanism/direction_check.hpp"

namespace mech = shulib::mechanism;
using mech::DirectionFinding;
using mech::DirectionObservation;
using mech::PersonSaw;

namespace {

constexpr double kMoved = 0.05;  // the page's stated "did not move" angle, rad (a test's number)
constexpr double kDraw = 0.3;    // the page's stated "drawing" current, A (a test's number)

/// `file`:`line` as the judgement prints a captured place, after checking the path's suffix.
std::string at(const char* file, std::uint32_t line, std::uint32_t expectLine) {
    REQUIRE(file != nullptr);
    CHECK(std::string_view{file}.ends_with("test/mechanism_fixtures.hpp"));
    CHECK(line == expectLine);
    return std::string{file} + ":" + std::to_string(line);
}

const mech::MechanismSpec kLift = mech_fixture::robot_x::lift.spec();
const mech::MechanismWiring& kLiftW = mech_fixture::robot_x::kRobotXMechanisms.wiring[1];
const mech::MechanismSpec kArm = mech_fixture::robot_x::arm.spec();
const mech::MechanismWiring& kArmW = mech_fixture::robot_x::kRobotXMechanisms.wiring[0];

}  // namespace

TEST_CASE("MECH-A4 P7: the motor findings, each sentence VERBATIM") {
    // Bug caught: a finding that names the wrong place to change — the declaration for a cable, the
    // wiring for a direction, the wrong motor, the wrong word to write — or a sentence that drifted.
    const std::string liftDecl = at(kLift.motors[0].file, kLift.motors[0].line, 51);
    const std::string liftWire = at(kLiftW.file, kLiftW.line, 94);
    const std::string armDecl = at(kArm.motors[0].file, kArm.motors[0].line, 38);
    const std::string armWire = at(kArmW.file, kArmW.line, 93);
    DirectionObservation o{.commandVolts = 4.0, .motorDeltaRad = {1.0, 1.0}, .motorPeakAmps = {0.5, 0.5}};

    // 1. AwaitingPerson: the encoders found nothing wrong and nobody answered yet
    mech::DirectionVerdict v = mech::judgeDirection(kLift, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::AwaitingPerson);
    CHECK(std::string{v.fix}.empty());

    // 2. MotorFights: motor 2 dragged backwards by its mate
    o.motorDeltaRad[1] = -0.4;
    v = mech::judgeDirection(kLift, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::MotorFights);
    CHECK(v.role == 1);
    CHECK(std::string{v.fix} == "'lift' motor 2 turns AGAINST its command (its mate drags it): in the declaration at " +
                                    liftDecl + ", .motors: make motor 2 Forward, then rebuild and upload");

    // 3. DidNotMove: motor 2 neither moved nor drew — its cable, its port in the wire line
    o.motorDeltaRad[1] = 0.0;
    o.motorPeakAmps[1] = 0.0;
    v = mech::judgeDirection(kLift, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::DidNotMove);
    CHECK(v.role == 1);
    CHECK(std::string{v.fix} == "'lift' motor 2 neither moved nor drew current: check its cable, and its port in the "
                                "wire(...) line at " + liftWire);

    // 4. FightOrJam: two equal motors stalled against each other — both still, both drawing
    const DirectionObservation f{.commandVolts = 4.0, .motorDeltaRad = {0.01, -0.01}, .motorPeakAmps = {0.8, 0.8}};
    v = mech::judgeDirection(kLift, kLiftW, f, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::FightOrJam);
    CHECK(std::string{v.fix} == "'lift' did not move but its motors drew current: they FIGHT (one is declared the "
                                "wrong way -- check each Forward/Reversed at " + liftDecl + ") or it is JAMMED / at a stop");
    // ... and one motor still and drawing: a jam, never "a fight"
    const DirectionObservation j{.commandVolts = 4.0, .motorDeltaRad = {0.0}, .motorPeakAmps = {0.9}};
    v = mech::judgeDirection(kArm, kArmW, j, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::FightOrJam);
    CHECK(std::string{v.fix} == "'arm' did not move but its motor drew current: it is JAMMED or at its stop -- clear it "
                                "and jog the other way");

    // 5-7. the person's three answers, on an arm whose encoder agrees with its command
    DirectionObservation a{.commandVolts = 4.0, .motorDeltaRad = {1.0}, .motorPeakAmps = {0.4},
                           .person = PersonSaw::TheOtherWay};
    v = mech::judgeDirection(kArm, kArmW, a, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::MechanismReversed);
    CHECK(std::string{v.fix} == "'arm' moved the OTHER way: in the declaration at " + armDecl +
                                    ", .motors: swap EVERY Forward and Reversed (and the angle sensor's), then rebuild "
                                    "and upload");
    a.person = PersonSaw::ItsPlusWay;
    v = mech::judgeDirection(kArm, kArmW, a, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::Ok);
    CHECK(std::string{v.fix} == "'arm': direction confirmed");
    a.person = PersonSaw::NothingOrSomethingElseMoved;  // (D6) a same-kind cable swap: ANOTHER mechanism moved
    v = mech::judgeDirection(kArm, kArmW, a, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::CableOnAnotherMechanism);
    CHECK(std::string{v.fix} == "'arm' did not move although its motors turned: a cable of 'arm' is on ANOTHER "
                                "mechanism's port (a swap the boot cannot see) -- check the wire(...) line at " +
                                    armWire + " and the wire line of what moved");
}

TEST_CASE("MECH-A4 P7: the ANGLE SENSOR reading against its motors names the sensor's mounting word") {
    // Bug caught: a sensor mounted backwards blamed on the motors (the fix would flip a good motor), or
    // the wrong word to write.
    const mech::MechanismSpec wrist = mech_fixture::robot_y::wrist.spec();  // rotationSensor(Reversed)
    const mech::MechanismWiring& w = mech_fixture::robot_y::kRobotYMechanisms.wiring[0];
    const std::string place = at(wrist.angleSensor.file, wrist.angleSensor.line, 134);
    const DirectionObservation o{.commandVolts = 4.0, .motorDeltaRad = {1.0}, .motorPeakAmps = {0.4},
                                 .hasAngleSensor = true, .sensorDeltaRad = -0.3};
    const mech::DirectionVerdict v = mech::judgeDirection(wrist, w, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::SensorReversed);
    CHECK(std::string{v.fix} == "'wrist' angle sensor reads against its motors: at " + place +
                                    " make it rotationSensor(Forward), then rebuild and upload");
    // a sensor that AGREES is no finding (the person is asked)
    DirectionObservation ok = o;
    ok.sensorDeltaRad = 0.3;
    CHECK(mech::judgeDirection(wrist, w, ok, kMoved, kDraw).finding == DirectionFinding::AwaitingPerson);
}

TEST_CASE("MECH-A4 P7: a GENERATED robot file names its JSON path, and the regenerate step") {
    // Bug caught (G3a's re-check item 3): a generated file's fix pointing at the generated C++ (which
    // the next generation overwrites) instead of the robot file's JSON field.
    using namespace shulib::mechanism::words;
    static constexpr Lift generated{
        .name = "lift",
        .motors = {mech::Motor{.present = true,
                               .changeAt = "src/robots/x.robot.json: mechanisms > \"lift\" > roles > \"motor 1\" > reversed"},
                   mech::Motor{.present = true, .reversed = true,
                               .changeAt = "src/robots/x.robot.json: mechanisms > \"lift\" > roles > \"motor 2\" > reversed"}},
        .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold,
        .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in},
        .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
    const DirectionObservation o{.commandVolts = 4.0, .motorDeltaRad = {1.0, -0.4}, .motorPeakAmps = {0.5, 0.5}};
    const mech::DirectionVerdict v = mech::judgeDirection(generated.spec(), kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::MotorFights);
    CHECK(std::string{v.fix} == "'lift' motor 2 turns AGAINST its command (its mate drags it): at "
                                "src/robots/x.robot.json: mechanisms > \"lift\" > roles > \"motor 2\" > reversed make it "
                                "false, then regenerate, rebuild and upload");
}

TEST_CASE("MECH-A4 P7: changeHere's sources, in order, and wireHere's fallback") {
    // Bug caught: a stated JSON path ignored for the captured line, a missing capture printing
    // "(null):0", or a wiring with no captured line printing garbage.
    char out[256];
    mech::MechanismSpec s = kArm;
    mech::changeHere(out, sizeof out, "a.json: x > y", "f.hpp", 7, s);
    CHECK(std::string{out} == "a.json: x > y");
    mech::changeHere(out, sizeof out, nullptr, "f.hpp", 7, s);
    CHECK(std::string{out} == "f.hpp:7");
    mech::changeHere(out, sizeof out, nullptr, nullptr, 0, s);
    CHECK(std::string{out} == std::string{s.declaredFile} + ":38 (its declaration)");
    s.declaredFile = nullptr;
    mech::changeHere(out, sizeof out, nullptr, nullptr, 0, s);
    CHECK(std::string{out} == "the robot file (its 'arm' declaration)");
    mech::changeHere(out, 8, "a-very-long-json-path", nullptr, 0, s);  // bounded, NUL-terminated
    CHECK(std::string{out} == "a-very-");
    mech::MechanismWiring w{};
    mech::wireHere(out, sizeof out, w, s);
    CHECK(std::string{out} == "the wiring list (its 'arm' line)");
}

TEST_CASE("MECH-A4 P7: the PNEUMATIC answers -- its position, the other one, or nothing / something else") {
    // Bug caught: a claw whose levels are swapped blamed on its cable, or a valve on the wrong line
    // blamed on its levels.
    const mech::MechanismSpec claw = mech_fixture::robot_two::claw.spec(1);
    const mech::MechanismWiring& w = mech_fixture::robot_two::kRobotTwoMechanisms.wiring[2];
    const std::string decl = at(claw.pneumatic.levelsFile, claw.pneumatic.levelsLine, 115);
    const std::string wire = at(w.file, w.line, 127);
    mech::DirectionVerdict v = mech::judgePneumatic(claw, w, 1, 1);
    CHECK(v.finding == DirectionFinding::Ok);
    CHECK(std::string{v.fix} == "'claw': 'open' confirmed");
    v = mech::judgePneumatic(claw, w, 1, 0);
    CHECK(v.finding == DirectionFinding::MechanismReversed);
    CHECK(std::string{v.fix} == "'claw' went 'closed' when told 'open': its line levels are swapped -- in the "
                                "declaration at " + decl + ", .positions: swap each position's levels, then rebuild "
                                "and upload");
    v = mech::judgePneumatic(claw, w, 1, mech::kNothingOrSomethingElseMoved);
    CHECK(v.finding == DirectionFinding::CableOnAnotherMechanism);
    CHECK(std::string{v.fix} == "'claw' did not move when told 'open': its valve is not on the line the wiring says "
                                "(a 3-wire swap, or an unplugged or empty valve -- the boot cannot see either): check "
                                "the wire(...) line at " + wire + ", the tubing and the air");
}

TEST_CASE("MECH-A4 P7 (M25): the command's SIGN -- a jog the other way is judged the other way") {
    // Bug caught: a judgement that assumes every jog is +: a −4 V jog whose motors both turned −
    // would be called a fight, and a real fight on a − jog missed.
    DirectionObservation o{.commandVolts = -4.0, .motorDeltaRad = {-1.0, -1.0}, .motorPeakAmps = {0.5, 0.5}};
    CHECK(mech::judgeDirection(kLift, kLiftW, o, kMoved, kDraw).finding == DirectionFinding::AwaitingPerson);
    o.motorDeltaRad[0] = 0.6;  // motor 1 dragged + while commanded −
    const mech::DirectionVerdict v = mech::judgeDirection(kLift, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::MotorFights);
    CHECK(v.role == 0);
    CHECK(std::string_view{v.fix}.find("make motor 1 Reversed") != std::string_view::npos);
}

TEST_CASE("MECH-A4 P7: encoder NOISE below the page's move threshold is not a fight") {
    // Bug caught: a still motor whose encoder ticked one count backwards called "turns AGAINST its
    // command" — the fix would flip a good motor.
    const DirectionObservation o{.commandVolts = 4.0, .motorDeltaRad = {1.0, -0.01}, .motorPeakAmps = {0.5, 0.5}};
    CHECK(mech::judgeDirection(kLift, kLiftW, o, kMoved, kDraw).finding == DirectionFinding::AwaitingPerson);
}

TEST_CASE("MECH-A4 P7: the page's thresholds are EDGES -- exactly the move angle is a move, exactly the "
          "draw current is drawing") {
    // Bug caught: an off-by-equality in either threshold — a motor that turned exactly the stated angle
    // called "still", or one drawing exactly the stated current called "not drawing" (its cable blamed
    // for a jam).
    const DirectionObservation moved{.commandVolts = 4.0, .motorDeltaRad = {kMoved}, .motorPeakAmps = {0.0}};
    CHECK(mech::judgeDirection(kArm, kArmW, moved, kMoved, kDraw).finding == DirectionFinding::AwaitingPerson);
    // ... and exactly the move angle WHILE drawing is a move too — never "still and drawing" (a jam)
    const DirectionObservation movedDrawing{.commandVolts = 4.0, .motorDeltaRad = {kMoved}, .motorPeakAmps = {kDraw}};
    CHECK(mech::judgeDirection(kArm, kArmW, movedDrawing, kMoved, kDraw).finding == DirectionFinding::AwaitingPerson);
    const DirectionObservation drawing{.commandVolts = 4.0, .motorDeltaRad = {0.0}, .motorPeakAmps = {kDraw}};
    CHECK(mech::judgeDirection(kArm, kArmW, drawing, kMoved, kDraw).finding == DirectionFinding::FightOrJam);
    const DirectionObservation against{.commandVolts = 4.0, .motorDeltaRad = {1.0, -kMoved}, .motorPeakAmps = {0.5, 0.5}};
    CHECK(mech::judgeDirection(kLift, kLiftW, against, kMoved, kDraw).finding == DirectionFinding::MotorFights);
}

TEST_CASE("MECH-A4 P7: the preconditions -- a motor kind, a real command, the page's thresholds") {
    // Bug caught: a 0 V jog judged (every motor "did not move"), a NaN threshold that makes every
    // comparison false, a pneumatic judged as a motor (motor role 0 of a spec with none).
    const DirectionObservation ok{.commandVolts = 4.0, .motorDeltaRad = {1.0}, .motorPeakAmps = {0.4}};
    DirectionObservation zero = ok;
    zero.commandVolts = 0.0;
    CHECK_THROWS_AS((void)mech::judgeDirection(kArm, kArmW, zero, kMoved, kDraw), shulib::PreconditionError);
    DirectionObservation nan = ok;
    nan.commandVolts = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)mech::judgeDirection(kArm, kArmW, nan, kMoved, kDraw), shulib::PreconditionError);
    CHECK_THROWS_AS((void)mech::judgeDirection(kArm, kArmW, ok, 0.0, kDraw), shulib::PreconditionError);
    CHECK_THROWS_AS((void)mech::judgeDirection(kArm, kArmW, ok, kMoved, std::numeric_limits<double>::quiet_NaN()),
                    shulib::PreconditionError);
    const mech::MechanismSpec claw = mech_fixture::robot_two::claw.spec(1);
    const mech::MechanismWiring& cw = mech_fixture::robot_two::kRobotTwoMechanisms.wiring[2];
    CHECK_THROWS_AS((void)mech::judgeDirection(claw, cw, ok, kMoved, kDraw), shulib::PreconditionError);
    CHECK_THROWS_AS((void)mech::judgePneumatic(kArm, kArmW, 0, 0), shulib::PreconditionError);
    CHECK_THROWS_AS((void)mech::judgePneumatic(claw, cw, 2, 0), shulib::PreconditionError);
    CHECK_THROWS_AS((void)mech::judgePneumatic(claw, cw, 0, 5), shulib::PreconditionError);
}

// ═══ MECH-A4's review round (2026-10-27) ═══════════════════════════════════════════════════════

TEST_CASE("MECH-A4 (review): a BACK-DRIVEN mechanism -- every motor that moved went AGAINST its command -- "
          "names NO declaration line") {
    // Bug caught: a one-motor arm that SAGGED under a gentle jog (gravity beat 3 V) told "make motor 1
    // Reversed" — a V5 motor reverses its command and encoder together (landmine 17), so no
    // declaration can make every moving motor read against its command; following that fix would
    // invert a working mechanism. A fight needs a mate moving WITH its own command.
    const std::string sentence =
        " was turned AGAINST its command by something stronger than the jog (gravity or a load): every motor that "
        "moved went the same wrong way, so no declaration is wrong -- jog it harder, or from another pose, and check "
        "again";
    const DirectionObservation one{.commandVolts = 3.0, .motorDeltaRad = {-0.4}, .motorPeakAmps = {1.0}};
    mech::DirectionVerdict v = mech::judgeDirection(kArm, kArmW, one, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::BackDriven);
    CHECK(v.role == 0);
    CHECK(std::string{v.fix} == "'arm'" + sentence);
    const DirectionObservation both{.commandVolts = 3.0, .motorDeltaRad = {-0.4, -0.4}, .motorPeakAmps = {1.0, 1.0}};
    v = mech::judgeDirection(kLift, kLiftW, both, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::BackDriven);
    CHECK(std::string{v.fix} == "'lift'" + sentence);
    // the sign: a DOWN jog that a load drove UP is back-driven too
    const DirectionObservation down{.commandVolts = -3.0, .motorDeltaRad = {0.4, 0.4}, .motorPeakAmps = {1.0, 1.0}};
    CHECK(mech::judgeDirection(kLift, kLiftW, down, kMoved, kDraw).finding == DirectionFinding::BackDriven);
    // one moved against, its mate exactly AT the move angle WITH the command: a fight (the mate drags it)
    const DirectionObservation fight{.commandVolts = 3.0, .motorDeltaRad = {kMoved, -0.4}, .motorPeakAmps = {1.0, 1.0}};
    CHECK(mech::judgeDirection(kLift, kLiftW, fight, kMoved, kDraw).finding == DirectionFinding::MotorFights);
    // one moved against, its mate below the move angle (noise) WITH the command: no mate moved — back-driven
    const DirectionObservation noisyMate{.commandVolts = 3.0, .motorDeltaRad = {0.01, -0.4},
                                         .motorPeakAmps = {1.0, 1.0}};
    CHECK(mech::judgeDirection(kLift, kLiftW, noisyMate, kMoved, kDraw).finding == DirectionFinding::BackDriven);
}

TEST_CASE("MECH-A4 (review): a GENERATED robot file -- EVERY finding that names a role says regenerate, with "
          "the JSON path and the JSON value") {
    // Bug caught (G3a's re-check item 3): a generated file's sensor or mechanism-wide fix printed in
    // C++ words ("rotationSensor(...)", ".motors: swap ...") at a JSON path, with no regenerate step —
    // a team would edit generated C++ that the next generation overwrites.
    using namespace shulib::mechanism::words;
    static constexpr const char* kM1 = "src/robots/x.robot.json: mechanisms > \"lift\" > roles > \"motor 1\" > reversed";
    static constexpr const char* kM2 = "src/robots/x.robot.json: mechanisms > \"lift\" > roles > \"motor 2\" > reversed";
    static constexpr Lift generated{
        .name = "lift",
        .motors = {mech::Motor{.present = true, .changeAt = kM1},
                   mech::Motor{.present = true, .reversed = true, .changeAt = kM2}},
        .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold,
        .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in},
        .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in)};
    const mech::MechanismSpec g = generated.spec();
    // motor 1 (FORWARD, reversed = false) dragged by its mate: "make it true"
    DirectionObservation o{.commandVolts = 4.0, .motorDeltaRad = {-0.4, 1.0}, .motorPeakAmps = {0.5, 0.5}};
    mech::DirectionVerdict v = mech::judgeDirection(g, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::MotorFights);
    CHECK(std::string{v.fix} == std::string{"'lift' motor 1 turns AGAINST its command (its mate drags it): at "} + kM1 +
                                    " make it true, then regenerate, rebuild and upload");
    // the person: the OTHER way
    o = DirectionObservation{.commandVolts = 4.0, .motorDeltaRad = {1.0, 1.0}, .motorPeakAmps = {0.5, 0.5},
                             .person = PersonSaw::TheOtherWay};
    v = mech::judgeDirection(g, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::MechanismReversed);
    CHECK(std::string{v.fix} == std::string{"'lift' moved the OTHER way: in the generated robot file, flip EVERY motor "
                                            "role's reversed (and the angle sensor's) -- motor 1's is at "} +
                                    kM1 + " -- then regenerate, rebuild and upload");
    // two motors still, both drawing
    o = DirectionObservation{.commandVolts = 4.0, .motorDeltaRad = {0.01, -0.01}, .motorPeakAmps = {0.8, 0.8}};
    v = mech::judgeDirection(g, kLiftW, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::FightOrJam);
    CHECK(std::string{v.fix} == std::string{"'lift' did not move but its motors drew current: they FIGHT (one is "
                                            "declared the wrong way -- check each motor role's reversed in the "
                                            "generated robot file, motor 1's at "} +
                                    kM1 + ") or it is JAMMED / at a stop");
    // the angle sensor
    static constexpr const char* kS = "src/robots/x.robot.json: mechanisms > \"wrist\" > roles > \"angle sensor\" > reversed";
    static constexpr Arm wrist{.name = "wrist", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                               .angleSensor = mech::Sensor{.kind = mech::SensorKind::Rotation, .changeAt = kS},
                               .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                               .range = {-90, 90}, .gravity = noGravity, .home = absoluteSensor(0, 180)};
    const DirectionObservation s{.commandVolts = 4.0, .motorDeltaRad = {1.0}, .motorPeakAmps = {0.4},
                                 .hasAngleSensor = true, .sensorDeltaRad = -0.3};
    v = mech::judgeDirection(wrist.spec(), kArmW, s, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::SensorReversed);
    CHECK(std::string{v.fix} == std::string{"'wrist' angle sensor reads against its motors: at "} + kS +
                                    " make it true, then regenerate, rebuild and upload");
    // a pneumatic's levels
    static constexpr const char* kP = "src/robots/two.robot.json: mechanisms > \"claw\" > positions";
    static constexpr Pneumatic claw{.name = "claw", .positions = {{"closed", {Low}, grabs}, {"open", {High}}},
                                    .start = "closed", .safe = "closed", .moveTime = 0.5_s, .changeAt = kP};
    const mech::MechanismWiring& cw = mech_fixture::robot_two::kRobotTwoMechanisms.wiring[2];
    v = mech::judgePneumatic(claw.spec(1), cw, 1, 0);
    CHECK(v.finding == DirectionFinding::MechanismReversed);
    CHECK(std::string{v.fix} == std::string{"'claw' went 'closed' when told 'open': its line levels are swapped -- at "} +
                                    kP + " swap each position's levels, then regenerate, rebuild and upload");
}

TEST_CASE("MECH-A4 (review): the hand-written words each way -- a FORWARD sensor's fix is Reversed; a "
          "pneumatic told its FIRST position; a still motor drawing EXACTLY the draw current is drawing") {
    // Bug caught: only one direction of each word tested (a Forward-mounted sensor's fix that says
    // Forward; a pneumatic judged Ok whenever it was told position 0), and the draw threshold's
    // EQUALITY edge in the "neither moved nor drew" rule (its cable blamed for a motor that drew).
    const mech::MechanismSpec wristZ = mech_fixture::robot_z::wrist.spec();  // rotationSensor() — Forward
    const mech::MechanismWiring& wz = mech_fixture::robot_z::kRobotZMechanisms.wiring[5];
    const std::string place = at(wristZ.angleSensor.file, wristZ.angleSensor.line, 163);
    const DirectionObservation o{.commandVolts = 4.0, .motorDeltaRad = {1.0}, .motorPeakAmps = {0.4},
                                 .hasAngleSensor = true, .sensorDeltaRad = -0.3};
    mech::DirectionVerdict v = mech::judgeDirection(wristZ, wz, o, kMoved, kDraw);
    CHECK(v.finding == DirectionFinding::SensorReversed);
    CHECK(std::string{v.fix} == "'wrist' angle sensor reads against its motors: at " + place +
                                    " make it rotationSensor(Reversed), then rebuild and upload");
    const mech::MechanismSpec claw = mech_fixture::robot_two::claw.spec(1);
    const mech::MechanismWiring& cw = mech_fixture::robot_two::kRobotTwoMechanisms.wiring[2];
    v = mech::judgePneumatic(claw, cw, 0, 1);
    CHECK(v.finding == DirectionFinding::MechanismReversed);
    CHECK(std::string_view{v.fix}.starts_with("'claw' went 'open' when told 'closed': its line levels are swapped"));
    CHECK(mech::judgePneumatic(claw, cw, 0, 0).finding == DirectionFinding::Ok);
    // motor 1 moved; motor 2 still and drawing EXACTLY kDraw: it drew — not "neither moved nor drew"
    const DirectionObservation edge{.commandVolts = 4.0, .motorDeltaRad = {1.0, 0.0}, .motorPeakAmps = {0.5, kDraw}};
    CHECK(mech::judgeDirection(kLift, kLiftW, edge, kMoved, kDraw).finding == DirectionFinding::AwaitingPerson);
    DirectionObservation below = edge;
    below.motorPeakAmps[1] = kDraw / 2.0;
    CHECK(mech::judgeDirection(kLift, kLiftW, below, kMoved, kDraw).finding == DirectionFinding::DidNotMove);
}
