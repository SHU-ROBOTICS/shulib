#pragma once
//
// Drivetrain degradation policy — what a drive program does when some of the
// motors it was built for do not answer (chunk R3b Part 0b, 2026-09-10).
//
// WHY THIS EXISTS: the team lead's instruction for the drive program was "some
// ports might die during comp; 1–2 dead ports shouldn't stop driving, just let
// it be a warning that gets logged." A dead port is therefore NOT a fault that
// ends the program (the tester's rule, right for diagnosis) and NOT a silent
// zero (the worst outcome — a side quietly down a motor and nobody told). It is
// a WARNING with a policy behind it, and the policy is this pure function so it
// can be host-tested against every boundary row rather than argued about on
// the field.
//
// THE RULE (brief §3, decided): refuse to drive only if ANY side has fewer than
// three answering motors, or the TOTAL dead exceeds two. Otherwise drive —
// degraded, with the counts shown.
//
// THE REASONING, stated so the numbers are not magic: robot two has five
// coupled motors per side on one gear train, four wheels per side. Two motors
// of five is a side that cannot pull its share — under load it lags, and the
// robot drags into a curve at speed that the driver cannot steer out of; that
// side is also carrying the mechanical drag of three unpowered rotors. Three of
// five drives: down 40 % of the side's torque, straight enough at reduced
// speed, and the driver is told. The total cap of two is the second guard: two
// dead on ONE side is 3/5 (drives); one dead on each is 4/5 + 4/5 (drives);
// three dead anywhere is a drivetrain with a systemic problem — a cable
// harness, a brain port bank, a battery sag — not a port, and driving it is
// the wrong answer even where the per-side floor would pass.
//
// DOMAIN, stated honestly: the floor is written for a five-per-side train and
// the bench bot's four-per-side (3/4 drives, 2/4 refuses). A drivetrain with
// fewer than three motors per side is outside this policy's domain — it would
// be refused at every boot — and that is deliberate: nobody has thought about
// such a robot here, and a policy that silently generalised to it would be an
// invented answer. Widen it, with a reason, when such a robot exists.
//
// PURE and PROS-free: numbers in, a verdict out, no state, no clock. The
// caller (src/drive_program.cpp) counts the motors whose adapter constructed —
// and re-counts when a member goes ABSENT at runtime — and acts on the verdict.

namespace shulib::teleop {

/// The per-side floor: a side with fewer answering motors than this cannot pull its share
/// (header: two of five drags the robot into a curve; three drives).
inline constexpr int kMinAnsweringPerSide = 3;

/// The whole-drivetrain cap: more dead motors than this, anywhere, is a systemic problem
/// (harness, port bank, battery), not a port, and the drivetrain is refused.
inline constexpr int kMaxDeadTotal = 2;

/// One side's census: how many motors the chassis table lists for it, and how many of those
/// actually answered (adapter constructed, and not since marked ABSENT).
struct SideCount {
    int expected = 0;   ///< Motors the chassis table lists on this side.
    int answering = 0;  ///< Of those, the ones that answer right now (0 <= answering <= expected).
};

/// What the drive program is allowed to do with the motors it has.
enum class DriveVerdict {
    Drive,          ///< Every listed motor answers: drive normally.
    DriveDegraded,  ///< One or two dead, every side still at the floor or above: drive, warn, log.
    Refuse,         ///< A side below the floor, the total over the cap, or nonsense counts: 0 V.
};

/// The verdict with the counts behind it, so the panel, the LCD and the log can say WHICH
/// side is down and by how much rather than just "degraded".
struct DegradationVerdict {
    DriveVerdict verdict = DriveVerdict::Refuse;  ///< The decision.
    int leftDead = 0;                             ///< expected − answering on the left.
    int rightDead = 0;                            ///< expected − answering on the right.
    int totalDead = 0;                            ///< leftDead + rightDead.
    const char* reason = "";                      ///< One short phrase for the log; "" when Drive.
};

/// The policy: refuse iff any side answers with fewer than kMinAnsweringPerSide, or the total
/// dead exceeds kMaxDeadTotal; DriveDegraded when anything is dead but neither limit is hit;
/// Drive when nothing is. Counts that make no sense — a negative, or more answering than
/// expected, or an empty side — are REFUSED with a reason, because the safe direction for a
/// nonsense census is 0 V, never "drive anyway".
[[nodiscard]] constexpr DegradationVerdict evaluateDegradation(SideCount left,
                                                               SideCount right) noexcept {
    DegradationVerdict v{};
    const bool sane = left.expected > 0 && right.expected > 0 && left.answering >= 0
                      && right.answering >= 0 && left.answering <= left.expected
                      && right.answering <= right.expected;
    if (!sane) {
        v.reason = "REFUSE: motor counts make no sense (side empty, negative, or more "
                   "answering than listed)";
        return v;
    }
    v.leftDead = left.expected - left.answering;
    v.rightDead = right.expected - right.answering;
    v.totalDead = v.leftDead + v.rightDead;
    if (left.answering < kMinAnsweringPerSide) {
        v.reason = "REFUSE: LEFT side has fewer than 3 answering motors";
        return v;
    }
    if (right.answering < kMinAnsweringPerSide) {
        v.reason = "REFUSE: RIGHT side has fewer than 3 answering motors";
        return v;
    }
    if (v.totalDead > kMaxDeadTotal) {
        v.reason = "REFUSE: more than 2 motors dead in total";
        return v;
    }
    if (v.totalDead == 0) {
        v.verdict = DriveVerdict::Drive;
        v.reason = "";
        return v;
    }
    v.verdict = DriveVerdict::DriveDegraded;
    v.reason = "DEGRADED: driving with dead motor(s) -- see WARNINGS";
    return v;
}

}  // namespace shulib::teleop
