// API REFERENCE FIDELITY PIN (chunk D3).
//
// Bug this file catches: the generated reference in docs/api/ rendering a
// signature that is NOT what the header says. A generator that silently drops
// a `const`, a `noexcept`, a default argument, or one member of an overload
// set is worse than no generator: readers trust generated output *more*
// precisely because nobody typed it, so a quiet omission is believed.
//
// HOW THIS PAIRS WITH THE OTHER TWO MECHANISMS — the three cover different
// failures and none of them subsumes another:
//
//   * test/f6_signature_pin_test.cpp asserts, at COMPILE time, that the
//     HEADER still has these exact signatures. It knows nothing about docs.
//   * tools/api_doc_tool.py check-fresh (run by the build) asserts that
//     docs/api/ is byte-identical to a fresh generation. It knows nothing
//     about whether the generation is CORRECT — a generator that dropped
//     every `const` would still be perfectly "fresh".
//   * THIS FILE closes that gap: it reads the committed markdown and asserts
//     the rendered text for a chosen set of hard cases. Combined with the F6
//     pin ("the header really is this") it makes the rendered line true of
//     the code, not merely stable.
//
// The members pinned below are chosen as the shapes a naive extractor gets
// wrong: a const-qualified overload beside its non-const twin, a noexcept
// accessor, a default argument, a template member, an initializer on a field,
// a brace-list overload, and a deleted special member.
//
// If this file fails: DO NOT edit docs/api/ (it is generated and will be
// overwritten). Either the header changed — regenerate — or the generator is
// lying, which is the bug.

#include "doctest.h"

#include <fstream>
#include <sstream>
#include <string>

#ifndef SHULIB_SOURCE_DIR
#error "SHULIB_SOURCE_DIR must be defined by the build (test/CMakeLists.txt)"
#endif

namespace {

std::string readDoc(const std::string& relative) {
    const std::string path = std::string{SHULIB_SOURCE_DIR} + "/" + relative;
    std::ifstream in{path};
    // A fidelity test that SKIPS when it cannot find its input is decoration:
    // it would go quietly green in exactly the situation where the reference
    // is missing. So a missing file is a failure, loudly.
    REQUIRE_MESSAGE(in.good(), "cannot open generated reference: " << path);
    std::ostringstream buf;
    buf << in.rdbuf();
    return buf.str();
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

/// The rendered signature must appear inside a fenced code block, so a match
/// against prose that happens to quote the text does not count.
bool rendersSignature(const std::string& page, const std::string& signature) {
    const std::string fenced = "```cpp\n" + signature + "\n```";
    return contains(page, fenced);
}

}  // namespace

// Bug caught: the chassis reference dropping a qualifier, a default argument,
// or an overload. Every string below is the EXACT header spelling; the F6 pin
// holds the header to those same shapes, so this pair cannot drift apart
// without one of the two going red.
TEST_CASE("D3 doc fidelity: docs/api/chassis.md renders the header's real signatures") {
    const std::string page = readDoc("docs/api/chassis.md");

    // A default argument, and a const reference parameter.
    CHECK(rendersSignature(
        page,
        "control::ExitReason moveTo(const math::Pose2d& target, "
        "const MotionOptions& options = {})"));

    // Typed time (D2's retype) — the rendered text must say units::Time, not
    // double, or the reference teaches back the bug the retype removed.
    CHECK(rendersSignature(
        page, "control::ExitReason hold(units::Time duration, "
              "const MotionOptions& options = {})"));
    CHECK(rendersSignature(page, "void wait(units::Time duration)"));

    // Both followTrajectory overloads: span AND the brace-list convenience.
    CHECK(rendersSignature(
        page, "TrajectoryResult followTrajectory(std::span<const math::Pose2d> waypoints, "
              "const MotionOptions& options = {})"));
    CHECK(rendersSignature(
        page,
        "TrajectoryResult followTrajectory(std::initializer_list<math::Pose2d> waypoints, "
        "const MotionOptions& options = {})"));

    // The Frame parameter with NO default — the frozen semantic (F6 pin 36).
    CHECK(rendersSignature(page, "void drive(const math::ChassisSpeeds& speeds, "
                                 "math::Frame frame)"));

    // noexcept must survive. D2's campaign hole #1 was a pin that could not see
    // noexcept being dropped; a reference that silently drops it is the same
    // class of lie, one layer out.
    CHECK(rendersSignature(
        page, "[[nodiscard]] control::ExitReason lastExitReason() const noexcept"));
    CHECK(rendersSignature(
        page, "[[nodiscard]] const motion::MotionDeps& deps() const noexcept"));

    // BOTH scheduler overloads, distinctly — an extractor that keys members by
    // name alone silently renders one and hides the other.
    CHECK(rendersSignature(
        page, "[[nodiscard]] motion::MotionScheduler& scheduler() noexcept"));
    CHECK(rendersSignature(
        page,
        "[[nodiscard]] const motion::MotionScheduler& scheduler() const noexcept"));

    // A template member, rendered with its template head.
    CHECK(rendersSignature(
        page, "template <typename Pred> [[nodiscard]] motion::WaitResult "
              "waitUntil(Pred&& pred, units::Time timeout)"));

    // A field's brace initializer is part of the fact — dropping it would make
    // the reference silent about the "0 means use the config default" contract.
    CHECK(rendersSignature(page, "units::Time timeout{0.0}"));
    CHECK(rendersSignature(page, "units::Velocity maxLinearSpeed{0.0}"));

    // A deleted special member is API too: "you cannot copy this".
    CHECK(rendersSignature(page, "Chassis(const Chassis&) = delete"));

    // The prose the header carries must arrive with the signature, not be
    // dropped on the floor.
    CHECK(contains(page, "the decoupled holonomic engine"));
    // …including the header's design commentary, which is the "why".
    CHECK(contains(page, "STATUS: FROZEN"));
}

// Bug caught: the routine reference losing the recipe layer's own shapes —
// notably the template steps and the defaulted step names, which are what a
// reader copies.
TEST_CASE("D3 doc fidelity: docs/api/routine.md renders the header's real signatures") {
    const std::string page = readDoc("docs/api/routine.md");

    CHECK(rendersSignature(
        page, "explicit Routine(Chassis& chassis, const char* name = \"routine\") noexcept"));
    CHECK(rendersSignature(
        page, "Routine& moveTo(const math::Pose2d& target, "
              "const MotionOptions& options = {})"));
    CHECK(rendersSignature(
        page, "Routine& driveTo(units::Length x, units::Length y, "
              "const MotionOptions& options = {})"));
    CHECK(rendersSignature(page, "Routine& pause(units::Time duration)"));

    // Template steps, with their default name arguments intact.
    CHECK(rendersSignature(
        page, "template <typename Pred> Routine& waitFor(Pred&& pred, units::Time timeout, "
              "const char* name = \"waitFor\")"));
    CHECK(rendersSignature(
        page,
        "template <typename Action> Routine& then(Action&& action, "
        "const char* name = \"action\")"));

    // noexcept observers.
    CHECK(rendersSignature(page, "[[nodiscard]] bool ok() const noexcept"));
    CHECK(rendersSignature(
        page, "[[nodiscard]] const TrajectoryResult& lastTrajectory() const noexcept"));

    // The enum and its enumerators are members too.
    CHECK(contains(page, "`enum class RoutineStopCause`"));
    CHECK(contains(page, "WaitTimedOut"));
    CHECK(contains(page, "ActionFailed"));
}

// Bug caught: the index losing a member, which is how a reference stops being
// a reference. The index is generated from the same parse as the pages, so a
// member missing here is a member missing everywhere.
TEST_CASE("D3 doc fidelity: the index lists every member of both surfaces") {
    const std::string index = readDoc("docs/api/README.md");

    for (const char* name : {"Chassis::moveTo", "Chassis::strafeTo", "Chassis::turnTo",
                             "Chassis::brake", "Chassis::hold", "Chassis::wait",
                             "Chassis::drive", "Chassis::cancel", "Chassis::waitUntil",
                             "Chassis::pose", "Chassis::setPose",
                             "Chassis::strafeAuthority", "Chassis::lastExitReason",
                             "Chassis::lastCompleted", "Chassis::motionConfig",
                             "Chassis::deps", "Routine::startAt", "Routine::driveTo",
                             "Routine::face", "Routine::followTrajectory",
                             "Routine::pause", "Routine::waitFor", "Routine::then",
                             "Routine::ok", "Routine::result", "Routine::lastTrajectory",
                             "Routine::chassis", "MotionOptions::timeout",
                             "TrajectoryResult::succeeded", "RoutineResult::cause"}) {
        CHECK_MESSAGE(contains(index, std::string{"`"} + name + "`"),
                      "missing from the generated index: " << name);
    }

    // Both overloads reach the index with distinguishable labels; an index that
    // silently merges them undercounts the surface it claims to enumerate.
    CHECK(contains(index, "`Chassis::scheduler`"));
    CHECK(contains(index, "`Chassis::scheduler (overload 2)`"));
}
