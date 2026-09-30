#pragma once
//
// auton_abort_watch.hpp -- the Auton Tester's ABORT (COMP-t): any controller button, a touch of the
// brain's screen, the controller dropping, or the robot being DISABLED ends the countdown or the
// run at once, through the run guard's abandon latch (sequence/run_guard.hpp, ABANDON).
//
// THE LAYERING (the COMP brief's B7, the one COMP-b's competition watch uses too):
//     Chassis --pace()--> RunGuard --inner--> AbortWatch --inner--> the provider's 10 ms pacer
// The guard checks its deadlines and its abandon latch FIRST, then calls this watch, which reads
// the inputs and -- once an abort is latched and a run is live -- calls guard.requestAbandon(),
// then lets the world advance. So the guard cuts the motion at the NEXT pace, before the world
// advances: at most one more commanded tick (10 ms) after the watch saw the abort.
//
// WHY IT ASKS EVERY TICK, NOT ONCE: a request made while no run is live is DROPPED when the next
// run arms (run_guard.hpp's ABANDON banner: "a program's watch requests on the competition STATE,
// every tick a run is live, not once on its edge"). An abort latched in the countdown's last
// tick, or between the countdown and the guard arming, must still end the run -- so the watch
// requests on its LATCHED cause, every pace while the guard is running.
//
// WHAT ABORTS (the team lead's words, 2026-09-27: "any button aborts"; D5's edges, as
// recommended and accepted): any of the controller's twelve BUTTONS (a level: a held button aborts
// at once) — never a stick, so a thumb resting on one cannot end a run; a TOUCH of the screen,
// counted when the finger LANDS (PROS's press_count, A4 register HA-178 (a)) after the read whose
// release touched RUN (the tester arms with that read's count: arm(pressBaseline)); the controller
// DISCONNECTING (with no controller the abort has no hands — HA-178 (d)); VEXos reporting the robot
// DISABLED. The first cause seen is latched; a second one never replaces it.
//
// PROS-free: the program hands PROS's raw reads to testerTouchFrom() / abortInputsFrom() below
// (host-tested on a PROS-shaped struct, test/auton_abort_watch_test.cpp); the host suite also
// scripts AbortInputs directly. The program's two reads are one line each, pinned by
// test/auton_tester_source_test.cpp. NOTHING HERE HAS RUN ON A ROBOT (worksheet Station M).
//
// NOT for the competition program: in a match a driver's bumped button must not end the auton, so
// COMP-b's watch has the competition causes only (it may share the layering and the
// latch-every-tick rule, never this class).

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "shulib/hal/controller.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

/// Why a countdown or a run was aborted: the FIRST cause the watch saw, latched until re-armed.
enum class AbortCause : std::uint8_t {
    None = 0,            ///< nothing has aborted
    Button = 1,          ///< a controller button was down
    Touch = 2,           ///< the brain's screen was pressed
    ControllerLost = 3,  ///< the controller stopped reporting connected: the abort had no hands
    Disabled = 4,        ///< VEXos reported the robot DISABLED (a field, a switch, a Timed Run)
};

/// The cause as a word for the screen and the log.
[[nodiscard]] constexpr const char* abortCauseWord(AbortCause c) noexcept {
    switch (c) {
        case AbortCause::None: return "none";
        case AbortCause::Button: return "a controller button";
        case AbortCause::Touch: return "a touch of the screen";
        case AbortCause::ControllerLost: return "the controller disconnected";
        case AbortCause::Disabled: return "the robot was disabled";
    }
    return "unknown";
}

/// The cause in at most 15 characters, for the brain's SMALL lines.
[[nodiscard]] constexpr const char* abortCauseShort(AbortCause c) noexcept {
    switch (c) {
        case AbortCause::None: return "none";
        case AbortCause::Button: return "button";
        case AbortCause::Touch: return "touch";
        case AbortCause::ControllerLost: return "controller lost";
        case AbortCause::Disabled: return "robot disabled";
    }
    return "unknown";
}

/// One read of everything that can abort -- the program fills it from PROS every tick.
struct AbortInputs {
    /// IController::isConnected().
    bool controllerConnected = false;
    /// Bit i set = hal::ControllerButton value i is down (buttonsDownOf()).
    std::uint16_t buttonsDown = 0;
    /// The touch read failed (PROS E_TOUCH_ERROR): its counts mean nothing and are ignored.
    bool touchError = false;
    /// PROS's press_count: one more for every finger that lands on the screen (A4: HA-178).
    std::int32_t pressCount = 0;
    /// competition_is_disabled().
    bool disabled = false;
};

/// The twelve buttons' levels as a mask, bit = the enumerator's value (L1 = bit 0 ... A = bit 11).
[[nodiscard]] inline std::uint16_t buttonsDownOf(const hal::IController& c) {
    std::uint16_t mask = 0;
    for (int b = 0; b <= static_cast<int>(hal::ControllerButton::A); ++b) {
        if (c.pressed(static_cast<hal::ControllerButton>(b))) {
            mask = static_cast<std::uint16_t>(mask | (1u << b));
        }
    }
    return mask;
}

/// One read of the brain's touch screen, reduced: the pages' release-edge sample, the press count
/// (the abort's touch) and whether a finger is on the glass right now.
struct TesterTouch {
    ui::TouchSample sample{};     ///< error, releaseCount, x, y -- the pages' release edge
    std::int32_t pressCount = 0;  ///< PROS's press_count: +1 when a finger LANDS (A4: HA-178 (a))
    bool fingerDown = false;      ///< a good read whose press_count is above its release_count
};

/// PROS's screen_touch_status_s_t, reduced (the one place its five fields are read). A template so
/// the host suite runs it on a struct with PROS's field names and the program on the real one;
/// `touchErrorStatus` is pros::E_TOUCH_ERROR (an error read: every field -1, nothing counts).
template <class ProsTouch>
[[nodiscard]] constexpr TesterTouch testerTouchFrom(const ProsTouch& t,
                                                    int touchErrorStatus) noexcept {
    TesterTouch r{};
    r.sample.error = static_cast<int>(t.touch_status) == touchErrorStatus;
    r.sample.releaseCount = t.release_count;
    r.sample.x = t.x;
    r.sample.y = t.y;
    r.pressCount = t.press_count;
    r.fingerDown = !r.sample.error && t.press_count > t.release_count;
    return r;
}

/// The abort's inputs from one touch read, the master controller and VEXos's disabled flag.
[[nodiscard]] inline AbortInputs abortInputsFrom(const TesterTouch& touch,
                                                 const hal::IController& master, bool disabled) {
    AbortInputs in{};
    in.controllerConnected = master.isConnected();
    in.buttonsDown = buttonsDownOf(master);
    in.touchError = touch.sample.error;
    in.pressCount = touch.pressCount;
    in.disabled = disabled;
    return in;
}

/// The abort decision for ONE read, against the press count taken when the watch was armed. PURE.
/// The most serious cause wins a tie: Disabled, then ControllerLost, then Button, then Touch.
[[nodiscard]] constexpr AbortCause abortCauseOf(const AbortInputs& in,
                                                std::int32_t pressBaseline) noexcept {
    if (in.disabled) {
        return AbortCause::Disabled;
    }
    if (!in.controllerConnected) {
        return AbortCause::ControllerLost;
    }
    if (in.buttonsDown != 0) {
        return AbortCause::Button;
    }
    if (!in.touchError && in.pressCount != pressBaseline) {
        return AbortCause::Touch;
    }
    return AbortCause::None;
}

/// Where the watch reads its inputs: PROS on the robot, a script on the host.
class IAbortInputs {
public:
    virtual ~IAbortInputs() = default;
    IAbortInputs() = default;
    IAbortInputs(const IAbortInputs&) = default;
    IAbortInputs(IAbortInputs&&) = default;
    IAbortInputs& operator=(const IAbortInputs&) = default;
    IAbortInputs& operator=(IAbortInputs&&) = default;
    /// This tick's inputs.
    [[nodiscard]] virtual AbortInputs read() = 0;
};

/// THE WATCH (file banner): a pacer between the RunGuard and the real pacer. Disarmed it only
/// paces. Armed, it latches the first abort cause it reads and, while the guard is running a
/// run, requests the abandon every tick. arm()/disarm()/pace() belong to the ONE task that runs
/// the Chassis; cause() and armed() may be read from any task.
class AbortWatch final : public motion::ITickPacer {
public:
    /// `inner` is the real pacer, `inputs` the reader; both must outlive the watch.
    AbortWatch(motion::ITickPacer& inner, IAbortInputs& inputs) noexcept
        : inner_{&inner}, inputs_{&inputs} {}

    /// Pinned: the guard holds it by reference as its inner pacer.
    AbortWatch(const AbortWatch&) = delete;
    AbortWatch& operator=(const AbortWatch&) = delete;
    AbortWatch(AbortWatch&&) = delete;
    AbortWatch& operator=(AbortWatch&&) = delete;
    ~AbortWatch() override = default;

    /// The guard to abandon (the COMP brief's B7: given after the guard is emplaced).
    void attachGuard(sequence::RunGuard& guard) noexcept { guard_ = &guard; }

    /// True when attachGuard() was given `guard` -- the watch-to-guard half of the pacer chain,
    /// which pacing alone cannot show: a watch never attached still paces and still latches, but
    /// its abandon request goes nowhere and the run drives on through the button (measured by
    /// COMP-t's review, 2026-09-29: 14.22 in after B, and a green FINISHED). The boot's chain
    /// proof requires it (pacerChainProven, auton_tester_run.hpp).
    [[nodiscard]] bool attachedTo(const sequence::RunGuard& guard) const noexcept {
        return guard_ == &guard;
    }

    /// Start watching: clear the latch, take the press count NOW as the baseline (a touch that
    /// landed before this call is not an abort), and arm. A FAILED touch read here (PROS reports
    /// -1 in every field) gives no baseline: the first good read supplies it and aborts nothing --
    /// otherwise its real count would differ from -1 and read as a touch (R2a's error-read defect).
    void arm() {
        const AbortInputs now = inputs_->read();
        baselineKnown_ = !now.touchError;
        pressBaseline_ = now.pressCount;
        buttonsAtLatch_.store(0);
        cause_.store(AbortCause::None);
        armed_.store(true);
    }

    /// Start watching with a press count the caller ALREADY read as the baseline: the good read
    /// whose release touched RUN (runTesterOnce). A finger that lands after THAT read is an abort,
    /// however soon it lands. arm() above would take its baseline from its own, later read, and a
    /// finger landing in between -- the RUN line's SD write is in that gap -- would be counted into
    /// the baseline and never abort, even held through the whole countdown (COMP-t's review,
    /// 2026-09-29). Fail-safe: the count only rises, so a baseline read earlier can only make a
    /// touch abort sooner, never later. Otherwise exactly arm().
    void arm(std::int32_t pressBaseline) noexcept {
        baselineKnown_ = true;
        pressBaseline_ = pressBaseline;
        buttonsAtLatch_.store(0);
        cause_.store(AbortCause::None);
        armed_.store(true);
    }

    /// Stop watching. The latched cause stays readable until the next arm().
    void disarm() noexcept { armed_.store(false); }

    /// The latched cause (None until something aborts). Any task.
    [[nodiscard]] AbortCause cause() const noexcept { return cause_.load(); }
    /// The buttons that were down when a Button cause latched (bit = enumerator). Any task.
    [[nodiscard]] std::uint16_t buttonsAtLatch() const noexcept { return buttonsAtLatch_.load(); }
    /// True while armed. Any task.
    [[nodiscard]] bool armed() const noexcept { return armed_.load(); }
    /// Every pace() since construction, armed or not (the pacing task only): the boot's chain
    /// proof reads it (pacerChainProven, auton_tester_run.hpp).
    [[nodiscard]] std::uint64_t paces() const noexcept { return paces_; }

    /// The pacer seam (file banner).
    void pace() override {
        ++paces_;  // every pace, armed or not: the boot's chain proof (R13)
        if (armed_.load()) {
            AbortCause c = cause_.load();
            if (c == AbortCause::None) {
                AbortInputs in = inputs_->read();
                if (!baselineKnown_ && !in.touchError) {
                    pressBaseline_ = in.pressCount;  // the first good read IS the baseline
                    baselineKnown_ = true;
                }
                if (!baselineKnown_) {
                    in.touchError = true;  // no baseline yet: the touch says nothing this tick
                }
                c = abortCauseOf(in, pressBaseline_);
                if (c != AbortCause::None) {
                    buttonsAtLatch_.store(in.buttonsDown);
                    cause_.store(c);
                }
            }
            if (c != AbortCause::None && guard_ != nullptr && guard_->running()) {
                guard_->requestAbandon();  // every tick while latched (file banner)
            }
        }
        inner_->pace();
    }

private:
    motion::ITickPacer* inner_;
    IAbortInputs* inputs_;
    sequence::RunGuard* guard_ = nullptr;
    std::int32_t pressBaseline_ = 0;
    bool baselineKnown_ = false;
    std::uint64_t paces_ = 0;
    std::atomic<bool> armed_{false};
    std::atomic<AbortCause> cause_{AbortCause::None};
    std::atomic<std::uint16_t> buttonsAtLatch_{0};
};

}  // namespace shulib::bench
