#pragma once
//
// CameraFeed — ONE read of the camera per period, handed to every consumer (chunk VIS1-A2, 2026-11-03).
//
// ── Who reads the camera, and how often ─────────────────────────────────────────────
// A read is one device call for the count and one per detection (up to 25), plus the adapter's own
// checks (hal::IVisionFrames::read()), and the library is single-task by contract: every device here is
// read from the caller's own loop, never from a task of the library's. So the program's loop — the
// driver loop's per-tick hook, an auton step's wait — calls tick() once per ITS tick, and the feed reads
// the camera only when at least `minReadPeriodSeconds` has passed since its last read BEGAN (never twice
// in one call). Every consumer — the Toggle tracker, the piece finder, and later the automatic flip and
// the drive to a piece — is handed the SAME frame through latest(): two consumers never cost two reads,
// and none reads faster than the camera changes (VEX states about 30 frames a second — A4 register
// HA-192 — so about 0.033 s is the natural floor; the period is the TEAM's number, and an UNSET one is
// refused: no library default).
//
// ── latest() is the latest READ — never the last good one ───────────────────────────
// A fault frame included: a frame held over a failed read would read as a fresh "nothing changed"
// (hal/vision_frames.hpp). Only `FrameStatus::Ok` is evidence, and each consumer checks it.
//
// ── Every read is timed ─────────────────────────────────────────────────────────────
// With the clock before and after it (on the robot that clock is pros::micros()-backed —
// hal/pros/clock.hpp), so stats() carries the last and the slowest read's cost: the measurement
// A4 register HA-203 (one read is cheap enough for the driver loop's tick) needs, from the loop the
// read actually runs in.
//
// No task, no allocation, PROS-free, sim-free; single-task, like every device here. NO HARDWARE CLAIM:
// it has run against the host's fake camera and nothing else.

#include <cstdint>

#include "shulib/config/camera_config.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/vision_frames.hpp"

namespace shulib::vision {

/// The feed's one number — the TEAM's. 0 = UNSET, and refused (no library default).
struct CameraFeedConfig {
    /// Read at most this often, seconds — e.g. 0.033 (VEX's ~30 frames a second, A4 register HA-192).
    double minReadPeriodSeconds = 0.0;
};

/// The first problem with a feed config, or nullptr: a period that is UNSET, zero, negative, NaN or
/// infinite (an infinite one would read once and never again).
[[nodiscard]] constexpr const char* cameraFeedConfigProblem(const CameraFeedConfig& c) noexcept {
    if (!(c.minReadPeriodSeconds > 0.0) || !config::finiteNumber(c.minReadPeriodSeconds)) {
        return "minReadPeriodSeconds UNSET -- the team's read period, seconds (about 0.033 for 30 a second)";
    }
    return nullptr;
}

/// What the feed has seen — for a bench page, and for A4 register HA-203's settle (the read's cost).
struct CameraFeedStats {
    std::uint32_t reads = 0;            ///< reads made
    std::uint32_t usable = 0;           ///< of them, status Ok
    std::uint32_t readFaults = 0;       ///< of them, status ReadFault
    std::uint32_t partialFaults = 0;    ///< of them, status PartialFault (a torn read)
    std::uint32_t notConfigured = 0;    ///< of them, status NotConfigured
    std::uint32_t truncated = 0;        ///< usable reads that were POSSIBLY FULL (24 or more reported)
    std::uint32_t objectsWithheld = 0;  ///< Object detections the adapter withheld, summed over the reads
    std::uint32_t coloursWithheld = 0;  ///< Colour and Code detections the adapter withheld, summed
    double lastReadSeconds = 0.0;       ///< how long the last read took: the clock after it less before it
    double slowestReadSeconds = 0.0;    ///< the longest read so far, seconds
};

/// One camera read per period for every consumer (header). Holds references: the camera and the clock
/// must outlive the feed. PRECONDITION: cameraFeedConfigProblem(cfg) == nullptr (a consumer that
/// builds one asks first and refuses to construct, naming it).
class CameraFeed {
public:
    /// Reads nothing until the first tick().
    CameraFeed(hal::IVisionFrames& camera, const hal::IClock& clock, const CameraFeedConfig& cfg) noexcept
        : camera_{camera}, clock_{clock}, cfg_{cfg} {}

    /// Read the camera if at least the period has passed since the last read BEGAN (the first call always
    /// reads); never more than one read per call. True when it read — latest() is then a new frame.
    bool tick() {
        const double before = clock_.now().value();
        if (haveRead_ && before - lastStart_ < cfg_.minReadPeriodSeconds) {
            return false;
        }
        haveRead_ = true;
        lastStart_ = before;
        latest_ = camera_.read();
        const double took = clock_.now().value() - before;
        ++stats_.reads;
        stats_.lastReadSeconds = took;
        if (took > stats_.slowestReadSeconds) {
            stats_.slowestReadSeconds = took;
        }
        switch (latest_.status) {
            case hal::FrameStatus::Ok:
                ++stats_.usable;
                if (latest_.truncated) {
                    ++stats_.truncated;
                }
                break;
            case hal::FrameStatus::ReadFault: ++stats_.readFaults; break;
            case hal::FrameStatus::PartialFault: ++stats_.partialFaults; break;
            case hal::FrameStatus::NotConfigured: ++stats_.notConfigured; break;
            case hal::FrameStatus::NotRead: break;
        }
        stats_.objectsWithheld += latest_.objectsWithheld;
        stats_.coloursWithheld += latest_.coloursWithheld;
        return true;
    }

    /// The latest READ — a fault frame included, never the last good one held; a default frame (status
    /// NotRead) before the first.
    [[nodiscard]] const hal::VisionFrame& latest() const noexcept { return latest_; }
    /// What the feed has seen.
    [[nodiscard]] const CameraFeedStats& stats() const noexcept { return stats_; }

private:
    hal::IVisionFrames& camera_;
    const hal::IClock& clock_;
    CameraFeedConfig cfg_;
    hal::VisionFrame latest_{};
    CameraFeedStats stats_{};
    double lastStart_ = 0.0;
    bool haveRead_ = false;
};

}  // namespace shulib::vision
