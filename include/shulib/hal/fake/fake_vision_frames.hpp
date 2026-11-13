#pragma once
//
// FakeVisionFrames — the host's settable camera (chunk VIS1-A1, 2026-10-18): an hal::IVisionFrames AND an
// hal::IReadFaults in ONE object, as hal::pros::ProsAiVision is.
//
// WHY ONE OBJECT, unlike the distance and optical fakes: MECH-A1 left the F4 fakes (FakeDistance,
// FakeOptical) unmodified, and an F4 fake cannot count its faults, so FakeDeviceProvider pairs each with
// a separate FakeReadFaults. This camera fake IS an IReadFaults, like the adapter, so its count moves
// WITH the fault frames it returns — a separate counter would be a second knob a test had to keep in
// step by hand (VIS1's re-check T9). FakeDeviceProvider therefore answers readFaults(port) with this
// object itself, and only once aiVision() has served it.
//
// setNext(frame) sets what every read() returns until changed (its readSeq is overwritten with the
// fake's own poll count); setFailing(true) makes every read a ReadFault with NO detections, counted;
// setStatus() sets what status() answers — the Robot's boot line is tested through it in every state
// the seam can report.
//
// Host-test infrastructure (outside the API reference, like every fake — test/README.md documents it).
// Single-task; allocation-free; PROS-free; sim-free.

#include <cstdint>

#include "shulib/hal/read_faults.hpp"
#include "shulib/hal/vision_frames.hpp"

namespace shulib::hal::fake {

/// A settable camera (header).
class FakeVisionFrames final : public IVisionFrames, public IReadFaults {
public:
    /// What every read() returns until changed. Its readSeq is overwritten with the poll count.
    void setNext(const VisionFrame& f) noexcept { next_ = f; }
    /// While true, every read() returns a ReadFault with no detections and counts one fault.
    void setFailing(bool failing) noexcept { failing_ = failing; }
    /// What status() answers from now on.
    void setStatus(const CameraStatus& s) noexcept { status_ = s; }
    /// How many times read() was called.
    [[nodiscard]] std::uint32_t reads() const noexcept { return reads_; }

    /// The set frame (or, while failing, a ReadFault with no detections stamped at the set frame's
    /// time); a returned ReadFault or PartialFault counts one fault either way.
    [[nodiscard]] VisionFrame read() noexcept override {
        ++reads_;
        if (failing_) {
            VisionFrame f{};
            f.status = FrameStatus::ReadFault;
            f.readAt = next_.readAt;
            f.readSeq = reads_;
            ++faulted_;
            return f;
        }
        VisionFrame f = next_;
        f.readSeq = reads_;
        if (f.status == FrameStatus::ReadFault || f.status == FrameStatus::PartialFault) {
            ++faulted_;
        }
        return f;
    }
    /// What setStatus() last set (a default CameraStatus — NOT set up — until then).
    [[nodiscard]] CameraStatus status() const noexcept override { return status_; }
    /// Fault frames returned so far (a failing read, or a set frame whose status is a fault). Monotonic.
    [[nodiscard]] int faultedReads() const noexcept override { return faulted_; }

private:
    VisionFrame next_{};
    CameraStatus status_{};
    bool failing_ = false;
    std::uint32_t reads_ = 0;
    int faulted_ = 0;
};

}  // namespace shulib::hal::fake
