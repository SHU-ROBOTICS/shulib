#pragma once
//
// ICharSink — where formatted diagnostic BYTES physically go (a terminal, a captured
// string in a test, later a serial port). It exists so TermSink's output is a TESTABLE
// claim: with the character device injected (mirroring the injected-clock pattern in
// control/), a host test asserts TermSink's exact bytes against a golden string —
// hard-coding std::cout would have made "readable, column-aligned" unfalsifiable.
//
// NOT part of the frozen F4 ten (that freeze covers the 10 runtime robot-HAL
// interfaces); this is an ADDITIVE diagnostics-output seam introduced at chunk A1.
// R1 adds the on-robot stdout adapter; test/ uses hal::fake::FakeCharSink.
//
// Contract: write() takes the bytes verbatim (the FORMATTER owns sanitization — see
// TermSink), is called synchronously on the caller's task, and MUST NOT throw. One
// write() call carries one complete line, so an implementation that is atomic per call
// never interleaves lines.

#include <string_view>

namespace shulib::hal {

/// Where formatted diagnostic BYTES physically go — a terminal, a captured
/// string in a test, later a serial port. Injecting the character device is what
/// makes TermSink's output a testable claim: a host test asserts its exact bytes
/// against a golden string, which hard-coding std::cout would have made
/// impossible. Sanitization is the FORMATTER's job, not a sink's; a sink takes
/// the bytes verbatim. NOT one of the frozen ten runtime robot-HAL interfaces —
/// this is an additive diagnostics-output seam.
class ICharSink {
public:
    /// Polymorphic-base plumbing: the destructor is virtual so a sink may be owned
    /// and destroyed through an `ICharSink*`, and declaring it suppresses the
    /// implicit copy/move, which are re-defaulted here. Defaulted rather than
    /// deleted because this seam holds no state — but the concrete sinks DO (a
    /// FILE* on the robot, a captured std::string in tests), so copying THROUGH
    /// this base slices one down to the empty interface. Nothing in the library
    /// owns a sink: TermSink keeps an `ICharSink&`, so the sink must outlive every
    /// TermSink pointed at it, and opening/closing the device stays the caller's job.
    virtual ~ICharSink() = default;
    ICharSink() = default;
    ICharSink(const ICharSink&) = default;
    ICharSink(ICharSink&&) = default;
    ICharSink& operator=(const ICharSink&) = default;
    ICharSink& operator=(ICharSink&&) = default;

    /// Write `text` verbatim to the device. MUST NOT throw.
    virtual void write(std::string_view text) = 0;
};

}  // namespace shulib::hal
