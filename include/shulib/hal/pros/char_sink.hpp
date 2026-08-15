#pragma once
//
// ProsCharSink — ICharSink over the V5's USB serial (chunk R1a): where
// TermSink's diagnostic bytes physically go on the robot (`pros terminal`
// displays them).
//
// BINDS: newlib stdout via std::fwrite + std::fflush. The PROS kernel wires
// stdout to the USB serial, so this adapter deliberately includes NO <pros/*>
// header at all — it lives under hal/pros/ because it is the ON-ROBOT sink
// (and so the PROS-free guard's one exemption covers it if it ever needs the
// direct serial API), but its only dependency is <cstdio>. This is the same
// sink main.cpp carried privately as StdoutCharSink since C7, promoted to the
// adapter tree so tests and future consumers share one implementation.
//
// CONTRACT (char_sink.hpp:31-32): bytes verbatim, synchronous on the caller's
// task, MUST NOT throw — fwrite/fflush cannot throw. One write() call carries
// one complete line (the FORMATTER's framing), and fwrite is atomic per call
// at this layer, so lines never interleave.
//
// FLUSH PER WRITE, deliberately: boot-banner visibility is worth more than
// buffered throughput here, and the diagnostics layer already rate-limits
// (the C7 ruling, carried forward unchanged).
//
// The FILE* is injectable (default stdout) so a host test can hand it a
// tmpfile() and assert exact bytes — the same injected-device pattern that
// made TermSink's output a testable claim in the first place (char_sink.hpp
// header).

#include <cstdio>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/hal/char_sink.hpp"

namespace shulib::hal::pros {

/// The on-robot ICharSink: TermSink's diagnostic bytes onto the V5's USB serial,
/// where `pros terminal` displays them. Writes through newlib stdout with
/// fwrite, so it pulls in <cstdio> and no PROS header at all. FLUSHES ON EVERY
/// write — boot-banner visibility is worth more here than buffered throughput,
/// and the diagnostics layer above already rate-limits. Cannot throw, as the
/// ICharSink contract requires.
class ProsCharSink final : public ICharSink {
public:
    /// `out` must be NON-NULL and must outlive the sink; defaults to the V5 USB serial
    /// (stdout). The null check is not decoration: the banner advertises the FILE* as
    /// injectable precisely so a host test can hand it a `tmpfile()`, and that is the one
    /// call site most likely to hand over a null on failure — after which write()'s
    /// std::fwrite is undefined behaviour rather than a loud contract breach. Every other
    /// pointer-taking constructor in the tree already checks (MotorMechanism,
    /// PneumaticMechanism, MechanismDeps, RunGuardConfig); this one did not.
    explicit ProsCharSink(std::FILE* out = stdout) : out_{out} {
        SHULIB_PRECONDITION(out != nullptr, "ProsCharSink: out is null");
    }

    /// Verbatim bytes + flush. MUST NOT throw (contract) — and cannot.
    void write(std::string_view text) override {
        std::fwrite(text.data(), 1, text.size(), out_);
        std::fflush(out_);
    }

private:
    std::FILE* out_;
};

}  // namespace shulib::hal::pros
