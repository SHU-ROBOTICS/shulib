#pragma once
//
// FakeCharSink — records every byte written through the ICharSink seam so host tests can
// assert TermSink's EXACT output (golden strings, column alignment, sanitization). The
// terminal equivalent of FakeTelemetrySink. Host-test only (uses std::string).

#include <string>
#include <string_view>

#include "shulib/hal/char_sink.hpp"

namespace shulib::hal::fake {

class FakeCharSink final : public ICharSink {
public:
    void write(std::string_view text) override { text_ += text; }

    /// Everything written so far, verbatim and in order.
    [[nodiscard]] const std::string& text() const noexcept { return text_; }
    [[nodiscard]] bool empty() const noexcept { return text_.empty(); }

    void clear() noexcept { text_.clear(); }

private:
    std::string text_;
};

}  // namespace shulib::hal::fake
