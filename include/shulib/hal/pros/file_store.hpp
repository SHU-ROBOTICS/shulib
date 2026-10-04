#pragma once
//
// ProsFileStore — hal::IFileStore on the V5's SD card (chunk SAFE-SAVE Part A, 2026-09-29): the
// two operations the safe save builds on — read a file whole, replace it by truncating it —
// composed from the two adapters that already own the card, one adapter per call.
//
// BINDS:
//  * ::pros::usd::is_installed() — cardPresent(), the adapters' own no-card probe (vendored
//    misc.hpp:555-568; HA-122 (1))
//  * hal::pros::ProsBlockSource — read(): its "rb" open, its readAll() from the start, its path()
//  * hal::pros::ProsBlockSink — replace(): its "wb" open (TRUNCATING — HA-166 (1)), its write(),
//    its flush(), and its destructor's close, which cannot report (block_sink.hpp)
//
// ── No fopen, no mode string and no "/usd/" of its own ──────────────────────────────
// Every call builds a ProsBlockSource or a ProsBlockSink in its OWN scope, so the file is closed
// before the call returns — a replace's writer is gone before the caller's read-back opens the
// file again. The adapters stay the only places in the library that call fopen and the only places
// that spell a mode, and the /usd/ prefix stays owned by them (HA-122; calibration_record.hpp's
// kRecordFileName says so). A default-constructed store builds each adapter with ONE argument, so
// the adapter's own default root applies; `ProsFileStore(root)` passes a root through, for host
// tests against a real temp directory. The platform scan (test/record_store_platform_scan_test.cpp)
// pins all three.
//
// ── What replace() can and cannot say ───────────────────────────────────────────────
// Not open → NoCard when cardPresent() is false (the sink never called fopen, so nothing was
// created), else Open. Then Write or Flush as they fail. The close is the sink's destructor, at
// the end of the call's scope: it cannot report (block_sink.hpp), and PROS's close returns 0
// whatever VEXos answers (HA-166 (5)). So replace() makes NO claim about the close, and `None`
// proves nothing — the caller's read-back is the proof (calibration/record_store.hpp's put()).
//
// ── Exceptions and allocation ───────────────────────────────────────────────────────
// Not noexcept (hal/file_store.hpp): a name that is not bare throws PreconditionError from the
// adapter's constructor, before any card is touched, and that must propagate. Allocation happens
// only inside the adapters (each one's std::string path, as today's calibration save already
// does); this class holds one pointer.
//
// HA register: HA-166 (what the card allows), HA-122 (the prefix, the probe).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.hpp"
#pragma GCC diagnostic pop

#include <cstddef>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/file_store.hpp"
#include "shulib/hal/pros/block_sink.hpp"
#include "shulib/hal/pros/block_source.hpp"

namespace shulib::hal::pros {

/// hal::IFileStore on the V5's SD card: read() builds a ProsBlockSource and replace() a
/// ProsBlockSink, each in its own scope, so every file is closed before the call returns. No
/// fopen, no mode string and no "/usd/" of its own — the two adapters own all three (HA-122).
/// replace() opens TRUNCATING (HA-166 (1)) and cannot report the close; read the file back.
class ProsFileStore final : public IFileStore {
public:
    /// The robot's store: each adapter is built with its OWN default root ("/usd/" — owned there).
    ProsFileStore() = default;

    /// A store rooted at `mountRoot` (must end in '/'; the adapters check it) — for host tests
    /// that exercise this exact code against a real temp directory. The pointer is borrowed and
    /// must outlive the store.
    explicit ProsFileStore(const char* mountRoot) : root_{mountRoot} {
        SHULIB_PRECONDITION(mountRoot != nullptr && mountRoot[0] != '\0',
                            "ProsFileStore: mountRoot is null/empty");
    }

    /// `usd_is_installed() != 0` — the adapters' own no-card probe (HA-122 (1)).
    [[nodiscard]] bool cardPresent() const override { return ::pros::usd::is_installed() != 0; }

    /// Build a ProsBlockSource for `bareName`; not open → `opened` false; open → its readAll()
    /// from the start. The path is the source's own path(). A name that is not bare throws.
    [[nodiscard]] StoreRead read(const char* bareName, std::span<std::byte> into) override {
        StoreRead out{};
        withAdapter<ProsBlockSource>(bareName, [&](ProsBlockSource& source) {
            detail::copyStorePath(out.path, source.path());
            if (!source.isOpen()) {
                return;
            }
            out.opened = true;
            out.count = source.readAll(into);
        });
        return out;
    }

    /// Build a ProsBlockSink for `bareName` (its "wb" open TRUNCATES — HA-166 (1)), write, flush,
    /// and let its scope close it. Not open → NoCard if cardPresent() is false, else Open; then
    /// Write or Flush as they fail. The close cannot report, so `None` proves nothing (header).
    [[nodiscard]] StoreReplace replace(const char* bareName,
                                       std::span<const std::byte> bytes) override {
        StoreReplace out{};
        withAdapter<ProsBlockSink>(bareName, [&](ProsBlockSink& sink) {
            detail::copyStorePath(out.path, sink.path());
            if (!sink.isOpen()) {
                out.failed = cardPresent() ? ReplaceStep::Open : ReplaceStep::NoCard;
                return;
            }
            if (!sink.write(bytes)) {
                out.failed = ReplaceStep::Write;
                return;
            }
            if (!sink.flush()) {
                out.failed = ReplaceStep::Flush;
            }
        });  // the sink's destructor closes the file HERE — unreportable (block_sink.hpp)
        return out;
    }

private:
    /// Build one `Adapter` for `name` in this scope — with ONE argument when no root was injected,
    /// so the adapter's own default root applies — run `body` on it, and let it close.
    template <typename Adapter, typename Body>
    void withAdapter(const char* name, Body&& body) const {
        if (root_ == nullptr) {
            Adapter adapter{name};
            body(adapter);
            return;
        }
        Adapter adapter{name, root_};
        body(adapter);
    }

    const char* root_ = nullptr;  // null = each adapter's own default root
};

}  // namespace shulib::hal::pros
