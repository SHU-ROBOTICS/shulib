#pragma once
//
// CommandRegistry — the binding from a marker's ID to the code that performs it
// (chunk G2, 2026-09-18; master plan §16.3; docs/vexbuilder-integration.md §4.2).
//
// ── Why this table is the keystone of the no-code story ─────────────────────────────
// A Route (config/route.hpp) says WHERE the robot goes and WHAT to do on the way, and the
// "what" is an ID — never code. The legacy planner embedded C++ snippets in its rows
// (`code_template: "mech.intakeIn();"`), which is rejected outright (ruling B5): data that
// can execute is a second, unreviewable source file. This table is the other half of that
// decision. A coder writes ONE line per capability,
//
//     runner::CommandRegistry registry;
//     registry.on("intake_in", intakeIn);      // intakeIn is a callable that OUTLIVES this
//
// and from then on a non-coder authoring a path in VexBuilder picks "intake_in" from a list
// the library itself publishes (runner/manifest.hpp). The auton is data a non-coder writes;
// the handlers are code the coders maintain.
//
// ── No std::function, no heap, no std::string ───────────────────────────────────────
// A V5 brain has 128 KB of user RAM and this table may be built during an autonomous
// period, so every allocation is a risk taken for nothing. A Handler is therefore a plain
// function pointer plus an opaque context pointer — the C idiom, chosen because it is the
// only one that is both type-erased and allocation-free. The template `on()` overload
// builds that pair from any callable and stores a NON-OWNING pointer to it: the callable
// must outlive the registry, the same ownership rule `Routine::then()`'s action and
// `StaticRoutes`'s span already carry. That is enforced by the signature, not by a comment
// — the overload takes `Callable&`, so `on("x", []{ ... })` (a temporary lambda) does not
// compile, and the lifetime bug it would cause cannot be written.
//
// ── A handler runs SYNCHRONOUSLY, on the caller's task, between legs ────────────────
// PathRunner calls a handler from inside run(), after the leg it hangs off has settled, and
// waits for it to return. A handler that blocks for two seconds blocks the route for two
// seconds. That is the right default for "close the clamp, then drive away" and the wrong
// one for "spin the intake for the rest of the match". Since the mechanism core (chunk
// MECH-A4) a robot's DECLARED mechanisms need no hand-written handler at all: their route
// ids are derived from the robot's list and registered in ONE call
// (runner/mechanism_routes.hpp) — each marker blocks through a Routine step until its
// verdict, and what goes on after it (an arm's hold, a flywheel's spin) is the mechanism
// service's, which ticks every mechanism inside every shulib wait. A step that must start
// mid-leg is the mechanism core's later work ("triggers partway through a drive"); for an
// F1 operation the idiom is unchanged: a handler that STARTS it and a later marker that
// confirms it (manipulation/mechanism_op.hpp).
//
// ── A doc per id (chunk MECH-A4, F8-additive) ───────────────────────────────────────
// An entry may carry a one-line `doc` — what the id does and what its args mean — which the
// manifest publishes for a picker to show beside the id ("args[0] = volts, args[1] = how
// long, ms" instead of a bare "2 args"). The four-argument on() sets it; an entry without one
// publishes `"doc":""`, so a registry with no docs writes exactly the bytes it wrote before.
//
// ── What is a programming error here, and what is data ──────────────────────────────
// Registration is code, so everything wrong with a registration is a PRECONDITION: a
// duplicate id (two handlers for one id is not a merge, it is a typo), a built-in id
// (library-owned — ruling R3), an id outside the vocabulary, a null function pointer, more
// than kMaxCommands. An UNKNOWN id at RUN time is the opposite: it is data, the runner logs
// one Warn and carries on (vexbuilder-integration.md's guarantee), and nothing about it
// reaches this class.
//
// FROZEN: F8 (docs/roadmap.md), pinned by test/f8_signature_pin_test.cpp.
//
// PROS-free, sim-free, allocation-free. Single-task, like everything it is called from.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>

#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"

namespace shulib::runner {

/// The most command ids one registry may hold. A DESIGN constant, on `kMaxWaypoints`'s
/// reasoning (B6): a robot with more than 32 distinct scoring capabilities is not a robot this
/// bound is the problem for. Nothing measured says 32; it is the fixed capacity that keeps the
/// table allocation-free.
inline constexpr std::size_t kMaxCommands = 32;

/// One bound action: a plain function pointer plus the opaque context it is called with — the
/// type-erased, allocation-free pair `std::function` would otherwise be used for (file banner).
///
/// `fn` receives the whole `config::Marker`, not just its args, so a handler can read its own id
/// (one function may serve several related ids) and its typed arguments without a second lookup.
/// It returns a `manipulation::MechanismOutcome`: anything other than `Succeeded` stops the route
/// under the runner's error policy, and the vocabulary has no conversion to bool, so a failed
/// grab can never read as success by accident (the T2 guarantee).
struct Handler {
    /// The action. Never null in a registered Handler (a precondition of `on`).
    manipulation::MechanismOutcome (*fn)(void* ctx, const config::Marker& marker) = nullptr;
    /// Whatever `fn` needs — the address of the caller's callable for the template `on()`
    /// overload, or anything at all for the explicit one. NON-OWNING: the pointee must outlive
    /// the registry. May be null when `fn` needs no context.
    void* ctx = nullptr;
};

/// One registration, as the registry stores it: the id, how many typed args a marker carrying it
/// must supply, and the bound action.
///
/// The id is a fixed-width NUL-terminated buffer rather than a `std::string_view` into the
/// caller's memory, because a registration commonly reads `on("intake_in", …)` from a string
/// literal but just as legitimately from a buffer a parser filled — and a table of views into
/// somebody else's parse buffer is a dangling read waiting for G3.
struct CommandEntry {
    /// The id text, NUL-terminated (`commandIdValid` at registration, so a terminator always
    /// fits). Read it through `idView()` rather than directly.
    char id[config::kMaxMarkerIdBytes]{};
    /// How many typed args a marker with this id carries — DECLARED at registration, and
    /// published in the manifest so a picker can prompt for exactly that many. The library does
    /// not enforce it on a marker at run time (an unenforceable claim about someone else's
    /// handler); it is the contract the picker and the handler share.
    std::uint8_t argCount = 0;
    /// The bound action.
    Handler handler{};
    /// What the id does and what its args mean, in one line, published in the manifest's `doc`
    /// field (file banner). NON-OWNING — the text must outlive the registry; free of `"` and `\`
    /// (the manifest's rule). nullptr = no doc: the manifest's `""`, as before it existed.
    /// (APPENDED at chunk MECH-A4 — F8's "new registry members", API 2.5.)
    const char* doc = nullptr;

    /// The id as text.
    [[nodiscard]] constexpr std::string_view idView() const noexcept {
        return config::commandIdView(id, config::kMaxMarkerIdBytes);
    }
};

/// The id → action table a team fills once at startup, and `PathRunner` reads while a route runs
/// (file banner). Fixed capacity, no heap, no `std::function`; every registration error is a
/// precondition, because registration is code.
class CommandRegistry {
public:
    /// An empty registry. The built-in ids (`config::kBuiltinCommands`) are live from here on
    /// without appearing in this table at all — they belong to the runner, not to a team.
    CommandRegistry() = default;

    /// Bind `id` to `handler`; `argCount` is how many typed args a marker with this id carries
    /// (published in the manifest; 0 for the common case).
    ///
    /// Preconditions, every one of them a programming error rather than a datum (file banner):
    /// `id` is `config::commandIdValid`; it is not one of `config::kBuiltinCommands`; it is not
    /// already registered; `handler.fn` is not null; `argCount <= config::kMaxMarkerArgs`; and the
    /// table has room (`kMaxCommands`).
    void on(std::string_view id, Handler handler, std::uint8_t argCount = 0) {
        SHULIB_PRECONDITION(config::commandIdValid(id),
                            "CommandRegistry::on: the id must be 1..23 bytes of [a-z0-9_]");
        SHULIB_PRECONDITION(config::findBuiltinCommand(id) == nullptr,
                            "CommandRegistry::on: that id is a library built-in (wait_ms / brake "
                            "/ hold_ms / log) and cannot be re-bound — a team that re-meant one "
                            "would break every route another team exported");
        SHULIB_PRECONDITION(find(id) == nullptr,
                            "CommandRegistry::on: that id is already registered — two handlers "
                            "for one id is a typo, not a merge");
        SHULIB_PRECONDITION(handler.fn != nullptr,
                            "CommandRegistry::on: the handler's function pointer is null");
        SHULIB_PRECONDITION(argCount <= static_cast<std::uint8_t>(config::kMaxMarkerArgs),
                            "CommandRegistry::on: argCount exceeds config::kMaxMarkerArgs");
        SHULIB_PRECONDITION(count_ < kMaxCommands,
                            "CommandRegistry::on: the registry is full (kMaxCommands)");
        CommandEntry& e = entries_[count_];
        std::memcpy(e.id, id.data(), id.size());
        e.id[id.size()] = '\0';
        e.argCount = argCount;
        e.handler = handler;
        ++count_;
    }

    /// Bind `id` to any callable, adapting its return type exactly as `Routine::then()` does:
    ///   * `void` — the action always succeeds,
    ///   * `bool` — `false` fails (reported as `MechanismOutcome::Unconfirmed`: the handler ran
    ///     and the task did not happen, which is precisely what that verdict means; claiming
    ///     `TimedOut` or `Stalled` would invent a pathology the `bool` never stated),
    ///   * `manipulation::MechanismOutcome` — passed through unchanged, so only `Succeeded`
    ///     continues the route and an `Unconfirmed` grab can never read as success.
    /// A callable returning anything else is a compile error naming the three.
    /// (`control::ExitReason`, which `then()` also accepts, is deliberately NOT accepted here:
    /// mapping a motion verdict onto a mechanism verdict would have to invent a meaning for
    /// `Running`, and a handler that wraps a facade verb can return the outcome it means.)
    ///
    /// The callable may take the marker or take nothing — `on("lift_to_level", liftTo)` where
    /// `liftTo` reads `marker.args[0]`, or `on("intake_in", intakeIn)` where it does not.
    ///
    /// NON-OWNING: a pointer to `callable` is stored, so it must outlive the registry. The
    /// `Callable&` parameter is what enforces that — a temporary lambda does not bind to it, so
    /// the dangling registration cannot be written (file banner).
    template <typename Callable>
    void on(std::string_view id, Callable& callable, std::uint8_t argCount = 0) {
        on(id, Handler{&adaptCall<Callable>, static_cast<void*>(&callable)}, argCount);
    }

    /// Bind `id` to `handler` exactly as the three-argument form does, and give it a one-line `doc`
    /// the manifest publishes (file banner) — how runner::MechanismRoutes registers every derived
    /// mechanism id. Every precondition of the three-argument form, and: `doc` is null or free of
    /// `"` and `\` (the manifest's own rule, checked HERE so a bad doc fails at registration, not
    /// when the manifest is written). NON-OWNING: `doc` must outlive the registry. (APPENDED at
    /// chunk MECH-A4 — F8's "new registry members", API 2.5.)
    void on(std::string_view id, Handler handler, std::uint8_t argCount, const char* doc) {
        SHULIB_PRECONDITION(doc == nullptr || (std::string_view{doc}.find('"') == std::string_view::npos &&
                                               std::string_view{doc}.find('\\') == std::string_view::npos),
                            "CommandRegistry::on: a doc may not contain a quote or a backslash (the manifest "
                            "cannot carry one)");
        on(id, handler, argCount);
        entries_[count_ - 1].doc = doc;
    }

    /// The handler bound to `id`, or NULLPTR when nothing is. A null return is the whole error
    /// channel on purpose (`IRouteSource::route()`'s convention): "no handler for this id" is a
    /// normal thing for a runner to find, and it answers it with a Warn and a skip.
    ///
    /// Built-in ids are NOT in this table and this returns nullptr for them; the runner checks
    /// the built-ins first.
    [[nodiscard]] const Handler* find(std::string_view id) const noexcept {
        for (std::size_t i = 0; i < count_; ++i) {
            if (entries_[i].idView() == id) {
                return &entries_[i].handler;
            }
        }
        return nullptr;
    }

    /// How many ids are registered (built-ins not included — they are never in this table).
    [[nodiscard]] std::size_t count() const noexcept { return count_; }

    /// Every registration, in REGISTRATION ORDER — which is the order the manifest publishes them
    /// in, so a diff of two manifests reads as a diff of two `on()` sequences.
    [[nodiscard]] std::span<const CommandEntry> ids() const noexcept {
        return std::span<const CommandEntry>{entries_, count_};
    }

private:
    /// The adaptation in the member comment on the template `on()`, as a free function with the
    /// `Handler::fn` signature. Split in two by `if constexpr` on whether the callable wants the
    /// marker; both arms then share `adaptResult`, so there is exactly ONE place the three
    /// accepted return types are mapped.
    template <typename Callable>
    static manipulation::MechanismOutcome adaptCall(void* ctx, const config::Marker& marker) {
        Callable& callable = *static_cast<Callable*>(ctx);
        if constexpr (std::is_invocable_v<Callable&, const config::Marker&>) {
            return adaptResult([&]() -> decltype(auto) { return std::invoke(callable, marker); });
        } else {
            return adaptResult([&]() -> decltype(auto) { return std::invoke(callable); });
        }
    }

    /// `Routine::then()`'s adaptation, expressed once (member comment above).
    template <typename Invoke>
    static manipulation::MechanismOutcome adaptResult(Invoke&& invoke) {
        using R = std::invoke_result_t<Invoke&>;
        if constexpr (std::is_void_v<R>) {
            invoke();
            return manipulation::MechanismOutcome::Succeeded;
        } else if constexpr (std::is_same_v<R, manipulation::MechanismOutcome>) {
            return invoke();
        } else {
            static_assert(std::is_same_v<R, bool>,
                          "CommandRegistry::on: a handler must return void, bool, or "
                          "manipulation::MechanismOutcome");
            return invoke() ? manipulation::MechanismOutcome::Succeeded
                            : manipulation::MechanismOutcome::Unconfirmed;
        }
    }

    CommandEntry entries_[kMaxCommands]{};
    std::size_t count_ = 0;
};

}  // namespace shulib::runner
