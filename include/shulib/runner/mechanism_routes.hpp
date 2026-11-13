#pragma once
//
// runner/mechanism_routes.hpp — a robot's MECHANISM ROUTE IDS (mechanism/route_ids.hpp), bound to its
// mechanism service and registered in a route registry in ONE call (the mechanism core, chunk
// MECH-A4, 2026-10-22; the team lead's ruling of 2026-10-08: "every mechanism publishes its actions as
// route ids AUTOMATICALLY ... a coder no longer registers each id by hand").
//
// ── One call ─────────────────────────────────────────────────────────────────────────
//     static runner::CommandRegistry registry;                 // static storage, a program's
//     static runner::MechanismRoutes routes{chassis, service, kRobotTwoMechanisms, sink};
//     routes.registerInto(registry);                           // after the service is bound
// Every id the robot's list derives — `arm_score`, `intake_until_piece`, `claw_closed` — is then
// registered, in the list's order, each with a one-line doc the manifest publishes for a route
// editor's marker picker ("claw pivot 6: run; args[0] = volts (-12..12), args[1] = how long, ms").
//
// ── A marker runs EXACTLY the step an auton's one line runs ─────────────────────────
// Through a chassis::Routine built on the same chassis and service — one verdict policy, not two
// (chassis/routine.hpp's MECH-A4 block): `arm_score [1500]` is `r.move(arm["score"], 1.5_s)`,
// `intake_until_piece [12, 3000]` is `r.run(intake, 12_volt, seesPiece, 3_s)`, `claw_closed` is
// `r.set(claw["closed"])`. The marker BLOCKS at its waypoint until the step's verdict (markers fire
// only after a leg SETTLES — F8's ruling R2 — so a route marker never runs while driving; a step that
// starts mid-leg is a Routine's PARTWAY TRIGGER since STEP1-B, `r.atPercent(60).startMove(...).driveTo(...)`
// (chassis/routine.hpp) — a Routine form only: route markers stay blocking, F8 unchanged). SUCCEEDED — and
// UNCONFIRMED counted done (robot two's sensorless grab, the team lead's Q2 (c)) — continue the
// route; every other verdict returns to the PathRunner, whose policy stops the route (MarkerFailed,
// the verdict in its line). A flywheel's spin and an arm's hold go on after their marker (the service
// keeps them); the run guard's end of run makes every mechanism safe. A long time limit on a marker
// is a long stop at its waypoint (the brief's landmine 23).
//
// ── A marker's args are DATA a route author wrote ───────────────────────────────────
// So they are checked HERE, never trusted: a time limit is required and above 0 (P4: every action
// bounded; P6: the limit is the route author's, never a library default); a power id's volts are
// within ±12 and not 0. A marker that fails is REFUSED (manipulation::MechanismOutcome::Refused) with
// ONE Warn naming the id and what it needs — `route marker 'arm_score' REFUSED: it needs its time
// limit in ms, above 0 (args[0])` — and the route stops there, nothing commanded.
//
// Lives in runner/ because it needs both the Routine (chassis/) and the registry; mechanism/ stays
// below chassis/. PROS-free, allocation-free. Nothing here has run on a robot.

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/until.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::runner {

/// The longest doc one derived id carries, bytes (NUL included) — "intake: run until the piece is
/// gone; args[0] = volts, args[1] = time limit, ms" with a 23-character name fits with room.
inline constexpr std::size_t kMechanismRouteDocBytes = 112;

/// One robot's mechanism route ids, bound to its chassis and mechanism service (file header). ONE
/// lives in the program's static storage beside its registry (the registry keeps pointers into it);
/// register after the service is bound. Non-copyable and non-movable for the same reason.
class MechanismRoutes {
public:
    /// Derive `robot`'s ids (routeIdsOf — the build already refused a list whose ids cannot be
    /// published) and write each one's doc. Borrows all four: they must outlive this object (static
    /// storage in a program). Registers nothing — registerInto() does.
    MechanismRoutes(chassis::Chassis& chassis, mechanism::MechanismService& service,
                    const mechanism::MechanismTable& robot, hal::ITelemetrySink& telemetry) noexcept
        : chassis_{&chassis}, svc_{&service}, robot_{&robot}, sink_{&telemetry},
          ids_{mechanism::routeIdsOf(robot.specSpan())} {
        for (std::size_t i = 0; i < ids_.count && i < mechanism::kMaxRouteIds; ++i) {
            ctx_[i] = Ctx{this, static_cast<std::uint8_t>(i)};
            describe(i);
        }
    }
    /// Non-copyable: the registry holds pointers into this object.
    MechanismRoutes(const MechanismRoutes&) = delete;
    /// Non-copy-assignable (the copy constructor's reason).
    MechanismRoutes& operator=(const MechanismRoutes&) = delete;
    /// Non-movable (the copy constructor's reason).
    MechanismRoutes(MechanismRoutes&&) = delete;
    /// Non-move-assignable (the copy constructor's reason).
    MechanismRoutes& operator=(MechanismRoutes&&) = delete;
    /// Nothing to release.
    ~MechanismRoutes() = default;

    /// THE ONE CALL: every id into `registry`, in the list's order, each with its doc and its arg
    /// count. PRECONDITIONS: this robot's ids can be published (the build refuses a list whose ids
    /// cannot, so a hand-built table is the only way here); and the registry's own — room for every
    /// id, and none already registered (calling this twice is a duplicate).
    void registerInto(CommandRegistry& registry) {
        SHULIB_PRECONDITION(ids_.problem == mechanism::RouteIdProblem::None,
                            "MechanismRoutes::registerInto: this robot's route ids cannot be published (the build "
                            "refuses such a list -- a hand-built table?)");
        for (std::size_t i = 0; i < ids_.count; ++i) {
            registry.on(ids_.ids[i].view(), Handler{&fire, &ctx_[i]}, ids_.ids[i].argCount, docs_[i]);
        }
    }
    /// The ids, as derived (routeIdsOf).
    [[nodiscard]] const mechanism::RouteIds& ids() const noexcept { return ids_; }
    /// Id `i`'s doc, as the manifest publishes it. PRECONDITION: `i` below ids().count.
    [[nodiscard]] const char* doc(std::size_t i) const {
        SHULIB_PRECONDITION(i < ids_.count, "MechanismRoutes::doc: no such id");
        return docs_[i];
    }

private:
    struct Ctx {
        MechanismRoutes* self = nullptr;
        std::uint8_t i = 0;
    };
    /// What a Routine's context constructor reads (competition::AutonContext has the same two).
    struct StepContext {
        chassis::Chassis& chassis;
        mechanism::MechanismService* mechanismService;
    };

    /// Id `i`'s one-line doc: what it does and what its args mean (the rule's table, route_ids.hpp).
    void describe(std::size_t i) {
        const mechanism::RouteId& r = ids_.ids[i];
        const mechanism::MechanismSpec& s = robot_->specs[r.mechanism];
        char* out = docs_[i];
        const std::size_t cap = kMechanismRouteDocBytes;
        using A = mechanism::RouteAction;
        switch (r.action) {
            case A::GoToPreset:
                std::snprintf(out, cap, "%.23s: move to '%.15s' and hold it; args[0] = time limit, ms", s.name,
                              s.position.presets[r.index].name);
                return;
            case A::Home:
                std::snprintf(out, cap, "%.23s: find zero; args[0] = time limit, ms", s.name);
                return;
            case A::SpinUp:
                std::snprintf(out, cap, "%.23s: spin up to '%.15s' and keep spinning; args[0] = time limit, ms", s.name,
                              s.speed.presets[r.index].name);
                return;
            case A::Run:
                std::snprintf(out, cap, "%.23s: run; args[0] = volts (-12..12), args[1] = how long, ms", s.name);
                return;
            case A::RunUntilPiece:
                std::snprintf(out, cap, "%.23s: run until it sees a piece; args[0] = volts, args[1] = time limit, ms",
                              s.name);
                return;
            case A::RunUntilGone:
                std::snprintf(out, cap,
                              "%.23s: run until the piece is gone; args[0] = volts, args[1] = time limit, ms", s.name);
                return;
            case A::Stop:
                std::snprintf(out, cap, "%.23s: stop (its safe state)", s.name);
                return;
            case A::SetPosition:
                std::snprintf(out, cap, "%.23s: go to '%.15s'", s.name, s.pneumatic.positions[r.index].name);
                return;
        }
        std::snprintf(out, cap, "%.23s", s.name);
    }

    /// The marker is REFUSED: one Warn naming the id and what it needs (file header).
    manipulation::MechanismOutcome refuseMarker(const mechanism::RouteId& r, const char* needs) {
        char line[256];
        std::snprintf(line, sizeof line, "route marker '%.23s' REFUSED: it needs %.150s", r.id, needs);
        sink_->log(hal::LogLevel::Warn, "MECH", line);
        return manipulation::MechanismOutcome::Refused;
    }

    /// The registered handler (Handler::fn): id `ctx`'s marker, run.
    static manipulation::MechanismOutcome fire(void* ctx, const config::Marker& marker) {
        const Ctx& c = *static_cast<const Ctx*>(ctx);
        return c.self->run(c.self->ids_.ids[c.i], marker);
    }

    /// One marker: its args checked (DATA), then EXACTLY the auton's step, through a Routine.
    manipulation::MechanismOutcome run(const mechanism::RouteId& r, const config::Marker& m) {
        using A = mechanism::RouteAction;
        const mechanism::MechanismSpec& s = robot_->specs[r.mechanism];
        const bool timed = r.action == A::GoToPreset || r.action == A::Home || r.action == A::SpinUp;
        const bool powered = r.action == A::Run || r.action == A::RunUntilPiece || r.action == A::RunUntilGone;
        if (timed && (m.argCount < 1 || m.args[0] <= 0)) {
            return refuseMarker(r, "its time limit in ms, above 0 (args[0])");
        }
        if (powered && (m.argCount < 2 || m.args[0] == 0 || m.args[0] < -12 || m.args[0] > 12 || m.args[1] <= 0)) {
            return refuseMarker(r, "its volts (args[0], -12..12, not 0) and its time in ms, above 0 (args[1])");
        }
        StepContext sc{*chassis_, svc_};
        chassis::Routine step{sc, r.id};
        const units::Time limit{static_cast<double>(timed ? m.args[0] : powered ? m.args[1] : 0) / 1000.0};
        switch (r.action) {
            case A::GoToPreset: {
                const mechanism::Preset& p = s.position.presets[r.index];
                step.move(mechanism::PositionGoal{nullptr, s.name, p.name, p.value, false, s.position.units}, limit);
                break;
            }
            case A::Home:
                // the service finds a position mechanism by NAME and UNITS (P12): an arm's or a lift's
                if (s.position.units == mechanism::Units::Inches) {
                    mechanism::Lift d{};
                    d.name.text = s.name;
                    step.home(d, limit);
                } else {
                    mechanism::Arm d{};
                    d.name.text = s.name;
                    step.home(d, limit);
                }
                break;
            case A::SpinUp: {
                const mechanism::Preset& p = s.speed.presets[r.index];
                step.spinUp(mechanism::SpeedGoal{nullptr, s.name, p.name, p.value, false}, limit);
                break;
            }
            case A::Run:
            case A::RunUntilPiece:
            case A::RunUntilGone: {
                mechanism::Power d{};
                d.name.text = s.name;
                const units::Voltage v{static_cast<double>(m.args[0])};
                if (r.action == A::Run) {
                    step.run(d, v, limit);
                } else {
                    step.run(d, v, r.action == A::RunUntilPiece ? mechanism::Until::sees() : mechanism::Until::stopsSeeing(),
                             limit);
                }
                break;
            }
            case A::Stop:
                if (s.kind == mechanism::Kind::Speed) {
                    mechanism::Flywheel d{};
                    d.name.text = s.name;
                    step.stop(d);
                } else {
                    mechanism::Power d{};
                    d.name.text = s.name;
                    step.stop(d);
                }
                break;
            case A::SetPosition:
                step.set(mechanism::PneumaticGoal{nullptr, s.name, r.index, s.pneumatic.positions[r.index].name});
                break;
        }
        if (step.ok()) {
            return manipulation::MechanismOutcome::Succeeded;  // SUCCEEDED, or UNCONFIRMED counted done (Q2 (c))
        }
        return step.result().mechanismVerdict;
    }

    chassis::Chassis* chassis_;
    mechanism::MechanismService* svc_;
    const mechanism::MechanismTable* robot_;
    hal::ITelemetrySink* sink_;
    mechanism::RouteIds ids_;
    Ctx ctx_[mechanism::kMaxRouteIds]{};
    char docs_[mechanism::kMaxRouteIds][kMechanismRouteDocBytes]{};
};

}  // namespace shulib::runner
