# R1b Progress Log — hal/pros adapters for the mechanism seams

Live log, appended in real time. Timestamps are local (24h).

[00:06:40] START   R1b begins. First action: this log. Next: required reading in order
                   (brief, R1a docs, bench session, existing adapters, seam headers, shim, HA register).
[00:06:52] DONE    Read R1b brief in full — 6 tensions, 14 mutations, HA-113+ register duty, 9999 rule is L1.
[00:07:02] DONE    Read R1a brief + R1a-COMPLETED + R1a-BENCH-SESSION. Key inherits: quoted includes, 2-flag fence, path-anchored guard, shim #error + adversarial defaults, screen->hold->expose idiom, mtime bump on restore, ADI expander UNVERIFIED (registry idx 21 out of range).
[00:07:18] START   Reading vendored PROS headers: distance.hpp, optical.hpp, adi.hpp, usd.h — the source of truth for shim semantics.
[00:15:30] DONE    Required reading complete: existing 10 adapters' shape (fence, BINDS banner,
                   HA citations, screen->hold->expose), seam headers (distance/optical/digital_out/
                   block_sink/mechanism/controller), shim conventions (#error, adversarial defaults,
                   beliefs cited file:line), fence guard test (pins I must keep green and extend),
                   test idiom, HA register format (next free HA-113), bench runbook step format,
                   doc surfaces (changelog/faq/guide13/guide14/roadmap). Doc-coverage gate TARGETS
                   only chassis.hpp+routine.hpp — new headers need /// discipline but not docs/api.
[00:15:30] FOUND   Vendored distance.hpp:71/98: "Will return 9999 if the sensor can not detect an
                   object" — confirmed against the REAL vendored header, and get_confidence()
                   (:133-135) "range of 0 to 63 ... only available when distance is > 200mm".
                   Also present: get_object_size() (0-400, -1 = undeterminable) and
                   get_object_velocity() (m/s) — NOT in the IDistance seam; adapters won't bind them.
[00:15:30] FOUND   Vendored adi.hpp:564/596: BOTH DigitalOut ctors default init_state = LOW —
                   construction actuates with a defaultable argument (T3's hazard verbatim).
                   DigitalIn: get_new_press() (:729) documented not-thread-safe, consuming;
                   get_value() via `using Port::get_value` (:757). ADI ports 1-8 or 'a'-'h'/'A'-'H';
                   expander is ext_adi_port_pair_t {smart 1-21(doc says 1-22 in one place!), adi}.
[00:15:30] FOUND   Vendored misc.h:824-825 usd_list_files: "DO NOT PREPEND YOUR PATHS WITH /usd/"
                   while newlib fopen on the brain requires the /usd/ prefix — the two-conventions
                   trap, confirmed in the vendored source. usd_is_installed() returns int32 1/0.
[00:15:30] DECIDE  Plan of build order: (1) hal/digital_in.hpp seam + fake, (2) pure conversions
                   distance/optical, (3) shim headers distance/optical/adi/usd(misc extension),
                   (4) adapters distance/optical/digital_out/digital_in/block_sink, (5) tests,
                   (6) guards+ARM+make, (7) mutation campaign, (8) HA-113+ register + runbook +
                   docs. Rationale: seams before adapters, beliefs (shim) written from vendored
                   docs BEFORE adapter code so the shim is not shaped to make the adapter pass.
[00:22:00] DECIDE  T4 ruling implemented as: raw==9999 -> confidence()==0.0 AND distance() returns
                   the honest conversion of 9999 (393.66141732... in, finite, far, fails any
                   proximity threshold) over hold-last-good (a stale object would read "present" —
                   the dangerous direction for capture-confirm). PROS_ERR (device failure) is a
                   DIFFERENT state: distance holds last-good-finite, confidence 0, faultedReads++.
                   Initial last-good = the far value, not 0.0 (0.0 = "object touching sensor" is
                   the dangerous boot default for a dead sensor).
[00:22:00] DECIDE  The <=200mm confidence rule: a valid reading <= 200 mm reports confidence 1.0 —
                   a returned non-9999 distance IS the detection; the confidence channel is a
                   quality signal PROS documents as unavailable there, and passing the raw value
                   through could read 0 on an object touching the sensor (the worst failure for a
                   mechanism). Rejected: raw/63 passthrough below 200 mm (reports a number PROS
                   says does not exist — the brief's exact prohibition).
[00:22:00] DECIDE  T5 implemented: ProsBlockSink ctor takes (fileName, mountRoot="/usd/") — ONE
                   code path for the prefix logic, host-testable by injecting a tmpdir root;
                   usd::is_installed()==0 or fopen failure -> sink constructs, isOpen()==false,
                   every write()/flush() false. Expose-don't-raise (hal below diag), R1a idiom.
                   Precondition: fileName must not start with '/' (the double-prefix trap, loud).
[00:22:00] DECIDE  T6 implemented: ONE ProsDigitalOut/ProsDigitalIn class, two ctors each
                   (built-in uint8 port; expander {smartPort, adiPort}) — the seam is identical
                   and the wire location is a construction fact. Expander existence on OUR robot
                   is UNKNOWN (R1a registry idx 21 out of range) -> runbook step + HA entry.
[00:22:00] DECIDE  T3 implemented: ProsDigitalOut initial state is a REQUIRED ctor arg, no
                   default (PROS defaults LOW — vendored adi.hpp:564/596; our adapter refuses the
                   default). Header documents "construction is a physical action"; test pins
                   ctor-actuation + agreement pattern with PneumaticMechanism's declared safe state.
[00:22:00] DECIDE  Optical proximity polarity: vendored doc documents ONLY "range of 0 to 255" —
                   larger=closer is a BELIEF (community), registered + flagged weak, bench step
                   added. Sentinels: hue/sat/bri PROS_ERR_F, proximity PROS_ERR -> screen to
                   hold-last-good + faultedReads (IOptical has no validity channel).
[00:22:00] DECIDE  make cannot compile R1b adapters (main.cpp does not include them — L3 forbids
                   wiring it). The -iquote semantics get proven by a scratch ARM probe compiled
                   with common.mk's exact -iquote form (measured, not shipped), and the fence test
                   GAINS an explicit angle-bracket ban so the R1a lesson is pinned in-suite.
[00:22:30] START   Writing hal/digital_in.hpp (the new seam) + hal/fake/fake_digital_in.hpp.
[00:28:00] DONE    Seam + fake: hal/digital_in.hpp (IDigitalIn, one DOF, no debounce/edge/validity —
                   each ruling cited), hal/fake/fake_digital_in.hpp (level + readCount).
[00:28:00] DONE    Pure conversions: hal/distance_conversion.hpp (mm->in 1/25.4, conf /63 clamped,
                   named constants kDistanceNoObjectMm=9999 / kDistanceConfidenceAvailableAboveMm=200,
                   3-clause ADAPTER BINDING CONTRACT), hal/optical_conversion.hpp (hue clamp
                   [0,359.999], sat/bri clamp, proximity /255 — polarity flagged UNMEASURED).
[00:28:00] DONE    Shim extended BEFORE adapters (so it models the vendored doc, not the adapter):
                   pros/distance.hpp (ADVERSARIAL default 9999+conf63; below-200mm poisoned channel),
                   pros/optical.hpp (PROS_ERR_F/PROS_ERR sentinels), pros/adi.hpp (ctor ACTUATES with
                   PROS's real =LOW default kept, per-line write history, letters normalize to 1-8,
                   expander pairs, get_new_press REALLY consumes per line), misc.hpp + pros::usd::
                   is_installed() with ADVERSARIAL no-card default, shim_control resetAll() extended.
[00:28:30] START   Writing the five adapters under include/shulib/hal/pros/.
[00:40:00] DONE    Five adapters written: hal/pros/distance.hpp (the 9999 rule + close-range rule +
                   T7 screen with far-not-zero initial hold), optical.hpp (per-channel sentinel
                   screens, LED/gesture refusal documented), digital_out.hpp (REQUIRED initial
                   state, ctor-actuates documented, faultedWrites exposure), digital_in.hpp
                   (get_value LEVEL only, PROS_ERR-reads-as-pressed trap screened), block_sink.hpp
                   (usd_is_installed probe, injectable mountRoot, one prefix join, refusing sink).
[00:40:00] DONE    Six test files written (2 conversion + 4 adapter) — every case names the bug it
                   would catch; hand-computed literals throughout (200mm=7.874015748031496in,
                   9999mm=393.66141732283464in, 128/255=0.5019607843137255...). Fence test extended:
                   digital_in get_new_press textual ban + NEW angle-bracket <pros/> ban (R1a's
                   -iquote lesson pinned in-suite — R1b adapters are invisible to make, so this scan
                   is the only guard that fires before a robot build).
[00:40:00] FOUND   /dev/full verified usable in this environment as a REAL refusing device (large
                   fwrite -> short count; small fwrite buffers 'true' then fflush -> EOF) — gives
                   mutation 10 a real device refusal instead of a mocked one, and pins the honest
                   limit that a buffered write can report true with the refusal landing at flush().
[00:40:30] START   First build.
[00:24:16] DONE    First build+run GREEN: 1120 cases / 1,523,324 asserts / 3 skipped (was 1083/1,523,069/3 — +37 cases, +255 asserts).
[00:27:18] START   Mutation campaign: 13 scripted (M1..M12,M14) + M13 live guard proof. Runner is build-gated, restores from scratchpad copies, bumps mtime on restore, traps PIPE.
[00:27:37] MUTATE  M1 drop mm->inch scale in ProsDistance -> RED (observed: 4 cases / 7 asserts failed)
[00:27:49] MUTATE  M2 pass 9999 through with nonzero confidence (THE rule) -> RED (observed: 1 case / 1 assert — the T4 test, confidence read 1.0 for an empty intake)
[00:27:59] MUTATE  M3 drop the confidence /63 -> RED (observed: 3 cases / 4 asserts — conversion test + wired adapter test)
[00:28:09] MUTATE  M4 drop the proximity /255 -> RED (observed: 3 cases / 8 asserts)
[00:28:19] MUTATE  M5 adapter computes conversion, returns raw (the C5 D-5 / E1 wiring hole) -> RED (observed: 4 cases / 6 asserts)
[00:28:29] MUTATE  M6 remove PROS_ERR screen on ProsDigitalIn::state() -> RED (observed: 1 case / 3 asserts — the dead-port-reads-as-PRESSED trap fired: INT32_MAX != 0)
[00:28:41] MUTATE  M7 invert ProsDigitalOut::set() -> RED (observed: 2 cases / 4 asserts)
[00:28:51] MUTATE  M8 commanded() reports the world (only successful writes) -> RED (observed: 1 case / 1 assert — the refused-write divergence test)
[00:29:13] MUTATE  M9 bind IDigitalIn to get_new_press -> RED THREE WAYS (observed: 3 cases / 6 asserts): the two-consumer STARVATION test (telemetry.state() read false on a held-high line — real consume semantics, the behavioural catch the brief demanded), newPressCalls==3 vs 0, the ButtonEdge N-consumer test, plus the textual pin. A single-consumer test would have passed — confirmed by reading the failure set.
[00:29:24] MUTATE  M10 write() returns true when the device refused -> RED (observed: 1 case / 1 assert — the REAL refusing device /dev/full caught it; the no-card guard path kept its own tests green, so the /dev/full test is the load-bearing one)
[00:29:35] MUTATE  M11 remove the usd_is_installed() check -> RED (observed: 1 case / 4 asserts — no-card test: sink opened anyway, wrote 'successfully', created the file)
[00:29:45] MUTATE  M12 remove the #error from shim pros/adi.hpp -> RED (observed: 1 case / 2 asserts — the shim-guard scan: missing #ifndef AND missing #error, both named)
[00:30:02] MUTATE  M14 fence pop moved past adapter code in distance.hpp -> RED (observed: 1 case / 1 assert — 'the fence pop must come BEFORE any shulib code')
[01:04:00] MUTATE  M13 EXECUTED LIVE: planted include/shulib/localization/pros/__smuggled.hpp with
                   a "pros/misc.hpp" include. The widened --exclude-dir=pros form MISSED it (the
                   measured hole, re-demonstrated); the shipped path-anchored form CAUGHT it
                   (file:line named); the IN-SUITE fence-test mirror went RED on the same plant
                   (observed: 1 case / 1 assert of 402 in that case). Plant removed; guard and
                   suite verified clean after restore.
[01:04:00] DONE    CAMPAIGN COMPLETE: 14/14 executed, 14/14 observed at the mandated outcome —
                   13 RED via the build-gated runner + M13 live. ZERO unexplained greens. Every
                   restore came from a scratchpad pristine copy with mtime bumped.
[01:25:00] DONE    Documentation contract discharged: HA-113..122 registered (index rows + full
                   entries, status line 7 of 122, counts 78/41/2/1), runbook steps 16-20 appended
                   (renumbering rejected — register entries cite steps by number), changelog entry
                   (deliverable #7), three FAQ entries (393-inch / pneumatic-at-boot / SD-log),
                   guide ch13 (construction-actuates warning in the mechanism extension), guide
                   ch14 (edited with the most care — see FOUND below), roadmap (WS2 [~] extended
                   with cited evidence, you-are-here, R1b chunk entry, F14 row NOT FROZEN + the
                   open-question honesty), briefing regenerated (23/43, F14 row, R1b in the list).
[01:25:00] FOUND   STALE CLAIMS IN AN EARLIER CHUNK'S FILE, found by reading (trap 5): guide ch14
                   still said "No shulib code has ever controlled a motor or read a real sensor"
                   and "no adapter has ever touched a physical device ... until [the bench
                   session] runs" — both falsified by R1a's OWN bench session (2026-08-13, eight
                   motors commanded, sensors read; the changelog records it). Fixed in ch14 (Rule
                   4, the layer that owns it), reworded so the governing constraint gets SHARPER:
                   never closed a loop / never driven, platform layer bench-checked once.
[01:25:00] FOUND   Vendored adi.hpp documents the expander smart-port range inconsistently: the
                   ENXIO errno text says smart port 1-21 (adi.hpp:59-ish block) while the same
                   ctors' param docs say "from 1-22" (adi.hpp:92). Vendored source — not ours to
                   fix; bench uses 1-21 (every other device API's range); noted under HA-120.
[01:26:00] DONE    R1b-COMPLETED.md written FROM this log. Final verification sweep, all green:
                   suite 1120 / 1,523,324 / 3 skipped; GUARD1 PASS; GUARD2 PASS; ARM gate PASS
                   148 headers (was 139, +9 = exactly the new header count); make PASS (hot+cold
                   .bin present); -iquote scratch probe of all five adapters PASS (common.mk's
                   exact form); api_doc_tool self-test/coverage/fresh/examples/removability PASS;
                   briefing check PASS; staleness self-test + audit PASS.
[01:26:00] DONE    R1b COMPLETE. Not committed, not pushed (per instruction). Tree left dirty and
                   buildable. verify-r1b.sh never created or touched. 14/14 mutations observed at
                   mandated outcomes, zero unexplained greens. The library has still never driven
                   a robot — and after this chunk the docs say that MORE precisely, not less.
