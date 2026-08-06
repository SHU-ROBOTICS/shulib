# A4 Progress Log — Hardware Assumptions Register + ARM CI Gate

Live log. Vocabulary: `START` / `DONE` / `MUTATE` / `DECIDE` / `BLOCKED` / `FOUND`.

---

10:11:31 START A4. Tree clean at 5e82549. First action: this file. Next: required reading (brief, A3/A2 completion records, build-order, RESUMING, ci.yml).
10:11:44 DONE Read brief + RESUMING.md. Next: A3/A2 completion records, build-order, ci.yml.
10:11:51 DONE Read A3-COMPLETED.md + A2-COMPLETED.md. A3 §8 = 25 seed entries. Next: build-order, roadmap, ci.yml.
10:12:04 DONE Required reading complete (brief, A3/A2 records, build-order, roadmap, ci.yml, RESUMING). START harvest: grep PROVISIONAL (A4) + assumption sources.
10:13:23 DONE Harvest pass 1: 33 in-include PROVISIONAL(A4) label sites across 8 headers + localizer bootSettleTime + conversion contracts + skipped GPS oracle. Reading scenario/pilons/motor headers next.
10:23:45 DONE Harvest complete. Sources: 33 PROVISIONAL(A4) label sites (8 headers), A3 §8's 25 claims, A2 §7 honest limits, gps/imu conversion binding contracts, tracking/pilons geometry, DrivePlantConfig placeholder gains, arc_step/pilons ~100Hz loop-rate assumption, skipped GPS field-cal oracle (gps_conversion_test.cpp:163), F5 on-V5 number-match, jitter schedule.
10:23:45 DECIDE Register = 49 entries grouped by OWNING CHUNK (R3 bindings/geometry 19, R4 characterization 25, R5 gains 3, R6 model adequacy 2) over grouping by sensor family — because the register IS R3's/R4's walk order; sensor-family grouping would make R3 skip around. Checkbox-per-entry format (literal checklist) + summary index table, over one wide table — blast-radius prose doesn't fit a 7-col row.
10:23:45 DECIDE Label format in headers: "PROVISIONAL (A4: HA-xx)" — keeps the "PROVISIONAL (A4" grep prefix stable AND makes the orphan check mechanical (any label without "HA-" = orphan).
10:23:45 START Header edits: cite register IDs at every label site + every unlabeled harvest source (conversions, geometry, gains, loop rate).
10:29:51 DONE Header/test edits: every PROVISIONAL(A4) label now cites its HA-ID; unlabeled harvest sources (conversions, geometry, gains, loop rate, driftHorizon, bootSettleTime, brownout premise, current-limit gap, no-fix pose) now carry citations too. START writing the register.
10:30:32 START Writing docs/planning/hardware-assumptions.md — 49 entries: R3 group HA-01..19, R4 group HA-20..44, R5 group HA-45..47, R6 group HA-48..49.
10:41:11 DONE Register written: 49 entries (33 invented / 13 reasoned / 2 measured-elsewhere / 1 mixed). Orphan check BOTH directions clean: 35 label sites all carry HA-IDs; 46 in-tree IDs all have entries; 3 entries (HA-18/33/47) exempt non-header sources, stated in-register. START suite re-run.
10:42:13 MUTATE ARM-gate proof: inject '#include <immintrin.h>' (x86-only) into sim/rng.hpp; expect host build GREEN (blind) + ARM gate RED.
10:42:31 MUTATE result: host build GREEN + ARM gate RED (fatal error at the injected line, exit 1) -> restored -> ARM CLEAN again + suite green. The gate provably catches what host CI cannot.
10:42:48 DONE Verification battery: suite 429/681086 green (3 skips), GUARD 1 PASS, GUARD 2 PASS, ARM 77-header compile CLEAN, gate mutation proven red+restored. START Phase A close-out: roadmap.md + build-order.md.
10:44:48 DONE roadmap.md (you-are-here -> Phase A complete/C1 next; A4 task item [x] with evidence; ARM-gate notes flipped; M0 CI item annotated) + build-order.md (current position; verified block; explicit no-Phase-B note under the phase table). START A4-COMPLETED.md.
10:47:41 DONE A4-COMPLETED.md written (incl. Phase A retrospective, 8-decision log, DoD all true). Final battery: suite 429/681086 green, GUARD1+GUARD2 PASS, ARM CLEAN 77 headers, orphans 0/0, mutation remnants 0, nothing committed (24 modified + 3 new files in working tree).
10:47:41 DONE A4 complete. Phase A closed. Next: C1 (IMotion + motion primitives). Tree left uncommitted for review, per the brief.
