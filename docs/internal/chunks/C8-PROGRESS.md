# C8 Progress — the user guide

00:02:24 START C8: reading the brief (C8-user-guide.md), README, master plan, and completed-chunk notes before writing anything.
00:02:32 DONE read brief + README. Brief demands: 9 parts + glossary in docs/guide/, compiled examples in test file, tutorial actually followed, no docs/internal links from guide.
00:02:46 START fan-out: two Explore agents (C1-C7 lessons for Part 9; diag output formats + fault codes for Part 6) while I read chassis/motion/units/tests myself.
00:06:04 DECIDE file map: README + 01..08, 10..15 (09 RESERVED for D1's recipe-API chapter — adding it later is adding a file, no renumbering). Part 2 gets five files (02-06): field-relative vs robot-relative owns ch. 02's second half.
00:06:04 START writing ch. 01 (orientation) + 02 (field/coordinates/pose/frames) from README + master plan section 2/7/14 facts.
00:07:48 DONE agent 1 (C1-C7 lessons): key corrections — register is 57 entries now (README's 49 is stale; date any number), never link docs/internal from guide (chunk ids OK as vocabulary), HA-19/HA-20/HA-01 are the teaching examples for blast radius.
00:07:48 DONE ch. 01 + 02 written (orientation; field/pose/frames — field-vs-robot-relative gets the worked flip example).
00:07:48 START ch. 03 (odometry + sensors) and 04 (drivetrains + kinematics).
00:08:56 DONE agent 2 (diag catalog): all 11 fault codes with values/strings/causes, byte-pinned session-header/tick/result/summary formats, 3 log gates, latch semantics, minimal wiring incl. RunReporter. Chapter 11 writes itself from this.
00:08:56 DONE ch. 03 (odometry/drift/sensors/fusion).
00:08:56 START ch. 04 (drivetrains) + 05 (PID/feedforward/settling) + 06 (failure modes).
00:14:53 START test/guide_examples_test.cpp — tutorial wiring (TermSink direct for header/results/summary; ForwardingSink variant for the full tick stream), plus the ch. 10 API idiom cases. Guide chapters will quote these bodies verbatim.
00:21:41 DONE test/guide_examples_test.cpp: 8 cases, 41 assertions, all green on first full run (suite total now 670 incl. 3 deliberate skips). Real transcripts captured for the chapter (SES header, result lines, summary; 08b tick stream; 08c ✗TIMEOUT).
00:21:41 START ch. 08 (tutorial) — quotes the test listings verbatim.
00:26:30 DONE ch. 08 (tutorial, quoting guide-08a/b/c verbatim + real transcripts), ch. 10 (API as prose, F6-not-frozen notice, quoting guide-10a..e), ch. 11 (diagnostics line-by-line, all 10 raisable fault codes with actions).
00:26:30 START ch. 12 (symptom-first troubleshooting) + 13 (extending) + 14 (can't-do-yet) + 15 (glossary) + guide README.
00:31:45 DONE ch. 12/13/14/15 + guide README (maintenance note: anti-rot rule, reserved ch. 9, update checklist). README now links the guide (replaced the 'guide is planned' bullet).
00:31:45 START verification: full suite, both CI guards, ARM cross-compile.
00:33:31 DONE verification: suite 667/915,605/3 skips GREEN (was 659/915,570 — my 8 cases/41 asserts; NOTE pre-existing baseline drifted -6 asserts with the new git hash, will record honestly). Both CI guards PASS. ARM gate PASS (103-line TU, all 102 headers; RESUMING's sed needed ^include/ for relative find paths).
00:33:31 START following the tutorial as written: fresh configure/build/run per ch. 7, then ch. 8 commands, then the type-it-yourself path with a real my_first_auton_test.cpp.
00:35:24 FOUND user committed eded9fd (docs/internal/guide-maintenance.md) mid-chunk — verbatim-example rule, link-don't-restate, no-internal-links all match what I built; ONE tension: its checklist says numbering contiguous vs the brief's reserved-09 slot for D1. Keeping the reserved slot (it implements the brief's add-a-file requirement + the doc's own prefer-inserting-at-a-gap rule); recording the tension in C8-COMPLETED.
00:38:12 DONE tutorial followed start to finish: ch. 7 commands verbatim (green), ch. 8 path 1 (all three guide-08 transcripts printed as documented), ch. 8 path 2 typed in as my_first_auton_test.cpp — compiled, PASSED, then deleted. FOUND+FIXED: build auto-discovers new test files (no reconfigure needed); corrected the claim in ch. 8 and ch. 12.
00:39:20 DONE link check: 164 links incl. anchors, all resolve. DONE removability: grep clean after rephrasing one textual mention in guide README; mv-aside pass shows all public links resolve with docs/internal absent.
00:42:27 DONE roadmap: C8 paragraph in you-are-here + M7 user-guide checkbox [x] with evidence and honest open-DoD note. DONE C8-COMPLETED.md (file map, example verification, decisions incl. the reserved-09 tension with guide-maintenance.md, deferrals to D1/D2/D3/G4/R3, all-six contract).
00:42:27 DONE C8. Final state: suite 667/915,611/3 GREEN · guards PASS · ARM gate PASS · 164 links resolve · removability grep clean + mv-aside clean · tutorial followed as written (both paths) · nothing committed, nothing pushed. Working tree: M README.md, M docs/roadmap.md, ?? docs/guide/ (15 files), ?? test/guide_examples_test.cpp, ?? C8-PROGRESS/COMPLETED.
