Real lines from robot one's brain, 2026-11-19 (the coordinator's `pros terminal` capture of the Auton Tester), for
RMP1-K's run reader (`src/model/run_log.mjs`) and the helper's fake `pros terminal`. Verbatim bytes (the `\r` and NULs
pros prints kept); only the per-tick `[MOT]`/`[EST]`/`[IMU]` lines are thinned (every 40th kept) and the capture's own
`=== connect` marks left out.

- `robot_one_2026-11-19_run3.log`: the boot lines, then run 3: stopped at step 2 (driveTo, motion TIMEOUT), steps 3-23
  skipped, `1 of 23 steps done`, the END estimate. A build before API 2.11: no per-step `done in` lines.
- `robot_one_2026-11-19_run1_cut.log`: run 1, step 7 FAILED and carried on, then the USB connection broken mid-run
  (pros's own `Connection to /dev/ttyACM0 broken` lines): no end.
- `d4_lines.log`: NOT from the robot -- the two API 2.11 per-step lines the host suite's robot two prints for
  `r.set(claw["open"]); r.set(claw["closed"])` (the sensorless grab: `(UNCONFIRMED)`), behind a run's start line. Shared:
  `test/routine_step_line_test.cpp` checks the library prints exactly these, `run_log.test.mjs` reads them.
