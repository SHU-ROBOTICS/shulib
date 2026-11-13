# SAFE-SAVE — live progress log

<!-- release: between parts — Part A landed 2026-10-04; Part B1 was absorbed into CAL1 (complete); nothing is building (declared by the coordinator 2026-11-13 for the first release since August) -->

> Appended in real time by the executor. Brief:
> [`SAFE-SAVE-calibrate-save-and-teleop-label.md`](SAFE-SAVE-calibrate-save-and-teleop-label.md).
> **This run builds Part A ONLY** (the library: `hal::IFileStore`, `ProsFileStore`, `FakeFileStore`,
> `calibration/record_store.hpp` with `chooseEffective`, `loadWithBackup`, `safeReplace`,
> `saveRecordSafely`, `loadRecord`, `formatSaveReport`). NOT Part B1 (Calibrate's save in
> `src/programs/calibrate_program.cpp`) and NOT Part B2 (the boot's fallback, Teleop's words; waits
> for COMP-b). HA number: **HA-166 only**.
>
> Executor: **Claude Opus 5.5 (1M context)** (subagent of the coordinator's workflow run), working
> in the git worktree `.claude/worktrees/safesave-a` (branch `safesave-a`, from `9239538`).
> The executor does not commit, push, run `pros`, or touch the serial port.

---

## 0. Start (2026-09-29)

- Worktree clean at `9239538`. Read RESUMING.md's standing rules and the brief in full (1250 lines):
  the team lead's answers (every recommendation accepted, D1–D8), the re-check amendments (which
  override the body), D4 (Part A now; B1 and B2 later).
- Scratch and backups: `.claude/coordinator/scratch/safesave-a/` (never `/tmp`, never the repo root).


## 1. The brief's citations re-checked against the tree at `9239538` (2026-09-29)

Every file:line the brief names for Part A, opened and compared. **Deltas, and how this run adapts:**

| # | The brief says | The tree says | Adapted |
|---|---|---|---|
| 1 | `src/bench_r3a.cpp`, `src/calibrate_program.cpp`, `src/boot_source.*` | SRC1 moved them: `src/programs/bench_r3a.cpp`, `src/programs/calibrate_program.cpp`, `src/common/boot_source.{hpp,cpp}` | The platform scan's `fopen` allow-list names `src/programs/bench_r3a.cpp` (its SD probe, still at :611). Part A edits no `src/` file. |
| 2 | `boot_source.hpp` `RecordRead`/`readRecord` :215-255 | now :374-413 (T3 Part 2 and SRC1 moved it); `boot_source.cpp` `ProsBlockSource card{kRecordFileName}` still :38, `readRecord(card, card.path())` :39 | Read only (B2's file). `FileRead`/`RecordFileRead` carry opened/count/why/path so B2 can rebuild the three sentences. |
| 3 | robot two's fixture `test/boot_source_test.cpp` ~:111-148, namespace :79-318 | :112-146 (`kRecordLeft` … `robotTwoRecord()`), anonymous namespace :79-331 | COPIED into the new test with a citation (landmine 12). |
| 4 | the fixture's record "written 2026-09-24 17:42:16" | `Sep 24 2026 17:42:16` is the Calibrate BUILD stamp in the provenance, not the write time: `R3d-PROGRESS.md` :2457-2462 says the record carries no date and which run wrote it is not provable (the team lead's account: the 2026-09-25 evening run) | The copy's comment says so; no "written at" claim is repeated. |
| 5 | `codeOnly` `test/mechanism_table_test.cpp` ~:170-200 | :154-204; a second copy now exists in `test/teleop_claws_test.cpp` :826-886 (T3 Part 2 landed) | COPIED once more, citing :154-204 (landmine 12). |
| 6 | T3 Part 2 and G1 Part 2c-final "in flight" (§4) | both LANDED; the in-flight set today is the coordinator's list (comp-t, tank1b, DIAG1-B, L9 A1, H1 A1, R5a A, T4 2a) | §4's in-flight list is historical; Part A still edits no code file any of them owns, except the one comment sentence in `hal/pros/block_sink.hpp`, which **comp-t also edits** (an Append mode) — kept to the one sentence the brief names, logged for the merge. |
| 7 | `block_sink.hpp` ctor :79-93, `path_` :89, `"wb"` :91, dtor :95-111, `path()` :148-149, flush note :37-40, "one blackbox file per boot" :64-67 | identical | none |
| 8 | `block_source.hpp` ctor :71-85, `"rb"` :83, `readAll` :107-115, `path()` :121-122 | identical | none |
| 9 | `fake_block_sink.hpp` false flush keeps bytes :41-52 | :41-45 (`flush()` returns the flag; the bytes were kept by `write`) | none — `FakeFileStore`'s false flush keeps the bytes the same way. |
| 10 | `check.hpp` :9-21 | the policy text :9-21, `PreconditionError` :43-53 | none |
| 11 | `calibration_record.hpp` NO HARDWARE CLAIM :52-54, `kRecordFileName` :81-84, `kRecordBytes` :99-104 | identical | none |
| 12 | fence test :71-120 | :71-125 | none — the new adapter uses exactly the two-flag fence. |
| 13 | shim `misc.hpp` :110-114, :193-199 | `UsdState` :106-114, `usd::is_installed` :193-199 | none |
| 14 | `hal/pros/` holds block_sink/block_source beside 2c-final's `device_provider.hpp` | also `tick_pacer.hpp` (landed) | none |
| 15 | HA-166's index row "after the highest number present at merge" | the register holds HA-179/180 (TANK1) above 166, and DIAG1-B's brief (landmine 9) corrected the same rule to NUMERIC order | HA-166's row goes after HA-161's and before HA-179's; its section after COMP-a's HA-161 section and before TANK1's. |
| 16 | changelog "Added since 2.2" | the API is 2.3 (TANK1) | "Added since 2.3". |
| 17 | `SaveReport`'s outcome "Saved / NotSaved / Unconfirmed" | `calibration::SaveOutcome` already exists — the PANEL's (`calibrate_panel.hpp` :202, `NotAttempted`/`Saved`/`NotSaved`), in the same namespace | The store's enum is `calibration::SaveResult { Saved, NotSaved, Unconfirmed }`; B1 maps `SaveResult::Saved` → `SaveOutcome::Saved`. |
| 18 | `BackupLoad::bytes` and `accept` over `std::uint8_t` | every byte seam in the tree is `std::byte` (`IBlockSink`, `IBlockSource`, `encodeRecord`, `decodeRecord`, `crc32Ieee`) | `std::byte` throughout, so the record's `accept` is `decodeRecord` with no cast. R5a's brief names no element type. |
| 19 | HA-122's entry ~:964-985 (§3.1, for the two cross-references) | the entry starts at :984 at `9239538` (:991 after this part's +7 lines above it) and runs to ~:1009 | none — the two sentences went at the end of the entry, after its blast radius. *(Row added 2026-10-01: the review found it missing from this table.)* |
| 20 | the worksheet's interim lines: G.0.1 ~:717-719 and :849-850 (§3.0) | G.0.1 is :729, its copy-first sentence :732-735; "A run that says NOT SAVED may already have ERASED …" is :867-868 | none — read only; both stay until B1 (§7). *(Added 2026-10-01.)* |
| 21 | Bench Tests' SD probe `probeSdCard` `src/bench_r3a.cpp` :592-630 | `src/programs/bench_r3a.cpp`: its comment from :592, the function at :597, its `fopen` at :611 | none — HA-166's settle (b) names the function, not a line. *(Added 2026-10-01.)* |
| 22 | HANDOFF :558 (quoted in §5) | :558 at `9239538`; :561 after this part's own three added lines at :513 | none — the quote is the note's text, which is unchanged. *(Added 2026-10-01.)* |
| 23 | T4 Part 1a — `src/team/robot_two_driver.hpp`, `src/common/driver_controls.hpp` (named by the run's task, not by the brief) | both present at `9239538` | none needed: both are inside the platform scan's `src/**` scope, and the scan reads them clean (no banned call, no `fopen`, no banned header — re-checked 2026-10-01). Part A edits neither. *(Added 2026-10-01.)* |
| 24 | §1.A.4 step 1 "read P and K, both, always"; landmine 13's exclusion of every read fault followed by a crash | the review of 2026-10-01 measured a single misread of P's first read, then one failure, losing the record on robot two's own card shape | **Changed after that review:** P is read a SECOND time, before K, when its first read is not accepted; read faults followed by a crash are now enumerated, and only the misread the re-read cannot catch stays excluded (§12). |

Baseline at `9239538` in this worktree (briefing regenerated, configure, full build): **1725 test cases,
2,648,032 assertions, 0 failed, 3 skipped.**

## 2. The library, written (2026-09-29)

New headers (all PROS-free except the adapter):
- `include/shulib/hal/file_store.hpp` — `hal::IFileStore` (`cardPresent()`, `read()`, `replace()`),
  `StoreRead`, `StoreReplace`, `ReplaceStep { None, NoCard, Open, Write, Flush }`,
  `kStorePathBytes` = 96, `detail::copyStorePath`. No rename/remove/stat/read-write open, and the
  header says why (HA-166 (1)-(3)); not `noexcept` (a bare-name precondition must propagate).
- `include/shulib/hal/pros/file_store.hpp` — `hal::pros::ProsFileStore`: one `ProsBlockSource` or
  `ProsBlockSink` per call, each in its own scope, built with ONE argument unless a root was
  injected (so the adapters' own `"/usd/"` applies); `cardPresent()` = `usd::is_installed() != 0`
  behind exactly the two-flag fence. No `fopen`, no mode string, no `"/usd/"` of its own.
- `include/shulib/hal/fake/fake_file_store.hpp` — `hal::fake::FakeFileStore`: named files in
  memory; reported faults (open refused, short write keeping the prefix, false flush keeping the
  bytes, short read), silent faults (cut to j, one byte inverted, a read with one byte inverted),
  a crash at operation k leaving the file Absent / Empty / Prefix(j) / AllNew / Torn(j) with the
  card then GONE or SILENT (still reported present), `reboot()`, an operation log.
  **Delta — one more silent fault, `SilentNoop`** (a replace that reports `None` and leaves the
  file as it was): the brief's kill for mutation 11 ("a silent cut on step 2") cannot go red —
  `put` also requires `accept`, and a cut or flipped record never decodes, so a put without its
  byte comparison still fails there. A write that silently never landed leaves the OLD record, which
  DOES decode; that is the case the byte comparison exists for, and the order check (K written with
  the new bytes before P verifiably held them) is what kills it.
- `include/shulib/calibration/record_store.hpp` — `Which`, `chooseEffective`, `FileRead`,
  `BackupLoad<MaxBytes>`, `loadWithBackup<MaxBytes>`, `SaveResult`, `SaveRefusal`,
  `UnconfirmedWhy`, `FileHolds`, `PutRecord`, `SaveReport`, `safeReplace<MaxBytes>`,
  `acceptRecord`, `saveRecordSafely`, `RecordFileRead`, `LoadedRecord`, `loadRecord`,
  `BootReadsBackup`, `kSaveReportLineBytes` = 240, `formatSaveReport`.
  - `SaveReport` carries the four puts as `PutRecord`s (attempted / reported step / read-back
    opened / count / verified) instead of a separate failure list: the list is derivable, and a
    put's record is also what "whether a restore ran and read back" needs.
  - Stack, MEASURED with `arm-none-eabi-g++` (a scratch probe): ~~the save's state 452 bytes (the two
    212-byte arrays + 28), `SaveReport` 496~~ — **TRANSPOSED; corrected 2026-10-01 (§12, F12):** the
    save's state `SafeSave<180>` is **496** bytes (the two 212-byte arrays + 72) and `SaveReport` is
    **452**; `BackupLoad<180>` 820 and `LoadedRecord` 792 were right. The header now states 496 / 452.
  - `kSaveReportLineBytes` is 240 (Calibrate's `emitf` buffer), not 200: `-Wformat-truncation`
    counts a bounded `%.95s` at its full 95, and two lines need 228 with a 95-character path.
    Compiled clean in a scratch probe with the robot's flags (`-Wall -Wextra -Wformat=2 -Os`,
    arm-none-eabi-g++), `formatSaveReport` instantiated.
- `calibration_record.hpp`: `kRecordBackupFileName = "shulib_drivetrain.bak"` beside
  `kRecordFileName`, with `static_assert`s that both are bare and that they differ (two
  `constexpr` helpers in `detail`); the NO HARDWARE CLAIM paragraph replaced by the 2026-09-25
  evidence (eight Teleop starts decoded the record), cited by date. **Layout, `kRecordVersion` and
  the golden untouched.**

## 3. The tests (2026-09-29)

Four new test files, **51 test cases, 808 assertions, all green** on the first full run of the
subset after two TEST-side fixes (below):

- `test/record_store_test.cpp` — the thirteen starting cards of brief §2 with their LITERAL record
  in effect before (which file, which fixture); robot two's description COPIED from
  `test/boot_source_test.cpp` :112-146 with a citation (landmine 12), plus an "older" variant
  (track width 11.5) and the NEW record (ipr 1.3551/1.4760, W 12.412). The oracles use the test's
  OWN `decodeRecord` calls and its own P-else-K rule, never the store's `accept`.
  - **Crash-point: 21,824 runs** = 4 backup-first cards (3, 5, 6, 7) × (3 replaces × 361 leave-as
    states + 7 reads) × 2 variants + 9 cards × (2 × 361 + 6) × 2 — derived by hand, pinned.
    Every prefix and every torn split j = 1..179 is swept.
  - **One fault per run: 45,908 runs** — every replace: open refused, false flush, silent no-op,
    and a short write / silent cut / silent flip at every j = 0..179 (543 per replace); every read:
    open refused, and a short read / silent read-flip at every j up to the file's length (≥ 180).
    Derived by hand: 4 × 4156 + 8 × 3252 + 3268 (card 11's 188-byte P) — pinned.
  - **Silent write then crash: 38,180 runs** — 11 silent faults per replace (no-op; cut at 0, 1, 90,
    176, 179; flip at 0, 33, 90, 176, 179), then a crash at every later operation in EVERY leave-as
    state, both variants. The silent fault's own number is sampled (every value is swept in the
    one-fault test; the full product is ~2 million runs); the crash states are not. Derived by
    hand: 50 restore sites × 2 × 361 = 36,100, plus 2,080 crashes at later reads (872 + 152 +
    1,056) — pinned. **A READ fault followed by a crash is EXCLUDED**, as the brief's landmine 13
    says: it is D1's two-fault limit (a good record misread at step 1, then a failure); HA-139 is the
    belief it rests on. The exclusion is written in the test's header.
  - The ORDER checks, from the log, on every run of all three enumerations.
  - Directed: the eleven of brief §2 (card 3 AND card 5 for the cut-short step 2), plus two more:
    a write that silently never landed on step 2 (mutation 11's real kill — §2's delta), and a
    create that reports success and leaves NO file (the one silent write fault that ends
    Unconfirmed — pinned so it stays a stated exception; the invariant holds).
  - Edges: no card; an invalid description; bytes that do not decode (the generic twin of a failed
    encode, which is unreachable from a valid description — `encodeRecord` fails only on a short
    buffer, and the save's is exactly 180 bytes); K longer than P sharing its first 180 bytes; K
    differing only in a CRC byte; a re-save of the record already there; a boot where P decodes (no
    read of K); a fallback with P's reason in the decoder's words; both damaged; the boot's three
    primary-file facts; the preconditions (null, equal, too long, not bare) before the card is
    touched; the generic save/load serving a 40-byte sibling file (what R5a's gains file does).
  - Words: a golden for every branch under BOTH `BootReadsBackup` values (15 branches), the BACKUP
    branches' Yes/No split, the three refusals, and **103,680 synthetic measured combinations**
    (paths at their full 95 characters): each prints ≥ 1 line, ASCII, no `%`, < 240 bytes, and no
    line claims what its report does not say (SAVED / NOT SAVED / UNCONFIRMED, "still on the card",
    "boot from the BACKUP", "will REFUSE", "refuse to start until a run saves", "re-reads
    identical", "reads back whole", "safe in the backup", "no usable record file before", and an
    Unconfirmed save never saying which record boots).
- `test/fake_file_store_test.cpp` — the fake does exactly what its header says, byte for byte:
  every reported and silent fault, every crash leave-as state with both variants (a refused replace
  after the crash changes nothing, other files untouched, `reboot()` keeps the files), a crashed
  read, bare names. The crash oracles are only as good as this.
- `test/pros_file_store_adapter_test.cpp` — through the shim and a REAL temp directory: a longer
  file replaced leaves exactly 180 bytes; no card → `NoCard`, not opened, **the directory stays
  empty**; a card with a missing root → `Open`, not `NoCard`; a leading `/` throws from read and
  replace; NUL/LF/CR/0x1A/0xFF survive; an empty file opens and reads 0; the result path is
  `<tmp>/shulib_drivetrain.bin` with an injected root and **`/usd/shulib_drivetrain.bin` /
  `.bak` from a default-constructed store with the card absent**; and the whole
  `saveRecordSafely` → both files identical → torn record file → `loadRecord` from the backup,
  end to end on real files.
- `test/record_store_platform_scan_test.cpp` — over every `include/shulib/**/*.hpp` and
  `src/**/*.{hpp,cpp}` (code only: a COPY of `mechanism_table_test.cpp`'s `codeOnly` :154-204,
  cited; literals skipped; member calls ignored): no call to rename/remove/unlink/link/stat/mkdir;
  `fopen` only in the two adapters and `src/programs/bench_r3a.cpp`, **exactly 3 sites**; every mode
  a literal with no `+`; `hal/pros/file_store.hpp` spells no `"/usd/"`. A scanner self-test pins
  that the `motion_scheduler.hpp` trailing comment, a string literal, a member call and a longer
  identifier are NOT matched, and a qualified call and a `+` or non-literal mode ARE.

**Two test-side defects found on the first run, fixed** (neither in the library): the enumeration's
`has("SAVED --")` also matched "NOT SAVED --" (now: a line STARTING with it); the pairs count was a
guessed floor (now derived and pinned). Runtime of the four enumerations on the host's -O0 build:
crash 1.5 s, one-fault 3.8 s, pairs 2.7 s, words 0.6 s.

## 4. Documentation (2026-09-29)

- `docs/hardware-assumptions.md`:
  - **HA-166's index row**, in NUMERIC order after HA-161's and before HA-179's (delta 15 of §1).
    The brief's settle column read "S-session / Station G", which names nothing a reader can find;
    it now says which part settles what: Station G once B1 is in the build (a), Bench Tests' SD probe
    (b, unowned), a deliberate power cut (c, unowned).
  - **HA-166's entry**, in a new "SAFE-SAVE Part A (2026-09-29) added ONE" section after COMP-a's
    HA-161 section and before TANK1's: the five beliefs as §3.5 words them, the Source extended with
    the adapter, the fake (which ASSUMES (4) and (5)) and the platform scan, and the two-fault limit
    stated in the blast radius. No internal path is named (C7).
  - The status paragraph: `7 of 158` → `7 of 159`, one sentence for HA-166, and "HA-162..HA-178
    are not in it" → "HA-162..HA-165 and HA-167..HA-178". **At merge the count becomes the combined
    one** (DIAG1-B's HA-165 and the others land in the same wave).
  - HA-122's entry: the two cross-reference sentences (PROS syncs after every write it passes on —
    HA-166 (5); the save's end-of-save `usd_is_installed()` read, never confirmed for a pulled card,
    and why no false claim rests on it). No status change.
- `docs/faq.md`: "Can a program rename or delete a file on the SD card?", after the `/usd/` answer —
  no, on PROS 4.2.2, why a rename-based save passes on a laptop and fails on the brain, what the
  library does instead, "delete BOTH files to reset", and that **no program uses it yet** (so the
  copy-first rule still stands until B1). Evidence by date.
- `docs/changelog.md`: "Added since 2.3 (2026-09-29)", newest first under API 2.3; no version bump;
  says nothing has run on a robot and no program uses it yet, and repeats the copy-first rule.
- Plan docs, own lines only: HANDOFF L9b gains a status sentence; REMAINING-WORK row 28 gains
  "Part A BUILT, awaiting verification"; `build-order.md`'s SAFE-SAVE deviations row gains the same
  with a link to this log. `roadmap.md` untouched: no row or "You are here" names SAFE-SAVE, and
  nothing Part A adds makes a roadmap sentence false (checked: its SD-card lines, :284-287 and
  :1614-1620, are history).

## 5. Mutation campaign 1, then two changes, then campaign 2 (2026-09-29)

**Recipe** (HANDOFF §3's stale-binary recipe, scripted in the scratch dir
`.claude/coordinator/scratch/safesave-a/mutate.py`, not in the tree): back the file up under the
scratch dir, apply ONE exact textual edit (asserted to match exactly once), `touch` every test file
that includes the edited header, `cmake --build build/test --target shulib_tests/fast`, **require
`Linking CXX executable shulib_tests`**, run the Part A cases (`SAFE-SAVE*`, `FakeFileStore*`,
`ProsFileStore*`; plus `ProsBlockSink*`/`ProsBlockSource*` for 12a), record the red, restore from
the backup, bump the timestamps, relink, **require green**. The briefing is regenerated only after
the last restore.

**Campaign 1 (on the code as first written): 23 runs — mutations 1-21, 12a and 12b, plus 6b — all
RED, every restore relinked GREEN.** It exposed nothing in the library, and two things in the words
and the tests, fixed before campaign 2:
1. **The words named "shulib Teleop" (a delta from the brief, from a plan note).** HANDOFF :558 (the
   re-checked T4 plan, 2026-09-28): "For SAFE-SAVE A/B1's re-check: `formatSaveReport`'s sentences
   name Teleop, which T4 Part 3 retires." And COMP-b's competition program boots from the record
   too. So the words now say "the next boot" ("The next boot reads the record file.", "... uses it as
   before.", "The next boot uses the BACKUP copy (amber on the brain's screen).", "The drive program in
   this build does not read the backup: the next boot will REFUSE.", "Nothing usable ...: the next
   boot will REFUSE until a run saves.", Unconfirmed: "... start the drive program and read its
   'calibration record:' line."). "shulib Calibrate" stays named (it is the program that saves, and it
   stays). `BootReadsBackup`'s doc says "the boot of the drive program this Calibrate ships with
   (today "shulib Teleop")". The goldens and the enumeration's claim checks moved with it; the
   enumeration now also checks "will REFUSE" only on a measured refusal (none, or a backup this boot
   ignores) and "does not read the backup" only under `BootReadsBackup::No`.
2. **A test-usability defect:** doctest prints a `const char*` as an ADDRESS, so every enumeration
   failure named its card `0x5bd7a5647d20` (seen in campaign 1's red output). `StartCard::label` is
   now a `std::string`. Messages only; no check changed.
Two header comments also lost their references to the brief's decision numbers ("D8 (a)"), which
mean nothing to a reader of the library: they now cite Calibrate's ruling P5 in words.

Campaign 2 was STOPPED once to make fix 2 (it was on M01): the stop left M01's edit in
`record_store.hpp`, which was restored from the runner's backup (verified: no `MUTANT` string left in
`include/` or `test/`) before the label fix and the restart.

## 6. RESUMED (2026-09-29, 19:03 EDT by `date`) after the team lead's pause at the usage limit (~13:50)

A new executor run (Claude Opus 5.5 (1M context), the coordinator's workflow) picked this worktree up.
What it found, before touching anything:

- `git status`: the Part A files above, uncommitted, as §2-§4 list them; **`git diff -- src/ Makefile` is
  EMPTY** — no gate self-test plant and no mutation was left in `src/` or the `Makefile` by the killed run.
  No `MUTANT` string in `include/` or `test/` (grep, below).
- **Campaign 2 FINISHED before the pause** (its log in the scratch dir, 11:01-11:11): **all 23 runs
  (M01-M21, M06b, M12a, M12b) RED, every restore relinked GREEN.** §5 above stopped at the restart; its
  result was never written here. The per-mutation kill lines are in §7 (re-run on the final text, below).
- **Edits made AFTER campaign 2 and never logged** (read from the files' timestamps and a diff against
  the runner's backups):
  1. `test/record_store_test.cpp` (11:12): the crash-point test's failure message printed its
     `" GONE"`/`" SILENT"` literal as an ADDRESS (`j 00x5cc36ee80a29` in campaign 2's M21 output — the
     `const char*` doctest landmine of §5's fix 2, a second site). Wrapped in `std::string{}`. Message only.
  2. 13:50: `record_store.hpp`, `hal/file_store.hpp`, `hal/fake/fake_file_store.hpp` and
     `calibration_record.hpp` re-wrapped to 100 columns (comments, two precondition calls, one
     `static_assert` message split into two literals). Verified against the runner's backup of
     `record_store.hpp`: whitespace and literal splits only, no token changed.
  3. 13:51-13:52: `docs/api` regenerated, the full build and the full suite run — **1776 cases,
     2,648,863 assertions, 0 failed, 3 skipped** (1725 + 51 new), and `PROJECT-BRIEFING.md` regenerated.
- Because (1) and (2) came after campaign 2, and the task requires every mutation on a fresh binary of
  the code as it will be committed, **the campaign is re-run on the final text (campaign 3, §7)** once
  this run's review of the library is done.

## 7. The prose review, logged (pace rule 4; brief §3.1) — run 2026-09-29 evening, hits READ before this was written

**Changed source files:** `calibration/calibration_record.hpp` (the new constant, its two checks, the
hardware paragraph), `hal/pros/block_sink.hpp` (one comment sentence), and the four new headers
(`hal/file_store.hpp`, `hal/pros/file_store.hpp`, `hal/fake/fake_file_store.hpp`,
`calibration/record_store.hpp`).

**Greps run** over `README.md` and `docs/` (the generated `docs/api/` and the historical chunk logs set
aside; the worksheet grepped on its own): the brief's list — `shulib_drivetrain`, `NOT SAVED`, `SAVED`,
`still on the card`, `earlier record`, `rename`, `fsync`, `block_sink`, `block_source`, `IBlockSink`,
`IBlockSource` — plus `file_store`, `record_store`, `IFileStore`, `kRecordFileName`, `HA-166`,
`NO HARDWARE CLAIM`, `ProsBlockSink`, `physical SD`, `calibration record` (guide, cookbook, README).

**Every hit, and the verdict:**
- `docs/faq.md` :137-158 (this part's answer) — true, with ONE precision fix made now: it said `"wb"`
  empties an existing file at the open as if read from the libraries; the libraries show only that
  `"wb"` goes to VEXos's create call (HA-166 (1) says what VEXos does with an existing file is its own).
  Now: "goes to VEXos's create-a-file call, which is taken to empty an existing file at the open ...
  (reasoned from those libraries, not yet measured)". The `/usd/` prefix answer above it (:117-135) is
  still true (it names `ProsBlockSink`'s prefix and no-card behaviour, both unchanged).
- `docs/changelog.md` :19-69 (this part's entry) — true. `docs/hardware-assumptions.md` — HA-166's row,
  section and status sentence (this part's), HA-122's two cross-references (this part's), HA-122 (3)
  "no fsync in PROS's exposed surface" (still true — nothing is EXPOSED), and **HA-139** (:1426-1447):
  its settle and blast radius still describe today's save ("the Done path of one program"), which is
  TRUE until B1 changes the save — **owed to Part B1** (brief §3.2), not changed here.
- `docs/internal/HANDOFF-remaining-work.md` :15, :111 ("refuses without a SAVED calibration record"),
  :507-517 (the SAFE-SAVE paragraph, L9b) — true; :111's rewording is **B2's** (brief §3.3).
  `REMAINING-WORK.md` row 28, `build-order.md` :2044 — this part's status words, true.
- The worksheet (`chunks/R3a-BENCH-WORKSHEET.md`) :547, :733-734, :838, :867-868, :875-879, :981,
  :1562 — all true of TODAY's Calibrate (which still saves the old way): the copy-first rule (G.0.1,
  :733) and "a run that says NOT SAVED may already have ERASED the record" (:867) must STAY until B1.
  Station G's rewrite is **B1's**.
- `rename` elsewhere (`roadmap.md` :15/:18/:383, `vexbuilder-integration.md` :270/:462/:534, HANDOFF :18,
  `MECH-PANEL-requirements.md` :320/:609/:636, `docs-publishing.md` :132) — renames of fields, IDs,
  mechanisms or headers, not files on the card; `MECH-PANEL-requirements.md` :636 already says the brain
  cannot rename or delete. Nothing false.
- `block_sink` / `block_source` / `IBlockSink` / `IBlockSource` (`roadmap.md` :287/:562/:1060/:1614/
  :1620, `diagnostics-plan.md` :88, `PROJECT-BRIEFING.md` :701-716) — history of E1/R1b; still true
  (the store is a sibling seam; neither handle changed).
- `docs/guide/14-what-it-cannot-do-yet.md` :46, :409 — true ("the V5 card adapter ... HAS written a
  file to a real SD card at the bench (2026-08-18)").
- **`docs/guide/11-reading-the-diagnostics.md` :370-374 — FALSE, and NOT this part's to fix:** "the
  piece that writes to `/usd/` on a real brain (`ProsBlockSink` ...) ... none of it has ever written to a
  physical SD card." `ProsBlockSink` HAS: it wrote the calibration record Teleop decoded at eight starts
  on 2026-09-25 (`src/programs/calibrate_program.cpp` :641 constructs it), and chapter 14 :409 already
  says the adapter wrote a file on 2026-08-18. What is still true is that the BLACKBOX has never recorded
  a run on a robot. Predates Part A; guide chapter 11 is not one of Part A's documents (TANK1 Part B
  edited it upstream at `a5bd83f`). **Owed, for the coordinator** — a proposed wording: "What is still
  true is the limit that matters: the blackbox itself has never recorded a run on a robot (the adapter
  has written files to a real card — chapter 14)."
- `README.md` :245, guide 07 :145, guide 08 :315 (the calibration record) — true; nothing about the save.
- `roadmap.md`: no row and no "You are here" names SAFE-SAVE; its calibration-record lines (:246-253,
  :339-344, :391, :1779) are history and still true. Untouched.

## 8. Campaign 3 — every mutation re-run on the FINAL text (2026-09-29, 19:08-19:21 EDT by the result files' times)

Same recipe as §5 (`mutate3.py` in the scratch dir: `mutate.py` with fresh backup and result
directories, `backups-c3/` and `mutations-c3/`, so no backup from campaigns 1-2 could be restored over
the re-wrapped files). Before the run, every mutation's text was asserted to match EXACTLY ONCE in the
final files (the 13:50 re-wrap touched none of them). Each run: one edit, `touch` every test that
includes the edited header, `cmake --build build/test --target shulib_tests/fast`, **`Linking CXX
executable shulib_tests` required**, the Part A cases run (`SAFE-SAVE*,FakeFileStore*,ProsFileStore*`;
plus `ProsBlockSink*,ProsBlockSource*` for 12a), restore, timestamps bumped, relink required, GREEN
required. Green baseline of the filter: **51 cases, 808 assertions.**

**Result: 23 runs, 23 RED, every restore relinked GREEN** — and afterwards every mutated file is
byte-identical to its pre-campaign backup (`cmp`), and no `MUTANT` string is left in `include/`,
`test/` or `src/`. The FAILING cases below are the blocks that printed an `ERROR` (doctest also prints a
TEST CASE header for a MESSAGE, so the header list alone over-counts); the brief's named kill is marked ✓.

| # | Mutation (brief §2) | Observed RED (failed cases of 51) — the kill |
|---|---|---|
| 1 | record written before the backup | 13 — ✓ ORDER "P opened for writing before K verifiably held the old record" (crash, one-fault, pairs), the clean sequence, 3 directed, both K-rewrite edges, 2 goldens |
| 2 | step 1's put(K, old) without its read-back | 10 — ✓ ORDER "a replace of shulib_drivetrain.bak was not read back before anything else" (crash, one-fault, ✓ pairs) |
| 3 | K written in step 1 from an unaccepted P | 7 — ✓ ORDER "K written before P held the new bytes, in a save whose P did not decode" (card 8/9 in the crash test), card 2's directed, the BACKUP words |
| 4 | the restore dropped | 4 — ✓ directed :759 `rep.restore.attempted` (the boot reads K, not P), 2 goldens, the pairs count (1980 ≠ 38180). The invariant alone does NOT kill it (K holds the old record, so the reboot still loads it) — the directed test is the kill, as the brief says |
| 5 | step 3 dropped | 10 — ✓ adapter :200/:204 and clean save (K ≠ P after a Saved), directed "silent cut on step 3" (no such replace), the three pinned counts |
| 6 | chooseEffective prefers K | compile error — `test/record_store_test.cpp:518:47: error: static assertion failed` (a failed compile is the red) |
| 6b | the same, compile-time pins off | 15 — ✓ the literal `before` (crash "report.before is K, the literal is P"), chooseEffective's runtime check, clean save ×28 |
| 7 | loadWithBackup reads K when P accepted | 7 — ✓ edge :1060 `rec.backup.tried`, directed :767 `boot.readBackup`, the invariant "the boot read K although P decoded" |
| 8 | the step-1 skip compares length only | 8 — ✓ edge :1039 (K differing only in its CRC is rewritten), ORDER in the enumerations, a golden |
| 9 | accept is length-only | 8 — ✓ crash test "loadWithBackup disagrees with the test's own rule" (a torn 180-byte file accepted), the refusal edges :1011/:1073/:1094 |
| 10 | step 4 skipped, report from the steps | 13 — ✓ directed short read on step 4 (:922), ✓ silent cut on step 3 (:909, golden :1194), (u1)/(u2) directed, the enumerations |
| 11 | step 2's put without its byte comparison | 4 — ✓ directed "a write that SILENTLY never landed" :947 (ORDER: K written with the new bytes while P held the old), one-fault, pairs |
| 12a | `ProsBlockSink` "wb" → "ab" | 2 — ✓ `ProsFileStore: replacing a LONGER existing file` :79 (file size), and `ProsBlockSource`'s truncated-file test |
| 12b | an `fopen(…, "r+b")` in `ProsFileStore` | 2 — ✓ the scan on both counts: :247 (fopen outside the adapters + a '+' mode) and :258 (ProsFileStore calls no fopen) |
| 13 | a `std::rename` in `ProsFileStore` | 1 — ✓ the scan :247 |
| 14 | "still on the card" in a BACKUP branch | 2 — ✓ the BACKUP golden :1285, and the enumeration "'still on the card' without the measurement" |
| 15 | (u3) removed | 7 — ✓ card 5 stopping at step 3 while present (:839), the step-4 short read (:927), the enumerations |
| 16 | the restore even when P reads back as old | 5 — ✓ open-refused directed :785 (`primaryReplaces == 1`), ORDER "a restore although P already read back as old" |
| 17 | step 1 reads K only when P accepted | 9 — ✓ card 2's short write at step 2 (:886), ORDER "the save did not begin by reading P, then K" |
| 18 | put's success from replace's report | 7 — ✓ false-flush directed :801 (must be Saved), ORDER (K written although P never held the new bytes) |
| 19 | (u2) removed | 4 — ✓ card 5 stopping at the first read while present (:855), card 1's refused create (:873), the (u2) golden |
| 20 | a refused read-back after a write leaves "not opened" | 3 — ✓ exactly the brief's kill: card 1, card-present variant, crash at step 2's write leaving ALL NEW (`crash at op 2 leave 3 ... SILENT: report names none, the reboot loads P`) |
| 21 | (u1) removed | 4 — ✓ card 2's pull test :822 and its golden :1255, the crash test's (u1) rule |

Per-mutation JSON and full doctest output: `mutations-c3/` in the scratch dir; the runner's log
`campaign3.log` (its SUMMARY: 23 × "RED restored green", exit 0).

## 9. Every gate, on the final tree (2026-09-29, after campaign 3; final lines verbatim)

- Briefing regenerated before the build (`briefing status: regenerated`); `cmake -S test -B build/test`
  exit 0; `cmake --build build/test -j8` exit 0, its doc-gate target printing every line below; the test
  binary is newer than every file under `include/`, `test/` and `src/` (`find -newer`: none).
- **Suite:** `[doctest] test cases: 1776 | 1776 passed | 0 failed | 3 skipped` /
  `[doctest] assertions: 2648863 | 2648863 passed | 0 failed |` (baseline at `9239538`: 1725 cases,
  2,648,032 assertions). **The +831 assertions, per test file** (doctest's XML reporter, the SAME parser
  over the baseline binary built at `9239538` and the final one): Part A's four new files 808
  (`record_store_test` 305, `record_store_platform_scan_test` 246, `fake_file_store_test` 218,
  `pros_file_store_adapter_test` 39), and `pros_adapter_fence_test` 830 → 853 (+23: the two-flag fence now
  also checks `hal/pros/file_store.hpp`); every other file's count is unchanged. Part A's own: 51 cases;
  the pinned enumerations print `crash-point runs: 21824`, `one-fault runs: 45908`,
  `silent-write-then-crash runs: 38180`, `formatter combinations: 103680`.
- **`src` build gate:** `src build gate self-test: OK (189 detector cases, each against a real build)`;
  `src build gate: PASS (all 6 builds link; policy: errors fail, vendor warnings path-ignored, no category
  allowed, every other warning fails; ...)` — each of bench / xdrive / tank / tank-drive / tank-library /
  tank-calibrate "exit 0, 8/8 TUs compiled, warnings: 6 vendor-ignored, 0 disallowed". Afterwards
  `git diff -- src/ Makefile` is EMPTY (no self-test plant left behind).
- **ARM header gate** (every `include/shulib` header, 196, `-Wall -Wextra -Wconversion -Wsign-conversion
  -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp`; TU in the scratch dir):
  `ARM compile gate: CLEAN (all v2 headers)` — re-run after the `src` gate finished, because its
  self-test plants a fault in a library header while it runs.
- **ARM instantiation probe** (scratch; the robot's flags `-mfloat-abi=hard -mthumb -Os -Wall -Wextra
  -Wformat=2 -Wno-psabi` plus the strict set and `-Werror`): `saveRecordSafely` and `loadRecord` over a
  default-constructed `ProsFileStore`, `formatSaveReport` under both `BootReadsBackup` values through a
  240-byte `emitf("%s")` — CLEAN. The stack figures the header states, re-measured on the final text with
  `arm-none-eabi-g++`: ~~`SafeSave<180>` 452, `SaveReport` 496~~ (TRANSPOSED — the probe prints them as
  `Size<452> a` and `Size<496> d`, with `a` = `SaveReport` and `d` = `SafeSave`; corrected 2026-10-01,
  §12: `SafeSave<180>` **496**, `SaveReport` **452**), `LoadedRecord` 792, `BackupLoad<180>` 820.
- **CI greps** (GNU grep, as CI runs them — this shell's `grep` is a ugrep wrapper): `library is PROS-free
  (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- **Doc gates:** `api_doc_tool self-test: OK`; `doc coverage: 3056 public entities across 159 headers, all
  documented`; `check-fresh` exit 0 (silent: nothing stale); `doc example scan: 467 quoted lines, all
  verbatim (5 source files); 3 listings of whole-run files, each one unbroken run`; `removability: no public
  doc references docs/internal/`; `doc staleness audit self-test: OK (10 detector/filter cases)`; `doc
  staleness audit: clean (27 live docs, 0 numeric claims checked) ...`.
- `briefing_status.py generate` then `check`: §11.

## 10. For the coordinator — the merge (checked READ-ONLY against the other worktrees, 2026-09-29 evening)

> **SUPERSEDED IN PART on 2026-10-01 — read §12.4 first.** Item 1's patch is OBSOLETE (do NOT apply
> it: the scan now carries COMP-t's forwarding opener itself and passes on `shulib-v2` with no patch);
> items 3-5 are re-checked against `shulib-v2` at `7daf809` there (COMP-t, L9 A1, DIAG1-B, H1 A1 and
> T4 Part 2a have LANDED; API 2.4 is out); item 6 gains four owed entries.

1. **COMP-t turns Part A's platform scan RED at merge — a verified patch is ready.** COMP-t's
   `hal/pros/block_sink.hpp` (its commit `393fbd8`, not yet on `shulib-v2`) adds an Append mode whose
   forwarding opener is `[](const char* p, const char* m) { return std::fopen(p, m); }`. Part A's scan
   (`test/record_store_platform_scan_test.cpp`) requires every `fopen` mode to be a string literal and pins
   3 `fopen` sites, so on the merged tree it fails: `an fopen() mode that is not a string literal: m` and
   `fopenSites == 3` (4). **Observed**, on a scratch approximation of the merge (this tree's `include/` and
   `src/` with COMP-t's 21 changed files from `393fbd8` laid over). The design is sound — every mode COMP-t
   hands the opener is a literal (`"rb"`, `"ab"`, `"wb"`) with no `+` — so the SCAN adapts, not COMP-t:
   `scan-merge/record_store_platform_scan_test.comp-t-merge.patch` in the scratch dir (83 lines;
   `git apply --check` clean on this worktree's file) allows a non-literal mode ONLY as that exact pinned
   opener text in `block_sink.hpp`, only once, and then requires every `open(path, "<mode>")` in that file
   to be a `+`-free literal; four scanner self-test lines; the count becomes 4. Verified in scratch: on the
   merged approximation, **all 51 Part A cases pass (820 assertions)** under the host suite's strict flags;
   the patched scan goes RED on each of four mutations of the merged sink (`"ab"` → `"a+b"` handed to the
   opener; a second non-literal `fopen`; the constructor's `"wb"` → `"w+b"`; a variable mode handed to the
   opener) and GREEN on restore; on THIS tree it differs only in the count (3 ≠ 4). Apply it in whichever
   merge brings the two together. Part A's `M12a` mutation text (`file_ = std::fopen(path_.c_str(), "wb");`)
   still matches COMP-t's sink.
2. **`hal/pros/block_sink.hpp` — the one file Part A shares with COMP-t:** Part A's edit is one comment
   sentence (:40-42, the `flush()` paragraph); COMP-t leaves that paragraph as it was and adds its APPEND
   section after OWNERSHIP, so the hunks are three lines apart. Regenerate `docs/api/pros-block_sink.md`.
3. **`calibration/calibration_record.hpp` is ALSO edited by H1 Part A1** (read in `h1-a1`, uncommitted):
   the CRC comment (:36-42), a `#include "shulib/diag/crc32.hpp"`, and `crc32Ieee`'s body, which becomes a
   forwarder to `diag::crc32Ieee`. Part A's hunks are the hardware paragraph (:49-56) and the new constant
   with its checks (:85-126) — apart from H1's. `record_store.hpp` calls `crc32Ieee` unqualified from
   inside `shulib::calibration`, which still finds the kept forwarder. Regenerate `docs/api/calibration_record.md`.
4. The ported scan (scratch `scan_port.py`) over the other trees: `diag1b`, `l9-a1`, `h1-a1`, `r5a-a`,
   `t4-2a` and `shulib-v2`'s `a5bd83f` — 0 problems, 3 sites each. Only COMP-t trips it (1 above).
5. **Shared documents:** `hardware-assumptions.md` — the status count becomes the combined one, HA-166's row
   stays in NUMERIC order, and the sentence "HA-162..HA-165 and HA-167..HA-178 are not in it" must be
   recomputed (COMP-t brings HA-177, DIAG1-B HA-165, others theirs). `changelog.md` — newest first; **if L9
   Part A1's API 2.4 lands first, this entry becomes "Added since 2.4" under "## API 2.4"**; TANK1 Part B
   already edited the API 2.3 section (`a5bd83f`), three lines from this entry's insertion. `faq.md` —
   COMP-t edits it too. `docs/api/*`, `mkdocs.yml`, `PROJECT-BRIEFING.md` — regenerate after the merge.
   `HANDOFF` :507-517, `REMAINING-WORK` row 28, `build-order.md` :2044 — own lines only; `a5bd83f`'s hunks
   in those files are elsewhere.
6. **Owed, not Part A's:** guide chapter 11 :370-374's false "none of it has ever written to a physical SD
   card" (§7, with a proposed wording); HA-139's settle and blast radius (B1, brief §3.2); HANDOFF :111
   (B2, brief §3.3); the worksheet's Station G (B1). COMP-t's diff also edits this chunk's BRIEF
   (`chunks/SAFE-SAVE-calibrate-save-and-teleop-label.md`); Part A did not touch the brief.
7. Status pointers: this part closes no chunk (B1 and B2 remain), so no `-COMPLETED.md`, and neither
   `build-order.md`'s `**Next:**` block nor the roadmap's "You are here" names SAFE-SAVE; the
   `PROJECT-BRIEFING` lists SAFE-SAVE among the open chunks, which is true until B2.

## 11. Close of this run (2026-09-29 evening) — Part A's DoD, measured

- `briefing_status.py generate` (after the suite binary existed): `briefing status: regenerated`; then
  `check`: `briefing status: generated block current (the durable narrative is NOT gated — see
  doc_staleness_audit.py)`. The briefing now reads 1,776 cases / 2,648,863 assertions, 196 public headers,
  159 hardware assumptions, and lists SAFE-SAVE among the open chunks (true until B2).
- [x] every §2 Part A test green, the enumeration counts logged (§3, §9);
- [x] all 21 mutations — 23 runs with 6b, 12a and 12b — observed RED on fresh binaries and logged (§8;
      campaign 3 on the final text, after campaigns 1 and 2 on earlier text, §5-§6);
- [x] no `src/` file changed (`git diff -- src/ Makefile` empty; `git status` lists nothing under `src/`);
- [x] HA-166 in, with all five beliefs;
- [x] the golden unchanged (`test/calibration_record_test.cpp` untouched; the record's layout, version and
      encoder untouched — the header's diff is the hardware paragraph and the new constant only);
- [x] docs per §3.1, and the prose review logged (§7).

Nothing committed, nothing pushed, no `pros` command, no serial port. **Part A is BUILT and gated, awaiting
the coordinator's verification; B1 and B2 remain.** Scratch (probes, runners, campaign results, the
merge patch): `.claude/coordinator/scratch/safesave-a/`.

## 12. SAFE-SAVE-A review findings and their disposition (2026-10-01, from ~19:50 EDT by `date`)

Executor: Claude Opus 5.5 (1M context), the coordinator's workflow, the builder again, same worktree
(still nothing committed). Three reviewers (correctness, tests, docs) returned 17 findings, with probes
and mutants in the scratch dir (`review-correctness/`, `review-tgs-r5/`, `review-docs/`). Overlaps
deduped: **F4 ≡ F13** (the FAQ's and the headers' unconditional guarantee); F1 and F2 share the
two-fault limit; F11 and F14 share the FAQ/`file_store.hpp` wording. Every finding was RE-CHECKED here
before acting (code read, a measurement, or a mutant re-run) — none taken from the report. Scratch for
this round: `.claude/coordinator/scratch/safesave-a/review-fix-2026-10-01/`. `shulib-v2` is at `7daf809`
(T4 Part 2a landed during this run).

### 12.1 Each finding: re-check, then disposition

**F1 (correctness, minor) — the two-fault case on robot two's own card shape. REAL; FIXED AT THE ROOT.**
Re-check: with ONE refused or short first read of a good P, `before` came out Backup/None, so step 1
backed nothing up, step 2 truncated P, and with `before != Primary` there was no restore — one failed
write or power cut then left nothing (card 3) or the older K (card 5). The save's own "last seen" rule
already says a refused read is not a measurement; step 1 did not apply it. **Fix:** the save reads P a
SECOND time, before K, when its first read is not accepted (`record_store.hpp` step 1, and the header's
step 1 says why). Tests: the new "A MISREAD THEN A CRASH" enumeration replaces the blanket exclusion of
read-fault-then-crash with the measured boundary (14,870 runs, derived in §12.2); a directed test (cards 3
and 5: the first read refused or short, then step 2's write cut short → re-read, backed up, restored,
the reboot loads robot two's record); and "THE STATED LIMIT, measured" (both P reads refused → card 3
loses the record, card 5 boots the older backup — and no line says "still on the card" or "no usable
record file"). The clean-sequence strings gained `rP rP rK …` for the eight cards whose P does not
decode; the order oracle requires the re-read exactly when P's first read did not decode; the
`before`-literal check is now skipped only when a fault touched a read step 1 DECIDED from (its last P
read, or K's), read from the run's own log — so a first-read fault the re-read saw past must still name
the literal record in effect before (a stronger oracle than "any fault at op 0 or 1"). Mutant R01 (no
re-read) red (§12.3). **Not changed:** `loadWithBackup` — it destroys nothing after its read, so a
misread at boot costs one boot from the backup (amber, B2) or one refusal, never the record. **The
reviewer's B1 suggestion** (keep the copy-first rule until a SAVED run has written a verified `.bak`) is a
change to B1's plan, so it is PROPOSED in §12.4, not written into the brief (pace rule 2).

**F2 (correctness, minor) — the stated limit was narrower than the measured one; "it never drives wrong".
REAL; FIXED.** The shapes the reviewer measured — a damaged K misread as identical to P (b′), a damaged P
misread as a VALID record (c) — need a misread that produces a valid record (a stale read); the re-read
cannot catch those, nor P misread at both of its reads. THE LIMIT now names all three, in
`record_store.hpp` (THE GUARANTEE / THE LIMIT), HA-166's blast radius and (regenerated) `docs/api/
record_store.md`. "It never drives wrong" → "the boot never drives from a DAMAGED record; it may drive
from the record in effect before the save, or an older backup copy (amber BACKUP once B2 lands)". **And
the words:** "There was no usable record file before this run, and an older backup copy exists in K"
claimed what step 1 could not measure (in the limit it is false). Now: "The record file did not read as
usable when this run began; the backup copy K holds an earlier record." — pinned by the card-2 directed
test, the BACKUP-branch test, the limit test and the enumeration (which also fails any "no usable record
file" text). **Delta from the brief's §1.A.5 wording**, logged here; mutant R10 (the old sentence) red.

**F3 (correctness, minor) — two names that are one file on a FAT card. REAL; FIXED.** `hal::sameFileOnCard()`
(constexpr: ASCII letters fold — only `A`-`Z`, so `@`/`` ` `` and `[`/`{` stay apart — and trailing dots
and spaces drop) in the store seam; `safeReplace`'s and `loadWithBackup`'s preconditions and
`calibration_record.hpp`'s compile-time check on the two record names use it (`detail::fileNamesDiffer`
removed: one rule, one place — `calibration_record.hpp` now includes `hal/file_store.hpp`; a trial
three-way merge with H1 A1's include line on `shulib-v2` is clean, §12.4). HA-166 gains **belief (6)** —
FAT's rule reasoned, its VEXos half invented; a settle step added to (b); blast radius: if wrong, a few
name pairs are refused needlessly. The number stays HA-166 (beliefs (1)-(5) keep their numbers — R5a's
brief cites them by number). R5a's names (`shulib_characterization.bin`/`.bak`) are distinct under the
rule. Mutants R05-R09 red (R07-R09 as compile errors: the `static_assert`s).

**F4 ≡ F13 (correctness minor / docs major) — the guarantee stated without its conditions. REAL; FIXED.**
The FAQ now says the save is DESIGNED so…, that it rests on two beliefs no robot has tested (HA-166 (4)
and (5), each said in a few words), and that it does not cover two faults beginning with a misread of
its first reads. Also qualified: `kRecordBackupFileName`'s doc (`calibration_record.hpp`), the first
sentence of `record_store.hpp` (it is `docs/api/README.md`'s row — regenerated; now "given HA-166 (4) and
(5), short of a stated two-fault limit"), `safeReplace`'s doc, and the test file's header. **Left, with
the verdict:** REMAINING-WORK row 28, HANDOFF L9b and the brief's title say "can never erase a good
record" — the chunk's GOAL in plan documents, not a claim about built behaviour (the built library's
claim is the qualified one above).

**F5 (correctness, minor) — a store's read count trusted past the buffer. REAL; FIXED.** Re-check:
`readFile` passed `r.count` raw to the CRC, `accept` and (through `keepAsOld`) a `memcpy`; `loadOne` to
`accept`. Fix: one helper, `detail::countRead(r, capacity)`, clamps to the buffer, used at both entry
points; the header's Contract says so. A clamp, not a precondition: a throw in the middle of a save could
leave P truncated with no restore, while a clamped count keeps the save's own safety logic in charge (it
becomes a length refusal or a read-back mismatch). Test: an `OvercountingStore` (+100 bytes on every read
that opens) — `accept` never sees more than 212 bytes, in the load and in the save. Mutants R02, R03, R04
red.

**F6 (tests, major) — the (u2)/(u3) reason was never checked on crash and pair runs. REAL; FIXED.** A
directed test (card 1, the store stops answering right after the record file's create got past its open
with every byte landed → (u3) naming the record file, never (u2)'s "nothing could be opened or
created"; the reboot loads the new record), and the oracle in `violation()`: on every Unconfirmed run,
the reason must be (u1) when the card is gone, else (u3) with a file named when the log shows anything
opened (`FakeFileLogEntry::opened` — a read that opened or a replace past its open), else (u2) — over
all five enumerations. Review mutants N01 and N02 red.

**F7 (tests, minor) — "a golden for every branch" was overstated. REAL; FIXED.** New goldens from real
saves: (u3) naming the BACKUP alone (card 5, a short read on the last operation), "it would not re-open
for the read-back" (card 3, step 2's read-back refused), "it read back different bytes" (card 3, a silent
flip on step 2), and a reported step-1 failure whose file step 3 later overwrote (card 5, a false flush
on the backup copy — no "reads back whole"). New goldens from SYNTHETIC reports for the three branches no
save reaches: the TOTAL line (once for each file), "does not hold the new record where the next boot
reads it", "the card was gone". The formatter enumeration now varies the primary's and the backup's
disagreement independently, with DIFFERENT 95-character paths, and checks that the (u3) line names
exactly the disagreeing file(s): 103,680 → 145,152 combinations. The goldens name operations by step
(`replaceOp`, `lastOp`), no longer by literal index (the re-read moved them on cards 1 and 2). The test
header now says how each branch is reached. Review mutants N03 (through the card-8 two-fault test: K's
step-1 read refused, then step 2's open refused → (u3) naming the backup only), N04, N05, N06, N06b, N18
red.

**F8 (tests, minor) — ProsFileStore's Write and Flush steps never exercised. REAL; FIXED.** An adapter test
against `/dev/full` through the real shim and adapters: 180 bytes → `Flush`; 64 KiB → `Write`; the path
is `/dev/full`. On a host without `/dev/full` it skips with a message. Review mutant N08 red.

**F9 (tests, minor) — the scan's banned list unpinned; a read-write `freopen` passed. REAL; FIXED.** The
scan now also bans `freopen`, `fdopen`, `openat`, `creat`, and an `#include` of `<fcntl.h>`,
`<sys/fcntl.h>`, `<fstream>` or `<filesystem>`. POSIX `open()` is caught by its header, not its name: this
scanner cannot tell a call from a member DECLARED `open(` (`diag/sd_sink.hpp` :180 has one). A new
self-test types every banned name, open and header from its OWN list (not the scan's tables), and checks
the bare, qualified, member and in-literal forms. The tree still scans clean (and so does `shulib-v2`).
Review mutants N09, N10 and this round's R11, R12, R14, R15 red.

**F10 (tests, minor) — the COMP-t merge patch could go vacuous. REAL; FIXED DIFFERENTLY, AT THE ROOT.** COMP-t
has LANDED on `shulib-v2` (`0f56301`), so the allowance moved INTO the scan and the merge needs no patch:
the pinned opener text, once, in `block_sink.hpp` only; then EXACTLY three `open(` calls, each a `+`-free
literal (a renamed parameter leaves zero → refused); `fopenSites == 3 + openers`. (A separate pin on the
text `Open&& open` was written and then REMOVED before the campaign: the count already refuses every
rename, so no mutant could ever be killed by the pin alone.) Self-test: the landed shape passes; `a+b`
handed, a variable handed, the plain open's `w+b`, a second variable `fopen`, the calls renamed (alone and
with the declaration), a fourth mode, and the opener in another file are each refused. **Verified on a
scratch merge** (`shulib-v2` `7daf809`'s `include/` and `src/` plus Part A's four new headers): the scan
passes (5 cases, 322 assertions, on the final scan text), five mutations of the LANDED sink each go RED (`"ab"`→`"a+b"`, the
parameter renamed, the constructor's `"w+b"`, a second variable `fopen`, a variable handed to the opener),
and the restored file is byte-identical. R13 red.

**F11 (docs, major) — "read and replace, and that is all" vs the append open. REAL; FIXED.** The brief's own
§0.2 (:342) records that an `O_APPEND` open goes to `vexFileOpenWrite`. Now: HA-166's row ("no rename, no
delete, no stat and no read-write open — the safe save is built from a whole-file read and a truncating
replace … an append open exists too, and cannot replace a record") and belief (1)'s parenthetical; the
FAQ (read, replace, append; cannot rename, delete, stat, or open read-write); `hal/file_store.hpp`'s and
`hal/pros/file_store.hpp`'s headers. HA-177 is not in this tree; the cross-reference is a merge note
(§12.4).

**F12 (docs, major) — the measured stack sizes were transposed. REAL; FIXED.** Measured again here:
`arm-none-eabi-g++ -std=gnu++20 -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -mthumb
-fsyntax-only arm_sizes.cpp` prints `Size<452> a`, `Size<820> b`, `Size<792> c`, `Size<496> d`, where
`a` = `SaveReport` and `d` = `SafeSave<180, …>`. **How it happened:** the probe labels the four sizes a-d,
and §2 and §9 mapped `a` and `d` the wrong way round; the header copied §2. Fixed in the header
("424 bytes for the record, 496 with the save's other state … a SaveReport (452 bytes)"), the API page,
and §2/§9 (struck through, corrected in place). Re-measured on the final text in §12.3.

**F14 (docs, minor) — "wb empties at the open" stated as read from the libraries. REAL; FIXED.** The
changelog, `hal/file_store.hpp` (its header, its "What replace() promises" bullet and `replace()`'s doc:
"to be treated as gone"), HA-166's section intro and `record_store.hpp`'s defect paragraph now all say
"reaches VEXos's create call, which is taken to empty an existing file at the open (reasoned, not
measured)".

**F15 (docs, minor) — the prose review missed README. REAL; FIXED.** README :87-89 now names the store built
over the two SD adapters, "not counted as an adapter of its own" — so "Seventeen" stays true of the list
it counts (17 named adapters). Whether `device_provider.hpp` should be counted predates Part A and is
not changed here. The layout block (:270-271) now says "its SD-card record and its safe save and load (a
verified backup copy)".

**F16 (docs, minor) — the citation table omitted moved rows. REAL; FIXED:** rows 19-23 added to §1 (HA-122's
entry, the worksheet's two interim lines, `probeSdCard`, the HANDOFF note, T4 Part 1a's two `src/`
files), each re-checked here, and row 24 for this round's design change.

**F17 (docs, minor) — §10's merge notes stale against `shulib-v2`. REAL; FIXED:** §10 is marked superseded in
part and §12.4 re-checks every item against `7daf809`.

### 12.2 The pinned counts, derived by hand BEFORE the run (each then matched the run exactly)

The re-read adds one READ (index 1) to the clean log of the eight cards whose P does not decode (1, 2,
8-13): `rP rP rK wP rP wK rK rP rK`. Cards 3, 5, 6, 7 keep `rP rK wK rK wP rP wK rK rP rK`; card 4 keeps
`rP rK wP rP wK rK rP rK`.

- **Crash-point: 21,824 → 21,840.** 30 replace indices (4 × 3 + 9 × 2) × 361 × 2 = 21,660, plus 90 read
  indices (4 × 7 + card 4's 6 + 8 × 7) × 2 = 180. (Before: 82 reads, 164.)
- **One fault per run: 45,908 → 48,812.** The re-read on the eight cards: refused, plus a short read and a
  silent read-flip at every j below max(P's length, 180) — 7 × 361 + card 11's 188-byte P, 377 = 2,904.
- **Silent write then crash: 38,180, unchanged** — the re-read precedes every replace.
- **A misread then a crash: 14,870 (new).** Five misreads per read index (refused; a short read of 0 and of
  100 bytes; byte 0 and byte 90 altered); per later operation of the FAULTED run, a read crashed 2 ways and
  a replace 2 × 15 (absent, empty, all new; prefix and torn at 1, 33, 34, 90, 176, 179). The faulted runs:
  - Backup-first cards (each): index 0 (any misread → the re-read, then the full sequence: 7 later reads,
    3 replaces → 104) × 5 = 520; index 1, K (any misread leaves K ≠ P, as it already was → 5 reads, 3
    replaces → 100) × 5 = 500; index 3, step 1's read-back (the copy fails, stop → 2 step-4 reads → 4) ×
    5 = 20; index 5, step 2's read-back (fails → restore: 1 replace, 3 reads → 36) × 5 = 180; index 7, step
    3's read-back (4) × 5 = 20; index 8 (2) × 5 = 10; index 9 (0). **1,250.**
  - Card 4: index 0 → 9 ops, 6 later reads + 2 replaces = 72 × 5 = 360; index 1 (K misread ≠ P → the
    backup copy now runs: 5 reads, 3 replaces → 100) × 5 = 500; index 3 → restore (36) × 5 = 180; index 5
    (4) × 5 = 20; index 6 (2) × 5 = 10; index 7 (0). **1,070.**
  - Cards whose P does not decode (each): index 0 (still not accepted; the same 9-op sequence: 6 reads, 2
    replaces → 72) × 5 = 360; index 1 (70) × 5 = 350; index 2, K (the sequence is the same whether
    `before` is Backup or None: 4 reads, 2 replaces → 68) × 5 = 340; index 4, step 2's read-back (fails,
    no restore since `before` ≠ Primary, no step 3 → 2 reads → 4) × 5 = 20; index 6 (4) × 5 = 20; index 7
    (2) × 5 = 10; index 8 (0). **1,100.**
  - 4 × 1,250 + 1,070 + 8 × 1,100 = **14,870** — the run printed `misread-then-crash runs: 14870`.
- **Formatter: 103,680 → 145,152** — 7 report kinds (Saved, NotSaved, (u1), (u2), and (u3) × {record file,
  backup, both}) × 20,736 (3 × 3 × 6 × 6 × 4 × 4 × 2 × 2).

### 12.3 Campaign 4 on the FINAL text, then every gate (2026-10-01, ~21:07-22:00 EDT by the result files' times and `date`)

**Recipe** as §5/§8, scripted in the scratch dir (`review-fix-2026-10-01/mutate4.py`; fresh `backups-c4/`
and `mutations-c4/`): one exact edit (asserted to match exactly once in the final files), `touch` the file
and every test that includes it, `cmake --build build/test --target shulib_tests/fast`, **`Linking CXX
executable shulib_tests` required** (or, for a compile-time pin, a FAILED build whose first error is that
pin), the Part A filter run (`SAFE-SAVE*,FakeFileStore*,ProsFileStore*`; plus `ProsBlockSink*,
ProsBlockSource*` for M12a), restore, timestamps bumped, relink required, GREEN required, and the restored
file `cmp`-identical to its backup. Green baseline of the filter: **64 cases, 998 assertions.**

**Result: 52 runs, 52 RED; every restore relinked, green and byte-identical.** No `MUTANT` string left in
`include/`, `test/` or `src/`.
- **Campaign 3's set, re-run on the final text:** M01-M21, M06b, M12a, M12b — all RED (M06 a compile
  error at the `chooseEffective` `static_assert`, as before). Their kills widened: e.g. M04 now also
  fails the new two-fault directed test and the misread enumeration's pinned count; M11 the one-fault
  ORDER check and a golden.
- **The review's 14 survivors, now killed:** N01, N02 (the card-1 directed test, and the new reason oracle
  in the crash, pairs and misread enumerations: "Unconfirmed with the card present and something opened,
  reason not (u3)"); N03 (card 8's two-fault test); N04 (card 8's test, the backup-only (u3) golden, the
  enumeration's file check); N05 (the synthetic TOTAL golden); N06 ("would not re-open" golden); N06b
  ("different bytes" golden, the synthetic TOTAL golden); N07 (the path-cut test: 96 kept, not 95); N08
  (`/dev/full`: Flush expected); N09 (the self-test's own list: `stat`); N10 (the tree scan: `freopen`);
  N12 (empty bytes refused); N18 (card 5's step-1 false-flush golden); N22 (the two
  `primaryOpenedForWriting()` checks).
- **This round's fixes, each broken once:** R01 no re-read (9 cases: the clean save, the sequence, the
  ORDER rule in every enumeration, the two-fault directed test, card 8's, the limit test); R02 / R03 /
  R04 the clamp (the overcount test, load and save halves); R05 / R06 `strcmp` back in `safeReplace` /
  `loadWithBackup` (the alias preconditions); R07 / R08 / R09 no case fold / trailing dots kept / a
  `c | 0x20` fold (compile errors at the `sameFileOnCard` `static_assert`s); R10 the old "no usable
  record file before" sentence (4 cases); R11 / R12 `freopen` / `<fstream>` dropped from the scan's
  tables, R14 headers never matched, R15 literals not skipped (the self-test's own lists); R13 the
  forwarded-mode count not checked (the renamed-calls case).
- **Two runner-text corrections, logged:** R15's first text and R07's first text failed to COMPILE for an
  incidental reason (`-Werror` on a variable the mutant left unused), which is not the kill they were
  meant to show; each was re-written to keep the build warning-clean and re-run: R15 then went RED on
  the in-literal self-test, R07 on the case-fold `static_assert`s (`record_store_test.cpp` :592-596). The
  first attempts' records are kept as `R07-first-text.json` / `R15-first-text.mutant.txt`.

**Every gate, on the final tree (after the campaign; final lines verbatim):**
- `briefing_status.py generate` before the build (`briefing status: regenerated`); `cmake -S test -B
  build/test` exit 0; `cmake --build build/test -j8` exit 0, its doc-gate target printing the lines below.
- **Suite:** `[doctest] test cases: 1789 | 1789 passed | 0 failed | 3 skipped` / `[doctest] assertions:
  2649053 | 2649053 passed | 0 failed |` (before this round: 1,776 / 2,648,863; Part A's own: 64 cases,
  998 assertions). Pinned counts printed: `crash-point runs: 21840`, `one-fault runs: 48812`,
  `silent-write-then-crash runs: 38180`, `misread-then-crash runs: 14870`, `formatter combinations:
  145152`.
- `briefing_status.py generate` again after the suite binary existed, then `check`: `briefing status:
  generated block current (the durable narrative is NOT gated — see doc_staleness_audit.py)`.
- **`src` build gate:** `src build gate self-test: OK (189 detector cases, each against a real build)`;
  `src build gate: PASS (all 6 builds link; …)`, each of the six builds "exit 0, 8/8 TUs compiled,
  warnings: 6 vendor-ignored, 0 disallowed". `git diff -- src/ Makefile` is EMPTY afterwards.
- **ARM header gate** (196 headers, the strict set, `-mfloat-abi=softfp`; run after the `src` gate, whose
  self-test plants a fault in a library header): `ARM compile gate: CLEAN (all v2 headers)`.
- **ARM instantiation probe** (robot flags `-mfloat-abi=hard -mthumb -Os -Wall -Wextra -Wformat=2
  -Wno-psabi` + the strict set + `-Werror`): CLEAN. **Sizes on the final text:** `Size<452> a`
  (`SaveReport`), `Size<820> b` (`BackupLoad<180>`), `Size<792> c` (`LoadedRecord`), `Size<496> d`
  (`SafeSave<180>`) — what the header now states.
- **CI greps** (GNU grep 3.11): `library is PROS-free (all of include/shulib except hal/pros/)`; `layering
  holds: core is sim-free`.
- **Doc gates**, each exit 0: `api_doc_tool self-test: OK`; `doc coverage: 3057 public entities across 159
  headers, all documented`; `check-fresh` silent; `doc example scan: 467 quoted lines, all verbatim (5
  source files); 3 listings of whole-run files, each one unbroken run`; `removability: no public doc
  references docs/internal/`; `doc staleness audit self-test: OK (10 detector/filter cases)`; `doc
  staleness audit: clean (27 live docs, 0 numeric claims checked) …`.
- **The scan on the scratch merge with `shulib-v2`** re-run with the final scan text: 5 cases, 322
  assertions, green; the five sink mutations RED; restored identical (§12.1 F10).

### 12.4 For the coordinator — the merge, re-checked against `shulib-v2` at `7daf809` (read-only)

A trial three-way merge of every shared file (`git merge-file`, base `9239538`, ours = this worktree,
theirs = `shulib-v2`; scratch `merge-try/`): **clean** — `calibration_record.hpp` (H1 A1's `crc32.hpp`
include and this part's `file_store.hpp` include are one line apart and merge), `hal/pros/block_sink.hpp`
(COMP-t's Append section and this part's one `flush()` sentence), `docs/faq.md`, `README.md`, HANDOFF.
**Conflicting, as expected for shared documents:**

1. **The platform scan — NO patch any more.** §10 item 1's `scan-merge/…comp-t-merge.patch` is OBSOLETE;
   do not apply it. The scan carries COMP-t's opener itself and passes on the scratch merge (§12.1 F10).
2. **`docs/changelog.md`** (1 conflict): L9 A1 opened `## API 2.4`. This entry moves under it as
   **"Added since 2.4 (2026-09-29)"**, newest first beside T4 2a's, H1 A1's and DIAG1-B's "since 2.4"
   entries; its text says "no version bump", which stays true.
3. **`docs/hardware-assumptions.md`** (3 conflicts): (a) the status line becomes **7 of 165** (`shulib-v2`
   says 164); (b) SAFE-SAVE's "added ONE — HA-166" sentence joins the running list after T4 2a's ("with
   it, 165"), and "The rest of HA-162..HA-176 (all but HA-165, HA-167, HA-175 and HA-176)" gains HA-166;
   the **Held numbers** line may keep HA-166 inside its range — its own rule says "a number that lands as
   a row may stay in its range"; (c) HA-166's index row goes in NUMERIC order, between HA-165's and
   HA-167's. HA-166's section merges clean where it is (after COMP-a's HA-161 section, before TANK1's).
   **Cross-reference at merge:** belief (1)'s parenthetical on the append open should cite **HA-177**
   (COMP-t's: what an append open does), and HA-177 may cite HA-166 (1)-(3) back.
4. **REMAINING-WORK row 28 and `build-order.md`'s SAFE-SAVE row** (1 conflict each): neighbouring rows
   (DIAG1-B, L9, H1) changed beside this part's status words — take both sides.
5. **`docs/api/*`, `mkdocs.yml`, `PROJECT-BRIEFING.md`**: regenerate after the merge.
6. **Owed, not this part's to edit** (each read, the verdict here):
   - **B1's plan — a PROPOSAL (pace rule 2):** when B1 rewrites the worksheet's copy-first lines (G.0.1
     :732-735, :867-868) and HANDOFF's, keep "copy `shulib_drivetrain.bin` off the card first" until a
     SAVED run has left a verified `.bak` (`cmp` on a laptop — which is also HA-166's settle (a)). Robot
     two's card holds no `.bak` until the first library save; that first save is the one where THE LIMIT,
     or a wrong HA-166 (4)/(5), could still cost the only copy.
   - **The SAFE-SAVE brief** (COMP-t edited it too): its HA-166 template (§3.5: "can only READ … and
     REPLACE", "it never drives wrong", "five beliefs"), its §1.A.5 sentence "there was no usable record
     file before", §1.A.4 step 1 ("P and K, both, always") and landmine 13's blanket exclusion are
     superseded by the register, the header and §12.1 — annotate at the merge if the brief is kept as a
     reference.
   - **R5a's brief** (`r5a-a` in flight) says "HA-166 … five beliefs" (:121, :1191) and paraphrases (1) as
     "`wb` empties an existing file at the open" (:122): now six beliefs, (1)-(5) unchanged in number; its
     two names are distinct under (6).
   - **T4's brief** (`shulib-v2` :1931-1932, :3042, :3155) and **HANDOFF** (`shulib-v2` :595) describe
     `formatSaveReport` as printing "Teleop boots from …": it already says "the next boot" / "the drive
     program" (§5) — only two doc comments name "shulib Teleop" (`BootReadsBackup`'s and
     `formatSaveReport`'s, as today's program). T4 Part 3's item for these words is already done for the
     library.
   - Still owed from §10 item 6: guide chapter 11 :370-374; HA-139 (B1); HANDOFF §2 (B2); Station G (B1).

### 12.5 The prose review for this round (pace rule 4) — greps run and hits READ before this was written

**Changed this round:** `calibration/record_store.hpp`, `calibration/calibration_record.hpp`,
`hal/file_store.hpp`, `hal/pros/file_store.hpp`, four test files; `docs/faq.md`, `docs/changelog.md`,
`docs/hardware-assumptions.md`, `README.md`, `docs/api/*` (regenerated). **Greps** over `README.md` and
`docs/` (the generated `docs/api/` and this log set aside): `can never leave the card`, `can never erase`,
`never drives wrong`, `no usable record file`, `five beliefs`, `that is all`, `can only READ`, `only read`,
`empties an existing file`, `empties a file`, `empties one`, `fileNamesDiffer`, `sameFileOnCard`,
`two-fault`, `two faults`, `misread`, `SafeSave<180>`, `496`, `452 bytes`. **Every hit, and the verdict:**
- This part's own lines (FAQ, changelog, HA-166, the status sentence) — rewritten above; true now.
- REMAINING-WORK :105, HANDOFF :507, the brief's title ("can never erase") — the chunk's GOAL in plan
  documents; left (§12.1 F4).
- The SAFE-SAVE brief :24, :79, :150, :634, :768, :792, :1012, :1017, :1072, :1178, :1241, :1245 — the
  brief's own text, superseded where §12.1 says; owed to the coordinator (§12.4 item 6).
- R5a's brief :121, :122, :1191 — owed (§12.4 item 6).
- `misread` in guide 14 :130, guide 11 :389, `build-order.md` :385/:772, DIAG1/E1/C4 logs; `that is all` in
  R3b's log; `only read` in `verify-f1.sh`; `496` in SRC1/T4/COMP/R1b/E1 records — other subjects
  (IMU tags, fixes, mutation history, line numbers, a file length); nothing about the save. No hit for
  `fileNamesDiffer`, `SafeSave<180>` or `452 bytes` outside the generated pages.
- README :87-95 and :268-271 — fixed (F15). §7's earlier verdicts stand (re-read: none of their lines
  moved in meaning).

### 12.6 Close of this round

Part A's DoD (§11) holds on the final tree, re-measured above: every Part A test green with the counts
logged and derived (§12.2); every mutation — campaign 3's 24 runs re-run, the review's 14 survivors, 15
new — observed RED on fresh binaries (§12.3); no `src/` file changed; HA-166 in, now with six beliefs; the
golden unchanged (`test/calibration_record_test.cpp` untouched; the record's layout, version and encoder
untouched); the docs moved in the same change (§12.1, §12.5). **Nothing committed, nothing pushed, no
`pros` command, no serial port.** Part A stays BUILT and gated, awaiting the coordinator's verification;
B1 and B2 remain. Scratch for this round: `.claude/coordinator/scratch/safesave-a/review-fix-2026-10-01/`.

## 13. Verification-round findings and their disposition (resumed 2026-10-04, 08:06 EDT by `date`)

Executor: Claude Opus 5.5 (1M context), the coordinator's workflow, the builder again, same worktree
(nothing committed). A fresh verifier returned three minor findings (V-F1 to V-F3 below), with probes and
a mutant runner in `.claude/coordinator/scratch/safesave-a/verify/`. Scratch for this round:
`.claude/coordinator/scratch/safesave-a/verify-fix-2026-10-02/`.

### 13.0 The resumption — what the stopped run left, checked before anything was touched

A first run of this round (2026-10-02) was STOPPED partway. What it left, measured on 2026-10-04:

- **No edit to the tree.** Every file this part changed is older than this round: `record_store.hpp`
  2026-10-01 21:28, `hal/file_store.hpp` 21:30, `faq.md` and `hardware-assumptions.md` 20:56, this log
  22:02 (§12.6, its last section). The stopped run wrote nothing here; its only output is three probe
  binaries in the scratch dir above (16:37), which this run does NOT trust — they are rebuilt from source.
- **No mutation or plant left applied.** Every file a mutation runner backed up — campaign 4's six
  (`review-fix-2026-10-01/backups-c4/`) and the verifier's one (`verify/backups/`, whose runner mutated
  a COPY under `verify/tree/`, never this worktree) — `cmp`-IDENTICAL to its backup. No `MUTANT` string
  in `include/`, `test/` or `src/`. `git diff -- src/ Makefile` EMPTY; `git status` lists only this
  part's files (the `src` gate's plant targets — `robot_config.hpp`, `doctest.h`, `lv_obj_style.h`, the
  `src/` files — are newer only by the timestamp bump the gate gives a restored file).
- The test binary (`build/test/shulib_tests`, 2026-10-01 21:30) is NOT trusted for any result here: a
  full build after a regenerated briefing relinks it before the suite is read (§13.3).
- `shulib-v2` is at `237c4fe` (COMP-b landed since §12.4's `7daf809`); §12.4's merge notes are
  re-checked against it in §13.4.

### 13.1 Each finding: re-check, then disposition

Every finding was re-checked here before acting. The verifier's probes were REBUILT FROM SOURCE on the
current headers (`verify-fix-2026-10-02/rebuilt/`), not run from the stopped run's binaries.

**V-F1 (minor) — THE LIMIT left out a GOOD OLDER backup read as identical, and HA-166 said "a single
misread followed by a failure IS defended". REAL, and wider than reported; FIXED.**
- *Re-check.* `probe_stale_backup`, rebuilt: control `rP rK wK rK wP rP wP rP rP rK`,
  `backupFirst.attempted=1`, reboot loads K at 12.006; misread `rP rK wP rP wP rP rP rK`,
  `backupFirst.attempted=0`, reboot loads K at **11.500** (12.006 was in effect). `probe_misread_shapes`,
  rebuilt: (b′) `reboot loads NOTHING`, (c) `rP* rK wK* rK rP rK … reboot loads NOTHING`. Both are one
  misread plus one failure, so HA-166's closing sentence was false as written.
- *Measured further, before rewording* (a new probe, `probes/probe_stale_shapes.cpp`: a store that hands
  back chosen bytes as an open read at one operation, then step 2's write cut short or crashed). The
  class is every read that returns a VALID record the file does not hold, not only a backup read as
  identical. It costs the record whether K is damaged, missing or older (crash: NOTHING, NOTHING, the
  older K). It also costs the record when P is read as a different record: P good read as an older
  record (card 3 and card 4 shapes), P damaged read as a valid one, or P missing read as one. Then the
  backup copy and the restore both write the STALE record. A REPORTED short write is enough there (no
  crash), and the next boot loads the older record. **Worse for the words:** in those runs the save
  prints "The earlier record was written back to the record file and re-read: it is still on the card,
  and the next boot uses it as before." That is true of what step 1 read and false of the card. The
  old header line "the words say what step 1 READ, never what the card held" was therefore also wrong.
  No check can tell a stale copy from the file (the bytes ARE a valid record), so this is stated, not
  fixed.
- *Also measured:* a misread the checks CAN see, followed by a reported or silent write fault, with no
  crash. The verifier's `probe_misread_then_writefault`, rebuilt: 19,680 runs, 0 failures. The suite
  enumerated only misread-then-CRASH, so the sentence "followed by a failure" rested on a scratch probe.
- *Fix.* (1) `record_store.hpp` THE GUARANTEE now reads "a misread the checks can see … followed by a
  crash or by a write fault". THE LIMIT now names two kinds: (i) the record file misread as damaged at
  both of its first reads; (ii) a stale read, either the record file read as some other record than it
  holds or the backup (damaged, older or missing) read as identical. Its cost is "lose the record, or
  leave the next boot a record other than the one in effect (an older one)". The words now "rest on
  what step 1 READ … after (ii) they call the record the stale read returned 'the earlier record', and
  can say it is still on the card". (2) HA-166's blast radius says the same. Its last sentence is now
  "A single misread the checks CAN see — refused, short, or failing the CRC or the length check —
  followed by a crash or a write fault IS defended, and the host tests enumerate it". (3) The FAQ ("in a
  way its checks cannot see — … or a stale copy of a file read in place of what it now holds") and the
  changelog ("that its checks cannot see") say the same; before, both described the excluded misread
  too broadly. (4) `docs/api/record_store.md` regenerated.
- *Tests.* (a) **A MISREAD THEN A WRITE FAULT**, a new enumeration, so the register's sentence is
  enumerated by the suite and not only by a probe. It uses the same five visible misreads, now one
  helper `visibleMisreadsAt()` shared with the crash enumeration. At every later REPLACE of the faulted
  run it injects the open refused, a false flush, a silent no-op, and a short write, silent cut and
  silent flip at 0, 1, 33, 34, 90, 176 and 179: 24 write faults. **9,840 runs, derived by hand before
  the run** (§13.2), and the run printed `misread-then-write-fault runs: 9840`. (b) **THE STATED LIMIT
  (ii), measured**, five pinned shapes through a `StaleReadStore`:
  - card 5 with K read as P's bytes, then a crash: the boot loads the OLDER K (the verifier's case);
  - card 7 with its bad-CRC K read as P's bytes, then a crash: nothing;
  - card 3 with its missing K read as P's bytes, then a crash: nothing;
  - card 3 with P read as an older record, then a short write: the boot loads the older record, and the
    words say it "is still on the card";
  - card 8 with its short P read as an older record while K was in effect, then a short write: the same.
  The limit (i) test is renamed "THE STATED LIMIT (i)". The test file's header names both kinds.
- *Mutation:* L2 (step 1 always copies P to K) turns the limit (ii) test RED, so the test sees the
  limit move (§13.3).

**V-F2 (minor) — unhedged hardware claims: the append open's behaviour, and FAT name folding. REAL;
FIXED, plus one in-tree falsehood found while fixing it.**
- *Re-check.* `git show shulib-v2:docs/hardware-assumptions.md` (now `237c4fe`): HA-177 (2), "On an
  EXISTING file, `vexFileOpenWrite` writes after its last byte", is rated **invented**. This tree stated
  the same behaviour as fact in four places:
  - the FAQ, under "Read from PROS's and newlib's own vendored libraries";
  - `hal/file_store.hpp`'s header, twice, the second time under "What the vendored libpros.a and libc.a
    show";
  - HA-166 (1)'s parenthetical, under a "reasoned" belief.
  FAT folding was stated as fact in the FAQ ("On the card, `GAINS.BIN` and `gains.bin` are one file"),
  in `hal/file_store.hpp` ("The card is FAT … ONE file there") and in `sameFileOnCard()`'s doc ("the
  brain's card would take …"). It also appeared in `record_store.hpp`'s Contract and `loadWithBackup`'s
  doc ("two names the card takes for one file") and in the changelog ("whether the card takes two
  names for ONE file"); the verifier named only the first two places.
- *Found while fixing:* `hal/file_store.hpp` :5-7 also said "(the blackbox's own adapter offers it)".
  That is **false in this tree**: this tree's `hal/pros/block_sink.hpp` has no append mode. COMP-t
  added one on `shulib-v2`, and it belongs to the Auton Tester's log, not the blackbox. Removed.
- *Fix.* The append behaviour is no longer claimed anywhere in this tree. *(Not quite, as first written:
  the FAQ's opening sentence still said a program can open a file with `"ab"` "to append to it". The
  resumption found it and fixed it, §13.6.)* HA-166 (1) keeps only what
  the libraries show: `"ab"` → `vexFileOpenWrite`, "What that call does to a file is VEXos's and is not
  a belief of this entry". The index row says "an `"ab"` open goes to a different VEXos call, not
  measured, which the save does not use". The FAQ says `"ab"` "goes to a different VEXos call — what
  that one does to an existing file is not yet measured either". `file_store.hpp` says "what that call
  does to an existing file is VEXos's, and not measured". The behaviour belongs to HA-177 at the merge
  (§13.4). FAT folding is now FAT's rule, which "the brain's card is TAKEN to follow (HA-166 (6) …
  not measured)", everywhere it was stated: the FAQ, `file_store.hpp`'s header and `sameFileOnCard()`'s
  doc, `record_store.hpp` (Contract, `loadWithBackup`'s doc) and the changelog. `file_store.hpp` also
  says what happens if the card tells such names apart (the check refuses a few pairs needlessly), as
  HA-166's blast radius does. Precondition and `static_assert` messages say "(FAT ignores letter case
  …)", which is a true statement of FAT's rule, and stay as they are. `docs/api/file_store.md` and
  `record_store.md` regenerated.
- *Merge note:* the confidence clash is added to §13.4 item 3, which supersedes §12.4 item 3.

**V-F3 (minor) — `loadOne` lost its own `opened` check; countRead's guard was untested (V14 survived).
REAL; FIXED as suggested.**
- *Re-check.* `probe_notopened_count`, rebuilt on the real code: `from=2 primary.opened=0
  primary.count=0 primary.decoded=0`. `loadOne` read `seen.accepted = seen.count > 0 && accept(…)`, so
  countRead's `!r.opened ? 0U :` was the only guard. No test reached it: both shipped stores return 0
  on a refused read (`ProsFileStore::read` returns before setting `count`; the fake sets it only when
  it opens).
- *Fix.* `loadOne` is `seen.accepted = seen.opened && seen.count > 0 && accept(…)` again, with a
  comment, matching `readFile`'s `out.opened &&`. `StoreRead::count`'s doc now says it is 0 when
  `opened` is false, that both shipped stores do this, and that `record_store.hpp` does not rely on it.
  `FileRead::count`, `RecordFileRead::count` and `FileRead::accepted` say "0 / never when the read did
  not open". The header's Contract gains the not-opened rule. The changelog says "a read that did not
  open counts as no bytes".
- *Test.* "a store whose read says NOT OPENED but carries a count" uses a `CountsWhenRefusedStore` on
  card 5. On the load: `accept` is never called; primary and backup are not opened, count 0, not
  accepted; `from` None; and `loadRecord` agrees. On the save: `accept` is called exactly once, for the
  bytes to save; `before` None; step 2's read-back has `readOpened` false and `readCount` 0; the result
  is Unconfirmed (u3).
- *Mutation (§13.3).* V14 (countRead keeps the count) and V14c (both guards dropped) go RED on that
  test. **V14b, `loadOne`'s own `opened` dropped ALONE, stays GREEN, as predicted: it is an EQUIVALENT
  mutant** while countRead returns 0 for a read that did not open, and `seen.count > 0` then refuses
  first. A deliberately redundant guard cannot be killed alone. It is kept anyway, as the verifier
  advised, so `loadOne` matches `readFile` and a future change to countRead cannot reopen the hole
  unseen. V14c shows the pair of guards is tested.

### 13.2 The new pinned count, derived by hand BEFORE the run (the run matched it exactly)

**A misread then a write fault: 9,840.** §12.2's table gives, for each read index, which operations the
FAULTED run still has. Only its later REPLACES count here, each faulted 24 ways (3 + 3 × 7), with 5
misreads per read index:
- **Backup-first cards (3, 5, 6, 7):** index 0 (the re-read, then the full sequence) leaves 3 replaces;
  index 1 (K) leaves 3; index 5 (step 2's read-back, then the restore) leaves 1; indices 3, 7, 8 and 9
  leave 0. That is 7 × 5 × 24 = **840 each**.
- **Card 4:** index 0 leaves 2; index 1 (K now differs, so the backup copy runs) leaves 3; index 3 (the
  restore) leaves 1; indices 5, 6 and 7 leave 0. That is 6 × 120 = **720**.
- **Cards whose P does not decode (1, 2, 8-13):** indices 0, 1 and 2 leave 2 each; the rest leave 0.
  That is 6 × 120 = **720 each**.
- **Total:** 4 × 840 + 720 + 8 × 720 = **9,840**, and the run printed `misread-then-write-fault runs:
  9840`.
- **Cross-check:** the verifier's probe used 10 misread kinds instead of 5 and got 19,680 = 2 × 9,840.

The other pinned counts are unchanged (21,840 / 48,812 / 38,180 / 14,870 / 145,152). The misread list
moved into `visibleMisreadsAt()` with the same five entries.

### 13.3 Campaign 5 on the round's final code, then every gate

*(Written on the resumption of 2026-10-04 09:35 EDT, §13.6, from the stopped run's own result files: the
campaign ran 08:44-08:51 EDT by their times, and its records were read before this was written.)*

**Recipe** as §12.3, scripted in `verify-fix-2026-10-02/mutate5.py` (fresh `backups-c5/` and
`mutations-c5/`). A mutant may be several exact edits to ONE file, each asserted to match exactly once.
Then: `touch` the file and every test that includes it, build `shulib_tests/fast`, **`Linking CXX
executable shulib_tests` required**, run the Part A filter (`SAFE-SAVE*,FakeFileStore*,ProsFileStore*`),
restore, bump the timestamps, relink (required), GREEN (required), and `cmp` the restored file against its
backup. Green baseline of the filter: **67 cases, 1,061 assertions.**

**Result: 7 mutants on the library, 6 RED as required, 1 GREEN as predicted.** Every restore relinked,
went green and was byte-identical (each record's `restored_linked`, `restored_green` and
`restored_identical` are true).
- **V14** (`countRead` keeps the count of a read that did not open): RED, 1 case, 5 assertions: "a store
  whose read says NOT OPENED but carries a count".
- **V14b** (`loadOne`'s own `opened` dropped ALONE): **GREEN, 67/67, as predicted.** It is an equivalent
  mutant while `countRead` returns 0 for a read that did not open (§13.1 V-F3).
- **V14c** (both guards dropped): RED, the same case, 10 assertions.
- **R01** (the record file's re-read removed): RED, 10 cases. They include the new A MISREAD THEN A WRITE
  FAULT enumeration and THE STATED LIMIT (i).
- **V11** (the re-read done but ignored): RED, 4 cases, the new enumeration among them.
- **V01** (the re-read only when the first read did not OPEN): RED, 9 cases, the new enumeration among
  them.
- **L2** (step 1 always copies P to K): RED, 9 cases. Among them is **THE STATED LIMIT (ii)**, so that
  test sees the limit move.

**Campaign 5b — does the new enumeration's INVARIANT kill, and not only its ORDER oracle or its pinned
count?** Script: `mutate5b.py`. Two-file mutants: a library defect, plus `violation()`'s ORDER line turned
off in the test. The filter is the two misread enumerations.
- **R01-noorder: RED.** A MISREAD THEN A WRITE FAULT fails on the invariant itself, for example "3: P
  good, K absent | misread 1/0 at op 0 then write fault 2/0 at op 2: a record was in effect before and the
  reboot loads NOTHING".
- **V11-noorder: RED.** The same test fails on "… at op 3: report.before is none, the literal is P".
- **noorder-only** (the oracle off, the library intact; the control): **GREEN**, 2 cases, 7 assertions.
- Each restore relinked, went green and was byte-identical.
- No `MUTANT` string is left in `include/`, `test/` or `src/`.

**The code is unchanged since the campaign.** `record_store.hpp` and `record_store_test.cpp` are
`cmp`-IDENTICAL to `backups-c5/`. `hal/file_store.hpp` was last edited at 08:19, before the campaign
began. The resumption changed only Markdown (§13.6). So campaign 5 stands on the final code, and it was
not re-run.

**Every gate, on the final tree.** Re-run IN FULL on the resumption, after its three prose edits; the
final lines are verbatim. Logs: `verify-fix-2026-10-04/`.
- `api_doc_tool.py generate`. Then `briefing_status.py generate` (`briefing status: regenerated`).
  `cmake -S test -B build/test` exit 0. `record_store.hpp` touched, then `cmake --build build/test -j8`
  exit 0, its log showing `[100%] Linking CXX executable shulib_tests` (a FRESH binary). Its doc-gate
  target printed the doc lines below. `find include test src -newer build/test/shulib_tests`: none.
- **Suite:** `[doctest] test cases: 1792 | 1792 passed | 0 failed | 3 skipped` / `[doctest] assertions:
  2649116 | 2649116 passed | 0 failed |`.
  - Before this round: 1,789 / 2,649,053. This round added the A MISREAD THEN A WRITE FAULT
    enumeration, THE STATED LIMIT (ii) and the not-opened test.
  - Part A's own: 67 cases, 1,061 assertions.
  - Pinned counts printed: `crash-point runs: 21840`, `one-fault runs: 48812`,
    `silent-write-then-crash runs: 38180`, `misread-then-crash runs: 14870`,
    `misread-then-write-fault runs: 9840`, `formatter combinations: 145152`.
- `briefing_status.py generate` again after the suite binary existed, then `check`: `briefing status:
  generated block current (the durable narrative is NOT gated — see doc_staleness_audit.py)`.
- **`src` build gate:** `src build gate self-test: OK (189 detector cases, each against a real build)`.
  Then `src build gate: PASS (all 6 builds link; …)`, each of the six builds "exit 0, 8/8 TUs compiled,
  warnings: 6 vendor-ignored, 0 disallowed". `git diff -- src/ Makefile` is EMPTY afterwards.
- **ARM header gate:** 196 headers, the strict set, `-mfloat-abi=softfp`. It ran after the `src` gate,
  whose self-test plants a fault in a library header. Result: `ARM compile gate: CLEAN (all v2 headers)`.
- **ARM instantiation probe:** robot flags `-mfloat-abi=hard -mthumb -Os -Wall -Wextra -Wformat=2
  -Wno-psabi`, plus the strict set and `-Werror`. Result: CLEAN. Sizes: `Size<452> a` (`SaveReport`),
  `Size<820> b` (`BackupLoad<180>`), `Size<792> c` (`LoadedRecord`) and `Size<496> d` (`SafeSave<180>`),
  unchanged and as the header states.
- **CI greps** (`/usr/bin/grep`, GNU grep 3.11): `library is PROS-free (all of include/shulib except
  hal/pros/)`; `layering holds: core is sim-free`.
- **Doc gates**, each run explicitly and each exit 0:
  - `api_doc_tool self-test: OK`;
  - `doc coverage: 3057 public entities across 159 headers, all documented`;
  - `check-fresh` silent;
  - `doc example scan: 467 quoted lines, all verbatim (5 source files); 3 listings of whole-run files,
    each one unbroken run`;
  - `removability: no public doc references docs/internal/`;
  - `briefing status: generated block current …`;
  - `doc staleness audit self-test: OK (10 detector/filter cases)`;
  - `doc staleness audit: clean (27 live docs, 0 numeric claims checked) …`.

### 13.4 For the coordinator — the merge, re-checked against `shulib-v2` at `237c4fe` (read-only; SUPERSEDES §12.4 where they differ)

A trial three-way merge (`git merge-file`; base `9239538`, ours = this worktree as of this round,
theirs = `237c4fe`; scratch `verify-fix-2026-10-02/merge-try/`).
- **Clean:** `calibration_record.hpp`, `hal/pros/block_sink.hpp`, `docs/faq.md`, `README.md`, HANDOFF and
  `mkdocs.yml`.
- **New here, absent on `shulib-v2`:** Part A's four headers and four test files.
- **Conflicting, all in shared documents:** `docs/changelog.md` (1), `docs/hardware-assumptions.md` (4,
  one more than at `7daf809`), REMAINING-WORK (1) and `build-order.md` (1).

1. **The platform scan:** no patch (unchanged from §12.4 item 1).
2. **`docs/changelog.md`:** `## API 2.4` now opens with T4 Part 2's "Added and changed since 2.4" and
   H1 A1's "Added since 2.4". This entry moves under `## API 2.4` as **"Added since 2.4 (2026-09-29)"**,
   in date order among those entries. "No version bump" stays true.
3. **`docs/hardware-assumptions.md`** (4 conflicts):
   - (a) The status line becomes **7 of 166** (`shulib-v2` says 165, after COMP-b's HA-162).
   - (b) SAFE-SAVE's "added ONE — HA-166" sentence joins the running list after COMP-b's ("with it,
     166"). "The rest of HA-162..HA-176 (all but HA-162, HA-165, HA-167, HA-175 and HA-176)" gains
     HA-166. The **Held numbers** line keeps HA-166 inside its range, by its own rule.
   - (c) HA-166's index row goes in NUMERIC order, between HA-165's and HA-167's.
   - (d) **New since §12.4:** HA-166's section and COMP-b's new HA-162 section both insert right after
     COMP-a's HA-161 section. Take both, HA-162's first (numeric).
   - **The confidence clash (V-F2), and the cross-reference:** HA-177 (COMP-t) rates "on an existing
     file, `vexFileOpenWrite` writes after its last byte" **invented**. This tree no longer states that
     behaviour anywhere: HA-166 (1) keeps only "`"ab"` → `vexFileOpenWrite`", which HA-177 (1) also
     rates reasoned, so the two now AGREE. At the merge, end HA-166 (1)'s parenthetical with "— HA-177"
     (it says the call's behaviour "is not a belief of this entry"). HA-177 may cite HA-166 (1)-(3) back.
     If an older copy of this tree's text is merged instead (belief (1) saying "an append, which adds
     after a file's last byte"), that IS the clash. Do not take it.
4. **REMAINING-WORK row 28 and `build-order.md`'s SAFE-SAVE row** (1 conflict each): neighbouring rows
   (DIAG1-B, L9, H1) now say "landed". Take theirs for those rows and ours for row 28 / the SAFE-SAVE row.
5. **`docs/api/*`, `mkdocs.yml`, `PROJECT-BRIEFING.md`:** regenerate after the merge.
6. **Owed, not this part's to edit:** §12.4 item 6 stands as written: the B1 copy-first proposal, the
   SAFE-SAVE and R5a briefs, T4's and HANDOFF's "Teleop boots from" wording, guide chapter 11, HA-139,
   HANDOFF §2 and Station G. One addition: **the B1 proposal is now stronger.** THE LIMIT's second kind
   (a stale read) can make the words of a failed save say "The earlier record was written back … it is
   still on the card" about an OLDER record. So keeping the laptop copy of `shulib_drivetrain.bin`
   until a SAVED run has left a `.bak` that `cmp`s equal also covers the one case the words cannot be
   trusted.

### 13.5 The prose review for this round (pace rule 4) — greps run and hits READ before this was written

**Changed this round:**
- code and tests: `calibration/record_store.hpp` (THE GUARANTEE, THE LIMIT, the Contract, two docs,
  `loadOne`), `hal/file_store.hpp` (header, `StoreRead::count`, `sameFileOnCard()`'s doc) and
  `test/record_store_test.cpp`;
- documents: `docs/faq.md`, `docs/changelog.md`, `docs/hardware-assumptions.md` (HA-166's row,
  belief (1), the blast radius) and `docs/api/*` (regenerated).

**Greps** over `README.md` and `docs/`, with the generated `docs/api/` and this log set aside: `adds
after`, `append call`, `damaged backup`, `single misread`, `never what the card held`, `OLDER backup`,
`card takes`, `card would take`, `blackbox's own adapter`, `vexFileOpenWrite`, `stale read`, `stale
copy`, `GAINS.BIN`, `letter case`, `did not read as usable`, `read back the same way twice`, `"ab"`,
`append open`, `an append`, `one file on`, `same file`, `FAT`.

**Every hit, and the verdict:**
- **This part's own lines.** HA-166's index row, belief (1) and its blast radius; the FAQ :139-166; the
  changelog :41 and :62-65. All were rewritten above and are true now. The blast radius's "the save's
  report also never claims more than it measured" was itself false under THE LIMIT's second kind (the
  words can say an older record "is still on the card"). It now begins "Short of the limit below".
- **HA-166 (6) and its row** state "ONE file on the card" as the BELIEF, with its confidence beside it
  ("invented that VEXos follows it"). That is the register's form, and it stays.
- **Not this part's to edit.** The SAFE-SAVE brief :335/:342 (`vexFileOpenWrite`), :554/:641/:886, and
  COMP-t's brief :303/:896-899 are plan documents that cite or describe the reading, owed as in §12.4
  item 6. R5a's brief :865 holds R5a's words. `stale reading` in A3/T4/R1b logs, `appended` in the
  changelog and F1 handoff, `FAT`-as-in-"fatal" in guide 11 and cookbook 02, and `appends` in
  build-order :703 are about other subjects.
- **No hit outside this part's lines** for `adds after`, `append call`, `single misread`, `never what
  the card held`, `OLDER backup`, `card takes`, `card would take` or `blackbox's own adapter`.
- §12.5's verdicts stand; re-read, none of their lines moved in meaning.

### 13.6 RESUMED (2026-10-04, 09:35 EDT by `date`) — the second run of this round was stopped too

Executor: Claude Opus 5.5 (1M context), the coordinator's workflow, the builder again, same worktree.
The run that wrote §13.0-§13.5 (08:06-09:16 EDT) stopped after its gates. It had not written §13.3 or
this round's close.

**What it left, checked before anything was touched:**
- **No mutation or plant left applied.** Campaign 5's two backups (`verify-fix-2026-10-02/backups-c5/`:
  `record_store.hpp` and `record_store_test.cpp`) are each `cmp`-IDENTICAL to the worktree file. No
  `MUTANT` string is in `include/`, `test/` or `src/`. `git diff -- src/ Makefile` is EMPTY. `git status`
  lists only this part's files; none of the `src` gate's plant targets is modified.
- **Its result files**, read: `campaign5.log` and `campaign5b.log`, and every `mutations-c5/*.json`
  (each mutant linked; each restore relinked, green and identical). Also `final-suite.log` (1,792 /
  2,649,116, with `misread-then-write-fault runs: 9840`), `gate-selftest.log`, `gate-check.log`, and the
  ARM probe's object (09:16). These are now written up in §13.3.
- **One edit after its gates:** `docs/hardware-assumptions.md` changed at 09:06, after the 09:01 suite.
  A word-diff against the trial-merge copy taken at 08:45 shows a single change, "Short of the limit
  below," in the blast radius, which §13.5 records. So every doc gate had to run again (done, §13.3).
- **The test binary** (09:01) was not trusted. It was relinked from source after a regenerated briefing
  (§13.3).

**Re-verified (each §13.1 disposition, against the tree):**
- **V-F1.** Both places were re-read:
  - `record_store.hpp` :42-58. THE GUARANTEE now says "a misread the checks can see (refused, short, or
    failing the CRC) … followed by a crash or by a write fault". THE LIMIT (ii) names "the backup
    (damaged, older or missing) read as identical".
  - HA-166's blast radius. Its last sentence is now "A single misread the checks CAN see … IS defended".
  - The tests are in place: `visibleMisreadsAt()`, A MISREAD THEN A WRITE FAULT (9,840), and THE STATED
    LIMIT (ii) with its five cases.
- **V-F3.** `loadOne` reads `seen.accepted = seen.opened && seen.count > 0 && accept(…)`, and `countRead`
  keeps its `!r.opened ? 0U :`. The `CountsWhenRefusedStore` test sits beside the overcount test.
- **V-F2.** `file_store.hpp`'s header, `sameFileOnCard()`'s doc, HA-166 (1), the changelog and
  `record_store.hpp` are hedged as §13.1 says. Two remnants were found, and FIXED in this run:
  - **The FAQ's first sentence** still said a program can "open one with `"ab"` to append to it". That
    states the append behaviour HA-177 rates invented. The same paragraph's later sentence says what that
    call does is not yet measured. Now: "open one with `"ab"` (C's append mode)", which is true of the C
    library and claims nothing of VEXos.
  - **The changelog's `kRecordBackupFileName` bullet** said the compile-time check proves the names are
    "two different files on the card". The check applies FAT's rule; whether the card follows that rule
    is invented. Now: "two different files under FAT's name rule".
- **Formatting only.** HA-166's blast-radius paragraph, from "(4) or (5)" to its end, was re-wrapped (the
  09:06 edit had left one long line). A word-diff against the 08:45 copy shows no word changed except
  that edit.
- **The merge (§13.4)**, re-checked for the three docs edited here, by `git merge-file` against
  `shulib-v2` (still `237c4fe`; scratch `verify-fix-2026-10-04/merge-try/`). `faq.md` merges clean;
  `changelog.md` has 1 conflict and `hardware-assumptions.md` 4. Those are §13.4's counts, unchanged.

**The prose review for these edits (pace rule 4).** The greps were run over `README.md`, `docs/` (not
`docs/api/`) and `include/`, and every hit was READ before this was written.
- Patterns: `append to it`, `append mode`, `different files on the card`, `two different files`, `Short of
  the limit` and `name rule`.
- `append to it` and `different files on the card`: no hit left.
- `append mode`: the FAQ line, as fixed.
- `two different files`: the changelog line, as fixed. The other hits are the two precondition messages
  in `record_store.hpp` (:217, :564), "must be two different files on the card (FAT ignores …)". Those
  are requirements, not claims, and stay as §13.1 decided.
- `Short of the limit`: HA-166, as recorded in §13.5.
- `name rule`: this part's changelog and `sameFileOnCard()` lines, all hedged. Other hits are in the
  G1, T3 and T4 logs, on other subjects (per-robot names, binding names).

### 13.7 Close of this round (2026-10-04, 10:14 EDT by `date`)

All three findings are dispositioned, each REAL and FIXED: V-F1, V-F2 (with the two remnants above) and
V-F3.
- **Mutation:** campaign 5 is RED as required, and V14b is an equivalent mutant, as predicted. The
  restores are verified. The code is unchanged since the campaign.
- **Gates:** every gate was re-run on the final tree and is green (§13.3).
- **The DoD (§11/§12.6) holds:**
  - no `src/` file changed;
  - the golden is unchanged;
  - the record's layout, version and encoder are unchanged;
  - HA-166 has six beliefs;
  - the docs moved in the same change.
- **Nothing committed, nothing pushed, no `pros` command, no serial port.**
- Part A stays BUILT and gated, awaiting the coordinator. B1 and B2 remain.
- Scratch: `verify-fix-2026-10-02/` (the round) and `verify-fix-2026-10-04/` (this resumption's gate
  logs, the ARM probe and the merge check).

## The coordinator's merge — Part A (2026-10-04, 10:05–10:50 EDT)

- **The run:** the build (2026-09-29), three reviews, the fix round, a fresh verifier (17 dispositions confirmed,
  three minor findings) and its fix pass — across usage-limit stops and two session ends, resumed each time with
  continuation notes. Transport commit `41ebed2` in the worktree `safesave-a`, squash-merged onto `237c4fe`.
- **No code conflict.** H1 Part A1's edit of `calibration_record.hpp` (the CRC moved to `diag/crc32.hpp`) and this
  part's (the backup's name and its two checks) merged cleanly; COMP-t's opener in `hal/pros/block_sink.hpp` passes
  this part's platform scan as the fix round amended it. Document conflicts resolved by hand: the changelog (this
  part's section moved under `## API 2.4`, "Added since 2.4 (2026-10-04)"), the register (status 7 of 166; this
  part's sentence after COMP-b's; rows in numeric order; its section after COMP-b's), REMAINING-WORK (row 28),
  build-order (the deviations row; the Next line). `docs/api` and the briefing regenerated.
- **Gates on the merged tree:** suite **2,103 | 2,103**, 3 skipped; `src_build_gate.py` self-test OK and check PASS on
  all eight builds; the ARM header gate (201 headers); both CI greps; `check-fresh`, removability and the staleness
  audit clean; no plant. The run's first build stopped on a briefing written from the previous test binary (the
  stale-binary order) — regenerated after the suite binary existed, then the full build with its doc gates passed
  and `check` is current (`.claude/coordinator/logs/ssa_merge_verify.log`, `build.ssa-final.log`).
- **Six coordinator mutations, each on a fresh binary or the compiler, all caught** (`muts_safesave_a.json`): the
  record written although the backup write failed (7 cases); a failed record write never restored (8); the backup
  refreshed although the record write failed (13); a write counted as done without reading it back (5); the backup
  overwritten when it already matches (9); and the backup preferred over a good record — refused AT COMPILE TIME by
  `test/record_store_test.cpp:571`'s `static_assert(chooseEffective(true, true) == Which::Primary)`.
- **Not done:** Part B1 (Calibrate's save uses it — before the next Calibrate rerun) and Part B2 (the boot's backup
  fallback and the driver panel's word). Until B1 is in the build, copy `shulib_drivetrain.bin` off the card before
  any Calibrate run. Nothing of it has run on a robot.
