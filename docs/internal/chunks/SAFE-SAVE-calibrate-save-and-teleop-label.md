# SAFE-SAVE — Calibrate's save can never erase a good calibration record; Teleop names where its numbers came from

> **THE TEAM LEAD'S ANSWERS (2026-09-26, ~23:10): every recommendation accepted** — "Accept all" on
> D1–D8 as written below. Builders start after tonight's claw and auton merges (his ruling of ~14:15:
> "briefs now, build after tonight"); each part still starts on its own go, at its turn in the order
> the brief gives. Committed by the coordinator with its plan entries.

> **Execution brief (DRAFT for the team lead's go).** Written 2026-09-26 (evening) by a read-only drafter
> at `shulib-v2` HEAD `a01e3a2`, after the power loss. It was re-drafted from the tree because the first
> draft was lost, then **re-checked the same evening against a checker's twelve findings** (section
> below). **Status: WAITS for the team lead's go and his answers to D1–D8 below. Nothing here has
> been built.** Three parts, each with its own go, verification and commit (D4):
> - **Part A** is the library. It edits no code file that is in flight. Four shared documents are
>   reconciled at merge (§4).
> - **Part B1** is Calibrate's save. It waits until T3 Part 2 and G1 Part 2c-final have landed.
> - **Part B2** is the boot's fallback, Teleop's words and the pin. It also waits for COMP-b.
>
> **Executor model: `opus`. The executor does not commit.** Live log:
> `docs/internal/chunks/SAFE-SAVE-PROGRESS.md`, **created FIRST** and appended as work proceeds. The
> coordinator may give the chunk a plan ID (for example DEFECTS3, in the DOCS/DEFECTS family) when it
> commits this brief. If it does, it tells the executor, and the log takes that name.
> **HA number reserved for this chunk: HA-166 only.** One entry holds all five beliefs (§3.5), and the
> coordinator renumbers at merge if needed.
> Size, stated honestly: **Part A ~1–1½ days, Part B1 ~½ day, Part B2 ~½ day.** The status board says
> "~½ day" for both defects. That estimate was made before anyone knew the brain cannot rename a file
> (§0.2), and before the re-check made the save's report a measurement of both files.

---

## Re-check 2026-09-26 (evening) — amendments

A checker returned twelve findings (6 major, 6 minor, verdict "needs changes"). Each was re-verified here
against `a01e3a2`, the `t3-part2` and `g1-2c-final` worktrees' diffs and the vendored archives. **All
twelve are real**, and each is fixed below in the design, the tests or the lists it concerns. Four more
problems were found during the re-check.

1. **Oracles contradicted the design (major).** Confirmed: `fake_block_sink.hpp` :41-52 keeps the bytes
   on a false flush. So a NOT SAVED run on a card with no earlier record could leave the new record
   booting, and "SAVED, backup not refreshed" broke "SAVED ⇔ K == P". Fixed at the root:
   - `put` succeeds only on its read-back.
   - Step 4 re-reads BOTH files and checks them against what the save last saw.
   - The word comes from that measurement.
   - The oracles are restated, and two branches are added (SAVED with a reported failure; SAVE
     UNCONFIRMED; D8).
   - *Also found:* the draft's order oracle "K is never written when P did not decode" contradicted its
     own step 3. It is restated in §2.
   - *Also found, while re-checking this fix:* a card that fails at the save's very FIRST read would be
     "measured" as empty, giving "nothing usable, Teleop will refuse" with a good record on it. So a
     refused read after the save's own write counts as UNKNOWN, not absent. Step 4 names three
     UNCONFIRMED reasons (u1–u3), and the crash test runs with the card both reported gone and still
     reported present.
2. **Mutations 2, 4 and 11 could not go red (major).** Confirmed. Fixed:
   - The fake gains SILENT faults, and §2 adds a silent-write-then-crash pair enumeration.
   - New checks: order checks from the log, a directed restore test, and literal expected records for
     13 starting cards. The cards now include "P absent, K good" and both a short P and a 180-byte
     bad-CRC P.
   - Mutation 12 is reworded, every mutation names what kills it, and mutations 15–21 are added.
3. **"Part A touches no in-flight file" was false (major).** Confirmed with `git diff` in both
   worktrees:
   - T3 Part 2 edits `hardware-assumptions.md`'s status paragraph (`@@ -68`), `build-order.md`, HANDOFF,
     README, the roadmap and guide 07. It also edits the worksheet's **Station G** (:832) and
     `test/mechanism_table_test.cpp`, and adds `src/teleop_claws.hpp` and its test.
   - 2c-final edits HA-111 and HA-128, and more (§4).

   Both lists are corrected. The header and D4 no longer claim "nothing in flight", and the four shared
   documents are reconciled at merge, worded as DIAG1-B's brief does.
4. **The boot's `/usd/` sentences had no seam (major).** Confirmed: `boot_source.hpp` :230-250 builds
   them from `card.path()`, and `IFileStore` had no path.
   - Fixed: each store result carries the path the ADAPTER used (`path()`).
   - `ProsFileStore` has no root of its own, so `/usd/` stays spelled only in the two adapters.
   - `loadRecord`'s per-file result is specified, and an absent backup adds no line.
5. **D1's power-cut claim rested on an unregistered belief (major).** Confirmed by disassembly:
   - `usd_write_r` calls `vexFileWrite` and then `vexFileSync`, and ignores what the sync returns.
   - `usd_close_r` returns 0 whatever `vexFileClose` answers.

   HA-166 gains belief (5), and D1 is qualified "given HA-166 (4) and (5)". D1 also states the one
   two-fault case the save does not defend. HA-122 gets two cross-reference sentences. This is still one HA number.
6. **The worksheet keeps a false sentence, and the only protection lived in a gitignored brief
   (major).** Confirmed: worksheet :849-850, and `.gitignore` :21 covers `.claude/`. Fixed in §3.0: at
   the brief's commit the coordinator corrects :849-850 and adds "copy the record off the card first" to
   G.0.1 and HANDOFF §5. B1 rewrites both lines once the save is safe.
7. **The LCD precedence ignored T3 Part 2's CLAW ABSENT (minor).** Confirmed: `t3-part2`'s
   `teleop_program.cpp` :755-761 orders fault > no controller > DEGRADED > CLAW ABSENT > ok.
   - The order is now decision D7.
   - The words for a BACKUP record whose port check is DEGRADED are defined.
   - The compile-time fit check covers only fixed texts. Runtime texts go through `sourceRows()`.
8. **Exception, allocation and buffer contracts were unstated (minor).** Confirmed: `check.hpp` :9-21
   (both policies throw), and each adapter's constructor allocates `path_` and checks preconditions
   (`block_sink.hpp` :79-93, `block_source.hpp` :71-85). Now stated:
   - The store methods and the `record_store` functions are exception-neutral, never `noexcept`.
   - The allocation rule is reworded (landmine 11).
   - `safeReplace` is a template on its largest file, with two local buffers and a stated stack cost.
9. **The scan missed HA-166 (3), and `cardPresent()` and the fence were unspecified (minor).**
   Confirmed: `fopen` appears only at `block_sink.hpp` :91, `block_source.hpp` :83 and
   `bench_r3a.cpp` :611. Fixed:
   - The scan bans `+` modes and allows `fopen` only in those three places.
   - `cardPresent()` is specified behind the standard fence, and mutation 12 is reworded.
   - *Also found:* a naive regex matches `motion_scheduler.hpp` :1217's trailing comment
     ("innermost link (validation hook)"). The scan strips trailing comments too.
10. **D6 misread L7, and D4 left L7 and COMP-b unordered (minor).** Confirmed against L7 as it stood
    at 20:16:
    - the line D6 cited (L7 :254) is `mergeSections`; L7's save was its §C.3;
    - L7 Parts C and D edit `calibrate_program.cpp`, `calibrate_panel.hpp`, `boot_source.*` and
      `teleop_program.cpp` (L7's parts table).
    - *Also found:* COMP's brief B11 (`COMP-competition-program.md` :90-95) may move the claw-port
      refusal and a shared panel into shared `src/` files.
    - *Also found:* L7 was revised in parallel (21:43) and has already adopted this chunk's
      `safeReplace` as its own **D12**. It still names a single "SAFE-SAVE Part B", and adds a byte-level
      load rule only "if SAFE-SAVE Part A has not already added it".

    Fixed:
    - D6 now cites L7 by section and lists the three reconciliations L7 needs.
    - Part A provides `loadWithBackup`.
    - D4 orders COMP-b and L7 outright, and **recommends splitting Part B (B1/B2)**, which changes the
      draft's recommendation. The reasons are in D4.
11. **The documentation list had gaps (minor).** Confirmed each gap. Added:
    - HANDOFF §2 :108-112;
    - worksheet :538 and Station 6 (:265);
    - the BACKUP row, moved from F.0 (refusals) to F.1/G.6;
    - 2c-final's `calibrateInitialize` banner;
    - HA-139's settle and blast radius;
    - HA-166's settle (b), pointed at the existing Bench Tests SD probe (button 5,
      `bench_r3a.cpp` :592-630).
12. **Test 6's fixture is file-local to a file both in-flight chunks edit (minor).** Confirmed:
    `test/boot_source_test.cpp` :79-318. Part A COPIES the fixture, and `codeOnly` for its scan, with
    citations. Consolidation waits until both chunks land.
    - *Also found:* the fixture is REBUILT from the 2026-09-25 capture's printed values. The record was
      written 2026-09-24 17:42:16. These are not the card's own bytes, and the brief now says so.

---

## Decisions for the team lead (each with a recommendation; nothing below is decided silently)

- **D1 — How to make the save safe when the brain cannot rename or delete a file (§0.2).**
  The plan was "write a temporary file, check it, then rename it over the old one". That plan **cannot
  be built on this brain.** **RECOMMENDED:** write **the backup first, then the record, then refresh the
  backup** (§1.A.4):
  - the old record is copied to a second file, `shulib_drivetrain.bak`, and read back to prove the copy
    before the record file is opened for writing;
  - if the new write does not read back, the old bytes are written back from memory and read back again;
  - when the record file does not decode, **the boot falls back to the backup copy** (D2).

  With this design, **given HA-166 (4) and (5)**, every failure at any step leaves the card holding
  either the record that was in effect before the save or the new one. That includes a power cut or a
  pulled card at any moment. (4) says that writing one file never damages another; (5) says a file that
  was flushed, closed and read back whole is on the medium.

  **The one case it does not defend** takes two faults in the same save. The card must first MISREAD a
  good record file at the save's first read, as short or altered. The save then treats the record as
  damaged, backs nothing up, and a second failure in the same save loses it. HA-139 is the belief that
  a read returns what the card holds.
  *Alternatives:*
  - **(b)** The same save but no boot fallback. The backup is then only something to copy back on a
    laptop, so a torn save leaves Teleop refusing until Calibrate saves again. Part B2 gets smaller (no
    change to `boot_source` or Teleop).
  - **(c)** Two record files used in turn, with a generation number. This changes the card layout.
    Every package already uploaded would refuse it, because the decoder refuses a version it was not
    built for (`calibration_record.hpp` ~:229-233). L7's draft also binds its gains file to the
    record's CRC. **Rejected.**
  - **(d)** Correct the message only. **Rejected:** the hazard stays.
- **D2 — When the boot may use the backup (only if D1 = recommended).**
  **RECOMMENDED:**
  - Use the backup **only when the record file does not decode**: missing, empty, short, wrong
    magic/version/length, bad CRC, or `valid()` refused.
  - **Never** use it when the record decodes and then fails its port check. That verdict is about the
    robot, and the backup is the same record or an older one. The existing rule "a record that fails
    its port check never falls back" (`src/boot_source.hpp` ~:22-23) is extended to the backup.
  - **Always say so:**
    - a serial line naming the BACKUP;
    - Teleop's source rows in amber: `MEASURED record from the BACKUP copy on the SD card, verified
      against the ports`, or `MEASURED record (BACKUP copy) | <the DEGRADED reason>` when its port check
      is DEGRADED;
    - the controller LCD `BACKUP see brain`, at the place D7 gives it.
  - To reset a card on purpose, **delete BOTH files** on a laptop. This is documented.

  *Alternative:* a boot from the backup refuses and names the file. That is safer against a stale
  backup, but a torn save becomes a robot that cannot drive until Calibrate runs again.
- **D3 — Part (2), the "table" label, was already fixed at G1 Part 2b (`7f9b9a1`; §0.3).**
  **RECOMMENDED:**
  - Close it as fixed.
  - Keep only a **regression pin** in Part B2. Teleop's source words become one host-tested function,
    and "table" can appear only when the typed table drove.
  - Remove the item from the status board's list.

  *Alternative:* drop it with no pin. But no host test covers the panel's source words today.
- **D4 — Order.** **RECOMMENDED: Part A now; SPLIT Part B into B1 and B2.**
  - **Part A now.** It edits no code file in flight. Its four shared documents (§4) are reconciled at
    merge.
  - **B1 (Calibrate's save) after T3 Part 2 and G1 Part 2c-final land, and BEFORE the next Calibrate
    rerun.** The careful rerun is owed for the ~8 % scale shortfall and to close R3d.
    - Of the in-flight chunks, only 2c-final edits `calibrate_program.cpp` (its `calibrateInitialize`).
      COMP-b does not touch it.
    - B1 edits the worksheet's Station G, which T3 Part 2 also edits (:832). B1 therefore also waits
      for T3 Part 2.
  - **B2 (the boot's fallback, Teleop's words, the LCD, the pin) after COMP-b lands as well.** COMP's
    brief B11 (`COMP-competition-program.md` :90-95) may move the claw-port refusal into the shared
    `src/boot_source.*` and a shared panel out of `teleop_program.cpp`. B2 edits both. B2 also makes
    COMP's status screen use the source-words function in the same change.
  - **Both before L7.** L7's Part C runs after A and B1; its Part D runs after B2. They edit the same
    files (L7's parts table).
  - **Until B1 lands, copy `shulib_drivetrain.bin` off the card on a laptop before any Calibrate
    rerun.** That is the only protection there is today. The coordinator writes it into the worksheet
    and HANDOFF at the brief's commit (§3.0), not only here.

  *Why the split now:*
  - COMP-b's likely edits to `boot_source.*` would put a single Part B behind three chunks. B1 alone
    removes the hazard at the moment it bites.
  - A single Part B's documentation list runs past pace rule 7's "about five documents": the worksheet,
    guide ch. 11, 12 and 14, HANDOFF, the register and `build-order.md`.
  - Cost: `formatSaveReport` takes a `BootReadsBackup` argument (§1.A.5). B1 passes `No`, so a
    BACKUP branch says "this Teleop will REFUSE". B2 flips the argument to `Yes`, a one-token change.

  *Alternative:* one Part B after T3 Part 2, 2c-final AND COMP-b, and before the next rerun. It is
  simpler (no `BootReadsBackup` argument), but the protection waits for COMP-b.
- **D5 — Does Calibrate's SCREEN say what the card holds after a NOT SAVED run?** **RECOMMENDED: serial
  only in this chunk.**
  - The screen keeps `NOT SAVED -- copy these numbers NOW` (`calibrate_panel.hpp` :404), which is true
    in every branch, SAVE UNCONFIRMED included.
  - The truth reaches the brain's screen at Teleop's next boot: a green record, an amber BACKUP, or a
    refusal that names the file.
  - `calibrate_panel.hpp` stays untouched. L7 Part C edits it next and can add the line then.

  *Alternative:* one SMALL line on the NOT SAVED screen. That means a `calibrate_panel.hpp` change, a
  defaulted parameter on `viewFor`/`TouchGate::view`, and proof that the layout still fits. One argument
  for it: 2026-09-24's run 2 had no serial capture.
- **D6 — L7's gains file should use this chunk's safe save AND its load rule.**
  - L7's first draft saved its sibling file "write, flush, close, re-open, read back" and named
    `calibrate_program.cpp ~:627-684` as the pattern. That is the function B1 REMOVES, and it has this
    chunk's hazard.
  - (The draft of this brief cited L7 :254 for the failure concern. That line is `mergeSections`, which
    merges sections within one file and is not about failure. L7's own D1, reason 1, is where it says a
    half-failed save must not cost the robot its drivetrain record.)
  - **L7 has since adopted this as its own D12** (re-checked the same evening, in parallel).

  **RECOMMENDED:** keep L7's D12, and have the coordinator reconcile three things in L7:
  1. **The split (D4).** L7's Part C waits for SAFE-SAVE **A and B1**. It edits `calibrate_program.cpp`
     and reads the record through Part A's `loadRecord`, not B2's boot. L7's Part D waits for **B2**.
     L7 today says "SAFE-SAVE Part B" in its re-check item R4, its D8 and D12, its parts table and its
     in-flight list.
  2. **The load rule.** Part A now provides the byte-level `loadWithBackup` (§1.A.4), so L7's "adds it
     beside `safeReplace` if SAFE-SAVE Part A has not already added it" becomes "uses Part A's".
  3. **The words and the buffer.**
     - `formatSaveReport` is the DRIVETRAIN record's: its sentences name that file and what Teleop does.
       L7 words its own lines from the same generic `SaveReport`; "SAFE-SAVE's `formatSaveReport`
       wording applies" is wrong as written.
     - `safeReplace` is a template on `MaxBytes` (capped at 1024), so L7 states its file's maximum size.

  This stays consistent with L7's binding. `decideGains` binds to the CRC of the drivetrain record the
  boot CHOSE. A boot from an older BACKUP therefore sees gains bound to a newer record and reports
  PLACEHOLDER by name, which is correct.

  This is a coordinator edit to a draft, not code in this chunk.
  *Alternative:* L7 saves safely but its boot ignores its gains backup. The backup is then for a laptop
  only, and a torn gains save degrades to PLACEHOLDER (L7's D4), which never refuses.
- **D7 — Where BACKUP sits on the controller LCD's third row (new at the re-check).** T3 Part 2 orders
  it fault > no controller > DEGRADED > CLAW ABSENT > ok (`t3-part2` `teleop_program.cpp` :755-761).
  **RECOMMENDED: fault > no controller > DEGRADED > CLAW ABSENT > BACKUP > ok.**
  - CLAW ABSENT changes what the driver can do right now: those buttons do nothing.
  - BACKUP drives correctly from a verified record. It asks for a Calibrate run, not a different way of
    driving.
  - The brain's source rows show BACKUP in amber for the whole session anyway.

  *Alternative:* BACKUP above CLAW ABSENT, to push the team toward the rerun sooner.
- **D8 — The measurement decides the word, in two cases the draft did not cover (new at the re-check).**
  **RECOMMENDED:**
  - **(a) SAVED when the record reads back whole**, even though the card reported a failure on the way
    (say a false flush). The failure is printed on its own line. This is ruling P5 applied as written:
    "SAVED is a measurement or it is not said" (`calibrate_program.cpp` ~:656-657).
  - **(b) A third word, `SAVE UNCONFIRMED`,** in three cases, each named:
    - the card is gone;
    - the card is present but refused every open in the save;
    - the card does not read back at the end the way the save last saw it (pulled, or failing).

    The program then cannot say which record Teleop will use, and says exactly that. The panel shows NOT
    SAVED.

  *Alternatives:*
  - (a) NOT SAVED on any reported failure. Then a NOT SAVED run on an empty card can still leave the
    new record booting, and the report must say so in a NOT SAVED sentence.
  - (b) None: exclude read faults at the end from the tests. But then a pulled card at the end prints
    "nothing is usable, Teleop will refuse", which is false once the card goes back in.

---

## 0. Why

### 0.1 Defect (1): a Calibrate save that fails part-way erases the good record, and the text says the opposite
Found by the 2026-09-26 audit of the status page (~01:17–01:27). **Nothing in the tree records it. This
brief is its first record in the tree.** The audit is not in the tree. The status board carries it as
"Calibrate's safe save" (the coordinator's status page, "Everything left").

**Where the damage happens:**
- `src/calibrate_program.cpp` `saveAndReadBack()` (~:627-684) constructs
  `hal::pros::ProsBlockSink sink{calibration::kRecordFileName}` (~:641).
- That constructor runs `std::fopen(path_, "wb")` (`include/shulib/hal/pros/block_sink.hpp` :90-92). It
  opens **truncating**; the adapter says so itself: "one blackbox file per boot" (:64-67, :73-74).
- **The old record is therefore gone at the open**, before a byte is written. Any failure after that
  point leaves no usable record on the card:
  - a refused write (~:647-650);
  - a refused flush (~:651-654);
  - the close, which cannot report its own failure (`block_sink.hpp` :95-111);
  - a failed re-open (~:658-661);
  - a short or differing read-back (~:665-675);
  - a power cut or a pulled card.

**What the program then says:**
- `finishDone()` prints `NOT SAVED -- <why>` and then **"(A record saved by an EARLIER run, if any, is
  still on the card -- this run did not replace it.)"** (~:903-904). That is false whenever the failure
  came after the open.
- Station G of the worksheet repeats it: "a run that stops, or says NOT SAVED, leaves the previous
  record where it was" (`R3a-BENCH-WORKSHEET.md` :849-850). **The coordinator corrects that line at this
  brief's commit (§3.0).**

**Consequence:** a rerun that ends NOT SAVED leaves "shulib Teleop" refusing at boot. Teleop cannot
drive without a record (HANDOFF §2): `src/chassis_table.hpp`'s track width and ratio are 0.

**Why now:** a careful Calibrate rerun is owed for two reasons. The saved scale reads ~8 % short (`est x`
44 at 48 in, `R3d-PROGRESS.md` ~:2507-2509). The recorded reproduction run also closes R3d. That rerun is
exactly the moment this defect can bite. It would erase the record that eight boots of Teleop drove from
on 2026-09-25.

### 0.2 The platform finding that changes the plan: the brain has no rename and no delete
Read from the **vendored** binaries on 2026-09-26 with `arm-none-eabi-objdump -dr` (re-read at the
re-check):
- **newlib `rename()` is `_link_r` followed by `_unlink_r`** (`firmware/libc.a`, `libc_a-renamer.o`).
  `remove()` and `unlink()` are `_unlink_r` (`libc_a-remove.o`, `libc_a-unlinkr.o`, `libc_a-sysunlink.o`).
- **PROS 4.2.2 defines `_link` and `_unlink` as stubs** that set `errno = 88` (newlib's `ENOSYS`) and
  return −1 (`firmware/libpros.a`, `file_system_stubs.c.o`).
- The same object stubs `_stat`, `mkdir`, `chdir`, `chmod`, `fchmod`, `fchmodat`, `pathconf`,
  `symlink`, `readlink` and `getcwd`.
- No other archive defines `_link` or `_unlink`.
- The VEXos jump table has no delete and no rename either. `libpros.a`'s `v5_apijump.c.obj` exports only:
  - `vexFileOpen`, `vexFileOpenWrite`, `vexFileOpenCreate`;
  - `vexFileRead`, `vexFileWrite`, `vexFileSeek`, `vexFileTell`, `vexFileSize`;
  - `vexFileSync`, `vexFileClose`, `vexFileStatus`;
  - `vexFileDirectoryGet`, `vexFileDriveStatus`, `vexFileMountSD`.
- **`usd_open_r`** (`libpros.a`, `usd_driver.c.o`) first calls `vexFileMountSD`, then handles the three
  access modes as follows:
  - **read-only** → `vexFileOpen(path, "")`;
  - **write-only with `O_APPEND`** (bit 0x8) → `vexFileOpenWrite`;
  - **write-only without it** → `vexFileOpenCreate`; this is `"wb"`. `O_CREAT` and `O_TRUNC` are never
    consulted;
  - **read-write** → `errno = 22` (EINVAL).
- **`usd_write_r`** passes each write to `vexFileWrite`, then calls **`vexFileSync`** on the file and
  **ignores what it returns**. It returns the write's count.
- **`usd_close_r`** calls `vexFileClose` and **returns 0 whatever it answers**. So `fclose` can fail only
  on newlib's own buffered bytes, and a close that VEXos refuses is invisible to the program.

So on this brain a program can **read a whole file** and **replace a file by truncating it**. That is
all it can do. "Write a temp file, then rename" is impossible. So is deleting a stale file or updating
one in place. PROS does sync after every write it passes on. What that sync commits is VEXos's
(HA-166 (5)).

**The host cannot see any of this.** The adapters' host tests run real `std::fopen` against a Linux temp
directory (`test/CMakeLists.txt` ~:86-97; `test/pros_block_sink_adapter_test.cpp` ~:33). There,
`std::rename` works, so a rename-based design would pass every host test and fail on the robot. This
finding becomes **HA-166** (§3.5). The design never calls rename, remove, unlink, link, stat or mkdir,
and never opens read-write. A test scans for all of these (§2).

### 0.3 Defect (2): Teleop's screen called Calibrate's port list "table" — already fixed at G1 Part 2b
The audit saw the **old** Teleop (`src/main.cpp` before `7f9b9a1`, the build that drove on 2026-09-25).
Its panel printed `table %s` in front of the port map whatever the source was:
`git show 7f9b9a1^:src/main.cpp` :874 (`std::snprintf(line, sizeof line, "table %s", tankPortMapString())`).

G1 Part 2b **dropped that row from the screen**. The port map is serial-only (`[SES] ports MEASURED …`),
and the table says why: "DROPPED from the screen — A3: the `table` label was false on a MEASURED record"
(`G1-PROGRESS.md` :1216, and F12(c) at :1342 for the worksheet).

At HEAD, `src/teleop_program.cpp` `TeleopPanel::paintAll()` (:590-613) chooses the source rows as
follows:
- `kTypedSource` ("TYPED TABLE -- unverified since the rebuild; run shulib Calibrate", :305-306), amber,
  **only** when `boot_.source == UseTable`;
- `MEASURED record | <reason>`, amber, for a DEGRADED verdict, cut to two rows by `sourceRows()` (:364)
  with ` ...` when longer;
- `kMeasuredOk` (:307-308), green, otherwise.

The serial `library graph BUILT from the …` line (:752-756) chooses by the same source.

T3 Part 2's worktree keeps this unchanged (`.claude/worktrees/t3-part2/src/teleop_program.cpp`
:633-647 for the choice, :790-795 for the serial line). Its LCD adds CLAW ABSENT (:755-761). The audit's
second half no longer applies either: the ipr and track values running off the edge belonged to that
same row.

**What remains (D3):** nothing host-tests which words the panel shows for which source, so the defect
could come back unseen. Part B2 adds the pin. It also adds the BACKUP wording that D2 needs.

### 0.4 What already exists to build on
| Piece | Where (opened) | Use |
|---|---|---|
| The record format | `include/shulib/calibration/calibration_record.hpp`: `kRecordFileName` :81-84, `kRecordBytes` = 180 :99-104, `crc32Ieee` :106-121, five named refusals :14-26 and ~:206-255, golden in `test/calibration_record_test.cpp` | "decodes" = all five checks pass. **Layout and version untouched** |
| The block seams and adapters | `hal/block_sink.hpp` (bool write, flush), `hal/block_source.hpp`; `hal/pros/block_sink.hpp` (ctor :79-93 with `path_` :89 and `"wb"` :91; unreportable fclose :95-111; `path()` :148-149), `hal/pros/block_source.hpp` (ctor :71-85, `"rb"` :83, `readAll` seeks to 0 :107-115, `path()` :121-122); the fakes `hal/fake/fake_block_sink.hpp` (a false flush KEEPS the bytes, :41-52), `fake_block_source.hpp` (`setShortRead`, `setOpen`, `readCalls`) | `ProsFileStore` composes the two PROS adapters, using THEIR default root and THEIR `path()`, so the `/usd/` prefix stays owned in those two places (HA-122; `kRecordFileName`'s comment :81-83 says so) |
| Preconditions | `include/shulib/core/check.hpp` :9-21 (both policies THROW `PreconditionError`) | Why the store is not `noexcept` (§1.A.1) |
| The PROS shim | `test/pros_shim/pros/misc.hpp` :110-114, :193-199 (`usd::is_installed()` programmable) | `ProsFileStore`'s host tests, card present or not |
| Today's save | `src/calibrate_program.cpp` `saveAndReadBack` ~:627-684; `finishDone` ~:841-909; `run.save = finishDone(...)` ~:974; the P5 banner ~:109-111 | B1 replaces the save with the library call |
| The boot's read | `src/boot_source.hpp` `RecordRead`/`readRecord` :215-255 (three sentences: not open / EMPTY / decoder's reason, each naming `path`), `decideBootSourceFrom` ~:296-374; `src/boot_source.cpp` `decideBootSource` :34-53 (`ProsBlockSource card{kRecordFileName}` :38, `readRecord(card, card.path())` :39) | B2 reads the primary file, and otherwise the backup, **through one rule** |
| The panel's outcome | `calibrate_panel.hpp` `SaveOutcome` :201-206; the Done screen ~:796-844 | Unchanged (D5) |
| The boot's host test | `test/boot_source_test.cpp` test 6: harness :548-615 (`kCardPath` :556, `runBoot` over a `FakeBlockSource`), 6k (robot two's card and census, the whole boot), 6m (a port-check refusal never falls back), 6n/6o (no card, `lines.size() == 3`), 6p (EMPTY), 6q (TRUNCATED) | B2 moves the harness to `FakeFileStore`; the LINES and counts are the pin |
| Robot two's record, as a fixture | `test/boot_source_test.cpp` ~:111-148 (anonymous namespace :79-318): the description REBUILT from the 2026-09-25 capture's printed values (Calibrate wrote it 2026-09-24 17:42:16, per its provenance) | Part A COPIES it (§2), and encodes it; it is not the card's own bytes |
| A comment stripper | `test/mechanism_table_test.cpp` `codeOnly` ~:170-200 | Part A COPIES it for the platform scan (§2) |
| The fence test | `test/pros_adapter_fence_test.cpp` :71-120 | Enforces the two-flag fence on every `hal/pros/*.hpp`, the new one included |
| The Bench Tests SD probe | `src/bench_r3a.cpp` `probeSdCard` :592-630 (button 5, worksheet G.0.1) | Where HA-166's settle (b) belongs; unowned |

---

## 1. Scope — IN

### Part A — the library (PROS-free except the one adapter; no `src/` file)
1. **`include/shulib/hal/file_store.hpp` — `hal::IFileStore`,** the card as the brain can actually use
   it:
   - `cardPresent()` → `bool`.
   - `read(bareName, into)` → `StoreRead { bool opened; std::size_t count; char path[96]; }`, reading
     from the start. `path` is the path the read used, **for messages only**. A longer path is cut and
     the operation is unaffected: the adapter holds the full path.
   - `replace(bareName, bytes)` → `StoreReplace { ReplaceStep failed; char path[96]; }`, with
     `ReplaceStep { None, NoCard, Open, Write, Flush }`. It opens **truncating**, writes, flushes and
     closes. The header says:
     - **the old contents are gone from the open onward (HA-166 (1));**
     - **`None` does not mean the bytes are on the card:** the close cannot report, and only a
       read-back proves anything (HA-166 (5)).
   - The header states why there is **deliberately no rename, no remove, no exists-by-`stat` and no
     read-write open**: HA-166 (2) and (3). A file exists when `read` opens it.
   - The header also says why this is a sibling of `IBlockSink`/`IBlockSource` and not a change to them:
     those two are single-file handles opened at construction, and the safe save needs two named files.
   - **Exceptions: the methods are NOT `noexcept`.**
     - An implementation may throw `PreconditionError` for a name that is not bare, plus whatever its
       adapters' construction throws (`check.hpp` :9-21; the adapters' constructors allocate `path_`).
     - A refusal by the CARD is always a returned value, never a throw.
   - Additive; not one of the frozen F4 seams.
2. **`include/shulib/hal/pros/file_store.hpp` — `hal::pros::ProsFileStore`.**
   - **`read` builds a `ProsBlockSource`,** each in its own scope. Not open → `opened = false`; open →
     `readAll`. `path` is copied from `source.path()`.
   - **`replace` builds a `ProsBlockSink`,** each in its own scope. Not open → `NoCard` if
     `cardPresent()` is false, else `Open`; then `Write` or `Flush` as they fail. `path` is copied from
     `sink.path()`.
     - Without a card, neither adapter calls `fopen` (block_sink.hpp :90), so nothing is created.
     - The sink closes at the end of its scope. Its failure cannot be reported (`block_sink.hpp`
       :95-111), and PROS's close returns 0 regardless (§0.2). So **`replace` makes no claim about the
       close**; the caller's read-back is the proof.
   - **The root.** Default-constructed, it constructs each adapter with ONE argument, so the adapter's
     own default `"/usd/"` applies. `explicit ProsFileStore(const char* mountRoot)` passes a root, for
     host tests against a real temp directory. **No `"/usd/"` literal of its own.**
   - **`cardPresent()` is `::pros::usd::is_installed() != 0`,** the adapters' own probe. The header
     includes `pros/misc.hpp` itself, behind **exactly** the `-Wshadow` + `-Wsign-conversion` fence
     (`test/pros_adapter_fence_test.cpp` :71-120 enforces that set on every `hal/pros/*.hpp`).
   - **No `fopen` and no mode string of its own.** The only modes stay in the two adapters, and the
     scan enforces it (§2).
   - The bare-name precondition fires from the adapters' constructors, so it propagates. This is why
     the methods are not `noexcept`.
3. **`include/shulib/hal/fake/fake_file_store.hpp` — `hal::fake::FakeFileStore`,** in-memory named files
   (host-test only; `std::vector` is fine). It is constructed with a display root, which tests pass as
   `"/usd/"` so the boot's sentences come out as on the brain.
   - **Reported faults:** fail the n-th operation, as today's fakes do:
     - open refused (the file is untouched);
     - short write keeping the prefix;
     - flush false (the bytes are KEPT, as `FakeBlockSink` does);
     - short read.
   - **SILENT faults** (new at the re-check; these are what the read-backs exist for):
     - (i) a `replace` that reports `None` but leaves the stored bytes cut to j, or with one byte
       changed. This is the unreportable close.
     - (ii) a `read` that reports `opened` and returns bytes with one byte changed.
   - **Crash:** at operation k, leave the file being replaced as one of:
     - absent;
     - empty;
     - its first j new bytes (j = 1 … n−1, n = the new length);
     - all new bytes;
     - TORN: the old length, with new bytes up to j and old bytes after.

     The crashed `replace` itself reports `Write`: it failed past its open. A crashed read reports not
     opened and changes no file. After the crash, **every** later operation is refused: `cardPresent()`
     false, reads not opened, replaces `NoCard`. That is also a PULLED card while the program keeps
     running. The test then "reboots": it clears the refusal and reads the card.

     **A variant keeps `cardPresent()` true after the crash.** That is a card that stops answering while
     VEXos still reports it; HA-122 (1) was confirmed only at boot, never for a card pulled mid-run.
     Every later read is then not opened and every later replace reports `Open`.
   - Every other file is unchanged. The model assumes HA-166 (4) and (5) and says so.
   - An **operation log** (file, operation, bytes, result) for the order assertions.
   - No rename method, because the brain has none.
4. **`include/shulib/calibration/record_store.hpp`** (PROS-free), plus
   **`kRecordBackupFileName = "shulib_drivetrain.bak"`** added **in `calibration_record.hpp` beside
   `kRecordFileName`**, with a `static_assert` that both names are bare (no `/`) and differ. That keeps
   the rule of one name, in one place, for both programs. **Layout, `kRecordVersion` = 1 and the golden
   stay byte-identical.**

   Below, P is the primary file and K the backup. `accept(bytes)` means "this is a complete, valid file
   of this kind". For the record it means `decodeRecord` succeeds, which also refuses any length other
   than 180. Contents:
   - **`chooseEffective(pAccepted, kAccepted)`** → `Which { Primary, Backup, None }`: P if accepted,
     else K if accepted, else none. **THE load rule, in one place.** Everything below uses it.
   - **`loadWithBackup<MaxBytes>(store, primary, backup, accept)`** → **`BackupLoad<MaxBytes>`** — the
     byte-level load. It reads P, and **reads K only when P is not accepted**, then applies
     `chooseEffective`. It is generic, so L7 can reuse it (D6). *(Named by the coordinator at the
     cross-brief reconciliation, 2026-09-26 late evening — L7's C.3 builds on these exact names.)*
     ```
     struct FileRead {              // one file, as the load saw it — generic over the file's kind
         bool tried = false;        // the load read this file (K only when P was not accepted)
         bool opened = false;       // false: no card, or no such file
         std::size_t count = 0;     // bytes read; 0 = an open but EMPTY file
         bool accepted = false;     // accept() took it
         char why[192] = "";        // accept()'s refusal when opened, count > 0 and not accepted
         char path[96] = "";        // from the store's read (the adapter's path() on the brain)
     };
     template <std::size_t MaxBytes> struct BackupLoad {
         Which from = Which::None;                           // Primary / Backup / None
         FileRead primary{}, backup{};
         std::size_t count = 0;                              // the chosen file's length
         std::array<std::uint8_t, MaxBytes + 32> bytes{};    // meaningful unless from == None
     };
     // accept: bool(std::span<const std::uint8_t> bytes, char* why, std::size_t whyCap)
     ```
     `BackupLoad` is a return value, so it lives in the CALLER's frame: the same `MaxBytes + 32` stack
     rule as `safeReplace` applies, and the header states it.
   - **`safeReplace<MaxBytes>(store, primary, backup, bytes, accept)`** → a `SaveReport`. It is
     byte-level and generic (D6).

     **`put(X, b)`** means: `replace(X, b)`, keeping the step it reports for the words; then **always**
     `read(X)`. `put` **succeeds if and only if** the read opened and returned exactly `b` (same length,
     same bytes) and `accept` takes it. **The decision never uses the step `replace` reported** (D8 (a)).

     **Last seen:** for each file the save keeps what it last read of that file: opened or not, its
     length and its CRC-32 (`crc32Ieee`). It is updated by these rules:
     - a read that OPENS sets it to (opened, length, CRC-32);
     - a read that does NOT open, after a `replace` of that file that got PAST its open (reported
       `Write`, `Flush` or `None`), sets it to **UNKNOWN**. The save has just written that file and
       there is no delete, so the refused read is a failed read, not a measurement of absence;
     - a read that does not open in any other case leaves it as it was: after `NoCard` or `Open` the
       file was not touched, and a refused read does not show it is gone.

     The save also keeps whether ANY operation in it got a file open (a read that opened, or a
     `replace` past its open).

     The sequence:
     0. `bytes` must be accepted, and `cardPresent()` must be true. Otherwise stop, NOT SAVED, with
        nothing opened.
     1. **READ THE STARTING CARD: read P and K, both, always.** K's read is the baseline step 4 checks
        against. `old` := P's bytes if P is accepted.
        - **BACKUP FIRST:** if `old` exists and K is not byte-identical to it (same length, same bytes),
          `put(K, old)`. **If that fails, stop NOT SAVED with P never opened for writing.**
        - If P is **not** accepted, **K is not written in this step.** It may hold the only good record.
     2. `put(P, bytes)`. If it fails and `old` exists:
        - if P's read-back from that `put` is already byte-identical to `old`, P was not changed, and
          there is **no restore**;
        - otherwise `put(P, old)`. This is the restore.
     3. If step 2 succeeded, **refresh the backup:** `put(K, bytes)`. Its failure changes nothing about
        P.
     4. **MEASURE, never assume.** Read P and K again. The outcome is **UNCONFIRMED** (D8 (b)), and
        nothing more is claimed about which record the next boot uses, when any of these holds. Each
        has its own words (§1.A.5):
        - **(u1) the card is gone:** `cardPresent()` is false;
        - **(u2) nothing could be opened:** no operation in the whole save got a file open (the card
          is present but refuses every open: write-protected, full or failing);
        - **(u3) the card did not read back the same way twice:** a file does not read back as its
          last-seen state (opened or not, length, CRC-32), or an UNKNOWN file again does not open.

        Otherwise `chooseEffective` names the file the next boot will use, and whether its bytes are
        the new ones, the ones in effect before the save, or others.
        - **SAVED ⇔ the effective file is P and P holds `bytes`.**
        - The backup's state is measured: the same bytes as P, another accepted file, or not accepted
          or absent.

     **`SaveReport`** carries:
     - the outcome (Saved / NotSaved / Unconfirmed);
     - which file was in effect before, and whether it was P or K;
     - the measured effective file and its bytes (new / earlier / other);
     - the backup's measured state;
     - whether P was opened for writing, and whether a restore ran and read back;
     - every step a `replace` reported failing (file, step);
     - on Unconfirmed, which file disagreed;
     - the two paths.

     Every field is filled from the log of what happened and the step-4 measurement. None comes from
     what a step meant to do.

     **Buffers and stack.** `safeReplace` is a function template on `MaxBytes`, the largest file of the
     caller's kind. It holds exactly **two** local arrays of `MaxBytes + 32` bytes: `old`, for the
     restore, and one read scratch that every read reuses. A longer file is therefore a LENGTH refusal
     (the existing rule: `boot_source.hpp` ~:240-242). Last-seen states are (opened, length, CRC-32),
     never copies. For the record that is 2 × 212 = 424 bytes of stack, and the header states it.
     `static_assert(MaxBytes <= 1024)`, so no caller puts a large file on a task stack by accident. L7's
     gains file must fit, or raise the cap with its stack cost stated.
   - **`saveRecordSafely(store, description)`** — encodes. It refuses an invalid description or a failed
     encode before touching the card. It calls `safeReplace<kRecordBytes>(kRecordFileName,
     kRecordBackupFileName, …)`.
   - **`loadRecord(store)`** → `LoadedRecord`: `loadWithBackup<kRecordBytes>` over the record's
     `accept` (which writes `decodeRecord`'s refusal into `why`), then the decode of the chosen bytes.
     `RecordFileRead` is `FileRead` with `accepted` read as "decoded" — the executor may make it an
     alias or copy the fields; the words below use "decoded". The structure:
     ```
     struct RecordFileRead {       // one file, as the load saw it
         bool tried = false;       // the load read this file (K only when P did not decode)
         bool opened = false;      // false: no card, or no such file
         std::size_t count = 0;    // bytes read; 0 = an open but EMPTY file
         bool decoded = false;
         char why[192] = "";       // decodeRecord's refusal when opened, count > 0 and not decoded
         char path[96] = "";       // from the store's read (the adapter's path() on the brain)
     };
     struct LoadedRecord {
         Which from = Which::None;                  // Primary / Backup / None
         hal::DrivetrainDescription description{};  // meaningful unless None
         RecordFileRead primary{}, backup{};
     };
     ```
     With this, `boot_source.hpp` can rebuild today's three sentences for P exactly. The sentences stay
     in `src/`; the library supplies opened, count, why and path.

   **Contract of the whole header:**
   - It allocates nothing.
   - It is **exception-neutral**, not `noexcept`: it catches nothing, and throws only what the store
     throws, plus `SHULIB_PRECONDITION` on its own arguments (null names, equal names, a `bytes` longer
     than `MaxBytes`).
   - `accept` is a template parameter (a callable), never `std::function`.
5. **`formatSaveReport(report, BootReadsBackup, emit)`** — the serial lines for each outcome. ASCII only,
   no `%` in the output, and a line short enough for Calibrate's `emitf`.
   - `BootReadsBackup` (`No` / `Yes`, no default) says whether the Teleop build this Calibrate ships
     with reads the backup (D4: B1 passes `No`, B2 passes `Yes`).
   - **Every sentence is derived from the measured report.** The branches:
     - **SAVED**, with the backup holding the same bytes.
     - **SAVED, but the backup does not hold them.** The measured state is named (an older record /
       damaged / missing), and the next save refreshes it. Teleop boots from the record file.
     - Either SAVED line, plus **"the card reported <step> failing on <file>, yet it reads back whole"**
       whenever a `replace` reported a failure (D8 (a)).
     - **NOT SAVED:**
       - before the card was touched: no card, the description is not valid, or it would not encode
         (nothing opened);
       - the record file was not changed by this run: the earlier record, if any, is still on the card
         and Teleop boots from it as before;
       - the earlier record was written back and re-read, and Teleop boots from it;
       - the record file is damaged and the earlier record is safe in the backup:
         - `Yes`: Teleop boots from the BACKUP (amber);
         - `No`: this Teleop does not read the backup and will REFUSE. Rerun Calibrate, or on a laptop
           copy `shulib_drivetrain.bak` over `shulib_drivetrain.bin`;
       - there was no usable record file before, and an older backup copy exists: `Yes`, Teleop boots
         from it; `No`, as above;
       - nothing usable is on the card: Teleop will refuse until a run saves.
     - **SAVE UNCONFIRMED.** One of three reasons is named:
       - (u1) the card is no longer in the brain;
       - (u2) the card is present but nothing on it could be opened or created (write-protected, full
         or failing);
       - (u3) the card did not read back the same way twice (pulled, or failing).

       Then, in all three cases: this run cannot say which record Teleop will use. Put the card back if
       it was pulled, copy it on a laptop, then boot Teleop and read its `calibration record:` line.
   - **The formatter is total.** A measured combination with no named branch prints the measurement
     itself: which file the next boot uses and whether it holds this run's bytes.
   - **The sentence "is still on the card" may only be printed when the measurement says the earlier
     record is untouched or was written back and re-read.**
   - Under D4's alternative (one Part B), drop `BootReadsBackup` and word only the `Yes` branches.
6. **Documentation (§3.1):**
   - HA-166 (§3.5), the FAQ, the changelog, the regenerated API reference.
   - Fix the now-false "NO HARDWARE CLAIM" paragraph in `calibration_record.hpp` (:52-54). Since
     2026-09-25 a record that Calibrate wrote on robot two has decoded at eight Teleop boots
     (`R3d-PROGRESS.md` ~:2463-2465). Cite that by date.
   - One sentence in `block_sink.hpp` :37-40 ("no fsync in PROS's exposed surface" stays true; PROS
     syncs after every write it passes on, HA-166 (5)), and the same cross-reference in HA-122's entry.

### Part B1 — Calibrate's save (after T3 Part 2 AND G1 Part 2c-final have landed; re-read every line below against the tree then)
1. **Calibrate.**
   - `finishDone()` (~:883-906) saves through `calibration::saveRecordSafely` over a default-constructed
     `ProsFileStore`.
   - **`saveAndReadBack()` (~:627-684) is REMOVED.** Its read-back now lives in the library.
   - The panel is told `SaveOutcome::Saved` **if and only if** the report's outcome is Saved. NotSaved
     and Unconfirmed both give `NotSaved` (D5).
   - The serial outcome lines come from `formatSaveReport(report, BootReadsBackup::No, …)`, so the false
     sentence (~:903-904) is gone.
   - Update the header comments (~:9-10, ~:109-111), the `record file:` line (~:1064 at HEAD; ~:1087 in
     2c-final's tree) and **2c-final's `calibrateInitialize` banner** ("writes them to /usd/%s", ~:1037-1057
     in its tree). Each must name both files and the order.
   - `finishStopped()`'s "Nothing was written to the card by this run" (~:919) stays. It is true.
2. **Worksheet** (internal), Station G:
   - rewrite the §3.0 interim lines (G.0.1 and :849-850) once the save is safe;
   - G.5.1's reasons and the new outcome words (SAVED with a reported failure; SAVE UNCONFIRMED);
   - one line on what `.bak` is, and "to reset, delete both files";
   - Station 6 (:265, SD card prep): the card holds the record and its backup.

### Part B2 — the boot and Teleop's words (after T3 Part 2, G1 Part 2c-final AND COMP-b have landed, and after B1; re-read every line below against the tree then)
1. **The boot.**
   - `src/boot_source.{hpp,cpp}` read through `calibration::loadRecord` over `ProsFileStore`. The card is
     still read, and closed, before the registry is asked anything (`boot_source.cpp` :35-40's rule).
   - `readRecord(IBlockSource&, path)` is replaced by a builder from `LoadedRecord`, so there is one
     read path.
   - The outcome records which file was used (an APPENDED field).
   - **P's three sentences stay byte-identical,** built from `primary`'s opened/count/why/path
     (6k/6p/6q are the pin).
   - **An absent backup (not opened) adds no line.** A backup that opened adds **exactly one** line: its
     use, or its reason. The refusal headline's reason stays P's.
   - A backup that decodes but fails its port check is **REFUSED**, never the table: P3, extended.
   - `calibration::chooseBootSource` and `boot_check.hpp` are **not** changed. A decoded backup is still
     "a record".
   - Calibrate's `formatSaveReport` call flips to `BootReadsBackup::Yes` in this change.
2. **Teleop — the pin for part (2), and the BACKUP wording.**
   - The source rows' words and tone come from **one pure function** beside the boot outcome, in
     `boot_source.hpp` (host-tested; COMP's status screen uses it in this change). Its cases:
     - TYPED TABLE, amber (fixed text);
     - MEASURED record from the SD card, verified against the ports, green (fixed);
     - MEASURED record | <DEGRADED reason>, amber (runtime);
     - **MEASURED record from the BACKUP copy on the SD card, verified against the ports, amber (fixed);**
     - **MEASURED record (BACKUP copy) | <DEGRADED reason>, amber (runtime).**
   - **Fit:**
     - each FIXED text fits two SMALL rows: `wrapWords<kSmallCols, 2>(…).complete` and `screenText`,
       checked at compile time as `teleop_program.cpp` checks its own (:336-338 at HEAD);
     - the RUNTIME texts go through `sourceRows()`, whose cut at a whole sentence with ` ...` T3 Part 2
       pins (`t3-part2` :450-457). One more `static_assert` beside it pins robot two's DEGRADED reason
       with the BACKUP prefix.
   - `paintAll()` calls that function.
   - The serial `library graph BUILT from the …` line names the backup.
   - **The LCD's third row** adds `BACKUP see brain` at D7's place: fault > no controller > DEGRADED >
     CLAW ABSENT > BACKUP > ok. The choice becomes a `constexpr` function in `teleop_program.cpp`, with
     `static_assert`s for the precedence.
3. **Worksheet** (internal):
   - F.0 :538 ("It reads `/usd/shulib_drivetrain.bin`"): it names the backup and when it is used. F.0's
     refusal table does NOT gain a BACKUP row, because a BACKUP boot is not a refusal.
   - F.1 and G.6 say what an amber BACKUP row and `BACKUP see brain` mean, and what to do (rerun
     Calibrate).

---

## 2. Tests (adversarial) and mutations

**Part A.** Starting cards, each with its **LITERAL expected record in effect before the save**, written
in the test. The expectation is never computed by calling the code under test.

| # | P (record file) | K (backup) | In effect before |
|---|---|---|---|
| 1 | absent | absent | none |
| 2 | absent | good | K |
| 3 | good | absent | P |
| 4 | good | equal to P | P |
| 5 | good | older good | P |
| 6 | good | short (100 bytes) | P |
| 7 | good | 180 bytes, bad CRC | P |
| 8 | short (100 bytes) | good | K |
| 9 | 180 bytes, bad CRC (a TORN one: new bytes to 90, old after) | good | K |
| 10 | empty (0 bytes) | good | K |
| 11 | a good record plus 8 bytes (a LENGTH refusal) | good | K |
| 12 | short | short | none |
| 13 | 180 bytes, bad CRC | 180 bytes, bad CRC | none |

"Good" and "older good" are robot two's description, COPIED from `test/boot_source_test.cpp` ~:111-148
(cite it; it was rebuilt from the 2026-09-25 capture, written 2026-09-24 17:42:16) and encoded, plus
one variant with a different track width. Do not move the fixture: both in-flight chunks edit that file
(landmine 12).

- **THE CRASH-POINT TEST (the load-bearing one).** Record the operation log of a clean save. Then crash
  at **every index** of that log, with every leave-as state in §1.A.3, from every starting card, in
  BOTH crash variants (the card reported gone, and still reported present). After the "reboot":
  - `loadRecord()` returns **byte for byte** either the literal record in effect before, or the new
    one. It never returns nothing when something was in effect before, and never a mixture.
  - The report, produced while the card was "pulled", is either **Unconfirmed**, or it names exactly the
    file and bytes the rebooted load returns. With the card reported gone, it is always Unconfirmed
    (u1). With the card still reported present, (u2) and (u3) must carry it. A crash at the save's very
    first read is the case that needs (u2).

  Enumerating from the recorded log means an operation added later is covered automatically.
- **ONE FAULT PER RUN:** every reported and silent fault in §1.A.3, at every index, from every starting
  card. Check the invariant above, and also:
  - **Saved ⇒** the rebooted load returns P with the new bytes. "The backup holds the same bytes" ⇔ K
    re-reads byte-equal to P.
  - **NotSaved ⇒** the rebooted load returns exactly the file and bytes the report named, and they
    equal the literal record in effect before.
  - **Unconfirmed ⇒** either the injected fault was on a READ (refused, short or altered), or no file
    was opened anywhere in the save (u2: for example card 1 with its only create refused). A write
    fault never produces (u3), because the read-back sees what the write did.
- **SILENT WRITE, THEN A CRASH (the pairs):** a silent fault (i) at index i, then a crash at every index
  after i, with every leave-as state, from every starting card. The invariant must hold. (This is what
  step 1's read-back exists for.) A READ fault followed by a crash is NOT required to hold; it is the
  two-fault limit in D1 and landmine 13. Log that it was excluded, and why.
- **Order, from the log:**
  - **every `replace(X)` is followed by a `read(X)`** before any other file is replaced;
  - **P is never opened for writing** unless, earlier in the same save, P's step-1 read was not
    accepted, or a read of K after K's last write returned bytes identical to `old` (or K's step-1 read
    already did);
  - **before P verifiably holds the new bytes, K is written only with `old`, and never in a save whose
    step-1 read of P was not accepted** (step 3 may then write K);
  - no write to K in step 1 when K already equals P;
  - the restore's `replace(P)` appears only when P's read-back after step 2 differed from `old`.
- **Directed tests:**
  - good P (card 3 and card 5), step 2's write cut short ⇒ the rebooted load returns **P, not K**
    (the log shows no read of K at the boot), byte-equal to `old`, and the report is "written back and
    re-read";
  - good P, step 2's open refused ⇒ no second `replace(P)`, and the report is "the record file was not
    changed";
  - a false flush on step 2 with its bytes kept ⇒ Saved, plus the reported-failure line (D8 (a));
  - card 2, the card pulled at step 2 (every later operation refused, the program running) ⇒
    Unconfirmed with (u1)'s words, never "nothing usable";
  - card 5, the card stops answering at step 3 while still reported present ⇒ Unconfirmed with (u3)'s
    words;
  - card 5, the card stops answering at the save's first read while still reported present ⇒
    Unconfirmed with (u2)'s words, never "nothing usable";
  - card 1, the create of P refused (`Open`) ⇒ Unconfirmed with (u2)'s words;
  - card 2, a short write at step 2 ⇒ NotSaved, "there was no usable record file before, and an
    older backup copy exists", naming K's bytes as the record in effect before;
  - a silent cut on step 3's write ⇒ Saved, with the backup measured "damaged";
  - card 5, a short read on step 4's read of P, with the card otherwise fine ⇒ Unconfirmed with (u3)'s
    words. The general oracle alone would accept a Saved here, because the rebooted load does return
    the new record; this test pins that the report never outruns its measurement.
- **Edges:**
  - no card ⇒ nothing opened, NotSaved;
  - an invalid description or a failed encode ⇒ nothing opened;
  - K longer than P but sharing its first 180 bytes ⇒ K is rewritten;
  - a K that differs only in its CRC bytes ⇒ K is rewritten;
  - at boot, P decodes ⇒ the log shows no read of K;
  - both damaged ⇒ both reasons named, and `primary.why`/`backup.why` are the decoder's;
  - `static_assert` on the two names.
- **`ProsFileStore`, through the shim and a real temp directory:**
  - replacing a LONGER existing file leaves exactly the new 180 bytes;
  - no card (shim `installed = false`) ⇒ `NoCard`, the read is not opened, and **no file is created**
    in the directory;
  - a leading `/` ⇒ `PreconditionError` from `read` and from `replace`;
  - a round trip of the awkward bytes (NUL, LF, CR, 0x1A, 0xFF);
  - the result's `path` is the adapter's: `<tmp>/shulib_drivetrain.bin` with an injected root, and
    **`/usd/shulib_drivetrain.bin` from a default-constructed store with the shim's card absent**, so
    nothing is touched. This pins that the path comes from the adapters' own default.
- **The platform scan** over `include/shulib/**` and `src/**`, code only. It strips `//` and `/* */`
  comments, TRAILING ones included (a naive regex matches `motion_scheduler.hpp` :1217's "innermost link
  (validation hook)"). Use a COPY of `test/mechanism_table_test.cpp`'s `codeOnly` (~:170-200), cited
  (landmine 12). It checks:
  - no call to `rename`, `remove`, `unlink`, `link`, `stat` or `mkdir`;
  - **`fopen` appears only in `hal/pros/block_sink.hpp`, `hal/pros/block_source.hpp` and
    `src/bench_r3a.cpp`** (its SD probe);
  - **no `fopen` mode string contains `+`** (HA-166 (3)).

  It passes at HEAD: checked at the re-check; the only raw match was that trailing comment. Prove that
  it catches a violation with mutations 12b and 13.
- **Formatter:** a golden line for every branch, under both `BootReadsBackup` values; ASCII only; no
  `%`; "still on the card" appears only when the measurement says the earlier record is untouched or
  restored; an enumeration of every measured combination prints something, and no combination prints a
  claim that contradicts it.

**Part A mutations.** Run each on a fresh binary, observe RED, restore, and log it. What kills each is
given after the arrow.
1. The record is written before the backup (steps 1 and 2 swapped) → the order check; the crash test.
2. Step 1's `put(K, old)` skips its read-back → "every replace is followed by a read"; the
   silent-write-then-crash pairs.
3. K is written in step 1 from an unaccepted P → the order check; card 9 in the crash test.
4. The restore is dropped → the directed test (the boot reads P, not K).
5. Step 3 is dropped → Saved ⇒ "the backup holds the same bytes" is false on re-read.
6. `chooseEffective` prefers K when both are accepted → card 5's literal expectation.
7. `loadWithBackup` reads K when P is accepted → the boot shows a read of K in the log.
8. The step-1 skip compares length only → "K differs only in its CRC bytes ⇒ rewritten".
9. `accept` is length-only → card 9 (the order check: K overwritten from a bad-CRC P; the crash test).
10. Step 4 is skipped and the report is built from the steps' own results → the directed short read on
    step 4's read of P (it must be Unconfirmed); the silent cut on step 3 (its golden must say
    "damaged").
11. Step 2's `put` skips its byte comparison → a silent cut on step 2 (the boot must read P = `old`).
12. (a) `ProsBlockSink`'s `"wb"` becomes `"ab"` → "replacing a LONGER file leaves exactly 180 bytes".
    (b) An `std::fopen(…, "r+b")` is added to `ProsFileStore` → the scan, on both counts.
13. A `std::rename` call is inserted in `ProsFileStore` → the scan.
14. The formatter's "still on the card" sentence is used for a BACKUP branch → the formatter goldens.
15. Step 4's check (u3) is removed → card 5 stopping at step 3 while still reported present.
16. The restore runs even when P already reads back as `old` → the open-refused directed test.
17. Step 1 reads K only when P is accepted → card 2 with a short write at step 2. That must be
    NotSaved, naming the BACKUP as the record in effect before, with its branch's words. Without K's
    step-1 read the save cannot know K held the earlier record.
18. `put`'s success is taken from the step `replace` reported → the false-flush directed test must be
    Saved.
19. Check (u2) is removed → card 5 stopping at the save's first read while still reported present.
20. A refused read-back after a `replace` past its open leaves last-seen as "not opened" instead of
    UNKNOWN → card 1 in the card-present crash variant, crashed at step 2's write leaving all the new
    bytes.
21. Check (u1) is removed → the card-2 pull test's golden words (u3's words print instead).

**Part B1.**
- **A source-text pin** (new test file, over a COPIED `codeOnly`; landmine 12):
  - `calibrate_program.cpp` calls `saveRecordSafely` once;
  - it constructs no `ProsBlockSink` and no `ProsBlockSource` for the record;
  - `saveAndReadBack` is gone;
  - `SaveOutcome::Saved` is chosen from the report's outcome;
  - `formatSaveReport` is called with `BootReadsBackup::No`.
- `test/mechanism_table_test.cpp`'s census pins (~:955-1000) stay green without edits (landmine 9).

**Part B1 mutations:**
1. `finishDone` tells the panel Saved when the report says NotSaved or Unconfirmed.
2. `BootReadsBackup::Yes` is passed in B1 (it claims a fallback the boot lacks).
3. The old save path is left in place beside the new one.

**Part B2.**
- **Test 6, extended.** The harness moves from `FakeBlockSource` to `FakeFileStore` (root `"/usd/"`).
  The expected LINES and outcomes of 6k–6r do not change. A counter assertion of the old fake (6n's
  `readCalls() == 0`) is restated for the new fake: with no card, no read opens either file. New cases:
  - a TRUNCATED record plus a backup holding robot two's record ⇒ boots from the BACKUP; the port check
    runs on the backup; exactly one added line names it; `built` carries it;
  - a record that decodes but fails its port check, plus a good backup ⇒ REFUSED, with the backup never
    read;
  - a 180-byte bad-CRC record plus a damaged backup ⇒ both reasons, and the headline's reason is P's;
  - neither file present ⇒ 6n and 6o, byte-identical, with the same line counts;
  - a good record ⇒ 6k's lines byte-identical, with the backup not read.
- **The source-words function:** every source × decision × file.
  - **"table"/"TABLE" appears if and only if the source is `UseTable`** (the 2026-09-25 defect's pin).
  - BACKUP, TABLE and DEGRADED are amber; green only for the primary file with Ok.
  - The fixed texts fit at compile time; the runtime texts are pinned through `sourceRows()`.
- **The LCD choice:** `static_assert`s for each step of D7's precedence.
- **The pin:** Calibrate now passes `BootReadsBackup::Yes`.

**Part B2 mutations:**
1. The boot prefers the backup over a decodable record.
2. The boot falls back to the backup after the record's port-check refusal.
3. The source function returns the table's words for a record.
4. BACKUP painted green.
5. A primary-file boot sentence edited, so 6p must go red.
6. An absent backup prints a line, so 6n's count must go red.
7. The LCD puts BACKUP above CLAW ABSENT (a `static_assert` goes red).
8. Calibrate still passes `BootReadsBackup::No`.

Use **the stale-binary recipe** for every mutation (HANDOFF §3):
1. `touch` every test file that includes the edited header.
2. Build `--target shulib_tests/fast`.
3. **Require `Linking CXX executable shulib_tests` in the output.**
4. Run it and observe RED.
5. Restore, relink and see GREEN.
6. Regenerate the briefing only after the last restore.

For a `static_assert` mutation, a failed compile IS the red. Log the error line.

---

## 3. Documentation (same change)

### 3.0 At this brief's commit (the coordinator; HANDOFF §5's rule for new work)
These correct what is false TODAY. They are not code.
- **`R3a-BENCH-WORKSHEET.md` :849-850** — replace "a run that stops, or says NOT SAVED, leaves the
  previous record where it was" with:

  > **A run that STOPS writes nothing. A run that says NOT SAVED may already have ERASED the record
  > that was on the card** (found 2026-09-26; the program's own NOT SAVED text says the opposite and is
  > wrong until the SAFE-SAVE fix to Calibrate's save is in the build). Copy the record off the card
  > first (G.0.1).
- **G.0.1** (~:717-719) gains:

  > **Before any Calibrate run on a card that already holds a record, copy `shulib_drivetrain.bin` off
  > the card on a laptop, and date the copy.** Until the SAFE-SAVE fix to Calibrate's save is in the
  > build, a NOT SAVED run can erase it, and that copy is the only way back.

  Both edits sit away from T3 Part 2's Station G hunk (:832) and are expected to merge clean; check it.
- **HANDOFF §5** gains the item, with the same copy-first instruction.
- **`build-order.md`**: add the chunk under the DOCS/DEFECTS row, and move the count 54 → 55 with its
  reason: "reactive: a failed Calibrate save could erase a good record, and the brain cannot rename a
  file".
- **A deviations-table row** for the order (D4).
- **The status board:** item (2) closed once the team lead answers D3; the estimate becomes A ~1–1½ d,
  B1 ~½ d, B2 ~½ d.

HANDOFF, `build-order.md` and the worksheet are edited in both in-flight worktrees. Carry these edits
through their merges.

### 3.1 Part A
- `docs/hardware-assumptions.md`:
  - the **HA-166** index row and entry (§3.5), in a new "SAFE-SAVE's addition" section;
  - the status count and the "added" sentence (~:39, ~:71);
  - two cross-reference sentences in HA-122's entry (~:964-985). No status change:
    - PROS syncs after every write it passes on, HA-166 (5);
    - SAFE-SAVE's report also reads belief (1)'s `usd_is_installed()` while a program runs, which was
      never confirmed for a card pulled mid-run. If a pulled card still reads 1, the report falls back on
      its two other checks (§1.A.4, step 4: u2, u3), so no false claim rests on it.

  T3 Part 2 edits the same status paragraph (its `@@ -68`). 2c-final edits HA-111's row (:210) and its
  entries and HA-128. Reconcile at merge: **the status count becomes the combined one, and HA-166's row
  and section go after the highest number present at merge** (DIAG1-B's brief, its merge note 9).
- `docs/faq.md`, beside the `/usd/` prefix answer (~:118-134): "Can a program rename or delete a file on
  the SD card?" Answer: no, on PROS 4.2.2, so shulib keeps a verified backup copy. Evidence by date,
  never an internal path.
- `docs/changelog.md`: "Added since 2.2 — the calibration record's safe save". **No version bump:
  additive, nothing frozen.** It lists the new headers and says nothing has run on a robot. T3 Part 2
  inserts a section at the top too, so place the entry at merge, newest first.
- `docs/api/` regenerated (after merge; both worktrees change pages).
- `calibration_record.hpp` :52-54 corrected; `block_sink.hpp` :37-40, one sentence.
- The build-order entry's status (§3.0's entry). Reconcile at merge.
- The prose review, logged: grep `docs/`, `README.md` and the worksheet for `shulib_drivetrain`,
  `NOT SAVED`, `SAVED`, `still on the card`, `earlier record`, `rename`, `fsync`, `block_sink`,
  `block_source`, `IBlockSink`, `IBlockSource`. Read every hit and fix what is false, but only in files
  this part owns; hits in in-flight files go into the log as owed.

### 3.2 Part B1
- `R3a-BENCH-WORKSHEET.md`: Station G (G.0.1 and :849-850 rewritten from §3.0's interim text, G.5.1,
  the `.bak` line) and Station 6 (:265). Fix what is **false**. Log other staleness.
- `docs/hardware-assumptions.md` **HA-139** (~:1392-1414):
  - its settle becomes: after a SAVED run, **both** files are 180 bytes and byte-identical (`cmp`);
  - its blast radius names the library's save and the boot's load, not "the Done path of one program";
  - its hardware evidence (eight decoded boots, 2026-09-25) is recorded by date.
- `docs/guide/12-when-things-go-wrong.md`: a row for "Calibrate says SAVE UNCONFIRMED", and one for "it
  says NOT SAVED and the record file is damaged".
- `build-order.md` status; `roadmap.md` only if a row or "You are here" becomes false.
- The prose review, logged, for `calibrate_program.cpp`.

### 3.3 Part B2
- `R3a-BENCH-WORKSHEET.md`: F.0 :538, F.1, G.6 (§1.B2.3).
- `docs/guide/12-when-things-go-wrong.md`: a row for "Teleop says it booted from the BACKUP calibration"
  (what happened; what to do: rerun Calibrate).
- `docs/guide/11-reading-the-diagnostics.md`: the new boot lines, if it lists boot lines.
- `docs/guide/14-what-it-cannot-do-yet.md`: the save's safety rests on HA-166 (4) and (5); nothing of it
  has run on a robot.
- `docs/internal/HANDOFF-remaining-work.md` §2 (:108-112): "refuses to start without a SAVED calibration
  record on the card" becomes "… without a record that decodes: the record file, or, when that does not
  decode, its verified backup copy (amber BACKUP)".
- `build-order.md` status; `roadmap.md` only if a row or "You are here" becomes false.
- The prose review, logged, for every changed `src/` file.
- Leave every "never driven a robot under its own control" line as it is. That sweep belongs to R3.
- The coordinator republishes the status board.

### 3.4 Merge notes for the coordinator
- `docs/hardware-assumptions.md`: COMP-b (HA-162), 2c (HA-163, HA-164) and DIAG1-B (HA-165) may land
  first. The status count becomes the combined one, and HA-166's row and section go after the highest
  number present.
- `docs/changelog.md`: newest first, beside T3 Part 2's section.
- `docs/api/*` and `docs/internal/PROJECT-BRIEFING.md`: regenerate after the merge.
- `build-order.md`, HANDOFF, the worksheet: §3.0's edits are carried through T3 Part 2's and 2c-final's
  merges.

### 3.5 HA-166 — the text to add (copy it; the coordinator renumbers only if HA-166 is taken at merge)
Index row:
`| HA-166 | The brain's SD card can only READ a whole file and REPLACE one by truncating it: fopen "wb" empties an existing file at the open; rename(), remove()/unlink(), stat() and mkdir() fail (ENOSYS); a read-write open fails (EINVAL); a crash while one file is written leaves every other file intact; a file flushed, closed and read back whole is on the medium before the next file is opened | **reasoned** (1)-(3), and (5)'s sync call, from the vendored libpros.a/libc.a disassembled 2026-09-26; **invented** (4) and what (5)'s sync commits; none measured | S-session / Station G |`

Entry: **HA-166 — what a program can do to a file on the brain's SD card, and what the safe save
assumes about it.**

*Claim:* five beliefs that the calibration record's safe save is designed around.
1. `fopen(path, "wb")` under `/usd/` on an **existing** file succeeds and empties it **at the open**.
   PROS's `usd_open_r` sends a write-only open without `O_APPEND` to `vexFileOpenCreate`. `O_CREAT` and
   `O_TRUNC` are never consulted, so what happens to an existing file is VEXos's.
2. There is **no rename and no delete.** newlib's `rename()` is `_link_r` followed by `_unlink_r`, and
   `remove()`/`unlink()` are `_unlink_r`. PROS 4.2.2's `file_system_stubs.c` defines `_link`, `_unlink`,
   `_stat`, `mkdir` and `chdir` as `errno = 88 (ENOSYS); return -1`. A program also cannot make a
   directory, and cannot ask `stat()` whether a file exists; an `"rb"` open can answer that.
3. A read-write open (`"r+b"`, `"w+b"`, `"a+b"`) fails with `EINVAL` (22). `usd_open_r` accepts only
   read-only or write-only, so a file cannot be updated in place.
4. A power cut, a brownout or a card pulled while ONE file is being written leaves every OTHER file
   readable with its previous bytes. FatFS writes the FAT and a directory sector for the file being
   written, two files' directory entries can share one 512-byte sector, and whether the card tears a
   sector write is the card's.
5. A file whose bytes went through `fflush`, whose writer closed it, and whose read-back matched is **on
   the medium** (its data, its FAT chain and its directory entry) before the program's next open of
   another file. A read-back reads what the medium holds.
   - PROS's `usd_write_r` passes each write to `vexFileWrite` and then calls `vexFileSync`, ignoring
     what the sync returns.
   - `usd_close_r` calls `vexFileClose` and returns 0 whatever it answers.

   So a sync is CALLED after every write the driver passes on. What `vexFileSync` commits (FatFS's
   `f_sync`, or less), and whether the card has a write cache of its own, is VEXos's and the card's.

*Source:*
- `firmware/libpros.a`:
  - `file_system_stubs.c.o`;
  - `usd_driver.c.o`: `usd_open_r`, `usd_write_r`, `usd_close_r`.
- `firmware/libc.a`: `libc_a-renamer.o`, `libc_a-remove.o`, `libc_a-unlinkr.o`.
- Both archives were read with `arm-none-eabi-objdump -dr` (2026-09-26).
- The save and the load in `calibration/record_store.hpp`.
- The store seam `hal/file_store.hpp`, which has, on purpose, no rename, no delete and no read-write
  open.

*Confidence:*
- (1)–(3), and the sync call in (5), are **reasoned** from the vendored binaries. **The host test cannot
  see them:** on Linux `std::rename` and `std::remove` work, so a rename-based save would pass every host
  test and fail on the brain. This entry exists so nobody simplifies the save back to a rename.
- (4), and what the sync in (5) commits, are **invented**.

*Settle:*
- (a) Station G, after a SAVED run: on a laptop, `shulib_drivetrain.bin` and `shulib_drivetrain.bak` both
  exist, 180 bytes each, byte-identical (`cmp`). That settles (1)'s success half for these names and the
  `.bak` long file name.
- (b) Settle (2) and (3) by extending Bench Tests' existing SD probe (`src/bench_r3a.cpp` `probeSdCard`,
  button 5). The extension calls `rename`, `remove`, `mkdir` and an `"r+b"` open on its own scratch file,
  and prints each return value and `errno`. Unowned. The design holds either way.
- (c) (4) and (5) need a deliberate power cut during a save and right after one, then both files read on
  a laptop (unowned).

*Blast radius if wrong:*
- (1) the other way, where an existing file refuses `"wb"`: every re-save says NOT SAVED and the record
  file is untouched. The failure is loud.
- (2)/(3): nothing breaks; a simpler design was available.
- (4) or (5): a power cut in the save's few milliseconds could damage both files. The boot then REFUSES
  by name (CRC or length). The robot will not drive; it never drives wrong.

**Contained:** two files, one program's save, one boot read.

---

## 4. OUT
- **Any new record layout, version bump or golden change** (D1 (c)).
- **Calibrate's screen** (D5).
- **Any rename, delete, directory or read-write open, anywhere.**
- The blackbox's `SdSink` file. It truncates once per boot by design (`block_sink.hpp` :64-67); it is a
  log, not a record.
- L7's gains file. D6 is a brief amendment only.
- An SD probe for HA-166's settle (the Bench Tests extension is unowned; `bench_r3a.cpp` is in 2c-final's
  diff).
- The Teleop panel beyond its source words and the LCD BACKUP line; the ipr and track numbers stay
  serial-only (2b's A3).
- Moving `sourceRows()` out of `teleop_program.cpp` (COMP-b's B11 decides where a shared panel lives).
- R3d's close.
- Any hardware claim.

**In flight — do NOT touch.** Confirmed by reading each worktree's diff at the re-check, 2026-09-26
evening. Both worktrees are based on `cc513a7`.
- **T3 Part 2** (`.claude/worktrees/t3-part2`, uncommitted):
  - `src/teleop_program.{cpp,hpp}`, `src/boot_source.{cpp,hpp}`, `src/mechanism_table.hpp`, and the new
    `src/teleop_claws.hpp`;
  - `include/shulib/teleop/{mechanism_binding,teleop_loop}.hpp`,
    `include/shulib/calibration/boot_check.hpp`;
  - `test/boot_check_test.cpp`, `test/boot_source_test.cpp`, `test/mechanism_binding_test.cpp`,
    `test/mechanism_table_test.cpp`, `test/toggle_binding_test.cpp`, and the new
    `test/teleop_claws_test.cpp`;
  - `README.md`, `docs/changelog.md`, `docs/roadmap.md`, `docs/cookbook/06`, guide ch. 07, 10, 11 and 14,
    `docs/api/{README,all-entities,boot_check,mechanism_binding,teleop_loop}.md`;
  - **`docs/hardware-assumptions.md`**: the status paragraph (its `@@ -68`), HA-147/153-159's entries
    (~:1620-1862) and ~:2940;
  - `docs/internal/`: `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, **`build-order.md`**,
    `chunks/DEFECTS2-COMPLETED.md`, `chunks/T3-PROGRESS.md`;
  - the worksheet's Stations F (~:563-584), **G (:832)** and J (~:1223-1440).
- **G1 Part 2c-final** (`.claude/worktrees/g1-2c-final`: WIP `80e2615` plus uncommitted changes):
  - `src/main.cpp`, the new `src/build_selection.hpp`, `src/chassis_table.hpp`,
    `src/mechanism_table.hpp`, `src/fault_latch_hook.hpp`;
  - **`src/calibrate_program.cpp` (it adds `calibrateInitialize` ~:1037) and `src/calibrate_program.hpp`**;
  - `src/bench_r3a.{cpp,hpp}`, `src/drive_program.{cpp,hpp}`, the new `src/xdrive_program.{cpp,hpp}`,
    `src/xdrive_config.hpp` and `src/state_screens.{cpp,hpp}`;
  - `include/shulib/{robot/robot.hpp, config/robot_config.hpp, diag/session_info.hpp,
    teleop/stick_mapping.hpp, hal/pros/device_provider.hpp}`;
  - `test/boot_source_test.cpp`, `test/chassis_table_test.cpp`, `test/mechanism_table_test.cpp`,
    `test/f6_signature_pin_test.cpp`, `test/guide_examples_test.cpp`, `test/robot_builder_test.cpp`,
    `test/routine_signature_pin_test.cpp`, and the new `test/build_selection_test.cpp` and
    `test/xdrive_config_test.cpp`;
  - `tools/src_build_gate.py`, `Makefile`, `.github/workflows/ci.yml`;
  - `README.md`, guide ch. 07, 08 and 13, `docs/roadmap.md`, `docs/vexbuilder-integration.md`,
    `docs/api/{pros-device_provider,robot,robot_config,session_info,stick_mapping}.md`;
  - **`docs/hardware-assumptions.md`**: HA-111's row (:210) and entries (~:745-790, ~:1059), and HA-128;
  - `docs/internal/`: `HANDOFF-remaining-work.md`, `RESUMING.md`, `PROJECT-BRIEFING.md`,
    **`build-order.md`**, `chunks/G1-PROGRESS.md`;
  - the worksheet (:1-8, ~:1490).
- **COMP-b** (next): a competition program in `src/`, `main.cpp`'s chain, Station K. Per its brief's B11
  (`COMP-competition-program.md` :90-95) it may also edit `src/boot_source.*` (the claw-port refusal)
  and move a shared panel out of `src/teleop_program.cpp`.

**Part A touches none of the CODE files above.** It shares four documents, which the coordinator
reconciles at merge (§3.4):
- `docs/hardware-assumptions.md`: the status paragraph, the index and a new section. HA-122's entry and
  HA-139's entry are touched by neither worktree.
- `build-order.md`: the status of this chunk's entry.
- `docs/changelog.md`: a new subsection.
- `docs/api/`: regenerated after merge.

Part A adds files under `include/shulib/hal/pros/` beside 2c-final's `device_provider.hpp`. It edits only
`block_sink.hpp` there (one comment sentence), which neither worktree touches.

**Found while reading, not this chunk's:**
- HA-139 has hardware evidence it has not recorded: a Calibrate-written record decoded, CRC and all, at
  eight Teleop boots on 2026-09-25. Its settle line is owed at P2 or R3d's close. (B1 records it with
  its two-file rewrite; §3.2.)
- The status board still lists item (2) as open (§3.0).
- The HANDOFF has no entry for either defect (§3.0).
- COMP's brief B11 may need a re-check against B2 if COMP-b is re-briefed.

---

## 5. Checks

**Every part:**
```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check
python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py
python3 tools/briefing_status.py generate   # LAST
python3 tools/briefing_status.py check
```
- Run the suite and report the counts.
- The `src` build gate must pass with zero disallowed warnings.
- Run the ARM header gate over every header (RESUMING §3). The new headers must compile under
  `-Wconversion -Wsign-conversion -Wshadow -Werror`.
- Run both CI greps: `pros/` is included only under `hal/pros/`, and no `sim/` include appears outside
  `sim/`.
- Run `api_doc_tool` self-test, coverage and examples as well as `check-fresh` and `check-removability`.

**Part A's DoD:**
- [ ] every §2 Part A test green, with the enumeration counts logged (starting cards × indices ×
      states, and the pairs);
- [ ] all 21 mutations (22 runs: 12a and 12b) observed RED on fresh binaries and logged;
- [ ] no `src/` file changed;
- [ ] HA-166 in, with all five beliefs;
- [ ] the golden unchanged;
- [ ] docs per §3.1.

**Part B1's DoD:**
- [ ] the source-text pin green;
- [ ] all 3 mutations RED;
- [ ] the builds link and each build's beacon is unchanged;
- [ ] docs per §3.2;
- [ ] the status pointers are true after the commit.

**Part B2's DoD:**
- [ ] the extended test 6 green;
- [ ] the source-words pin green;
- [ ] all 8 mutations RED;
- [ ] the builds link and each build's beacon is unchanged;
- [ ] docs per §3.3;
- [ ] the status pointers are true after the commit.

---

## 6. Landmines
1. **The host has rename; the brain does not** (HA-166). A test that passes with `std::rename` proves
   nothing about the robot, so the scan test is not optional.
2. **The destructive moment is the `"wb"` OPEN, not the write.** Never open the record for writing before
   the backup verifiably holds its decodable bytes. Never write the backup from a record that does not
   decode.
3. **Nothing reports the close.** `ProsBlockSink`'s destructor cannot (`block_sink.hpp` :95-111), and
   PROS's `usd_close_r` returns 0 regardless. The read-back after its scope is the only proof: never skip
   it on any `put`, and never decide from what `replace` reported.
4. **The report MEASURES the card, both files, at the end.** A sentence chosen from what the code meant to
   do repeats the exact defect this chunk fixes.
   - **A refused read is not a measurement of absence** once the save has written that file. There is
     no delete, so the file is UNKNOWN, not gone.
   - If the card is gone, refused every open, or does not read back as the save last saw it, say
     UNCONFIRMED (u1–u3) and claim nothing more.
5. **Decoding means all five checks on exactly 180 bytes.** Read with a larger buffer. The CRC is what
   turns a TORN file into a refusal instead of a misread.
6. **`calibration_record.hpp`: add the constant (and its `static_assert`) only, plus the :52-54 prose.**
   The layout, the version and the golden stay. L7 binds to the record's CRC.
7. **The boot's existing sentences are pinned** (6k/6p/6q). The backup adds lines and edits none. An
   absent backup adds nothing.
8. **After 2c-final, `-Wunused-function` FAILS the `src` gate** (the team lead's ruling):
   - no file-local function that a build does not call;
   - one `#if` chain (`src/build_selection.hpp`);
   - no `SHULIB_RUNS_*` names in new comments;
   - every `src/*.cpp` compiles in every build.
9. **`test/mechanism_table_test.cpp` pins `calibrate_program.cpp`'s census and filter as text**
   (~:955-1000). Do not reformat those lines.
10. **`src/` rules.** Only literal formats (`-Wformat-nonliteral`, `-Wformat-truncation` are errors). No
    `%` and no non-ASCII character may reach a PROS screen, and `printText` is the only path. No public
    doc may name a `docs/internal` path (C7). Cite evidence by date.
11. **Allocation and exceptions.**
    - `hal/file_store.hpp` and `calibration/record_store.hpp` allocate nothing.
    - `ProsFileStore` allocates only inside the existing adapters (each one's `std::string` path, as
      today's save already does).
    - `FakeFileStore` (host only) may allocate.
    - Nothing in the store or the save is `noexcept`. A `noexcept` wrapper would turn an adapter's
      precondition into `std::terminate` with no message, and would make the bare-name test impossible.
12. **Copy, don't move, while two chunks are in flight.** Robot two's fixture (`boot_source_test.cpp`)
    and `codeOnly` (`mechanism_table_test.cpp`) live in files that BOTH T3 Part 2 and 2c-final edit.
    Part A copies each, with a comment citing the source file and line. Consolidating them into a shared
    test header happens only after both have landed, in B1 or B2, and is logged.
13. **The two-fault limit is real; do not "fix" it with a test that cannot fail.** A misread of a good
    record file at step 1, followed by a failure in the same save, can lose it (D1). The pairs test
    covers silent WRITES followed by a crash, and excludes read faults followed by a crash on purpose.
    Log the exclusion and its reason (HA-139).
14. **HA-166 only.** One entry with five beliefs. Amend HA-122 (two sentences, Part A) and HA-139 (B1)
    without changing their statuses. Take no other number.
15. **In-flight files** (§4). B1 and B2 start by re-reading `calibrate_program.cpp`, `boot_source.hpp`
    and `teleop_program.cpp` **as landed**. T3 Part 2 rewrote about 234 lines of `boot_source.hpp` and
    about 134 lines of `teleop_program.cpp`, and 2c-final added ~23 lines to `calibrate_program.cpp`, so
    every line number above will have moved.
