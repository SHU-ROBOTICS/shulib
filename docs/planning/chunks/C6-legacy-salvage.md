# Chunk C6 — legacy salvage (the last look before deletion)

> **Phase C, chunk 6 of 7.** Predecessors: C1–C5 ✅.
> **C7 deletes `legacy/`. C6 is the last chance to look at it.**

**Workstream:** WS11 (Tooling/cutover) · **Milestone:** M2

---

## Why this chunk exists, and why it is separate from C7

C7 is the only chunk in the project that **deletes** things. Everything else has been additive. The
unique failure mode is: salvage something incompletely, delete the original, and it is gone from the
working tree forever — recoverable from git, but nobody will ever know to go looking.

So the sequencing is strict and non-negotiable: **C6 salvages and proves the salvage. C7 deletes.**
Never the same chunk. C7 does not start until C6's output is verified **independently of `legacy/`
existing at all.**

---

## What the roadmap said, and what is actually left

The roadmap's salvage list is **mostly already done** — verify this rather than assume it:

| Roadmap item | Actual status |
|---|---|
| Re-derive the Pilons arc math | ✅ **done at M2** — `arcStep` re-derived clean, and the rewrite *caught a real legacy bug* (the old odom rotated each tick's chord by the **new** heading instead of the **average**) |
| `logger.hpp` → `io/Telemetry` | ✅ **superseded** — A1 explicitly rejected porting it (clean-room), re-derived the diagnostics fresh, and C5 completed them. The three legacy defects became *design constraints*, not code |
| `RobotCommands` → `sequence/` seed | ⬜ **the real remaining work** — see below |

**Confirm each of these claims before relying on it.** If something was assumed done and isn't, this
is the last chunk where finding out is cheap.

---

## The real work: salvage **knowledge**, not code

The clean-room principle stands — *re-derive, don't port.* It has already paid twice (the `arcStep`
bug; the logger's three defects designed out rather than inherited). Nothing in `legacy/` should be
copied into the new tree.

But `legacy/` contains something that **cannot be re-derived**: what this team actually needed a robot
to do. That is domain knowledge, and deleting it unexamined loses it.

### 1. Mine the command vocabulary
`include/legacy/shulib/RobotCommands/` and **`src/legacy/autonomous_commands.csv`** (845 rows,
`command,x,y,heading,speed`, with `MOVE_WITH_HEADING` and friends) are a record of the commands the
team actually authored routines with.

Produce **`docs/planning/legacy-command-vocabulary.md`**: every distinct command id found, what it
did, and how it maps to shulib v2 today — a C1/C2 motion, a Phase F mechanism primitive, a G2
command-id, or *nothing yet* (which is a gap worth knowing about).

This becomes a **requirements input for F2 and G2** — G2 owns the canonical command-id vocabulary, and
the honest way to design it is knowing what the team actually used. Also note the `.shupaths`-style
CSV shape: G4 ships a legacy importer, and this file is a real specimen of what it must read.

### 2. Audit every remaining file
Walk **all 34 files** in `include/legacy/` and `src/legacy/`. For each, one line: **superseded by X** ·
**salvage (and where it went)** · **discard, nothing of value** — each with a reason. No file may be
unclassified when C6 closes.

Watch for: hard-won numbers (measured geometry, tuned gains, port maps) — those become
`hardware-assumptions.md` entries or R-phase inputs; the `.ignore`/`.txt` GUI files (probably
discard, but say so); and anything encoding a *field or game* fact rather than code.

### 3. Port only what genuinely earns it
Expect this to be small, possibly empty. **An empty port list is a fine outcome** if the audit
justifies it — the value of this chunk is the audit and the vocabulary, not a line count.

---

## Design constraints

1. **Re-derive, never copy.** If something is worth having, write it fresh with tests. Clean-room has
   caught two real bugs already.
2. **Prove salvage independent of `legacy/`.** Anything ported must build and pass with `legacy/`
   conceptually absent — C7 must be able to `rm -rf` without breaking anything.
3. **Classify every file.** An unclassified file is an unmade decision, and C7 makes it irreversible.
4. **Don't build F2 or G2.** Capture requirements; the sequence engine and command registry are theirs.
5. **Don't touch the working tree's build.** C6 is additive: docs plus (maybe) small tested ports.
   `make` is still expected to fail until C7.

---

## Test requirements

Light — this is mostly an audit — but real where it matters:

- Anything ported ships with adversarial tests at the project's standard, and a **proven-red mutation**
  if it carries logic
- Anything ported is demonstrably independent of `legacy/`
- The vocabulary document is **complete**: every distinct command in the header set *and* the CSV
  appears, cross-checked mechanically (script it — a hand-read list is a list with holes)

---

## Definition of Done

- [ ] All 34 legacy files classified: superseded / salvaged / discarded, each with a reason
- [ ] The three roadmap salvage claims verified, not assumed
- [ ] `docs/planning/legacy-command-vocabulary.md` complete, mechanically cross-checked, with each
      command mapped to its v2 home or flagged as a gap
- [ ] Any hard-won constant found is captured (register entry or R-phase input)
- [ ] Anything ported has tests and is independent of `legacy/`
- [ ] **A clear, explicit statement that `legacy/` is safe to delete** — that sentence is what C7 acts on
- [ ] Suite green; both guards pass; ARM gate passes

---

## Live progress log — required

`docs/planning/chunks/C6-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`docs/planning/chunks/C6-COMPLETED.md`**. Depth here is measured by the **completeness
of the audit**, not line count — but the file-by-file classification table must be exhaustive, and the
"safe to delete" statement must be unambiguous and evidence-backed.

Flag anything discovered that changes a later chunk's assumptions — especially F2, G2, or G4 (the
`.shupaths` importer, for which the CSV is a specimen).

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **Deletion is one-way in practice.** Git keeps it, but nobody will look. Classify everything.
- **Don't port bad code to feel thorough.** Clean-room means the audit *is* the deliverable.
- **Don't lose the domain knowledge.** Code is replaceable; "what the team actually needed" is not.
- **Don't declare "safe to delete" loosely.** C7 acts on that sentence.
