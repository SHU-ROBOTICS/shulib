# Keeping the guide true — a maintenance procedure

> **Read this before changing anything in `docs/guide/`, and before landing a code change that the
> guide describes.**
>
> `docs/guide/README.md` tells a *reader* how the guide is organised. **This file tells a
> *maintainer* what to do when the library changes.** It is internal; it never ships to `main`.

---

## Why this exists

A manual that drifts out of date is worse than no manual, because people trust it. The guide is
written for someone who does not yet know enough to notice when it is lying to them.

Everything below exists to make that failure mode structurally unlikely instead of relying on someone
remembering.

---

## The three rules that do not bend

**1. Every code example compiles.**
Examples live in `test/*example*_test.cpp` — `guide_examples_test.cpp`,
`cookbook_examples_test.cpp`, `readme_example_test.cpp` — and the documents quote them
**verbatim**. CI builds them. If you change an example, change it *there* and re-quote — never
edit a code block in a markdown file alone. **An uncompiled example is already wrong; it just
doesn't know it yet.**

Since D3 (2026-08-12) the verbatim half is **machine-checked**, in the build and in CI:
`python3 tools/api_doc_tool.py check-examples` requires every non-blank line inside a ```` ```cpp ````
block in any public document to appear verbatim in one of those test files. Before D3 the rule
was real but the enforcement was a person running an internal script, and a mutation proved the
consequence: a chapter's `.hold(300_ms)` retyped to `.hold(300_s)` built clean and passed the
entire suite. The source set is a *glob*, so a new examples file is covered the moment it exists.

`docs/api/` is deliberately outside this scan — it is generated in full, and `check-fresh`
compares the whole directory byte-for-byte against a fresh run, which also catches a hand-written
file dropped there. Every public document is covered by exactly one of the two mechanisms; do not
add a third.

**2. Link, don't restate.**
Test counts, register entries, roadmap status, milestone progress — link to the source. Any fact
copied into the guide is a fact that will silently go stale. If a number genuinely helps an
explanation, write it *as of* a date so a reader can tell it might have moved.

**Since D3 this rule has teeth for signatures specifically.** `docs/api/` is generated from the
headers, so it is the single source of truth for what exists and how it is spelled. Chapter 10
and the cookbook name verbs and arguments conversationally and **never restate a signature**;
when a reader needs an exact type, they follow a link. If the reference and a chapter ever
disagree, the reference is right — nobody typed it.

**Useful check:** if a single library change forces edits in more than two or three chapters, the
guide has duplicated something it should have linked. Fix the duplication, not just the chapters.

**3. Write for someone who doesn't know yet.**
Plain sentences. Every term defined at first use. No slogans — if a line sounds like a tagline, cut
it. Explain *why*, not just *what*: a reader who understands why field-relative and robot-relative
differ will never mix them up; one who memorised a rule will. And never oversell — where the library
hasn't been proven, say so plainly.

---

## What to update when you change something

Find your change on the left; update everything on the right.

| You changed… | Update |
|---|---|
| A `Chassis` verb — name, arguments, behaviour | The API chapter · `test/guide_examples_test.cpp` · the tutorial if it uses that verb · **regenerate `docs/api/`** (the build fails until you do) |
| A `///` comment in `chassis.hpp` / `routine.hpp` | **Nothing by hand** — run `python3 tools/api_doc_tool.py generate` and commit what it writes |
| Added a public member to a documented header | **Write its `///` comment.** The build fails naming the member otherwise, and an empty `///` does not count |
| A `Routine` step — name, arguments, behaviour | The recipe chapter · the cookbook recipes that use it · `test/cookbook_examples_test.cpp` · **and read the freeze procedure: `Routine` is locked (F10)** |
| Added or changed a motion primitive | The API chapter · the "getting there" concept chapter if the *idea* changed |
| Added a `FaultCode` | The diagnostics chapter (what it means, what to do) · the troubleshooting chapter (the symptom it produces) |
| Changed the terminal output format | The diagnostics chapter — **re-capture a real transcript, don't hand-edit the old one** |
| Added a drivetrain | The drivetrains chapter · the extending chapter |
| Changed units, types, or a convention | The coordinates chapter · the API chapter · the examples |
| Changed the build, layout, or commands | The setup chapter · `README.md` — **and run the commands as written to confirm** |
| Anything that shifts what's possible | The "what it can't do yet" chapter · the glossary if a new term appeared |

### Milestone events that need a deliberate sweep

Some changes touch the guide's *framing*, not one chapter:

| Event | Sweep |
|---|---|
| **D1 — recipe API lands** | **Add a chapter; don't rewrite the API chapter.** The full API stays valid — the recipe layer sits on top of it. Point new readers at the easier tier first. |
| **D3 — the cookbook + generated reference land; `Routine` freezes** | ✅ Executed 2026-08-12. Three documents now, with one job each: the guide teaches (in order), the [cookbook](../cookbook/README.md) answers a task (out of order), the [reference](../api/README.md) states what exists (generated). **Do not let two of them own the same fact.** `Routine`'s spellings froze as register row F10, so ch. 9's stability notice and ch. 14's "recipe spellings" limitation were rewritten; `then()` is the one deliberate exception and both places say so. |
| **D2 — F6 freezes** | ✅ Executed 2026-08-12: stability notices removed/rewritten in ch. 09, 10, 14, `chassis.hpp`'s banner, and the guide README's check-order list — atomically. Ch. 9 keeps ONE deliberate notice: `Routine`'s spellings stay unfrozen until D3 (they freeze with the cookbook). |
| **R3 — first real robot run** | **The big one.** "Never run on a robot" appears in the README, the orientation chapter, the setup chapter, and the can't-do-yet chapter. Grep for it and update every instance in the same commit — a half-updated claim is worse than a stale one. |
| **Phase E — correctors land** | The odometry/drift explanation changes meaningfully: drift becomes bounded rather than unbounded. Concept chapter and can't-do-yet chapter. |
| **Phase F — mechanisms land** | New concepts, new API surface, new fault codes. Likely a new chapter. |
| **R4/R5 — assumptions measured** | Anywhere the guide says a number is a guess. Prefer linking `hardware-assumptions.md` so this is a no-op. |

---

## The procedure

1. **Make the library change**, with its tests, as normal.
2. **Check the table above.** If nothing matches, confirm that's really true — "the docs don't mention
   this" is often wrong.
3. **Update the example test first**, if an example is affected. Get it compiling and passing.
4. **Update the chapters**, quoting the example verbatim.
5. **Re-capture transcripts** if output changed. Run it; paste what it actually printed.
6. **Run the verification checklist below.**
7. **Commit the code and doc changes together.** A doc update in a later commit is a doc update that
   might not happen.

---

## Verification checklist

Before calling a guide change done:

- [ ] `cmake --build build/test && ./build/test/shulib_tests` — green, including the example tests
      **and the four documentation gates the build runs first** (coverage, freshness, verbatim
      examples, removability)
- [ ] Every link resolves (check mechanically, not by eye)
- [ ] **No public document links into `docs/internal/`** — public docs must stay cleanly
      removable, or the squash-merge to `main` breaks links (C7 established this property; D3
      mechanized it as `check-removability`, after a mutation showed a planted link passed both
      the build and CI). The build now answers this for you.
- [ ] Any new term appears in the glossary
- [ ] No jargon introduced without a definition at first use
- [ ] Every cross-reference matches the actual filenames. **Numbering need not be contiguous** —
      gaps are deliberately reserved slots (chapter 09 was held for D1's recipe API and filled at D1
      exactly this way: an *added file*, nothing renumbered). Leave reserved gaps alone; don't "fix"
      them.
- [ ] Nothing was restated that could have been linked
- [ ] No sentence sounds like advertising

---

## Renumbering chapters

Numbers exist to give a reading order, not an identity. If you must renumber:

- Renumber with `git mv` so history follows the file
- Grep for the old numbers everywhere — chapters cross-reference each other, and so do
  `docs/guide/README.md`, the root `README.md`, and code comments
- Prefer **inserting** at a decimal-free gap or appending, over renumbering a whole run — every
  renumber risks a dangling reference
- Re-run the link check afterwards. Always.

---

## Common mistakes

- **Editing a code block in markdown instead of the test file.** Now the guide and the compiled
  example disagree, and only one of them is checked.
- **Hand-editing a transcript** to match new output. Re-run and paste the real thing; invented output
  is how a manual starts lying.
- **Copying a number "just this once."** That's how staleness starts.
- **Adding a term without adding it to the glossary.**
- **Documenting something as working before it's verified.** The whole repo runs on a claim meaning
  something. The guide is not exempt.
- **Updating one instance of "never run on a robot."** Sweep all of them together.

---

---

## The three documents, and which one owns what

Added at D3, because the failure mode of three documents is that two of them own the same fact.

| Document | Owns | Must never |
|---|---|---|
| `docs/guide/` | Concepts, order, "how to think about it", gotchas, worked idioms | Restate a signature |
| `docs/cookbook/` | Task-shaped recipes, compiled and quoted verbatim | Teach a concept from scratch, or restate a signature |
| `docs/api/` | Every public member and its exact spelling | Contain a single hand-written character |

When something belongs in two of them, it belongs in the one further down this table, with a link
from the one further up.

---

*Companion to `docs/guide/README.md` (reader-facing), `docs/internal/docs-publishing.md` (where
the generated reference goes), and `docs/internal/RESUMING.md` (the working protocol). Created
2026-08-11 at chunk C8; extended at D3.*
