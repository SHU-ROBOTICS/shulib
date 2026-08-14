# Chunk DOCS1 — the full documentation pass, then the release to `main`

> **A whole chunk that ships no code.** Called at the team lead's direction on 2026-08-14:
> *"nothing but update ALL the documentation since we've done a lot, then push and merge everything
> new to main. Taking that time to slow down will save us in the long run."*
>
> **They are right, and here is the mechanical reason:** merging to `main` is what **publishes to
> docs.shurobotics.com**. Anything wrong in the documentation becomes *publicly* wrong at that
> moment, under the project's own name, in front of judges and other teams. The documentation pass
> is not tidying before a release — it **is** the release gate.
>
> **Status:** brief. Not started. **Live log:** `DOCS1-PROGRESS.md`, created first.

---

## 1. Why now

Twenty-three chunks have landed. **35 commits are unpushed.**

**Do NOT measure `main` by commit distance.** `main` is a *squash* of `release/v2` with
`docs/internal/` dropped, so its history is deliberately disjoint and `git rev-list --count main..HEAD`
returns a meaningless 195. Ask what `main` **contains** instead (`git cat-file -e main:<path>`).

Measured that way: **`main` is current through Phase D** — it was updated 2026-08-12 and has the
recipe API, the frozen contracts and the docs site. **Eight chunks are missing from it:** E1
(blackbox), E2 (GPS correction), E3 (AprilTag heading), E4 (the EKF), F1 (mechanism seam), F2 (the
guaranteed safe stop), R1a and R1b (the entire hardware layer). That is the whole drift-correction
and hardware story — roughly three weeks of the highest-value work in the project.

The last four chunks changed what is *true* about this library more than any stretch since Phase C:

- The library **has hardware adapters** now — fourteen of them. Many chapters were written when it
  had none and say so.
- It **has touched a real robot** and commanded a motor. Several documents still say it never has.
- Seven hardware assumptions **became measurements**. Documents describing them as guesses are stale.
- The available robot is **not what any planning document described** until yesterday.

**This has already bitten twice in two days**, both found by *reading* and neither by any gate:
guide ch. 14 claimed "no adapter has ever touched a physical device" the day after one did, and the
README carried a test count that went stale twice in a single day.

## 2. Scope

**The surface: ~103,000 public words.** 16 guide chapters · 6 cookbook pages · 3 generated API pages ·
8 top-level documents · the README.

**In scope — every public document, read end to end.** Not skimmed, not grepped. The four build-time
doc gates plus the staleness audit already catch what a machine can catch; **what remains is
exactly the class no tool can see**, so the deliverable here is *reading time*.

**Also in scope:**
- `docs/internal/` — the briefing, build-order, RESUMING, and the handoff files
- The `mkdocs.yml` nav (two new pages landed: `changelog.md`, `faq.md`)
- **The release itself** — §6

**Out of scope:** any library code. If a documentation error turns out to be a *code* error, stop and
report it; do not fix code in a documentation chunk.

## 3. Known-stale, found already — start here, do not stop here

| Where | What is wrong |
|---|---|
| `PROJECT-BRIEFING.md` §2 | Points at `HANDOFF-2026-08-13-F2.md` as "the most recent chunk handoff". Three chunks have landed since. It misdirects the very next session — **fix first** |
| Guide ch. 01–06, 08–12, 15 | **Never reviewed against R1a/R1b at all.** Written when the library had no hardware layer |
| Cookbook (all 6) | Same — never reviewed since Phase D |
| `shulib-v2-master-plan.md` §3 / §13 | Locked and open decisions. R1a and R1b ruled ~15 things between them; none are reflected |
| `roadmap.md` | "You are here" and the milestone badges |
| `docs/changelog.md` | New at R1a. Verify it actually covers every chunk that changed the API, not just the last two |
| `docs/faq.md` | New. Three entries so far; several traps found this week deserve entries |
| `ORIENTATION.md` | Written 2026-08-14. **Decide whether a public version belongs** — an outside team evaluating shulib wants exactly that page, and no public document currently answers it |
| Old `HANDOFF-*.md` files | Historical records. Decide: keep as history, or supersede. **Do not silently edit history** |

## 4. The standard for this pass

**Read for truth, not for typos.** The question on every page is *"is this still true?"* — not
*"is this well written?"*

The four failure classes this project has actually suffered, all found by reading:

1. **Prose that is stale but still parses** — a chapter claiming 57 assumptions when there were 67
2. **A feature that needs a whole new chapter** and nothing demands one exist
3. **Reading-order breakage** — chapter N assuming something chapter N−1 no longer says
4. **Conceptual drift** — the guide teaching a mental model the code stopped following

**And the one that matters most here: do not let the hardware work inflate any claim.** The library
has still never driven a robot. That sentence is defended in ~6 places deliberately. R1a/R1b make it
*tempting* to soften — ch. 14 was already edited once this week to make it **sharper** rather than
weaker, and that is the standard.

**Numbers:** prefer a command over a figure. The README's test count went stale twice in one day; it
now states the shape and points at `./build/test/shulib_tests`. Apply that everywhere a figure would
have to be maintained by hand.

## 5. Definition of Done — the documentation half

- [ ] Every public document read end to end, with a per-document verdict recorded (current / fixed /
      needs-new-content)
- [ ] Briefing §2's stale handoff pointer fixed
- [ ] Master plan §3 and §13 carry R1a's and R1b's rulings
- [ ] Roadmap "you are here" current; every checkbox has cited evidence or an honest `[~]`
- [ ] Changelog covers every API-affecting chunk, not just recent ones
- [ ] FAQ carries this week's traps (the 9999 magic number, the two port conventions, the `/usd/`
      prefix quirk, what a dropped USB looks like)
- [ ] A ruling on whether `ORIENTATION.md` gets a public sibling
- [ ] All gates green: coverage, freshness, examples, removability, briefing, staleness self-test +
      audit
- [ ] **The governing constraint is intact in every place it appears** — grep for it and read each hit

## 6. Then the release — and its own gates

The path is `shulib-v2` → merge into `release/v2` → drop `docs/internal/` → **squash** onto `main`.
`main` publishes to docs.shurobotics.com via GitHub Pages.

**Before merging, all of these must pass:**

1. `python3 tools/api_doc_tool.py check-fresh` — **publishing a stale reference is worse than
   publishing none**, because a published document looks authoritative
2. `python3 tools/prepare_site.py` — refuses to build if any internal document would be published
3. `check-removability` — no public document may reference `internal/`, `chunks/`, `RESUMING` or
   `build-order`
4. The full suite, both CI guards, and the ARM gate

**Two things to confirm with the team lead before pushing**, both open decisions:

- **Pushing at all.** `main` is current only through Phase D; the eight chunks listed in §1 are
  absent from it, so none of the drift-correction or hardware work is public. This publishes all of
  it at once.
- **HTTPS.** GitHub had not issued the TLS certificate; the site serves over HTTP. Tick "Enforce
  HTTPS" once the cert appears. If Cloudflare's proxy is re-enabled, SSL mode must be **Full
  (strict)**, never Flexible.

## 7. Landmines

**L1 — Do not soften the governing constraint.** Hardware progress is exactly when it erodes.

**L2 — Do not edit history.** The `-COMPLETED.md` records and `HANDOFF-*.md` files are records of what
was believed *then*. Correct them by superseding, never by rewriting.

**L3 — Do not fix code.** If a document is wrong because the *code* is wrong, stop and report it.

**L4 — The removability rule is load-bearing at merge.** A public document containing the string
`chunks/` fails the build, and `docs/internal/` is dropped from the published tree.

**L5 — Read, don't grep.** Every staleness incident in this project was found by a person reading a
sentence. Budget the time.

**L6 — Under-claim.** A pass that reports only "all current" has not read carefully enough.
