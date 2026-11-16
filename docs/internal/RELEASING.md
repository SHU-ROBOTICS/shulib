# Releasing shulib — the protocol

> **Read this before any release.** It replaces a set of instructions that existed only as prose
> in `DOCS1-COMPLETED.md`'s handoff section, which is where the release mechanic lived until
> DEFECTS1 (2026-08-15) and is a bad place for it: a handoff is written once, read once, and
> then quoted from memory. `RESUMING.md` owns the chunk loop; this owns the release.

## The whole thing, if everything is normal

```sh
python3 tools/release.py check                     # changes nothing; run it any time
python3 tools/release.py stage "what this means"   # builds the commits LOCALLY, verifies, stops
git log --oneline -3 release/v2                    # look at what it built
python3 tools/release.py push                      # pushes what stage built
```

`stage` never pushes and `push` never builds. That split is the point: the thing that is hard to
reverse is one command, it does nothing but transmit, and you get to look first.

---

## What a release actually is

Three refs, and the relationship between them is not obvious:

| Ref | What it is |
|---|---|
| `shulib-v2` | the development branch. Carries `docs/internal/` — the whole development record |
| `release/v2` | `shulib-v2` **merged in, with `docs/internal/` dropped**. Ordinary merge history |
| `main` | a **single-parent tree snapshot** of `release/v2`. Not a merge of anything. **This is what publishes docs.shurobotics.com** |

**`main` is deliberately disjoint from the development history.** That is why
`git merge --squash` into it produces 49 add/add conflicts and is the wrong tool — DOCS1
measured that in a throwaway clone. The right tool is `git commit-tree`, which makes a commit
from an existing tree with a parent you choose, and gives every release the same single-parent
shape the previous ones have.

**Nothing under `docs/internal/` may ever reach `main`.** C7 made that directory a cleanly
removable unit and the entire no-leak promise rests on it. `prepare_site.py` refuses to build a
site that would publish an internal doc, and `check-removability` fails if a public doc so much
as links into one — but neither of those runs on the *tree*, so the release script checks the
tree directly.

## The four invariants the script enforces

1. **`main`'s tree is byte-identical to `release/v2`'s tree.** If it is not, something was
   committed to one and not the other and the published site no longer matches the release.
2. **The snapshot has exactly one parent.** Two means somebody merged into `main`, which breaks
   the disjoint-history shape every earlier release has.
3. **No `docs/internal/` anywhere in the published tree.**
4. **Every push is a fast-forward.** No release has ever needed a force. One that appears to is
   a signal to stop and find out why, not a flag to add.

---

## The three things that used to go wrong, and what changed

### 1. `git branch -f main` fails, and the failure is a warning worth heeding

`main` is often checked out in a scratch worktree (`git worktree list` will show them). At
DEFECTS1 one held **157 uncommitted files**. Forcing the branch ref while a worktree has it
checked out either refuses outright — which is what happened — or, if you route around it with
`update-ref`, leaves that worktree's index describing a tree it has never seen.

**So the script never touches your local `main` at all.** It builds the snapshot as a detached
commit and pushes it *by sha*:

```sh
git push origin <sha>:refs/heads/main
```

That is not a workaround for a permission problem. It is the correct shape: `main` here is a
publishing target, not a place anyone works, and the only copy that matters is the remote's.
Your local `main` stays wherever it was, and so does every worktree.

### 2. The `docs/internal/` conflicts look alarming and are the mechanism

Merging `shulib-v2` into `release/v2` **always** produces modify/delete conflicts on every
`docs/internal/` file that changed since the last release. That is not a merge going wrong —
`release/v2` exists precisely to be `shulib-v2` without those files, so the conflict *is* how
the drop is expressed. The resolution is always deletion, and the script does it.

**What the script will NOT guess at:** a conflict outside `docs/internal/`. If one appears it
stops and says so, because that means real content diverged and a human has to choose.
`git merge --abort` backs the whole thing out.

### 3. The doc-gate deadlock, which is once *per build*, not once ever

`shulib_tests` depends on `shulib_doc_gates`, and `briefing_status.py check` derives the suite
state by running **the binary that already exists**. So the moment the suite is red — or the
binary is stale — the gate fails, and the failing gate blocks the rebuild that would fix it.

For a red or stale binary the escape is the same every time: `python3 tools/briefing_status.py generate`,
then build (for a BROKEN one, see below). During a
mutation campaign that is once per mutation, which is worth knowing before you start one.

**A binary that is present but BROKEN** (since 2026-10-22). Every tool that runs the suite —
`briefing_status.py`, `doc_staleness_audit.py` (both in this gate) and `release.py` — runs it through
`tools/suite_run.py` (its header has the details): 4 GiB of address space (Linux; macOS does not
enforce RLIMIT_AS, so there the run has the other bounds only), 256 MiB of output (to a file, never
held by the tool), 600 s of CPU, 300 s of wall time; the run stays in the caller's process group, so
an outer `timeout` or Ctrl-C reaches it, and on Linux it dies with the tool that started it (the
timeout kill and that tie reach the suite process itself — it does not fork). The bounds are per
process. (On 2026-10-15 a mutation run's mutant reached 58 GB in the mutation runner;
left linked, it reached 54 GB through `briefing_status.py generate`, unbounded, and took the desktop
down; this gate ran the same unbounded code.) A run is BROKEN when it is killed by a signal or a
bound, does not finish, prints no doctest summary, exits non-zero with a summary that says nothing
failed, uses half or more of its memory bound at its peak (an allocation failure the code under
test caught and swallowed — the library's own `catch (...)` blocks do that by design), or prints
any of doctest's reports of a CAUGHT `std::bad_alloc` — a test case or an assertion
that threw it, or "threw a DIFFERENT exception" that was it: doctest catches an allocation failure and
goes on to a normal summary, so a memory-bound hit would otherwise read as an ordinary red count. A
BROKEN binary is never "not built" and never a count to commit: `check` exempts its suite line and
SAYS so (failing here would block the rebuild that replaces it; the staleness audit then derives no
counts and says why), and `generate` refuses it. The order that works — each step needs the one
before, because the build runs this gate before it relinks:
`rm -f build/test/shulib_tests && python3 tools/briefing_status.py generate && cmake --build build/test && python3 tools/briefing_status.py generate`.
Known limit: a red run that prints more than 256 MiB of failure reports reads BROKEN.

**And a second-order trap that only exists since DOCS2:** `docs/api/` records the declaration
*line number* of every entity, so any edit that shifts lines fails `check-fresh` — and if that
happens mid-mutation, the binary is never relinked and running it reports the **previous**
result. Make mutations line-count-neutral, and if a mutation changes a rendered declaration (a
default member initializer, a signature) regenerate the docs alongside it.

---

## When the push is blocked

`git push` may be refused by the agent permission layer even when the release is correct and
authorized. That is a client-side policy, not a repository state, and the fix is a permission
rule rather than anything in git:

```
Bash(git push:*)              # allow all pushes
Bash(git push origin *:refs/heads/main)   # or narrow it to the release target
```

Until such a rule exists, `release.py stage` still does all the work and prints the exact shas,
so a human can run the three pushes.

## A publish builds the route builder (added 2026-11-14, RMP1-A1)

`pages.yml` builds the route builder from the published commit (`tools/route_maker/build.py`, its robots from
`shulib_route_maker describe` on that commit) into `route-builder/` beside its docs page, and its PG1 step fails the
publish if `route-builder/app.html` is missing or is not the app. After the deploy, **open
docs.shurobotics.com/route-builder/app.html, place a start, add a drive and press Export once** — the robots it
offers are that commit's, and a robot fact typed since reaches the published builder only at the next publish
(the RMP1 brief's D17: a docs release at each route-builder merge before the 21st, on the team lead's OK for the
push).

**OPEN — for the coordinator to decide before RMP1-A1's merge (found by A1's review, 2026-11-15):** D17 publishes at
EACH route-builder merge, but the rule just below ("Chunks between parts") fails the preflight on RMP1's PROGRESS file
while another RMP1 part is building, and A.7 / W4 plans A2e to build from ~Tue 11-17 and A2u from A1's merge — so as
written, A1's own publish is refused if A2e or A2u has started. The two ways out: (a) publish A1 BEFORE A2e / A2u
launch (the between-parts marker in RMP1-PROGRESS.md, with A2e and A2u named as the next parts, until they start);
or (b) a between-parts reason that names A2e and A2u as separate runs in their own worktrees, whose half-built state
is not on the branch being released — a change to the rule below, so it is the coordinator's and the team lead's.
Whichever is chosen is written here, replacing this paragraph.

## Chunks between parts (added 2026-11-13)

The preflight fails on an **interrupted chunk**: a `docs/internal/chunks/<CHUNK>-PROGRESS.md` with no
`<CHUNK>-COMPLETED.md`. Most chunks since September are built in parts, and many have later parts scheduled
deliberately (after the competition, after the panel) — their landed parts are merged and gated, nothing is
half-built, and still they have no COMPLETED file. On 2026-11-13 sixteen of them blocked the first release
since August.

Such a chunk **declares itself between parts** with ONE line in the first 40 lines of its PROGRESS file:

```
<!-- release: between parts — STEP1-A and STEP1-B landed; nothing is building -->
```

The preflight then prints it as a `WARN` with its reason and does not fail. The rule (`between_parts_reason()`
in `tools/release.py`, covered by `python3 tools/release.py self-test`): the marker must be a whole HTML
comment, near the top (a note buried in a log is not a declaration), with a non-empty reason. **The coordinator
writes it at a part's merge, and REMOVES it when another part of that chunk starts building** — so a chunk
that is genuinely mid-build still fails the release. When a chunk's last part lands, its COMPLETED record
replaces the marker.

## What still is not automated, on purpose

- **Deciding to release.** The script checks that a release would be *correct*; it has no view
  on whether it should happen. Merging publishes everything at once, and the master plan's §18
  question — whether to push at all — is a person's.
- **The release message.** `stage` takes it as an argument. A release that cannot be described
  in one line is usually two releases.
- **HTTPS / DNS.** Settled 2026-08-14 and recorded in `PROJECT-BRIEFING.md` §17, including the
  non-obvious cause (GitHub had never *requested* a certificate, because the custom domain was
  set before DNS resolved, and nothing retries). If the site ever serves plain HTTP again, that
  section — not this one — is where the answer is.
