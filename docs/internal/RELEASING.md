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

The escape is always the same: `python3 tools/briefing_status.py generate`, then build. During a
mutation campaign that is once per mutation, which is worth knowing before you start one.

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
