#!/usr/bin/env python3
"""shulib release driver — shulib-v2 → release/v2 → main, with the invariants enforced.

    python3 tools/release.py check     # preflight only, changes nothing
    python3 tools/release.py stage     # preflight + build the release commits LOCALLY
    python3 tools/release.py push      # push what stage built (refuses if not staged)

WHY THIS EXISTS, and it is not tidiness. Before DEFECTS1 the release mechanic lived as prose
in a chunk record's handoff section, and it said what NOT to do more clearly than what to do
("`git merge --squash` is wrong — 49 add/add conflicts, main is deliberately disjoint"). Every
release therefore rediscovered the same four things by hand: that docs/internal conflicts are
resolved by DELETION, that `main` is a single-parent tree snapshot made with commit-tree, that
`git branch -f main` fails when main is checked out in a worktree, and that nothing verifies
the published tree before it is published. Encoding it is cheaper than rediscovering it, and a
release is the one operation in this project that is outward-facing and hard to reverse.

THE THREE INVARIANTS, checked rather than remembered:

  1. main's tree is BYTE-IDENTICAL to release/v2's tree. main is not a merge of anything; it is
     a snapshot, and every prior release has the same single-parent shape.
  2. NOTHING under docs/internal/ is ever published. C7 made that directory a removable unit and
     the whole no-leak promise rests on it.
  3. Every push is a FAST-FORWARD. No release has ever needed a force, and one that appears to
     is a signal to stop, not a flag to add.

WHY IT NEVER TOUCHES YOUR LOCAL `main`. It builds the snapshot with `git commit-tree` and pushes
it BY SHA (`git push origin <sha>:refs/heads/main`). That is not a workaround, it is the safer
shape: `main` is frequently checked out in a scratch worktree, and at DEFECTS1 one held 157
uncommitted files. Moving the branch ref under a worktree leaves its index describing a tree it
has never seen. Pushing by sha updates the thing that actually publishes and leaves every
working copy alone.
"""

import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = "shulib-v2"
RELEASE = "release/v2"
PUBLISH = "main"
STAGED = os.path.join(REPO, ".git", "shulib-release-staged")


def git(*args, check=True, quiet=False):
    r = subprocess.run(["git", "-C", REPO, *args], capture_output=True, text=True)
    if check and r.returncode != 0:
        if not quiet:
            print(f"  git {' '.join(args)}\n{r.stdout}{r.stderr}", file=sys.stderr)
        raise SystemExit(f"FAILED: git {' '.join(args)}")
    return r.stdout.strip()


def run(cmd, label):
    r = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, shell=isinstance(cmd, str))
    ok = r.returncode == 0
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        tail = (r.stdout + r.stderr).strip().splitlines()[-12:]
        for line in tail:
            print(f"        {line}")
    return ok


def head_of(ref):
    return git("rev-parse", ref, check=False, quiet=True)


def preflight():
    """Everything that must be true before a release is even attempted."""
    print("── preflight ─────────────────────────────────────────────────────────")
    ok = True

    if git("status", "--porcelain"):
        print("  FAIL  working tree is dirty — commit or stash first")
        ok = False
    else:
        print("  PASS  working tree clean")

    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    if branch != SOURCE:
        print(f"  FAIL  on '{branch}', expected '{SOURCE}'")
        ok = False
    else:
        print(f"  PASS  on {SOURCE}")

    git("fetch", "origin", "--quiet", check=False)
    if head_of(SOURCE) != head_of(f"origin/{SOURCE}"):
        print(f"  WARN  {SOURCE} differs from origin/{SOURCE} — it will be pushed too")
    else:
        print(f"  PASS  {SOURCE} matches origin")

    # A -PROGRESS.md with no -COMPLETED.md means a chunk was interrupted. Releasing mid-chunk
    # publishes a half-finished state, and the briefing gate already knows how to spot it.
    chunks = os.path.join(REPO, "docs/internal/chunks")
    if os.path.isdir(chunks):
        names = os.listdir(chunks)
        interrupted = sorted(
            n[: -len("-PROGRESS.md")]
            for n in names
            if n.endswith("-PROGRESS.md")
            and f"{n[: -len('-PROGRESS.md')]}-COMPLETED.md" not in names
        )
        if interrupted:
            print(f"  FAIL  interrupted chunk(s): {', '.join(interrupted)} — finish before releasing")
            ok = False
        else:
            print("  PASS  no interrupted chunks")

    return ok


def _gates():
    """The suite and every gate, with the build FIRST — and it must be first for a reason.

    `shulib_tests` DEPENDS on `shulib_doc_gates`, and one of those gates derives the suite state
    by running the binary that already exists. So a stale binary can fail the gate, which then
    blocks the rebuild that would fix it. If this function reports the build failing on
    briefing_status alone, run `python3 tools/briefing_status.py generate` and try again — that
    is the documented deadlock, and it is once per build, not once ever.
    """
    print("── the suite and every gate ──────────────────────────────────────────")
    ok = True
    ok &= run(["cmake", "--build", "build/test", "-j", str(os.cpu_count() or 4)], "build (carries the doc gates)")
    ok &= run("./build/test/shulib_tests > /dev/null", "suite green")
    for c in ("self-test", "check-coverage", "check-fresh", "check-examples", "check-removability"):
        ok &= run(["python3", "tools/api_doc_tool.py", c], f"api_doc_tool {c}")
    ok &= run(["python3", "tools/briefing_status.py", "check"], "briefing status current")
    ok &= run(["python3", "tools/doc_staleness_audit.py"], "doc staleness audit")
    ok &= run(
        "if grep -rnE '#[[:space:]]*include[[:space:]]*[<\"]pros/' include/shulib "
        "| grep -v '^include/shulib/hal/pros/'; then exit 1; fi",
        "GUARD 1 — PROS-free outside hal/pros/",
    )
    ok &= run(
        "if grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<\"]shulib/sim/' "
        "include/shulib; then exit 1; fi",
        "GUARD 2 — core is sim-free",
    )
    ok &= run(
        "find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort "
        "| awk '{print \"#include \\\"\"$0\"\\\"\"}' > /tmp/shulib_arm.cpp "
        "&& echo 'int main(){return 0;}' >> /tmp/shulib_arm.cpp "
        "&& arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion "
        "-Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp "
        "-c /tmp/shulib_arm.cpp -o /dev/null -Iinclude",
        "ARM gate — every header, one TU",
    )
    ok &= run(["python3", "tools/prepare_site.py", "/tmp/shulib_site_preflight"], "release gate (prepare_site)")
    return ok


def stage(message):
    if not preflight() or not _gates():
        raise SystemExit("\nPREFLIGHT FAILED — nothing staged, nothing changed.")

    print("── building the release commits ──────────────────────────────────────")
    source_sha = head_of(SOURCE)
    git("checkout", RELEASE, quiet=True)
    try:
        # docs/internal/ ALWAYS conflicts modify/delete, and the resolution is ALWAYS deletion.
        # That is not a merge accident: release/v2 exists to be shulib-v2 without the
        # development record, so the conflict is the mechanism, not a problem.
        subprocess.run(["git", "-C", REPO, "merge", "--no-commit", "--no-ff", SOURCE],
                       capture_output=True, text=True)
        git("rm", "-r", "-q", "-f", "--ignore-unmatch", "docs/internal")
        unresolved = git("diff", "--name-only", "--diff-filter=U")
        if unresolved:
            raise SystemExit(
                "UNRESOLVED CONFLICTS OUTSIDE docs/internal/:\n  "
                + "\n  ".join(unresolved.splitlines())
                + "\n\nThis is the case the script will not guess at. Resolve by hand, commit,\n"
                  "then re-run `stage`. (git merge --abort backs the whole thing out.)"
            )
        if any(p.startswith("docs/internal") for p in git("ls-files").splitlines()):
            raise SystemExit("INVARIANT 2 VIOLATED: docs/internal survived into the release index.")

        if not run(["python3", "tools/prepare_site.py", "/tmp/shulib_site_release"],
                   "release gate on the RELEASE tree"):
            raise SystemExit("The release tree does not pass prepare_site. Nothing committed.")

        git("commit", "-q", "-m", f"release: {message}",
            "-m", "Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>")
        release_sha = head_of(RELEASE)
        print(f"  built  {RELEASE} → {release_sha[:7]}")

        snapshot = git("commit-tree", f"{RELEASE}^{{tree}}", "-p", PUBLISH,
                       "-m", f"shulib v2 — {message}",
                       "-m", "Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>")
        print(f"  built  {PUBLISH} snapshot → {snapshot[:7]}")

        print("── the three invariants ──────────────────────────────────────────────")
        okay = True
        same = git("rev-parse", f"{snapshot}^{{tree}}") == git("rev-parse", f"{RELEASE}^{{tree}}")
        print(f"  {'PASS' if same else 'FAIL'}  1. snapshot tree is identical to {RELEASE}")
        okay &= same
        parents = len(git("rev-list", "--parents", "-n", "1", snapshot).split()) - 1
        print(f"  {'PASS' if parents == 1 else 'FAIL'}  2. snapshot has exactly one parent ({parents})")
        okay &= parents == 1
        leaked = [p for p in git("ls-tree", "-r", "--name-only", snapshot).splitlines()
                  if p.startswith("docs/internal")]
        print(f"  {'PASS' if not leaked else 'FAIL'}  3. no development record in the snapshot")
        okay &= not leaked
        for ref, sha in ((SOURCE, source_sha), (RELEASE, release_sha), (PUBLISH, snapshot)):
            ff = subprocess.run(["git", "-C", REPO, "merge-base", "--is-ancestor",
                                 f"origin/{ref}", sha], capture_output=True).returncode == 0
            print(f"  {'PASS' if ff else 'FAIL'}  4. {ref} push is a fast-forward")
            okay &= ff
        if not okay:
            raise SystemExit("\nINVARIANTS FAILED. The commits exist locally; nothing was pushed.")

        with open(STAGED, "w", encoding="utf-8") as f:
            f.write(f"{source_sha}\n{release_sha}\n{snapshot}\n")
        print(f"\nSTAGED. Review, then:  python3 tools/release.py push")
        print(f"  {SOURCE} {source_sha[:7]} · {RELEASE} {release_sha[:7]} · {PUBLISH} {snapshot[:7]}")
    finally:
        git("checkout", SOURCE, quiet=True)


def push():
    if not os.path.exists(STAGED):
        raise SystemExit("Nothing staged. Run `python3 tools/release.py stage \"<message>\"` first.")
    source_sha, release_sha, snapshot = open(STAGED, encoding="utf-8").read().split()
    print("── pushing (fast-forward only; no force anywhere) ────────────────────")
    for spec, label in ((f"{source_sha}:refs/heads/{SOURCE}", SOURCE),
                        (f"{release_sha}:refs/heads/{RELEASE}", RELEASE),
                        (f"{snapshot}:refs/heads/{PUBLISH}", PUBLISH)):
        print(f"  pushing {label} …")
        print("   ", git("push", "origin", spec).replace("\n", "\n    ") or "ok")
    os.remove(STAGED)
    git("fetch", "origin", "--quiet", check=False)
    same = git("rev-parse", f"origin/{PUBLISH}^{{tree}}") == git("rev-parse", f"origin/{RELEASE}^{{tree}}")
    print(f"\n  {'PASS' if same else 'FAIL'}  origin/{PUBLISH} tree == origin/{RELEASE} tree")
    print(f"  origin/{PUBLISH} is now {head_of(f'origin/{PUBLISH}')[:7]} — the site publishes from here.")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "check":
        raise SystemExit(0 if (preflight() and _gates()) else 1)
    if cmd == "stage":
        if len(sys.argv) < 3:
            raise SystemExit('stage needs a message: release.py stage "what this release means"')
        stage(sys.argv[2])
    elif cmd == "push":
        push()
    else:
        raise SystemExit(__doc__)
