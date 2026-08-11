# Independent verification harnesses

> **These exist because step 4 of the chunk loop is "verify independently — never take the report
> at face value."** Running the checks by hand each time invites skipping the inconvenient ones.
> These scripts re-derive everything from scratch: rebuild, re-run the suite, re-run both CI
> guards, re-run the ARM gate, and re-check the chunk's own non-negotiables.

Internal, like the rest of `docs/internal/` — dropped when `shulib-v2` squash-merges to `main`.

## Using them

```sh
docs/internal/verify/verify-d2.sh     # run from the repo root
```

Exit code is nonzero if any mechanical gate fails. Each script ends by printing what it
**cannot** mechanize — read that list; it is the part that needs a person.

## What they check beyond the obvious

Both harnesses assert things a chunk is specifically *forbidden* to do, not just things it
should achieve:

- **Nothing was committed** (chunks are told "do not commit"), and HEAD is where it should be.
- **The tree is dirty** — a clean tree after a chunk means it produced nothing.
- **F6 is still unfrozen** (for any chunk before D2) across all four notice locations.
- **No control math leaked into the recipe layer**, which would violate delegate-only.
- **The C7/C8 removability property** — no public doc may link into `docs/internal/`, or the
  squash-merge to `main` breaks.
- **The guide is quoted verbatim** — every non-blank line inside a ```` ```cpp ```` block in
  `docs/guide/*.md` must appear in `test/guide_examples_test.cpp`. This is the anti-rot coupling
  the guide depends on, and it is the check most likely to catch real drift: a rename in the test
  file silently invalidates every chapter quoting it.

## A caution learned the hard way

`verify-d1.sh` initially reported a removability FAILURE that turned out to be **C8's**, not
D1's — five pre-existing prose mentions of completion-record names, none of them links. The
harness was stricter than the property actually required.

**A red gate is a question, not a verdict.** Before attributing a failure to the chunk under
review, check whether it predates the chunk (`git show HEAD:<file>`) and whether it violates the
real property or only a proxy for it. The narrow four-term grep
(`internal/|chunks/|RESUMING|build-order`) is the true removability gate; the broader one is a
release-review nit.
