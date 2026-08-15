# Publishing the documentation — the path, and what is actually done

> Written at chunk D3 (2026-08-12), when the generated API reference landed. Internal, like the
> rest of `docs/internal/`; dropped when `shulib-v2` squash-merges to `main`.

---

## The honest status, first

**UPDATED 2026-08-14 — this section said "Nothing is published" until DOCS2 read it back.**

The site is live: **docs.shurobotics.com**, over enforced HTTPS, published from `main` by
`.github/workflows/pages.yml`, which runs `check-fresh` before it renders anything. The roadmap's
M7 line is `[x]`. The paragraph below describes what D3 delivered and is kept because the
reasoning still holds; it is history, not status.

Two things have changed since it was written, and both matter to anyone editing the reference:

- **DOCS2 pointed the generator at the whole tree.** `docs/api/` is a page per shipped header —
  117 of them plus an A–Z index, covering 1,625 public entities — and the target list is a GLOB,
  not a list. See the regeneration table at the bottom of this file.
- **The site nav is generated too**, into `mkdocs.yml` between markers, and byte-checked by
  `check-fresh`. A page absent from the nav was measured to publish *unreachable* with exit
  code 0, which is not a failure any gate would have reported.

What D3 delivered is the half that is documentation work rather than infrastructure work:

- the reference is **generated** from the headers (`tools/api_doc_tool.py`),
- its output is **web-portable**: plain CommonMark with relative links and explicit `<a id>`
  anchors, which renders correctly on GitHub today with no build step at all,
- it is **committed**, so it is readable in the repository by anyone, including on a phone,
- and it is **verified**: the build fails if the committed copy differs from a fresh run.

Standing up hosting is a separate job. This file is what that job needs to know.

---

## Why the output is already web-portable

Three properties, each chosen for this:

1. **CommonMark, no extensions.** No footnotes, no definition lists, no admonition syntax —
   nothing that renders on one site and appears as literal punctuation on another. The only HTML
   is `<a id="…"></a>` before each heading, which every static-site generator and GitHub itself
   pass through.
2. **Explicit anchors, not inferred ones.** Heading-derived anchors differ between GitHub,
   Docusaurus, MkDocs and mdBook, and overloaded members (`scheduler()` and `scheduler() const`)
   collide under all of them. The generator emits its own unique anchors and links to exactly
   those, so the table of contents works identically everywhere.
3. **Relative links only.** `../guide/README.md`, `../../include/shulib/chassis/chassis.hpp`.
   Nothing assumes a domain, a base path, or a URL scheme.

---

## The publish path, concretely

Three options, in increasing order of effort. None is done.

### Option A — GitHub Pages from the repository (smallest step)

Serve `docs/` with a generator that understands relative markdown links.

```yaml
# .github/workflows/pages.yml — SKETCH, not committed
on:
  push:
    branches: [main]
permissions:
  contents: read
  pages: write
  id-token: write
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Verify the reference is current
        run: python3 tools/api_doc_tool.py check-fresh
      - name: Render
        run: |
          pip install mkdocs mkdocs-material
          mkdocs build            # needs an mkdocs.yml, which does not exist yet
      - uses: actions/upload-pages-artifact@v3
        with: { path: site }
      - uses: actions/deploy-pages@v4
```

What it needs that does not exist: an `mkdocs.yml` (or equivalent) with the navigation tree, the
Pages setting enabled on the repository, and a decision about whether `main` or `shulib-v2` is
the published branch. Note the **`check-fresh` step is not optional** — publishing a stale
reference is worse than publishing none, because a published document looks authoritative.

### Option B — publish the markdown as-is

GitHub already renders `docs/api/README.md`, `docs/guide/README.md` and
`docs/cookbook/README.md` acceptably. "The team website" could, for now, be a pinned link to
those three files. Zero infrastructure, honest, and it works on a phone in the pits — which is
the actual use case. This is the option to take if nobody wants to own a site.

### Option C — VexBuilder hosts it

The master plan's §16 ecosystem has VexBuilder as the student-facing surface. If VexBuilder
grows a documentation panel, the generated markdown is the natural payload, and the command-id
manifest is a second one. This is a real option but it is downstream of work that does not
exist; do not plan around it.

---

## The rule that must survive whichever option is chosen

**The published reference must be generated in the publishing job, or verified fresh in it.**
The reason this chunk exists is that documentation which drifts is worse than documentation that
is missing: people trust it. A publish step that copies whatever is committed, without checking
it against the headers, reintroduces exactly that failure at the last possible moment — and at
the one moment where it is most authoritative-looking.

`python3 tools/api_doc_tool.py check-fresh` is one line. Put it in the job.

---

## What to regenerate, and when

| You changed | Do |
|---|---|
| A `///` comment in **any** header under `include/shulib/` | `python3 tools/api_doc_tool.py generate`, commit the result |
| A signature on a FROZEN surface (`Chassis` = F6, `Routine` = F10) | Read the freeze procedure FIRST. The freshness gate fires before the signature pins and names the wrong problem; regenerating would only make the break look intentional |
| A signature anywhere else | `generate`, commit. Being documented is not being frozen |
| A code example in a chapter | Change `test/*example*_test.cpp` first, re-quote the listing from it |
| **Added a header** | **Nothing.** The target list is a glob over `include/shulib/` — the header is covered because it exists. Write a `///` on everything public in it, run `generate`, and commit the new page *and* the `mkdocs.yml` nav line the tool writes |
| Added a public member to any shipped header | Write its `///`. The build fails naming it, its file and its line; an empty `///` does not count |
| Deleted or renamed a header | `generate`, then delete the orphaned page — `check-fresh` fails on a page the generator no longer produces |

The build runs `check-coverage`, `check-fresh`, `check-examples` and `check-removability` before
it compiles anything, so forgetting any of the above fails locally, not in review.
