# Chunk DOCS2 — the generated reference covers the whole public API

> **Called by the team lead on 2026-08-14**, during DOCS1, on seeing that
> `include/shulib/` holds fourteen subsystems while `docs/api/` documents two types.
> The question was fair and the answer was not a good one: *the generator was pointed at the two
> types a routine author touches, and has never been pointed anywhere else.*
>
> **Status:** brief. Not started. **Live log:** `DOCS2-PROGRESS.md`, to be created first.
>
> **Why this is NOT part of DOCS1:** DOCS1's landmine L3 is "do not fix code", and this chunk
> writes roughly four hundred documentation comments into library headers. It would also hold the
> release hostage: DOCS1's whole purpose is that merging to `main` publishes, and the documents
> are true *now*. Merge first, then do this.

---

## 1. The measurement, taken before the brief was written

Every number below came from running the tool's own parser over the tree
(`api_doc_tool.parse_header`), not from estimating.

| Subsystem | Headers | Public items | **Undocumented** |
|---|---:|---:|---:|
| `hal/` | 22 | 238 | **151** |
| `control/` | 6 | 63 | **54** |
| `motion/` | 6 | 127 | **47** |
| `localization/` | 12 | 143 | **43** |
| `diag/` | 12 | 185 | **28** |
| `math/` | 3 | 33 | **26** |
| `chassis/` | 3 | 98 | **18** |
| `units/` | 1 | 14 | **13** |
| `kinematics/` | 3 | 26 | **10** |
| `manipulation/` | 2 | 25 | **8** |
| `sequence/` | 1 | 19 | **1** |
| **TOTAL** | **71** | **971** | **399** |

*(`sim/` and `hal/fake/` excluded — test-only by CI guard and by charter.)*

**Two things this table settles.** The work is ~400 comments, not a configuration change. And
the debt is *not* where you would guess: `sequence/` is already complete, `diag/` — the largest
omission by user impact — is 85% documented, while `control/` and `hal/` carry most of the debt.

## 2. The blocker nobody knew about

**The generator cannot parse four headers at all.** It refuses, loudly and correctly:

```
api_doc_tool: a PUBLIC nested type was found and this tool does not handle them:
    BlackboxReader :: struct Frame { ... }
Extend the tool deliberately rather than letting its members vanish from the reference
(which is the exact failure the doc coverage gate exists to prevent).
```

| Header | Nested public type |
|---|---|
| `diag/blackbox_reader.hpp` | `BlackboxReader::Frame` |
| `diag/health_monitor.hpp` | `HealthMonitor::Observations` |
| `diag/tick_attribution.hpp` | `TickAttribution::PhaseScope` |
| `localization/tracking_wheel.hpp` | `TrackingWheel::Role` |

**So step one is a tool change, not a documentation change** — and the tool's own error message
already rules on how to do it: extend it deliberately, never let members vanish. Anchor names,
`check-fresh` byte-stability and the alphabetical index all have to keep working for a nested
type. That is a design decision with a rejected alternative (flattening `Outer::Inner` into the
parent's member list, which loses the nesting a reader needs), and it belongs in this brief's
successor, written after reading the parser.

## 3. The tension this chunk has to rule on

**Gating an unfrozen seam pins it, and most of this tree is unfrozen.**

Adding a header to `TARGETS` does two things at once: it generates a page, *and* it puts every
public member under `check-coverage`. Freeze Register rows F11–F14 each say, in writing,
"deliberately not in the api-doc coverage TARGETS until it freezes". That ruling was made four
separate times and this chunk is about to walk into it.

The counter-argument, which is real and should be weighed rather than dismissed: the coverage
gate requires that a member *have* documentation, not that the documentation stay the same.
Changing an unfrozen seam stays legal — it costs one `///` edit and a regeneration. That is
**friction, not a freeze.**

**The tension to rule on:** is that friction acceptable on seams the project has deliberately
kept liquid? Three shapes are available, and the chunk must pick one and say why:

1. **Generate and gate everything** — simplest, honours the team lead's call literally, and
   overrides four recorded rulings. If chosen, F11–F14's parentheticals must be *amended in the
   register*, not silently contradicted.
2. **Split `TARGETS` from the coverage set** — generate pages for everything, gate only the
   frozen contracts. Costs a tool change; keeps both properties. **This is the shape to beat.**
3. **Frozen contracts only** — F1–F5 join F6/F10; the unfrozen seams stay documented in headers
   and the guide. Smallest, and leaves the largest user-facing gap (`diag/`) unfixed.

**Whichever is chosen, the reference's own "what is not here" paragraph must be rewritten to
match** — DOCS1 already had to correct that paragraph once for claiming a rationale the repo
contradicted, and it is the sentence most likely to go wrong again.

## 4. Scope

**In:** the tool extension for nested public types; the `TARGETS`/coverage ruling above; the
~400 documentation comments; regeneration; the register amendment if shape 1 is chosen; the nav.

**Out:** changing any signature or behaviour. A `///` comment that reveals a genuinely wrong API
is a *finding to report*, not a fix to make here — and it is worth expecting: DOCS1 found
`hal/battery.hpp` describing a battery-compensation model the code does not implement, and that
was one header read by accident. **Four hundred members is a large net for that class of bug.**

**Explicitly rejected:** writing filler. A `///` that restates the member's name ("`/// Sets the
voltage.`" on `setVoltage`) satisfies the gate and teaches nothing — it converts a real gap into
a hidden one, which is worse, because the page then *looks* complete. Most of these headers
already carry excellent `//` banner prose explaining *why*; the work is largely promoting and
sharpening that, not inventing text.

## 5. Definition of Done

- [ ] Nested public types parse, generate, anchor and survive `check-fresh` — with a test
- [ ] The `TARGETS`/coverage tension ruled, with the rejected alternatives written down
- [ ] Freeze Register amended if the ruling contradicts F11–F14's recorded parentheticals
- [ ] Zero undocumented public members in every targeted header (`check-coverage` green)
- [ ] No filler: every comment says something the member's name does not
- [ ] `docs/api/` regenerated; `check-fresh` green; nav updated
- [ ] Every API defect found while writing is **reported, not fixed** — with a list
- [ ] All gates green, both guards, ARM, and the release gate

## 6. Landmines

**L1 — The tool refuses nested types on purpose.** Do not make it skip them. Its error message is
the ruling: letting members vanish is the exact failure the coverage gate exists to prevent.

**L2 — Do not freeze by accident.** Four register rows say these seams are deliberately ungated.
Overriding that is allowed; doing it *silently* is not.

**L3 — Do not fix code.** ~400 members is a wide net for real API bugs. Report them.

**L4 — Filler passes the gate.** The gate counts comments, not meaning. Only a reader catches
filler, which is the same lesson DOCS1 learned about prose: the machine checks existence, a person
checks truth.
