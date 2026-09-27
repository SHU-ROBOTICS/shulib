# Probe programs behind the next-wave briefs (2026-09-26)

Small host programs the read-only brief drafters, checkers and revisers wrote to measure claims their
briefs make (the median on NaN / ±Inf, a minor-version pin under `-Werror=type-limits`, the ramp fit's
identifiability on the host test plant, the reversed-approach geometry). They are evidence, not project
code: nothing builds them. Each was written in the coordinator's ignored scratch folder and copied here
at the briefs' commit so the numbers the briefs quote stay traceable.

| File | Cited by | Measures |
|---|---|---|
| `DIAG1-B-probe_recheck.cpp` | `../../chunks/DIAG1-partB-median-nonfinite.md` | `hal::medianInPlace` on non-finite members, against the real header |
| `H1-minor_pin.cpp` | `../../chunks/H1-shul2-wire.md` | `kVersionMinor >= 0` on a `u16` fails with `-Werror=type-limits` |
| `R5a-probe.py` | `../../chunks/R5a-gains-and-noise-on-the-sd-card.md` | the ramp/step fit on the host test plant's model (kS shift, curve per segment) |
| `L9-geom_probe.cpp`, `L9-wrap_probe.cpp` | `../../chunks/L9-rest-reverse-driving-and-one-line-boot.md` | where a reversed approach's mutation is detectable; angle wrap |
