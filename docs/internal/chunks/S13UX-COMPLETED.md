# S13UX — COMPLETED (S13UX-A built 2026-11-19, LANDED 2026-11-19; host- and build-gated, NOT run on a robot in this form)

> Bench Tests' station 13 made plain so robot two's builders can measure its mechanisms alone: a "what to do now" line
> (the controller's link, L1, jogging, each cut), the lift's measuring guide with RECORD's number shown large, a 3 s jog
> cut that re-arms on letting go of L1 and holding it again (a fault cut still needs DONE), and each mechanism's declared
> current cap applied at the station (R1 safe on the cascade) — from `src/` alone, no `include/` change. The team lead's
> go 2026-11-19 ("put all resources into getting everything done"). Written FROM
> [`S13UX-PROGRESS.md`](S13UX-PROGRESS.md); brief [`S13UX-station-13-made-plain.md`](S13UX-station-13-made-plain.md).

## Rounds

Build → three reviewers → fix → a fresh verifier → fix2 (cut once by the 5-hour limit, resumed), all green: suite 3,864
in its tree, src gate PASS and self-test OK (398), 11 mutations caught.

## The coordinator's merge (2026-11-19, `integrate`)

Merged onto main after RMP1-C (b9afb9a; the changelog's entries both kept); suite 3,893 and src gate PASS on the merged
tree with the setup tutorial (guide 23) squashed in beside it.

## Owed

- **On a robot:** station 13 run by the builders on robot two (its Stage A facts first), and the cap's `NOT SET` line
  never seen on robot one.
