# RMP1-A1 — the checks a person runs (the Windows laptop, and the non-coder's card)

Written by RMP1-A1's build (2026-11-15) from the RMP1 brief's A.4 ("the manual check on a team Windows laptop") and
§2.5.4 ("the usability task"), cut to what RMP1-A1 carries: **Export writes into the folder only once robot two's
update (ROB1-BT, which adds `src/team/skills_route.cpp`) is in the published build** — until then the page offers
Copy. **Updated by RMP1-A2u (2026-11-17):** PREVIEW's steps (4 and 7), and robot one's cascade has NO levels since
ROB1-C2 (its menu offers home and says "No levels yet: record them at Bench Tests station 13"), so the route's
mechanism step is the claw. Log what you find in [`RMP1-PROGRESS.md`](RMP1-PROGRESS.md).

## 1. On a team Windows laptop — the team lead, about 10 minutes, after A1 is published (D17)

1. Press **Win+R**, type `winver`, note the Windows version. Open Edge, go to `edge://version`, note the version.
2. Open **docs.shurobotics.com**, click the **Route builder** tab, then **Open the route builder**. No yellow
   "too old" box; the bottom bar names the library version and the commit it was built from.
3. Click a red start tile to place robot one; drag its red knob to turn it. Add a **Drive** (press D, click), a
   **Curve** (C, click two points, Enter), and **Mechanism → claw → Set a position** (M); choose **open** on the
   right. In the Mechanism menu the cascade says **No levels yet: record them at Bench Tests station 13** and offers
   **Home it**. The bottom bar says **Ready to export** (a red number on **Export** counts what is left to fix;
   Export itself lists it, each with **Go there**).
4. Press **PREVIEW** (Ctrl+Enter): the robot plays the route within about a second, its path solid beside the
   dashed plan. Drag the timeline; press Space, Left and Right; click a step in the list.
5. In the top bar, click **Connect your shulib folder**, pick the repository's top folder, then **Save** (Ctrl+S):
   `src/team/route_maker.json` appears in File Explorer.
6. **Export** (Ctrl+E): **Copy the code** works (paste it into Notepad). *After robot two's update is published:*
   **Write into src/team/skills_route.cpp**; in VS Code the file changed only between the two `ROUTE MAKER` lines,
   the status bar shows the same line ending (CRLF or LF) as before, and `git diff` shows only the region.
7. **Download for offline use** (it saves one file; if Edge opens it instead, right-click → **Save link as**), turn
   Wi-Fi off, double-click the downloaded `shulib-route-builder.html`: it opens, and steps 3, 4 (PREVIEW) and 6
   (Copy) work with no network. **Then, in that same downloaded file in Edge: is there a "Connect your shulib folder" button in the
   top bar?** If there is, press it, pick the repository's top folder, and **Save**: does `src/team/route_maker.json`
   change in File Explorer? Write down which happened — the button and the write, the button but no write, or no
   button and "Save downloads · Export copies" in its place. (This is the brief's open question A.10 / landmine 23:
   the folder API needs a secure page, and a file opened from disk is the browser's call. CI records it for
   Playwright's Chromium on Linux — `isSecureContext=true, showDirectoryPicker=true` there on 2026-11-15 — and for
   the Windows Edge and Chrome rows in each run's log; only this step tries the real write.)
8. If Firefox is installed, open the builder in it: building and Copy work; the top bar says Save downloads and
   Export copies (folder writing needs Edge or Chrome).
9. Tell the coordinator anything that looked wrong — with step 1's two versions.

## 2. The usability card — a NON-CODER, unaided (§2.5.4)

The coordinator puts the builder (or another non-coding team member) in front of the published route builder — on a
team Windows laptop when one is at hand — with ONE card, no help and no terminal, and a timer:

> **Make a 3-step route: start on a red tile, drive somewhere, curve to somewhere else, and open the claw. Preview
> it, then export it.**

- **Pass:** done in **10 minutes or less**, with no help. *(Before 2026-11-17 the card raised the cascade to a level
  and met "home it first"; robot one's cascade has no levels since ROB1-C2 — that card comes back with its levels.)*
- **Log every hesitation** (where they paused, what they clicked first, what they read) — the findings go to RMP1-C.
- The export must also RUN: the suite's golden harness already compiles and runs every export shape on robot x's
  fixture lift (`test/route_maker_golden_test.cpp`); the card's own export is checked by the coordinator's
  end-to-end run after ROB1-BT (the brief's HS9).
- **Before the team member:** each reviewer of the run does the same card COLD, using only the page and its `?`
  help, and logs every hesitation in their review.

## 3. The UX lens every reviewer applies (§2.5.3), beside correctness

The tokens used everywhere (no stray colour or size); one primary action per area; every control labelled, with
hover help in plain words; empty and error states as the brief's §1.4; a keyboard-only path through "make a route,
export"; focus visible; contrast ≥ 4.5:1 in both themes; nothing under 32 px to hit; motion ≤ 200 ms and none under
reduced motion; the words a non-coder reads (no "Routine", no "precondition" in the page's own sentences — the
library's quoted words sit beside a plain one); no "honest" disclaimers. The screens to judge are the run's
screenshots (`.claude/coordinator/scratch/rmp1-a1/shots-review/`, Playwright's Firefox: every A1 screen in light and
dark at 1440 × 900 and 1024 × 768): Chromium and Edge come from the CI run's artifacts (`route-builder-shots-*`).
