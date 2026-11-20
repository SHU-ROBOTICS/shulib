# RMP1-C — named routes, robot two on the field, and "both robots run this together?"

> Brief by the coordinator, 2026-11-17 ~10:20. **The team lead's go, 2026-11-17: "yeah do that"** (to: named routes in
> the builder, with robot two's ghost and the both-robots switch). The route builder stays open to the 21st (RMP1's D19);
> library code is NOT this run's. Base: main AFTER RMP1-A2u (PREVIEW) lands — the page files are shared.

## 1. Why (the team lead, at the robot, 2026-11-17)

- "you can name routes right?" — today the page makes ONE route, always named `skills` (checks.mjs `nameIssues`: no
  rename; the export writes the one marked region of `src/team/skills_route.cpp`). The team wants several routes —
  the competition's skills route, test routes, practice routes — each picked BY NAME on the brain's Auton Tester menu.
- His first real route started at (-36, 54.076) facing -90 deg. Mirrored for robot two (the skills diagonal), robot
  two's start is (-54.076, 36) — the two 15 in squares 4.35 in apart: the host pair check
  (`test/mirrored_pair_check.hpp`, autons (1d)) fails the suite. The page never said so: robot two is not drawn, and the
  page has no pair check. "I don't want to drag anything that is unintuitive, it should be legit." AND "there isn't
  always going to be two robots on the field at all times" — the gap only matters when both robots run the route AT THE
  SAME TIME.

## 2. Decisions

- **D1 — Named routes in the page.** The project file already holds `routes[]`. The page gets a route switcher: add,
  rename, duplicate, delete (delete confirmed in-page), one route on the field at a time, undo/redo across them. A name
  is what the brain's menu shows: the library's own rules for an auton's name (find them — the registry / the
  Auton Tester menu's width; refuse in plain words, beside the build's own rule); unique in the project (case-
  insensitive); `skills` keeps its meaning (the competition's skills route — the one the competition program and the
  docs point at). An old project (one route `skills`) opens unchanged.
- **D2 — Export: every route into the code, by name.** All routes go inside the ONE marked region of
  `src/team/skills_route.cpp` (BEGIN/END unchanged — the region contract §6.3 is updated, its GL0 golden re-made and
  its C++ twin kept compiling): one steps function per route and ONE list of entries the region defines. Outside the
  region (plumbing the coordinator owns, written by this run, minimal): a function both autons files call, e.g.
  `void addTeamRoutes(competition::AutonRegistry&, TeamRouteUse)` — robot one registers every route AS WRITTEN; robot
  two registers each route marked "mirrored for robot two" MIRRORED (the existing `competition::mirrored(...,
  kSkillsMirror)`), and each route not so marked as written. The twin rule (test/robot_twin_test.cpp: the two robots'
  lists equal but for the mirror) holds for every route. NO library (include/) change.
- **D3 — "Both robots run this together?"** A route mirrored for robot two asks ONCE, plainly, when the mirror is
  turned on: "Will both robots run this at the SAME TIME?" (Yes / No — no silent default; the answer shown on the route
  and changeable). Saved in the project file; exported where the host pair check can read it per entry WITHOUT a library
  change (e.g. a `bool` in the region's entry list, read by the plumbing into a test-visible table).
- **D4 — Robot two on the field.** For a mirrored route, the page draws robot two's mirrored start and path (dimmed,
  labelled "robot two (mirrored)") beside robot one's — always, so the mirror is never invisible; hidden only when the
  route is not mirrored.
- **D5 — The too-close check in the page,** ONLY for a route answered "Yes, together": the same rule as
  `test/mirrored_pair_check.hpp` (the robots' squares — `kRobotSquareIn` — at least `kMirrorGapIn` apart at every
  moment of the page's own path prediction, the start included), flagged like the wall checks: the step and the moment,
  where each robot is, and the plain fix ("move robot one's start away from the diagonal", "send ONE robot there with
  isMirrored(ctx)"). Numbers from the C++ (baked, like the robots' descriptions) — never retyped in JS.
- **D6 — The host pair check follows D3:** `autons (1d)` checks a mirrored entry only when its route says "together";
  a route answered "No" is reported as not checked, by name. A test proves both ways (the team lead's real start fails
  "together" and passes "not together").

## 3. Tests (adversarial; each seen RED against a mutant)

Page model (node): names (empty, too long, duplicate, case-only duplicate, a quote/backslash), rename keeps steps and
undo, delete/duplicate, the old one-route project, the export of N routes byte-for-byte (goldens re-made with
`goldens.mjs --write`, each read), the region contract (GL0) and its C++ compile (the suite compiles every golden), the
pair check at the start / mid-path / never (the team lead's real start: too close; 3 in further out: clear), "together"
off = no pair check. C++: the plumbing registers every route on both robots (twin rule), robot two mirrored only where
marked, the pair check honours D3 both ways. Browser (Playwright, Firefox + Chromium): the switcher, the ghost, the
question, the warning and its fix — screenshots light/dark at 1440 and 1024.

## 4. Docs

Guide 22 (named routes; robot two on the field; the question), `docs/route-builder.md`, the changelog (top of API 2.9:
no library change), RMP1-PROGRESS (`## RMP1-C`). Short and plain.

## 5. Landmines

- The coordinator holds the team lead's real routes outside the repo (scratch/skills_route.jal-hard.cpp) — do not
  write any team route into the repo; your goldens are TEST routes (G4).
- A2u's PREVIEW plays ONE robot (the two-robot PREVIEW, SM4, has no owner) — do not build it; a "together" route's
  PREVIEW stays robot one's, saying so.
- The page files are shared with A2u/B — build on main after A2u lands.

## 6. ADDED 2026-11-17 ~10:40 — the team lead at the field, on the MECHANISM steps and small moves (same run)

The team lead: "the mechanism section on the builder is a bit unintuitive. like for the cascade … you'd think you'd see
like go up x inches go down x inches. and base (return to bottom) and for claw you'd think you'd see like open and close
also micro adjustments like things within like 5 inches of each other or less are very hard on the builder".

- **D7 — A lift's steps in plain words** (robot one's cascade has NO levels: today the page offers only "Home it" and
  "Wait"): **"Go up _ in"**, **"Go down _ in"**, **"Move to _ in"**, **"Back to the bottom"**, "Home it (find the bottom
  stop — do this first)", "Wait until it is done". The library has NO relative move: the page TRACKS the lift's planned
  height through the route (unknown until a home or an absolute move; after home = the bottom row's height; after "move to
  X" = X) and exports every step as the library's constant target `r.move(cascade.to(<abs>_in), <limit>)` with the step's
  own words in its comment ("Step 4 — the cascade UP 6 in, to 18 in"). A relative step while the height is unknown, or a
  target outside the lift's travel (its description's range), is refused in plain words with its one-click fix. Levels,
  when a robot has them, stay offered as now. Also an ARM (degrees) gets "Turn up/down _ deg" and "Move to _ deg".
- **D8 — A pneumatic's steps named by its positions:** each position its own menu button — "Open claw", "Close claw",
  "Ramp down", "Ramp up", "Front toggle out/in" (the positions' own names from the robot's description) — no "Set a
  position" then a list. The menu groups each mechanism under its name, plainly ("Claw", "Cascade", ...), the kind
  secondary.
- **D9 — Small moves:** "Drive forward _ in" and "Back up _ in" steps (relative to the planned pose at that point of the
  route; exported as the driveTo / reverseTo point they compute, the words in the comment; refused where the planned pose
  is unknown); arrow keys nudge the selected point 1 in (Shift: 0.25 in), with undo; every point's x / y typed in the
  inspector (exact; snap only when the student drags). Two points 1 in apart must be easy to place and to select.
- **D10 — Tests:** the height tracker (home → up → down → to → bottom; unknown height refused; out of range refused),
  every new step's export byte-for-byte AND compiled/run in the suite (goldens), the nudge keys, the relative drive's
  point; browser tests for each new menu item and the nudge; screenshots of the new mechanism menu (light/dark).
