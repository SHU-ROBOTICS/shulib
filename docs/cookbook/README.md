# Recipes: which to read

<a id="the-shulib-cookbook"></a>Short, complete code for the job in front of you — read out of order, mid-task. Every recipe is a
real program, compiled and run on the host test plant in every build, so a recipe that stopped
working would fail the build before you saw it. To learn the ideas first, read
[The steps of a route](../guide/09-the-recipe-api.md).

<a id="how-to-use-a-recipe"></a>Each recipe says **when to use it**, gives **the code** (quoted word for word from
the compiled file it runs in — a test, or the team's own driver file), **why it is written that way**,
and **what to watch out for**. The recipes assume `using namespace shulib::units::literals;` and the
`Chassis` and `Routine` names in scope. Exact signatures are in the [API reference](../api/README.md),
not here.

## [Getting there](01-getting-there.md)

| <a id="the-recipes"></a><a id="1--getting-there"></a>Recipe | Use this when |
|---|---|
| [The skeleton](01-getting-there.md#the-skeleton) | You are starting a new routine from nothing |
| [A two-goal side run](01-getting-there.md#a-two-goal-side-run) | Your auton visits several places and does something at each |
| [A reusable scoring step](01-getting-there.md#a-reusable-scoring-step) | The same three-or-four-step sequence appears more than once |
| [A waypoint sweep](01-getting-there.md#a-waypoint-sweep) | You want one call to drive a whole path |
| [Flow through points: a curve, and its fallback](01-getting-there.md#flow-through-points-a-curve-and-its-fallback) | A tank route should pass through a point without stopping and turning there |

## [When a step fails](02-when-a-step-fails.md)

| <a id="2--when-a-step-fails"></a>Recipe | Use this when |
|---|---|
| [Bail out and park somewhere useful](02-when-a-step-fails.md#bail-out-and-park-somewhere-useful) | A failed grab should abandon the plan, not carry on with empty jaws |
| [Attempt something and keep going](02-when-a-step-fails.md#attempt-something-and-keep-going) | One missed goal in a sweep should not end the routine, and you want its outcome in a variable |
| [Carry on past a missed piece](02-when-a-step-fails.md#carry-on-past-a-missed-piece) | A grab can miss, and a miss should cost that piece, not the auton |
| [Give up on this piece, go for the next](02-when-a-step-fails.md#give-up-on-this-piece-go-for-the-next) | A miss makes the next few steps pointless, but not the rest |
| [Recover from a broken sweep](02-when-a-step-fails.md#recover-from-a-broken-sweep) | A trajectory stopped partway and you need to know how far it got |

## [Timing and partners](03-timing-and-partners.md)

| <a id="3--timing-and-teammates"></a>Recipe | Use this when |
|---|---|
| [Wait for your alliance partner](03-timing-and-partners.md#wait-for-your-alliance-partner) | Your path crosses your partner's and one of you has to go second |
| [Wait for a condition, with a deadline](03-timing-and-partners.md#wait-for-a-condition-with-a-deadline) | The next step is meaningless until something is true |
| [Wait, but go anyway](03-timing-and-partners.md#wait-but-go-anyway) | The thing you are waiting for is nice to have, not required |
| [Fit the match window](03-timing-and-partners.md#fit-the-match-window) | Your routine is longer than the autonomous period allows |

## [Routes for a tank](04-drivetrains.md)

| <a id="4--drivetrains"></a>Recipe | Use this when |
|---|---|
| [A tank routine](04-drivetrains.md#a-tank-routine) | Your drivetrain cannot slide sideways |
| [Budget a sideways leg](04-drivetrains.md#budget-a-sideways-leg) | Your drivetrain strafes, but slowly (an H-drive) |

## [Mixing the two styles](05-mixing-tiers.md)

| <a id="5--mixing-tiers"></a>Recipe | Use this when |
|---|---|
| [Call the full API mid-routine](05-mixing-tiers.md#call-the-full-api-mid-routine) | A recipe step cannot express what this one leg needs |
| [Make a direct call count as a step](05-mixing-tiers.md#make-a-direct-call-count-as-a-step) | You dropped a tier and still want the chain to stop if it fails |
| [Re-seed the estimate mid-routine](05-mixing-tiers.md#re-seed-the-estimate-mid-routine) | The robot has just squared itself against a wall |
| [Re-seat x against a wall](05-mixing-tiers.md#re-seat-x-against-a-wall) | The robot is parked beside a field wall and a side distance sensor faces it |

## [Buttons for your mechanisms](06-driver-control.md)

| <a id="6--driver-control"></a>Recipe | Use this when |
|---|---|
| [Drive a mechanism from two buttons](06-driver-control.md#drive-a-mechanism-from-two-buttons) | A claw, arm or intake moves while a button is held and stops when the driver lets go |
| [Open and close a pneumatic claw with one button](06-driver-control.md#open-and-close-a-pneumatic-claw-with-one-button) | A claw on air opens on one press and closes on the next |
| [A whole driver file: robot two's](06-driver-control.md#a-whole-driver-file-robot-twos) | You want one robot's buttons, feel and loop in one file a student can defend |

## [Mechanisms in a route](07-mechanisms.md)

| <a id="7--mechanisms"></a>Recipe | Use this when |
|---|---|
| [An arm with presets, held while the robot drives](07-mechanisms.md#an-arm-with-presets-held-while-the-robot-drives) | An arm or a lift must get to a height and stay there while the robot drives |
| [Start the lift partway through a drive](07-mechanisms.md#start-the-lift-partway-through-a-drive) | A mechanism should start partway along a drive — not before it, not after it |

## [Skills routes](08-skills-toggles.md)

| Recipe | Use this when |
|---|---|
| [A skills route that visits the Toggles](08-skills-toggles.md#a-skills-route-that-visits-the-toggles) | Your skills route should flip each Toggle to its Quadrant's colour, checking with the camera first |

## [One route, two robots](09-one-route-two-robots.md)

| <a id="9--one-route-two-robots"></a>Recipe | Use this when |
|---|---|
| [Write once, run mirrored](09-one-route-two-robots.md#write-once-run-mirrored) | Robot two is robot one's build and runs robot one's skills route mirrored, at the same time |

## What every recipe assumes

1. <a id="the-three-things-every-recipe-assumes"></a>**Steps run in the order you write them.** Each step blocks until its motion ends, so a printed
   line, a sensor read or an `if` between two steps runs exactly there. (Print with `serialLine`,
   never `printf` — [Reading the log](../guide/11-reading-the-diagnostics.md) says why.)
2. **A failed step stops the chain and parks the robot** — unless the step was written after
   `onFail(carryOn)` or `onFail(skipTo("..."))` ([When a step fails](02-when-a-step-fails.md)). You check
   `r.ok()` to *do* something different, not to stay safe.
3. **Nothing here is a ceiling.** Every step is one call on the `Chassis` underneath (`r.chassis()`);
   dropping to it for one leg is the intended move ([The API in words](../guide/10-the-api.md)).

<a id="what-is-not-in-here"></a>**Not in here:** strategy — which goals to score, and in which order, is your team's to decide and
defend; and anything untested — a recipe that is not compiled and run in a test is not a recipe.
