# 20 — The camera: which colour the Toggle shows, and which piece is which

> **Read [chapter 10's "The camera — a frame at a time"](10-the-api.md#the-camera--a-frame-at-a-time)
> first** — it shows how the robot file declares the AI Vision camera, what the library writes to it at
> every boot, what a FRAME is, and how pixels become directions and distances that carry their error.
> This chapter is what the camera MEANS: the Toggle tracker (RED, BLUE, YELLOW — or UNKNOWN, with the
> reason), the piece finder (take ONE named piece, or none), and the feed that reads the camera once per
> period for both. Each one says only what it is sure of; everything else is UNKNOWN, and UNKNOWN never
> triggers anything.
>
> **What exists, and what does not yet.** The camera's first part (the device, the frame, the geometry)
> and its second (this chapter) are in. Robot two's camera and its bench page are the camera's third part;
> the automatic flip, driving to a piece and choosing our alliance colour are later chunks that USE what
> is here — none of them exists yet. **No AI Vision sensor has run this code.** Every number below is
> from the host test plant's camera model (written with its own projection, so a sign error in the
> library cannot cancel out) looking through a TRUE camera set somewhere inside the stated tolerances —
> and the beliefs it rests on are HA-192 to HA-203 in the
> [Hardware Assumptions Register](../hardware-assumptions.md), every one unmeasured.

## One loop reads the camera for everyone

A read costs one device call for the count and one per detection — up to 25 — and the library is
single-task: every device is read from the loop the program already runs, never from a task of the
library's. So the program's loop calls `tick()` once per pass, and `vision::CameraFeed` reads the camera
only when at least the team's period has passed since its last read BEGAN; every consumer is handed the
SAME frame. Two consumers never cost two reads, and nothing reads faster than the camera changes (VEX
states about 30 pictures a second — HA-192).

```cpp
struct CameraUser {
    vision::CameraFeed feed;
    vision::ToggleTracker toggle;
    vision::PieceFinder pins;

    CameraUser(hal::IVisionFrames& camera, const hal::IClock& clock, const vision::CameraModel& model)
        : feed{camera, clock, {.minReadPeriodSeconds = 0.033}},
          toggle{kToggleNumbers, model},
          pins{kPieceNumbers, model} {}

    void tick(const vision::PoseBound& pose) {  // once per pass of the loop the program already runs
        if (feed.tick()) {
            toggle.update(feed.latest(), pose);
            pins.update(feed.latest(), pose);
        }
    }
};
```

Ticked every 10 ms for a second, that reads the camera 25 times, not 100: a read happens on the first tick at
least the period after the last read started, so the period is rounded UP to the loop's tick — 0.033 s on a
10-ms loop is every fourth tick, 40 ms. `feed.latest()` is the
latest READ — a failed read included, with no detections — never the last good picture held over (a held
picture reads as a fresh "nothing changed"). `feed.stats()` counts the reads by how they went and TIMES
each one with the loop's own clock: the slowest read is the number HA-203 (one read is cheap enough for
the driver loop) needs from the bench.

`camera` is `*robot.camera()` — `nullptr` when the camera is absent (`robot.cameraAbsence()` says why),
and a consumer that needs it refuses to start. `model` is
`vision::CameraModel{vision::kAiVisionOptics, robot.config().sensors.camera.placement}` once
`vision::cameraModelProblem()` accepts the mount — an UNSET (unmeasured) mount is refused by name.

## The team's numbers — every one from the bench

Every threshold the tracker and the finder use is the TEAM's: none has a library default, and a config
left UNSET is refused by name (`toggleTrackerConfigProblem`, `pieceFinderConfigProblem`,
`cameraFeedConfigProblem`). Two of the finder's numbers mean something at 0 — `minScore` 0 takes every
score, `viewMarginPx` 0 asks for no margin — so their UNSET is −1, refused like the rest. These are the
numbers the library's own tests use; yours come from the camera's bench station, except `raisedMinIn`,
which is a fact of the field (below).

```cpp
inline constexpr vision::ToggleTrackerConfig kToggleNumbers{
    .ids = {.red = 1, .blue = 2, .yellow = 3},  // the descriptors taught in VEXcode's AI Vision Utility
    .holdSeconds = 0.15,       // the same colour this long before it counts
    .aliveSeconds = 0.10,      // and a usable read no older than this
    .minAspect = 3.0,          // a face's band: at least 3 times wider than tall
    .minLengthFraction = 0.6,  // and 0.6 .. 1.3 of the Toggle's 25.8 in long
    .maxLengthFraction = 1.3,
    .minRangeIn = 2.0,         // "in front of my arm": 2 .. 96 in from the lens,
    .maxRangeIn = 96.0,
    .bearingWindowDeg = 25.0,  // within 25 degrees of straight ahead
    .frozenMoveIn = 2.0,       // the lens moved 2 in or the robot turned 2 degrees
    .frozenTurnDeg = 2.0,      // and the band did not move a pixel: "camera frozen?"
};
inline constexpr vision::PieceFinderConfig kPieceNumbers{
    .holdSeconds = 0.15,     // FOUND must hold this long
    .aliveSeconds = 0.10,
    .minScore = 50.0,        // the model's score below this is ignored
    .lyingAspect = 1.5,      // a box 1.5 times wider than tall lies on its side
    .standingAspect = 0.8,   // one at most 0.8 stands
    .viewMarginPx = 4.0,     // the place must be 4 px inside the picture
    .raisedMinIn = 5.0,      // the lowest a raised piece sits on this field (a Goal's top), inches
};
```

## Freshness is a clock, not a counter

PROS gives no frame number and no camera timestamp (HA-192): a read only knows when it FINISHED. So both
consumers ask two questions of time — has the same answer held for `holdSeconds` on usable reads, and did
a usable read arrive in the last `aliveSeconds`? A read that faulted, tore, or found the camera not set
up breaks the hold: positive evidence only. And:

- **A frozen camera looks exactly like a still scene.** While the camera MOVES — its LENS travels more
  than `frozenMoveIn` (the robot's pose carried through the mount: turning in place hardly moves the
  robot's centre, but the lens sweeps), or the robot turns more than `frozenTurnDeg` — a band whose box
  has not moved by a single pixel is UNKNOWN "camera frozen?"; the first read whose box moves again clears
  it and RESTARTS the hold. **Parked, nothing can tell** — which is exactly where a robot checks whether
  its push worked. That check is never the camera's.
- **The picture is older than the read.** The image was taken some time before the read finished — how
  long, nobody has measured yet (HA-192). A robot that moved since is not where its pose says.
  Anything that pairs a frame with a pose — the finder's field position, the tracker's position gate —
  must be handed the pose's bound AT THE IMAGE's time:

```cpp
const vision::PoseBound atImage = vision::boundAtImage(atRead, speedInPerS, turnDegPerS, kImageAgeSeconds);
```

It widens the position by speed × age and the heading by turn rate × age. Until the age is measured on
the bench, use the camera's field positions only with the robot stopped — on the host plant, turning at
180 degrees a second, an image 66 ms old already put a Pin outside a field position computed without it.

## The Toggle tracker: RED, BLUE, YELLOW — or UNKNOWN, with the reason

It reports a colour ONLY when one band of that colour has passed every gate on every usable read for
`holdSeconds`, with a usable read within `aliveSeconds` — the SAME band: read to read its direction from
the lens and its range must agree with the last read's (allowing only for how far the lens moved), so
another band of the colour taking over starts the hold again:

```cpp
const vision::ToggleView seen = user.toggle.view(clock.now().value());
if (seen.colour == vision::ToggleColour::Red) {
    // a RED face is SEEN where a seated face could be — the far-range cue, never a set Toggle
    sawRed = true;
}
```

It reports COLOURS, never "ours" — which colour a robot wants is its alliance's (a later chunk). The
gates, in order, each turning a lookalike away:

1. **Kind and id** — a colour blob of one of the three Toggle descriptors.
2. **Shape** — a long, flat band (`minAspect`; a standing Pin's half is tall, a Cup is not a band), and
   more than twice a box edge's jitter tall: a thinner band cannot be measured.
3. **Cut** — a band touching the picture's top or bottom is a face leaving the frame: "cut by the image",
   never a colour from part of a face. A band cut at a SIDE has no measurable length, so it is judged only
   when it spans the picture's centre column (a real face close in does); a short lookalike reaching in
   from the edge — a 4.5-in licence plate, a lying Pin — is "cut".
4. **One camera explains it** — is there ONE camera inside the stated tolerances for which the band's
   bottom row is the face's lower edge, its top row the face's top edge 0.9–1.2 in further back (the face
   LEANS back; a vertical plate does not), and its width 0.6–1.3 of a Toggle's length at that range? Never
   by overlapping two separately widened intervals: the pitch error is common to both edges of one box.
5. **Not the bottom face** — the Toggle's BOTTOM face is a different colour, visible from a camera below
   it, and close in it mimics the face that counts. A band that could be the bottom face is kept only
   when the place its field face would occupy, directly above, is in the picture and empty; a band there
   makes this one the bottom face; that place out of the picture is UNKNOWN "maybe the bottom face".
   This rests on the camera SEEING the field face whenever it sees the bottom face: an empty place is
   read as "no face there". If glare made the field face's detection vanish on every read while its bottom
   face's fired, the bottom face's colour would read as sure — on the host plant, with the field face's
   detection removed, about one parking in ten did. (A field face detected but cut short, even by 6 px,
   never let it through; and refusing every band that COULD be a bottom face would have cost the real face
   at almost every distance from a lens below it, and still let some bottom faces through.) Whether the
   real camera ever misses the face that way is one of the bench's camera checks (HA-199).
6. **Where** — its bearing wholly inside ±`bearingWindowDeg`, its range overlapping the range window. The
   view carries both intervals, so a consumer can demand more.
7. **Position** (optional, `positionToleranceIn` > 0) — with a pose bound the team trusts, the band's
   field position must lie on one of the four Toggles' face lines; `toggleIndex` says which wall.

And over the whole read: two DIFFERENT colours passing at once — UNKNOWN "two colours", never "pick one";
a passing band beside a band of ANOTHER Toggle colour the gates could not judge (cut, or maybe a bottom
face) — UNKNOWN; a band with another Toggle colour's box directly ABOVE it — UNKNOWN "another colour
directly above"; a read reporting 24 or more detections — UNKNOWN "camera full" (the camera sends at most
24, so the band that contradicts the colour may be the one it never sent).

**What a colour MEANS.** The colour of the face the camera SEES where a seated face could be — NOT that
the Toggle is "set" in the game's sense (fully seated, a face parallel with its mounts, and no robot
touching it). A Toggle stuck mid-roll shows its DESTINATION face from below and can read as that colour:
on the host plant a Toggle rolled 20–100 degrees reported a colour in 78 of 216 parkings. Nothing the
camera measures tells seated from rolled, or touched from untouched. So the camera's colour is the
FAR-RANGE cue — "a Toggle ahead, showing red" — and **anything that pushes a Toggle must have the face's
colour confirmed AT THE PUSH POINT by a sensor that sees colour there** (an Optical sensor at the
flipper). A distance sensor says something is there, never which colour.

**What turning costs close in.** Off the camera's axis a horizontal edge looks SLANTED, so a face seen
from a turned robot fills a taller box than its band and stops looking like a face — while the thin
bottom face under it can pass. That is what the colour-above gate is for (without it, a robot turned
15 degrees to a RED Toggle 30 in away read the YELLOW bottom face as "sure"); the cost is that close in
the colour holds only when the robot is SQUARE. On the host plant, lenses 8, 10 and 11 in up, turned in
5-degree steps:

| Lens to the wall | 8-in lens | 10-in lens | 11-in lens |
|---|---|---|---|
| 14 in | square only | square only | square only |
| 20 in | square only | square only | square, and 5° one way |
| 30 in | square only | ±5° | −15° .. +10° |
| 40 in | ±5° | ±15° | ±15° |
| 48 in | ±10° | ±15° | ±15° |

Never past 15 degrees. Square up before judging. Over the 390 turned parkings (−30..+30 degrees,
14..48 in): 0 wrong colours, 106 RED.

**What it cannot do — the lookalikes.** A colour band the gates cannot tell from a face is reported as one
(HA-199). With the position gate off, 70 of 540 lookalike bands placed 30 in in front of a wall read RED
on the host plant, and a red LED strip on the wall's top reads RED at 20 in and closer. With the position
gate on and a pose the team trusts, none of the 540 did, and the strip never did — none of them is where a
Toggle's face is. What the gate CANNOT remove is a red strip about a Toggle's length lying ON a Toggle's
face line: a vertical plate there reads as a face from about 22 in out (closer, the face's lean tells them
apart) — with the gate on, from 22 to 30 in, named as that Toggle (farther, its field position is too vague
for an 8-in limit). A frozen camera parked at a Toggle is never caught.

### Where the camera sits decides what it sees

A lens about level with the face's lower edge looks along it: the lower edge's ray is too near level to
range, and the bottom-face gate cannot see above the band. On the host plant (a robot square to a seated
Toggle, parked every 2 in from 80 in to 4 in):

| Lens height, pitch | The colour reported from (far) .. to (near), lens to the wall |
|---|---|
| 8 in, level | 66–70 .. 14 in |
| **10 in, level** | **68–72 .. 12 in** |
| 10 in, tipped up 10° | 68–72 .. 8 in |
| **11 in, level** | **68–72 .. 10–12 in** |
| 11¾ in, level | 50–60 .. 12–14 in, with gaps |
| **12 in, level — the blind band** | 42–44 .. 22 in, at only 9–11 of 39 distances |
| **12¼ in, level** | **never** |
| 12½ in, level | 24 .. 4–8 in, at only 5–9 of 39 distances |
| 12¾ in, level | 14–16 .. 4 in |
| 13¼ in, level | 38–40 .. 4 in |
| 12 in, tipped up 10° | 70–74 .. 10–12 in |

So: **a lens about 10–11 in up, level** — the colour from about 70 in down to about 12 in — and never
11¾–12¾ in level; if the build can only put the lens there, tip it up about 10 degrees. The trade is far
range against near range: below the face the camera is the far-range cue and loses the colour inside
about 12 in; above it (13–14 in) the colour holds to about 4 in but is first seen only 20–40 in out.

## The piece finder: take ONE named piece, or none

A route names a piece by its TYPE (the AI model's class) and its EXPECTED FIELD POSITION within a
TOLERANCE — the route's numbers, typed by the students from the field layout:

```cpp
user.pins.setTarget({.classMask = 1U << static_cast<unsigned>(override2026::ModelClass::YellowPin),
                     .x = 66.2, .y = -6.0, .toleranceIn = 6.0, .want = vision::Posture::OnFloor});
```

and the finder answers one of five verdicts — never the nearest of several, never a guess:

```cpp
const vision::PieceFinder::Result answer = user.pins.answer(clock.now().value());
switch (answer.verdict) {
    case vision::PieceVerdict::Found:  // drive to answer.sighting.fix (L4's job)
        took = true;
        break;
    default:  // NONE, AMBIGUOUS, NOT IN VIEW, UNKNOWN: skip the step — never another piece
        skipped = true;
        break;
}
```

| Verdict | When |
|---|---|
| **FOUND** | exactly one candidate of the type and posture CERTAINLY inside the tolerance — its whole error disc inside — none on the border, the same one held for `holdSeconds` |
| **NONE** | nothing of the type and posture inside or on the border — it may be gone, or hidden ("wrong posture inside" when one of the type stands there in another posture) |
| **AMBIGUOUS** | two inside; one on the border; one of the type it cannot measure whose bearing could point inside; or its own error larger than the tolerance ("drive closer, or measure the mount tighter") |
| **NOT IN VIEW** | the expected place — its whole tolerance disc, at the floor AND at a Pin's height — is not certainly inside the picture |
| **UNKNOWN** | no target set (or one the target check refuses), no usable read, none recent, the latest read a fault, FOUND not held yet, a read of 24 or more, or the camera's AI model not confirmed BY NAME (a read whose objects were withheld is never "nothing there") |

Every verdict but FOUND is a reason for the route's step to be SKIPPED — never to grab another piece.

**How a piece stands.** A box cut by the picture's edge is UNSURE (one cut only by the bottom edge
carries a bound: its base is nearer than the floor at the picture's bottom row, so a Pin right in front
of the robot does not block a farther one). Wider than `lyingAspect`, it is LYING. Narrower than
`standingAspect`, it MAY stand — and then:

- **ON THE FLOOR** when some camera of the tolerance box puts its base (the box's bottom row on the floor)
  and its height (the box's size) at one range, AND no camera of the box could see it raised
  `raisedMinIn` or more. Agreeing with the floor is not enough on its own: far out, or from a high lens,
  a Pin on a Goal agrees with the floor through a camera at the far edge of the tolerance box — on the
  host plant 73 of 2,720 such sightings from a 10-in lens and 639 of 2,772 from an 18-in one did, each
  with a field position that missed the Pin. `raisedMinIn` is a fact of the FIELD, not of the bench —
  the lowest a raised piece's base sits (a Goal's top, a stack), read from the game manual's field
  drawings; a smaller number refuses more. (The library's tests use 5.0 against their own Goal, 5.8 in
  tall — a stand-in, not the manual's figure.) AND the box must be a STANDING Pin's, not a Pin lying
  toward the camera: its width must match a Pin's diameter at the range it agreed on, and no camera of
  the box could see it lying on the floor. A Pin lying nearly end-on has a box as tall and narrow as a
  standing one's; on the host plant, before these two checks, such Pins read "on the floor" with a field
  position far from them (from an 18-in lens, 5,876 of 169,542 sightings).
- **RAISED** (on a Goal, in a Cup, on a stack) when no camera can put it on the floor.
- **UNSURE** otherwise — the stated tolerances cannot place it on the floor or above it, or it could be
  lying toward the camera (close in, from a high lens, the two look alike). A route that wants a floor
  piece never takes it.

A Pin's box takes in its top face when the lens is above its top, so its size reads a little NEARER than
its near rim — by at most 2r(a − H)/H for a Pin of radius r and height H under a lens a high; the finder
allows for exactly that. On the host plant, Pins standing on the floor across the whole picture, through
true cameras at the edges of the tolerance box, their boxes' edges jittered by 1 px: never RAISED, and
every field position held the Pin. Two limits are open. A box edge must be within 2 px of the true edge
as the camera REPORTS it — rounding and any bias included (HA-202): with the host plant's edges jittered
by 2 px and then rounded, a box edge can sit about 3 px off, and standing floor Pins' field positions
then missed them by up to 1.8 in. And a Pin about 30 degrees off the camera's axis and close in, seen by a
camera measured far tighter than ±1 degree, with its box's bottom edge a pixel low, was missed by 0.02 in
past its stated radius (the box's centre column is the Pin's top's, not its middle's). That arithmetic is
not fixed yet — nothing uses the finder before drive-to-piece — so until it is, treat a field position near
the picture's side as a little looser than its radius.

**How close is close enough.** The field position's error is mostly the floor range's: with a mount
measured to ±1 degree and a robot pose good to ±1 in, the team lead's "two or three yellow Pins a foot
apart" scene — the route naming the middle one within 6 in, half the spacing — is AMBIGUOUS from 4 ft and
FOUND (the middle one, never a neighbour) with the lens about 22–23 in from it; closer than about 20 in
the place's near edge leaves the picture (NOT IN VIEW). With a 5-in tolerance it is never FOUND: AMBIGUOUS
until the place leaves the picture. A lying Pin's position is wide — its centre may be up to half its
length beyond its near end, whichever way it lies — so a lying piece needs a generous tolerance.
To take pieces from farther out: measure the mount tighter, or let the route's tolerance be wider where
the pieces are far apart.

## What none of this does

It moves nothing, flips nothing, drives nowhere. Pushing a Toggle (with the colour confirmed at the push
point), driving to a piece, and choosing our colour are later chunks that build on it. It does not read
AprilTags. And until a real camera has been read — the bench's camera station measures the mount, the
model's names, the colours under event light, the picture's age and a read's cost — every number on this
page is the host plant's.

---

*Next: back to [the guide's index](README.md). For exact signatures: [the generated API
reference](../api/README.md) — `camera_feed`, `toggle_tracker` and `piece_finder`.*
