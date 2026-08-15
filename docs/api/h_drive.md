<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/h_drive.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `h_drive.hpp`

hDrive() — the H-drive (tank base + one transverse strafe wheel), as a MatrixKinematics preset (chunk C3; the hybrid backend §13 #15: a holonomic LINEAR drive is a coefficient table, exactly as xDrive() is).

This header declares **1** type (4 members) and **1** free function.

Extracted from [`include/shulib/kinematics/h_drive.hpp`](../../include/shulib/kinematics/h_drive.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct HDriveConfig`](#struct-hdriveconfig)
  - [`trackWidth`](#hdriveconfig-trackwidth)
  - [`strafeWheelOffset`](#hdriveconfig-strafewheeloffset)
  - [`strafeSpeedRatio`](#hdriveconfig-strafespeedratio)
  - [`strafeTractionDerate`](#hdriveconfig-strafetractionderate)
- [`hDrive`](#hdrive) — *free function*

<a id="struct-hdriveconfig"></a>

## `struct HDriveConfig`

```cpp
struct HDriveConfig
```

The H-drive's geometry + capability inputs. Lengths are REQUIRED (no invented-by-default geometry); the two ratios default to the same-hardware strafe wheel with the registered provisional derate (header note).

*struct, declared at [`include/shulib/kinematics/h_drive.hpp:83`](../../include/shulib/kinematics/h_drive.hpp#L83).*

<a id="hdriveconfig-trackwidth"></a>

### `HDriveConfig::trackWidth`

```cpp
units::Length trackWidth{}
```

Lateral distance between the left and right drive-wheel contact lines (> 0).

*field, declared at [`include/shulib/kinematics/h_drive.hpp:85`](../../include/shulib/kinematics/h_drive.hpp#L85).*

<a id="hdriveconfig-strafewheeloffset"></a>

### `HDriveConfig::strafeWheelOffset`

```cpp
units::Length strafeWheelOffset{}
```

The strafe wheel's +FORWARD body coordinate, SIGNED (negative = mounted aft of centre). 0 = exactly on centre (orthogonal table — the historical exact path; still one preset). Finite.

*field, declared at [`include/shulib/kinematics/h_drive.hpp:89`](../../include/shulib/kinematics/h_drive.hpp#L89).*

<a id="hdriveconfig-strafespeedratio"></a>

### `HDriveConfig::strafeSpeedRatio`

```cpp
double strafeSpeedRatio = 1.0
```

Strafe-wheel top surface speed / drive-wheel top surface speed — the derivable kinematic ceiling (gearing × wheel radius). > 0, finite.

*field, declared at [`include/shulib/kinematics/h_drive.hpp:92`](../../include/shulib/kinematics/h_drive.hpp#L92).*

<a id="hdriveconfig-strafetractionderate"></a>

### `HDriveConfig::strafeTractionDerate`

```cpp
double strafeTractionDerate = 0.35
```

Sustained fraction of that ceiling the strafe wheel holds under real load on foam. In [0, 1]. PROVISIONAL (A4: HA-54) — sysid-measured at R5.

*field, declared at [`include/shulib/kinematics/h_drive.hpp:95`](../../include/shulib/kinematics/h_drive.hpp#L95).*

<a id="hdrive"></a>

## `hDrive`

```cpp
[[nodiscard]] inline MatrixKinematics hDrive(const HDriveConfig& cfg)
```

Build the H-drive preset. Preconditions red-on-failure (see HDriveConfig). Wheel order: 0 = left, 1 = right, 2 = strafe (header).

*free function, declared at [`include/shulib/kinematics/h_drive.hpp:100`](../../include/shulib/kinematics/h_drive.hpp#L100).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 69 lines, click to expand</summary>

```text

 hDrive() — the H-drive (tank base + one transverse strafe wheel), as a
 MatrixKinematics preset (chunk C3; the hybrid backend §13 #15: a holonomic
 LINEAR drive is a coefficient table, exactly as xDrive() is). This is the
 15″ H-bot's drivetrain — the second robot that must run the SAME motion code
 as the 24″ X-bot, unmodified (the M2 Definition of Done).

 Geometry — body frame per F1 (frame.hpp): +X = forward, +Y = left, ω CCW+.
 Canonical wheel order (3 kinematic wheels; wheel_speeds.hpp's "H=3"):
   wheel 0: LEFT drive line   — rolls along +X at body y = +trackWidth/2
   wheel 1: RIGHT drive line  — rolls along +X at body y = −trackWidth/2
   wheel 2: STRAFE wheel      — rolls along +Y at body x = strafeWheelOffset

 Rows from rigid-body kinematics (v_point = v_body + ω × r, projected on the
 wheel's powered-roll direction — the same derivation drive_plant.hpp uses for
 tracking wheels, independently):
   left   = vx − ω·(trackWidth/2)      → [1, 0, −trackWidth/2]
   right  = vx + ω·(trackWidth/2)      → [1, 0, +trackWidth/2]
   strafe = vy + ω·strafeWheelOffset   → [0, 1,  strafeWheelOffset]
 The left/right rows are TankKinematics' closed form verbatim (pinned by test);
 strafeWheelOffset is the wheel's +FORWARD body coordinate, SIGNED — the same
 convention as a Lateral tracking wheel (drive_plant.hpp TrackingWheelSpec).

 ── Why this drive is the reason the pseudo-inverse exists ──────────────────────────
 The v and turn columns have inner product v·t = strafeWheelOffset: an
 OFF-CENTRE strafe wheel (the physical norm — the centre of a 15″ chassis is
 occupied) makes the table non-orthogonal, which the pre-C3 forward() rejected.
 With the strafe wheel EXACTLY on centre the table is orthogonal and
 MatrixKinematics runs its historical exact path — both cases are one preset,
 zero special cases downstream. The 3×3 table is square full-rank (relDet =
 2w²/(2w²+a²) ≈ 0.94 for the H-bot stand-in — nowhere near the conditioning
 floor), so forward() is an EXACT inverse, not a least-squares approximation.

 ── Strafe authority: derived ceiling × registered derate ───────────────────────────
 F5 semantics (kinematics.hpp, C1 D11 — confirmed here at C3): strafeAuthority()
 is the sustainable |body vy| as a FRACTION OF THE DRIVE'S LINEAR SPEED BUDGET;
 the motion layer clamps |vy| ≤ authority·maxLinearSpeed. For the H-drive:

   authority = strafeSpeedRatio × strafeTractionDerate

   * strafeSpeedRatio — the DERIVABLE kinematic ceiling: the strafe wheel's top
     surface speed over the drive wheels' (motor free speed × gearing × wheel
     radius, each side). 1.0 when the strafe wheel shares the drive's cartridge
     and wheel size (the H-bot stand-in).
   * strafeTractionDerate — the part NO geometry can supply: one lightly-loaded
     omni pushing the whole robot's mass across foam sustains only a fraction
     of its free surface speed (traction + load, sysid-measured at R5).
     PROVISIONAL (A4: HA-54); default 0.35 = the master plan's locked
     "HDrive ≈ 0.35 sysid-measured default, not a hardcoded constant".

 Sensible limits, pinned by test: derate 0 ⇒ authority 0 (a tank with a dead
 strafe wheel — StrafeTo times out honestly, C1 D12); ratio·derate = 1 ⇒ a
 fully-symmetric drive (no extra constraint, like the X-drive); the wheel's
 POSITION does not enter authority (placement changes the ω coupling, not the
 sustainable lateral speed).

 ── Known simplifications, stated (not hidden) ──────────────────────────────────────
   * Combined strafe+rotation makes the strafe wheel serve vy + ω·a; the scalar
     authority deliberately reflects SUSTAINED PURE-STRAFE capability (the F5
     scalar cannot carry a coupled budget). Over-budget combinations are handled
     downstream by desaturate() direction-preservingly, as everywhere else.
   * v1 assumes the shared MotionConfig::maxWheelSpeed budget fits all three
     wheels (true at strafeSpeedRatio ≈ 1). A strafe wheel geared much slower
     than the drive would want a per-wheel budget — an R5/Frontier refinement,
     flagged for F6 rather than silently absorbed.

 That the BUILT 15″ H-bot matches any configured geometry is A4-registered
 (HA-55, the C3 stand-ins; HA-17's preset-geometry claim applies here too);
 R3 measures, R5 sysids the derate.
```

</details>
