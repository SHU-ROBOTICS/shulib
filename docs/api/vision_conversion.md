<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/vision_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `vision_conversion.hpp`

vision_conversion.hpp — the ONE place raw AprilTag image corners become shulib's canonical robot-relative tag pose (§7: convert exactly once, at the edge).

This header declares **4** types (13 members) and **1** free function.

Extracted from [`include/shulib/hal/vision_conversion.hpp`](../../include/shulib/hal/vision_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct CameraIntrinsics`](#struct-cameraintrinsics)
  - [`fx`](#cameraintrinsics-fx)
  - [`fy`](#cameraintrinsics-fy)
  - [`cx`](#cameraintrinsics-cx)
  - [`cy`](#cameraintrinsics-cy)
- [`struct CameraMount`](#struct-cameramount)
  - [`x`](#cameramount-x)
  - [`y`](#cameramount-y)
  - [`yaw`](#cameramount-yaw)
- [`struct TagCorners`](#struct-tagcorners)
  - [`u`](#tagcorners-u)
  - [`v`](#tagcorners-v)
- [`struct TagPnpResult`](#struct-tagpnpresult)
  - [`valid`](#tagpnpresult-valid)
  - [`poseInRobot`](#tagpnpresult-poseinrobot)
  - [`range`](#tagpnpresult-range)
  - [`reprojectionError`](#tagpnpresult-reprojectionerror)
- [`tagCornersToRobotPose`](#tagcornerstorobotpose) — *free function*

<a id="struct-cameraintrinsics"></a>

## `struct CameraIntrinsics`

```cpp
struct CameraIntrinsics
```

Pinhole intrinsics, in pixels. Focal lengths must be non-zero; no distortion model — the adapter is expected to hand over UNDISTORTED corners (R2 owns that, and owns proving it).

*struct, declared at [`include/shulib/hal/vision_conversion.hpp:98`](../../include/shulib/hal/vision_conversion.hpp#L98).*

<a id="cameraintrinsics-fx"></a>

### `CameraIntrinsics::fx`

```cpp
double fx = 0.0
```

Horizontal focal length in PIXELS (the projection is u = fx * X/Z + cx). The 0.0 default is deliberately unusable: tagCornersToRobotPose rejects |fx| < 1e-9, so an intrinsics block nobody filled in fails closed rather than returning a plausible pose.

*field, declared at [`include/shulib/hal/vision_conversion.hpp:102`](../../include/shulib/hal/vision_conversion.hpp#L102).*

<a id="cameraintrinsics-fy"></a>

### `CameraIntrinsics::fy`

```cpp
double fy = 0.0
```

Vertical focal length in PIXELS (v = fy * Y/Z + cy). Rejected at 0 exactly like fx. Equal to fx only for square pixels, which is why the two are carried separately.

*field, declared at [`include/shulib/hal/vision_conversion.hpp:105`](../../include/shulib/hal/vision_conversion.hpp#L105).*

<a id="cameraintrinsics-cx"></a>

### `CameraIntrinsics::cx`

```cpp
double cx = 0.0
```

principal point, PIXELS right from the image origin

*field, declared at [`include/shulib/hal/vision_conversion.hpp:106`](../../include/shulib/hal/vision_conversion.hpp#L106).*

<a id="cameraintrinsics-cy"></a>

### `CameraIntrinsics::cy`

```cpp
double cy = 0.0
```

principal point, PIXELS DOWN from the image origin (+v is down)

*field, declared at [`include/shulib/hal/vision_conversion.hpp:107`](../../include/shulib/hal/vision_conversion.hpp#L107).*

<a id="struct-cameramount"></a>

## `struct CameraMount`

```cpp
struct CameraMount
```

Where the camera sits on the robot, in the canonical body frame (F1: +X forward, +Y left). `yaw` is the direction the OPTICAL AXIS points, CCW-positive from +X. The camera is assumed level (A4 register HA-70). One owner for this offset, exactly as gps_conversion.hpp insists for the GPS lever arm: applying it twice is inches of silent bias.

*struct, declared at [`include/shulib/hal/vision_conversion.hpp:114`](../../include/shulib/hal/vision_conversion.hpp#L114).*

<a id="cameramount-x"></a>

### `CameraMount::x`

```cpp
units::Length x{}
```

camera position FORWARD of the robot origin (body +X)

*field, declared at [`include/shulib/hal/vision_conversion.hpp:115`](../../include/shulib/hal/vision_conversion.hpp#L115).*

<a id="cameramount-y"></a>

### `CameraMount::y`

```cpp
units::Length y{}
```

camera position LEFT of the robot origin (body +Y is LEFT, F1)

*field, declared at [`include/shulib/hal/vision_conversion.hpp:116`](../../include/shulib/hal/vision_conversion.hpp#L116).*

<a id="cameramount-yaw"></a>

### `CameraMount::yaw`

```cpp
math::Angle yaw{}
```

Direction the OPTICAL AXIS points, CCW-positive from body +X; 0 means the camera looks straight forward. This one rotation is ALL that is modelled — the camera is assumed level, so there is no mount pitch or roll, and a pitched camera silently becomes a range error that nothing downstream can detect.

*field, declared at [`include/shulib/hal/vision_conversion.hpp:121`](../../include/shulib/hal/vision_conversion.hpp#L121).*

<a id="struct-tagcorners"></a>

## `struct TagCorners`

```cpp
struct TagCorners
```

Four image corners in pixels, in the order documented at the top of this file.

*struct, declared at [`include/shulib/hal/vision_conversion.hpp:125`](../../include/shulib/hal/vision_conversion.hpp#L125).*

<a id="tagcorners-u"></a>

### `TagCorners::u`

```cpp
double u[4] = {0.0, 0.0, 0.0, 0.0}
```

Pixel column (image RIGHT) of each corner. u[k] and v[k] are the SAME corner — these are parallel arrays, not two independent lists. The index order is the one documented at the top of this file, and the pixels must already be UNDISTORTED: a cyclic rotation of the order changes nothing, but a REVERSED winding is silently catastrophic.

*field, declared at [`include/shulib/hal/vision_conversion.hpp:130`](../../include/shulib/hal/vision_conversion.hpp#L130).*

<a id="tagcorners-v"></a>

### `TagCorners::v`

```cpp
double v[4] = {0.0, 0.0, 0.0, 0.0}
```

Pixel row of each corner, measured DOWN from the image origin (+v is down, the camera convention at the top of this file). Paired with u by index.

*field, declared at [`include/shulib/hal/vision_conversion.hpp:133`](../../include/shulib/hal/vision_conversion.hpp#L133).*

<a id="struct-tagpnpresult"></a>

## `struct TagPnpResult`

```cpp
struct TagPnpResult
```

The planar reduction, plus the numbers a caller needs to decide whether to believe it.

*struct, declared at [`include/shulib/hal/vision_conversion.hpp:137`](../../include/shulib/hal/vision_conversion.hpp#L137).*

<a id="tagpnpresult-valid"></a>

### `TagPnpResult::valid`

```cpp
bool valid = false
```

false => the geometry was degenerate; poseInRobot is unset

*field, declared at [`include/shulib/hal/vision_conversion.hpp:138`](../../include/shulib/hal/vision_conversion.hpp#L138).*

<a id="tagpnpresult-poseinrobot"></a>

### `TagPnpResult::poseInRobot`

```cpp
math::Pose2d poseInRobot{}
```

tag pose relative to the robot (canonical body frame)

*field, declared at [`include/shulib/hal/vision_conversion.hpp:139`](../../include/shulib/hal/vision_conversion.hpp#L139).*

<a id="tagpnpresult-range"></a>

### `TagPnpResult::range`

```cpp
units::Length range{}
```

HORIZONTAL distance from the CAMERA to the tag centre

*field, declared at [`include/shulib/hal/vision_conversion.hpp:140`](../../include/shulib/hal/vision_conversion.hpp#L140).*

<a id="tagpnpresult-reprojectionerror"></a>

### `TagPnpResult::reprojectionError`

```cpp
double reprojectionError = 0.0
```

RMS pixel error of the recovered pose (0 for exact input)

*field, declared at [`include/shulib/hal/vision_conversion.hpp:141`](../../include/shulib/hal/vision_conversion.hpp#L141).*

<a id="tagcornerstorobotpose"></a>

## `tagCornersToRobotPose`

```cpp
[[nodiscard]] inline TagPnpResult tagCornersToRobotPose(const TagCorners& corners, const CameraIntrinsics& intrinsics, units::Length tagSize, const CameraMount& mount) noexcept
```

Corners -> the tag's pose relative to the robot. THE function hal/vision.hpp:12 reserves.  Returns `{valid = false}` rather than throwing on: a non-finite input, a non-positive tag size or focal length, a degenerate corner set, a tag behind the camera, or a recovered normal with no horizontal component (a tag lying flat, which a ground-plane reduction cannot use). Never throwing matters because R2's adapter runs this on sensor data, and sensor data is exactly where the impossible input comes from.

*free function, declared at [`include/shulib/hal/vision_conversion.hpp:198`](../../include/shulib/hal/vision_conversion.hpp#L198).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 84 lines, click to expand</summary>

```text

 vision_conversion.hpp — the ONE place raw AprilTag image corners become shulib's canonical
 robot-relative tag pose (§7: convert exactly once, at the edge). Companion to
 gps_conversion.hpp and imu_conversion.hpp, and named to match them on purpose: everything in
 this file is a PURE FUNCTION over plain numbers, so it is host-testable with no sensor, no
 clock and no PROS.

 hal/vision.hpp scopes this precisely (lines 10-14): ITagSource yields each tag as a RELATIVE
 POSE in the robot BODY frame, "a planar reduction of the tag's 6-DOF pose suitable for
 ground-plane localization", and "the corners->pose PnP (needs camera intrinsics + tag size) is
 a pure, host-testable function built with the M3 AprilTagCorrector".

 ── WHY THIS IS A FREE FUNCTION AND NOT A METHOD ON THE CORRECTOR (chunk tension T3) ─────────
 The CALLER of this function is the R2 adapter, not the corrector. The seam already hands
 AprilTagCorrector a reduced `poseInRobot`; the corrector never sees a pixel. If the PnP lived
 inside the corrector, R2 would have to either duplicate it or reach into a localization class
 from a HAL adapter, and both are worse than the seam that already exists. So it lives here,
 beside the other two edge conversions, and the corrector does not include this header at all.

 ── FRAMES, STATED ONCE, EXPLICITLY ─────────────────────────────────────────────────────────
 CAMERA frame (the standard pinhole convention):
     +X_cam = image right, +Y_cam = image DOWN, +Z_cam = out along the optical axis.
     Projection: u = fx * X/Z + cx,  v = fy * Y/Z + cy.
 TAG frame (planar, origin at the tag's centre):
     +X_tag = the tag's own right AS SEEN BY A VIEWER FACING IT, +Y_tag = DOWN the tag face,
     +Z_tag = X_tag x Y_tag, which therefore points AWAY from that viewer (into the wall).
     The tag's OUTWARD normal — the direction it "faces" — is -Z_tag.
     A tag seen head-on has R == I, so a head-on tag's outward normal is -Z_cam: it looks
     back down the optical axis. That identity is the sign convention's anchor.
 ROBOT BODY frame (F1, locked): +X forward, +Y LEFT, heading CCW-positive.

 CORNER ORDER — the input contract, and the one thing R2 must get right:
     corners[0] = tag-frame (-s/2, -s/2)   (the viewer's TOP-LEFT)
     corners[1] = tag-frame (+s/2, -s/2)   (TOP-RIGHT)
     corners[2] = tag-frame (+s/2, +s/2)   (BOTTOM-RIGHT)
     corners[3] = tag-frame (-s/2, +s/2)   (BOTTOM-LEFT)
 i.e. clockwise in the image when the tag is upright and facing the camera. Which order the V5
 AI Vision sensor / a Pi detector actually reports is UNVERIFIED (A4 register HA-69) — mapping
 the detector's order onto this one is R2's job and R2's test.

 HOW MUCH THE ORDER ACTUALLY MATTERS — measured, not assumed (E3; pinned by
 test/vision_conversion_test.cpp):
   * A CYCLIC ROTATION of the order (0123 -> 1230, ...) changes NOTHING. It rotates the
     recovered tag frame about its own face normal, and the planar reduction discards exactly
     that degree of freedom. R2 does not have to find the detector's starting corner.
   * A REVERSAL of the order (0123 -> 0321) is CATASTROPHIC AND SILENT. It mirrors the tag
     plane, so the recovered face normal points 180 degrees the wrong way — while the
     reprojection error stays at ~1e-14, because a mirrored pose reprojects onto the same four
     pixels. Nothing in this function, and nothing downstream of it, can detect a reversed
     winding. R2 must pin the detector's WINDING against a physical tag; a self-check cannot.
   * A SWAP of two ADJACENT corners produces a self-intersecting quad and returns
     {valid = false}, so that miswiring at least fails loudly.

 ── THE PLANAR REDUCTION, AND WHAT IT THROWS AWAY ───────────────────────────────────────────
 The 6-DOF solution is computed in full and then reduced to (x, y, heading) in the body frame:
   * the tag's HEIGHT above the camera (t_y) is discarded — correct for a ground-plane
     localizer, and the reason a tag mounted above the camera still yields the right x/y;
   * the tag's PITCH and ROLL are discarded — its outward normal is projected onto the
     horizontal plane and the heading is the direction of that projection.
 This assumes the camera is mounted LEVEL (no pitch/roll). A pitched camera turns a height
 difference into a range error, and nothing downstream can see that it happened
 (A4 register HA-70).

 ── THE ALGORITHM, AND ITS ONE STRUCTURAL ASSUMPTION ────────────────────────────────────────
 Four coplanar correspondences determine a homography exactly. With tag-plane points
 X = (x, y, 0):
     [u v 1]^T ~ K * (x*r1 + y*r2 + t) = K * [r1 r2 t] * [x y 1]^T   =>   H = lambda*K*[r1 r2 t]
 so the pipeline is: DLT for H (8x8 solve, partial pivoting) -> G = K^-1 * H -> recover the
 scale from |r1| = |r2| = 1 -> orthonormalize (r1, r2) -> r3 = r1 x r2 -> reduce.

 H is normalized with h33 == 1, which fixes lambda = 1/t_z. That is legitimate here and not in
 general: h33 is the projective scale of the tag's CENTRE, which is zero only for a tag at
 infinity or exactly in the camera's plane — neither is a visible tag. The benefit is that the
 depth sign comes out right by construction (t_z = s > 0), so there is no "which of the two
 solutions is in front of the camera" branch to get wrong.

 PLANAR PnP HAS A KNOWN NEAR-AMBIGUITY: as a tag shrinks toward the image centre or turns
 nearly face-on, two poses with mirrored out-of-plane rotation project almost identically, so
 the recovered HEADING degrades much faster with range than the recovered POSITION does. This
 function does not pretend otherwise — it reports the reprojection error it achieved, and the
 corrector's trusted-range band (A4 register HA-73) is what keeps the caller out of the region
 where the ambiguity bites. A second (mirror) solution and a proper R_heading belong to E4.

 Nothing here allocates, throws, or reads a clock.
```

</details>
