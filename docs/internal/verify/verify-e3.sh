#!/usr/bin/env bash
# E3 mutation harness — break it, rebuild, run, OBSERVE the red, restore.
#
# A mutation that stays GREEN is a hole in the suite and the most valuable thing this
# script can find. The runner is GATED ON BUILD SUCCESS: a mutation that fails to compile
# is reported as BUILD-FAIL, never as "red", because a compile error proves nothing about
# the tests.
#
# Usage: docs/internal/verify/verify-e3.sh          (from the repo root)
set -uo pipefail
cd "$(dirname "$0")/../../.." || exit 1
BUILD=build/test
BIN=$BUILD/shulib_tests

pass=0; holes=0; buildfail=0; skipped=0

# NEVER use `git checkout` to undo a mutation: these files carry UNCOMMITTED work, and a
# checkout discards the chunk, not the mutation. Restore is always from the byte copy taken
# immediately before the edit. (E2's harness cost an hour by getting that wrong once.)
#
# Do not pipe this script into `head`: SIGPIPE kills it mid-mutation and leaves a header
# broken on disk. It prints a bounded amount; redirect to a file if you need to page it.
trap 'echo "INTERRUPTED — restoring $CURRENT"; [ -n "$CURRENT" ] && cp /tmp/e3-mut-backup "$CURRENT"; exit 130' INT TERM PIPE
CURRENT=""

# mutate NAME FILE FROM TO [FROM2 TO2]
# The optional second pair exists because a few properties are guarded in two places
# (a per-proposal clamp AND a sum clamp, say), and removing only one leaves the other
# enforcing the invariant — which would report GREEN for a mutation that never actually
# broke anything.
mutate() {
  local name="$1" file="$2" from="$3" to="$4" from2="${5:-}" to2="${6:-}"
  printf '\n=== MUTATION: %s\n    %s\n' "$name" "$file"
  cp "$file" /tmp/e3-mut-backup
  CURRENT="$file"
  if ! grep -qF -- "$from" "$file"; then
    echo "    *** SKIP — PATTERN NOT FOUND. This mutation did NOT run. Either the code"
    echo "        moved (fix this script) or a previous mutation was left applied. ***"
    skipped=$((skipped+1))
    CURRENT=""
    return
  fi
  if [ -n "$from2" ] && ! grep -qF -- "$from2" "$file"; then
    echo "    *** SKIP — SECOND PATTERN NOT FOUND. This mutation did NOT run. ***"
    skipped=$((skipped+1))
    CURRENT=""
    return
  fi
  python3 - "$file" "$from" "$to" "$from2" "$to2" <<'PY'
import sys
p, a, b, a2, b2 = sys.argv[1:6]
s = open(p).read()
s = s.replace(a, b, 1)
if a2:
    s = s.replace(a2, b2, 1)
open(p, 'w').write(s)
PY
  # THE EDIT MUST HAVE CHANGED THE FILE. `grep -F` with a MULTI-LINE pattern matches if ANY
  # single line matches, so the pre-flight check above can pass while python's exact
  # whole-string replace finds nothing — and then the build succeeds, the suite passes, and the
  # mutation is reported GREEN having never run. That happened twice during E3, and it is exactly
  # the class of harness fault E2's postmortem was written about: the report looks fine and the
  # only tell is the count. A byte-compare makes it impossible.
  if cmp -s "$file" /tmp/e3-mut-backup; then
    echo "    *** SKIP — THE EDIT CHANGED NOTHING. This mutation did NOT run (a multi-line"
    echo "        pattern that did not match exactly). Fix the pattern in this script. ***"
    skipped=$((skipped+1))
    CURRENT=""
    return
  fi
  if ! cmake --build "$BUILD" -j"$(nproc)" >/tmp/e3-mut-build.log 2>&1; then
    echo "    BUILD-FAIL — mutation did not compile; proves nothing"
    buildfail=$((buildfail+1))
    cp /tmp/e3-mut-backup "$file"; CURRENT=""
    cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
    return
  fi
  local out
  out=$("$BIN" 2>&1 | tail -3)
  if echo "$out" | grep -q "FAILURE"; then
    echo "    RED (good) — $(echo "$out" | grep 'test cases' | tr -s ' ')"
    pass=$((pass+1))
  else
    echo "    *** GREEN — THIS IS A HOLE IN THE SUITE ***"
    holes=$((holes+1))
  fi
  cp /tmp/e3-mut-backup "$file"
  CURRENT=""
  cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
}

PNP=include/shulib/hal/vision_conversion.hpp
MAP=include/shulib/localization/tag_map.hpp
COR=include/shulib/localization/apriltag_corrector.hpp
FUS=include/shulib/localization/complementary_fusion.hpp
LOC=include/shulib/localization/localizer.hpp
SCH=include/shulib/motion/motion_scheduler.hpp

# ══ the five the brief REQUIRES ═══════════════════════════════════════════════════════

mutate "TAG-MAP AXES SWAPPED (brief #1)" "$MAP" \
  "        return math::Pose2d{units::Length{tagField.x().value() - (rx * c - ry * s)},
                            units::Length{tagField.y().value() - (rx * s + ry * c)}," \
  "        return math::Pose2d{units::Length{tagField.x().value() - (rx * s + ry * c)},
                            units::Length{tagField.y().value() - (rx * c - ry * s)},"

mutate "RELATIVE-POSE INVERSION SIGN FLIPPED (brief #2)" "$MAP" \
  "        return math::Pose2d{units::Length{tagField.x().value() - (rx * c - ry * s)},
                            units::Length{tagField.y().value() - (rx * s + ry * c)}," \
  "        return math::Pose2d{units::Length{tagField.x().value() + (rx * c - ry * s)},
                            units::Length{tagField.y().value() + (rx * s + ry * c)},"

mutate "A TAG FIX SNAPS INSTEAD OF NUDGING (brief #3)" "$FUS" \
  "            double nudgeH = config_.maxHeadingGain * conf * innoH;" \
  "            double nudgeH = innoH;" \
  "        const double maxHeadingNudge = config_.maxHeadingNudgeRate.value() * dt.value();" \
  "        const double maxHeadingNudge = 1e9;"

mutate "CONVERGENCE ACCUMULATION BROKEN — the bias is assigned, not accumulated (brief #4)" "$LOC" \
  "            const double next = headingBias_ + nudgeH;" \
  "            const double next = nudgeH;"

mutate "NO TAGS RETURNS A LOW-CONFIDENCE PULL (brief #5)" "$COR" \
  "        if (frameCount_ == 0) {
            ++noTagTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }" \
  "        if (frameCount_ == 0) {
            ++noTagTicks_;
            CorrectionProposal weak;
            weak.valid = true;
            weak.confidence = 0.01;
            weak.positionStdDev = units::Length{99.0};
            return weak;
        }"

# ══ PnP — the geometry (T3) ═══════════════════════════════════════════════════════════

mutate "PnP: camera->body axes swapped (optical axis read as image-right)" "$PNP" \
  "    const double bodyX = mount.x.value() + t[2] * cpsi + t[0] * spsi;" \
  "    const double bodyX = mount.x.value() + t[0] * cpsi + t[2] * spsi;"

mutate "PnP: the tag's outward normal is not negated (faces into the wall)" "$PNP" \
  "    const double nCamX = -r3[0];
    const double nCamZ = -r3[2];" \
  "    const double nCamX = r3[0];
    const double nCamZ = r3[2];"

mutate "PnP: the camera mount offset is dropped (the AprilTag lever arm)" "$PNP" \
  "    const double bodyX = mount.x.value() + t[2] * cpsi + t[0] * spsi;" \
  "    const double bodyX = t[2] * cpsi + t[0] * spsi;"

mutate "PnP: the camera mounting yaw is ignored" "$PNP" \
  "    const double psi = mount.yaw.radians();" \
  "    const double psi = 0.0;"

mutate "PnP: the physical tag size is ignored (a fixed 1-inch tag)" "$PNP" \
  "    const double half = 0.5 * size;" \
  "    const double half = 0.5;"

mutate "PnP: the rotation columns are not orthonormalized" "$PNP" \
  "        r1[k] = kInvSqrt2 * (a + b);" \
  "        r1[k] = a + b;"

mutate "PnP: the scale is taken from one column only (ignores |r2|)" "$PNP" \
  "    const double scale = 2.0 / (n1 + n2);" \
  "    const double scale = 1.0 / n1;"

mutate "PnP: a degenerate corner set returns a pose instead of {valid=false}" "$PNP" \
  "        if (!(best > 1e-12)) {
            return false;  // singular: four collinear corners, or a degenerate view" \
  "        if (false) {
            return false;  // singular: four collinear corners, or a degenerate view"

# ══ the tag map (T2) ══════════════════════════════════════════════════════════════════

mutate "TagMap: the position term is rotated by the TAG's heading, not the ROBOT's" "$MAP" \
  "        const double c = std::cos(robotHeading.radians());
        const double s = std::sin(robotHeading.radians());" \
  "        const double c = std::cos(tagField.heading().radians());
        const double s = std::sin(tagField.heading().radians());"

mutate "TagMap: the heading inversion adds instead of subtracting" "$MAP" \
  "            math::Angle::radians(tagField.heading().radians() - tagInRobot.heading().radians());" \
  "            math::Angle::radians(tagField.heading().radians() + tagInRobot.heading().radians());"

mutate "TagMap: provenance is no longer required (a guess can pass as a spec)" "$MAP" \
  "        SHULIB_PRECONDITION(placement.provenance != TagProvenance::Unspecified," \
  "        SHULIB_PRECONDITION(true," \
  "        SHULIB_PRECONDITION(placement.source != nullptr && placement.source[0] != '\\0'," \
  "        SHULIB_PRECONDITION(true,"

mutate "TagMap: a duplicate tag id is silently accepted" "$MAP" \
  "        SHULIB_PRECONDITION(find(placement.id) == nullptr, \"TagMap: duplicate tag id\");" \
  "        SHULIB_PRECONDITION(true, \"TagMap: duplicate tag id\");"

mutate "TagMap: anyInvented() always says no" "$MAP" \
  "            if (entries_[k].provenance == TagProvenance::Invented) {" \
  "            if (false) {"

# ══ the corrector ═════════════════════════════════════════════════════════════════════

mutate "corrector: providesHeading is never set (yaw correction silently dead)" "$COR" \
  "        p.providesHeading = true;" \
  "        p.providesHeading = false;"

mutate "corrector: HEADING latency compensation dropped" "$COR" \
  "        const double zh = absolute.heading().radians() + (unwrappedHeading_ - baseH);" \
  "        const double zh = absolute.heading().radians();"

# KNOWN GREEN, recorded rather than papered over (E3-COMPLETED.md §THE HOLES). The two versions
# differ by ~0.1% in convergence rate (11.8852 vs 11.8987 degrees at 9 s on a 12-degree
# correction) and NEITHER overshoots, so no test here separates them — and any test tight enough
# to would be pinning an invented constant, which this chunk refuses to do.
mutate "corrector: rotation history taken from the PREDICTED heading, not the IMU
    (KNOWN GREEN — see E3-COMPLETED.md; kept as an argument from algebra, not a measurement)" "$COR" \
  "        const math::Angle imuHeading = imu_.heading();" \
  "        const math::Angle imuHeading = predicted.heading();"

mutate "corrector: POSITION latency compensation dropped" "$COR" \
  "        const double zx = zx0 + (px - baseX);
        const double zy = zy0 + (py - baseY);" \
  "        const double zx = zx0;
        const double zy = zy0;"

mutate "corrector: the trusted range band is removed" "$COR" \
  "            if (range < config_.minRange.value() || range > config_.maxRange.value()) {" \
  "            if (false) {"

mutate "corrector: the detector-confidence floor is removed" "$COR" \
  "            if (obs.confidence < config_.minConfidence) {" \
  "            if (false) {"

mutate "corrector: the yaw-rate rejection is removed" "$COR" \
  "        if (!std::isfinite(yawRate) || std::abs(yawRate) > config_.maxYawRate.value()) {" \
  "        if (!std::isfinite(yawRate)) {"

mutate "corrector: the freshness guard is removed (one frame folded every tick)" "$COR" \
  "        if (frameSeq_ == foldedSeq_) {" \
  "        if (false) {"

mutate "corrector: the observation-age guard is removed (a dead vision task looks healthy)" "$COR" \
  "        if (!std::isfinite(frameTime_) || now - frameTime_ > config_.maxObservationAge.value()) {" \
  "        if (!std::isfinite(frameTime_)) {"

mutate "corrector: an unmapped tag id is reported as an out-of-range tag (config error hidden)" "$COR" \
  "            if (map_.find(obs.id) == nullptr) {
                sawUnmapped = true;" \
  "            if (map_.find(obs.id) == nullptr) {
                sawOutOfRange = true;"

mutate "corrector: the anti-lockout widening is removed" "$COR" \
  "        const double sigmaDr = std::hypot(config_.postFixStdDev.value(),
                                          config_.driftStdDevPerInch * travelSinceFix_);" \
  "        const double sigmaDr = config_.postFixStdDev.value();"

mutate "corrector: sigma ignores the detector confidence" "$COR" \
  "            const double sigma = (config_.baseStdDev.value() + config_.stdDevPerInch * range) /
                                 std::max(obs.confidence, kMinConfidenceFloor);" \
  "            const double sigma = config_.baseStdDev.value() + config_.stdDevPerInch * range;"

mutate "corrector: the position innovation gate is inverted" "$COR" \
  "            residual > config_.gateSigma * sigmaEff) {" \
  "            residual < config_.gateSigma * sigmaEff) {"

mutate "corrector: multi-tag selection keeps the WORST tag" "$COR" \
  "            if (bestIndex == kMaxTagsPerFrame || sigma < bestSigma) {" \
  "            if (bestIndex == kMaxTagsPerFrame || sigma > bestSigma) {"

mutate "T4 VIOLATION: propose() calls the allocating tags() seam on the control path" "$COR" \
  "        if (!haveFrame_) {
            ++noFrameTicks_;" \
  "        if (!tags_.tags().empty() && !haveFrame_) {
            ++noFrameTicks_;"

# ══ the fusion policy ═════════════════════════════════════════════════════════════════

mutate "fusion: the heading gate is removed (a mirrored tag rotates the robot)" "$FUS" \
  "        const double headingGate = config_.headingGate.value();" \
  "        const double headingGate = 1e9;"

mutate "fusion: the per-tick heading budget is removed (never-snap bound gone)" "$FUS" \
  "        const double maxHeadingNudge = config_.maxHeadingNudgeRate.value() * dt.value();" \
  "        const double maxHeadingNudge = 1e9;"

mutate "fusion: providesHeading is ignored (the GPS's pass-through heading is folded)" "$FUS" \
  "            if (!p.providesHeading) {
                continue;
            }" \
  "            if (false) {
                continue;
            }"

mutate "fusion: the heading residual never reaches the audit" "$FUS" \
  "            audit.residualHeading = units::AngleDim{auditInnoHeading};" \
  "            audit.residualHeading = units::AngleDim{auditInnoHeading * 0.0};"

# ══ the Localizer ═════════════════════════════════════════════════════════════════════

mutate "Localizer: the learned bias is never published (correction computed, then discarded)" "$LOC" \
  "        return raw + math::Angle::radians(headingBias_);" \
  "        return raw;"

mutate "Localizer: the odometry delta is not re-expressed under the corrected heading" "$LOC" \
  "        if (headingBias_ != 0.0) {" \
  "        if (false) {"

mutate "Localizer: the heading nudge never reaches AppliedCorrection::dtheta" "$LOC" \
  "                appliedHeadingNudge_ = nudgeH;" \
  "                appliedHeadingNudge_ = 0.0;"

mutate "Localizer: an exceptional verdict no longer outranks routine staleness (E3's rule)" "$LOC" \
  "                    if (selfAuditSource == nullptr || (selfAuditRoutine && !routine)) {" \
  "                    if (selfAuditSource == nullptr || (selfAuditRoutine && routine)) {"

mutate "Localizer: the corrector-verdict substitution is removed entirely (E2's guard)" "$LOC" \
  "        if (tickAudit.reason == diag::GateReason::None && selfAuditSource != nullptr) {" \
  "        if (false && selfAuditSource != nullptr) {"

mutate "Localizer: substitution OVERWRITES a real fusion verdict (E2's hole, now live)" "$LOC" \
  "        if (tickAudit.reason == diag::GateReason::None && selfAuditSource != nullptr) {" \
  "        if (selfAuditSource != nullptr) {"

# ══ the record producer ═══════════════════════════════════════════════════════════════

mutate "record: correctionDTheta is never stamped (never-snap unauditable from telemetry)" "$SCH" \
  "        stamped.correctionDTheta = audit_.dtheta;" \
  "        stamped.correctionDTheta = units::AngleDim{};"

printf '\n============================================================\n'
printf 'E3 mutations: %d RED (good), %d GREEN (HOLES), %d build-fail, %d SKIPPED\n' \
  "$pass" "$holes" "$buildfail" "$skipped"
printf '============================================================\n'
# A skip is a failure of the harness, not a neutral outcome — it means a mutation the
# brief requires was never actually executed, which is exactly what "a mutation you
# reasoned about but did not run does not count" forbids.
[ "$holes" -eq 0 ] && [ "$skipped" -eq 0 ]
