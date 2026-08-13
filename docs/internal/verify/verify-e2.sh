#!/usr/bin/env bash
# E2 mutation harness — break it, rebuild, run, OBSERVE the red, restore.
#
# A mutation that stays GREEN is a hole in the suite and the most valuable thing this
# script can find. The runner is GATED ON BUILD SUCCESS: a mutation that fails to compile
# is reported as BUILD-FAIL, never as "red", because a compile error proves nothing about
# the tests.
#
# Usage: docs/internal/verify/verify-e2.sh          (from the repo root)
set -uo pipefail
cd "$(dirname "$0")/../../.." || exit 1
ROOT=$(pwd)
BUILD=build/test
BIN=$BUILD/shulib_tests

pass=0; holes=0; buildfail=0; skipped=0

# NEVER use `git checkout` to undo a mutation: these files carry UNCOMMITTED work, and a
# checkout discards the chunk, not the mutation. Restore is always from the byte copy taken
# immediately before the edit. (This script cost an hour at E2 by getting that wrong once.)
#
# Do not pipe this script into `head`: SIGPIPE kills it mid-mutation and leaves a header
# broken on disk. It prints a bounded amount; redirect to a file if you need to page it.
trap 'echo "INTERRUPTED — restoring $CURRENT"; [ -n "$CURRENT" ] && cp /tmp/e2-mut-backup "$CURRENT"; exit 130' INT TERM PIPE
CURRENT=""

mutate() {
  local name="$1" file="$2" from="$3" to="$4" scope="${5:-}"
  printf '\n=== MUTATION: %s\n    %s\n' "$name" "$file"
  cp "$file" /tmp/e2-mut-backup
  CURRENT="$file"
  if ! grep -qF -- "$from" "$file"; then
    echo "    *** SKIP — PATTERN NOT FOUND. This mutation did NOT run. Either the code"
    echo "        moved (fix this script) or a previous mutation was left applied. ***"
    skipped=$((skipped+1))
    return
  fi
  python3 - "$file" "$from" "$to" <<'PY'
import sys
p, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(p).read()
open(p, 'w').write(s.replace(a, b, 1))
PY
  if ! cmake --build "$BUILD" -j"$(nproc)" >/tmp/e2-mut-build.log 2>&1; then
    echo "    BUILD-FAIL — mutation did not compile; proves nothing"
    buildfail=$((buildfail+1))
    cp /tmp/e2-mut-backup "$file"; CURRENT=""
    cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
    return
  fi
  local out
  if [ -n "$scope" ]; then out=$("$BIN" -tc="$scope" 2>&1 | tail -3)
  else out=$("$BIN" 2>&1 | tail -3); fi
  if echo "$out" | grep -q "FAILURE"; then
    echo "    RED (good) — $(echo "$out" | grep 'test cases' | tr -s ' ')"
    pass=$((pass+1))
  else
    echo "    *** GREEN — THIS IS A HOLE IN THE SUITE ***"
    holes=$((holes+1))
  fi
  cp /tmp/e2-mut-backup "$file"
  CURRENT=""
  cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
}

GPS=include/shulib/hal/gps_conversion.hpp
COR=include/shulib/localization/gps_corrector.hpp
LOC=include/shulib/localization/localizer.hpp

# ── the five the brief REQUIRES ────────────────────────────────────────────────────
mutate "lever-arm sign flipped (offY)" "$GPS" \
  "const double offY = s * lf + c * ll;" \
  "const double offY = s * lf - c * ll;"

mutate "lever-arm cross-term sign flipped (offX)" "$GPS" \
  "const double offX = c * lf - s * ll;" \
  "const double offX = c * lf + s * ll;"

mutate "frame axes swapped (East/North)" "$GPS" \
  "const double cxMeters = xMeters * s + yMeters * c;" \
  "const double cxMeters = xMeters * c + yMeters * s;"

mutate "metres->inches conversion DROPPED on rmsError" "$GPS" \
  "return units::Length{errorMeters * kMetersToInches};" \
  "return units::Length{errorMeters};"

mutate "metres->inches conversion DROPPED on position" "$GPS" \
  "return math::Pose2d{units::Length{cxMeters * kMetersToInches}," \
  "return math::Pose2d{units::Length{cxMeters},"

mutate "gate INVERTED (reject what it should accept)" "$COR" \
  "residual > config_.gateSigma * sigmaEff) {" \
  "residual < config_.gateSigma * sigmaEff) {"

mutate "off-strip returns a LOW-CONFIDENCE FIX instead of {valid=false}" "$COR" \
  "        if (!gps_.hasFix()) {
            ++noFixTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }" \
  "        if (!gps_.hasFix()) {
            ++noFixTicks_;
            CorrectionProposal weak;
            weak.valid = true;
            weak.fieldPose = gps_.pose();
            weak.confidence = 0.01;
            weak.positionStdDev = units::Length{99.0};
            return weak;
        }"

# ── the ones E2's own logic lives or dies on ───────────────────────────────────────
mutate "latency compensation DROPPED" "$COR" \
  "        const double zxc = zx + (px - baseX);
        const double zyc = zy + (py - baseY);" \
  "        const double zxc = zx;
        const double zyc = zy;"

mutate "latency compensation applied BACKWARDS" "$COR" \
  "        const double zxc = zx + (px - baseX);
        const double zyc = zy + (py - baseY);" \
  "        const double zxc = zx - (px - baseX);
        const double zyc = zy - (py - baseY);"

mutate "staleness guard REMOVED (one fix folded every tick)" "$COR" \
  "        if (haveSample_ && zx == sampleX_ && zy == sampleY_ && rms == sampleRms_) {" \
  "        if (false) {"

mutate "anti-lockout widening REMOVED (sigma_dr constant)" "$COR" \
  "            std::hypot(config_.postFixStdDev.value(), config_.driftStdDevPerInch * travelSinceFix_);" \
  "            config_.postFixStdDev.value();"

mutate "rmsTrustFactor dropped (device claim taken raw)" "$COR" \
  "            std::max(config_.rmsTrustFactor * rms, config_.minPositionStdDev.value());" \
  "            std::max(rms, config_.minPositionStdDev.value());"

mutate "sigma FLOOR removed" "$COR" \
  "            std::max(config_.rmsTrustFactor * rms, config_.minPositionStdDev.value());" \
  "            config_.rmsTrustFactor * rms;"

mutate "yaw-rate rejection removed" "$COR" \
  "        if (!std::isfinite(yawRate) || std::abs(yawRate) > config_.maxYawRate.value()) {" \
  "        if (!std::isfinite(yawRate)) {"

mutate "sensor-quality ceiling removed" "$COR" \
  "        if (rms > config_.maxReportedRms.value()) {" \
  "        if (false) {"

mutate "GPS heading leaks into the proposal (T3)" "$COR" \
  "p.fieldPose = math::Pose2d{units::Length{zxc}, units::Length{zyc}, predicted.heading()};" \
  "p.fieldPose = math::Pose2d{units::Length{zxc}, units::Length{zyc}, fix.heading()};"

mutate "corrector verdict never reaches the record (Localizer substitution removed)" "$LOC" \
  "        if (tickAudit.reason == diag::GateReason::None && selfAuditSource != nullptr) {" \
  "        if (false && selfAuditSource != nullptr) {"

mutate "substitution OVERWRITES a real fusion verdict" "$LOC" \
  "        if (tickAudit.reason == diag::GateReason::None && selfAuditSource != nullptr) {" \
  "        if (selfAuditSource != nullptr) {"

mutate "confidence inverted (trust the sensor more when it is worse)" "$COR" \
  "        const double confidence = sdr2 / (sdr2 + sigmaMeas * sigmaMeas);" \
  "        const double confidence = sigmaMeas * sigmaMeas / (sdr2 + sigmaMeas * sigmaMeas);"

mutate "travel accumulator never resets on an accepted fix" "$COR" \
  "        travelSinceFix_ = 0.0;
        ++accepted_;" \
  "        ++accepted_;"

printf '\n============================================================\n'
printf 'E2 mutations: %d RED (good), %d GREEN (HOLES), %d build-fail, %d SKIPPED\n' \
  "$pass" "$holes" "$buildfail" "$skipped"
printf '============================================================\n'
# A skip is a failure of the harness, not a neutral outcome — it means a mutation the
# brief requires was never actually executed, which is exactly what "a mutation you
# reasoned about but did not run does not count" forbids.
[ "$holes" -eq 0 ] && [ "$skipped" -eq 0 ]
