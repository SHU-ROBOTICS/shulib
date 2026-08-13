#!/usr/bin/env bash
# E4 mutation harness — break it, rebuild, run, OBSERVE the red, restore.
#
# A mutation that stays GREEN is a hole in the suite and the most valuable thing this
# script can find. The runner is GATED ON BUILD SUCCESS: a mutation that fails to compile
# is reported as BUILD-FAIL, never as "red", because a compile error proves nothing about
# the tests.
#
# Usage: docs/internal/verify/verify-e4.sh          (from the repo root)
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
trap 'echo "INTERRUPTED — restoring $CURRENT"; [ -n "$CURRENT" ] && cp /tmp/e4-mut-backup "$CURRENT"; exit 130' INT TERM PIPE
CURRENT=""

# mutate NAME FILE FROM TO [FROM2 TO2]
mutate() {
  local name="$1" file="$2" from="$3" to="$4" from2="${5:-}" to2="${6:-}"
  printf '\n=== MUTATION: %s\n    %s\n' "$name" "$file"
  cp "$file" /tmp/e4-mut-backup
  CURRENT="$file"
  if ! grep -qF -- "$from" "$file"; then
    echo "    *** SKIP — PATTERN NOT FOUND. This mutation did NOT run. ***"
    skipped=$((skipped+1)); CURRENT=""; return
  fi
  if [ -n "$from2" ] && ! grep -qF -- "$from2" "$file"; then
    echo "    *** SKIP — SECOND PATTERN NOT FOUND. This mutation did NOT run. ***"
    skipped=$((skipped+1)); CURRENT=""; return
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
  # mutation is reported GREEN having never run. That happened twice during E3; a byte-compare
  # makes it impossible.
  if cmp -s "$file" /tmp/e4-mut-backup; then
    echo "    *** SKIP — THE EDIT CHANGED NOTHING. This mutation did NOT run. ***"
    skipped=$((skipped+1)); CURRENT=""; return
  fi
  if ! cmake --build "$BUILD" -j"$(nproc)" >/tmp/e4-mut-build.log 2>&1; then
    echo "    BUILD-FAIL — mutation did not compile; proves nothing"
    buildfail=$((buildfail+1))
    cp /tmp/e4-mut-backup "$file"; CURRENT=""
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
  cp /tmp/e4-mut-backup "$file"
  CURRENT=""
  cmake --build "$BUILD" -j"$(nproc)" >/dev/null 2>&1
}

EKF=include/shulib/localization/ekf_fusion.hpp
LOC=include/shulib/localization/localizer.hpp
FUS=include/shulib/localization/complementary_fusion.hpp

# ══ the six the brief REQUIRES ════════════════════════════════════════════════════════

mutate "COVARIANCE SYMMETRY BROKEN — symmetrize() is a no-op (brief #1)" "$EKF" \
  "                const double avg = 0.5 * (at(P_, i, j) + at(P_, j, i));
                at(P_, i, j) = avg;
                at(P_, j, i) = avg;" \
  "                const double avg = 0.5 * (at(P_, i, j) + at(P_, j, i));
                (void)avg;"

mutate "JOSEPH FORM DROPPED — P = (I-KH)P, valid only at the OPTIMAL gain (brief #2)" "$EKF" \
  "        Mat AP{};
        multiply(A, P_, AP);
        Mat next{};
        multiplyTransposed(AP, A, next);
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = 0; j < kN; ++j) {
                double sum = 0.0;
                for (std::size_t a = 0; a < m; ++a) {
                    for (std::size_t b = 0; b < m; ++b) {
                        sum += K[i * 2 + a] * R[a * m + b] * K[j * 2 + b];
                    }
                }" \
  "        Mat next{};
        multiply(A, P_, next);
        for (std::size_t i = 0; i < kN; ++i) {
            for (std::size_t j = 0; j < kN; ++j) {
                double sum = 0.0;
                for (std::size_t a = 0; a < m; ++a) {
                    for (std::size_t b = 0; b < m; ++b) {
                        sum += 0.0 * K[i * 2 + a] * R[a * m + b] * K[j * 2 + b];
                    }
                }"

mutate "MAHALANOBIS COMPARISON INVERTED (brief #3)" "$EKF" \
  "        if (!(std::isfinite(d2) && d2 >= 0.0 && d2 <= gate * gate)) {" \
  "        if (!(std::isfinite(d2) && d2 >= 0.0 && d2 >= gate * gate)) {"

mutate "PROCESS NOISE INDEPENDENT OF TRAVEL (brief #4)" "$EKF" \
  "        const double spBefore = cfg_.posNoisePerInch * travelBefore;
        const double spAfter = cfg_.posNoisePerInch * travelSinceFix_;" \
  "        const double spBefore = 0.0 * travelBefore;
        const double spAfter = 0.0 * travelSinceFix_;"

mutate "RE-INIT FIRES SILENTLY — no word on the record (brief #5)" "$EKF" \
  "        out.audit.reason = diag::GateReason::CovarianceReinit;" \
  "        (void)out;"

mutate "MULTI-PROPOSAL WEIGHTING BROKEN — only the first proposal is folded (brief #6)" "$EKF" \
  "        for (std::size_t k = 0; k < n; ++k) {
            const CorrectionProposal& p = valid[order[k]];" \
  "        for (std::size_t k = 0; k < n && k < 1; ++k) {
            const CorrectionProposal& p = valid[order[k]];"

# ══ the five DEFECTS this chunk actually found, re-armed ══════════════════════════════
# Each of these was a real bug in the first working version of the filter. If any goes
# green, the test that caught it during development is not in the suite.

mutate "DEFECT 1 — the odometry measurement back in the FIELD frame (P_theta swamps it)" "$EKF" \
  "        H[0 * kN + kVx] = 1.0;
        H[1 * kN + kVy] = 1.0;

        const double zx = (ux * c + uy * s) / h;   // R(θ)ᵀ u / dt
        const double zy = (-ux * s + uy * c) / h;
        const std::array<double, 2> r{zx - x_[kVx], zy - x_[kVy]};" \
  "        H[0 * kN + kTh] = -(x_[kVx] * s + x_[kVy] * c);
        H[0 * kN + kVx] = c;
        H[0 * kN + kVy] = -s;
        H[1 * kN + kTh] = (x_[kVx] * c - x_[kVy] * s);
        H[1 * kN + kVx] = s;
        H[1 * kN + kVy] = c;

        const std::array<double, 2> r{ux / h - (x_[kVx] * c - x_[kVy] * s),
                                      uy / h - (x_[kVx] * s + x_[kVy] * c)};"

mutate "DEFECT 2 — process noise as a RANDOM WALK instead of a systematic bias" "$EKF" \
  "        const double dPosVar = spAfter * spAfter - spBefore * spBefore +
                               cfg_.posNoiseRate.value() * h * cfg_.posNoiseRate.value() * h;" \
  "        const double dPosVar = (cfg_.posNoisePerInch * travel) * (cfg_.posNoisePerInch * travel) +
                               cfg_.posNoiseRate.value() * h * cfg_.posNoiseRate.value() * h;
        (void)spBefore; (void)spAfter;"

mutate "DEFECT 3 — the ODOMETRY channel put back through the Mahalanobis gate" "$EKF" \
  "                    /*headBudget=*/kUnbounded, /*gate=*/kUnbounded, ignored);" \
  "                    /*headBudget=*/kUnbounded, /*gate=*/cfg_.gateSigma, ignored);"

mutate "DEFECT 4 — the standing-still floor made systematic (P grows linearly at rest)" "$EKF" \
  "        const double spBefore = cfg_.posNoisePerInch * travelBefore;
        const double spAfter = cfg_.posNoisePerInch * travelSinceFix_;" \
  "        const double spBefore = cfg_.posNoisePerInch * travelBefore +
                                cfg_.posNoiseRate.value() * timeBefore;
        const double spAfter = cfg_.posNoisePerInch * travelSinceFix_ +
                               cfg_.posNoiseRate.value() * timeSinceFix_;"

mutate "DEFECT 5 — the filtering residual no longer charged against the per-tick budget" "$EKF" \
  "        const double alreadyMoved = std::hypot(x_[kPx] - predX, x_[kPy] - predY);" \
  "        const double alreadyMoved = 0.0 * std::hypot(x_[kPx] - predX, x_[kPy] - predY);"

# ══ never-snap, T1, T2, T4, T5 ═══════════════════════════════════════════════════════

mutate "NEVER-SNAP: the rate clamp applied to the STATE, not to the GAIN" "$EKF" \
  "        if (scale < 1.0) {
            out.clamped = true;
            for (std::size_t i = 0; i < kN * 2; ++i) {
                K[i] *= scale;
            }" \
  "        if (scale < 1.0) {
            out.clamped = true;
            for (std::size_t i = 0; i < kN * 2; ++i) {
                K[i] *= 1.0;
            }"

mutate "NEVER-SNAP: the per-tick budget is not spent, so N proposals each get a full one" "$EKF" \
  "                posBudget = std::max(0.0, posBudget - o.dPos);" \
  "                posBudget = std::max(0.0, posBudget - 0.0 * o.dPos);"

mutate "NEVER-SNAP: the heading budget is not spent" "$EKF" \
  "                headBudget = std::max(0.0, headBudget - oh.dHeading);" \
  "                headBudget = std::max(0.0, headBudget - 0.0 * oh.dHeading);"

mutate "T1: the theta gain row is NOT blocked — a position fix rotates the robot" "$EKF" \
  "            const bool blocked = (i == kTh && !mayMoveHeading) ||
                                 ((i == kPx || i == kPy) && !mayMovePosition);" \
  "            const bool blocked = (i == kTh && !mayMoveHeading && false) ||
                                 ((i == kPx || i == kPy) && !mayMovePosition);"

mutate "T1: providesHeading ignored — any proposal's heading is folded" "$EKF" \
  "            if (!p.providesHeading) {
                continue;
            }" \
  "            if (false) {
                continue;
            }"

# THE FAILURE THE p-ROW BLOCK STANDS FOR, in its dangerous form: the odometry increment folded
# as an ABSOLUTE POSITION measurement anchored at the previous posterior. This is the version
# that collapses the position covariance every tick and locks the gate shut forever.
mutate "T1: the odometry folded as an ABSOLUTE POSITION measurement (P collapses)" "$EKF" \
  "        std::array<double, 2 * kN> H{};
        H[0 * kN + kVx] = 1.0;
        H[1 * kN + kVy] = 1.0;

        const double zx = (ux * c + uy * s) / h;   // R(θ)ᵀ u / dt
        const double zy = (-ux * s + uy * c) / h;
        const std::array<double, 2> r{zx - x_[kVx], zy - x_[kVy]};" \
  "        std::array<double, 2 * kN> H{};
        H[0 * kN + kPx] = 1.0;
        H[1 * kN + kPy] = 1.0;
        (void)c; (void)s;
        const std::array<double, 2> r{lastX_ + ux - x_[kPx], lastY_ + uy - x_[kPy]};" \
  "                    /*mayMovePosition=*/false, /*posBudget=*/kUnbounded," \
  "                    /*mayMovePosition=*/true, /*posBudget=*/kUnbounded,"

# KNOWN GREEN, recorded with its measurement (E4-COMPLETED). Unblocking the position rows of
# the VELOCITY update leaks only through the p–v cross-covariance, and the leak is tiny:
# measured over three blind trajectories the position trace moves 223878 -> 223877,
# 6765.24 -> 6764.94 and 2469.77 -> 2469.41, i.e. between 4e-6 and 1.5e-4 relative. Any test
# tight enough to separate those numbers would be pinning an invented constant, which this
# chunk refuses to do. The block stays because it is the structural expression of the ruling
# above, whose dangerous form IS caught.
mutate "T1 (KNOWN GREEN): the position rows of the VELOCITY update unblocked" "$EKF" \
  "                    /*mayMovePosition=*/false, /*posBudget=*/kUnbounded," \
  "                    /*mayMovePosition=*/true, /*posBudget=*/kUnbounded,"

mutate "T2: re-init TELEPORTS the state onto the rejected fix (the snap §13 #4 forbids)" "$EKF" \
  "        at(P_, kPx, kPx) = sp * sp;
        at(P_, kPy, kPy) = sp * sp;
        at(P_, kVx, kVx) = sv * sv;
        at(P_, kVy, kVy) = sv * sv;
        ++reinitCount_;" \
  "        at(P_, kPx, kPx) = sp * sp;
        at(P_, kPy, kPy) = sp * sp;
        at(P_, kVx, kVx) = sv * sv;
        at(P_, kVy, kVy) = sv * sv;
        x_[kPx] += out.audit.residualX.value();
        x_[kPy] += out.audit.residualY.value();
        ++reinitCount_;"

mutate "T2: the re-init COOLDOWN removed (a re-init storm)" "$EKF" \
  "        if (reinitCount_ > 0 && (elapsed_ - lastReinitAt_) < cfg_.reinitCooldown.value()) {" \
  "        if (false) {"

mutate "T2: the consecutive-rejection threshold removed (fires on the first bad fix)" "$EKF" \
  "        if (consecutiveRejects_ < cfg_.reinitRejectCount) {" \
  "        if (consecutiveRejects_ < 1) {"

mutate "T2: the mean-innovation bar removed (re-init on borderline rejections)" "$EKF" \
  "        if (meanInnovation < cfg_.reinitInnovation.value()) {" \
  "        if (meanInnovation < 0.0 * cfg_.reinitInnovation.value()) {"

mutate "T2: an accepted fix does not reset the consecutive-rejection run" "$EKF" \
  "        if (out.applied) {
            consecutiveRejects_ = 0;
            rejectSum_ = 0.0;" \
  "        if (false) {
            consecutiveRejects_ = 0;
            rejectSum_ = 0.0;"

mutate "T4/T5: sigma-weighting replaced by the CONFIDENCE gain knob" "$EKF" \
  "            const double rr = sigma * sigma;" \
  "            const double rr = (1.0 - std::clamp(p.confidence, 0.0, 0.99)) *
                              (1.0 - std::clamp(p.confidence, 0.0, 0.99));"

mutate "T4/T5: proposals folded in ARRIVAL order rather than ascending sigma" "$EKF" \
  "            while (j > 0 && valid[order[j - 1]].positionStdDev.value() >
                                valid[i].positionStdDev.value()) {" \
  "            while (false && valid[order[j - 1]].positionStdDev.value() >
                                valid[i].positionStdDev.value()) {"

mutate "T5: gateMahalanobis reports the raw residual instead of the normalized distance" "$EKF" \
  "        out.mahalanobis = (std::isfinite(d2) && d2 >= 0.0) ? std::sqrt(d2) : 0.0;" \
  "        out.mahalanobis = std::hypot(r[0], (m > 1) ? r[1] : 0.0);"

mutate "T5: covarianceTrace reports the FULL 5-state trace (inches + radians + in/s)" "$EKF" \
  "        return at(P_, kPx, kPx) + at(P_, kPy, kPy);" \
  "        return at(P_, kPx, kPx) + at(P_, kPy, kPy) + at(P_, kTh, kTh) + at(P_, kVx, kVx) +
               at(P_, kVy, kVy);"

# ══ the rest of the structure ════════════════════════════════════════════════════════

mutate "A REJECTED FIX STILL SHRINKS THE COVARIANCE (an outlier buys confidence)" "$EKF" \
  "        if (!(std::isfinite(d2) && d2 >= 0.0 && d2 <= gate * gate)) {
            return;  // rejected: nothing is touched
        }" \
  "        const bool rejected = !(std::isfinite(d2) && d2 >= 0.0 && d2 <= gate * gate);
        if (rejected) {
            at(P_, kPx, kPx) *= 0.5;
            at(P_, kPy, kPy) *= 0.5;
            return;
        }"

mutate "A dt<=0 TICK IS TREATED AS A NORMAL PREDICTION (a teleport integrated as motion)" "$EKF" \
  "        if (!(h > 0.0) || h > cfg_.maxDt) {
            resync(px, py, ph);
            return passThrough(px, py);
        }" \
  "        if (false) {
            resync(px, py, ph);
            return passThrough(px, py);
        }"

mutate "THE VELOCITY UPDATE MOVED AFTER THE PROPAGATION (a full tick of odometry lag)" "$EKF" \
  "        odometryUpdate(ux, uy, h, travel);

        // ── STEP C — propagate position with the posterior velocity ─────────────────────
        propagatePosition(h);" \
  "        propagatePosition(h);
        odometryUpdate(ux, uy, h, travel);"

mutate "THE INITIAL PRIOR IS ZERO — the filter starts infinitely confident" "$EKF" \
  "        at(P_, kPx, kPx) = sp * sp;
        at(P_, kPy, kPy) = sp * sp;
        at(P_, kTh, kTh) = sh * sh;" \
  "        at(P_, kPx, kPx) = 1e-12 * sp;
        at(P_, kPy, kPy) = 1e-12 * sp;
        at(P_, kTh, kTh) = sh * sh;"

mutate "THE NON-FINITE SCREEN ON THE INPUTS REMOVED (a NaN enters the covariance)" "$EKF" \
  "            const bool wellFormed = std::isfinite(zx) && std::isfinite(zy) &&
                                    std::isfinite(sigma) && sigma > 0.0;" \
  "            const bool wellFormed = true;"

mutate "THE POSTERIOR FINITENESS GUARD REMOVED" "$EKF" \
  "        for (std::size_t i = 0; i < kN; ++i) {
            if (!std::isfinite(delta[i])) {
                ++numericGuardTrips_;
                return;
            }" \
  "        for (std::size_t i = 0; i < kN; ++i) {
            if (false) {
                ++numericGuardTrips_;
                return;
            }"

# REMOVED, and the removal is the finding. This mutation targeted a guard that zeroed
# `headingNudge` when nothing was applied. The harness proved the guard could never fire
# (`headingSum` accumulates only inside the branch that sets `headingApplied`, and the Localizer
# ignores the nudge unless that flag is set), so the guard was DELETED rather than tested around.
# A defensive line no mutation can kill is a line that should not be there. Left as a comment so
# the next reader does not re-add it.

mutate "THE COMPLEMENTARY TIER'S 12-INCH GATE REMOVED (the fallback tier degraded)" "$FUS" \
  "            if (!std::isfinite(innoMag) || !std::isfinite(p.confidence) || innoMag > gate) {" \
  "            if (!std::isfinite(innoMag) || !std::isfinite(p.confidence) || innoMag > gate * 1e9) {"

mutate "THE COMPLEMENTARY TIER STARTS FAKING A MAHALANOBIS DISTANCE (E2's T1 reversed)" "$FUS" \
  "            audit.covarianceTrace = maxConf;" \
  "            audit.covarianceTrace = maxConf;
            audit.mahalanobis = std::hypot(auditInnoX, auditInnoY) / 3.0;"

mutate "THE LOCALIZER STOPS FOLDING THE HEADING NUDGE (E3's accumulator, under the EKF)" "$LOC" \
  "        if (fr.headingApplied && std::isfinite(nudgeH)) {" \
  "        if (false && fr.headingApplied && std::isfinite(nudgeH)) {"

printf '\n────────────────────────────────────────────────────────────\n'
printf 'E4 mutations: %d RED (good), %d GREEN (HOLES), %d build-fail, %d SKIPPED\n' \
  "$pass" "$holes" "$buildfail" "$skipped"
if [ "$skipped" -ne 0 ]; then
  echo 'A SKIP MEANS A REQUIRED MUTATION NEVER RAN. Exiting non-zero.'
  exit 2
fi
[ "$holes" -eq 0 ] || exit 1
