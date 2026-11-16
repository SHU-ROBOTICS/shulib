// curve.mjs — THE LIBRARY'S CURVE, in the page (RMP1 brief [R6]; landmine 1).
//
// A line-for-line JavaScript port of include/shulib/motion/curve_path.hpp's CurvePath (CURVE1): the knots
// (the robot's pose, then the author's points), each knot's tangent (its heading, or Catmull-Rom from its
// neighbours), the cubic Hermite segments whose tangent magnitude is the chord, the arc-length table
// (17 samples per segment, stored as FLOAT, exactly as the C++ stores it — Math.fround), at(s), the
// tightest bend, and checkCurve()'s verdicts (Invalid / StartBehind / TightBend) with the library's own
// words (appendCurveCheckWords). The field draws THIS curve — the path the follower is told to follow —
// never a smoothed guess, and the page refuses exactly what checkCurve refuses.
//
// Pinned by CV1 (tools/route_maker/test/curve.test.mjs): 200 seeded curves made and judged by the C++
// (test/goldens/route_maker/curves.txt, written by `shulib_route_maker curves`, kept fresh by the suite)
// — every verdict, why, point and sentence EXACTLY; every sampled position within 0.001 in (the library
// stores its arc-length table as float; the maths libraries round sin/cos/atan2 differently in the last
// bit, A.1).
//
// Only what a tool may rely on is ported (the header says which: CurvePath, build(), length(), at(),
// checkCurve()); the follower's speed plan is NOT — the page needs only the tightest bend, which the plan's
// interval scan finds, so that scan is ported in the C++'s order.
//
// Pure: no DOM, no file API.

/** The most points one curve takes (curve_path.hpp: kMaxCurvePoints). */
export const kMaxCurvePoints = 16;
/** The closest two knots may be, in (curve_path.hpp: kCurveMinSpacing). */
export const kCurveMinSpacing = 1.0;
/** Arc-length samples per segment (curve_path.hpp: CurvePath::kSamples). */
const kSamples = 17;
/** The curvature scan's grid (in) (curve_path.hpp: kScanStep). */
const kScanStep = 0.25;

const kPi = 3.14159265358979323846;
const kDegToRad = kPi / 180.0;
const kRadToDeg = 180.0 / kPi;

/** The library's default CurveConfig numbers the page reads (motion_config.hpp; the robot's own if its
 *  description states them). */
export const kDefaultCurveConfig = Object.freeze({ maxStartOff: 1.5707963267948966, lookAhead: 8.0 });

/** IEEE 754 remainder (std::remainder): x - n*y, n the quotient rounded half to even. */
function remainderIEEE(x, y) {
    const q = x / y;
    let n = Math.round(q);
    if (Math.abs(q - Math.trunc(q)) === 0.5) {
        n = 2 * Math.round(q / 2);
    }
    return x - n * y;
}

/** math::Angle::wrapRad: into (-pi, pi]. */
export function wrapRad(r) {
    let w = remainderIEEE(r, 2.0 * kPi);
    if (w <= -kPi) {
        w += 2.0 * kPi;
    }
    return w;
}

/** math::Angle::degrees(d).radians(). */
export function degToRad(d) {
    return wrapRad(d * kDegToRad);
}

function dist(a, b) {
    return Math.hypot(b.x - a.x, b.y - a.y);
}

function angleBetween(tx, ty, cx, cy) {
    const l = Math.hypot(cx, cy);
    if (!(l > 0.0)) {
        return 0.0;
    }
    const c = Math.min(1.0, Math.max(-1.0, (tx * cx + ty * cy) / l));
    return Math.acos(c);
}

/** The verdicts, in the order checkCurve decides them (curve_path.hpp: CurveVerdict). */
export const CurveVerdict = Object.freeze({ Ok: 'Ok', Invalid: 'Invalid', StartBehind: 'StartBehind', TightBend: 'TightBend' });
/** Why a path is Invalid (curve_path.hpp: CurveInvalid). */
export const CurveInvalid = Object.freeze({
    None: 'None', NoPoints: 'NoPoints', AtRobot: 'AtRobot', TooClose: 'TooClose', NoDirection: 'NoDirection',
    DoublesBack: 'DoublesBack',
});

/**
 * The smooth path of one curve. `points`: 1..16 of {x, y} or {x, y, headingDeg} (field inches; a heading
 * is the robot's NOSE, also reversing). `reverse`: the robot's back leads (reverseCurveTo).
 */
export class CurvePath {
    constructor(points, reverse) {
        if (!Array.isArray(points) || points.length < 1 || points.length > kMaxCurvePoints) {
            throw new Error('CurvePath: a curve takes 1 to 16 points (a longer route is two curves)');
        }
        for (const p of points) {
            if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) {
                throw new Error("CurvePath: every point's x and y must be finite");
            }
        }
        this.reverse = !!reverse;
        this.authorCount = points.length;
        this.knots = [];
        for (let i = 0; i <= kMaxCurvePoints; ++i) {
            this.knots.push({ x: 0, y: 0, heading: 0, hasHeading: false, tx: 1, ty: 0 });
        }
        points.forEach((p, i) => {
            const k = this.knots[i + 1];
            const has = p.headingDeg !== undefined && p.headingDeg !== null;
            k.x = p.x;
            k.y = p.y;
            k.heading = has ? degToRad(p.headingDeg) : 0.0;
            k.hasHeading = has;
        });
        this.base = 0;
        this.n = 0;
        this.droppedDistance = -1.0;
        this.droppedX = 0;
        this.droppedY = 0;
        this.degenerate = false;
        this.invalid = { verdict: CurveVerdict.Ok, why: CurveInvalid.None, point: 0, distance: 0, angle: 0 };
        this.cum = new Array(kMaxCurvePoints + 1).fill(0);
        this.arc = [];
        for (let k = 0; k < kMaxCurvePoints; ++k) {
            this.arc.push(new Float32Array(kSamples));
        }
        this.startOff = 0;
        this.maxCurvature = 0;
        this.maxCurvatureAt = 0;
        this.built = false;
    }

    /** Build from the robot's pose {x, y, headingDeg} (curve_path.hpp: build()). */
    build(start, cfg = kDefaultCurveConfig) {
        if (this.base === 1) {
            this.knots[1] = { x: this.droppedX, y: this.droppedY, heading: 0, hasHeading: false, tx: 1, ty: 0 };
        }
        this.base = 0;
        this.droppedDistance = -1.0;
        this.invalid = { verdict: CurveVerdict.Ok, why: CurveInvalid.None, point: 0, distance: 0, angle: 0 };
        const startKnot = { x: start.x, y: start.y, heading: degToRad(start.headingDeg), hasHeading: true, tx: 1, ty: 0 };
        const d1 = dist(startKnot, this.knots[1]);
        if (d1 < kCurveMinSpacing && !this.knots[1].hasHeading) {
            this.droppedDistance = d1;
            this.droppedX = this.knots[1].x;
            this.droppedY = this.knots[1].y;
            this.base = 1;
        }
        this.knots[this.base] = startKnot;
        this.n = this.authorCount - this.base;
        this.degenerate = this.n === 0;
        if (this.degenerate) {
            this.invalid = { verdict: CurveVerdict.Invalid, why: CurveInvalid.NoPoints, point: 1,
                             distance: this.droppedDistance, angle: 0 };
            this.knots[2] = { ...startKnot };
            this.n = 1;
        }
        this.computeTangents();
        this.computeDirectionFacts();
        this.firstInvalid(cfg);
        this.computeArcLengths();
        this.scanCurvature();
        this.built = true;
        return this;
    }

    K(i) {
        return this.knots[this.base + i];
    }

    /** Segments in the built path. */
    segments() {
        return this.n;
    }

    /** The author's number of the first point driven through (2 when point 1 was dropped). */
    firstDrivenPoint() {
        return this.base + 1;
    }

    computeTangents() {
        const flip = this.reverse ? kPi : 0.0;
        for (let i = 0; i <= this.n; ++i) {
            const k = this.K(i);
            if (k.hasHeading) {
                const h = k.heading + flip;
                k.tx = Math.cos(h);
                k.ty = Math.sin(h);
                continue;
            }
            const prev = this.K(i - 1);
            const next = i < this.n ? this.K(i + 1) : k;
            const dx = next.x - prev.x;
            const dy = next.y - prev.y;
            const l = Math.hypot(dx, dy);
            if (i < this.n && l < kCurveMinSpacing) {
                const ax = k.x - prev.x;
                const ay = k.y - prev.y;
                const al = Math.hypot(ax, ay);
                k.tx = al > 0.0 ? ax / al : prev.tx;
                k.ty = al > 0.0 ? ay / al : prev.ty;
                continue;
            }
            if (l > 0.0) {
                k.tx = dx / l;
                k.ty = dy / l;
            } else {
                k.tx = prev.tx;
                k.ty = prev.ty;
            }
        }
    }

    chordAngle(i) {
        const k = this.K(i);
        let worst = 0.0;
        if (i > 0) {
            const p = this.K(i - 1);
            worst = Math.max(worst, angleBetween(k.tx, k.ty, k.x - p.x, k.y - p.y));
        }
        if (i < this.n) {
            const q = this.K(i + 1);
            worst = Math.max(worst, angleBetween(k.tx, k.ty, q.x - k.x, q.y - k.y));
        }
        return worst;
    }

    computeDirectionFacts() {
        const s = this.K(0);
        const f = this.K(1);
        this.startOff = this.degenerate ? 0.0 : angleBetween(s.tx, s.ty, f.x - s.x, f.y - s.y);
    }

    firstInvalid(cfg) {
        if (this.invalid.verdict === CurveVerdict.Invalid) {
            return;
        }
        for (let i = 1; i <= this.n; ++i) {
            const author = i + this.base;
            const d = dist(this.K(i - 1), this.K(i));
            if (d < kCurveMinSpacing) {
                const atRobot = i === 1 && this.K(i).hasHeading;
                this.invalid = { verdict: CurveVerdict.Invalid, why: atRobot ? CurveInvalid.AtRobot : CurveInvalid.TooClose,
                                 point: author, distance: d, angle: 0 };
                return;
            }
            if (!this.K(i).hasHeading && i < this.n) {
                const nd = dist(this.K(i - 1), this.K(i + 1));
                if (nd < kCurveMinSpacing) {
                    this.invalid = { verdict: CurveVerdict.Invalid, why: CurveInvalid.NoDirection, point: author,
                                     distance: nd, angle: 0 };
                    return;
                }
            }
            const ca = this.chordAngle(i);
            if (ca > cfg.maxStartOff) {
                this.invalid = { verdict: CurveVerdict.Invalid, why: CurveInvalid.DoublesBack, point: author,
                                 distance: 0, angle: ca };
                return;
            }
        }
    }

    evalSeg(seg, u) {
        const a = this.K(seg);
        const b = this.K(seg + 1);
        const m = dist(a, b);
        const u2 = u * u;
        const u3 = u2 * u;
        const h00 = 2 * u3 - 3 * u2 + 1;
        const h10 = u3 - 2 * u2 + u;
        const h01 = -2 * u3 + 3 * u2;
        const h11 = u3 - u2;
        const d00 = 6 * u2 - 6 * u;
        const d10 = 3 * u2 - 4 * u + 1;
        const d01 = -6 * u2 + 6 * u;
        const d11 = 3 * u2 - 2 * u;
        const e00 = 12 * u - 6;
        const e10 = 6 * u - 4;
        const e01 = -12 * u + 6;
        const e11 = 6 * u - 2;
        return {
            x: h00 * a.x + h10 * m * a.tx + h01 * b.x + h11 * m * b.tx,
            y: h00 * a.y + h10 * m * a.ty + h01 * b.y + h11 * m * b.ty,
            dx: d00 * a.x + d10 * m * a.tx + d01 * b.x + d11 * m * b.tx,
            dy: d00 * a.y + d10 * m * a.ty + d01 * b.y + d11 * m * b.ty,
            ddx: e00 * a.x + e10 * m * a.tx + e01 * b.x + e11 * m * b.tx,
            ddy: e00 * a.y + e10 * m * a.ty + e01 * b.y + e11 * m * b.ty,
        };
    }

    evalSample(seg, u) {
        const e = this.evalSeg(seg, u);
        const sp = Math.hypot(e.dx, e.dy);
        const c = { x: e.x, y: e.y, direction: 0, heading: 0, curvature: 0 };
        if (sp > 1e-12) {
            c.direction = Math.atan2(e.dy, e.dx);
            c.curvature = (e.dx * e.ddy - e.dy * e.ddx) / (sp * sp * sp);
        } else {
            const a = this.K(seg);
            c.direction = Math.atan2(a.ty, a.tx);
            c.curvature = 0.0;
        }
        c.heading = this.reverse ? wrapRad(c.direction + kPi) : c.direction;
        return c;
    }

    speedIntegral(seg, u0, u1) {
        const kX = [0.0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640, 0.9061798459386640];
        const kW = [0.5688888888888889, 0.4786286704993665, 0.4786286704993665, 0.2369268850561891, 0.2369268850561891];
        const half = 0.5 * (u1 - u0);
        const mid = 0.5 * (u1 + u0);
        let sum = 0.0;
        for (let i = 0; i < kX.length; ++i) {
            const e = this.evalSeg(seg, mid + half * kX[i]);
            sum += kW[i] * Math.hypot(e.dx, e.dy);
        }
        return sum * half;
    }

    computeArcLengths() {
        this.cum[0] = 0.0;
        for (let k = 0; k < this.n; ++k) {
            let s = 0.0;
            this.arc[k][0] = 0.0;
            for (let j = 1; j < kSamples; ++j) {
                const u0 = (j - 1) / (kSamples - 1);
                const u1 = j / (kSamples - 1);
                s += this.speedIntegral(k, u0, u1);
                this.arc[k][j] = Math.fround(s);
            }
            this.cum[k + 1] = this.cum[k] + this.arc[k][kSamples - 1];
        }
    }

    /** The path's length (in). */
    length() {
        return this.cum[this.n];
    }

    locate(s) {
        const sc = Math.min(Math.max(s, 0.0), this.length());
        let k = 0;
        while (k + 1 < this.n && this.cum[k + 1] <= sc) {
            ++k;
        }
        const local = sc - this.cum[k];
        let j = 0;
        while (j + 1 < kSamples && this.arc[k][j + 1] <= local) {
            ++j;
        }
        if (j >= kSamples - 1) {
            return { seg: k, u: 1.0, sample: j, local };
        }
        const a = this.arc[k][j];
        const b = this.arc[k][j + 1];
        const f = b > a ? Math.min(Math.max((local - a) / (b - a), 0.0), 1.0) : 0.0;
        return { seg: k, u: (j + f) / (kSamples - 1), sample: j, local };
    }

    /** The path at arc length `s`: {x, y, direction, heading, curvature} (radians inside the model only). */
    at(s) {
        const l = this.locate(s);
        return this.evalSample(l.seg, l.u);
    }

    /** The robot's NOSE pose at the end, {x, y, headingDeg} (curve_path.hpp: endPose()). */
    endPose() {
        const e = this.K(this.n);
        const h = e.hasHeading && !this.degenerate ? e.heading : this.at(this.length()).heading;
        return { x: e.x, y: e.y, headingDeg: wrapRad(h) * kRadToDeg };
    }

    absCurvature(seg, u) {
        const e = this.evalSeg(seg, u);
        const sp = Math.hypot(e.dx, e.dy);
        return sp > 1e-12 ? Math.abs(e.dx * e.ddy - e.dy * e.ddx) / (sp * sp * sp) : 0.0;
    }

    intervalCurvature(seg, j) {
        const a = this.arc[seg][j];
        const b = this.arc[seg][j + 1];
        const m = Math.max(1, Math.ceil((b - a) / kScanStep));
        let worst = 0.0;
        for (let i = 0; i <= m; ++i) {
            const f = i / m;
            const u = (j + f) / (kSamples - 1);
            const c = this.absCurvature(seg, u);
            worst = Math.max(worst, c);
            if (c > this.maxCurvature) {
                this.maxCurvature = c;
                this.maxCurvatureAt = this.cum[seg] + a + f * (b - a);
            }
        }
        return worst;
    }

    /** The speed plan's curvature scan, in the C++'s order (only the tightest bend is kept: the plan's
     *  2 in look-ahead, kBendAhead, reads curvature for the SPEED only and never moves the tightest bend,
     *  so it is not ported). */
    scanCurvature() {
        this.maxCurvature = 0.0;
        this.maxCurvatureAt = 0.0;
        const kLast = kSamples - 1;
        for (let k = 0; k < this.n; ++k) {
            for (let j = 0; j < kSamples; ++j) {
                if (j > 0) {
                    this.intervalCurvature(k, j - 1);
                } else if (k > 0) {
                    this.intervalCurvature(k - 1, kLast - 1);
                }
                if (j < kLast) {
                    this.intervalCurvature(k, j);
                } else if (k + 1 < this.n) {
                    this.intervalCurvature(k + 1, 0);
                }
            }
        }
    }

    /** The tightest bend's radius (in); +Infinity for a path with no bend. */
    tightestRadius() {
        return this.maxCurvature > 0.0 ? 1.0 / this.maxCurvature : Number.POSITIVE_INFINITY;
    }

    /** Points along the path every `step` in (for drawing and the off-field check). */
    samples(step = 0.5) {
        const out = [];
        const L = this.length();
        const n = Math.max(1, Math.ceil(L / step));
        for (let i = 0; i <= n; ++i) {
            const c = this.at((L * i) / n);
            out.push({ x: c.x, y: c.y, headingDeg: c.heading * kRadToDeg, s: (L * i) / n });
        }
        return out;
    }
}

/** checkCurve (curve_path.hpp): Invalid first, then StartBehind (exactly maxStartOff drives), then
 *  TightBend (tightest radius < lookAhead / 2), else Ok. */
export function checkCurve(path, cfg = kDefaultCurveConfig) {
    if (path.invalid.verdict === CurveVerdict.Invalid) {
        return { ...path.invalid, tightestRadius: 0, reverse: false };
    }
    if (path.startOff > cfg.maxStartOff) {
        return { verdict: CurveVerdict.StartBehind, why: CurveInvalid.None, point: path.firstDrivenPoint(),
                 distance: 0, angle: path.startOff, tightestRadius: 0, reverse: path.reverse };
    }
    if (path.tightestRadius() < cfg.lookAhead / 2.0) {
        return { verdict: CurveVerdict.TightBend, why: CurveInvalid.None, point: 0, distance: 0, angle: 0,
                 tightestRadius: path.tightestRadius(), reverse: false };
    }
    return { verdict: CurveVerdict.Ok, why: CurveInvalid.None, point: 0, distance: 0, angle: 0,
             tightestRadius: path.tightestRadius(), reverse: false };
}

/** printf("%*.*f", width, prec, v) as diag::lineformat::appendNum writes it (width 1: no padding). */
function num(v, prec) {
    if (Number.isNaN(v)) {
        return 'NaN';
    }
    if (!Number.isFinite(v)) {
        return v > 0 ? '+Inf' : '-Inf';
    }
    return v.toFixed(prec);
}

/** The library's own sentence for a check (curve_path.hpp: appendCurveCheckWords); '' for Ok. */
export function curveCheckWords(c, cfg = kDefaultCurveConfig) {
    const point = (p) => `point ${p > 0 ? p : 0}`;
    switch (c.verdict) {
        case CurveVerdict.Ok:
            return '';
        case CurveVerdict.TightBend:
            return `curve: tightest bend ${num(c.tightestRadius, 1)} in -- tighter than the look-ahead can follow; expect it to cut the corner`;
        case CurveVerdict.StartBehind:
            return `curve not driven: its first point is ${num(c.angle * kRadToDeg, 0)}`
                + (c.reverse ? " deg off the robot's back (at most " : " deg off the robot's heading (at most ")
                + num(cfg.maxStartOff * kRadToDeg, 0)
                + (c.reverse ? ') -- turn first: faceAway()' : ') -- turn first: face()');
        default:
            break;
    }
    switch (c.why) {
        case CurveInvalid.AtRobot:
            return `curve not driven: ${point(c.point)} is where the robot already is (${num(c.distance, 1)} in away) -- do not list the start`;
        case CurveInvalid.TooClose:
            return `curve not driven: ${point(c.point)} is ${num(c.distance, 2)} in from the point before it (at least ${num(kCurveMinSpacing, 1)})`;
        case CurveInvalid.NoDirection:
            return `curve not driven: ${point(c.point)} has no direction: the points either side of it are ${num(c.distance, 2)} in apart -- give it a heading`;
        case CurveInvalid.DoublesBack:
            return `curve not driven: ${point(c.point)} doubles back: its direction is ${num(c.angle * kRadToDeg, 0)} deg from the path into or out of it (at most ${num(cfg.maxStartOff * kRadToDeg, 0)}) -- a heading is the robot's NOSE, also backwards`;
        default:
            return 'curve not driven: no point left to drive to -- do not list the start';
    }
}
