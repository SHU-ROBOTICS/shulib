// checks.mjs — EVERY refusal and warning the page shows while the student edits (RMP1 brief §2.2's RC
// list as A.7 cuts it for RMP1-A1: RC1–RC3, RC15–RC21, RC24–RC26; [R1, R6, R13]; landmine 4).
//
// The page refuses what the LIBRARY refuses — a step the library would refuse at build, throw on at run
// time, or stop the route on — BEFORE the student exports it, in the library's own words (library_words.mjs,
// pinned to the headers) with plain English beside them. The C++ stays the final authority: the goldens
// the suite compiles and runs are the twins of these rules (test/route_maker_golden_test.cpp).
//
// Each issue: {code, level: "error" | "warn", stepIndex (-1: the route or its start), field, words, library,
// robot, fix}. `words` is the page's plain sentence (no "precondition", no "Routine"); `library` the
// library's own, or '' when the rule is the page's (said so in the rule's comment). `fix`, when present, is
// a one-click change the page offers — the step it adds is one the student SEES and can remove, and any
// time limit in it is left EMPTY for the student to type (§4: no invented number).
//
// THE PATH RULES (RC15, RC17, RC25) run on EVERY robot that builds the shared route (R13) and follow every
// way the route can go: a step that fails stops the route, carries on, or skips FORWARD to a label
// (STEP1-A). They are checked by walking the steps once and carrying, at each step, every state a path can
// arrive in.
//
// Pure: no DOM, no file API.

import { plannedPoses, MOTION_KINDS, MECHANISM_KINDS, CARE_MOTION_FIELDS, STEP_KINDS, ROUTE_NAME } from './route.mjs';
import { CurvePath, checkCurve, curveCheckWords, CurveVerdict, kMaxCurvePoints } from './curve.mjs';
import { squareCorners, dist, wrapDeg } from './geometry.mjs';
import { libraryWords, tankNotSteeredWords } from './library_words.mjs';
import { fmt } from './numbers.mjs';

/** A corner may sit this far (in) past a wall's inner face — RPV1's THROUGH rule (more than 1 in is THROUGH). */
export const WALL_SLACK_IN = 1.0;
/** "turn to face it first" is off by default only within this (deg) of the leg's line (R6 — a display
 *  rounding, not a robot claim). */
export const TURN_FIRST_EDGE_DEG = 0.5;
/** Labels: what the C++ literal and the page both carry safely (a page rule). */
const LABEL_MAX = 29;

function issue(code, level, stepIndex, field, words, library = '', extra = {}) {
    return { code, level, stepIndex, field, words, library, robot: null, fix: null, alsoFix: null, ...extra };
}

/** The mechanism kinds' words. */
const KIND_WORDS = { arm: 'an arm', lift: 'a lift', flywheel: 'a flywheel', power: 'a power mechanism', pneumatic: 'a pneumatic' };
/** Which mechanism kinds each step takes (the C++ overloads; anything else is a build error there). */
const STEP_TAKES = {
    move: ['arm', 'lift'], startMove: ['arm', 'lift'], home: ['arm', 'lift'], startHome: ['arm', 'lift'],
    set: ['pneumatic'], run: ['power'], startRun: ['power'],
    waitUntilDone: ['arm', 'lift', 'flywheel', 'power', 'pneumatic'],
};

/** How far (in) the worst corner of the robot's square is past the field's walls at a pose (≤ 0 inside). */
export function wallOvershoot(x, y, headingDeg, field, size) {
    const w = field.wallInnerFaceIn;
    let worst = Number.NEGATIVE_INFINITY;
    for (const c of squareCorners(x, y, headingDeg, size)) {
        worst = Math.max(worst, c.x - w, -w - c.x, c.y - w, -w - c.y);
    }
    return worst;
}

function wallName(x, y, field) {
    const w = field.wallInnerFaceIn;
    const d = [[w - x, 'the right wall'], [x + w, 'the left wall'], [w - y, 'the blue wall'], [y + w, 'the red wall']];
    d.sort((a, b) => a[0] - b[0]);
    return d[0][1];
}

/** The square's worst overshoot while it turns in place from `fromDeg` to `toDeg` the short way. */
function turnOvershoot(x, y, fromDeg, toDeg, field, size) {
    const sweep = wrapDeg(toDeg - fromDeg);
    const n = Math.max(1, Math.ceil(Math.abs(sweep) / 3));
    let worst = Number.NEGATIVE_INFINITY;
    for (let i = 0; i <= n; ++i) {
        worst = Math.max(worst, wallOvershoot(x, y, fromDeg + (sweep * i) / n, field, size));
    }
    return worst;
}

function mechOf(robot, ident) {
    return (robot.mechanisms || []).find((m) => m.ident === ident) || null;
}

/** A label a C++ string literal and the page both carry: printable ASCII, no " or \, at most 29. */
export function labelProblem(label) {
    if (label.length === 0) {
        return 'a label needs a name';
    }
    if (label.length > LABEL_MAX) {
        return `a label is at most ${LABEL_MAX} characters`;
    }
    if (!/^[\x20-\x7e]+$/.test(label) || /["\\]/.test(label)) {
        return 'a label uses letters, digits, spaces and plain punctuation only (no " or \\)';
    }
    if (label.trim() !== label) {
        return 'a label cannot start or end with a space';
    }
    return null;
}

/**
 * The route's name, as checkAutonEntry and MIRROR1 D5 take it (RC21) — and, in A1, §6.3's CONTRACT: the one
 * route is named "skills" (its marker lines and `e.name`, which robot one's and robot two's code register and
 * ROB1-BT's tests pin). The page offers no rename, so ANY other name — one the library's rules refuse too
 * ("NONE", empty, too long, a quote, a %) — carries the one-click fix "Name it skills" on its error, and no
 * sentence here asks for a name the page cannot take; a hand-edited file's first export cannot rewrite the markers.
 */
function nameIssues(route, mirrored) {
    const out = [];
    const n = route.name ?? '';
    const max = mirrored ? 25 : 29;
    if (n.length === 0 || n.length > 29 || !/^[\x20-\x7e]*$/.test(n) || n.includes('%')) {
        out.push(issue('RC21', 'error', -1, 'name', 'The route\'s name is shown on the brain\'s screen: it is 1 to 29 plain '
            + 'characters (letters, digits, spaces), without %.', libraryWords('autonName')));
    } else if (n === 'NONE') {
        out.push(issue('RC21', 'error', -1, 'name', 'NONE is the brain menu\'s own choice for "no auton", so no route can '
            + 'have that name.', libraryWords('autonNone')));
    } else if (n.length > max) {
        out.push(issue('RC21', 'error', -1, 'name', `Robot two shows this route as "${n} SKILLS MIR" on one screen line, `
            + 'so a mirrored route\'s name is at most 25 characters.', ''));
    } else if (/["\\]/.test(n)) {
        out.push(issue('RC21', 'error', -1, 'name', 'A route name cannot contain " or \\.', ''));
    }
    if (n !== ROUTE_NAME) {
        const fix = { kind: 'nameSkills', stepIndex: -1 };
        if (out.length === 0) {
            out.push(issue('RC21', 'error', -1, 'name', `This route is named "${n}". Both robots' code registers the skills route as `
                + `"${ROUTE_NAME}" (the lines the export writes between are named for it too), so this route builder keeps that name.`,
            '', { contract: true, fix }));
        } else {
            out[0].fix = fix;  // whichever rule refused it: the page's one way to a name is "skills"
        }
    }
    return out;
}

// ── the path walk (RC17, RC25): every state a path can arrive at each step in ───────────────────────────

/** The step kinds that START something on a mechanism (a step the routine "began" on it). */
const BEGINS = new Set(['move', 'startMove', 'set', 'run', 'startRun', 'home', 'startHome']);

/**
 * For each step index, the list of distinct states a path can arrive in. A state: {homed: {m: 'no' |
 * 'running' | 'yes' | 'failed'}, started: {m: bool}, via: description of the path's last branch}. Mechanisms
 * homed at power-up (restsAtStop, an absolute sensor) start 'yes'; a switch- or stall-homed one starts 'no'
 * (the page cannot know a switch is pressed at boot — the host test plant always presses it, P6 (3)).
 */
function walkPaths(route, robot) {
    const n = route.steps.length;
    const labelAt = new Map();
    route.steps.forEach((s, i) => { if (s.label) { labelAt.set(s.label, i); } });
    const initial = { homed: {}, started: {}, via: null };
    for (const m of robot.mechanisms || []) {
        if (m.kind === 'arm' || m.kind === 'lift') {
            initial.homed[m.ident] = (m.homing === 'restsAtStop' || m.homing === 'absolute') ? 'yes' : 'no';
        }
    }
    const arrive = Array.from({ length: n + 1 }, () => new Map());
    const add = (i, st) => {
        if (i > n) {
            return;
        }
        const key = JSON.stringify([st.homed, st.started]);
        if (!arrive[i].has(key)) {
            arrive[i].set(key, st);
        }
    };
    add(0, initial);
    for (let i = 0; i < n; ++i) {
        const s = route.steps[i];
        for (const st of arrive[i].values()) {
            // success
            const ok = { homed: { ...st.homed }, started: { ...st.started }, via: st.via };
            const m = s.mechanism;
            if (m !== undefined && BEGINS.has(s.do)) {
                ok.started[m] = true;
            }
            if (s.do === 'home' && m in ok.homed) {
                ok.homed[m] = 'yes';
            } else if (s.do === 'startHome' && m in ok.homed) {
                ok.homed[m] = 'running';
            } else if (s.do === 'waitUntilDone' && ok.homed[m] === 'running') {
                ok.homed[m] = 'yes';
            } else if ((s.do === 'move' || s.do === 'startMove') && ok.homed[m] === 'running') {
                ok.homed[m] = 'no';  // a new command REPLACES a running home (service.hpp :399-402)
            }
            add(i + 1, ok);
            // failure: stop (no successor), carry on, or skip forward
            const f = s.care && s.care.onFail;
            if (!f || f === 'stop') {
                continue;
            }
            const bad = { homed: { ...st.homed }, started: { ...st.started },
                          via: f === 'carryOn' ? `step ${i + 1} fails and carries on` : `step ${i + 1} fails and skips to "${f.skipTo}"` };
            if (m !== undefined && BEGINS.has(s.do)) {
                bad.started[m] = true;  // it began, even if it then failed
            }
            if ((s.do === 'home' || s.do === 'startHome') && m in bad.homed) {
                bad.homed[m] = 'failed';
            } else if (s.do === 'waitUntilDone' && bad.homed[m] === 'running') {
                bad.homed[m] = 'failed';
            }
            if (f === 'carryOn') {
                add(i + 1, bad);
            } else if (typeof f === 'object') {
                const j = labelAt.get(f.skipTo);
                if (j !== undefined && j > i) {
                    for (let k = i + 1; k < j; ++k) {
                        // the skipped steps never run; nothing to apply
                    }
                    add(j, bad);
                }
            }
        }
    }
    return arrive.map((m) => [...m.values()]);
}

/**
 * Every issue on `route` for the robots in `robots` (each a robot description; the first is the route's
 * own robot, whose numbers it is written in). `field`: {wallInnerFaceIn}.
 */
export function checkRoute(route, robots, field) {
    const out = [];
    const primary = robots[0];
    const size = primary.drive && primary.drive.footprintIn ? primary.drive.footprintIn : 15;
    const curveCfg = {
        maxStartOff: ((primary.drive && primary.drive.curve && primary.drive.curve.maxStartOffDeg) || 90) * Math.PI / 180,
        lookAhead: (primary.drive && primary.drive.curve && primary.drive.curve.lookAheadIn) || 8,
    };
    const tank = !!(primary.drive && primary.drive.tank);
    const maxSteer = primary.drive && primary.drive.maxSteerDeg !== undefined ? primary.drive.maxSteerDeg : 15;

    out.push(...nameIssues(route, !!route.mirroredFor));

    // RC3: the start.
    if (!route.start) {
        if (route.steps.length > 0) {
            out.push(issue('RC3', 'error', -1, 'start', 'Place robot one\'s start on the field first: every step is planned from it.'));
        }
    } else {
        const s = route.start;
        if (![s.x, s.y, s.headingDeg].every(Number.isFinite)) {
            out.push(issue('RC3', 'error', -1, 'start', 'The start needs an x, a y and a heading (numbers).'));
        } else {
            const o = wallOvershoot(s.x, s.y, s.headingDeg, field, size);
            if (o > WALL_SLACK_IN) {
                out.push(issue('RC3', 'error', -1, 'start', `At the start the robot's ${size} in square goes ${o.toFixed(1)} in into `
                    + `${wallName(s.x, s.y, field)}: move it onto the field.`));
            }
        }
    }

    const plan = plannedPoses(route, curveCfg);
    const labels = new Map();
    route.steps.forEach((s, i) => {
        if (s.label !== undefined) {
            const p = labelProblem(s.label);
            if (p) {
                out.push(issue('RC16', 'error', i, 'label', `${p[0].toUpperCase()}${p.slice(1)}.`,
                               s.label === '' ? libraryWords('labelEmpty') : ''));
            } else if (labels.has(s.label)) {
                out.push(issue('RC16', 'error', i, 'label', `Step ${labels.get(s.label) + 1} already has the label "${s.label}": `
                    + 'each label names one step, so a skip knows where to land.'));
            } else {
                labels.set(s.label, i);
            }
        }
    });

    route.steps.forEach((s, i) => {
        const p = plan[i];
        const nmb = (v) => Number.isFinite(v);
        // care: the motion options are validated finite and >= 0 by the library; 0 = the robot's own.
        if (s.care) {
            for (const f of CARE_MOTION_FIELDS) {
                if (s.care[f] !== undefined && (!nmb(s.care[f]) || s.care[f] < 0)) {
                    out.push(issue('RC19', 'error', i, f, 'This needs a number, 0 or more (0 means the robot\'s own).'));
                }
                if (s.care[f] !== undefined && !MOTION_KINDS.has(s.do)) {
                    out.push(issue('RC19', 'error', i, f, 'Only a step that drives or turns takes this.'));
                }
            }
            if (s.care.toleranceDeg !== undefined && s.care.toleranceDeg >= 180) {
                out.push(issue('RC19', 'error', i, 'toleranceDeg', 'A heading tolerance is under 180 deg.'));
            }
            // RC15: a skip lands on a LATER step's label, on every robot's list.
            const f = s.care.onFail;
            if (f && typeof f === 'object') {
                const j = labels.get(f.skipTo);
                if (!f.skipTo) {
                    out.push(issue('RC15', 'error', i, 'onFail', 'A skip needs the label of the step to skip to.', libraryWords('skipEmpty')));
                } else if (j === undefined) {
                    out.push(issue('RC15', 'error', i, 'onFail', `No step has the label "${f.skipTo}": give the step to skip to that label.`));
                } else if (j <= i) {
                    out.push(issue('RC15', 'error', i, 'onFail', `"${f.skipTo}" is step ${j + 1}, which is not after this one: a skip `
                        + 'only goes FORWARD.'));
                }
            } else if (f !== undefined && f !== 'carryOn' && f !== 'stop') {
                out.push(issue('RC15', 'error', i, 'onFail', 'If it fails: stop, carry on, or skip to a later step.'));
            }
        }

        switch (s.do) {
            case 'driveTo':
            case 'reverseTo':
            case 'face':
            case 'faceAway': {
                if (!nmb(s.x) || !nmb(s.y)) {
                    out.push(issue('RC1', 'error', i, 'x', 'This step needs a point: an x and a y.'));
                    break;
                }
                if (!p.from) {
                    break;
                }
                const d = dist(p.from.x, p.from.y, s.x, s.y);
                if (d < 1) {
                    out.push(issue('RC1', 'error', i, 'x', `This point is ${d.toFixed(2)} in from where the robot already is: `
                        + 'there is no clear direction to it. Pick a point farther away.', libraryWords('noBearing')));
                    break;
                }
                const drive = s.do === 'driveTo' || s.do === 'reverseTo';
                const turnTo = drive ? p.bearingDeg : p.to.headingDeg;
                // RC1: the turn in place (a face, or a drive's turn-first) and the end of the leg.
                if (!drive || s.faceFirst) {
                    const t = turnOvershoot(p.from.x, p.from.y, p.from.headingDeg, turnTo, field, size);
                    if (t > WALL_SLACK_IN) {
                        out.push(issue('RC1', 'error', i, 'x', `Turning in place here, a corner of the robot goes ${t.toFixed(1)} in into `
                            + `${wallName(p.from.x, p.from.y, field)}. Move the step before it away from the wall first.`));
                    }
                }
                if (drive) {
                    const o = wallOvershoot(s.x, s.y, p.to.headingDeg, field, size);
                    if (o > WALL_SLACK_IN) {
                        out.push(issue('RC1', 'error', i, 'x', `At this point the robot's ${size} in square goes ${o.toFixed(1)} in into `
                            + `${wallName(s.x, s.y, field)}: the robot cannot get there.`));
                    }
                    // RC24 (R6): a tank leg not turned first, off its own line.
                    if (tank && !s.faceFirst && p.offDeg > TURN_FIRST_EDGE_DEG) {
                        const steer = tankNotSteeredWords(p.offDeg, maxSteer);
                        if (p.offDeg > maxSteer) {
                            out.push(issue('RC24', 'error', i, 'faceFirst', `The leg starts ${p.offDeg.toFixed(1)} deg off its line and a `
                                + `tank steers back only within ${fmt(maxSteer)} deg: it will stop short. Turn "turn to face it first" on.`,
                                steer, { fix: { kind: 'faceFirst', stepIndex: i } }));
                        } else {
                            out.push(issue('RC24', 'warn', i, 'faceFirst', `The leg starts ${p.offDeg.toFixed(1)} deg off its line: it may stop `
                                + 'short of the point. "Turn to face it first" makes it straight.', '', { fix: { kind: 'faceFirst', stepIndex: i } }));
                        }
                    }
                }
                break;
            }
            case 'turnTo':
                if (!nmb(s.headingDeg)) {
                    out.push(issue('RC19', 'error', i, 'headingDeg', 'A turn needs a heading (degrees).'));
                } else if (p.from) {
                    const t = turnOvershoot(p.from.x, p.from.y, p.from.headingDeg, s.headingDeg, field, size);
                    if (t > WALL_SLACK_IN) {
                        out.push(issue('RC1', 'error', i, 'headingDeg', `Turning in place here, a corner of the robot goes ${t.toFixed(1)} in `
                            + `into ${wallName(p.from.x, p.from.y, field)}.`));
                    }
                }
                break;
            case 'curveTo':
            case 'reverseCurveTo': {
                const pts = s.points || [];
                if (pts.length < 1 || pts.length > kMaxCurvePoints) {
                    out.push(issue('RC26', 'error', i, 'points', `A curve takes 1 to ${kMaxCurvePoints} points (a longer path is two curves).`));
                    break;
                }
                if (!pts.every((q) => nmb(q.x) && nmb(q.y))) {
                    out.push(issue('RC26', 'error', i, 'points', 'Every curve point needs an x and a y.'));
                    break;
                }
                if (!p.curve) {
                    break;
                }
                // RC26 (R6): checkCurve's rules, from the planned start pose, in the library's words.
                const c = checkCurve(p.curve, curveCfg);
                const lib = curveCheckWords(c, curveCfg);
                if (c.verdict === CurveVerdict.StartBehind) {
                    const back = s.do === 'reverseCurveTo';
                    out.push(issue('RC26', 'error', i, 'points', back
                        ? 'The first point is behind the robot\'s back by more than 90 deg: it will not start. Turn its back to the first point first.'
                        : 'The first point is more than 90 deg off where the robot faces: it will not start. Turn to face the first point first.',
                    lib, { fix: { kind: 'faceCurveStart', stepIndex: i, x: pts[0].x, y: pts[0].y, away: back } }));
                } else if (c.verdict === CurveVerdict.Invalid) {
                    out.push(issue('RC26', 'error', i, 'points', `The library will not drive this curve (point ${c.point}). `
                        + 'Move that point, or give it a heading.', lib, { point: c.point }));
                } else if (c.verdict === CurveVerdict.TightBend) {
                    out.push(issue('RC26', 'warn', i, 'points', `Its tightest bend is ${c.tightestRadius.toFixed(1)} in across: the robot `
                        + 'will cut that corner.', lib));
                }
                // RC2 (R6): the SAMPLED library curve, not the polyline — a bend can bulge past a wall.
                for (const q of p.curve.samples(0.5)) {
                    const o = wallOvershoot(q.x, q.y, q.headingDeg, field, size);
                    if (o > WALL_SLACK_IN) {
                        out.push(issue('RC2', 'error', i, 'points', `Along this curve, near (${q.x.toFixed(1)}, ${q.y.toFixed(1)}), the robot's `
                            + `square goes ${o.toFixed(1)} in into ${wallName(q.x, q.y, field)}.`));
                        break;
                    }
                }
                break;
            }
            case 'wait':
                if (!nmb(s.seconds) || s.seconds <= 0) {
                    out.push(issue('RC19', 'error', i, 'seconds', 'A wait needs a time above 0 s.', libraryWords('guardPause')));
                }
                break;
            default:
                if (!STEP_KINDS.includes(s.do)) {
                    out.push(issue('RC19', 'error', i, 'do', `"${s.do}" is not a step this route builder writes: delete it, or open `
                        + 'the project in the newest route builder.'));
                }
                break;
        }
        if (MECHANISM_KINDS.has(s.do)) {
            const boundKey = { move: 'moveBound', startMove: 'moveBound', home: 'homeBound', startHome: 'homeBound',
                               waitUntilDone: 'waitBound' }[s.do];
            if (boundKey && (!nmb(s.boundS) || s.boundS <= 0)) {
                out.push(issue('RC19', 'error', i, 'boundS', 'Type the most time this may take (seconds, above 0).',
                               libraryWords(boundKey), { empty: s.boundS === undefined || Number.isNaN(s.boundS) }));
            }
            if (s.do === 'run' || s.do === 'startRun') {
                if (!nmb(s.forS) || s.forS <= 0) {
                    out.push(issue('RC19', 'error', i, 'forS', 'Type how long it runs (seconds, above 0).', libraryWords('runBound')));
                }
                if (!nmb(s.volts)) {
                    out.push(issue('RC19', 'error', i, 'volts', 'Type the volts (a number).', libraryWords('runVolts')));
                } else if (Math.abs(s.volts) > 12) {
                    out.push(issue('RC19', 'error', i, 'volts', 'A V5 motor takes -12 to 12 V.'));
                }
            }
            // RC18 / RC20: on EVERY robot that builds the route.
            for (const robot of robots) {
                const rname = robot.name || `robot ${robot.id}`;
                const mech = mechOf(robot, s.mechanism);
                if (!mech) {
                    out.push(issue('RC18', 'error', i, 'mechanism', `${cap(rname)} has no mechanism called ${s.mechanism}.`, '',
                                   { robot: robot.id }));
                    continue;
                }
                const takes = STEP_TAKES[s.do];
                if (takes && !takes.includes(mech.kind)) {
                    out.push(issue('RC18', 'error', i, 'mechanism', `${cap(rname)}'s ${mech.name} is ${KIND_WORDS[mech.kind] || mech.kind}: `
                        + 'this step does not fit it.', '', { robot: robot.id }));
                    continue;
                }
                if ((s.do === 'move' || s.do === 'startMove') && s.preset === undefined) {
                    out.push(issue('RC18', 'error', i, 'preset', 'Choose which level it moves to.', '', { robot: robot.id }));
                } else if (s.do === 'move' || s.do === 'startMove') {
                    const ok = typeof s.preset === 'number'
                        ? (mech.presets || []).some((q) => q.ledger === s.preset)
                        : (mech.presets || []).some((q) => q.name === s.preset && q.ledger === undefined);
                    if (!ok) {
                        out.push(issue('RC18', 'error', i, 'preset', `${cap(rname)}'s ${mech.name} has no ${typeof s.preset === 'number'
                            ? `level ${s.preset} in its facts` : `preset "${s.preset}"`}.`, libraryWords('noPreset'), { robot: robot.id }));
                    }
                }
                if ((s.do === 'move' || s.do === 'startMove') && mech.homing === 'never') {
                    out.push(issue('RC25', 'error', i, 'mechanism', `${cap(rname)}'s ${mech.name} is never homed, so it can never move `
                        + 'to a level (it only jogs).', '', { robot: robot.id }));
                }
                if (s.do === 'set' && s.position === undefined) {
                    out.push(issue('RC18', 'error', i, 'position', 'Choose which position it goes to.', '', { robot: robot.id }));
                } else if (s.do === 'set' && !(mech.positions || []).includes(s.position)) {
                    out.push(issue('RC18', 'error', i, 'position', `${cap(rname)}'s ${mech.name} has no position "${s.position}".`,
                                   libraryWords('noPosition'), { robot: robot.id }));
                }
                if ((s.do === 'home' || s.do === 'startHome') && mech.homing === 'never') {
                    out.push(issue('RC20', 'error', i, 'mechanism', `${cap(rname)}'s ${mech.name} is declared never homed: it has no `
                        + 'way to find its zero.', libraryWords('homeNever'), { robot: robot.id }));
                }
            }
        }
    });

    // The path rules, per robot (R13): RC17 (a wait on a mechanism nothing started) and RC25 (the home rule).
    for (const robot of robots) {
        const arrive = walkPaths(route, robot);
        const rname = robot.name || `robot ${robot.id}`;
        route.steps.forEach((s, i) => {
            if (s.do === 'waitUntilDone') {
                const states = arrive[i];
                if (states.length && states.every((st) => !st.started[s.mechanism])) {
                    out.push(issue('RC17', 'error', i, 'mechanism', `Nothing before this step starts the ${s.mechanism} on ${rname}: `
                        + 'the library stops the auton with an error here. Start it first.', libraryWords('waitNeverStarted'),
                    { robot: robot.id }));
                } else {
                    const missed = states.find((st) => !st.started[s.mechanism]);
                    if (missed) {
                        out.push(issue('RC17', 'error', i, 'mechanism', `On ${rname}, if ${missed.via}, the step that starts the `
                            + `${s.mechanism} is passed over and this wait fails. Start it where every path runs it.`, '',
                        { robot: robot.id }));
                    }
                }
            }
            if (s.do === 'move' || s.do === 'startMove') {
                const mech = mechOf(robot, s.mechanism);
                if (!mech || !(mech.homing === 'stall' || mech.homing === 'switch')) {
                    return;
                }
                const states = arrive[i];
                const notHomed = states.find((st) => st.homed[s.mechanism] === 'no');
                const running = states.find((st) => st.homed[s.mechanism] === 'running');
                const failed = states.find((st) => st.homed[s.mechanism] === 'failed');
                const how = mech.homing === 'stall' ? 'by pushing into its stop until it stalls' : 'by its switch';
                if (running) {
                    out.push(issue('RC25', 'error', i, 'mechanism', `The ${mech.name} is still homing here: a move now REPLACES the home and `
                        + `stops the route (NOT_HOMED). Put a "wait until done" on the ${mech.name} before this move.`, '',
                    { robot: robot.id, fix: { kind: 'waitForHome', stepIndex: i, mechanism: s.mechanism } }));
                } else if (notHomed) {
                    const all = states.every((st) => st.homed[s.mechanism] === 'no');
                    out.push(issue('RC25', 'error', i, 'mechanism', `The ${mech.name} finds its zero only when it is HOMED (${how}); `
                        + `${all ? 'nothing before this step homes it' : `on ${rname}, if ${notHomed.via}, the home is passed over`}, so `
                        + 'this move stops the route (NOT_HOMED). Home it first.', '',
                    { robot: robot.id, fix: { kind: 'homeFirst', stepIndex: i, mechanism: s.mechanism },
                      alsoFix: { kind: 'homeWhileDriving', stepIndex: i, mechanism: s.mechanism } }));
                } else if (failed) {
                    out.push(issue('RC25', 'warn', i, 'mechanism', `If the ${mech.name}'s home fails (${failed.via}), this move runs on an `
                        + 'un-homed mechanism and stops the route (NOT_HOMED).', '', { robot: robot.id }));
                }
            }
        });
    }

    // The same issue on two robots reads once (it names the first robot it was found on).
    const seen = new Set();
    return out.filter((x) => {
        const k = `${x.code}|${x.level}|${x.stepIndex}|${x.field}|${x.words}`;
        if (seen.has(k)) {
            return false;
        }
        seen.add(k);
        return true;
    });
}

function cap(s) {
    return s.length ? s[0].toUpperCase() + s.slice(1) : s;
}

/** The first error, if any (while there is one, Export lists the errors instead of the code — §1.4). */
export function firstError(issues) {
    return issues.find((x) => x.level === 'error') || null;
}

/**
 * A one-click fix applied: returns the route with the step the fix adds, which the student sees and can
 * remove. A home's time limit is left EMPTY (NaN) for the student to type — the page never invents one.
 */
export function applyFix(route, fix, addStep, updateStep) {
    switch (fix.kind) {
        case 'faceFirst':
            return updateStep(route, fix.stepIndex, { faceFirst: true });
        case 'homeFirst':
            return addStep(route, { do: 'home', mechanism: fix.mechanism, boundS: undefined }, 0);
        case 'homeWhileDriving': {
            const r = addStep(route, { do: 'startHome', mechanism: fix.mechanism, boundS: undefined }, 0);
            return addStep(r, { do: 'waitUntilDone', mechanism: fix.mechanism, boundS: undefined }, fix.stepIndex + 1);
        }
        case 'waitForHome':
            return addStep(route, { do: 'waitUntilDone', mechanism: fix.mechanism, boundS: undefined }, fix.stepIndex);
        case 'faceCurveStart':
            return addStep(route, { do: fix.away ? 'faceAway' : 'face', x: fix.x, y: fix.y }, fix.stepIndex);
        case 'nameSkills':
            return { ...route, name: ROUTE_NAME };
        default:
            return route;
    }
}
