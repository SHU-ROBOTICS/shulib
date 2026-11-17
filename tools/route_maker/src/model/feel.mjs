// feel.mjs — DRIVING FEEL (RMP1-B): the library's stick shaping, copied operation for operation from
// include/shulib/teleop/input_shaping.hpp (FE1 pins it bit for bit against the C++), the feel's checks, and the
// driver file's `feel` region (THE FEEL heading, its comment, kFeel and kLoop's two team lines) read and written.
// The project keeps it as G3a's `driving` block (D14). Pure.

import { fmt, round3 } from './numbers.mjs';

/** The library's defaults (input_shaping.hpp kDefaultDeadband / kDefaultExpo; slew off). */
export const DEFAULT_DEADBAND = 0.05;
export const DEFAULT_CURVE = 0;

/** continuousDeadband, as the C++ (same comparisons, same order). */
export function continuousDeadband(axis, deadband) {
    if (!(deadband > 0.0 && deadband < 1.0)) {
        return axis;
    }
    const magnitude = axis < 0.0 ? -axis : axis;
    if (magnitude <= deadband) {
        return 0.0;
    }
    const scaled = (magnitude - deadband) / (1.0 - deadband);
    return axis < 0.0 ? -scaled : scaled;
}

/** expoCurve, as the C++: axis * (expo*axis^2 + (1 - expo)). */
export function expoCurve(axis, expo) {
    if (expo === 0.0) {
        return axis;
    }
    return axis * (expo * axis * axis + (1.0 - expo));
}

/** shapeAxis: the dead zone first, then the curve. */
export function shapeAxis(axis, { deadband, expo }) {
    return expoCurve(continuousDeadband(axis, deadband), expo);
}

/** AxisShaping::valid(): deadband in [0, 1), expo in [0, 1] (a NaN fails). */
export function axisValid({ deadband, expo }) {
    return deadband >= 0.0 && deadband < 1.0 && expo >= 0.0 && expo <= 1.0;
}

/** Seconds from rest to full speed at `ratePerSecond` (the slew); null when the ramp is off (<= 0). */
export function rampSeconds(ratePerSecond) {
    return ratePerSecond > 0 ? 1 / ratePerSecond : null;
}

/** The ramp's readout. The library's slew limits BOTH ways (slew_limiter.hpp): a stop takes as long. */
export function rampWords(ratePerSecond) {
    const s = rampSeconds(ratePerSecond);
    return s === null ? 'no ramp: the robot gets what the stick asks at once'
        : `full speed from rest in ${fmt(s)} s, and a stop from full speed takes as long`;
}

export const MODES = ['arcade', 'tankSticks', 'arcadeField'];
const MODE_CPP = { arcade: 'ArcadeBody', arcadeField: 'ArcadeField', tankSticks: 'TankSticks' };
const RELEASE_CPP = { coast: 'Coast', brake: 'Brake', hold: 'Hold' };
export const AXES = [['forward', 'forward'], ['strafe', 'left'], ['turn', 'yaw']];  // [driving key, C++ field]

/** One axis's settings, the library's default where the driving block states none. */
export function axisOf(driving, axis) {
    const a = (driving && driving[axis]) || {};
    return { deadband: a.deadband ?? DEFAULT_DEADBAND, expo: a.curve ?? DEFAULT_CURVE };
}

/** The drive mode ('arcade' when unstated). */
export const modeOf = (driving) => (driving && driving.mode) || 'arcade';

/** The drive program's own refusal while its boot line names the default feel (drive_program.cpp's static_assert). */
export const DEFAULT_ONLY_REFUSAL = 'the driver line says deadband 0.05, no expo, no slew';

/** True when the feel is the library's default on every axis, with no ramp. */
export function isDefaultFeel(driving) {
    return AXES.every(([axis]) => {
        const a = axisOf(driving, axis);
        return a.deadband === DEFAULT_DEADBAND && a.expo === DEFAULT_CURVE;
    }) && !(driving && driving.rampPerSecond > 0);
}

/**
 * What the feel's checks find: [{level, field, words, refusal?}]. The C++ valid() rule, the ramp's (stricter) one,
 * and — `defaultOnly`, while the robot's programs can take only the default (the page is built with that fact) —
 * the drive program's refusal of any other feel.
 */
export function checkFeel(driving, { holonomic = false, defaultOnly = false } = {}) {
    const out = [];
    if (defaultOnly && !isDefaultFeel(driving)) {
        out.push({ level: 'error', field: 'defaultOnly', refusal: DEFAULT_ONLY_REFUSAL,
                   words: 'The robot\'s programs cannot take a feel other than the library\'s default yet: "shulib Drive" '
                        + 'names the default feel when it starts, and the build stops on any other. Keep the default '
                        + '("Use the library\'s defaults") until that is updated; the buttons export as usual.' });
    }
    for (const [axis] of AXES) {
        const a = driving && driving[axis];
        if (!a) {
            continue;
        }
        if ('deadband' in a && !(a.deadband >= 0 && a.deadband < 1)) {
            out.push({ level: 'error', field: `${axis}.deadband`,
                       words: `The ${axis} dead zone must be at least 0 and below 1 (it is ${a.deadband}).` });
        }
        if ('curve' in a && !(a.curve >= 0 && a.curve <= 1)) {
            out.push({ level: 'error', field: `${axis}.curve`,
                       words: `The ${axis} curve must be from 0 to 1 (it is ${a.curve}): outside it, a full push is no longer full speed.` });
        }
    }
    if (driving && 'rampPerSecond' in driving && !(driving.rampPerSecond >= 0 && Number.isFinite(driving.rampPerSecond))) {
        out.push({ level: 'error', field: 'rampPerSecond', words: 'The ramp must be 0 (off) or a positive number per second.' });
    }
    if (driving && driving.mode && !MODES.includes(driving.mode)) {
        out.push({ level: 'error', field: 'mode', words: `"${driving.mode}" is not a drive mode.` });
    }
    if (driving && driving.strafe && !holonomic) {
        out.push({ level: 'warn', field: 'strafe', words: 'A tank cannot strafe: the strafe settings change nothing.' });
    }
    return out;
}

// ── the feel region ────────────────────────────────────────────────────────────────────────────────
export const FEEL_HEADING = '// ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════════';
const REZERO_WORDS = 'field-centric\'s "away from me"';
const RELEASE_WORDS = { coast: 'let go of the sticks: the drive coasts', brake: 'let go of the sticks: the drive brakes',
                        hold: 'let go of the sticks: the drive holds where it is' };

/** The feel's comment in plain words, made from the values (written whenever the feel changes). */
export function feelComment(driving) {
    const mode = modeOf(driving);
    const head = { arcade: 'Arcade: the left stick drives, the right stick turns.',
                   tankSticks: 'Tank: the left stick drives the left side, the right stick the right side.',
                   arcadeField: 'Field-centric arcade: the left stick pushes the robot that way on the field, the right stick turns.' }[mode];
    const axis = (k, name) => {
        const a = axisOf(driving, k);
        const db = a.deadband === DEFAULT_DEADBAND ? `dead zone ${fmt(a.deadband)} (the library's)` : `dead zone ${fmt(a.deadband)}`;
        return `${name}: ${db}, ${a.expo === 0 ? 'no curve' : `curve ${fmt(a.expo)}`}.`;
    };
    const parts = [head, axis('forward', 'Forward'), axis('turn', 'Turn')];
    if (driving && driving.strafe) {
        parts.push(axis('strafe', 'Strafe'));
    }
    const ramp = driving && driving.rampPerSecond > 0 ? `A ramp: ${rampWords(driving.rampPerSecond)}.` : 'No ramp.';
    parts.push(ramp, mode === 'arcadeField' ? 'Field-centric is ON.' : 'Field-centric is OFF.');
    const axes = AXES.filter(([k]) => k !== 'strafe' || (driving && driving.strafe)).map(([k]) => axisOf(driving, k));
    if (axes.some((a) => a.deadband === DEFAULT_DEADBAND && a.expo === DEFAULT_CURVE)) {
        parts.push('The library\'s numbers here are ones no driver has felt yet (HA-112).');
    }
    return wrap(parts.join(' '), 105);
}

/** Plain words wrapped at `width` columns. */
export function wrap(text, width) {
    const out = [];
    let line = '';
    for (const w of text.split(/\s+/).filter(Boolean)) {
        if (line && line.length + 1 + w.length > width) {
            out.push(line);
            line = w;
        } else {
            line = line ? `${line} ${w}` : w;
        }
    }
    if (line) {
        out.push(line);
    }
    return out;
}

/** The kFeel statement: only the stated fields, in declaration order (forward, left, yaw, slew, mode). */
export function kFeelText(driving) {
    const fields = [];
    for (const [axis, cpp] of AXES) {
        const a = driving && driving[axis];
        if (a && ('deadband' in a || 'curve' in a)) {
            const inner = [];
            if ('deadband' in a) {
                inner.push(`.deadband = ${fmt(a.deadband)}`);
            }
            if ('curve' in a) {
                inner.push(`.expo = ${fmt(a.curve)}`);
            }
            fields.push(`.${cpp} = {${inner.join(', ')}}`);
        }
    }
    if (driving && 'rampPerSecond' in driving) {
        fields.push(`.slew = {.ratePerSecond = ${fmt(driving.rampPerSecond)}}`);
    }
    fields.push(`.mode = ${MODE_CPP[modeOf(driving)]}`);
    const one = `inline constexpr DriverProfile kFeel{${fields.join(', ')}};`;
    if (one.length <= 110) {
        return [one];
    }
    return ['inline constexpr DriverProfile kFeel{', ...fields.map((f) => `    ${f},`), '};'];
}

/** The feel region's lines (between its markers). */
export function exportFeel(driving) {
    const d = driving || {};
    const rezero = d.rezeroButton || 'Y';
    const release = d.onRelease || 'coast';
    const words = d.words || {};
    const comment = words.comment || feelComment(d);
    const pad = (s) => s.padEnd(35);
    return [
        FEEL_HEADING,
        ...comment.map((l) => (l ? `// ${l}` : '//')),
        ...kFeelText(d),
        'inline constexpr DriverLoopConfig kLoop{',
        `${pad(`    .rezeroButton = Button::${rezero},`)}// ${words.rezero || REZERO_WORDS}`,
        `${pad(`    .releaseBrakeMode = ${RELEASE_CPP[release]},`)}// ${words.release || RELEASE_WORDS[release]}`,
        '};',
    ];
}

// ── reading a feel region back ─────────────────────────────────────────────────────────────────────
const NUM = '(-?\\d+(?:\\.\\d+)?)';
const invert = (o) => Object.fromEntries(Object.entries(o).map(([k, v]) => [v, k]));

/** A one-line kFeel's fields, split at the commas OUTSIDE braces (an axis holds its own commas). */
function topFields(s) {
    const out = [];
    let depth = 0;
    let from = 0;
    for (let k = 0; k < s.length; ++k) {
        depth += s[k] === '{' ? 1 : s[k] === '}' ? -1 : 0;
        if (s[k] === ',' && depth === 0) {
            out.push(s.slice(from, k));
            from = k + 1;
        }
    }
    return [...out, s.slice(from)].filter((x) => x.trim());
}

/**
 * The `driving` block a feel region says: {ok, driving} or {ok: false, line, words}. Reads exactly the
 * forms exportFeel writes (one-line or block kFeel); anything else is a hand edit the page will not guess at.
 */
export function parseFeel(lines) {
    const bad = (i, why) => ({ ok: false, line: i + 1, words: `line ${i + 1} of THE FEEL: ${why}` });
    let i = 0;
    if (lines[i] !== FEEL_HEADING) {
        return bad(i, 'the heading "═══ THE FEEL ═══" was expected');
    }
    ++i;
    const comment = [];
    while (i < lines.length && /^\/\/( |$)/.test(lines[i])) {
        comment.push(lines[i].slice(3));
        ++i;
    }
    const driving = {};
    let body;
    if (/^inline constexpr DriverProfile kFeel\{.*\};$/.test(lines[i] || '')) {
        body = topFields(lines[i].slice('inline constexpr DriverProfile kFeel{'.length, -2));
        ++i;
    } else if (lines[i] === 'inline constexpr DriverProfile kFeel{') {
        body = [];
        for (++i; i < lines.length && lines[i] !== '};'; ++i) {
            const m = /^ {4}(.*),$/.exec(lines[i]);
            if (!m) {
                return bad(i, 'a kFeel line the page cannot read');
            }
            body.push(m[1]);
        }
        ++i;
    } else {
        return bad(i, 'kFeel was expected');
    }
    const cppAxis = Object.fromEntries(AXES.map(([k, c]) => [c, k]));
    for (const f of body.map((s) => s.trim())) {
        let m;
        if ((m = /^\.(forward|left|yaw) = \{(.*)\}$/.exec(f))) {
            const a = {};
            for (const part of m[2].split(/,\s*/)) {
                const p = new RegExp(`^\\.(deadband|expo) = ${NUM}$`).exec(part);
                if (!p) {
                    return bad(i - 1, `"${part}" is not a dead zone or a curve`);
                }
                a[p[1] === 'expo' ? 'curve' : 'deadband'] = round3(Number(p[2]));
            }
            driving[cppAxis[m[1]]] = a;
        } else if ((m = new RegExp(`^\\.slew = \\{\\.ratePerSecond = ${NUM}\\}$`).exec(f))) {
            driving.rampPerSecond = round3(Number(m[1]));
        } else if ((m = /^\.mode = (\w+)$/.exec(f)) && invert(MODE_CPP)[m[1]]) {
            driving.mode = invert(MODE_CPP)[m[1]];
        } else {
            return bad(i - 1, `"${f}" is not a feel field the page writes`);
        }
    }
    if (!driving.mode) {
        return bad(i - 1, 'kFeel states no .mode');
    }
    const loop = [
        /^inline constexpr DriverLoopConfig kLoop\{$/,
        /^ {4}\.rezeroButton = Button::(\w+), +\/\/ (.*)$/,
        /^ {4}\.releaseBrakeMode = (\w+), +\/\/ (.*)$/,
        /^\};$/,
    ];
    const got = [];
    for (const re of loop) {
        const m = re.exec(lines[i] || '');
        if (!m) {
            return bad(i, 'kLoop is not the two lines the page writes (the re-zero button, then what the drive does on release)');
        }
        got.push(m);
        ++i;
    }
    if (i !== lines.length) {
        return bad(i, 'something after kLoop');
    }
    const release = invert(RELEASE_CPP)[got[2][1]];
    if (!release) {
        return bad(i - 2, `"${got[2][1]}" is not Coast, Brake or Hold`);
    }
    driving.rezeroButton = got[1][1];
    driving.onRelease = release;
    driving.words = { comment, rezero: got[1][2], release: got[2][2] };
    // the text read back is kept only while it is still what exportFeel would write from it
    const again = exportFeel(driving);
    if (again.join('\n') !== lines.join('\n')) {
        return bad(0, 'its spacing is not what the page writes (a hand edit)');
    }
    return { ok: true, driving };
}

/** A `driving` block from a project file, every number rounded as the export prints it (W7): the page judges what
 *  the file will say, never a 0.9996 the export writes as 1. */
export function drivingFromSaved(d) {
    const out = { ...d };
    for (const [k] of AXES) {
        if (d[k] && typeof d[k] === 'object') {
            out[k] = Object.fromEntries(Object.entries(d[k]).map(([n, v]) => [n, typeof v === 'number' ? round3(v) : v]));
        }
    }
    if (typeof d.rampPerSecond === 'number') {
        out.rampPerSecond = round3(d.rampPerSecond);
    }
    return out;
}

/** The feel's numbers and mode — what its comment describes. */
const said = (d) => JSON.stringify([modeOf(d), ...AXES.map(([k]) => axisOf(d, k)), d.rampPerSecond > 0 ? d.rampPerSecond : 0, !!d.strafe]);

/** `driving` with `patch` applied; a comment whose numbers changed is made again (a stale one is never kept). */
export function editDriving(driving, patch) {
    const next = Object.fromEntries(Object.entries({ ...driving, ...patch }).filter(([, v]) => v !== undefined));
    const words = { ...(driving.words || {}) };
    if (said(next) !== said(driving)) {
        delete words.comment;
    }
    if ('rezeroButton' in patch && patch.rezeroButton !== driving.rezeroButton) {
        delete words.rezero;
    }
    if ('onRelease' in patch && patch.onRelease !== driving.onRelease) {
        delete words.release;
    }
    next.words = words;
    return next;
}
