// words.mjs — what the page CALLS each step, in plain words (RMP1 brief §2.5.3: the words a non-coder reads —
// no "Routine", no "precondition"). One place, so the step list, the inspector and the help say the same thing.

import { fmt } from '../model/numbers.mjs';
import { targetKind, unitOf, bottomOf } from '../model/lift.mjs';
import { robotName } from '../model/robot_names.mjs';

/** The toolbar's tools and the step kinds they add. */
export const TOOLS = [
    { id: 'select', label: 'Select', key: 'V', icon: 'select', help: 'Select and drag: the start, any point, any step' },
    { id: 'driveTo', label: 'Drive', key: 'D', icon: 'drive', help: 'Drive: click where the robot drives to' },
    { id: 'reverseTo', label: 'Back up', key: 'B', icon: 'backup', help: 'Back up: click where the robot backs up to' },
    { id: 'face', label: 'Turn', key: 'T', icon: 'turn', help: 'Turn: click the point the robot turns to face (a heading can be typed after)' },
    { id: 'curveTo', label: 'Curve', key: 'C', icon: 'curve', help: 'Curve: click 1 to 16 points, then Enter or double-click to finish' },
    { id: 'straight', label: 'Straight', key: 'F', icon: 'straight', help: 'Straight: drive forward or back up a number of inches, from where the robot is' },
    { id: 'mechanism', label: 'Mechanism', key: 'M', icon: 'mechanism', help: 'Mechanism: pick one of the robot\'s mechanisms and what it does' },
    { id: 'wait', label: 'Wait', key: 'P', icon: 'wait', help: 'Wait: a pause in seconds (the 57 s cut still ends it)' },
];

/** A step's icon. */
export function stepIcon(step) {
    return {
        driveTo: 'drive', reverseTo: 'backup', turnTo: 'turn', face: 'face', faceAway: 'face', curveTo: 'curve',
        reverseCurveTo: 'curve', forward: 'straight', backward: 'straight', move: 'mechanism', startMove: 'mechanism', set: 'mechanism', run: 'mechanism',
        startRun: 'mechanism', home: 'home', startHome: 'home', waitUntilDone: 'wait', wait: 'wait',
    }[step.do] || 'help';
}

const n = (v, unit) => (Number.isFinite(v) ? `${fmt(v)}${unit}` : `?${unit}`);
const pt = (x, y) => `(${Number.isFinite(x) ? fmt(x) : '?'}, ${Number.isFinite(y) ? fmt(y) : '?'})`;

/** The mechanism's name as the robot calls it (its `.name`), from its C++ name. */
export function mechName(robot, ident) {
    const m = robot && (robot.mechanisms || []).find((x) => x.ident === ident);
    return m ? m.name : ident;
}

/** A preset's words: a facts-ledger level by its index and today's name, a named preset by its name. */
export function presetWords(robot, step) {
    if (step.preset === undefined) {
        return 'a level (choose one)';
    }
    const m = robot && (robot.mechanisms || []).find((x) => x.ident === step.mechanism);
    if (typeof step.preset === 'number') {
        const p = m && (m.presets || []).find((q) => q.ledger === step.preset);
        return p ? `level ${step.preset} "${p.name}"` : `level ${step.preset}`;
    }
    return `"${step.preset}"`;
}

/** Where a move goes, in the page's words: "up 6 in", "to 18 in", "back to the bottom", a level. */
export function targetWords(robot, step) {
    const m = robot && (robot.mechanisms || []).find((x) => x.ident === step.mechanism);
    const u = unitOf(m);
    switch (targetKind(step)) {
        case 'up': return `${m && m.kind === 'arm' ? 'turn' : 'go'} up ${n(step.up, ` ${u}`)}`;
        case 'down': return `${m && m.kind === 'arm' ? 'turn' : 'go'} down ${n(step.down, ` ${u}`)}`;
        case 'to': return `to ${n(step.to, ` ${u}`)}`;
        case 'bottom': return m && m.range ? `back to the bottom (${fmt(bottomOf(m))} ${u})` : 'back to the bottom';
        case 'preset': return `to ${presetWords(robot, step)}`;
        default: return 'where? (choose)';
    }
}

/** A pneumatic position's button: "Open claw", "Close claw", else "Ramp down" (the position's own name). */
export function positionWords(m, position) {
    const verb = { open: 'Open', opened: 'Open', closed: 'Close', close: 'Close' }[position.toLowerCase()];
    return verb ? `${verb} ${m.name}` : `${m.name[0].toUpperCase()}${m.name.slice(1)} ${position}`;
}

/** [title, subtitle] of the start for the list. A part with no number (a hand-edited file's `null`) shows as "?":
 *  the start is kept as the file has it, and RC3 asks for the part. */
export function startTitle(start) {
    if (!start) {
        return ['Start: not placed yet', `click the field to place ${robotName('one')}`];
    }
    return [`Start at ${pt(start.x, start.y)}`, `facing ${n(start.headingDeg, '°')}`];
}

/** [title, subtitle] of a step for the list. */
export function stepTitle(step, robot) {
    const m = step.mechanism !== undefined ? mechName(robot, step.mechanism) : '';
    switch (step.do) {
        case 'driveTo': return [`Drive to ${pt(step.x, step.y)}`, step.faceFirst ? 'turns to face it first' : 'straight on'];
        case 'reverseTo': return [`Back up to ${pt(step.x, step.y)}`, step.faceFirst ? 'turns its back to it first' : 'straight back'];
        case 'turnTo': return [`Turn to ${n(step.headingDeg, '°')}`, 'turn in place'];
        case 'face': return [`Turn to face ${pt(step.x, step.y)}`, 'turn in place'];
        case 'faceAway': return [`Turn its back to ${pt(step.x, step.y)}`, 'turn in place'];
        case 'curveTo':
        case 'reverseCurveTo': {
            const k = (step.points || []).length;
            return [`${step.do === 'reverseCurveTo' ? 'Curve backwards' : 'Curve'} through ${k} point${k === 1 ? '' : 's'}`,
                    k ? `ends at ${pt(step.points[k - 1].x, step.points[k - 1].y)}` : 'no points yet'];
        }
        case 'forward': return [`Drive forward ${n(step.inches, ' in')}`, 'straight on, from where it is'];
        case 'backward': return [`Back up ${n(step.inches, ' in')}`, 'straight back, from where it is'];
        case 'move': return [`${m}: ${targetWords(robot, step)}`, `waits until there · at most ${n(step.boundS, ' s')}`];
        case 'startMove': return [`${m}: start ${targetWords(robot, step).replace(/^to /, 'toward ')}`, `keeps going · at most ${n(step.boundS, ' s')}`];
        case 'set': {
            const mech = robot && (robot.mechanisms || []).find((x) => x.ident === step.mechanism);
            return [mech && step.position ? positionWords(mech, step.position) : `${m}: set "${step.position ?? '?'}"`, `${m} · pneumatic`];
        }
        case 'run': return [`${m}: run ${n(step.volts, ' V')}`, `for ${n(step.forS, ' s')}, then stops`];
        case 'startRun': return [`${m}: start ${n(step.volts, ' V')}`, `for ${n(step.forS, ' s')} · keeps going`];
        case 'home': return [`${m}: home it`, `finds its zero · at most ${n(step.boundS, ' s')}`];
        case 'startHome': return [`${m}: start homing`, `keeps going · at most ${n(step.boundS, ' s')}`];
        case 'waitUntilDone': return [`Wait until the ${m} is done`, `at most ${n(step.boundS, ' s')} more`];
        case 'wait': return [`Wait ${n(step.seconds, ' s')}`, 'the 57 s cut still ends it'];
        default: return [step.do, 'a step this route builder does not know'];
    }
}

/**
 * What the mechanism menu offers for `m` (RMP1-C D7, D8): each a button's words and the step it adds — a number the
 * student types next is left EMPTY (null: the inspector asks for it), never invented.
 */
export function mechanismActions(m, levels = true) {
    const step = (patch) => ({ mechanism: m.ident, ...patch });
    switch (m.kind) {
        case 'arm':
        case 'lift': {
            const lift = m.kind === 'lift';
            const u = lift ? 'in' : 'deg';
            return [
                { words: `${lift ? 'Go' : 'Turn'} up _ ${u}`, icon: 'mechanism', step: step({ do: 'move', up: null }) },
                { words: `${lift ? 'Go' : 'Turn'} down _ ${u}`, icon: 'mechanism', step: step({ do: 'move', down: null }) },
                { words: `Move to _ ${u}`, icon: 'mechanism', step: step({ do: 'move', to: null }) },
                ...(lift ? [{ words: 'Back to the bottom', icon: 'mechanism', step: step({ do: 'move', bottom: true }) }] : []),
                ...(levels ? [{ words: 'Move to a level', icon: 'mechanism', step: step({ do: 'move' }) }] : []),
                { words: lift ? 'Home it (find the bottom stop — do this first)' : 'Home it (find its zero — do this first)', icon: 'home',
                  step: step({ do: 'home' }) },
                { words: 'Wait until it is done', icon: 'wait', step: step({ do: 'waitUntilDone' }) },
            ];
        }
        case 'pneumatic':
            return (m.positions || []).map((p) => ({ words: positionWords(m, p), icon: 'mechanism', step: step({ do: 'set', position: p }) }));
        case 'power':
            return [
                { words: 'Run it for a time', icon: 'mechanism', step: step({ do: 'run' }) },
                { words: 'Wait until it is done', icon: 'wait', step: step({ do: 'waitUntilDone' }) },
            ];
        default:
            return [];
    }
}

/** The words for each kind of mechanism. */
export const KIND_LABEL = { arm: 'arm', lift: 'lift', pneumatic: 'pneumatic', power: 'motor', flywheel: 'flywheel' };

/** The homing method in words. */
export const HOMING_WORDS = {
    stall: 'homes by pushing into its stop until it stalls',
    switch: 'homes by its switch',
    restsAtStop: 'rests on its stop when the program starts',
    absolute: 'knows its angle from an absolute sensor',
    never: 'is never homed (it only jogs)',
};
