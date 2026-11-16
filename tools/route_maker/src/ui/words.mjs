// words.mjs — what the page CALLS each step, in plain words (RMP1 brief §2.5.3: the words a non-coder reads —
// no "Routine", no "precondition"). One place, so the step list, the inspector and the help say the same thing.

import { fmt } from '../model/numbers.mjs';

/** The toolbar's tools and the step kinds they add. */
export const TOOLS = [
    { id: 'select', label: 'Select', key: 'V', icon: 'select', help: 'Select and drag: the start, any point, any step' },
    { id: 'driveTo', label: 'Drive', key: 'D', icon: 'drive', help: 'Drive: click where the robot drives to' },
    { id: 'reverseTo', label: 'Back up', key: 'B', icon: 'backup', help: 'Back up: click where the robot backs up to' },
    { id: 'face', label: 'Turn', key: 'T', icon: 'turn', help: 'Turn: click the point the robot turns to face (a heading can be typed after)' },
    { id: 'curveTo', label: 'Curve', key: 'C', icon: 'curve', help: 'Curve: click 1 to 16 points, then Enter or double-click to finish' },
    { id: 'mechanism', label: 'Mechanism', key: 'M', icon: 'mechanism', help: 'Mechanism: pick one of the robot\'s mechanisms and what it does' },
    { id: 'wait', label: 'Wait', key: 'P', icon: 'wait', help: 'Wait: a pause in seconds (the 57 s cut still ends it)' },
];

/** A step's icon. */
export function stepIcon(step) {
    return {
        driveTo: 'drive', reverseTo: 'backup', turnTo: 'turn', face: 'face', faceAway: 'face', curveTo: 'curve',
        reverseCurveTo: 'curve', move: 'mechanism', startMove: 'mechanism', set: 'mechanism', run: 'mechanism',
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

/** [title, subtitle] of the start for the list. A part with no number (a hand-edited file's `null`) shows as "?":
 *  the start is kept as the file has it, and RC3 asks for the part. */
export function startTitle(start) {
    if (!start) {
        return ['Start: not placed yet', 'click the field to place robot one'];
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
        case 'move': return [`${m}: to ${presetWords(robot, step)}`, `waits until there · at most ${n(step.boundS, ' s')}`];
        case 'startMove': return [`${m}: start toward ${presetWords(robot, step)}`, `keeps going · at most ${n(step.boundS, ' s')}`];
        case 'set': return [`${m}: set "${step.position ?? '?'}"`, 'pneumatic'];
        case 'run': return [`${m}: run ${n(step.volts, ' V')}`, `for ${n(step.forS, ' s')}, then stops`];
        case 'startRun': return [`${m}: start ${n(step.volts, ' V')}`, `for ${n(step.forS, ' s')} · keeps going`];
        case 'home': return [`${m}: home it`, `finds its zero · at most ${n(step.boundS, ' s')}`];
        case 'startHome': return [`${m}: start homing`, `keeps going · at most ${n(step.boundS, ' s')}`];
        case 'waitUntilDone': return [`Wait until the ${m} is done`, `at most ${n(step.boundS, ' s')} more`];
        case 'wait': return [`Wait ${n(step.seconds, ' s')}`, 'the 57 s cut still ends it'];
        default: return [step.do, 'a step this route builder does not know'];
    }
}

/** What a mechanism of each kind can do in A1 (D2's mechanism row). [action, words, the steps it adds]. */
export function mechanismActions(kind) {
    switch (kind) {
        case 'arm':
        case 'lift':
            return [
                { id: 'move', words: 'Move to a level', do: 'move' },
                { id: 'home', words: 'Home it (find its zero)', do: 'home' },
                { id: 'wait', words: 'Wait until it is done', do: 'waitUntilDone' },
            ];
        case 'pneumatic':
            return [{ id: 'set', words: 'Set a position', do: 'set' }];
        case 'power':
            return [
                { id: 'run', words: 'Run it for a time', do: 'run' },
                { id: 'wait', words: 'Wait until it is done', do: 'waitUntilDone' },
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
