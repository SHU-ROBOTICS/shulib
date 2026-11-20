// to_robot.mjs — AFTER EXPORT (RMP1-F): what was exported, and the commands that put it on a robot. Pure.

/** The program that runs a route on the bench: its slot and name on the brain (the Makefile; guide 16). */
export const TESTER = Object.freeze({ slot: 6, name: 'shulib Auton Tester' });

/** Robot `which`'s ('one' | 'two') commands: build, upload, and tools/upload_auton.py (`python` on Windows). */
export function robotCommands(which, { windows = false } = {}) {
    const two = which === 'two';
    return {
        build: `make ROBOT=${two ? 'tank' : 'one'} PROGRAM=autontest`,
        upload: `pros upload --slot ${TESTER.slot} --name "${TESTER.name}"`,
        helper: `${windows ? 'python' : 'python3'} tools/upload_auton.py ${two ? 'two' : 'one'}`,
    };
}

/** What was exported: the routes' names and their steps, and "2 routes (skills, left 1), 5 steps". */
export function exportSummary(routes) {
    const names = routes.map((r) => r.name);
    const steps = routes.reduce((n, r) => n + r.steps.length, 0);
    const count = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;
    return { names, steps, words: `${count(names.length, 'route')} (${names.join(', ')}), ${count(steps, 'step')}` };
}

/**
 * One line of tools/upload_auton.py (Send to robot, RMP1-I) said plainly: {kind, words, then} — 'step' (Building…,
 * Uploading…), 'built', 'uploaded' (`then`: where it is on the brain), 'failed' (`words` its title, `then` the rest),
 * 'detail' (an indented line under a failure), 'log' (where everything was printed: the panel's fold) — or null (blank).
 */
export function sendStage(line) {
    const t = line.replace(/\s+$/, '');
    let m;
    if (!t) {
        return null;
    }
    if ((m = /^Building (robot \w+)'s Auton Tester/.exec(t))) {
        return { kind: 'step', words: `Building ${m[1]}'s Auton Tester… (a few minutes the first time)` };
    }
    if (/^✓ BUILT\b/.test(t)) {
        return { kind: 'built', words: 'BUILT' };
    }
    if ((m = /^✓ (UPLOADED.*?)\.\s+(On the brain: .*)$/.exec(t))) {
        return { kind: 'uploaded', words: m[1], then: m[2] };
    }
    if ((m = /^✗ (.*?)(?:[:.](?:\s+|$)(.*))?$/.exec(t))) {
        return { kind: 'failed', words: m[1], then: m[2] || '' };
    }
    if (/^\s+\(everything (make|pros) printed: /.test(t)) {
        return { kind: 'log' };
    }
    return /^\s/.test(t) ? { kind: 'detail', words: t.trim() } : { kind: 'step', words: t.replace(/\.\.\.$/, '…') };
}
