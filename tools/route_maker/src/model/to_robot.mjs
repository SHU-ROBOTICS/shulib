// to_robot.mjs — AFTER EXPORT (RMP1-F): what was exported, and the commands that put it on a robot. Pure.

/** The program that runs a route on the bench: its slot and name on the brain (the Makefile; guide 16). */
export const TESTER = Object.freeze({ slot: 2, name: 'shulib Auton Tester' });

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
