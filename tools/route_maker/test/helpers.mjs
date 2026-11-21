// helpers.mjs — shared by the route builder's node tests (not a test file itself).
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

/** The repository root (tools/route_maker/test/ -> ../../..). */
export const REPO = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..', '..');
/** The route builder's goldens. */
export const GOLDENS = path.join(REPO, 'test', 'goldens', 'route_maker');

/** A committed fixture robot's description (x, y or z — made by the C++, kept fresh by the suite). */
export function fixtureRobot(id) {
    return JSON.parse(fs.readFileSync(path.join(GOLDENS, 'robots', `${id}.json`), 'utf8'));
}

/**
 * A robot shaped like robot one (its cascade's levels marked as its facts ledger's), made from robot x's
 * committed description — the tests own every number in it (R8); robot one's own description is generated
 * at build time from its real facts, which change this week.
 */
export function ledgerRobot() {
    const x = fixtureRobot('x');
    return {
        ...x,
        id: 'one',
        name: 'robot one',
        mechanisms: [{ name: 'cascade', ident: 'cascade', kind: 'lift', units: 'in', homing: 'stall', homeAt: 0, range: [0, 40],
                       presets: [{ name: 'floor', value: 0, ledger: 0 }, { name: 'L1', value: 8, ledger: 1 },
                                 { name: 'L2', value: 16, ledger: 2 }] }],
    };
}

/** A tiny seeded generator (mulberry32) for the randomized tests: the same numbers on every run and OS. */
export function seeded(seed) {
    let a = seed >>> 0;
    return () => {
        a = (a + 0x6d2b79f5) >>> 0;
        let t = a;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

/** Every golden route file's name (without .route.json). */
export function goldenNames() {
    return fs.readdirSync(GOLDENS).filter((n) => n.endsWith('.route.json')).map((n) => n.slice(0, -11)).sort();
}
