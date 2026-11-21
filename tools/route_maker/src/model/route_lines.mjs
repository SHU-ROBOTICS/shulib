// route_lines.mjs — the route → THE LINES the preview engine runs (RMP1 brief A.2, §1.1; run RMP1-A2e).
//
// The engine (test/route_maker_runner.hpp, compiled into the WebAssembly module) has no JSON reader, so the
// page hands it the route as text: one line per step, `key=value` words. The runner makes the SAME library
// calls export.mjs writes, so every number here is printed by numbers.mjs's ONE formatter — the text the C++
// file carries, character for character (W7, NF1). A mechanism goes by its declaration's `.name` (the runner
// finds it in the robot's own table; it cannot read the C++ name), and a facts-ledger level by its preset's
// name: both looked up in the robot's description. A value that is a name is %-encoded (no spaces).
//
//   shulib-route 1
//   route robot=one name=skills park=57 stop=59
//   start x=-36 y=-55.7 h=0                  (or: start relative)
//   step do=driveTo x=-12 y=-55.7 face=1 timeout=2.5 label=finish fail=carryOn
//   step do=curveTo p=0,-50 p=12,-36,90 skipTo=finish
//   step do=move mech=cascade preset=L2 bound=2
//   step do=moveTo mech=cascade to=18 bound=2  (RMP1-C D7: up / down / to / the bottom, as the height the export writes)
//
// A "drive forward / back up N in" step (D9) is the driveTo / reverseTo the export writes, its point from the plan.
// Pure: no DOM, no file API. It THROWS on a route it cannot express (the checks refuse those first).

import { fmt } from './numbers.mjs';
import { CARE_MOTION_FIELDS, MOTION_KINDS, plannedPoses } from './route.mjs';
import { plannedHeights, targetKind, moveTarget } from './lift.mjs';
import { robotName } from './robot_names.mjs';

/** The line format's version (the runner refuses another). */
export const LINES_VERSION = 1;

/** Each motion option's key in the lines (MotionOptions' order, as the export writes them). */
const CARE_KEY = { timeoutS: 'timeout', maxSpeedInPerS: 'speed', maxTurnDegPerS: 'turn', toleranceIn: 'tolIn',
                   toleranceDeg: 'tolDeg' };

/** A name as one word: letters, digits and `_ . ~ -` kept, every other byte %XX. */
export function enc(s) {
    return [...new TextEncoder().encode(String(s))]
        .map((b) => (/[A-Za-z0-9_.~-]/.test(String.fromCharCode(b)) ? String.fromCharCode(b)
                                                                       : `%${b.toString(16).toUpperCase().padStart(2, '0')}`))
        .join('');
}

function need(v, what) {
    if (v === undefined || v === null || v === '') {
        throw new Error(`route lines: ${what} is missing`);
    }
    return v;
}

/** The lines for `route` on the robot `robot` describes (its description: robots.json's, or describe's). */
export function routeLines(route, robot) {
    const byIdent = new Map(robot.mechanisms.map((m) => [m.ident, m]));
    const mech = (step) => {
        const m = byIdent.get(step.mechanism);
        if (!m) {
            throw new Error(`route lines: ${robotName(robot.id)} has no mechanism "${step.mechanism}"`);
        }
        return m;
    };
    const preset = (step, m) => {
        if (typeof step.preset !== 'number') {
            return need(step.preset, 'a preset');
        }
        const p = (m.presets || []).find((q) => q.ledger === step.preset);
        if (!p) {
            throw new Error(`route lines: ${m.name} has no level ${step.preset} in its facts ledger`);
        }
        return p.name;
    };
    const out = [`shulib-route ${LINES_VERSION}`,
                 `route robot=${enc(robot.id)} name=${enc(route.name)} park=${fmt(route.endActionAtS)} stop=${fmt(route.hardStopAtS)}`];
    const s0 = route.start;
    out.push(s0 ? `start x=${fmt(s0.x)} y=${fmt(s0.y)} h=${fmt(s0.headingDeg)}` : 'start relative');
    const plan = plannedPoses(route);
    const heights = plannedHeights(route, robot);
    route.steps.forEach((step, i) => {
        const w = [`do=${step.do}`];
        switch (step.do) {
            case 'forward':
            case 'backward': {
                const q = plan[i].point;
                if (!q) {
                    throw new Error('route lines: a drive forward or back from a pose the plan does not know');
                }
                w[0] = `do=${step.do === 'forward' ? 'driveTo' : 'reverseTo'}`;
                w.push(`x=${fmt(q.x)}`, `y=${fmt(q.y)}`, 'face=0');
                break;
            }
            case 'driveTo':
            case 'reverseTo':
                w.push(`x=${fmt(step.x)}`, `y=${fmt(step.y)}`, `face=${step.faceFirst ? 1 : 0}`);
                break;
            case 'face':
            case 'faceAway':
                w.push(`x=${fmt(step.x)}`, `y=${fmt(step.y)}`);
                break;
            case 'turnTo':
                w.push(`h=${fmt(step.headingDeg)}`);
                break;
            case 'curveTo':
            case 'reverseCurveTo':
                for (const p of step.points) {
                    w.push(`p=${fmt(p.x)},${fmt(p.y)}${p.headingDeg !== undefined ? `,${fmt(p.headingDeg)}` : ''}`);
                }
                break;
            case 'move':
            case 'startMove': {
                const m = mech(step);
                if (targetKind(step) === 'preset') {
                    w.push(`mech=${enc(m.name)}`, `preset=${enc(preset(step, m))}`, `bound=${fmt(step.boundS)}`);
                    break;
                }
                const to = moveTarget(step, m, heights[i].before[m.ident] ?? null);
                if (to === null) {
                    throw new Error(`route lines: the ${m.name}'s target is not known`);
                }
                w[0] = `do=${step.do}To`;
                w.push(`mech=${enc(m.name)}`, `to=${fmt(to)}`, `bound=${fmt(step.boundS)}`);
                break;
            }
            case 'set':
                w.push(`mech=${enc(mech(step).name)}`, `pos=${enc(need(step.position, 'a position'))}`);
                break;
            case 'run':
            case 'startRun':
                w.push(`mech=${enc(mech(step).name)}`, `volts=${fmt(step.volts)}`, `for=${fmt(step.forS)}`);
                break;
            case 'home':
            case 'startHome':
            case 'waitUntilDone':
                w.push(`mech=${enc(mech(step).name)}`, `bound=${fmt(step.boundS)}`);
                break;
            case 'wait':
                w.push(`s=${fmt(step.seconds)}`);
                break;
            default:
                throw new Error(`route lines: the engine cannot run a "${step.do}" step`);
        }
        const care = step.care || {};
        for (const f of MOTION_KINDS.has(step.do) ? CARE_MOTION_FIELDS : []) {  // as the export: a motion's only
            if (care[f] !== undefined && care[f] !== 0) {
                w.push(`${CARE_KEY[f]}=${fmt(care[f])}`);
            }
        }
        if (step.label) {
            w.push(`label=${enc(step.label)}`);
        }
        if (care.onFail === 'carryOn') {
            w.push('fail=carryOn');
        } else if (care.onFail && typeof care.onFail === 'object') {
            w.push(`skipTo=${enc(care.onFail.skipTo)}`);
        }
        out.push(`step ${w.join(' ')}`);
    });
    return `${out.join('\n')}\n`;
}
