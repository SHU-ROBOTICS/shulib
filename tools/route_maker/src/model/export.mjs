// export.mjs — the route → THE C++ the students read (RMP1 brief D4 (a); §6.3 THE CONTRACT; run RMP1-A1).
//
// One function, `exportRegion(routes, robot)`, writes the whole marked region of src/team/skills_route.cpp — the
// BEGIN line to the END line, inclusive — as commented `Routine` code: EVERY route of the project (RMP1-C D2), each
// its own steps function and entry, then the ONE list `kTeamRoutes` both robots register (addTeamRoutes, outside
// the region). Every step numbered as the page numbers it and explained in plain words, every number through
// numbers.mjs's ONE formatter with its unit literal (`_in`, `_deg`, `_s`, `_volt`), a level named through the facts
// ledger (`facts::lv(i).name`) so renaming a level never breaks a route. It is DETERMINISTIC: it depends on the
// routes and on robot one's description only where a step needs it (a lift's height to go up FROM, D7; its
// bottom) — no time, no hash, no machine — which is what the hand-edit check (region.mjs) and the goldens
// (test/goldens/route_maker/) rely on. The empty project writes §6.3's empty region byte for byte (GL0).
//
// THE RULES IT KEEPS (each pinned by a test or a golden the C++ suite compiles and runs):
//   * on one step it writes `label`, then `onFail`, then the call — the only order the library accepts
//     (routine.hpp: each binds to the ONE step after it; [R12]);
//   * a drive with "turn to face it first" (R6) is TWO calls, `r.face(x, y)` (`r.faceAway` reversing) then
//     `r.driveTo` — the label before the face (a skip lands on the turn), the step's `onFail` before EACH; the
//     turn takes the robot's own options (the step's speed, time limit and tolerances are the DRIVE's, and the
//     comment says so); and since the brain's log counts the two calls as two steps, every comment from there
//     on says which step number the brain's log gives it (`STOPPED at step N` names the brain's N);
//   * a wait is the run guard's pause inside `r.then(...)` — never `r.pause`, which the 57 s cut cannot end;
//   * a speed cap is `N_in / 1_s` and a turn cap `units::AngularVelocity{N * math::Angle::kPi / 180.0}` with
//     its `// at most N deg/s` comment — NEVER through `_deg`, which wraps (270_deg is -90) [R10];
//   * MotionOptions' fields in declaration order (C++20 designated initializers must be);
//   * no `#include`, no `using` line, nothing outside the markers (the file around the region is the
//     team's, §6.3 item 2 and landmine 10).
// It never writes a route FOR the team (G4): it turns the steps the students placed into code.
//
// Pure: no DOM, no file API.

import { fmt } from './numbers.mjs';
import { plannedPoses, CARE_MOTION_FIELDS, MOTION_KINDS, ROUTE_NAME } from './route.mjs';
import { dist } from './geometry.mjs';
import { plannedHeights, targetKind, moveTarget, unitOf } from './lift.mjs';

/** Every marker line is this many characters (§6.3's BEGIN and END lines are 91). */
const MARKER_WIDTH = 91;
/** The column the entry's trailing comments start at (§6.3's empty region). */
const ENTRY_COMMENT_COLUMN = 54;
/** Where a comment wraps. */
const WRAP = 108;

/** The BEGIN or END marker line for route `name` (§6.3: `// ═══ ROUTE MAKER — route "skills" — BEGIN ═══…`). */
export function markerLine(name, which) {
    const head = `// ═══ ROUTE MAKER — route "${name}" — ${which} `;
    const fill = Math.max(3, MARKER_WIDTH - [...head].length);
    return head + '═'.repeat(fill);
}

/** A C++ string literal's body for a name the checks already limited to printable ASCII without " or \. */
function lit(s) {
    return `"${s}"`;
}

const len = (v) => `${fmt(v)}_in`;
const sec = (v) => `${fmt(v)}_s`;
const deg = (v) => `${fmt(v)}_deg`;
const volt = (v) => `${fmt(v)}_volt`;

/** The motion options a step's care states, in MotionOptions' order — '' when it states none. */
function options(step) {
    const c = step.care || {};
    const parts = [];
    let turnCapNote = '';
    for (const f of CARE_MOTION_FIELDS) {
        const v = c[f];
        if (v === undefined || v === 0) {
            continue;  // 0 or absent: the robot's own (MotionOptions' 0 = the default)
        }
        switch (f) {
            case 'timeoutS':
                parts.push(`.timeout = ${sec(v)}`);
                break;
            case 'maxSpeedInPerS':
                parts.push(`.maxLinearSpeed = ${fmt(v)}_in / 1_s`);
                break;
            case 'maxTurnDegPerS':
                parts.push(`.maxAngularSpeed = units::AngularVelocity{${fmt(v)} * math::Angle::kPi / 180.0}`);
                turnCapNote = `  // at most ${fmt(v)} deg/s`;
                break;
            case 'toleranceIn':
                parts.push(`.positionTolerance = ${len(v)}`);
                break;
            case 'toleranceDeg':
                parts.push(`.headingTolerance = ${deg(v)}`);
                break;
            default:
                break;
        }
    }
    return { text: parts.length ? `{${parts.join(', ')}}` : '', note: turnCapNote };
}

/** `r.<verb>(<args>[, <options>]);` with the turn cap's comment after it. */
function call(verb, args, opt) {
    const all = opt && opt.text ? [...args, opt.text] : args;
    return `r.${verb}(${all.join(', ')});${opt && opt.note ? opt.note : ''}`;
}

/** The goal of a move: `m[facts::lv(i).name]` for a ledger level, `m["name"]` for a named preset. */
function presetGoal(step) {
    return typeof step.preset === 'number' ? `${step.mechanism}[facts::lv(${step.preset}).name]`
                                           : `${step.mechanism}[${lit(step.preset)}]`;
}
function presetWords(step) {
    return typeof step.preset === 'number' ? `level ${step.preset} (facts::lv(${step.preset}) in the robot's facts)`
                                           : `"${step.preset}"`;
}

/** Where a heading points, in field words (0 = toward the right wall, 90 = toward blue). */
export function headingWords(h) {
    const named = { 0: 'toward the right wall', 90: 'toward blue', 180: 'toward the left wall', '-90': 'toward red' };
    return named[String(h)] ? `${fmt(h)} deg (${named[String(h)]})` : `${fmt(h)} deg`;
}

function careWords(step, lead = ', ') {
    const c = step.care || {};
    const w = [];
    if (c.timeoutS) {
        w.push(`at most ${fmt(c.timeoutS)} s`);
    }
    if (c.maxSpeedInPerS) {
        w.push(`at most ${fmt(c.maxSpeedInPerS)} in/s`);
    }
    if (c.maxTurnDegPerS) {
        w.push(`turning at most ${fmt(c.maxTurnDegPerS)} deg/s`);
    }
    if (c.toleranceIn) {
        w.push(`arriving within ${fmt(c.toleranceIn)} in`);
    }
    if (c.toleranceDeg) {
        w.push(`within ${fmt(c.toleranceDeg)} deg`);
    }
    return w.length ? `${lead}${w.join(', ')}` : '';
}

function failWords(step) {
    const f = step.care && step.care.onFail;
    if (f === 'carryOn') {
        return ' If it fails, carry on with the next step.';
    }
    if (f && typeof f === 'object') {
        return ` If it fails, skip to "${f.skipTo}".`;
    }
    return '';
}

function onFailLine(step) {
    const f = step.care && step.care.onFail;
    if (f === 'carryOn') {
        return 'r.onFail(chassis::carryOn);';
    }
    if (f && typeof f === 'object') {
        return `r.onFail(chassis::skipTo(${lit(f.skipTo)}));`;
    }
    return null;
}

/** Word-wrap one comment into `//` lines (continuations indented "//   ", as D4's listing). */
function comment(text, indent) {
    const words = text.split(' ');
    const lines = [];
    let cur = `${indent}//`;
    let first = true;
    for (const w of words) {
        const add = ` ${w}`;
        if ([...cur].length + [...add].length > WRAP && !(first && cur === `${indent}//`)) {
            lines.push(cur);
            cur = `${indent}//  ${add}`;
        } else {
            cur += add;
        }
        first = false;
    }
    lines.push(cur);
    return lines;
}

function pt(x, y) {
    return `(${fmt(x)}, ${fmt(y)}) in`;
}

/** How many steps the brain's log counts for one page step: a turned-first drive is two calls (the turn, the drive). */
export function libraryCalls(step) {
    return (step.do === 'driveTo' || step.do === 'reverseTo') && step.faceFirst ? 2 : 1;
}

/** A move to a number (D7): its goal `m.to(<target>)` and its words, from the height before it. */
function targetMove(step, n, robot, before) {
    const m = robot && (robot.mechanisms || []).find((x) => x.ident === step.mechanism);
    const target = m ? moveTarget(step, m, before) : null;
    if (target === null) {
        throw new Error(`export: step ${n} moves the ${step.mechanism} to a height this route builder does not know`);
    }
    const u = unitOf(m);
    const tk = targetKind(step);
    const at = `${fmt(target)} ${u}`;
    const how = tk === 'up' || tk === 'down' ? `${tk.toUpperCase()} ${fmt(step[tk])} ${u}, to ${at}`
        : tk === 'bottom' ? `BACK TO THE BOTTOM (${at})` : `to ${at}`;
    return { goal: `${step.mechanism}.to(${u === 'deg' ? fmt(target) : len(target)})`, how };
}

/** The plain-words description and the C++ lines of one step (numbered `n` on the page, `lib` in the brain's log). */
function stepLines(step, n, plan, lib = n, robot = null, before = null) {
    const labelPart = step.label ? `"${step.label}": ` : '';
    let words;
    const code = [];
    const opt = options(step);
    const of = onFailLine(step);
    switch (step.do) {
        case 'driveTo':
        case 'reverseTo': {
            const back = step.do === 'reverseTo';
            const far = plan && plan.from ? ` about ${fmt(Math.round(dist(plan.from.x, plan.from.y, step.x, step.y) * 10) / 10)} in` : '';
            words = back ? `BACK UP to ${pt(step.x, step.y)}:${far}, its back toward the point`
                         : `DRIVE to ${pt(step.x, step.y)}:${far}`;
            if (step.faceFirst) {
                words += back ? '. First it turns its back to the point (at the robot\'s own turn settings)'
                              : '. First it turns to face the point (at the robot\'s own turn settings)';
                code.push(call(back ? 'faceAway' : 'face', [len(step.x), len(step.y)], null));
                if (of) {
                    code.push(of);
                }
            }
            code.push(call(step.do, [len(step.x), len(step.y)], opt));
            break;
        }
        case 'turnTo':
            words = `TURN to face ${headingWords(step.headingDeg)}`;
            code.push(call('turnTo', [deg(step.headingDeg)], opt));
            break;
        case 'face':
            words = `TURN to face ${pt(step.x, step.y)}`;
            code.push(call('face', [len(step.x), len(step.y)], opt));
            break;
        case 'faceAway':
            words = `TURN its back to ${pt(step.x, step.y)}`;
            code.push(call('faceAway', [len(step.x), len(step.y)], opt));
            break;
        case 'curveTo':
        case 'reverseCurveTo': {
            const pts = step.points;
            const last = pts[pts.length - 1];
            const through = pts.slice(0, -1).map((p) => `(${fmt(p.x)}, ${fmt(p.y)})`);
            const ending = `ending at ${pt(last.x, last.y)}`
                + (last.headingDeg !== undefined ? ` facing ${headingWords(last.headingDeg)}` : '');
            words = (step.do === 'reverseCurveTo' ? 'CURVE BACKWARDS (its back leading)' : 'CURVE')
                + (through.length ? ` through ${through.join(', ')} in,` : '') + ` ${ending}`;
            const list = pts.map((p) => (p.headingDeg !== undefined ? `{${len(p.x)}, ${len(p.y)}, ${deg(p.headingDeg)}}`
                                                                     : `{${len(p.x)}, ${len(p.y)}}`));
            code.push(call(step.do, [`{${list.join(', ')}}`], opt));
            break;
        }
        case 'forward':
        case 'backward': {
            const q = plan && plan.point;
            if (!q) {
                throw new Error(`export: step ${n} drives from a pose this route builder does not know`);
            }
            const back = step.do === 'backward';
            words = back ? `BACK UP ${fmt(step.inches)} in, straight back, to ${pt(q.x, q.y)}`
                         : `DRIVE FORWARD ${fmt(step.inches)} in, straight on, to ${pt(q.x, q.y)}`;
            code.push(call(back ? 'reverseTo' : 'driveTo', [len(q.x), len(q.y)], opt));
            break;
        }
        case 'move':
        case 'startMove': {
            const t = targetKind(step) === 'preset' ? null : targetMove(step, n, robot, before);
            const goal = t ? t.goal : presetGoal(step);
            if (step.do === 'move') {
                words = `MOVE the ${step.mechanism} ${t ? `${t.how},` : `to ${presetWords(step)}`} and wait until it is there: at most ${fmt(step.boundS)} s`;
            } else {
                words = `START the ${step.mechanism} ${t ? t.how.replace(', to ', ', toward ') : `toward ${presetWords(step)}`} and keep going (at most `
                    + `${fmt(step.boundS)} s; a WAIT UNTIL DONE collects it)`;
            }
            code.push(call(step.do, [goal, sec(step.boundS)], null));
            break;
        }
        case 'set':
            words = `SET the ${step.mechanism} to "${step.position}" (it takes the ${step.mechanism}'s move time)`;
            code.push(call('set', [`${step.mechanism}[${lit(step.position)}]`], null));
            break;
        case 'run':
            words = `RUN the ${step.mechanism} at ${fmt(step.volts)} V for ${fmt(step.forS)} s, then stop it`;
            code.push(call('run', [step.mechanism, volt(step.volts), sec(step.forS)], null));
            break;
        case 'startRun':
            words = `START the ${step.mechanism} at ${fmt(step.volts)} V for ${fmt(step.forS)} s and keep going`;
            code.push(call('startRun', [step.mechanism, volt(step.volts), sec(step.forS)], null));
            break;
        case 'home':
            words = `HOME the ${step.mechanism}: it finds its zero the way its declaration says, at most `
                + `${fmt(step.boundS)} s. Nothing can move it to a level before this`;
            code.push(call('home', [step.mechanism, sec(step.boundS)], null));
            break;
        case 'startHome':
            words = `START homing the ${step.mechanism} and keep going (at most ${fmt(step.boundS)} s); a WAIT UNTIL DONE`
                + ' must finish the home before its first move';
            code.push(call('startHome', [step.mechanism, sec(step.boundS)], null));
            break;
        case 'waitUntilDone':
            words = `WAIT until the ${step.mechanism}'s last step is done: at most ${fmt(step.boundS)} s more`;
            code.push(call('waitUntilDone', [step.mechanism, sec(step.boundS)], null));
            break;
        case 'wait':
            words = `WAIT ${fmt(step.seconds)} s. (The run's own wait: at 57 s the run cuts it at once.)`;
            code.push(`r.then([&] { return ctx.guard.pause(${sec(step.seconds)}) == sequence::GuardedWaitResult::Satisfied; },`);
            code.push(`       ${lit(`wait ${fmt(step.seconds)} s`)});`);
            break;
        default:
            throw new Error(`export: step ${n} is "${step.do}", which this route builder cannot write`);
    }
    const isMotion = MOTION_KINDS.has(step.do);
    const twoCalls = libraryCalls(step) === 2;
    let sentence = `Step ${n} — ${labelPart}${words}${isMotion ? careWords(step, twoCalls ? '; the drive: ' : ', ') : ''}`;
    if (!sentence.endsWith('.') && !sentence.endsWith('.)')) {
        sentence += '.';
    }
    sentence += failWords(step);
    if (twoCalls) {
        sentence += ` (The brain's log calls the turn step ${lib} and the drive step ${lib + 1}.)`;
    } else if (lib !== n) {
        sentence += ` (The brain's log calls this step ${lib}.)`;
    }
    const out = [...comment(sentence, '    ')];
    if (step.label) {
        out.push(`    r.label(${lit(step.label)});`);
    }
    // The step's onFail goes right before its call; a turn-first drive's two calls each get it (the one
    // for the drive is already between them, in `code`).
    if (of) {
        out.push(`    ${of}`);
    }
    for (const c of code) {
        out.push(`    ${c}`);
    }
    return out;
}

/** `    e.name = "skills";` with the trailing comment at §6.3's column. */
function entryLine(code, note) {
    const pad = Math.max(2, ENTRY_COMMENT_COLUMN - [...code].length);
    return `${code}${' '.repeat(pad)}// ${note}`;
}

/**
 * Each route's C++ name stem (D2): its name's letters and digits in camelCase ("skills" -> skills, "left side 2" ->
 * leftSide2), "route" before a leading digit (or for a name with none), and a number after a repeat — so each route's
 * `<stem>Steps` and `<stem>Route` are its own.
 */
export function routeIdents(routes) {
    const used = new Set();
    return routes.map((r, i) => {
        const words = String(r.name).match(/[A-Za-z0-9]+/g) || [];
        let id = words.map((w, k) => (k === 0 ? (w === w.toUpperCase() ? w.toLowerCase() : w[0].toLowerCase() + w.slice(1))
                                              : w[0].toUpperCase() + w.slice(1))).join('');
        if (!id || /^[0-9]/.test(id)) {
            id = `route${id || i + 1}`;
        }
        let out = id;
        for (let k = 2; used.has(out); ++k) {
            out = `${id}${k}`;
        }
        used.add(out);
        return out;
    });
}

/** One route's steps function and entry (inside the region's anonymous namespace). */
function routeBlock(route, id, robot) {
    const L = [];
    L.push(...(route.mirroredFor
        ? [`// The route "${route.name}", in ROBOT ONE's numbers. Robot two runs this same function MIRRORED across`,
           "// the field's diagonal (the list at the bottom says so)."]
        : [`// The route "${route.name}", in ROBOT ONE's numbers. Robot two runs it as written`,
           '// (the list at the bottom says so).']));
    L.push(`chassis::RoutineResult ${id}Steps(competition::AutonContext& ctx) {`);
    L.push(`    chassis::Routine r{ctx, ${lit(route.name)}};`);
    if (route.steps.length === 0) {
        L.push("    // NO STEPS YET: the team's route is made in the route maker and written here.");
    } else {
        const plan = plannedPoses(route);
        const heights = plannedHeights(route, robot);
        let lib = 1;  // the brain's log's number for the next call
        route.steps.forEach((s, i) => {
            L.push('');
            L.push(...stepLines(s, i + 1, plan[i], lib, robot, heights[i].before[s.mechanism] ?? null));
            lib += libraryCalls(s);
        });
        L.push('');
    }
    L.push('    return r.result();');
    L.push('}');
    L.push('');
    L.push(route.start ? "// Its entry: robot one's start, the 60 s skills schedule." : '// Its entry.');
    L.push(`competition::AutonEntry ${id}Route() {`);
    L.push('    competition::AutonEntry e{};');
    L.push(entryLine(`    e.name = ${lit(route.name)};`, "what the brain's menu shows"));
    L.push(entryLine('    e.kind = competition::AutonKind::Skills;', 'the 60 s skills run'));
    L.push(entryLine(`    e.routine = &${id}Steps;`, 'the steps above'));
    if (route.start) {
        const s = route.start;
        L.push(`    e.start = competition::StartPose::field(${len(s.x)}, ${len(s.y)}, ${deg(s.headingDeg)});  `
               + `// robot one's start, facing ${headingWords(s.headingDeg)}`);
    } else {
        L.push(entryLine('    e.start = competition::StartPose::relative();',
                         "NO START YET: the route maker writes robot one's start"));
    }
    L.push(`    e.endActionAt = ${sec(route.endActionAtS)};  // still running at ${fmt(route.endActionAtS)} s? it is stopped, `
           + 'and the end action (a brake) starts');
    L.push(`    e.hardStopAt = ${sec(route.hardStopAtS)};   // everything stops, ${fmt(60 - route.hardStopAtS)} s before the run's 60 s`);
    L.push('    return e;');
    L.push('}');
    return L;
}

/** The list entry's two words: robot two's way, and whether both robots run it at the same time (D3). */
function listEntry(route, id) {
    const mirror = route.mirroredFor ? 'Mirror::ForRobotTwo' : 'Mirror::None';
    const together = !route.mirroredFor || route.together === false ? 'Together::No'
        : route.together === true ? 'Together::Yes' : 'Together::NotAnswered';
    return `    {&${id}Route, ${mirror}, ${together}},`;
}

/**
 * The whole marked region for `routes` (the project's, in order; one route is taken too) BEGIN to END, inclusive,
 * WITHOUT a final newline — region.mjs keeps the file's own line ending and final newline when it splices this in.
 * `robot`: robot one's description, needed only by a step that moves an arm or a lift by or to a number.
 */
export function exportRegion(routes, robot = null) {
    const list = Array.isArray(routes) ? routes : [routes];
    const ids = routeIdents(list);
    const L = [];
    L.push(markerLine(ROUTE_NAME, 'BEGIN'));
    L.push('// Made in the route maker from src/team/route_maker.json. Change the route there and export again:');
    L.push('// the route maker will not overwrite a hand edit between BEGIN and END.');
    L.push('// To make this hand-written code instead, delete the BEGIN and END lines.');
    L.push('namespace {');
    list.forEach((route, k) => {
        if (k) {
            L.push('');
        }
        L.push(...routeBlock(route, ids[k], robot));
    });
    L.push('}  // namespace');
    L.push('');
    L.push("// EVERY ROUTE, in the order the brain's menu lists them (addTeamRoutes, below the END line, registers them).");
    L.push('// Robot one registers each as written; robot two MIRRORS one marked Mirror::ForRobotTwo. Together::Yes: both');
    L.push('// robots run it at the SAME time, so the host suite checks they never come too close.');
    L.push('const TeamRoute kTeamRoutes[] = {');
    const entries = list.map((route, k) => listEntry(route, ids[k]));
    const col = Math.max(...entries.map((e) => [...e].length)) + 2;
    entries.forEach((e, k) => L.push(`${e}${' '.repeat(col - [...e].length)}// ${lit(list[k].name)}`));
    L.push('};');
    L.push(markerLine(ROUTE_NAME, 'END'));
    return L.join('\n');
}
