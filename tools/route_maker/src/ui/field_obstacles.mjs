// field_obstacles.mjs — THE FIELD'S SOLID STRUCTURES on the screen (RMP1-H D2, D4): each goal and Loader a quiet
// outline under the route, named on hover ("red goal by the right wall — solid"), never a handle. The selected step's
// red or amber structure is the one highlight, with the robot's square at that moment. In the inspector, a step that
// comes close to one can say it touches it on purpose (placing a cone): that silences its amber, and lets the robot's
// front end over a red or blue goal's base (obstacles.mjs) -- never a red on the way.

import { h, s } from './dom.mjs';
import { nearStructures, frontBayOf, robotOutline } from '../model/obstacles.mjs';

/** The outlines (an SVG group) for field_view's layer; `P` maps field inches to picture pixels. */
export function obstacleShapes(app, P) {
    const obs = (app.field() || {}).obstacles || [];
    const sel = app.state().selected;
    const at = sel === 'start' ? -1 : sel;
    const hit = typeof at === 'number' ? app.issues().find((x) => x.obstacle && x.stepIndex === at) : null;
    const pts = (list) => list.map(([x, y]) => P(x, y)).map((q) => `${q.px},${q.py}`).join(' ');
    const g = s('g', { class: 'g-obstacles' });
    for (const o of obs) {
        const on = hit && hit.obstacle.id === o.id ? ` hit-${hit.level}` : '';
        g.append(s('polygon', { class: `g-obstacle${on}`, points: pts(o.footprint), 'data-obstacle': o.id }, s('title', {}, `${o.name} — solid`)));
    }
    if (hit) {  // RMP1-Q: against a goal, the outline it was checked with (its bay's radius for that goal)
        const o = obs.find((x) => x.id === hit.obstacle.id);
        const who = hit.code === 'RCO3' ? app.baked.robots[app.route().mirroredFor] : app.robot();  // robot two's: its own
        const bay = o && o.kind === 'goal' ? frontBayOf(who) : null;
        const r = bay && (o.overhangIn > 0 ? bay.lip : bay.body);
        g.append(s('polygon', { class: `g-obstacle-moment ${hit.level}`, 'data-moment': hit.obstacle.id,
                                points: pts(robotOutline(hit.obstacle.at, app.size(), bay, r).map((c) => [c.x, c.y])) }));
    }
    return g;
}

/** D4: step `i`'s "touches it on purpose" switches — one per structure the switch would clear (or already says), or null. */
export function touchOnPurpose(step, i, app) {
    const obs = (app.field() || {}).obstacles || [];
    const had = step.touches || [];
    const ids = [...new Set([...nearStructures(app.route(), app.plan(), app.field(), app.size(), i, frontBayOf(app.robot())), ...had])];
    if (!ids.length) {
        return null;
    }
    return h('div', { class: 'section touch-purpose' }, ids.map((id) => {
        const o = obs.find((x) => x.id === id);
        return h('label', { class: 'switch' },
            h('span', {}, `Touches the ${o ? o.name : id} on purpose`, h('small', {}, o && o.kind === 'Loader' ? 'taking a load' : 'placing a cone')),
            h('input', { type: 'checkbox', role: 'switch', checked: had.includes(id), name: `touches-${id}`,
                         onchange: (e) => app.update(i, { touches: e.target.checked ? [...had, id] : had.filter((x) => x !== id) }) }));
    }));
}
