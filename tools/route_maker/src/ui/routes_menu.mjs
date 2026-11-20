// routes_menu.mjs — THE ROUTE SWITCHER in the top bar (RMP1-C D1): the project's named routes, one on the field at a
// time — switch, add, rename, duplicate, delete (confirmed in the page). A name is what the brain's menu shows: the
// library's rules (checks.mjs nameProblem), unique in the project, capitals aside. "skills" keeps its name and stays.

import { h, icon, ask, askText, openMenu } from './dom.mjs';
import { nameProblem } from '../model/checks.mjs';
import { ROUTE_NAME } from '../model/route.mjs';
import { freeName, routeCap } from '../model/project.mjs';

/** The switcher over `app` (app.mjs): {button(), rename(k)}. */
export function createRoutesMenu(app) {
    const robot = () => app.robot();
    const cap = () => routeCap(robot());

    /** Why `name` cannot be route `k`'s (k: -1 for a new one), in words — or null. */
    function problem(name, k, mirrored) {
        const p = nameProblem(name, mirrored, robot());
        if (p) {
            return p.words;
        }
        const twin = app.routes().findIndex((r, i) => i !== k && r.name.toLowerCase() === name.toLowerCase());
        return twin >= 0 ? `"${app.routes()[twin].name}" is taken: two routes' names must differ by more than capitals.` : null;
    }
    const nameIt = (title, value, k, mirrored, ok) => askText({
        title, value, ok, label: 'The route\'s name',
        words: 'What the brain\'s Auton Tester menu shows: plain keyboard characters, but no %, " or \\.',
        check: (t) => problem(t, k, mirrored) });

    async function add() {
        const name = await nameIt('A new route', freeName(app.routes(), robot()), -1, false, 'Add the route');
        if (name !== null) {
            app.addRoute(name);
        }
    }
    async function rename(k) {
        const r = app.routes()[k];
        const name = await nameIt(`Rename "${r.name}"`, r.name, k, !!r.mirroredFor, 'Rename');
        if (name !== null && name !== r.name) {
            app.renameRoute(k, name);
        }
    }
    async function duplicate(k) {
        const r = app.routes()[k];
        const name = await nameIt(`Duplicate "${r.name}"`, freeName(app.routes(), robot(), `${r.name} copy`), -1, !!r.mirroredFor, 'Duplicate');
        if (name !== null) {
            app.duplicateRoute(k, name);
        }
    }
    async function remove(k) {
        const r = app.routes()[k];
        const n = r.steps.length;
        const yes = await ask({ title: `Delete "${r.name}"?`, cancel: false,
            words: `${n ? `Its ${n} step${n === 1 ? '' : 's'} go${n === 1 ? 'es' : ''} with it.` : 'It has no steps.'} Undo (Ctrl+Z) brings it back.`,
            choices: [{ label: 'Delete the route', value: true, kind: 'danger' }, { label: 'Keep it', value: false }] });
        if (yes) {
            app.deleteRoute(k);
        }
    }

    function open(anchor) {
        const at = app.at();
        const full = app.routes().length >= cap();
        const rows = app.routes().map((r, k) => {
            const errors = app.issuesOf(k).filter((x) => x.level === 'error').length;
            const skills = r.name === ROUTE_NAME;
            const how = r.mirroredFor ? `robot two mirrored${r.together === true ? ' · together' : r.together === false ? ' · one at a time' : ''}` : 'robot two as written';
            return h('div', { class: 'route-row', 'data-route': r.name },
                h('button', { class: 'menu-item route-pick', role: 'menuitemradio', 'aria-checked': String(k === at), onclick: () => app.switchRoute(k) },
                  h('span', { class: 'route-dot', 'data-state': errors ? 'error' : r.steps.length ? 'ok' : 'empty', title: errors ? `${errors} error${errors === 1 ? '' : 's'}` : '' }),
                  h('span', { class: 'route-text' }, h('strong', {}, r.name), h('small', {}, `${r.steps.length ? `${r.steps.length} step${r.steps.length === 1 ? '' : 's'}` : 'no steps'} · ${how}`)),
                  skills ? h('span', { class: 'chip robot' }, 'skills run') : null),
                h('button', { class: 'menu-item act', 'aria-label': `Rename "${r.name}"`, title: skills ? 'The skills route keeps its name' : 'Rename',
                              disabled: skills, onclick: () => rename(k) }, icon('edit')),
                h('button', { class: 'menu-item act', 'aria-label': `Duplicate "${r.name}"`, title: full ? `At most ${cap()} routes` : 'Duplicate',
                              disabled: full, onclick: () => duplicate(k) }, icon('dup')),
                h('button', { class: 'menu-item act danger', 'aria-label': `Delete "${r.name}"`, title: skills ? 'The skills route is always there' : 'Delete',
                              disabled: skills, onclick: () => remove(k) }, icon('trash')));
        });
        openMenu(anchor, [h('h3', {}, 'Routes · the brain\'s menu lists them in this order'), rows,
            h('button', { class: 'menu-item add-route', disabled: full, onclick: add }, icon('plus'),
              full ? `At most ${cap()} routes (the brain's menu is full)` : 'New route')], 'Routes');
    }

    function button() {
        const r = app.routes()[app.at()];
        const n = app.routes().length;
        return h('button', { class: 'route-switch', id: 'route-switch', 'aria-haspopup': 'menu',
                             title: 'The route on the field — its name on the brain\'s menu. Click to switch, add, rename or delete routes.',
                             onclick: (e) => open(e.currentTarget) },
            h('span', { class: 'route-switch-label' }, 'Route'), h('strong', {}, r.name),
            n > 1 ? h('span', { class: 'count' }, `${app.at() + 1}/${n}`) : null, icon('chevron'));
    }

    return { button, rename };
}
