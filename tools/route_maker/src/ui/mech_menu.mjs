// mech_menu.mjs — RMP1-J D1: M's menu in two short steps — the robot's mechanisms, one row each; then what the chosen
// one does, each a big button in plain words. Up / Down and Enter walk it; Esc goes back, then closes.

import { h, icon, mount } from './dom.mjs';
import { mechanismActions, KIND_LABEL } from './words.mjs';
import { menuRows, menuActions } from '../model/mech_steps.mjs';
import { noLevels } from '../model/checks.mjs';
import { robotName } from '../model/robot_names.mjs';

/** Open it under `anchor` for a step at `at` of `route`; `add(step, words)` adds the one chosen. Returns its close. */
export function openMechMenu(anchor, robot, route, at, add) {
    const mechs = (robot && robot.mechanisms) || [];
    const actionsOf = (m) => menuActions(m, mechanismActions(m, !noLevels(m)), route, at);
    const pop = h('div', { class: 'popover mech-menu', role: 'menu' });
    let chosen = null;
    const items = () => [...pop.querySelectorAll('.menu-item:not(:disabled)')];
    const focus = (sel) => (pop.querySelector(sel) || items()[0] || pop).focus();
    function close() {
        pop.remove();
        document.removeEventListener('pointerdown', outside, true);
        document.removeEventListener('keydown', onKey, true);
    }
    function list(back = null) {
        chosen = null;
        pop.setAttribute('aria-label', 'Mechanisms');
        mount(pop, h('h3', {}, 'Which mechanism?'), mechs.length ? menuRows(robot).map((r, k) => {
            const n = actionsOf(mechs[k]).length;
            return h('button', { class: 'menu-item mech-row', role: 'menuitem', 'data-mech': r.ident, disabled: !n, onclick: () => show(mechs[k]) },
                     icon(r.icon), r.name, h('small', {}, n ? KIND_LABEL[r.kind] || r.kind : 'steps come later'));
        }) : h('p', { class: 'hint', style: 'padding:8px' }, `${robotName(robot.id)} declares no mechanism yet.`));
        focus(back ? `[data-mech="${back}"]` : '.menu-item:not(:disabled)');
    }
    function show(m) {
        chosen = m;
        pop.setAttribute('aria-label', `${m.name[0].toUpperCase()}${m.name.slice(1)}`);
        mount(pop, h('button', { class: 'menu-item menu-back', role: 'menuitem', onclick: () => list(m.ident) }, '‹ Back'),
              h('h3', { class: 'mech-head' }, `${m.name[0].toUpperCase()}${m.name.slice(1)}`),
              actionsOf(m).map((a) => h('button', { class: 'menu-item big', role: 'menuitem', 'data-act': a.words,
                                                    onclick: () => { close(); add(a.step, a.words); } }, icon(a.icon), a.words)));
        focus('.menu-item.big');
    }
    const outside = (e) => !pop.contains(e.target) && !anchor.contains(e.target) && close();
    const onKey = (e) => {
        if (e.key === 'Escape') {
            e.stopPropagation();
            if (chosen) {
                list(chosen.ident);
            } else {
                close();
                anchor.focus();
            }
        } else if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
            const all = items();
            const i = all.indexOf(document.activeElement);
            if (all.length) {
                e.preventDefault();
                e.stopPropagation();
                all[(i + (e.key === 'ArrowDown' ? 1 : all.length - 1)) % all.length].focus();
            }
        }
    };
    const r = anchor.getBoundingClientRect();
    pop.style.left = `${Math.max(8, Math.min(r.left, window.innerWidth - 340))}px`;
    pop.style.top = `${r.bottom + 6}px`;
    document.body.append(pop);
    list();
    document.addEventListener('keydown', onKey, true);  // at once: keys typed right after M are the menu's
    setTimeout(() => document.addEventListener('pointerdown', outside, true), 0);  // after the click that opened it
    return close;
}
