// step_view.mjs — how a step SHOWS on the page (RMP1-G): hidden, locked, and the named sections of the step list.
// Stored on the step as `view` ({hidden, locked, section, folded}, each only when set) in route_maker.json; the
// export and the preview never read it. A section starts at the step whose `view.section` names it and runs to the
// step before the next one. Pure: no DOM.

/** Every field of a step's `view` (a file with any other is from a newer builder: project.mjs refuses it). */
export const VIEW_KEYS = Object.freeze(['hidden', 'locked', 'section', 'folded']);

/** `view` as stored: only what is set, in one order; undefined when nothing is. `folded` only on a section's head. */
export function normalizeView(view) {
    if (!view || typeof view !== 'object') {
        return undefined;
    }
    const out = {};
    const name = typeof view.section === 'string' ? view.section.trim() : '';
    if (view.hidden === true) {
        out.hidden = true;
    }
    if (view.locked === true) {
        out.locked = true;
    }
    if (name) {
        out.section = name;
        if (view.folded === true) {
            out.folded = true;
        }
    }
    return Object.keys(out).length ? out : undefined;
}

export const isHidden = (step) => !!(step && step.view && step.view.hidden);
export const isLocked = (step) => !!(step && step.view && step.view.locked);

/** The route with `patch` merged into the view of every step in `indices`. */
export function setView(route, indices, patch) {
    const set = new Set(indices);
    return { ...route, steps: route.steps.map((s, i) => (set.has(i) ? withView(s, { ...(s.view || {}), ...patch }) : s)) };
}

function withView(step, view) {
    const { view: _, ...rest } = step;
    const v = normalizeView(view);
    return v ? { ...rest, view: v } : rest;
}

/** A step left index `at` of `steps` (deleted, or moved away): the step now there takes its section's name and fold,
 *  so the rest of the section stays in it (and never drops into a folded one above); unless that step starts a section
 *  itself. Changes `steps` in place; returns the step that left, without its section when it was passed on. */
export function passSection(steps, at, gone) {
    const v = gone && gone.view;
    const next = steps[at];
    if (!v || !v.section || !next || (next.view && next.view.section)) {
        return gone;
    }
    steps[at] = withView(next, { ...(next.view || {}), section: v.section, folded: v.folded });
    return withView(gone, { ...v, section: undefined, folded: undefined });
}

/** Every section, in order: {name, head, from, to, folded} (from = head, to inclusive). */
export function sectionsOf(route) {
    const heads = route.steps.map((s, i) => (s.view && s.view.section ? i : -1)).filter((i) => i >= 0);
    return heads.map((head, k) => ({ name: route.steps[head].view.section, head, from: head,
                                     to: (k + 1 < heads.length ? heads[k + 1] : route.steps.length) - 1,
                                     folded: !!route.steps[head].view.folded }));
}

/** The route with the section holding step `i` unfolded (a step moved into a folded one is never put away unseen). */
export function unfoldAt(route, i) {
    const sec = sectionsOf(route).find((x) => x.from <= i && i <= x.to);
    return sec && sec.folded ? setView(route, [sec.head], { folded: false }) : route;
}

/** The indices of section `sec`. */
export const indicesOf = (sec) => Array.from({ length: sec.to - sec.from + 1 }, (_, k) => sec.from + k);

/** Focus on `at` (a step's index or 'start'): it and its neighbours — the start counts as step 1's. */
export function focusSet(route, at) {
    const out = new Set();
    if (at === 'start') {
        out.add('start').add(0);
    } else if (Number.isInteger(at)) {
        [at - 1, at, at + 1].forEach((i) => out.add(i < 0 ? 'start' : i));
    }
    return new Set([...out].filter((i) => i === 'start' || (i >= 0 && i < route.steps.length)));
}
