// undo.mjs — the edit history (RMP1 brief §1.1 `undo.mjs`; §1.4: Ctrl+Z / Ctrl+Shift+Z).
//
// A bounded list of whole states: every edit, every drag's END and every import is one entry, so undoing
// it puts back exactly the state before it (RT2). A drag's intermediate positions are never entries — the
// page keeps them out of the history and commits the drag once, when it ends. Pure; states are never
// mutated (the route model returns new objects).

/** How many states the history keeps. */
export const HISTORY_LIMIT = 200;

/** A history whose present is `state`. */
export function historyOf(state) {
    return { past: [], present: state, future: [] };
}

/** `h` with `state` as the new present (the redo list is dropped, as in every editor). */
export function commit(h, state) {
    if (state === h.present) {
        return h;
    }
    const past = [...h.past, h.present];
    if (past.length > HISTORY_LIMIT) {
        past.splice(0, past.length - HISTORY_LIMIT);
    }
    return { past, present: state, future: [] };
}

/** One step back (unchanged when there is nothing to undo). */
export function undo(h) {
    if (h.past.length === 0) {
        return h;
    }
    return { past: h.past.slice(0, -1), present: h.past[h.past.length - 1], future: [h.present, ...h.future] };
}

/** One step forward again (unchanged when there is nothing to redo). */
export function redo(h) {
    if (h.future.length === 0) {
        return h;
    }
    return { past: [...h.past, h.present], present: h.future[0], future: h.future.slice(1) };
}

/** True when there is something to undo / redo. */
export const canUndo = (h) => h.past.length > 0;
export const canRedo = (h) => h.future.length > 0;
