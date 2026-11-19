// engine_module.mjs — ONE instance of the PREVIEW ENGINE, the WebAssembly module (RMP1-A2e; brief A.3.10), run the way
// the page's Worker (RMP1-A2u) and the engine's node tests run it. Pure: no DOM, no Worker, no file API.
//
// The module's callMain is made to be called once: each call leaves its arguments — the route's lines — on the
// module's 1 MB C stack (Emscripten 6.0.13 never restores it), so some hundreds of PREVIEWs would overflow it (679
// of a 1.4 KB route). Every call here restores the stack. A run stopped by the no-pace stop (the record's `looped`;
// std::_Exit ends only the call, P-W1f) leaves what it built on the module's heap: the instance runs on correctly
// (WG7), but a fresh one frees that memory. A call that THROWS (the module aborted) leaves the instance dead.

/** An instance made by `createPreview` (the default export of preview.mjs or preview_single.mjs). */
export async function engineInstance(createPreview) {
    const out = [];
    const err = [];
    const mod = await createPreview({ print: (l) => out.push(l), printErr: (l) => err.push(l) });
    /** The program's `main` with `args`: {rc, out (stdout, whole lines), err}. */
    const call = (args) => {
        out.length = 0;
        err.length = 0;
        const sp = mod.stackSave();
        let rc;
        try {
            rc = mod.callMain(args);
        } finally {
            mod.stackRestore(sp);
        }
        return { rc, out: out.length ? `${out.join('\n')}\n` : '', err: err.join('\n') };
    };
    return {
        call,
        /**
         * PREVIEW `lines` (route_lines.mjs): {rc, text, record, words}. rc 0 it ran (a failed, skipped, jumped or cut
         * step too: read the record's `word` and each page step's `verdict`), 1 it threw, went THROUGH, was refused or
         * hit the no-pace stop, 2 the lines were refused, 3 the program failed (no record for 2 or 3; `words` say
         * why). The record is the runner's JSON (schema 1, README.md). Throws when the module aborts (above).
         */
        preview(lines) {
            const r = call(['preview', lines]);
            return { rc: r.rc, text: r.out, record: r.out ? JSON.parse(r.out) : null, words: r.err };
        },
    };
}
