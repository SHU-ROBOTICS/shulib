// engine_backend.mjs — the REAL engine for worker.mjs (RMP1-A2u): the library in WebAssembly (RMP1-A2e's module,
// run through its engine_module.mjs), handed the route as the lines route_lines.mjs writes. build.py bundles this
// into the Worker's script only when it is given the built module (--engine). Robot two's mirrored run waits for
// the runner's pair mode (README "PREVIEW"): until then this backend has no `pair`, and the page says so.

import { engineInstance } from './engine_module.mjs';
import { routeLines } from '../model/route_lines.mjs';

/** One instance of the module made by `createPreview`, as worker.mjs calls it. */
export async function wasmBackend(createPreview) {
    const e = await engineInstance(createPreview);
    return {
        preview(route, robot) {
            const r = e.preview(routeLines(route, robot));
            return { rc: r.rc, record: r.record, words: r.words };
        },
    };
}
