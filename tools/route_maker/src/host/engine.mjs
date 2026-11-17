// engine.mjs — THE PREVIEW ENGINE's interface (RMP1 brief A.6). In RMP1-A1 there is no engine: PREVIEW is
// shown disabled, saying it comes next. RMP1-A2e builds the WebAssembly module (the REAL library, the host test
// plant and RPV1's engine, run in a Worker) and A2u wires it here — the route tab calls only these two:
//   describe(robotId)        -> the robot's description (today the baked one, robots.mjs)
//   preview(route, robotId)  -> {ok, record} | {ok: false, words}
// The record's shape is the runner's JSON (schema 1, A2e). Nothing here animates the page's own guess as if it
// were the robot (landmine 1): until the engine exists, nothing is played at all.

/** True when this build can preview. */
export const PREVIEW_AVAILABLE = false;

/** What the PREVIEW button says while there is no engine. */
export const PREVIEW_COMING = 'PREVIEW comes next: the robot will play this route from the real library, right here.';

/** The engine of this build (none in A1). */
export function makeEngine() {
    return {
        available: PREVIEW_AVAILABLE,
        async preview() {
            return { ok: false, words: PREVIEW_COMING };
        },
    };
}
