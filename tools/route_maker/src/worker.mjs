// worker.mjs — runs the PREVIEW engine OFF the page's thread (RMP1-A2u; brief A.3.10), so the page never freezes.
// build.py puts this file after the engine's own code, which defines self.shulibBackend: an async function making
// ONE engine instance with
//   preview(route, robot) -> {rc, record, words}       a robot's run of the route (the runner's record, schema 1)
//   pair(route, robots)   -> {rc, record, verdict}     robot two's MIRRORED run and the pair check (when it has one)
// A run that returns non-zero may have stopped mid-route, leaving what it built in the module: the next run gets a
// NEW instance (P-W1f). Messages out: {id, phase} while it works, then {id, done}.
(() => {
    let instance = null;
    const make = async (say) => {
        if (!instance) {
            say({ phase: 'starting' });
            instance = await self.shulibBackend();
        }
        return instance;
    };
    self.onmessage = async ({ data }) => {
        const { id, route, robots, pair } = data;
        const say = (m) => self.postMessage({ id, ...m });
        try {
            const eng = await make(say);
            say({ phase: 'running' });
            const one = eng.preview(route, robots.one);
            if (one.rc !== 0) {
                instance = null;
            }
            if (!one.record) {
                say({ done: { ok: false, words: one.words || 'the engine gave no record' } });
                return;
            }
            const result = { one: one.record, two: null, verdict: null };
            let pairWords = '';
            const noPair = pair && !eng.pair;  // no pair mode: the page says so, in its robots' names (RMP1-L)
            if (pair && eng.pair) {
                const eng2 = await make(say);
                say({ phase: 'two' });
                const two = eng2.pair(route, robots);
                if (two.rc !== 0) {
                    instance = null;
                }
                result.two = two.record;
                result.verdict = two.verdict;
                pairWords = two.record ? '' : two.words || '';
            }
            say({ done: { ok: true, result, pairWords, ...(noPair ? { noPair } : {}) } });
        } catch (e) {
            instance = null;
            say({ done: { ok: false, words: `The preview engine stopped: ${e && e.message ? e.message : e}` } });
        }
    };
})();
