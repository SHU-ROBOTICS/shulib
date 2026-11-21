// engine.mjs — THE PREVIEW ENGINE's interface (RMP1 brief A.6; RMP1-A2u). The route tab calls only this:
//   makeEngine() -> {available, why, preview(route, robots, onPhase) -> Promise<outcome>, cancel()}
//   outcome: {ok: true, result: {one, two, verdict}, pairWords, noPair?} | {ok: false, words, cancelled}
// The engine runs in a Web Worker (worker.mjs): the page never freezes. A run that has not answered in 20 s is
// stopped and the page says so; the next PREVIEW makes a new Worker (A.3.10). Loading the engine (a slow connection
// downloading it) gets 60 s of its own. Where the Worker's script is: a file beside the page (the site form), inside
// the page (the one file), or nowhere (a build without the engine).

/** What the PREVIEW button says in a build without the engine. */
export const PREVIEW_COMING = 'PREVIEW comes next: the robot will play this route from the real library, right here.';
/** How long a PREVIEW may take before the page stops it. A 60 s route takes about 0.2 s. */
export const TIMEOUT_MS = 20000;
export const TIMEOUT_WORDS = 'A run did not finish in 20 s, so the preview was stopped. Press PREVIEW to try again; if it '
    + 'happens again, tell the coordinator which route it was.';
/** How long the engine may take to LOAD (the site form downloads it) before the page stops it. */
export const LOAD_MS = 60000;
export const LOAD_WORDS = 'The preview engine did not load in 60 s (a slow connection?), so it was stopped. Press PREVIEW to try '
    + 'again, or use the one-file download, which has the engine inside.';
const OLD_BROWSER = 'This browser is too old to run the preview: update it, or open the route builder in Chrome, Edge or Firefox.';

/** This page's engine script: {url} (the site form's file), {text} (the one file's), or null (none built in). */
export function engineSource(doc = typeof document !== 'undefined' ? document : null) {
    const inline = doc && doc.getElementById('shulib-engine');
    if (inline) {
        return { text: inline.textContent };
    }
    const meta = doc && doc.querySelector('meta[name="shulib-engine"]');
    return meta ? { url: meta.content } : null;
}

/** Whether this page can PREVIEW, and if not why: the engine built in, a browser that can run it, and Workers. */
export function previewAvailability(source = engineSource(), browser = globalThis.shulibBrowser, WorkerClass = globalThis.Worker) {
    const can = (!browser || browser.preview) && !!WorkerClass;
    return { available: !!source && can, why: !source ? PREVIEW_COMING : !can ? OLD_BROWSER : '' };
}

/** The engine of this page. Every option is for the tests: a stand-in script, a fake Worker, a shorter timeout. */
export function makeEngine({ source = engineSource(), browser = globalThis.shulibBrowser, WorkerClass = globalThis.Worker,
                             timeoutMs = TIMEOUT_MS, loadMs = LOAD_MS, timers = globalThis } = {}) {
    const { available, why } = previewAvailability(source, browser, WorkerClass);
    let worker = null;
    let pending = null;
    let next = 0;
    let blobUrl = null;

    const stop = () => {
        if (worker) {
            worker.terminate();
            worker = null;
        }
    };
    const finish = (outcome) => {
        if (pending) {
            const p = pending;
            pending = null;
            timers.clearTimeout(p.timer);
            p.resolve(outcome);
        }
    };
    /** (Re)start the stop: `ms` from now, saying `words`. The load, and each robot's run, get their own. */
    const arm = (ms, words) => {
        timers.clearTimeout(pending.timer);
        pending.timer = timers.setTimeout(() => {
            stop();
            finish({ ok: false, words });
        }, ms);
    };
    const start = () => {
        if (source.text !== undefined && !blobUrl) {
            blobUrl = URL.createObjectURL(new Blob([source.text], { type: 'text/javascript' }));
            source = { blob: true };
        }
        // CLASSIC: Chromium refuses a module Worker from a blob: URL on a page opened from disk (the one file)
        worker = new WorkerClass(blobUrl || source.url);
        worker.onmessage = ({ data }) => {
            if (!pending || data.id !== pending.id) {
                return;
            }
            if (data.phase) {
                arm(data.phase === 'starting' ? loadMs : timeoutMs, data.phase === 'starting' ? LOAD_WORDS : TIMEOUT_WORDS);
                pending.onPhase(data.phase);
            } else if (data.done) {
                finish(data.done);
            }
        };
        worker.onerror = (e) => {
            e.preventDefault?.();
            stop();
            finish({ ok: false, words: `The preview engine did not load (${e.message || 'no reason given'}). Everything else works; press PREVIEW to try again.` });
        };
    };

    return {
        available,
        why,
        preview(route, robots, onPhase = () => {}) {
            if (!available) {
                return Promise.resolve({ ok: false, words: why });
            }
            if (pending) {
                return Promise.resolve({ ok: false, words: 'A PREVIEW is already running.' });
            }
            return new Promise((resolve) => {
                pending = { id: ++next, resolve, onPhase };
                try {
                    arm(worker ? timeoutMs : loadMs, worker ? TIMEOUT_WORDS : LOAD_WORDS);
                    if (!worker) {
                        start();
                    }
                    worker.postMessage({ id: next, route, robots, pair: !!robots.two });
                } catch (e) {
                    stop();
                    finish({ ok: false, words: `The preview engine did not start (${e && e.message ? e.message : e}). Everything else works; press PREVIEW to try again.` });
                }
            });
        },
        /** Stop the PREVIEW that is running (its Worker goes; the next PREVIEW makes a new one). */
        cancel() {
            if (pending) {
                stop();
                finish({ ok: false, cancelled: true, words: 'PREVIEW stopped.' });
            }
        },
    };
}
