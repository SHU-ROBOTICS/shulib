// engine.test.mjs — THE PREVIEW ENGINE IN A REAL BROWSER (RMP1-A2e; the RMP1 brief A.2 WG3, A.4): the SITE form of the
// module, run in a module Worker through the page's own wrapper (src/host/engine_module.mjs) — as the page's PREVIEW
// will run it (A2u) — gives every golden's record byte for byte as Node does here, and (given ROUTE_MAKER_WASM_RECORDS)
// as the Linux job's Node did. Runs when ROUTE_MAKER_BROWSER and ROUTE_MAKER_WASM are both set.
//
//   ROUTE_MAKER_BROWSER=firefox ROUTE_MAKER_WASM=build/wasm node --test tools/route_maker/test/page/engine.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { WASM, goldenRoutes, instance } from '../wasm/engine.mjs';
import { allowedWarning } from './allowed_warnings.mjs';
// Firefox's bounce-tracking protection warns about the tests' own 127.0.0.1 server on Windows ("classified as a
// bounce tracker"): a warning from the browser's privacy feature, not the page's. It is turned off in the test browser.
const FIREFOX_PREFS = { 'privacy.bounceTrackingProtection.mode': 0 };

const BROWSER = process.env.ROUTE_MAKER_BROWSER || '';
const HOST = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'src', 'host');
const skip = !BROWSER || !WASM ? 'set ROUTE_MAKER_BROWSER and ROUTE_MAKER_WASM to run the engine in a browser' : false;

const PAGE = `<!doctype html><meta charset="utf-8"><title>engine</title><link rel="icon" href="data:,">
<script type="module">
const w = new Worker('/worker.mjs', { type: 'module' });
window.run = (cases) => new Promise((ok, no) => { w.onmessage = (e) => ok(e.data); w.onerror = (e) => no(String(e.message)); w.postMessage(cases); });
document.documentElement.dataset.ready = '1';
</script>`;
const WORKER = `import create from '/wasm/preview.mjs';
import { engineInstance } from '/host/engine_module.mjs';
self.onmessage = async (e) => {
    const engine = await engineInstance(create);
    const out = {};
    for (const [name, lines] of e.data) { out[name] = engine.call(['preview', lines]).out; }
    self.postMessage(out);
};`;

function serve() {
    const types = { '.mjs': 'text/javascript', '.wasm': 'application/wasm' };
    return http.createServer((req, res) => {
        const u = new URL(req.url, 'http://x').pathname;
        const file = u.startsWith('/wasm/') ? path.join(WASM, path.basename(u)) : u.startsWith('/host/') ? path.join(HOST, path.basename(u)) : '';
        const body = u === '/' ? PAGE : u === '/worker.mjs' ? WORKER : file && fs.existsSync(file) ? fs.readFileSync(file) : null;
        res.writeHead(body === null ? 404 : 200, { 'content-type': u === '/' ? 'text/html; charset=utf-8' : types[path.extname(u)] || 'text/plain' });
        res.end(body ?? '');
    });
}

async function launch() {
    const pw = await import('playwright');
    if (BROWSER === 'firefox') {
        return pw.firefox.launch({ firefoxUserPrefs: FIREFOX_PREFS });
    }
    return pw.chromium.launch(BROWSER === 'msedge' || BROWSER === 'chrome' ? { channel: BROWSER } : {});
}

test(`WG3: the module in ${BROWSER || 'a browser'}'s module Worker gives every golden's record byte for byte as Node does`, { skip }, async (t) => {
    const goldens = goldenRoutes();
    const node = await instance('preview.mjs');
    const want = Object.fromEntries(goldens.map((g) => [g.name, node.run(['preview', g.lines]).out]));
    if (process.env.ROUTE_MAKER_WASM_RECORDS) {
        assert.deepEqual(want, JSON.parse(fs.readFileSync(process.env.ROUTE_MAKER_WASM_RECORDS, 'utf8')), 'Node here vs the Linux job');
    }
    const server = serve();
    await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
    let browser;
    try {  // a browser that will not launch still closes the server (else node --test hangs to its timeout)
        browser = await launch();
        const page = await browser.newPage();
        const errors = [];
        const allowed = [];
        page.on('pageerror', (e) => errors.push(String(e)));
        page.on('console', (m) => {  // the Worker's console reaches the page's (checked: Firefox's W6 warning)
            if (m.type() === 'warning' && allowedWarning(m.text())) {
                allowed.push(m.text());
            } else if (m.type() === 'error' || m.type() === 'warning') {
                errors.push(`${m.type()}: ${m.text()}`);
            }
        });
        await page.goto(`http://127.0.0.1:${server.address().port}/`);
        await page.waitForFunction(() => document.documentElement.dataset.ready);
        const got = await page.evaluate((cases) => window.run(cases), goldens.map((g) => [g.name, g.lines]));
        assert.deepEqual(errors, []);
        t.diagnostic(`allowed warnings: ${allowed.length ? allowed.join(' | ') : 'none'}`);
        for (const g of goldens) {
            assert.equal(got[g.name], want[g.name], `${g.name}: the browser's record differs from Node's`);
        }
    } finally {
        await browser?.close();
        server.close();
    }
});
