// harness.mjs — what the browser tests share (RMP1-B; the same rules as page.test.mjs, A1's): the browser by
// ROUTE_MAKER_BROWSER, the built site from a tiny static server, and a page that FAILS on a CSP violation, a page
// error, a console error or a request leaving the page. Not a test file itself.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
// Firefox's bounce-tracking protection warns about the tests' own 127.0.0.1 server on Windows ("classified as a
// bounce tracker"): a warning from the browser's privacy feature, not the page's. It is turned off in the test browser.
const FIREFOX_PREFS = { 'privacy.bounceTrackingProtection.mode': 0 };

export const BROWSER = process.env.ROUTE_MAKER_BROWSER || '';
const HERE = path.dirname(fileURLToPath(import.meta.url));
export const REPO = path.resolve(HERE, '..', '..', '..', '..');
export const SITE = path.resolve(process.env.ROUTE_MAKER_SITE || path.join(REPO, 'build', 'route_maker_site'));
export const SHOTS = process.env.ROUTE_MAKER_SHOTS ? path.resolve(process.env.ROUTE_MAKER_SHOTS) : '';
export const skip = !BROWSER ? 'set ROUTE_MAKER_BROWSER (firefox, chromium, msedge or chrome) to run the page tests' : false;
const ALLOWED_WARNINGS = [];

/** §2.5.2's four (light and dark, 1440 x 900 and 1024 x 768) when the screenshots are taken; else one. */
export const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1024, 768], ['dark', 1024, 768]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));

let pw;
export async function launch(extra = {}) {
    pw = pw || (await import('playwright'));
    if (BROWSER === 'firefox') {
        return pw.firefox.launch({ firefoxUserPrefs: FIREFOX_PREFS, ...extra });
    }
    if (BROWSER === 'msedge' || BROWSER === 'chrome') {
        return pw.chromium.launch({ channel: BROWSER, ...extra });
    }
    return pw.chromium.launch(extra);
}

/** The built site from 127.0.0.1 (listening once this resolves): {server, base}. */
export async function serve(root = SITE) {
    const mime = { '.html': 'text/html; charset=utf-8', '.mjs': 'text/javascript', '.js': 'text/javascript', '.json': 'application/json', '.wasm': 'application/wasm' };
    const server = http.createServer((req, res) => {
        const f = path.join(root, decodeURIComponent(new URL(req.url, 'http://x').pathname));
        if (!f.startsWith(root) || !fs.existsSync(f) || fs.statSync(f).isDirectory()) {
            res.writeHead(404);
            res.end();
            return;
        }
        res.writeHead(200, { 'content-type': mime[path.extname(f)] || 'application/octet-stream' });
        res.end(fs.readFileSync(f));
    });
    await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
    return { server, base: `http://127.0.0.1:${server.address().port}` };
}

/** The site form in a fresh context; every problem recorded (done() returns them, CSP violations included). */
export async function openPage(browser, base, { viewport = { width: 1440, height: 900 }, scheme = 'light' } = {}) {
    const ctx = await browser.newContext({ viewport, colorScheme: scheme, acceptDownloads: true });
    const problems = [];
    const page = await ctx.newPage();
    page.on('console', (m) => {
        if (m.type() === 'error' || (m.type() === 'warning' && !ALLOWED_WARNINGS.some((w) => m.text().includes(w)))) {
            problems.push(`console ${m.type()}: ${m.text()}`);
        }
    });
    page.on('pageerror', (e) => problems.push(`page error: ${e}`));
    page.on('request', (r) => {
        const u = r.url();
        if (!u.startsWith(base) && !u.startsWith('data:') && !u.startsWith('blob:')) {
            problems.push(`a request outside the page: ${u}`);
        }
    });
    await page.goto(`${base}/app.html`);
    await page.waitForFunction(() => document.documentElement.dataset.ready, null, { timeout: 30000 });
    await page.evaluate(() => {
        window.__csp = [];
        document.addEventListener('securitypolicyviolation', (e) => window.__csp.push(`${e.violatedDirective} ${e.blockedURI}`));
    });
    return { page, ctx, done: async () => {
        const csp = await page.evaluate(() => window.__csp);
        await ctx.close();
        return [...problems, ...csp.map((c) => `CSP violation: ${c}`)];
    } };
}

/** A screenshot when ROUTE_MAKER_SHOTS is set (after the 200 ms animations; `rest`: after the toasts fade too). */
export async function shot(page, name, rest = false) {
    if (SHOTS) {
        fs.mkdirSync(SHOTS, { recursive: true });
        await page.waitForTimeout(rest ? 4500 : 400);
        await page.screenshot({ path: path.join(SHOTS, `${BROWSER}-${name}.png`) });
    }
}

/**
 * The site's robots with the field's solid structures taken out (RMP1-H). The recorded PREVIEW runs
 * (test/fixtures/preview/) were made before the structures were checked, and four of their routes drive through a
 * goal: PREVIEW refuses a route with an error, so the pages that play them are baked without the structures. The
 * structures, PREVIEW's THROUGH one included, are obstacles.page.test.mjs's.
 */
export function noStructures() {
    const baked = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots;
    return Object.fromEntries(Object.entries(baked).map(([id, r]) => [id, { ...r, field: { ...r.field, obstacles: [] } }]));
}
