// Records the narrated Controls tab tutorial: the REAL route builder (the built site), driven by Playwright in
// Chromium, with an overlay injected into the page (a spotlight, a pulsing ring, a cursor, a click ripple and a
// caption band: never the app's code), then ffmpeg puts each beat's narration at its start, with the music under it.
//
//   node tools/docs_video/controls_tab/record.mjs [--site DIR] [--out DIR]
//
// Needs: narration/<id>.wav (narrate.py, from beats.json); the built site (--site, default build/route_maker_site:
// python3 tools/route_maker/build.py); Playwright through PW_NODE_MODULES (else tools/route_maker/node_modules);
// MUSIC (default ~/shulib-render-tmp/music/Wallpaper.mp3); FFMPEG (default ffmpeg); DRIVER_DIR, the folder holding
// robot_one_driver.hpp and robot_two_driver.hpp for the in-memory "connected folder" (default src/team). Writes
// docs/assets/video/controls-tab.mp4 and controls-tab.jpg. Nothing is written into any folder by the page: the
// Export drawer is shown, its Write button is never pressed.
import {createRequire} from 'node:module';
import {execFileSync} from 'node:child_process';
import fs from 'node:fs';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '..', '..', '..');
const args = process.argv.slice(2);
const opt = (name) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : undefined; };
const modules = process.env.PW_NODE_MODULES || path.join(repo, 'tools', 'route_maker', 'node_modules');
const {chromium} = createRequire(path.join(modules, 'noop.js'))('playwright');
const ffmpeg = process.env.FFMPEG || 'ffmpeg';
const music = process.env.MUSIC || path.join(os.homedir(), 'shulib-render-tmp', 'music', 'Wallpaper.mp3');
const site = path.resolve(opt('--site') || path.join(repo, 'build', 'route_maker_site'));
const driverDir = process.env.DRIVER_DIR || path.join(repo, 'src', 'team');
const out = opt('--out') || fs.mkdtempSync(path.join(os.tmpdir(), 'controls-tab-'));
fs.mkdirSync(out, {recursive: true});
const mp4 = path.join(repo, 'docs', 'assets', 'video', 'controls-tab.mp4');
const jpg = path.join(repo, 'docs', 'assets', 'video', 'controls-tab.jpg');

const beats = JSON.parse(fs.readFileSync(path.join(here, 'beats.json'), 'utf8'));
/** A WAV file's length in seconds (its data chunk / its byte rate). */
function wavSeconds(file) {
  const b = fs.readFileSync(file);
  const byteRate = b.readUInt32LE(28);
  let i = 12;
  while (i < b.length - 8) {
    const id = b.toString('ascii', i, i + 4);
    const size = b.readUInt32LE(i + 4);
    if (id === 'data') return size / byteRate;
    i += 8 + size + (size % 2);
  }
  throw new Error(`no data chunk in ${file}`);
}
for (const b of beats) {
  b.wav = path.join(here, 'narration', `${b.id}.wav`);
  if (!fs.existsSync(b.wav)) throw new Error(`${b.wav} is missing: run narrate.py first (see README.md)`);
  b.len = wavSeconds(b.wav);
  b.dur = Math.max(3.5, b.len + 0.7);
}

// the built site, served on 127.0.0.1 (a free port) for as long as the recording runs
const TYPES = {'.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.json': 'application/json', '.css': 'text/css',
  '.svg': 'image/svg+xml', '.png': 'image/png', '.wasm': 'application/wasm', '.woff2': 'font/woff2', '.jpg': 'image/jpeg'};
const server = http.createServer((req, res) => {
  const p = path.join(site, decodeURIComponent(new URL(req.url, 'http://x').pathname));
  if (!p.startsWith(site) || !fs.existsSync(p) || fs.statSync(p).isDirectory()) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, {'content-type': TYPES[path.extname(p)] || 'application/octet-stream'});
  fs.createReadStream(p).pipe(res);
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const base = `http://127.0.0.1:${server.address().port}`;

// ---- the overlay: one spotlight, one ring, one cursor, one caption band; appended to <html>, outside the app ----
const OVERLAY_CSS = `
#tut-hole{position:fixed;z-index:2147483640;pointer-events:none;border-radius:12px;left:-40px;top:-40px;width:0;height:0;
  box-shadow:0 0 28px 6px rgba(12,8,28,.62),0 0 0 4000px rgba(12,8,28,.62);transition:all .55s cubic-bezier(.4,0,.2,1)}
#tut-hole.full{left:50%;top:50%;width:0;height:0}
#tut-hole.off{box-shadow:0 0 0 4000px rgba(12,8,28,0)}
#tut-ring{position:fixed;z-index:2147483641;pointer-events:none;border:3px solid #ffc94d;border-radius:12px;opacity:0;
  transition:all .55s cubic-bezier(.4,0,.2,1)}
#tut-ring.on{opacity:1;animation:tut-pulse 1.3s ease-in-out infinite}
@keyframes tut-pulse{0%,100%{box-shadow:0 0 0 0 rgba(255,201,77,.75)}50%{box-shadow:0 0 0 9px rgba(255,201,77,0)}}
#tut-cursor{position:fixed;z-index:2147483646;pointer-events:none;left:0;top:0;width:30px;height:30px;
  transform:translate(640px,400px);transition:transform .75s cubic-bezier(.45,0,.25,1);filter:drop-shadow(0 2px 3px rgba(0,0,0,.45))}
#tut-cursor.drag{transition:none}
.tut-ripple{position:fixed;z-index:2147483645;pointer-events:none;width:16px;height:16px;margin:-8px 0 0 -8px;border-radius:50%;
  border:3px solid #ffc94d;animation:tut-rip .6s ease-out forwards}
@keyframes tut-rip{from{transform:scale(.4);opacity:1}to{transform:scale(3.4);opacity:0}}
#tut-cap{position:fixed;z-index:2147483644;pointer-events:none;left:0;right:0;bottom:0;min-height:58px;display:flex;align-items:center;
  justify-content:center;background:rgba(14,10,30,.92);color:#fff;font:600 24px/1.25 system-ui,"Segoe UI",Roboto,sans-serif;
  padding:8px 24px;letter-spacing:.005em;transition:opacity .25s}
.drawer-body,.controls-mid{padding-bottom:76px!important}
#tut-cap b{color:#ffc94d;font-weight:800}
#tut-card{position:fixed;z-index:2147483643;pointer-events:none;left:50%;top:44%;transform:translate(-50%,-50%) scale(.96);opacity:0;
  background:#fff;color:#1b1530;border-radius:18px;padding:30px 40px;min-width:560px;max-width:760px;box-shadow:0 20px 60px rgba(0,0,0,.45);
  font:500 22px/1.45 system-ui,"Segoe UI",Roboto,sans-serif;transition:all .45s cubic-bezier(.4,0,.2,1)}
#tut-card.on{opacity:1;transform:translate(-50%,-50%) scale(1)}
#tut-card h2{margin:0 0 10px;font-size:34px;color:#6b2fc0}
#tut-card .row{display:flex;gap:14px;align-items:center;margin:8px 0}
#tut-card .credit{margin-top:16px;font-size:14px;color:#5a5470}
#tut-card .k{background:#6b2fc0;color:#fff;border-radius:8px;padding:2px 12px;font-weight:800;min-width:30px;text-align:center}
#tut-key{position:fixed;z-index:2147483646;pointer-events:none;opacity:0;background:#1b1530;color:#fff;border:2px solid #ffc94d;border-radius:10px;
  padding:6px 14px;font:800 22px system-ui,sans-serif;transition:opacity .25s}
#tut-key.on{opacity:1}
`;
const CURSOR_SVG = `<svg viewBox="0 0 24 24" width="30" height="30"><path d="M4 2 L4 20 L9 15.5 L12.5 22.5 L15.5 21 L12 14 L18.5 14 Z"
  fill="#fff" stroke="#111" stroke-width="1.6" stroke-linejoin="round"/></svg>`;

async function installOverlay(page) {
  await page.evaluate(({css, cursor}) => {
    const st = document.createElement('style');
    st.textContent = css;
    document.head.append(st);
    const mk = (id, html) => { const e = document.createElement('div'); e.id = id; if (html) e.innerHTML = html; document.documentElement.append(e); return e; };
    mk('tut-hole').classList.add('full', 'off');
    mk('tut-ring');
    mk('tut-card');
    mk('tut-key');
    mk('tut-cap').style.opacity = '0';
    mk('tut-cursor', cursor);
    /** The union of the rectangles of the given selectors / elements (visible ones), in viewport pixels. */
    window.tutRect = (sels) => {
      let r = null;
      for (const s of sels) {
        const els = typeof s === 'string' ? [...document.querySelectorAll(s)] : [s];
        for (const e of els) {
          const b = e.getBoundingClientRect();
          if (!b.width && !b.height) continue;
          r = r ? {l: Math.min(r.l, b.left), t: Math.min(r.t, b.top), r: Math.max(r.r, b.right), b: Math.max(r.b, b.bottom)}
            : {l: b.left, t: b.top, r: b.right, b: b.bottom};
        }
      }
      return r && {x: r.l, y: r.t, w: r.r - r.l, h: r.b - r.t};
    };
    /** The innermost element whose own text is exactly t (inside root). */
    window.tutText = (t, root) => [...(root ? document.querySelector(root) : document.body).querySelectorAll('*')]
      .find((e) => [...e.childNodes].some((n) => n.nodeType === 3 && n.textContent.trim() === t));
  }, {css: OVERLAY_CSS, cursor: CURSOR_SVG});
}

let page;
const pause = (ms) => page.waitForTimeout(ms);
/** Spotlight one rectangle (padding pad); null dims everything; 'none' lifts the dim. */
async function spot(rect, pad = 8) {
  await page.evaluate(({rect, pad}) => {
    const hole = document.getElementById('tut-hole');
    const ring = document.getElementById('tut-ring');
    if (rect === 'none' || !rect) {
      hole.classList.add('full');
      hole.classList.toggle('off', rect === 'none');
      hole.style.cssText = '';
      ring.classList.remove('on');
      return;
    }
    hole.classList.remove('full', 'off');
    const box = `left:${rect.x - pad}px;top:${rect.y - pad}px;width:${rect.w + 2 * pad}px;height:${rect.h + 2 * pad}px`;
    hole.style.cssText = box;
    ring.style.cssText = `left:${rect.x - pad - 2}px;top:${rect.y - pad - 2}px;width:${rect.w + 2 * pad}px;height:${rect.h + 2 * pad}px`;
    ring.classList.add('on');
  }, {rect, pad});
}
const rectOf = (...sels) => page.evaluate((s) => window.tutRect(s), sels);
const textRect = (t, root) => page.evaluate(({t, root}) => { const e = window.tutText(t, root); return e && window.tutRect([e]); }, {t, root});
/** A feel field's whole label: its name, its hint, its slider and its number box. */
const fieldRect = (name) => page.evaluate((n) => window.tutRect([document.querySelector(`input[name="${n}"]`).closest('label')]), name);
/** The Driving feel card at the top of the middle column (smoothly). */
async function showFeel() {
  const moved = await page.evaluate(() => {
    const mid = document.querySelector('.controls-mid');
    const card = document.querySelector('.feel-card');
    const top = mid.scrollTop + card.getBoundingClientRect().top - mid.getBoundingClientRect().top - 16;
    if (Math.abs(top - mid.scrollTop) < 4) return false;
    mid.scrollTo({top, behavior: 'smooth'});
    return true;
  });
  if (moved) await pause(900);
}
let cursorAt = {x: 640, y: 400};
async function moveCursor(x, y, ms = 750) {
  cursorAt = {x, y};
  await page.evaluate(({x, y}) => { document.getElementById('tut-cursor').style.transform = `translate(${x - 5}px,${y - 3}px)`; }, {x, y});
  await page.mouse.move(x, y);
  await pause(ms);
}
async function ripple(x, y) {
  await page.evaluate(({x, y}) => {
    const r = document.createElement('div'); r.className = 'tut-ripple'; r.style.left = `${x}px`; r.style.top = `${y}px`;
    document.documentElement.append(r); setTimeout(() => r.remove(), 700);
  }, {x, y});
}
/** Move the cursor to the centre of rect, then click there (the ripple with the click). */
async function clickAt(rect) {
  const x = rect.x + rect.w / 2, y = rect.y + rect.h / 2;
  await moveCursor(x, y);
  await ripple(x, y);
  await page.mouse.click(x, y);
}
async function caption(html) {
  await page.evaluate((h) => { const c = document.getElementById('tut-cap'); c.innerHTML = h ? `<span>${h}</span>` : ''; c.style.opacity = h ? '1' : '0'; }, html);
}
async function card(html) {
  await page.evaluate((h) => { const c = document.getElementById('tut-card'); if (h) c.innerHTML = h; c.classList.toggle('on', !!h); }, html);
}
async function keyChip(text, rect) {
  await page.evaluate(({text, rect}) => {
    const k = document.getElementById('tut-key');
    if (!text) { k.classList.remove('on'); return; }
    k.textContent = text; k.style.left = `${rect.x - 20}px`; k.style.top = `${rect.y + rect.h + 16}px`; k.classList.add('on');
  }, {text, rect});
}
/** Press the mouse at (x0, y0) and drag to (x1, y1) in steps, the drawn cursor following. */
async function drag(x0, y0, x1, y1, ms = 900) {
  await moveCursor(x0, y0, 600);
  // a text selection left by an earlier drag turns the next press on it into the browser's own drag (no more
  // pointermove): clear it first, as a click elsewhere would
  await page.evaluate(() => { window.getSelection().removeAllRanges(); document.getElementById('tut-cursor').classList.add('drag'); });
  await page.mouse.down();
  const n = Math.max(8, Math.round(ms / 40));
  for (let i = 1; i <= n; i++) {
    const t = i / n, e = t < .5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    const x = x0 + (x1 - x0) * e, y = y0 + (y1 - y0) * e;
    await page.evaluate(({x, y}) => { document.getElementById('tut-cursor').style.transform = `translate(${x - 5}px,${y - 3}px)`; }, {x, y});
    await page.mouse.move(x, y);
    await pause(ms / n);
  }
  await page.mouse.up();
  await page.evaluate(() => { window.getSelection().removeAllRanges(); document.getElementById('tut-cursor').classList.remove('drag'); });
  cursorAt = {x: x1, y: y1};
}
const readout = () => page.locator('.readout strong').textContent();
/** **word** -> the accent, bold. */
const md = (s) => s.replace(/\*\*(.+?)\*\*/g, '<b>$1</b>').replace(/→/g, '&rarr;');

// ---- each beat's actions (the narration plays from the beat's start; the beat lasts its audio + 0.7 s at least) ----
const PANE = '#controls .pane.right';
const ACTIONS = {
  async c01() {
    await spot(null);
    await card('<h2>The Controls tab</h2><div class="row"><span class="k">1</span><span>Which button does what</span></div>'
      + '<div class="row"><span class="k">2</span><span>How the sticks feel</span></div>');
  },
  async c02() {
    await card(null);
    const r = await page.getByRole('tab', {name: 'Controls'}).boundingBox();
    await spot({x: r.x, y: r.y, w: r.width, h: r.height}, 4);
    await pause(400);
    await clickAt({x: r.x, y: r.y, w: r.width, h: r.height});
    await page.locator('svg.pad').waitFor();
  },
  async c03() {
    await spot(await rectOf('svg.pad'), 10);
    await moveCursor(620, 380, 900);
  },
  async c04() {
    await spot(await rectOf('[data-button="R1"]'), 6);
    await pause(300);
    await clickAt(await rectOf('[data-button="R1"] .cbtn-hit'));
    await pause(500);
    await spot(await rectOf(`${PANE} .insp-title`, `${PANE} select`), 10);
    await moveCursor(1100, 300, 700);
  },
  async c05() {
    await spot(await rectOf('[data-button="A"]'), 6);
    await pause(300);
    await clickAt(await rectOf('[data-button="A"] .cbtn-hit'));
    await pause(500);
    await spot(await rectOf(`${PANE} .insp-title`, `${PANE} select`), 10);
    await moveCursor(1100, 300, 700);
  },
  async c06() {
    await spot(await rectOf('[data-button="Up"]', '[data-button="Down"]', '[data-button="Left"]', '[data-button="Right"]'), 8);
    await moveCursor(500, 300, 800);
  },
  async c07() {
    const up = await rectOf('[data-button="Up"] .cbtn-hit');
    await spot(up, 6);
    await pause(300);
    await clickAt(up);
    await page.locator('.popover').waitFor();
    await pause(450);
    await spot(await rectOf('.popover'), 4);
    await moveCursor(720, 330, 700);
  },
  async c08() {
    await spot(null);
    await page.keyboard.press('Escape');  // Up is left free: nothing chosen
    await showFeel();
    const inp = await rectOf('input[name="feel-deadband"]');
    await spot(await fieldRect('feel-deadband'), 8);
    await moveCursor(inp.x + inp.w / 2, inp.y + inp.h / 2, 800);
  },
  async c09() {
    const inp = await rectOf('input[name="feel-expo"]');
    await spot(await fieldRect('feel-expo'), 8);
    await pause(300);
    await clickAt(inp);
    await page.locator('input[name="feel-expo"]').selectText();
    await page.keyboard.type('0.6', {delay: 160});
    await page.keyboard.press('Enter');
    await pause(300);
    await showFeel();  // a kept change draws the tab again, from its top
    await spot(await rectOf('svg.graph'), 8);
  },
  async c10() {
    const g = await rectOf('svg.graph', '.readout');
    await spot(g, 8);
    const p = await rectOf('.g-probe');
    const x0 = p.x + p.w / 2, y0 = p.y + p.h / 2;
    await drag(x0, y0, x0 + 45, y0, 900);
    await pause(400);
    await drag(x0 + 45, y0, x0, y0, 900);
    await pause(300);
    const t = await readout();
    if (!/^30 % stick → 12 % speed$/.test(t)) throw new Error(`c10: the readout says "${t}"`);
  },
  async c11() {
    const plot = await rectOf('svg.graph');
    const p = await rectOf('.g-probe');
    const y0 = p.y + p.h / 2;
    await drag(p.x + p.w / 2, y0, plot.x + plot.w - 4, y0, 1000);
    await pause(300);
    const t = await readout();
    if (!/^100 % stick → 100 % speed$/.test(t)) throw new Error(`c11: the readout says "${t}"`);
  },
  async c12() {
    const ex = await rectOf('#export');
    await spot(ex, 6);
    await moveCursor(ex.x + ex.w / 2, ex.y + ex.h / 2, 800);
    await keyChip('Ctrl + E', {x: ex.x - 10, y: ex.y, w: ex.w, h: ex.h});
    await pause(500);
    await page.keyboard.press('Control+e');
    await page.locator('.drawer').waitFor();
    await pause(600);
    await keyChip(null);
  },
  async c13() {
    const btn = page.getByRole('button', {name: 'Write into src/team/robot_one_driver.hpp'});
    await btn.scrollIntoViewIfNeeded();
    await pause(500);
    const r = await page.evaluate(() => {
      const b = [...document.querySelectorAll('.drawer button')].find((e) => e.textContent.includes('Write into'));
      return window.tutRect([b.closest('.card')]);
    });
    await spot(r, 6);
    const b = await btn.boundingBox();
    await moveCursor(b.x + b.width / 2, b.y + b.height / 2, 800);  // pointed at, never pressed
  },
  async c14() {
    await spot(null);
    await page.evaluate(() => document.querySelector('.drawer-body').scrollTo({top: 0, behavior: 'smooth'}));
    await pause(700);
    const row = await page.evaluate(() => window.tutRect([document.querySelector('.drawer .region .row')]));
    await spot(row, 8);
    const c = await page.evaluate(() => window.tutRect([document.querySelector('.drawer .region .row button')]));
    await moveCursor(c.x + c.w / 2, c.y + c.h / 2, 800);
  },
  async c15() {
    await spot(null);
    await page.keyboard.press('Escape');
    await page.locator('.drawer').waitFor({state: 'detached'}).catch(() => {});
    await pause(500);
    const two = await page.getByRole('radio', {name: 'two'}).boundingBox();
    const sw = await page.evaluate(() => window.tutRect([document.querySelector('[role="radio"]').parentElement]));
    await spot(sw, 6);
    await pause(300);
    await clickAt({x: two.x, y: two.y, w: two.width, h: two.height});
    await pause(700);
    const title = await textRect("robot two's controller");
    await spot(title, 10);
  },
  async c16() {
    await spot(null);
    await showFeel();
    const inp = await rectOf('input[name="feel-expo"]');
    await spot(await fieldRect('feel-expo'), 8);
    await pause(200);
    await clickAt(inp);
    await page.locator('input[name="feel-expo"]').selectText();
    await page.keyboard.type('0.6', {delay: 140});
    await page.keyboard.press('Enter');
    await pause(400);
    const ex = await rectOf('#export');
    await spot(ex, 6);
    await moveCursor(ex.x + ex.w / 2, ex.y + ex.h / 2, 700);
    await page.keyboard.press('Control+e');
    await page.locator('.drawer').waitFor();
    await pause(500);
    await spot(await textRect("Export robot two's controls"), 10);
  },
  async c17() {
    await spot(null);
    await page.keyboard.press('Escape');
    await moveCursor(1180, 600, 600);
    await card('<h2>Then on the robot</h2><div class="row"><span class="k">1</span><span>Build</span></div>'
      + '<div class="row"><span class="k">2</span><span>Upload <b>shulib Drive</b> to slot 1</span></div>'
      + '<div class="row"><span class="k">3</span><span>Upload <b>Competition</b> to slot 6</span></div>'
      + '<div class="row"><span class="k">4</span><span>Check each button once</span></div>'
      + '<div class="credit">Music: "Wallpaper" by Kevin MacLeod (incompetech.com), CC BY 4.0. Voice: Kokoro.</div>');
  },
};

const launchServer = await chromium.launchServer({args: ['--font-render-hinting=none']});
const pid = launchServer.process().pid;
console.log(`chromium pid ${pid}, site ${base}`);
const browser = await chromium.connect(launchServer.wsEndpoint());
try {
  const context = await browser.newContext({viewport: {width: 1280, height: 720}, recordVideo: {dir: out, size: {width: 1280, height: 720}}});
  page = await context.newPage();
  const opened = Date.now();
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(`${base}/app.html`);
  await page.getByRole('button', {name: 'Got it'}).click();
  // Edge/Chrome with the shulib folder connected, in memory (the drivers' files as they are): Export shows its
  // "Write into" button; nothing is written anywhere
  const files = Object.fromEntries(['one', 'two'].map((r) => [`src/team/robot_${r}_driver.hpp`, fs.readFileSync(path.join(driverDir, `robot_${r}_driver.hpp`), 'utf8')]));
  await page.evaluate((f) => window.routeBuilder.useMemoryFolder(f), files);
  await page.waitForTimeout(2500);  // its toasts go
  await page.evaluate(() => document.querySelectorAll('.toast').forEach((t) => t.remove()));
  await installOverlay(page);
  await page.mouse.move(640, 400);
  await page.waitForTimeout(600);
  const lead = (Date.now() - opened) / 1000;
  const starts = [];
  for (const b of beats) {
    const t0 = Date.now();
    starts.push((t0 - opened) / 1000 - lead);
    await caption(md(b.caption));
    await ACTIONS[b.id]();
    const used = (Date.now() - t0) / 1000;
    const left = Math.max(b.dur - used, 1.0);  // hold the finished state at least 1 s
    if (used + 1.0 > b.dur) console.log(`${b.id}: actions took ${used.toFixed(2)} s of ${b.dur.toFixed(2)} s`);
    await pause(left * 1000);
  }
  await caption('');
  await card(null);
  await pause(1200);
  const total = (Date.now() - opened) / 1000 - lead;
  if (errors.length) console.log('page errors:', errors);
  const video = page.video();
  await context.close();
  const webm = path.join(out, 'raw.webm');
  await video.saveAs(webm);
  // ---- the sound: each beat's narration at its start, the music under it; then H.264 + AAC ----
  const inputs = ['-ss', lead.toFixed(3), '-i', webm, '-stream_loop', '-1', '-i', music];
  const parts = [];
  beats.forEach((b, i) => {
    inputs.push('-i', b.wav);
    const ms = Math.round((starts[i] + 0.25) * 1000);
    parts.push(`[${i + 2}:a]aresample=48000,aformat=channel_layouts=mono,adelay=${ms}:all=1[n${i}]`);
  });
  const T = total.toFixed(3);
  const filter = [
    ...parts,
    `${beats.map((_, i) => `[n${i}]`).join('')}amix=inputs=${beats.length}:normalize=0:duration=longest,apad,atrim=0:${T}[voice]`,
    `[1:a]aresample=48000,aformat=channel_layouts=mono,atrim=0:${T},volume=0.07,afade=t=in:d=2,afade=t=out:st=${(total - 3).toFixed(3)}:d=3[bed]`,
    `[voice][bed]amix=inputs=2:normalize=0:duration=first,alimiter=limit=0.95[a]`,
    `[0:v]trim=0:${T},setpts=PTS-STARTPTS,fps=30,format=yuv420p[v]`,
  ].join(';');
  execFileSync(ffmpeg, ['-y', '-loglevel', 'error', ...inputs, '-filter_complex', filter, '-map', '[v]', '-map', '[a]',
    '-c:v', 'libx264', '-preset', 'slow', '-crf', '28', '-pix_fmt', 'yuv420p', '-c:a', 'aac', '-b:a', '128k', '-ac', '1',
    '-movflags', '+faststart', '-t', T, mp4], {stdio: 'inherit'});
  const pIdx = beats.findIndex((b) => b.id === 'c04');
  const at = (starts[pIdx] + beats[pIdx].dur - 0.6).toFixed(2);
  execFileSync(ffmpeg, ['-y', '-loglevel', 'error', '-ss', at, '-i', mp4, '-frames:v', '1', '-q:v', '4', jpg], {stdio: 'inherit'});
  fs.writeFileSync(path.join(out, 'timeline.json'), JSON.stringify(beats.map((b, i) => ({id: b.id, caption: b.caption, start: +starts[i].toFixed(2), audio: +b.len.toFixed(2)})), null, 1));
  console.log(`${mp4}\n${jpg}\nwebm ${webm}, lead ${lead.toFixed(2)} s, total ${T} s, timeline ${path.join(out, 'timeline.json')}`);
} finally {
  await browser.close();
  await launchServer.close();
  try { process.kill(pid, 'SIGKILL'); } catch { /* already gone */ }
  server.close();
}
