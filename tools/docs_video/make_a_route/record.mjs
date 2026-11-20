// Records the narrated "Make a route and run it on the robot" tutorial: the REAL route builder's Route tab (the built
// site, its PREVIEW engine included), driven by Playwright in Chromium, with an overlay injected into the page (a
// spotlight, a pulsing ring, a cursor, a click ripple and a caption band: never the app's code), then three drawn
// closing cards (VS Code's Run Task, its terminal with tools/upload_auton.py's own words, the brain), clearly labelled
// as drawn; ffmpeg puts each beat's narration at its start, with the music under it.
//
//   node tools/docs_video/make_a_route/record.mjs [--site DIR] [--out DIR]
//
// The app's toasts are hidden while it records (one idea on the screen at a time); everything else is the page as is.
//
// Needs: narration/<id>.wav (narrate.py, from beats.json); the built site WITH its engine (--site, default
// build/route_maker_site: python3 tools/route_maker/build.py ... --engine build/wasm); Playwright through
// PW_NODE_MODULES (else tools/route_maker/node_modules); MUSIC (default ~/shulib-render-tmp/music/Wallpaper.mp3);
// FFMPEG (default ffmpeg); TEAM_DIR, the folder whose skills_route.cpp and autons files fill the in-memory "shulib
// folder" (default src/team). Writes docs/assets/video/make-a-route.mp4 and make-a-route.jpg. The page writes only
// into that in-memory folder (the page tests' hook): nothing on disk.
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
const teamDir = process.env.TEAM_DIR || path.join(repo, 'src', 'team');
const out = opt('--out') || fs.mkdtempSync(path.join(os.tmpdir(), 'make-a-route-'));
fs.mkdirSync(out, {recursive: true});
const mp4 = path.join(repo, 'docs', 'assets', 'video', 'make-a-route.mp4');
const jpg = path.join(repo, 'docs', 'assets', 'video', 'make-a-route.jpg');

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
.drawer-body{padding-bottom:76px!important}
.toast{visibility:hidden!important}
#tut-scene{position:fixed;inset:0;z-index:2147483638;pointer-events:none;opacity:0;transition:opacity .4s;background:#14101f;
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:22px;font:500 22px/1.4 system-ui,"Segoe UI",Roboto,sans-serif;color:#eee}
#tut-scene.on{opacity:1}
#tut-scene .tag{font-size:15px;letter-spacing:.06em;text-transform:uppercase;color:#b9a8e6;border:1px solid #5b4b8a;border-radius:20px;padding:4px 14px}
#tut-scene .win{background:#1f1f1f;border:1px solid #3c3c3c;border-radius:10px;width:880px;max-width:92vw;min-height:300px;position:relative;
  box-shadow:0 20px 60px rgba(0,0,0,.5);font-size:18px}
#tut-scene .bar{display:flex;gap:22px;padding:10px 16px;background:#2b2b2b;border-bottom:1px solid #3c3c3c;border-radius:10px 10px 0 0;color:#ccc}
#tut-scene .bar .on{color:#fff;background:#3d3d3d;border-radius:5px;padding:0 8px;margin:0 -8px}
#tut-scene .menu{position:absolute;left:150px;top:46px;background:#252526;border:1px solid #454545;border-radius:6px;padding:6px 0;width:260px}
#tut-scene .menu div,#tut-scene .pick div{padding:6px 16px}
#tut-scene .hl{background:#04395e;color:#fff}
#tut-scene .pick{position:absolute;left:300px;top:70px;width:500px;background:#252526;border:1px solid #454545;border-radius:6px;opacity:0;
  transition:opacity .3s;box-shadow:0 8px 24px rgba(0,0,0,.5)}
#tut-scene .pick.on{opacity:1}
#tut-scene .pick .q{color:#9d9d9d;font-size:15px;border-bottom:1px solid #454545}
#tut-scene .term{padding:18px 20px;font:16px/1.7 ui-monospace,"Cascadia Mono",Consolas,monospace;color:#cfcfcf;min-height:0}
#tut-scene .term .ok{color:#7ee787;font-weight:700}
#tut-scene .flow{display:flex;align-items:center;gap:22px}
#tut-scene .flow .b{background:#26203a;border:2px solid #6b5aa6;border-radius:14px;padding:22px 28px;font-weight:700;font-size:26px;text-align:center}
#tut-scene .flow .b small{display:block;font-weight:500;font-size:16px;color:#b9a8e6;margin-top:4px}
#tut-scene .flow .run{background:#2e7d32;border-color:#66bb6a;color:#fff}
#tut-scene .flow .arr{font-size:34px;color:#b9a8e6}
#tut-scene .credit{position:absolute;bottom:84px;font-size:14px;color:#a9a3bd}
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
const R = '.pane.right';
/** The screen point of field inches (x, y). */
const fieldPoint = (x, y) => page.evaluate(([fx, fy]) => {
  const pic = JSON.parse(document.getElementById('shulib-field').textContent);
  const svg = document.querySelector('.field-svg');
  const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(svg.getScreenCTM());
  return {x: pt.x, y: pt.y};
}, [x, y]);
/** A rectangle around field inches (x0, y0)-(x1, y1). */
async function fieldBox(x0, y0, x1, y1) {
  const a = await fieldPoint(x0, y1), b = await fieldPoint(x1, y0);
  return {x: a.x, y: a.y, w: b.x - a.x, h: b.y - a.y};
}
const tool = (name) => rectOf(`button.tool[aria-label^="${name} ("]`);
const box = (b) => ({x: b.x, y: b.y, w: b.width, h: b.height});
/** Click the element (cursor, ripple), spotlighting it first. */
async function press(rect, pad = 6, wait = 250) {
  await spot(rect, pad);
  await pause(wait);
  await clickAt(rect);
}
/** A number box on the right, with its label (the field it sits in). */
const fld = (name) => page.evaluate((n) => { const i = document.querySelector(`.pane.right input[name="${n}"], .pane.right select[name="${n}"]`); return window.tutRect([i.closest('.fld') || i]); }, name);
/** Type a number into a box on the right, as a student does (click, type, Enter). */
async function typeBox(name, value, quick) {
  const inp = page.locator(`${R} input[name="${name}"]`);
  await spot(await fld(name), 8);
  await pause(quick ? 150 : 300);
  await clickAt(box(await inp.boundingBox()));
  await inp.selectText().catch(() => {});
  await page.keyboard.type(String(value), {delay: quick ? 110 : 150});
  await page.keyboard.press('Enter');
  await pause(300);
}
/** Mechanism -> one item of its menu. */
async function mechItem(re) {
  await press(await tool('Mechanism'), 4, 200);
  await page.locator('.popover').waitFor();
  await pause(400);
  const item = page.locator('.popover .menu-item', {hasText: re});
  await item.scrollIntoViewIfNeeded();
  await press(box(await item.boundingBox()), 4, 400);
  await pause(400);
}
const stepItem = (re) => page.locator('.pane.left .step', {hasText: re});
async function key(label, k, near) {
  await keyChip(label, near);
  await pause(350);
  await page.keyboard.press(k);
  await pause(450);
}
/** A drawn scene over the page (the closing cards: VS Code, its terminal, the brain), never the app. */
async function scene(html) {
  await page.evaluate((h) => {
    let s = document.getElementById('tut-scene');
    if (!s) { s = document.createElement('div'); s.id = 'tut-scene'; document.documentElement.append(s); }
    s.innerHTML = h;
    s.classList.toggle('on', !!h);
  }, html);
  await pause(450);
}
const sceneRect = (sel) => page.evaluate((s) => window.tutRect([document.querySelector(`#tut-scene ${s}`)]), sel);
const TERMINAL_LINES = [
  "Building robot one's Auton Tester: make ROBOT=one PROGRAM=autontest (a few minutes the first time; Ctrl+C stops it)...",
  '✓ BUILT',
  'Uploading to slot 2...',
  '✓ UPLOADED to slot 2. On the brain: shulib Auton Tester → skills → RUN',
];
const esc = (t) => t.replace(/&/g, '&amp;').replace(/</g, '&lt;');

const ACTIONS = {
  async m01() {
    await spot(await rectOf('.coach'), 8);
    await moveCursor(640, 420, 900);
  },
  async m02() {
    const g = page.getByRole('button', {name: 'Got it'});
    await press(box(await g.boundingBox()), 4, 200);
    await pause(500);
    await spot(await rectOf('.field-svg'), 4);
    await moveCursor(620, 600, 800);
  },
  async m03() {
    await press(await rectOf('#route-switch'), 4, 250);
    await pause(500);
    await spot(await rectOf('.popover'), 4);
    await pause(1600);
  },
  async m04() {
    await spot(null);
    await page.keyboard.press('Escape');
    await pause(300);
    const p = await fieldPoint(-36, -48);
    await moveCursor(p.x, p.y, 800);
    await ripple(p.x, p.y);
    await page.mouse.click(p.x, p.y);
    await pause(500);
    await spot(await fieldBox(-48, -60, -24, -36), 6);
    await moveCursor(p.x + 90, p.y + 30, 600);
  },
  async m05() {
    await typeBox('startH', 90);
    await pause(300);
    await spot(await fieldBox(-48, -60, -24, -36), 6);
  },
  async m06() {
    const q = page.locator(`${R} .banner, ${R} [class*="alert"]`).filter({hasText: 'SAME time'}).first();
    const qb = await q.count() ? box(await q.boundingBox()) : await rectOf(`${R} [data-together]`);
    await spot(qb, 6);
    await pause(2200);
    const no = page.getByRole('button', {name: 'No, one at a time'}).first();
    await clickAt(box(await no.boundingBox()));
    await pause(400);
    await page.evaluate(() => document.querySelector('.pane.right [data-together="false"]').scrollIntoView({block: 'center', behavior: 'smooth'}));
    await pause(700);
    await spot(await rectOf(`${R} [data-together="false"]`), 6);
  },
  async m07() {
    await mechItem(/^Home it/);
    await typeBox('boundS', 2);
    await spot(await rectOf('.pane.left .step.selected'), 6);
  },
  async m08() {
    await press(await tool('Drive'), 4, 250);
    await pause(300);
    const p = await fieldPoint(-36, -36);
    await moveCursor(p.x, p.y, 800);
    await ripple(p.x, p.y);
    await page.mouse.click(p.x, p.y);
    await pause(500);
    await moveCursor(p.x + 120, p.y - 60, 500);
  },
  async m09() {
    await spot(await fieldBox(-46, -58, -26, -26), 6);
    await pause(400);
  },
  async m10() {
    await press(await tool('Straight'), 4, 250);
    await pause(400);
    await typeBox('inches', 3);
    await spot(await rectOf('.pane.left .step.selected'), 6);
  },
  async m11() {
    await press(box(await stepItem('Drive to (-36, -36)').boundingBox()), 4, 250);
    await pause(400);
    await spot(await fld('y'), 8);
    await moveCursor(1180, 330, 500);
    await key('↑', 'ArrowUp', await fld('y'));
    // steps are added after the selected one: the drive is selected, so Straight comes next, in order
    await pause(500);
  },
  async m12() {
    const near = await fld('y');
    await keyChip('Shift + ↓', near);
    for (let i = 0; i < 4; i++) {
      await page.keyboard.press('Shift+ArrowDown');
      await pause(420);
    }
    await pause(300);
    await keyChip(null);
    const y = (await page.evaluate(() => window.routeBuilder.route())).steps[1].y;
    if (y !== -36) throw new Error(`m12: the drive's y is ${y}, not -36`);
    await keyChip(null);
  },
  async m13() {
    await press(await tool('Turn'), 4, 250);
    await pause(300);
    const p = await fieldPoint(0, -33);
    await moveCursor(p.x, p.y, 800);
    await ripple(p.x, p.y);
    await page.mouse.click(p.x, p.y);
    await pause(500);
    await spot(await fieldBox(-46, -43, 4, -23), 6);
    await moveCursor(p.x + 60, p.y + 80, 500);
  },
  async m14() {
    await mechItem(/^Go up/);
    await typeBox('up', 12);
    await typeBox('boundS', 2, true);
    await spot(await rectOf('.pane.left .step.selected'), 6);
  },
  async m15() {
    await mechItem(/^Back to the bottom/);
    await typeBox('boundS', 2);
    await spot(await rectOf('.pane.left .step.selected'), 6);
  },
  async m16() {
    await mechItem(/^Open claw$/);
    await mechItem(/^Close claw$/);
    await spot(await rectOf('.pane.left .step.selected'), 6);
    const order = (await page.evaluate(() => window.routeBuilder.route())).steps.map((x) => x.do).join(' ');
    if (order !== 'home driveTo forward face move move set set') throw new Error(`m16: the steps are ${order}`);
  },
  async m17() {
    await press(box(await stepItem('Drive to (-36, -36)').boundingBox()), 4, 250);
    await pause(400);
    const sp = page.locator(R).getByText('Speed, time limit and how close');
    await press(box(await sp.boundingBox()), 4, 250);
    await pause(400);
    await typeBox('Time limit', 1.5);
    if (!(await page.locator(`${R} input[name="Time limit"]`).isVisible())) {
      await clickAt(box(await page.locator(R).getByText('Speed, time limit and how close').boundingBox()));
      await pause(400);
    }
    await spot(await fld('Time limit'), 8);
    const s = (await page.evaluate(() => window.routeBuilder.route())).steps[1];
    if (!JSON.stringify(s).includes('1.5')) throw new Error(`m17: the drive kept no 1.5 s: ${JSON.stringify(s)}`);
    console.log('m17 drive step', JSON.stringify(s));
  },
  async m18() {
    const sel = page.locator(`${R} select[name="onFail"]`);
    await sel.scrollIntoViewIfNeeded();
    await pause(300);
    await spot(await fld('onFail'), 8);
    await pause(300);
    await clickAt(box(await sel.boundingBox()));
    await sel.selectOption('carryOn');
    await pause(500);
    await spot(await fld('onFail'), 8);
    const s = (await page.evaluate(() => window.routeBuilder.route())).steps[1];
    if (s.onFail !== 'carryOn' && JSON.stringify(s).indexOf('carryOn') < 0) throw new Error(`m18: ${JSON.stringify(s)}`);
  },
  async m19() {
    await press(await rectOf('#preview'), 4, 250);
    await page.getByText('FINISHED', {exact: true}).waitFor({timeout: 20000});
    await pause(300);
    await spot(await rectOf('.field-svg'), 4);
    await moveCursor(700, 640, 600);
  },
  async m20() {
    // from the cascade's step (click a step in the list: the run goes to where it began, and plays on)
    await spot(null);
    await stepItem('go up 12 in').scrollIntoViewIfNeeded();
    await press(box(await stepItem('go up 12 in').boundingBox()), 4, 200);
    if (await page.locator('.dock button.play[aria-label*="Play"], .dock button.play[title*="Play"]').count()) await page.keyboard.press('Space');
    await pause(300);
    await spot(await rectOf('.mech-strip'), 6);
    await moveCursor(1000, 640, 500);
  },
  async m21() {
    await spot(await rectOf('.dock .track'), 8);
    await pause(400);
  },
  async m22() {
    await spot(null);
    await page.keyboard.press('Escape');  // the preview closes
    await pause(500);
    const ex = await rectOf('#export');
    await spot(ex, 6);
    await moveCursor(ex.x + ex.w / 2, ex.y + ex.h / 2, 800);
    await page.locator('.field-svg').focus();
    await key('Ctrl + E', 'Control+e', ex);
    await page.locator('.drawer').waitFor();
    await keyChip(null);
    await pause(500);
    await spot(await page.evaluate(() => { const h = [...document.querySelectorAll('.drawer *')].find((e) => e.children.length === 0 && e.textContent.trim() === "Put it into your robot's code"); return window.tutRect([h.closest('.card') || h.parentElement]); }), 6);
    await moveCursor(900, 600, 500);
  },
  async m23() {
    const c = page.locator('.drawer').getByRole('button', {name: 'Connect your shulib folder'});
    await press(box(await c.boundingBox()), 6, 1200);
    await pause(700);
    await spot(await page.evaluate(() => window.tutRect([[...document.querySelectorAll('.drawer *')].find((e) => e.children.length === 0 && e.textContent.trim() === 'Connected: shulib')])), 10);
  },
  async m24() {
    const w = page.getByRole('button', {name: 'Write into src/team/skills_route.cpp'});
    await press(box(await w.boundingBox()), 6, 1500);
    await page.locator('.drawer [data-result="ok"]').waitFor();
    await pause(400);
    await spot(await page.evaluate(() => {
      const ok = document.querySelector('.drawer [data-result="ok"]');
      const h = [...ok.querySelectorAll('*')].filter((e) => /^(Written and checked|1 route)/.test(e.textContent.trim()) && e.children.length <= 1);
      return window.tutRect(h.length ? h : [ok]);
    }), 10);
    await moveCursor(1100, 640, 500);
  },
  async m25() {
    await spot(await page.evaluate(() => {
      const li = [...document.querySelectorAll('.drawer .xr-build *')].find((e) => /read back after writing/.test(e.textContent) && e.children.length <= 1);
      return window.tutRect([li]);
    }), 8);
  },
  async m26() {
    await spot(await rectOf('.drawer .xr-build'), 8);
  },
  async m27() {
    const next = await page.evaluate(() => {
      const h = [...document.querySelectorAll('.drawer *')].find((e) => e.children.length === 0 && /^Next: build and upload$/.test(e.textContent.trim()));
      const card = h.closest('.card, section, .xr-next') || h.parentElement;
      card.scrollIntoView({block: 'start', behavior: 'smooth'});
      return true;
    });
    void next;
    await pause(900);
    await spot(await page.evaluate(() => {
      const h = [...document.querySelectorAll('.drawer *')].find((e) => e.children.length === 0 && /^Next: build and upload$/.test(e.textContent.trim()));
      const card = h.closest('.card, section, .xr-next') || h.parentElement;
      const r = card.getBoundingClientRect();
      return {x: r.left, y: Math.max(r.top, 70), w: r.width, h: Math.min(r.bottom, 655) - Math.max(r.top, 70)};
    }), 6);
  },
  async m28() {
    await spot(null);
    await page.keyboard.press('Escape');
    await pause(500);
    const sv = await rectOf('button[aria-label^="Save"], button[title^="Save"]');
    await spot(sv, 6);
    await moveCursor(sv.x + sv.w / 2, sv.y + sv.h / 2, 700);
    await key('Ctrl + S', 'Control+s', sv);
    await keyChip(null);
    const pill = await page.evaluate(() => { const e = [...document.querySelectorAll('.topbar *')].find((x) => x.children.length === 0 && /^Saved/.test(x.textContent.trim())); return e && window.tutRect([e]); });
    if (pill) await spot({x: Math.min(pill.x, sv.x), y: Math.min(pill.y, sv.y), w: Math.max(pill.x + pill.w, sv.x + sv.w) - Math.min(pill.x, sv.x), h: Math.max(pill.y + pill.h, sv.y + sv.h) - Math.min(pill.y, sv.y)}, 6);
  },
  async m29() {
    await spot(null);
    await moveCursor(1180, 640, 400);
    await scene(`<div class="tag">Drawn: VS Code, in your shulib folder</div>
      <div class="win"><div class="bar"><span>File</span><span>Edit</span><span>View</span><span class="on" id="s-term">Terminal</span><span>Help</span></div>
      <div class="menu" id="s-menu"><div>New Terminal</div><div class="hl" id="s-run">Run Task...</div><div>Run Build Task...</div></div>
      <div class="pick" id="s-pick"><div class="q">Select the task to run</div><div class="hl" id="s-one">Upload routes to robot one</div><div>Upload routes to robot two</div></div></div>`);
    await spot(await sceneRect('#s-term'), 6);
    await pause(900);
    await spot(await sceneRect('#s-run'), 6);
    await pause(900);
    await page.evaluate(() => document.getElementById('s-pick').classList.add('on'));
    await pause(300);
    await spot(await sceneRect('#s-one'), 6);
  },
  async m30() {
    await scene(`<div class="tag">Drawn: VS Code's terminal (tools/upload_auton.py's own words)</div>
      <div class="win term">${TERMINAL_LINES.map((l, i) => `<div id="t${i}" class="${l.startsWith('✓') ? 'ok' : ''}">${esc(l)}</div>`).join('')}</div>`);
    await spot(await sceneRect('#t3'), 6);
  },
  async m31() {
    await scene(`<div class="tag">Drawn: on the robot's brain</div>
      <div class="flow"><div class="b" id="b0">shulib Auton Tester<small>slot 2</small></div><div class="arr">→</div>
      <div class="b" id="b1">your route<small>skills</small></div><div class="arr">→</div><div class="b run" id="b2">RUN</div></div>
      <div class="credit">Music: "Wallpaper" by Kevin MacLeod (incompetech.com), CC BY 4.0. Voice: Kokoro.</div>`);
    for (const id of ['#b0', '#b1', '#b2']) {
      await spot(await sceneRect(id), 8);
      await pause(1000);
    }
    await spot('none');
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
  // Edge/Chrome with the shulib folder NOT connected yet, in memory (the team files as they are): Export's
  // "Connect your shulib folder" connects it, and "Write into" writes into memory only
  const files = Object.fromEntries(['skills_route.cpp', 'robot_one_autons.cpp', 'autons.cpp', 'robot_one_driver.hpp', 'robot_two_driver.hpp']
    .map((f) => [`src/team/${f}`, fs.readFileSync(path.join(teamDir, f), 'utf8')]));
  await page.evaluate((f) => window.routeBuilder.useMemoryFolder(f, {connected: false}), files);
  await page.locator('.coach').waitFor();
  await page.waitForTimeout(800);
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
  const pIdx = beats.findIndex((b) => b.id === 'm19');
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
