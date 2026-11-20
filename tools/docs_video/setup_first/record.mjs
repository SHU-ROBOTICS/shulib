// Records index.html (the "Set up your robot first" tutorial) to docs/assets/video/setup-first.mp4 and its
// poster, setup-first.jpg: Playwright's recordVideo in Chromium, then ffmpeg to H.264 with the narration and
// the music mixed in.
//
//   node record.mjs [--music <file>]          the video and the poster
//   node record.mjs --stills 3,60 [--out DIR] PNGs of those moments (seconds)
//   node record.mjs --reencode --out DIR      encode and mix DIR's last recording again (no new recording)
//
// The beats come from beats.json; each beat lasts its narration (narration/<id>.wav) + 0.7 s. The narration is
// placed at each beat's start; the music runs under it at 0.07 (0.15 with no narration), fading in over 2 s and
// out over the last 3 s, looped if it is shorter than the video. Without --music and narration, no audio track.
// Playwright is found through PW_NODE_MODULES (a node_modules folder that has it), else the route builder's
// (tools/route_maker/node_modules). FFMPEG names ffmpeg (default: ffmpeg on PATH), FFPROBE ffprobe.
import {createRequire} from 'node:module';
import {execFileSync} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath, pathToFileURL} from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '..', '..', '..');
const modules = process.env.PW_NODE_MODULES || path.join(repo, 'tools', 'route_maker', 'node_modules');
const {chromium} = createRequire(path.join(modules, 'noop.js'))('playwright');
const ffmpeg = process.env.FFMPEG || 'ffmpeg';
const ffprobe = process.env.FFPROBE || (ffmpeg.endsWith('ffmpeg') ? ffmpeg.slice(0, -6) + 'ffprobe' : 'ffprobe');

const args = process.argv.slice(2);
const opt = (name) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : undefined; };
const out = opt('--out') || fs.mkdtempSync(path.join(os.tmpdir(), 'setup-first-'));
fs.mkdirSync(out, {recursive: true});
const music = opt('--music');
if (music && !fs.existsSync(music)) throw new Error(`--music: no file ${music}`);
const pageUrl = pathToFileURL(path.join(here, 'index.html')).href;
const mp4 = path.join(repo, 'docs', 'assets', 'video', 'setup-first.mp4');
const jpg = path.join(repo, 'docs', 'assets', 'video', 'setup-first.jpg');

// the beats and each narration file's length
const beats = JSON.parse(fs.readFileSync(path.join(here, 'beats.json'), 'utf8'));
const narr = {};
for (const b of beats) {
  const f = path.join(here, 'narration', `${b.id}.wav`);
  if (!fs.existsSync(f)) continue;
  narr[b.id] = +execFileSync(ffprobe, ['-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', f]).toString().trim();
}
console.log(`${beats.length} beats, ${Object.keys(narr).length} narrated`);

function encode(webm, lead, total, times) {
  const silent = path.join(out, 'video.mp4');
  const ss = Math.max(0, lead - 0.15).toFixed(2);  // trim the page's load (t = 0 was already showing)
  execFileSync(ffmpeg, ['-y', '-loglevel', 'error', '-ss', ss, '-i', webm, '-t', total.toFixed(2), '-r', '30', '-c:v', 'libx264',
    '-preset', 'slow', '-crf', '30', '-tune', 'stillimage', '-pix_fmt', 'yuv420p', '-an', silent], {stdio: 'inherit'});
  // the sound: each narration at its beat's start, the music under it
  const inputs = [], parts = [];
  const narrated = times.filter((b) => narr[b.id] !== undefined);
  narrated.forEach((b, i) => {
    inputs.push('-i', path.join(here, 'narration', `${b.id}.wav`));
    const ms = Math.round(b.start * 1000);
    parts.push(`[${i + 1}:a]aresample=48000,aformat=channel_layouts=stereo,adelay=${ms}|${ms}[n${i}]`);
  });
  const T = total.toFixed(3);
  let mix = null;
  if (narrated.length) {
    parts.push(`${narrated.map((_, i) => `[n${i}]`).join('')}amix=inputs=${narrated.length}:normalize=0:dropout_transition=0,apad,atrim=0:${T}[narr]`);
    mix = '[narr]';
  }
  if (music) {
    inputs.push('-stream_loop', '-1', '-i', music);
    const m = narrated.length + 1;
    const vol = narrated.length ? 0.07 : 0.15;
    parts.push(`[${m}:a]aresample=48000,aformat=channel_layouts=stereo,atrim=0:${T},volume=${vol},afade=t=in:st=0:d=2,afade=t=out:st=${(total - 3).toFixed(3)}:d=3[mus]`);
    if (mix) { parts.push(`[narr][mus]amix=inputs=2:normalize=0,atrim=0:${T}[a]`); mix = '[a]'; } else mix = '[mus]';
  }
  if (mix) {
    execFileSync(ffmpeg, ['-y', '-loglevel', 'error', '-i', silent, ...inputs, '-filter_complex', parts.join(';'), '-map', '0:v', '-map', mix,
      '-c:v', 'copy', '-c:a', 'aac', '-b:a', '128k', '-t', T, '-movflags', '+faststart', mp4], {stdio: 'inherit'});
  } else {
    execFileSync(ffmpeg, ['-y', '-loglevel', 'error', '-i', silent, '-c', 'copy', '-movflags', '+faststart', mp4], {stdio: 'inherit'});
  }
}

if (args.includes('--reencode')) {  // the last recording again (raw.webm + rec-info.json in --out), e.g. after a mix change
  const info = JSON.parse(fs.readFileSync(path.join(out, 'rec-info.json'), 'utf8'));
  encode(path.join(out, 'raw.webm'), info.lead, info.total, info.times);
  console.log(mp4);
  process.exit(0);
}
const server = await chromium.launchServer({args: ['--font-render-hinting=none']});
const pid = server.process().pid;
console.log(`chromium pid ${pid}`);
const browser = await chromium.connect(server.wsEndpoint());
async function open(context) {
  const page = await context.newPage();
  await page.addInitScript(([b, n]) => { window.BEATS = b; window.NARR = n; }, [beats, narr]);
  await page.goto(pageUrl);
  await page.evaluate(() => window.fontsReady);
  return page;
}
try {
  if (args.includes('--stills')) {
    const page = await open(await browser.newContext({viewport: {width: 1280, height: 720}}));
    for (const s of opt('--stills').split(',')) {
      // play the second before it, so the eased camera and highlight settle as they do in the video
      for (let k = 10; k >= 0; k--) await page.evaluate((t) => window.render(t), Math.max(0, +s - k * 0.1));
      const f = path.join(out, `still-${String(s).padStart(6, '0')}.png`);
      await page.screenshot({path: f});
      console.log(f);
    }
    console.log(JSON.stringify(await page.evaluate(() => ({total: window.TOTAL, beats: window.BEAT_TIMES}))));
  } else {
    const context = await browser.newContext({viewport: {width: 1280, height: 720}, recordVideo: {dir: out, size: {width: 1280, height: 720}}});
    const opened = Date.now();
    const page = await open(context);
    await page.waitForTimeout(800);  // the first frame (t = 0) is on screen before the clock starts
    const {total, times} = await page.evaluate(() => { window.start(); return {total: window.TOTAL, times: window.BEAT_TIMES}; });
    const lead = (Date.now() - opened) / 1000;
    await page.waitForTimeout(Math.ceil(total * 1000) + 700);
    const video = page.video();
    await context.close();
    const webm = path.join(out, 'raw.webm');
    await video.saveAs(webm);
    fs.writeFileSync(path.join(out, 'rec-info.json'), JSON.stringify({lead, total, times}));
    encode(webm, lead, total, times);
    const poster = await open(await browser.newContext({viewport: {width: 1280, height: 720}}));
    for (let k = 10; k >= 0; k--) await poster.evaluate((t) => window.render(t), 2.4 - k * 0.1);
    const png = path.join(out, 'poster.png');
    await poster.screenshot({path: png});
    execFileSync(ffmpeg, ['-y', '-loglevel', 'error', '-i', png, '-q:v', '4', jpg], {stdio: 'inherit'});
    fs.writeFileSync(path.join(out, 'beat-times.json'), JSON.stringify(times, null, 1));
    console.log(`${mp4}\n${jpg}\nwebm ${webm}, lead ${lead.toFixed(2)} s, total ${total.toFixed(2)} s, narration ${Object.keys(narr).length}, music ${music ? 'yes' : 'no'}`);
  }
} finally {
  await browser.close();
  await server.close();
  try { process.kill(pid, 'SIGKILL'); } catch { /* already gone */ }
}
