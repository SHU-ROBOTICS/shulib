# The "Make a route and run it on the robot" video

The video at the top of [guide 22](../../../docs/guide/22-making-your-first-route.md): the real route builder's
Route tab, one idea per beat, one highlight at a time, narrated, with quiet music. `beats.json` is the script: each
beat's spoken sentence (`say`) and its caption. `record.mjs` opens the built route builder (its PREVIEW engine
included) in Chromium, fills an in-memory "shulib folder" with the team files (`src/team/skills_route.cpp` and the
two autons files, as they are) through the page tests' hook, so Export's **Connect your shulib folder** and **Write
into src/team/skills_route.cpp** run for real — into memory, never onto the disk — and injects an overlay (spotlight,
ring, cursor, click ripple, caption band — never the app's code; the app's toasts are hidden while it records). It
builds a route as a student does (start, home, drive, nudge, straight, turn, the cascade up and back, the claw, a
time limit and "carry on"), plays PREVIEW, saves, exports, then shows three DRAWN cards, each labelled as drawn:
VS Code's **Terminal → Run Task → Upload routes to robot one**, its terminal (the words are `tools/upload_auton.py`'s
own), and the brain. Each beat lasts its narration's length + 0.7 s (at least 3.5 s); the narration and the music
are mixed in, and `docs/assets/video/make-a-route.mp4` and its poster `make-a-route.jpg` are written (about 2 min
40 s, in real time). `narration/` (Kokoro's WAVs) is not committed.

From the repository root, with the route builder built with its engine (see `tools/route_maker/README.md`:
`build/wasm`, then `python3 tools/route_maker/build.py --out build/route_maker_site --describe
build/test/shulib_route_maker --engine build/wasm`):

```sh
KOKORO_MODELS=<dir with kokoro-v1.0.onnx, voices-v1.0.bin> <python with kokoro-onnx, soundfile> \
    tools/docs_video/setup_first/narrate.py --dir tools/docs_video/make_a_route --voice af_heart
MUSIC=<path to Wallpaper.mp3> PW_NODE_MODULES=<a node_modules with playwright> PLAYWRIGHT_BROWSERS_PATH=<its browsers> \
    node tools/docs_video/make_a_route/record.mjs --site build/route_maker_site
```

The voice model is Kokoro v1.0 from https://github.com/thewh1teagle/kokoro-onnx/releases (model-files-v1.0).
The music is "Wallpaper" by Kevin MacLeod, from https://incompetech.com/music/royalty-free/mp3-royaltyfree/Wallpaper.mp3,
licensed under Creative Commons: By Attribution 4.0 (credit required: it is on the video's last card and under
the video in guide 22). It is not kept in the repository: download it and pass its path as `MUSIC`.

Keep the beats in step with guide 22 and the page. The recorder stops with an error when the page does not do what
the narration says: the arrow-key nudge must land back on the drive's point, the steps must be in the order spoken,
the drive must keep its 1.5 s time limit, and PREVIEW must finish. The drive is 12 in long on purpose: on the host
test plant a 24 in drive needs about 1.5 s and would fail its own time limit.
