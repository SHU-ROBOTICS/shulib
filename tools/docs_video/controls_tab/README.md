# The Controls tab video

The video at the start of the Controls tab section of [guide 22](../../../docs/guide/22-making-your-first-route.md):
the real route builder, one idea per beat, one highlight at a time, narrated, with quiet music. `beats.json` is
the script: each beat's spoken sentence (`say`) and its caption. `record.mjs` opens the built route builder in
Chromium, connects an in-memory folder holding the two driver files (so Export shows its **Write into** button;
it is pointed at, never pressed, and nothing is written anywhere), injects an overlay (spotlight, ring, cursor,
click ripple, caption band — never the app's code), plays each beat (its narration's length + 0.7 s, at least
3.5 s), then mixes the narration and the music and writes `docs/assets/video/controls-tab.mp4` and its poster
`controls-tab.jpg` (about 85 s, in real time). `narration/` (Kokoro's WAVs) is not committed.

From the repository root, with the route builder built (`python3 tools/route_maker/build.py`):

```sh
KOKORO_MODELS=<dir with kokoro-v1.0.onnx, voices-v1.0.bin> <python with kokoro-onnx, soundfile> \
    tools/docs_video/setup_first/narrate.py --dir tools/docs_video/controls_tab --voice af_heart
MUSIC=<path to Wallpaper.mp3> PW_NODE_MODULES=<a node_modules with playwright> PLAYWRIGHT_BROWSERS_PATH=<its browsers> \
    node tools/docs_video/controls_tab/record.mjs --site build/route_maker_site
```

The voice model is Kokoro v1.0 from https://github.com/thewh1teagle/kokoro-onnx/releases (model-files-v1.0).
The music is "Wallpaper" by Kevin MacLeod, from https://incompetech.com/music/royalty-free/mp3-royaltyfree/Wallpaper.mp3,
licensed under Creative Commons: By Attribution 4.0 (credit required: it is on the video's last card and under
the video in guide 22). It is not kept in the repository: download it and pass its path as `MUSIC`.

Keep the beats in step with guide 22's Controls tab section and the page: the recorder stops with an error if
the graph's readout does not say what the narration says ("30 % stick → 12 % speed" at a curve of 0.6, and
100 % at a full push).
