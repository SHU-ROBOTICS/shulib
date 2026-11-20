# The "Set up your robot first" video

The video in [guide 23](../../../docs/guide/23-set-up-your-robot-first.md): one idea per beat, narrated, with
quiet music. `beats.json` is the script: each beat's spoken sentence (`say`) and its caption. `index.html`
draws each beat (one element, large, with one highlight); `narrate.py` speaks the beats into `narration/`
(Kokoro, offline; not committed); `record.mjs` records the page in Chromium, times each beat to its narration
(its length + 0.7 s), mixes the narration and the music, and writes `docs/assets/video/setup-first.mp4` and its
poster `setup-first.jpg` (about 4 minutes, in real time).

From the repository root:

```sh
KOKORO_MODELS=<dir with kokoro-v1.0.onnx, voices-v1.0.bin> <python with kokoro-onnx, soundfile> \
    tools/docs_video/setup_first/narrate.py --voice af_heart
PW_NODE_MODULES=<a node_modules with playwright> PLAYWRIGHT_BROWSERS_PATH=<its browsers> \
    node tools/docs_video/setup_first/record.mjs --music <path to Wallpaper.mp3>
```

The music is "Wallpaper" by Kevin MacLeod, from
https://incompetech.com/music/royalty-free/mp3-royaltyfree/Wallpaper.mp3, licensed under Creative Commons:
By Attribution 4.0 (credit required: it is on the video's last card and under the video in guide 23). It is
not kept in the repository: download it and pass its path.

`--stills 3,60` writes PNGs of those moments instead. Change a beat in `beats.json` (then narrate again), and a
screen word in `index.html`, in step with guide 23 and the programs it quotes: Bench Tests' menu and station 13
(`src/programs/bench_r3a.{hpp,cpp}`, station 13's plain words from S13UX) and Calibrate's screens
(`include/shulib/calibration/calibrate_panel.hpp`, FINDING LEFT and the A / B groups from CAL2).
