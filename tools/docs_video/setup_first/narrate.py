#!/usr/bin/env python3
"""Speaks beats.json into narration/<id>.wav with Kokoro (offline; Apache-2.0 model), one file per beat.

  KOKORO_MODELS=<dir with kokoro-v1.0.onnx and voices-v1.0.bin> <a python with kokoro-onnx, soundfile> \
      tools/docs_video/setup_first/narrate.py [--voice af_heart] [--speed 1.0]

The models come from https://github.com/thewh1teagle/kokoro-onnx/releases (model-files-v1.0). A beat whose
"say" did not change keeps its file (the text is stored beside it), so re-running after an edit re-speaks only
what changed. record.mjs reads each file's length to time its beat.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--voice", default="af_heart")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--dir", default=HERE, help="the video's folder (holds beats.json; narration/ is written there)")
    args = ap.parse_args()
    models = os.environ.get("KOKORO_MODELS", "")
    if not models:
        sys.exit("set KOKORO_MODELS to the folder holding kokoro-v1.0.onnx and voices-v1.0.bin")
    import soundfile as sf
    from kokoro_onnx import Kokoro

    beats = json.load(open(os.path.join(args.dir, "beats.json"), encoding="utf-8"))
    out = os.path.join(args.dir, "narration")
    os.makedirs(out, exist_ok=True)
    tts = Kokoro(os.path.join(models, "kokoro-v1.0.onnx"), os.path.join(models, "voices-v1.0.bin"))
    spoken = 0
    for b in beats:
        say = b.get("say", "").strip()
        wav, txt = os.path.join(out, b["id"] + ".wav"), os.path.join(out, b["id"] + ".txt")
        key = f"{args.voice}|{args.speed}|{say}"
        if not say:
            continue
        if os.path.exists(wav) and os.path.exists(txt) and open(txt, encoding="utf-8").read() == key:
            continue
        audio, rate = tts.create(say, voice=args.voice, speed=args.speed, lang="en-us")
        sf.write(wav, audio, rate)
        with open(txt, "w", encoding="utf-8") as f:
            f.write(key)
        spoken += 1
        print(f"{b['id']}: {len(audio) / rate:.2f} s  {say}")
    print(f"narrate: {spoken} beat(s) spoken, {len(beats)} in beats.json -> {out}")


if __name__ == "__main__":
    main()
