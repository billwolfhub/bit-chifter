#!/bin/sh
# Builds the render tool and runs it: render.sh in.wav out.wav [tune=Hz] [fb=] [mix=] [ring=] [carrier=Hz] [filter=Hz] [q=] [open=] [stereo=linked|mirror|rotate]
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/out"
SDK=$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk 2>/dev/null | head -1)
c++ ${SDK:+-isysroot "$SDK"} -std=c++17 -O2 -Wall -I"$HERE/../engine" "$HERE/render.cpp" "$HERE/../engine/engine.cpp" -o "$HERE/out/render"
"$HERE/out/render" "$@"
