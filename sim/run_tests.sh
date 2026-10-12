#!/bin/sh
# Builds the engine for the Mac and runs the tests.
set -e
cd "$(dirname "$0")"
mkdir -p out
# The newest macOS SDK may not match the installed linker; use the 26.x SDK that works
SDK=$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX26*.sdk 2>/dev/null | head -1)
SOURCES="test_engine.cpp"
[ -f ../engine/engine.cpp ] && SOURCES="$SOURCES ../engine/engine.cpp"
c++ ${SDK:+-isysroot "$SDK"} -std=c++17 -O2 -Wall -Wextra -I../engine $SOURCES -o out/test_engine
./out/test_engine
