#!/bin/bash

# Check if Emscripten is loaded
if ! command -v em++ &> /dev/null; then
    echo "Emscripten not found! Did you run 'source ./emsdk_env.sh'?"
    exit 1
fi

echo "Building WebAssembly..."

em++ main.cpp liblouis/liblouis.a \
  -I . \
  --embed-file liblouis/tables@/tables \
  -s ALLOW_MEMORY_GROWTH=1 \
  -o index.js

echo "Run 'python3 -m http.server' to test."