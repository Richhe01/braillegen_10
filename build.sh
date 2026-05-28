#!/bin/bash

echo "Building BrailleGen 10 WebAssembly..."

em++ main.cpp \
  liblouis/liblouis.a \
  occt/lib/libTKMath.a \
  occt/lib/libTKernel.a \
  occt/lib/libTKG2d.a \
  occt/lib/libTKG3d.a \
  occt/lib/libTKBRep.a \
  occt/lib/libTKGeomBase.a \
  occt/lib/libTKGeomAlgo.a \
  occt/lib/libTKTopAlgo.a \
  occt/lib/libTKPrim.a \
  occt/lib/libTKMesh.a \
  occt/lib/libTKShHealing.a \
  -I . \
  -I liblouis \
  -I occt/include \
  --embed-file liblouis/tables@/tables \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s EXPORTED_RUNTIME_METHODS="['ccall','FS']" \
  -o index.js

echo "Run 'python3 -m http.server' to test."