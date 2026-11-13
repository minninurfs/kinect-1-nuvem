#!/bin/bash
# Compila o visualizador do Kinect (kinect_cloud) com a saida Syphon.
# Precisa: Command Line Tools (xcode-select --install) e libfreenect (brew install libfreenect).
set -e
cd "$(dirname "$0")"
SY=syphon/Syphon
OBJ=build/syphon
mkdir -p "$OBJ" build/inc
ln -sfn "../../$SY" build/inc/Syphon

# biblioteca do Syphon (so a parte OpenGL), compilada uma vez
if [ ! -f build/libsyphon.a ]; then
  echo "Compilando Syphon..."
  for f in "$SY"/*.m "$SY"/*.c; do
    n=$(basename "$f")
    clang -c -O2 -mmacosx-version-min=11.0 -fobjc-arc \
      -Wno-deprecated-declarations -include "$SY/Syphon_Prefix.pch" -I"$SY" -Ibuild/inc \
      "$f" -o "$OBJ/${n%.*}.o"
  done
  ar rcs build/libsyphon.a "$OBJ"/*.o
fi

echo "Compilando kinect_cloud..."
clang -O2 -Wall -Wno-deprecated-declarations -mmacosx-version-min=11.0 \
  -I/opt/homebrew/include/libfreenect -Ibuild/inc \
  kinect_cloud.c syphon_out.m -fobjc-arc build/libsyphon.a \
  -L/opt/homebrew/lib -lfreenect \
  -framework OpenGL -framework GLUT -framework Cocoa -framework IOSurface -framework CoreVideo \
  -o kinect_cloud
echo "Pronto: ./kinect_cloud"
