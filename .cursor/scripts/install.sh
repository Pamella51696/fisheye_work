#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

OPENCV_JAR="/usr/share/java/opencv.jar"
OPENCV_NATIVE_DIR="/usr/lib/jni"

if ! command -v ffmpeg >/dev/null 2>&1 || ! command -v javac >/dev/null 2>&1; then
  sudo apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ffmpeg openjdk-21-jdk-headless
fi

if [[ ! -f "$OPENCV_JAR" ]]; then
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libopencv-java
fi

mkdir -p "$ROOT/build"

javac -d "$ROOT/build" -cp "$OPENCV_JAR" "$ROOT/VideoStreamingServer.java"

for role in left front right rear; do
  clip="$ROOT/${role}.mp4"
  if [[ -f "$clip" ]]; then
    continue
  fi
  ffmpeg -hide_banner -loglevel error -y \
    -f lavfi -i "testsrc=size=640x480:rate=15,drawtext=text=${role}:fontsize=48:fontcolor=white:x=(w-tw)/2:y=(h-th)/2" \
    -t 4 -c:v libx264 -pix_fmt yuv420p -an "$clip"
done

echo "Install complete: OpenCV Java (system), compiled server, fixture clips ready."
