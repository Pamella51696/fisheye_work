#!/usr/bin/env bash
set -euo pipefail

cd /workspace

if [[ ! -f build/VideoStreamingServer.class ]]; then
  echo "VideoStreamingServer not compiled; run install first." >&2
  exit 1
fi

exec java -Djava.library.path=/usr/lib/jni \
  -cp build:/usr/share/java/opencv.jar \
  VideoStreamingServer 9090
