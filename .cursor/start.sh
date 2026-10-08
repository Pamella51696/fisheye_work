#!/usr/bin/env bash
set -euo pipefail

cd /workspace

if [[ ! -f build/VideoStreamingServer.class ]]; then
  echo "VideoStreamingServer not compiled; run install first." >&2
  exit 1
fi

exec java -Djava.library.path=/usr/lib/jni \
  -cp build:/usr/share/java/opencv.jar \
  VideoStreamingServer 9090 \
  ${UDP_TARGET_HOST:+--udp-target "$UDP_TARGET_HOST"} \
  ${UDP_PORT:+--udp-port "$UDP_PORT"} \
  ${UDP_HZ:+--udp-hz "$UDP_HZ"} \
  ${POSE_MODE:+--pose-mode "$POSE_MODE"}
