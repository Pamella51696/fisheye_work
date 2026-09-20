#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

PORT="${PANORAMA_PORT:-9090}"
LOG="/tmp/panorama-server.log"
PIDFILE="/tmp/panorama-server.pid"
JAVA_CP="$ROOT/build:/usr/share/java/opencv.jar"
JAVA_LIB="/usr/lib/jni"

if curl -sf "http://127.0.0.1:${PORT}/play" >/dev/null 2>&1; then
  exit 0
fi

if [[ -f "$PIDFILE" ]]; then
  old_pid="$(cat "$PIDFILE")"
  if kill -0 "$old_pid" 2>/dev/null; then
    for _ in $(seq 1 30); do
      if curl -sf "http://127.0.0.1:${PORT}/play" >/dev/null 2>&1; then
        exit 0
      fi
      sleep 1
    done
  fi
fi

nohup java -Djava.library.path="$JAVA_LIB" \
  -cp "$JAVA_CP" \
  VideoStreamingServer "$PORT" >>"$LOG" 2>&1 &
echo $! >"$PIDFILE"

for _ in $(seq 1 45); do
  if curl -sf "http://127.0.0.1:${PORT}/play" >/dev/null 2>&1; then
    echo "Panorama server ready on http://localhost:${PORT}/play"
    exit 0
  fi
  sleep 1
done

echo "Panorama server failed to become ready; see $LOG" >&2
exit 1
