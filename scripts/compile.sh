#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/out"
rm -rf "$OUT"
mkdir -p "$OUT"
mapfile -t SOURCES < <(find "$ROOT" -name '*.java' ! -path '*/out/*')
CP_ARGS=()
if [[ -n "${OPENCV_JAR:-}" ]]; then
  CP_ARGS=(-cp "$OPENCV_JAR")
fi
javac -encoding UTF-8 "${CP_ARGS[@]}" -d "$OUT" "${SOURCES[@]}"
echo "Compiled to $OUT"
echo "Run: java -cp out:\${OPENCV_JAR} -Djava.library.path=\${JAVA_LIBRARY_PATH} VideoStreamingServer"
echo "(See BUILD.md — compile all sources, not only VideoStreamingServer.java)"
