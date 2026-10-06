#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/out"
rm -rf "$OUT"
mkdir -p "$OUT"
mapfile -t SOURCES < <(find "$ROOT" -name '*.java' ! -path '*/out/*')
javac -encoding UTF-8 -d "$OUT" "${SOURCES[@]}"
echo "Compiled to $OUT"
