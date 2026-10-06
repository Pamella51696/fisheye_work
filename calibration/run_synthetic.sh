#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 calibration/synthetic_calibrate.py .
python3 calibration/ground_plane_align.py .
echo "Edit config/rig.json for real-camera K,D,R,T when available."
