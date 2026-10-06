#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 calibration/synthetic_calibrate.py .
# Optional: python3 calibration/optimize_extrinsics.py .
echo "Edit config/rig.json for real-camera K,D,R,T when available."
