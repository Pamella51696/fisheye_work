#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 calibration/synthetic_calibrate.py .
# Ground-plane panel warp is experimental — keep disabled in rig.json (ground_plane_enabled: 0).
echo "Edit config/rig.json for real-camera K,D,R,T when available."
