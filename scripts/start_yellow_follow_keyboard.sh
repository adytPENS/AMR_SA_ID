#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Jalankan start_color_roi_tracker.sh di terminal lain; centang profil yellow/kuning."
echo "Z = toggle follow kuning | W/S/A/D = manual | E/X = stop | Q = keluar"
exec bash "${WORKSPACE_DIR}/scripts/start_full_keyboard.sh" "$@"
