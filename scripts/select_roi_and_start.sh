#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: Tidak ada desktop display." >&2
  echo "Jalankan script ini dari Terminal pada desktop Raspberry Pi, bukan SSH." >&2
  exit 1
fi

if pgrep -f 'component_container.*camera' >/dev/null 2>&1; then
  echo "ERROR: node kamera ROS masih berjalan." >&2
  echo "Hentikan terminal kamera lama dengan Ctrl+C, lalu coba lagi." >&2
  exit 1
fi

"${WORKSPACE_DIR}/scripts/configure_object_roi.py"

echo
echo "ROI tersimpan. Menjalankan detector dan Foxglove..."
exec "${WORKSPACE_DIR}/scripts/start_orbbec_foxglove.sh"
