#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vision_python=/usr/bin/python3
if [[ -f "$script_dir/../.venv-vision/.ready" ]]; then
  vision_python="$script_dir/../.venv-vision/bin/python"
fi
exec "$vision_python" "$script_dir/object_feature_detector.py" "$@"
