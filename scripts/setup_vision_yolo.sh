#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 -m venv --system-site-packages "$workspace_dir/.venv-vision"
vision_python="$workspace_dir/.venv-vision/bin/python"
# Select the official CPU wheels explicitly, before resolving Ultralytics.
"$vision_python" -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu 'torch==2.6.0' 'torchvision==0.21.0'
"$vision_python" - "$workspace_dir/.venv-vision/cpu-constraints.txt" <<'PY'
import sys
from pathlib import Path
from importlib.metadata import version
import torch
if torch.version.cuda is not None:
    raise SystemExit('A CUDA build is installed. CPU setup stopped; no ready marker will be written.')
Path(sys.argv[1]).write_text('\n'.join(f'{name}=={version(name)}' for name in ('torch', 'torchvision')) + '\n')
PY
"$vision_python" -m pip install --no-cache-dir -c "$workspace_dir/.venv-vision/cpu-constraints.txt" 'ultralytics>=8.3,<9' 'numpy<2'
"$vision_python" - <<'PY'
import torch
import torchvision
import ultralytics
assert torch.version.cuda is None, 'CPU-only PyTorch is required'
print('PyTorch:', torch.__version__, '| CUDA:', torch.version.cuda)
print('Ultralytics:', ultralytics.__version__)
PY
touch "$workspace_dir/.venv-vision/.ready"
printf '%s\n' 'CPU YOLO environment installed. Restart with scripts/start_object_feature_detector.sh.'
