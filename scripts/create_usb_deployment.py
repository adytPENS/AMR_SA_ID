#!/usr/bin/env python3
"""Create a USB-safe source deployment kit, including local uncommitted changes."""
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'deployment' / 'STUDICA_VMX_USB'
TEMPLATES = ROOT / 'deployment' / 'templates'
EXCLUDE = {'.git', '.agents', '.codex', '__pycache__', '.pytest_cache',
           'build', 'install', 'log', 'logs', 'Log', '.vscode'}

def include(info):
    p = Path(info.name)
    if any(part in EXCLUDE for part in p.parts) or p.suffix in {'.o', '.d', '.pyc'}:
        return None
    if p.name == 'libstudica_drivers.so':
        return None
    # Driver examples may contain old ARM executables; source is sufficient.
    if info.isfile() and 'drivers/examples/' in info.name:
        if not p.suffix and p.name != 'Makefile':
            return None
    info.uid = info.gid = 0
    info.uname = info.gname = ''
    if info.isdir():
        info.mode = 0o755
    elif info.isfile():
        info.mode = 0o755 if info.mode & 0o111 else 0o644
    return info

def revision(path):
    result = subprocess.run(['git', '-C', str(path), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None

def main():
    sdk = Path('/home/vmx/YDLidar-SDK')
    lidar = Path('/home/vmx/ydlidar_ros2_ws/src')
    if not (sdk / 'CMakeLists.txt').is_file() or not lidar.is_dir():
        raise SystemExit('YDLidar source missing: cannot create a complete kit.')
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ('INSTALL.sh', 'README.md'):
        shutil.copyfile(TEMPLATES / name, OUT / name)
    payload = OUT / 'payload.tar.gz'
    # PAX supports long names; gzip file is portable to FAT32 and preserves symlinks.
    with tarfile.open(payload, 'w:gz', dereference=False) as archive:
        for name in ('src', 'drivers', 'scripts', 'config', 'maps', 'docs', 'README.md', 'LICENSE'):
            if (ROOT / name).exists():
                archive.add(ROOT / name, arcname='studica_ws/' + name, filter=include)
        archive.add(TEMPLATES, arcname='studica_ws/deployment/templates', filter=include)
        archive.add(ROOT / 'deployment/README.md', arcname='studica_ws/deployment/README.md', filter=include)
        archive.add(sdk, arcname='YDLidar-SDK', filter=include)
        archive.add(lidar, arcname='ydlidar_ros2_ws/src', filter=include)
    with tarfile.open(payload) as archive:
        members = archive.getmembers()
    manifest = {
        'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'target': 'Studica Ubuntu 22.04 ARM64, user vmx, ROS Humble',
        'snapshot': 'working tree including local modifications; excludes build/install/cache',
        'project_revision': revision(ROOT),
        'ydlidar_sdk_revision': revision(sdk),
        'ydlidar_driver_revision': revision(lidar / 'ydlidar_ros2_driver'),
        'archive_members': len(members),
        'uncompressed_file_bytes': sum(m.size for m in members if m.isfile()),
        'requires_internet': True,
        'vmx_hal_included': False,
    }
    (OUT / 'MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
    names = ('INSTALL.sh', 'README.md', 'MANIFEST.json', 'payload.tar.gz')
    sums = []
    for name in names:
        digest = hashlib.sha256()
        with (OUT / name).open('rb') as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b''):
                digest.update(chunk)
        sums.append(f'{digest.hexdigest()}  {name}\n')
    (OUT / 'SHA256SUMS').write_text(''.join(sums))
    with zipfile.ZipFile(OUT.with_suffix('.zip'), 'w', compression=zipfile.ZIP_STORED) as bundle_zip:
        for name in (*names, 'SHA256SUMS'):
            bundle_zip.write(OUT / name, arcname=f'STUDICA_VMX_USB/{name}')
    print(f'Created {OUT} ({payload.stat().st_size / 1024**2:.1f} MiB payload, {len(members)} entries)')

if __name__ == '__main__':
    main()
