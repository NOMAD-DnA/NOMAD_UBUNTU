#!/usr/bin/env python3
"""Shared folder = development/distribution source; each laptop runs a local copy.

    bash <공유폴더>/NOMAD_실물시험앱_20261106/GUI_설치_업데이트.sh

copies laptop_monitor/ and config/ into ~/nomad_gui/versions/<version>/,
points ~/nomad_gui/current at it and registers the app "NOMAD GUI (로컬)".
A running GUI is never changed. Recordings of a local copy go to
~/nomad_gui/주행기록 and are offered for upload to the shared 주행기록 when the
shared folder is reachable again. Every access to the shared folder from a
running GUI goes through a short subprocess timeout, because a dropped network
mount (laptop switched to its hotspot) can block file access for a long time.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

MARKER = '.nomad_local_install.json'
FEATURE = 'nomad-gui-local-install-20261111-1'
PARTS = ('laptop_monitor', 'config')
DOCS = ('사용방법.md', '핫스팟_연결_안내.md', 'OAK_점군_선택_안내.md', '경사_계산_선택_CPU_안내.md')
KEEP_VERSIONS = 3
APP_NAME = 'NOMAD GUI (로컬)'
DESKTOP_ID = 'nomad-gui-local.desktop'


def home_root():
    return Path(os.environ.get('NOMAD_GUI_HOME', str(Path.home() / 'nomad_gui'))).expanduser()


def _skip(path):
    return '__pycache__' in path.parts or path.suffix in ('.pyc', '.pyo') or path.name == MARKER


def tree_files(source):
    source = Path(source)
    files = []
    for part in PARTS:
        base = source / part
        if not base.is_dir():
            raise FileNotFoundError(str(base))
        for path in sorted(base.rglob('*')):
            if path.is_file() and not path.is_symlink() and not _skip(path.relative_to(source)):
                files.append(path.relative_to(source))
    for doc in DOCS:
        if (source / doc).is_file():
            files.append(Path(doc))
    return files


def tree_version(source):
    """Content hash of what a local copy contains (paths + bytes)."""
    digest = hashlib.sha256()
    for relative in tree_files(source):
        digest.update(str(relative).encode('utf-8') + b'\0')
        digest.update(hashlib.sha256((Path(source) / relative).read_bytes()).digest())
    return digest.hexdigest()[:12]


def installed_info(app_dir=None):
    """Marker of the local copy this code runs from, or None (e.g. shared folder)."""
    app_dir = Path(app_dir or Path(__file__).resolve().parent)
    try:
        value = json.loads((app_dir.parent / MARKER).read_text(encoding='utf-8'))
        return value if value.get('feature') == FEATURE else None
    except (OSError, ValueError):
        return None


def _desktop_entry(root):
    quoted = '"' + str(root / 'current' / 'laptop_monitor' / 'run_app.sh').replace('"', '\\"') + '"'
    return ('[Desktop Entry]\nType=Application\nName=' + APP_NAME + '\n'
            'Comment=로컬 사본 실행 · 핫스팟 중에도 동작 · 공유폴더 연결 시 업데이트 확인\n'
            'Exec=/bin/bash ' + quoted + '\n'
            'Icon=' + str(root / 'current' / 'laptop_monitor' / 'nomad-trial.svg') + '\n'
            'Terminal=false\nCategories=Science;Education;\nStartupNotify=true\n')


def install(source, root=None, register=True):
    source = Path(source).resolve()
    root = Path(root or home_root())
    version = tree_version(source)
    versions = root / 'versions'
    target = versions / version
    if not target.exists():
        staging = versions / ('.staging-' + version + '-' + str(os.getpid()))
        if staging.exists():
            shutil.rmtree(str(staging))
        for relative in tree_files(source):
            destination = staging / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(source / relative), str(destination))
        marker = {'feature': FEATURE, 'version': version, 'source': str(source), 'root': str(root),
                  'installed_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                  'note': '배포 사본입니다. 여기서 고치지 말고 source의 공유폴더 원본을 고친 뒤 다시 설치하세요.'}
        (staging / MARKER).write_text(json.dumps(marker, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
        os.replace(str(staging), str(target))
    link = root / 'current'
    temporary = root / ('.current-' + str(os.getpid()))
    if temporary.is_symlink() or temporary.exists():
        temporary.unlink()
    temporary.symlink_to(Path('versions') / version)
    os.replace(str(temporary), str(link))
    (root / '주행기록').mkdir(parents=True, exist_ok=True)
    (root / 'AGENTS.md').write_text(
        '# 로컬 배포 사본 — 수정 금지\n\n'
        '이 폴더(~/nomad_gui)는 NOMAD GUI의 **배포 사본**이다. 코드 수정은 여기서 하지 않는다.\n\n'
        '- 원본(개발·배포처): `' + str(source) + '` (공유폴더). 원본을 고치고 확인한 뒤 '
        '`bash <원본>/GUI_설치_업데이트.sh`로 다시 설치한다.\n'
        '- `current` → 지금 실행하는 버전. `versions/<해시>`는 내용 해시로 이름 붙인 사본이며 최근 3개를 남긴다.\n'
        '- `주행기록/`: 로컬 사본이 저장하는 .nomad. 공유폴더에 연결되면 GUI가 올리기를 묻는다. 로컬 원본은 지우지 않는다.\n'
        '- 상세 규칙: 원본의 `laptop_monitor/AGENTS.md` "배포 구조" 절.\n', encoding='utf-8')
    pruned = []
    for old in sorted((p for p in versions.iterdir() if p.is_dir() and not p.name.startswith('.')),
                      key=lambda p: p.stat().st_mtime, reverse=True)[KEEP_VERSIONS:]:
        if old.name != version:
            shutil.rmtree(str(old), ignore_errors=True)
            pruned.append(old.name)
    entry = None
    if register:
        apps = Path(os.environ.get('XDG_DATA_HOME', str(Path.home() / '.local/share'))) / 'applications'
        apps.mkdir(parents=True, exist_ok=True)
        entry = apps / DESKTOP_ID
        entry.write_text(_desktop_entry(root), encoding='utf-8')
        entry.chmod(0o755)
        desktop = Path.home() / 'Desktop'
        if desktop.is_dir():
            shortcut = desktop / DESKTOP_ID
            shortcut.write_text(_desktop_entry(root), encoding='utf-8')
            shortcut.chmod(0o755)
            subprocess.run(['gio', 'set', str(shortcut), 'metadata::trusted', 'true'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    return {'version': version, 'path': str(target), 'current': str(link), 'pruned': pruned,
            'entry': str(entry) if entry else ''}


# ---------- used by a running local GUI; shared-folder access is time-bounded ----------

def _bounded(args, timeout):
    try:
        result = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout
    except (OSError, subprocess.TimeoutExpired):
        return None, ''


def source_version(source, timeout=15):
    """Version of the shared source, or None when the folder is unreachable/slow."""
    code, _ = _bounded(['stat', '-t', str(Path(source) / 'laptop_monitor' / 'launcher.py')], 3)
    if code != 0:
        return None
    code, out = _bounded([sys.executable, '-I', str(Path(__file__).resolve()), 'version', '--source', str(source)], timeout)
    out = out.strip()
    return out if code == 0 and len(out) == 12 else None


def update_available(info=None):
    info = info or installed_info()
    if not info:
        return None
    newer = source_version(info['source'])
    return newer if newer and newer != info['version'] else None


def run_update(info, timeout=120):
    code, out = _bounded([sys.executable, '-I', str(Path(info['source']) / 'laptop_monitor' / 'local_install.py'),
                          'install', '--source', info['source']], timeout)
    return code == 0, out


def local_recordings(info):
    return Path(info.get('root', str(home_root()))) / '주행기록'


def shared_recordings(info):
    return Path(info['source']).parent / '주행기록'


def pending_uploads(info, settle_s=60):
    """Finished local .nomad files not yet in the shared 주행기록 (by name)."""
    local = local_recordings(info)
    if not local.is_dir():
        return []
    code, out = _bounded(['ls', '-1', str(shared_recordings(info))], 5)
    if code is None or code != 0:
        return []
    remote = set(out.split('\n'))
    now = time.time()
    found = []
    for path in sorted(local.glob('*.nomad')):
        sidecars = [path.with_name(path.name + suffix) for suffix in ('-wal', '-journal', '-shm')]
        if path.name in remote or any(s.exists() for s in sidecars) or now - path.stat().st_mtime < settle_s:
            continue
        found.append(path)
    return found


def upload(info, paths, timeout=120):
    """Copy (never move) to the shared folder via a temporary name, then rename."""
    done = []
    for path in paths:
        destination = shared_recordings(info) / path.name
        code, _ = _bounded(['bash', '-c', 'cp -p -- "$1" "$2.part" && mv -n -- "$2.part" "$2"', '_', str(path), str(destination)],
                           timeout)
        if code == 0:
            done.append(path.name)
    return done


def offer(window):
    """Launcher hook: ask about an update, then about uploading recordings."""
    from PyQt5 import QtWidgets
    info = installed_info()
    if not info:
        return
    newer = update_available(info)
    if newer:
        answer = QtWidgets.QMessageBox.question(
            window, 'GUI 업데이트',
            '공유폴더에 새 버전({})이 있습니다. 지금 버전은 {}입니다.\n업데이트하고 GUI를 다시 시작할까요?\n'
            '(시험 중이 아닐 때만 업데이트하세요.)'.format(newer, info['version']))
        if answer == QtWidgets.QMessageBox.Yes:
            ok, _ = run_update(info)
            if ok:
                runner = Path(info['root']) / 'current' / 'laptop_monitor' / 'run_app.sh'
                os.execv('/bin/bash', ['/bin/bash', str(runner)])
            QtWidgets.QMessageBox.warning(window, 'GUI 업데이트', '업데이트하지 못했습니다. 지금 버전으로 계속합니다.')
    files = pending_uploads(info)
    if files:
        answer = QtWidgets.QMessageBox.question(
            window, '주행 기록 올리기',
            '공유폴더에 아직 없는 로컬 주행 기록 {}개가 있습니다.\n공유폴더 주행기록으로 복사할까요? (로컬 원본은 남습니다)'.format(len(files)))
        if answer == QtWidgets.QMessageBox.Yes:
            done = upload(info, files)
            QtWidgets.QMessageBox.information(window, '주행 기록 올리기', '{}개 복사했습니다.'.format(len(done)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('install', 'version'))
    parser.add_argument('--source', default=str(Path(__file__).resolve().parent.parent))
    parser.add_argument('--root', default=None)
    parser.add_argument('--no-register', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'version':
        print(tree_version(args.source))
        return 0
    if installed_info(Path(args.source) / 'laptop_monitor'):
        print('이미 로컬 사본입니다. 공유폴더 원본의 GUI_설치_업데이트.sh를 실행하세요.', file=sys.stderr)
        return 2
    result = install(args.source, args.root, register=not args.no_register)
    print('설치 완료: 버전 {version}\n실행: 앱 목록 또는 바탕화면의 "{name}"\n위치: {path}\n현재: {current}'.format(
        name=APP_NAME, **result))
    if result['pruned']:
        print('정리한 이전 버전: ' + ', '.join(result['pruned']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
