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
import signal
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
# 2026-11-17: 같은 사본에 들어 있는 VIO 시험기(서브 GUI)도 함께 등록한다.
TESTER_NAME = 'NOMAD VIO 시험기 (로컬)'
TESTER_ID = 'nomad-vio-tester-local.desktop'


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


def tree_fingerprint(source):
    """Quick change check (2026-11-12): paths + sizes + whole-second mtimes, no file
    contents. Over a hotspot GVFS mount reading every byte took longer than the old
    15 s limit, so the update question never appeared."""
    digest = hashlib.sha256()
    for relative in tree_files(source):
        stat = (Path(source) / relative).stat()
        digest.update('{}\0{}\0{}\0'.format(relative, stat.st_size, int(stat.st_mtime)).encode('utf-8'))
    return digest.hexdigest()[:12]


def installed_info(app_dir=None):
    """Marker of the local copy this code runs from, or None (e.g. shared folder)."""
    app_dir = Path(app_dir or Path(__file__).resolve().parent)
    try:
        value = json.loads((app_dir.parent / MARKER).read_text(encoding='utf-8'))
        return value if value.get('feature') == FEATURE else None
    except (OSError, ValueError):
        return None


def _desktop_entry(root, runner='run_app.sh', name=APP_NAME,
                   comment='로컬 사본 실행 · 핫스팟 중에도 동작 · 공유폴더 연결 시 업데이트 확인'):
    quoted = '"' + str(root / 'current' / 'laptop_monitor' / runner).replace('"', '\\"') + '"'
    return ('[Desktop Entry]\nType=Application\nName=' + name + '\n'
            'Comment=' + comment + '\n'
            'Exec=/bin/bash ' + quoted + '\n'
            'Icon=' + str(root / 'current' / 'laptop_monitor' / 'nomad-trial.svg') + '\n'
            'Terminal=false\nCategories=Science;Education;\nStartupNotify=true\n')


def _share_relative():
    """Where the GUI source lives inside the team share (share_servers.json), so a
    local copy finds it again through any address (classroom name or hotspot)."""
    try:
        return json.loads((Path(__file__).resolve().parent / 'share_servers.json').read_text(encoding='utf-8'))['relative']
    except (OSError, ValueError, KeyError, TypeError):
        return ''


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
                  'share_relative': _share_relative(), 'fingerprint': tree_fingerprint(source),
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
        entries = [(DESKTOP_ID, _desktop_entry(root))]
        if (target / 'laptop_monitor' / 'run_vio_tester.sh').is_file():
            entries.append((TESTER_ID, _desktop_entry(root, 'run_vio_tester.sh', TESTER_NAME,
                                                      'VIO·융합 위치추정 보기 + 키보드 주행(조종·녹화 창) · 운영 GUI 옆에서 사용')))
        desktop = Path.home() / 'Desktop'
        for desktop_id, text in entries:
            path = apps / desktop_id
            path.write_text(text, encoding='utf-8')
            path.chmod(0o755)
            if desktop_id == DESKTOP_ID:
                entry = path
            if desktop.is_dir():
                shortcut = desktop / desktop_id
                shortcut.write_text(text, encoding='utf-8')
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


def update_check(info, quick_timeout=30, full_timeout=120):
    """(newer version or None, reason): reason is unreachable / same / newer / timeout / error.
    Compares the quick fingerprint first; reads file contents only if it changed."""
    source = str(info['source'])
    code, _ = _bounded(['stat', '-t', str(Path(source) / 'laptop_monitor' / 'launcher.py')], 5)
    if code != 0:
        return None, 'unreachable'
    me = str(Path(__file__).resolve())
    if info.get('fingerprint'):
        code, out = _bounded([sys.executable, '-I', me, 'version', '--quick', '--source', source], quick_timeout)
        if code is None:
            return None, 'timeout'
        if code == 0 and out.strip() == info['fingerprint']:
            return None, 'same'
    code, out = _bounded([sys.executable, '-I', me, 'version', '--source', source], full_timeout)
    out = out.strip()
    if code is None:
        return None, 'timeout'
    if code != 0 or len(out) != 12:
        return None, 'error'
    return (out, 'newer') if out != info['version'] else (None, 'same')


def update_available(info=None):
    info = info or installed_info()
    if not info:
        return None
    return update_check(info)[0]


def run_update(info, timeout=300):
    code, out = _bounded([sys.executable, '-I', str(Path(info['source']) / 'laptop_monitor' / 'local_install.py'),
                          'install', '--source', info['source']], timeout)
    return code == 0, out


def local_recordings(info):
    return Path(info.get('root', str(home_root()))) / '주행기록'


def shared_recordings(info):
    return Path(info['source']).parent / '주행기록'


def reachable_shared_recordings(info=None):
    """2026-11-12: shared 주행기록 of a local copy if the share is already reachable
    (recorded path or an existing GVFS mount). Never mounts or asks; quick, bounded."""
    info = info or installed_info()
    if not info:
        return None
    import share_mount
    source = Path(info['source']) if share_mount.path_ready(info['source']) else None
    if source is None:
        config = share_mount.load_config()
        found, _ = share_mount.find_mounted(config) if config else (None, None)
        source = Path(found) if found else None
    if source is None:
        return None
    folder = source.parent / '주행기록'
    code, _ = _bounded(['test', '-d', str(folder)], 3)
    return folder if code == 0 else None


def pending_uploads(info, settle_s=60):
    """Finished local .nomad files not yet in the shared 주행기록 (by name).
    A remote "<name>.part" without the final name still counts as pending (2026-11-16).
    Symlinks are skipped (2026-11-22): they point at recordings that already live on the
    share (e.g. NOMAD_센서녹화), and stat of the link gave its own size -> "크기 다름"."""
    local = local_recordings(info)
    if not local.is_dir():
        return []
    code, out = _bounded(['ls', '-1', str(shared_recordings(info))], 5)
    if code is None or code != 0:
        return []
    remote = set(line for line in out.split('\n') if line)
    now = time.time()
    found = []
    for path in sorted(local.glob('*.nomad')):
        sidecars = [path.with_name(path.name + suffix) for suffix in ('-wal', '-journal', '-shm')]
        if path.is_symlink() or path.name in remote or any(s.exists() for s in sidecars) or now - path.stat().st_mtime < settle_s:
            continue
        found.append(path)
    return found


# 2026-11-16: no `cp -p` (GVFS/SMB cannot keep mode/times and cp then failed before
# the rename, leaving only *.part). Size check, optional times, rename with mv -n
# then `gio move`; success only when the final name exists and the .part is gone.
# A remote .part of the same size is only renamed, not copied again.
UPLOAD_SCRIPT = r'''
src=$1; dst=$2; part=$2.part
exec 2>&1
fail() { printf '%s\n' "$1" | tr '\n' ' '; echo; exit 1; }
[ -e "$dst" ] && fail "이미 있음"
size=$(stat -c %s -- "$src") || fail "원본 읽기 실패"
if [ -f "$part" ] && [ "$(stat -c %s -- "$part" 2>/dev/null)" = "$size" ]; then
  :
else
  msg=$(cp -- "$src" "$part" 2>&1) || fail "복사 실패: $msg"
fi
got=$(stat -c %s -- "$part" 2>/dev/null)
[ "$got" = "$size" ] || fail "크기 다름 ($got/$size)"
touch -r "$src" "$part" 2>/dev/null || true
if ! mv -n -- "$part" "$dst" 2>/dev/null || [ -e "$part" ]; then
  [ -e "$dst" ] || msg=$(gio move -T -- "$part" "$dst" 2>&1)
fi
[ -e "$dst" ] && [ ! -e "$part" ] || fail "이름 바꾸기 실패${msg:+: $msg}"
echo ok
'''


def _short_reason(code, out):
    if code is None:
        return '시간 초과'
    lines = [line.strip() for line in out.split('\n') if line.strip()]
    return (lines[-1] if lines else '오류 {}'.format(code))[:80]


def _bounded_group(args, timeout):
    """Like _bounded, but on timeout kill the whole process group (cp/mv/gio too),
    so a stalled copy cannot keep writing the .part after the GUI gave up."""
    try:
        proc = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, start_new_session=True)
    except OSError:
        return None, ''
    try:
        out, _ = proc.communicate(timeout=timeout)
        return proc.returncode, out or ''
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except OSError:
            pass
        try:
            proc.communicate(timeout=2)   # a cp stuck in D state may not die; do not wait on it
        except subprocess.TimeoutExpired:
            pass
        return None, ''


def upload(info, paths, timeout=120, total_timeout=300):
    """Copy (never move) to the shared folder via a temporary name, then rename.
    Returns (copied names, [(name, short reason)]). The whole run is bounded by
    total_timeout and stops at the first timeout (likely a dead mount)."""
    done, failed = [], []
    deadline = time.monotonic() + total_timeout
    stop = ''
    for path in paths:
        left = deadline - time.monotonic()
        if not stop and left <= 0:
            stop = '중단: 전체 시간 초과'
        if stop:
            failed.append((path.name, stop))
            continue
        destination = shared_recordings(info) / path.name
        code, out = _bounded_group(['bash', '-c', UPLOAD_SCRIPT, '_', str(path), str(destination)],
                                   min(timeout, left))
        if code == 0:
            done.append(path.name)
        else:
            failed.append((path.name, _short_reason(code, out)))
            if code is None:
                stop = '중단: 앞 파일 시간 초과'
    return done, failed


def upload_message(done, failed):
    if not failed:
        return '{}개 복사했습니다.'.format(len(done))
    return '{}개 복사, {}개 실패({}: {})'.format(len(done), len(failed), failed[0][0], failed[0][1])


def show_share_state(window, text):
    """Launcher status line for the team shared folder (no popup)."""
    label = getattr(window, 'share_label', None)
    if label is not None:
        label.setText('공유폴더: ' + text)


def share_state_text(found, newer, why, version):
    if not found.get('ok'):
        return '연결 안 됨 ({}) · 연결 창 → 이 노트북 핫스팟 → 공유폴더 연결'.format(found.get('message', ''))
    where = 'smb://{}/'.format(found['server']) if found.get('server') else found.get('source', '')
    update = {'same': 'GUI 최신 버전({})'.format(version), 'newer': '새 버전 있음({})'.format(newer),
              'timeout': '업데이트 확인 시간 초과', 'error': '업데이트 확인 실패',
              'unreachable': '연결이 끊겨 업데이트 확인 못 함'}.get(why, why)
    return '연결됨 · {} · {}'.format(where, update)


def offer(window, runner='run_app.sh'):
    """Launcher hook: ask about an update, then about uploading recordings.
    runner: script restarted after an update (run_vio_tester.sh for the VIO tester)."""
    from PyQt5 import QtWidgets
    info = installed_info()
    if not info:
        show_share_state(window, '공유폴더 원본에서 직접 실행 중')
        return
    show_share_state(window, '확인 중…')
    QtWidgets.QApplication.processEvents()
    # 2026-11-12: the recorded path may be a classroom mount that is empty in the
    # field; find the share by name or hotspot address (asks for the password once).
    import share_mount
    asked = []

    def ask(result):
        asked.append(True)
        return share_mount.ask_password_dialog(window, result)
    found = share_mount.resolve_source(info, ask)
    if not found['ok']:
        show_share_state(window, share_state_text(found, None, None, info['version']))
        if asked:
            QtWidgets.QMessageBox.warning(window, '공유폴더 연결', '공유폴더에 연결하지 못했습니다: ' + found['message'])
        return
    info = dict(info, source=found['source'])
    newer, why = update_check(info)
    show_share_state(window, share_state_text(found, newer, why, info['version']))
    if not newer and (asked or why in ('timeout', 'error')):
        # Never silent after the user typed a password or when the check failed.
        where = ' (smb://{}/)'.format(found['server']) if found.get('server') else ''
        text = {'same': '공유폴더에 연결했습니다{}.\nGUI는 이미 최신 버전({})입니다.'.format(where, info['version']),
                'timeout': '공유폴더에 연결했지만{} 업데이트 확인 시간이 초과됐습니다.\n'
                           '연결 창의 공유폴더 연결을 다시 누르거나 GUI를 다시 켜 보세요.'.format(where),
                'error': '공유폴더에 연결했지만{} 새 버전을 확인하지 못했습니다.'.format(where),
                'unreachable': '공유폴더 연결이 끊겨 업데이트를 확인하지 못했습니다.'}.get(why, why)
        box = QtWidgets.QMessageBox.information if why == 'same' else QtWidgets.QMessageBox.warning
        box(window, 'GUI 업데이트', text)
    if newer:
        answer = QtWidgets.QMessageBox.question(
            window, 'GUI 업데이트',
            '공유폴더에 새 버전({})이 있습니다. 지금 버전은 {}입니다.\n업데이트하고 GUI를 다시 시작할까요?\n'
            '(시험 중이 아닐 때만 업데이트하세요.)'.format(newer, info['version']))
        if answer == QtWidgets.QMessageBox.Yes:
            ok, _ = run_update(info)
            if ok:
                restart = Path(info['root']) / 'current' / 'laptop_monitor' / runner
                if not restart.is_file():
                    restart = Path(info['root']) / 'current' / 'laptop_monitor' / 'run_app.sh'
                os.execv('/bin/bash', ['/bin/bash', str(restart)])
            QtWidgets.QMessageBox.warning(window, 'GUI 업데이트', '업데이트하지 못했습니다. 지금 버전으로 계속합니다.')
    files = pending_uploads(info)
    if files:
        answer = QtWidgets.QMessageBox.question(
            window, '주행 기록 올리기',
            '공유폴더에 아직 없는 로컬 주행 기록 {}개가 있습니다.\n공유폴더 주행기록으로 복사할까요? (로컬 원본은 남습니다)'.format(len(files)))
        if answer == QtWidgets.QMessageBox.Yes:
            done, failed = upload(info, files)
            box = QtWidgets.QMessageBox.warning if failed else QtWidgets.QMessageBox.information
            box(window, '주행 기록 올리기', upload_message(done, failed))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('install', 'version'))
    parser.add_argument('--source', default=str(Path(__file__).resolve().parent.parent))
    parser.add_argument('--root', default=None)
    parser.add_argument('--no-register', action='store_true')
    parser.add_argument('--quick', action='store_true', help='version: 내용 대신 크기·수정시각 지문')
    args = parser.parse_args(argv)
    if args.command == 'version':
        print(tree_fingerprint(args.source) if args.quick else tree_version(args.source))
        return 0
    if installed_info(Path(args.source) / 'laptop_monitor'):
        print('이미 로컬 사본입니다. 공유폴더 원본의 GUI_설치_업데이트.sh를 실행하세요.', file=sys.stderr)
        return 2
    result = install(args.source, args.root, register=not args.no_register)
    print('설치 완료: 버전 {version}\n실행: 앱 목록 또는 바탕화면의 "{name}" / VIO 시험기 "{tester}"\n위치: {path}\n현재: {current}'.format(
        name=APP_NAME, tester=TESTER_NAME, **result))
    if result['pruned']:
        print('정리한 이전 버전: ' + ', '.join(result['pruned']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
