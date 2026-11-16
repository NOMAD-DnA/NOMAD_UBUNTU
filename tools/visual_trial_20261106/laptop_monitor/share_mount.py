#!/usr/bin/env python3
"""Reach the team shared folder from a teammate laptop, also in the field (2026-11-12).

The shared folder is served by one laptop (Samba [shared]). In the classroom it
has a DHCP address; in the field teammates join that laptop's hotspot
(10.42.0.1) because phone hotspots block device-to-device traffic. A local GUI
copy remembers where the share was when it was installed, which is usually a
classroom mount that appears empty in the field. This module finds the share by
name/hotspot address (share_servers.json), mounts it with gio (GVFS, no sudo)
and returns the GUI source folder inside it.

Password: taken from the caller (dialog) or the desktop keyring (libsecret,
only if the user chose to save it); passed to gio on stdin, never written to a
file, argument or log. Every network step is time-bounded.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import subprocess

CONFIG = Path(__file__).with_name('share_servers.json')
SECRET_LABEL = 'NOMAD 공유폴더'
REASON_TEXT = {
    'local_source': '공유폴더 원본 경로가 열려 있음',
    'mounted': '공유폴더 연결됨',
    'password_needed': '공유폴더 비밀번호가 필요함',
    'auth_failed': '공유폴더 아이디·비밀번호가 맞지 않음',
    'unreachable': '공유폴더 PC를 찾지 못함 (같은 Wi-Fi·핫스팟인지 확인)',
    'gio_missing': 'gio 명령이 없어 연결할 수 없음',
    'folder_missing': '공유폴더에 GUI 원본 폴더가 없음',
    'config_missing': '공유폴더 주소 설정(share_servers.json) 없음',
}


def load_config(path=CONFIG):
    try:
        value = json.loads(Path(path).read_text(encoding='utf-8'))
        servers = [str(s).strip() for s in value.get('servers', []) if str(s).strip()]
        if not servers or not value.get('share') or not value.get('relative'):
            raise ValueError()
        return {'share': str(value['share']), 'servers': servers, 'relative': str(value['relative']),
                'user': str(value.get('user', 'user')), 'domain': str(value.get('domain', 'WORKGROUP'))}
    except (OSError, ValueError, AttributeError, TypeError):
        return None


def _run(args, timeout, stdin_text=None):
    try:
        result = subprocess.run(args, input=stdin_text, stdin=None if stdin_text is not None else subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=timeout,
                                env=dict(os.environ, LC_ALL='C'))
        return result.returncode, result.stdout + result.stderr
    except (OSError, subprocess.TimeoutExpired):
        return None, ''


def path_ready(path, timeout=3):
    """True when <path>/laptop_monitor/launcher.py answers within timeout (dead mounts block)."""
    code, _ = _run(['stat', '-t', str(Path(path) / 'laptop_monitor' / 'launcher.py')], timeout)
    return code == 0


def gvfs_root():
    return Path(os.environ.get('XDG_RUNTIME_DIR', '/run/user/{}'.format(os.getuid()))) / 'gvfs'


def mounted_path(server, share, root=None):
    """GVFS directory of smb://server/share if it is mounted, else None."""
    root = Path(root or gvfs_root())
    want_server, want_share = 'server=' + server.lower(), 'share=' + share.lower()
    try:
        for entry in root.iterdir():
            parts = entry.name.lower().split(':', 1)
            if parts[0] == 'smb-share' and len(parts) == 2:
                fields = parts[1].split(',')
                if want_server in fields and want_share in fields:
                    return entry
    except OSError:
        pass
    return None


def reachable(server, timeout=1.5):
    """Name resolves (bounded) and SMB port 445 answers."""
    code, out = _run(['getent', 'ahostsv4', server], 3)
    if code != 0 or not out.split():
        return False
    address = out.split()[0]
    try:
        with socket.create_connection((address, 445), timeout=timeout):
            return True
    except OSError:
        return False


def keyring_lookup(config):
    try:
        import gi
        gi.require_version('Secret', '1')
        from gi.repository import Secret
        return Secret.password_lookup_sync(_schema(Secret), _attributes(config), None)
    except Exception:
        return None


def keyring_store(config, password):
    try:
        import gi
        gi.require_version('Secret', '1')
        from gi.repository import Secret
        return bool(Secret.password_store_sync(_schema(Secret), _attributes(config), Secret.COLLECTION_DEFAULT,
                                               SECRET_LABEL, password, None))
    except Exception:
        return False


def keyring_clear(config):
    try:
        import gi
        gi.require_version('Secret', '1')
        from gi.repository import Secret
        return bool(Secret.password_clear_sync(_schema(Secret), _attributes(config), None))
    except Exception:
        return False


def _schema(Secret):
    return Secret.Schema.new('kr.nomad.SharedFolder', Secret.SchemaFlags.NONE,
                             {'share': Secret.SchemaAttributeType.STRING, 'user': Secret.SchemaAttributeType.STRING})


def _attributes(config):
    return {'share': config['share'], 'user': config['user']}


def mount(server, config, password, timeout=15):
    """gio mount smb://server/share with user/domain/password on stdin. Returns (ok, reason)."""
    if password is None or '\n' in password or '\r' in password:
        return False, 'password_needed'
    code, out = _run(['gio', 'mount', 'smb://{}/{}'.format(server, config['share'])], timeout,
                     stdin_text='{}\n{}\n{}\n'.format(config['user'], config['domain'], password))
    if code is None and not out:
        return False, 'unreachable'
    if mounted_path(server, config['share']) is not None:
        return True, 'mounted'
    lowered = out.lower()
    if 'password' in lowered or 'permission' in lowered or 'authentication' in lowered or 'logon' in lowered:
        return False, 'auth_failed'
    return False, 'unreachable'


def find_mounted(config):
    """(source path, server) of an already mounted share that holds the GUI source."""
    for server in config['servers']:
        base = mounted_path(server, config['share'])
        if base is not None and path_ready(base / config['relative']):
            return base / config['relative'], server
    return None, None


def connect(password=None, save=False, config=None, interactive=True):
    """Find or mount the shared folder. Result dict: ok, reason, source, server, message.
    interactive=False: use only an existing mount or the keyring (no password from caller)."""
    config = config or load_config()
    if not config:
        return _result(False, 'config_missing')
    source, server = find_mounted(config)
    if source:
        return _result(True, 'mounted', source, server)
    if _run(['which', 'gio'], 3)[0] != 0:
        return _result(False, 'gio_missing')
    servers = [s for s in config['servers'] if reachable(s)]
    if not servers:
        return _result(False, 'unreachable')
    stored = keyring_lookup(config)
    secret = password if (interactive and password) else stored
    if not secret:
        return _result(False, 'password_needed', server=servers[0])
    reason = 'unreachable'
    for server in servers:
        ok, reason = mount(server, config, secret)
        if ok:
            base = mounted_path(server, config['share'])
            if not path_ready(base / config['relative']):
                return _result(False, 'folder_missing', server=server)
            if save and password:
                keyring_store(config, password)
            return _result(True, 'mounted', base / config['relative'], server)
        if reason == 'auth_failed':
            break
    return _result(False, reason, server=servers[0])


def _result(ok, reason, source=None, server=None):
    message = REASON_TEXT.get(reason, reason)
    if server:
        message += ' (smb://{}/)'.format(server)
    return {'ok': ok, 'reason': reason, 'source': str(source) if source else '', 'server': server or '',
            'message': message}


def resolve_source(info, ask_password=None):
    """GUI source folder for a local copy: the recorded path if it answers, else the
    shared folder found/mounted by name or hotspot address. ask_password() -> (password, save)
    or None is called only when a share PC answers but no saved password works."""
    recorded = info.get('source')
    if recorded and path_ready(recorded):
        return _result(True, 'local_source', recorded)
    result = connect(interactive=False)
    if result['reason'] in ('password_needed', 'auth_failed') and ask_password is not None:
        answer = ask_password(result)
        if answer:
            result = connect(password=answer[0], save=answer[1])
    return result


def ask_password_dialog(parent, result):
    """Qt dialog: shared folder password + save-to-keyring choice. Returns (password, save) or None."""
    from PyQt5 import QtWidgets
    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle('공유폴더 연결')
    layout = QtWidgets.QVBoxLayout(dialog)
    text = '공유폴더 PC를 찾았습니다 {}.\n공유폴더 비밀번호를 입력하세요 (아이디: {}).'.format(
        'smb://' + result['server'] + '/' if result.get('server') else '', (load_config() or {}).get('user', 'user'))
    if result.get('reason') == 'auth_failed':
        text = '저장된 비밀번호가 맞지 않습니다.\n' + text
    layout.addWidget(QtWidgets.QLabel(text))
    field = QtWidgets.QLineEdit()
    field.setEchoMode(QtWidgets.QLineEdit.Password)
    layout.addWidget(field)
    save = QtWidgets.QCheckBox('이 PC 키링에 저장 (다음부터 자동 연결)')
    save.setChecked(True)
    layout.addWidget(save)
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)
    try:
        if dialog.exec_() != QtWidgets.QDialog.Accepted or not field.text():
            return None
        return field.text(), save.isChecked()
    finally:
        field.clear()
        dialog.deleteLater()


if __name__ == '__main__':
    print(json.dumps(connect(interactive=False), ensure_ascii=False))
