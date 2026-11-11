"""Operator window: classroom Wi-Fi plus saved laptop hotspots on the three boards."""
import json
import os
from pathlib import Path
import re
import socket
from PyQt5 import QtCore, QtWidgets
import network_control as control

LABELS = {'pi4': 'Pi4 / 플래닝 제어', 'pi5': 'Pi5 / 위치추정', 'jetson': 'Jetson / 인지 지도'}
USERS = {'pi4': 'user', 'pi5': 'user', 'jetson': 'nvidia'}
MODE_TEXT = {'classroom': '교실 Wi-Fi', 'laptop': '노트북 핫스팟', 'other': '다른 Wi-Fi', 'disconnected': 'Wi-Fi 연결 없음'}
REASON_TEXT = {'authentication_failed': 'SSH 인증 실패', 'ssh_unreachable': 'SSH 연결 불가',
               'ssh_timeout_unconfirmed': 'SSH 응답 시간 초과 / 성공 여부 미확인',
               'agent_missing_or_invalid_result': '보드 기능 응답 확인 필요 / 설치·정리 먼저',
               'agent_not_installed': '보드 기능 미설치 / 설치·정리 먼저',
               'wrong_board': '보드 종류가 다름', 'host_key_check_failed': 'SSH 호스트 키 확인 필요',
               'ssh_password_required': 'SSH 비밀번호 필요', 'root_required': 'sudo 권한 필요',
               'home_wifi_unknown': '교실 Wi-Fi 미기록 / 교실 Wi-Fi에 연결된 상태에서 등록',
               'not_on_classroom_wifi': '교실 Wi-Fi에 연결돼 있지 않음',
               'no_laptop_registered': '등록된 노트북 없음', 'laptop_in_use': '사용 중인 노트북은 삭제 불가',
               'invalid_laptop_addresses': '노트북 주소/보드 주소 오류', 'invalid_laptop_name': '노트북 이름 오류',
               'peer_address_not_current': '입력 주소가 보드의 현재 주소와 다름',
               'schedule_failed': '예약 실패 / 연결 변경 안 됨', 'invalid_board_address': '계정/주소 형식 오류'}
STATE_ROOT = (Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'nomad-trial').expanduser()


def default_laptop_name():
    name = re.sub(r'[^A-Za-z0-9_-]', '-', socket.gethostname())[:24].strip('-')
    return name or 'laptop'


class Section(QtWidgets.QWidget):
    """Collapsible block: a header button that shows or hides its body."""

    def __init__(self, title, expanded=True, parent=None):
        super().__init__(parent)
        self.title = title
        self.toggle = QtWidgets.QToolButton()
        self.toggle.setCheckable(True)
        self.toggle.setChecked(expanded)
        self.toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.toggle.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.toggle.setStyleSheet('QToolButton { font-weight: bold; text-align: left; padding: 4px; }')
        self.body = QtWidgets.QFrame()
        self.body.setFrameShape(QtWidgets.QFrame.StyledPanel)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(self.toggle)
        layout.addWidget(self.body)
        self.toggle.toggled.connect(self.set_expanded)
        self.set_expanded(expanded)

    def set_expanded(self, expanded):
        self.toggle.setArrowType(QtCore.Qt.DownArrow if expanded else QtCore.Qt.RightArrow)
        self.toggle.setText(self.title + ('' if expanded else '  (펼치기)'))
        self.body.setVisible(expanded)


class BoardWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str, dict)
    result = QtCore.pyqtSignal(dict)

    def __init__(self, action, targets, passwords, options, parent=None):
        super().__init__(parent)
        self.action, self.targets, self.passwords, self.options = action, targets, passwords, options

    def run(self):
        values = {}
        order = ['pi5', 'jetson', 'pi4'] if self.action.startswith('time_') else list(self.targets)
        try:
            for board in [b for b in order if b in self.targets]:
                host = self.targets[board]
                options = dict(self.options.get(board, {}))
                if self.action.startswith('time_'):
                    kind = self.action[5:]
                    values[board] = control.time_request(board, host, self.passwords[board], kind,
                                                         timeout_s=700 if kind == 'install' else 40)
                    if (kind == 'status' and board != 'pi5' and self.targets.get('pi5')
                            and not values[board].get('ok') and 'servers' in values[board]):
                        # Follower still polls an old Pi5 address: give it the one this window uses.
                        import clock_sync
                        values[board] = clock_sync._learn_pi5(values[board], board, host, self.passwords[board],
                                                              self.targets['pi5'])
                    values[board]['time'] = True
                else:
                    timeout = 90 if self.action == 'install' else 40
                    values[board] = control.request(board, host, self.passwords[board], self.action, timeout_s=timeout, **options)
                self.progress.emit(board, values[board])
            self.result.emit({'ok': bool(values) and all(v.get('ok') for v in values.values()),
                              'phase': self.action, 'boards': values})
        except Exception:
            self.result.emit({'ok': False, 'phase': self.action, 'boards': values, 'reason': 'local_network_control_error'})
        finally:
            self.passwords.clear()
            self.options.clear()


class NetworkDialog(QtWidgets.QDialog):
    hosts_applied = QtCore.pyqtSignal(dict)

    def __init__(self, hosts=None, ssh_password='', parent=None):
        super().__init__(parent)
        self.setWindowTitle('NOMAD / 교실 Wi-Fi · 노트북 핫스팟')
        self.resize(1040, 760)
        self.worker = None
        self.finder = None
        self.find_output = ''
        self.latest = {}
        self.after_success = None
        self.hotspots = []
        outer = QtWidgets.QVBoxLayout(self)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        content = QtWidgets.QWidget()
        scroll.setWidget(content)
        outer.addWidget(scroll, 10)
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        self.help_section = Section('설명', expanded=False)
        help_layout = QtWidgets.QVBoxLayout(self.help_section.body)
        note = QtWidgets.QLabel(
            '보드는 평소 교실 Wi-Fi를 쓰고, 등록한 노트북 핫스팟을 예비 Wi-Fi로 저장합니다. 야외처럼 교실 Wi-Fi가 없으면 '
            '보드가 스스로 노트북 핫스팟에 붙습니다. 실내 시험은 ‘실내 시험 시작’으로 교실 Wi-Fi를 끊어 노트북으로 옮깁니다. '
            'Wi-Fi 없는 상태가 안전장치 시간 이상이고 교실 Wi-Fi가 보이면 보드가 교실 Wi-Fi로 돌아갑니다. '
            '핫스팟은 한 번에 노트북 하나만 켜세요. Pi5와 Jetson 사이 직결 LAN은 그대로 씁니다.')
        note.setWordWrap(True)
        help_layout.addWidget(note)
        layout.addWidget(self.help_section)

        self.board_section = Section('보드', expanded=True)
        grid = QtWidgets.QGridLayout(self.board_section.body)
        for col, text in enumerate(('대상', 'SSH 주소', '개별 SSH 비밀번호 (빈칸이면 공통)', '상태')):
            grid.addWidget(QtWidgets.QLabel(text), 0, col)
        self.selected, self.hosts, self.passwords, self.statuses = {}, {}, {}, {}
        defaults = {'pi4': 'user@lpp.local', 'pi5': 'user@user-desktop.local', 'jetson': 'nvidia@ubuntu.local'}
        defaults.update(hosts or {})
        for row, board in enumerate(LABELS, 1):
            selected = QtWidgets.QCheckBox(LABELS[board])
            selected.setChecked(True)
            host = QtWidgets.QLineEdit(defaults[board])
            host.setMinimumWidth(210)
            password = QtWidgets.QLineEdit()
            password.setEchoMode(QtWidgets.QLineEdit.Password)
            password.setAccessibleName(LABELS[board] + ' SSH 비밀번호')
            label = QtWidgets.QLabel('미확인')
            label.setWordWrap(True)
            label.setMinimumWidth(220)
            self.selected[board], self.hosts[board], self.passwords[board], self.statuses[board] = selected, host, password, label
            for col, widget in enumerate((selected, host, password, label)):
                grid.addWidget(widget, row, col)
        self.ssh_password = QtWidgets.QLineEdit(ssh_password)
        self.ssh_password.setEchoMode(QtWidgets.QLineEdit.Password)
        grid.addWidget(QtWidgets.QLabel('공통 SSH 비밀번호 (sudo에도 사용)'), 4, 0)
        grid.addWidget(self.ssh_password, 4, 1)
        self.subnet = QtWidgets.QLineEdit()
        self.subnet.setPlaceholderText('검색 대역 / 빈칸이면 현재 PC 네트워크, 노트북 핫스팟이면 10.42.0.0/24')
        self.find_button = QtWidgets.QPushButton('보드 찾기')
        self.find_button.clicked.connect(self.discover)
        grid.addWidget(self.subnet, 5, 0, 1, 3)
        grid.addWidget(self.find_button, 5, 3)
        row_buttons = QtWidgets.QHBoxLayout()
        self.status_button = QtWidgets.QPushButton('현재 상태 확인')
        self.status_button.clicked.connect(lambda: self.board_action('status'))
        self.install_button = QtWidgets.QPushButton('보드 기능 설치 / 기존 300초 기능 정리')
        self.install_button.clicked.connect(lambda: self.board_action('install'))
        row_buttons.addWidget(self.status_button)
        row_buttons.addWidget(self.install_button)
        grid.addLayout(row_buttons, 6, 0, 1, 4)
        layout.addWidget(self.board_section)

        self.hotspot_section = Section('이 노트북 핫스팟', expanded=False)
        form = QtWidgets.QGridLayout(self.hotspot_section.body)
        self.hotspot_combo = QtWidgets.QComboBox()
        self.hotspot_info = QtWidgets.QLabel('핫스팟 프로필을 불러오세요.')
        self.hotspot_info.setWordWrap(True)
        self.refresh_button = QtWidgets.QPushButton('새로고침')
        self.refresh_button.clicked.connect(self.refresh_hotspots)
        self.hotspot_on = QtWidgets.QPushButton('핫스팟 켜기')
        self.hotspot_on.clicked.connect(lambda: self.switch_hotspot(True))
        self.hotspot_off = QtWidgets.QPushButton('핫스팟 끄기')
        self.hotspot_off.clicked.connect(lambda: self.switch_hotspot(False))
        self.hotspot_combo.currentIndexChanged.connect(self.show_hotspot)
        form.addWidget(QtWidgets.QLabel('프로필'), 0, 0)
        form.addWidget(self.hotspot_combo, 0, 1)
        form.addWidget(self.refresh_button, 0, 2)
        form.addWidget(self.hotspot_on, 0, 3)
        form.addWidget(self.hotspot_off, 0, 4)
        form.addWidget(self.hotspot_info, 1, 0, 1, 5)
        self.new_ssid = QtWidgets.QLineEdit()
        self.new_ssid.setPlaceholderText('새 핫스팟 이름(SSID)')
        self.new_password = QtWidgets.QLineEdit()
        self.new_password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.new_password.setPlaceholderText('새 핫스팟 비밀번호 8~63자')
        self.create_button = QtWidgets.QPushButton('새 핫스팟 만들기 (2.4GHz, 10.42.0.1)')
        self.create_button.clicked.connect(self.create_hotspot)
        form.addWidget(self.new_ssid, 2, 0, 1, 2)
        form.addWidget(self.new_password, 2, 2, 1, 2)
        form.addWidget(self.create_button, 2, 4)
        layout.addWidget(self.hotspot_section)

        self.register_section = Section('보드에 노트북 핫스팟 등록 (예비 Wi-Fi)', expanded=False)
        rform = QtWidgets.QGridLayout(self.register_section.body)
        self.laptop_name = QtWidgets.QLineEdit(default_laptop_name())
        self.ssid = QtWidgets.QLineEdit()
        self.wifi_password = QtWidgets.QLineEdit()
        self.wifi_password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.gateway = QtWidgets.QLineEdit(control.DEFAULT_GATEWAY)
        self.gateway.textChanged.connect(self.show_board_addresses)
        self.board_address_label = QtWidgets.QLabel()
        self.board_address_label.setWordWrap(True)
        self.fill_button = QtWidgets.QPushButton('이 노트북 핫스팟 정보 넣기')
        self.fill_button.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        self.fill_button.clicked.connect(self.fill_from_hotspot)
        self.watchdog = QtWidgets.QSpinBox()
        self.watchdog.setRange(30, 900)
        self.watchdog.setValue(120)
        self.watchdog.setSuffix('초')
        self.watchdog.setToolTip('Wi-Fi 없음이 이 시간 이상이고 교실 Wi-Fi가 보이면 교실로 복귀')
        for r, (text, widget) in enumerate((('노트북 이름(영문·숫자)', self.laptop_name), ('핫스팟 SSID', self.ssid),
                                             ('핫스팟 비밀번호', self.wifi_password), ('노트북 주소', self.gateway),
                                             ('안전장치 시간', self.watchdog))):
            rform.addWidget(QtWidgets.QLabel(text), r, 0)
            rform.addWidget(widget, r, 1)
        rform.addWidget(self.fill_button, 0, 2)
        rform.addWidget(self.board_address_label, 3, 2)
        self.register_button = QtWidgets.QPushButton('보드에 등록')
        self.register_button.clicked.connect(self.register)
        self.remove_button = QtWidgets.QPushButton('보드에서 이 노트북 삭제')
        self.remove_button.clicked.connect(lambda: self.board_action('remove', {'name': self.laptop_name.text().strip()}))
        self.watchdog_button = QtWidgets.QPushButton('안전장치 시간만 적용')
        rform.addWidget(QtWidgets.QLabel('Wi-Fi 없음이 이 시간 이상이고 교실 Wi-Fi가 보이면 교실로 복귀'), 4, 2)
        self.watchdog_button.clicked.connect(lambda: self.board_action('watchdog', {'seconds': self.watchdog.value()}))
        rbuttons = QtWidgets.QHBoxLayout()
        for button in (self.register_button, self.remove_button, self.watchdog_button):
            rbuttons.addWidget(button)
        rform.addLayout(rbuttons, 5, 0, 1, 3)
        layout.addWidget(self.register_section)

        self.time_section = Section('보드 시간 동기화 (Pi5 기준, 5ms)', expanded=False)
        tform = QtWidgets.QGridLayout(self.time_section.body)
        time_note = QtWidgets.QLabel('Pi5가 기준 시계이고 Jetson·Pi4가 chrony로 따라갑니다. 시험 시작과 플래너 연결 전에 각 보드가 '
                                     '스스로 5ms 이내인지 확인합니다. 설치는 시험을 끈 상태에서, 인터넷이 되는 교실 Wi-Fi에서 하세요. '
                                     'Pi5–Jetson 유선 주소(10.77.0.2 / 10.77.0.1)도 고정합니다.')
        time_note.setWordWrap(True)
        self.time_install_button = QtWidgets.QPushButton('시간 동기화 설치 · 유선 주소 고정')
        self.time_install_button.clicked.connect(lambda: self.board_action('time_install'))
        self.time_status_button = QtWidgets.QPushButton('시간 상태 확인')
        self.time_status_button.clicked.connect(lambda: self.board_action('time_status'))
        tform.addWidget(time_note, 0, 0, 1, 2)
        tform.addWidget(self.time_install_button, 1, 0)
        tform.addWidget(self.time_status_button, 1, 1)
        layout.addWidget(self.time_section)

        self.mode_section = Section('시험 모드와 GUI 주소', expanded=True)
        mform = QtWidgets.QGridLayout(self.mode_section.body)
        self.indoor_button = QtWidgets.QPushButton('실내 시험 시작 (8초 뒤 교실 Wi-Fi 끊기)')
        self.indoor_button.clicked.connect(lambda: self.board_action('indoor'))
        self.indoor_hotspot = QtWidgets.QCheckBox('예약 뒤 이 노트북 핫스팟 켜고 GUI 주소를 노트북 주소로')
        self.indoor_hotspot.setChecked(True)
        self.classroom_button = QtWidgets.QPushButton('교실로 복귀 (8초 뒤 교실 Wi-Fi 연결)')
        self.classroom_button.clicked.connect(lambda: self.board_action('classroom'))
        self.classroom_hotspot = QtWidgets.QCheckBox('예약 뒤 이 노트북 핫스팟 끄기')
        self.classroom_hotspot.setChecked(True)
        self.apply_laptop = QtWidgets.QPushButton('노트북 주소를 GUI에 적용')
        self.apply_laptop.clicked.connect(self.apply_laptop_addresses)
        self.apply_classroom = QtWidgets.QPushButton('교실 주소를 GUI·보드에 적용')
        self.apply_classroom.clicked.connect(self.apply_classroom_addresses)
        self.load_classroom = QtWidgets.QPushButton('저장된 교실 주소 넣기')
        self.load_classroom.clicked.connect(self.load_classroom_addresses)
        mform.addWidget(self.indoor_button, 0, 0)
        mform.addWidget(self.indoor_hotspot, 0, 1, 1, 2)
        mform.addWidget(self.classroom_button, 1, 0)
        mform.addWidget(self.classroom_hotspot, 1, 1, 1, 2)
        mform.addWidget(self.apply_laptop, 2, 0)
        mform.addWidget(self.apply_classroom, 2, 1)
        mform.addWidget(self.load_classroom, 2, 2)
        layout.addWidget(self.mode_section)
        layout.addStretch(1)

        self.message = QtWidgets.QLabel('처음 한 번 ‘보드 기능 설치’를 누른 뒤, 교실 Wi-Fi에 연결된 상태에서 노트북을 등록하세요.')
        self.message.setWordWrap(True)
        self.message.setMinimumHeight(48)
        outer.addWidget(self.message)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setMinimumHeight(48)
        self.log.setMaximumHeight(90)
        outer.addWidget(self.log, 0)
        privacy = QtWidgets.QLabel('PC에는 비밀번호를 저장하지 않습니다. 보드에는 NetworkManager의 root 전용 연결 프로필로 저장합니다.')
        privacy.setWordWrap(True)
        outer.addWidget(privacy)
        self.close_button = QtWidgets.QPushButton('닫기')
        self.close_button.clicked.connect(self.accept)
        outer.addWidget(self.close_button)
        self.find_deadline = QtCore.QTimer(self)
        self.find_deadline.setSingleShot(True)
        self.find_deadline.timeout.connect(self.cancel_discovery)
        self.show_board_addresses()
        QtCore.QTimer.singleShot(0, self.refresh_hotspots)

    # ---------- helpers ----------
    def controls(self):
        return [self.find_button, self.subnet, self.status_button, self.install_button, self.refresh_button,
                self.time_install_button, self.time_status_button,
                self.hotspot_on, self.hotspot_off, self.create_button, self.fill_button, self.register_button,
                self.remove_button, self.watchdog_button, self.indoor_button, self.classroom_button,
                self.apply_laptop, self.apply_classroom, self.load_classroom, self.ssh_password,
                *self.hosts.values(), *self.selected.values(), *self.passwords.values()]

    def busy(self, value):
        for widget in self.controls():
            widget.setEnabled(not value)
        self.close_button.setEnabled(not value)

    def say(self, text):
        self.message.setText(text)
        self.log.appendPlainText(text)

    def laptop_addresses(self):
        return control.board_addresses(self.gateway.text().strip())

    def show_board_addresses(self):
        try:
            values = self.laptop_addresses()
            self.board_address_label.setText('보드 고정 주소: Pi4 {pi4} / Pi5 {pi5} / Jetson {jetson}'.format(**values))
        except ValueError as error:
            self.board_address_label.setText(str(error))

    def targets_and_passwords(self, all_boards=False):
        targets = {}
        for board, checkbox in self.selected.items():
            if checkbox.isChecked() or all_boards:
                targets[board] = control.validate_host(self.hosts[board].text())
        if not targets:
            raise ValueError('대상 보드를 선택하세요.')
        if all_boards and set(targets) != set(LABELS):
            raise ValueError('세 보드를 모두 선택하세요.')
        if len({v.split('@')[1].lower() for v in targets.values()}) != len(targets):
            raise ValueError('서로 다른 보드 주소를 입력하세요.')
        passwords = {b: self.passwords[b].text() or self.ssh_password.text() for b in targets}
        if not all(passwords.values()):
            raise ValueError('보드 SSH 비밀번호를 입력하세요.')
        return targets, passwords

    # ---------- board actions ----------
    def board_action(self, action, common=None, per_board=None, all_boards=False):
        if self.worker is not None or self.finder is not None:
            return
        try:
            if action in ('indoor', 'classroom', 'peers', 'time_install'):
                all_boards = True
            targets, passwords = self.targets_and_passwords(all_boards)
            if action == 'remove' and not re.fullmatch(r'[A-Za-z0-9_-]{1,24}', (common or {}).get('name', '')):
                raise ValueError('노트북 이름은 영문·숫자·-·_ 1~24자입니다.')
        except ValueError as error:
            self.message.setText(str(error))
            return
        options = {b: dict(common or {}, **((per_board or {}).get(b, {}))) for b in targets}
        self.busy(True)
        self.say({'status': '보드 상태를 확인합니다.', 'install': '보드 기능을 설치하고 기존 300초 기능과 예전 핫스팟 프로필을 정리합니다.',
                  'register': '노트북 핫스팟을 보드에 예비 Wi-Fi로 등록합니다. 현재 연결은 바꾸지 않습니다.',
                  'remove': '보드에서 노트북 프로필을 삭제합니다.', 'watchdog': '안전장치 시간을 적용합니다.',
                  'indoor': '세 보드에 8초 뒤 교실 Wi-Fi 끊기를 예약합니다.',
                  'classroom': '세 보드에 8초 뒤 교실 Wi-Fi 연결을 예약합니다.',
                  'peers': '교실 주소를 보드의 플래닝 연결 설정에 적용합니다.',
                  'time_install': 'Pi5 → Jetson → Pi4 순서로 시간 동기화를 설치하고 유선 주소를 고정합니다. 보드당 수 분 걸릴 수 있습니다.',
                  'time_status': '각 보드의 시간 동기화 상태를 확인합니다.'}[action])
        self.worker = BoardWorker(action, targets, passwords, options, self)
        self.worker.progress.connect(self.on_progress)
        self.worker.result.connect(self.on_result)
        self.worker.finished.connect(self.worker_finished)
        self.worker.start()

    def on_progress(self, board, value):
        if value.get('time'):
            text = ('시간 ✓ ' if value.get('ok') else '시간 ✗ ') + (value.get('message') or
                                                               REASON_TEXT.get(value.get('reason', ''), value.get('reason', '')))
            if value.get('wired'):
                text += ' / 유선 ' + {'fixed_up': '고정·연결', 'fixed_down': '고정·케이블 없음',
                                      'temporary_up': '임시', 'temporary_down': '임시·끊김'}.get(value['wired'], value['wired'])
            self.statuses[board].setText(text)
            self.log.appendPlainText(LABELS[board] + ': ' + text)
            return
        self.latest[board] = value
        if value.get('ok'):
            text = MODE_TEXT.get(value.get('mode'), '미확인')
            if value.get('active_ssid'):
                text += ' (' + value['active_ssid'] + ')'
            text += ' / ' + (', '.join(value.get('ipv4', [])) or '주소 없음')
            laptops = ', '.join(item['name'] + '=' + item['address'] for item in value.get('laptops', []))
            text += '\n등록 노트북: ' + (laptops or '없음') + ' / 안전장치 {}초 {}'.format(
                value.get('watchdog_seconds'), '동작' if value.get('watchdog') == 'active' else '꺼짐')
            if value.get('pending'):
                text += ' / 예약: ' + value['pending']
            if value.get('removed'):
                text += '\n정리: ' + ', '.join(value['removed'])
        else:
            reason = value.get('reason', 'remote_network_error')
            text = REASON_TEXT.get(reason, reason)
        self.statuses[board].setText(text)
        self.log.appendPlainText(LABELS[board] + ': ' + text.replace('\n', ' / '))

    def on_result(self, result):
        phase, ok = result.get('phase'), result.get('ok')
        self.wifi_password.clear()
        if phase == 'indoor' and ok:
            self.say('세 보드가 8초 뒤 교실 Wi-Fi를 끊고 저장된 노트북 핫스팟을 찾습니다.')
            if self.indoor_hotspot.isChecked() and self.current_hotspot():
                QtCore.QTimer.singleShot(1500, lambda: self.switch_hotspot(True, then_apply_laptop=True))
        elif phase == 'indoor':
            self.say('일부 보드 예약에 실패했습니다. 예약된 보드만 옮겨갈 수 있으니 상태 확인 후 교실로 복귀를 누르세요.')
        elif phase == 'classroom' and ok:
            self.say('세 보드가 8초 뒤 교실 Wi-Fi에 연결합니다. 이 PC도 교실 Wi-Fi로 돌아온 뒤 저장된 교실 주소를 적용하세요.')
            if self.classroom_hotspot.isChecked() and self.current_hotspot() and self.current_hotspot()['active']:
                QtCore.QTimer.singleShot(1500, lambda: self.switch_hotspot(False))
        elif phase == 'peers' and ok:
            hosts = {b: self.hosts[b].text().strip() for b in LABELS}
            self.save_sets(classroom=hosts)
            self.hosts_applied.emit(hosts)
            self.say('교실 주소를 GUI와 세 보드의 플래닝 연결 설정에 적용했습니다.')
        elif phase == 'time_install':
            self.say('시간 동기화 설치를 마쳤습니다. 방금 설치한 보드는 Pi5에 맞추는 데 수십 초 걸릴 수 있으니 잠시 뒤 ‘시간 상태 확인’을 누르세요.'
                     if ok else '일부 보드 설치에 실패했거나 아직 5ms 안에 들지 않았습니다. 잠시 뒤 ‘시간 상태 확인’으로 다시 보세요.')
        elif phase == 'time_status':
            self.say('세 보드 모두 시간 기준을 만족합니다.' if ok else '시간 기준을 만족하지 않는 보드가 있습니다. 보드별 상태를 확인하세요.')
        elif phase == 'register' and ok:
            self.say('세 보드에 노트북 핫스팟을 등록했습니다. 야외에서는 자동으로, 실내에서는 ‘실내 시험 시작’으로 연결합니다.')
        elif ok:
            self.say('보드 응답을 확인했습니다. 보드별 상태를 확인하세요.')
        else:
            self.say('일부 보드에서 실패했습니다. 보드별 상태를 확인하세요.')

    def worker_finished(self):
        self.worker.deleteLater()
        self.worker = None
        self.busy(False)

    def register(self):
        try:
            name = self.laptop_name.text().strip()
            if not re.fullmatch(r'[A-Za-z0-9_-]{1,24}', name):
                raise ValueError('노트북 이름은 영문·숫자·-·_ 1~24자입니다.')
            control.validate_credentials(self.ssid.text(), self.wifi_password.text())
            addresses = self.laptop_addresses()
            gateway = self.gateway.text().strip()
        except ValueError as error:
            self.message.setText(str(error))
            return
        self.board_action('register', {'name': name, 'ssid': self.ssid.text(), 'password': self.wifi_password.text(),
                                       'gateway': gateway, 'addresses': addresses, 'seconds': self.watchdog.value()})

    # ---------- addresses ----------
    def sets_path(self):
        return STATE_ROOT / 'network_sets.json'

    def save_sets(self, **values):
        try:
            current = json.loads(self.sets_path().read_text())
        except (OSError, ValueError):
            current = {}
        current.update(values)
        self.sets_path().parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        temporary = self.sets_path().with_suffix('.tmp')
        temporary.write_text(json.dumps(current, ensure_ascii=False) + '\n')
        temporary.chmod(0o600)
        temporary.replace(self.sets_path())

    def load_classroom_addresses(self):
        try:
            saved = json.loads(self.sets_path().read_text())['classroom']
            for board in LABELS:
                self.hosts[board].setText(control.validate_host(saved[board]))
            self.message.setText('저장된 교실 주소를 넣었습니다. 상태 확인 후 ‘교실 주소를 GUI·보드에 적용’을 누르세요.')
        except (OSError, ValueError, KeyError, TypeError):
            self.message.setText('저장된 교실 주소가 없습니다. 보드 찾기로 주소를 찾으세요.')

    def apply_laptop_addresses(self):
        try:
            addresses = self.laptop_addresses()
        except ValueError as error:
            self.message.setText(str(error))
            return
        hosts = {b: USERS[b] + '@' + addresses[b] for b in LABELS}
        for board, host in hosts.items():
            self.hosts[board].setText(host)
        self.hosts_applied.emit(hosts)
        self.say('GUI 보드 주소를 노트북 핫스팟 고정 주소로 바꿨습니다: ' + ', '.join(hosts.values()))

    def apply_classroom_addresses(self):
        try:
            peers = {}
            for board in LABELS:
                address = control.validate_host(self.hosts[board].text()).split('@')[1]
                state = self.latest.get(board, {})
                if not state.get('ok') or address not in state.get('ipv4', []) or state.get('mode') != 'classroom':
                    raise ValueError('교실 Wi-Fi에서 ‘현재 상태 확인’ 후, 각 보드의 실제 주소를 주소 칸에 넣으세요.')
                peers[board] = address
        except ValueError as error:
            self.message.setText(str(error))
            return
        self.board_action('peers', {'peers': peers})

    # ---------- this laptop's hotspot ----------
    def current_hotspot(self):
        index = self.hotspot_combo.currentIndex()
        return self.hotspots[index] if 0 <= index < len(self.hotspots) else None

    def refresh_hotspots(self):
        keep = (self.current_hotspot() or {}).get('uuid')
        try:
            self.hotspots = control.local_hotspots()
        except RuntimeError as error:
            self.hotspots = []
            self.hotspot_info.setText('핫스팟 정보를 읽지 못했습니다: ' + str(error))
        self.hotspot_combo.blockSignals(True)
        self.hotspot_combo.clear()
        for item in self.hotspots:
            self.hotspot_combo.addItem('{} (SSID {})'.format(item['name'], item['ssid']))
        for i, item in enumerate(self.hotspots):
            if item['uuid'] == keep:
                self.hotspot_combo.setCurrentIndex(i)
        self.hotspot_combo.blockSignals(False)
        self.show_hotspot()

    def show_hotspot(self):
        item = self.current_hotspot()
        if item is None:
            if not self.hotspot_info.text().startswith('핫스팟 정보를'):
                self.hotspot_info.setText('이 PC에 핫스팟 프로필이 없습니다. 아래에서 새로 만드세요.')
            return
        self.hotspot_info.setText('SSID {} / 주소 {} / 대역 {} / {}'.format(
            item['ssid'], item['gateway'] or '미정', item['band'], '켜짐' if item['active'] else '꺼짐 (켜면 이 PC의 교실 Wi-Fi가 끊길 수 있음)'))

    def fill_from_hotspot(self):
        item = self.current_hotspot()
        if item is None:
            self.message.setText('먼저 이 노트북 핫스팟을 만들거나 선택하세요.')
            return
        try:
            password = control.local_hotspot_password(item['uuid'])
        except RuntimeError as error:
            self.message.setText('핫스팟 비밀번호를 읽지 못했습니다. 직접 입력하세요. ' + str(error))
            password = ''
        self.ssid.setText(item['ssid'])
        self.wifi_password.setText(password)
        if item['gateway']:
            self.gateway.setText(item['gateway'])
        self.message.setText('이 노트북 핫스팟 정보를 넣었습니다. ‘보드에 등록’을 누르세요.')

    def create_hotspot(self):
        try:
            control.local_hotspot_create(self.new_ssid.text(), self.new_password.text())
        except (ValueError, RuntimeError) as error:
            self.message.setText('핫스팟 생성 실패: ' + str(error))
            return
        finally:
            self.new_password.clear()
        self.refresh_hotspots()
        for i, item in enumerate(self.hotspots):
            if item['name'] == control.LOCAL_PROFILE:
                self.hotspot_combo.setCurrentIndex(i)
        self.say('이 노트북 핫스팟 프로필을 만들었습니다(꺼진 상태). ‘이 노트북 핫스팟 정보 넣기’ 후 보드에 등록하세요.')

    def switch_hotspot(self, on, then_apply_laptop=False):
        item = self.current_hotspot()
        if item is None:
            self.message.setText('핫스팟 프로필을 선택하세요.')
            return
        self.say('이 노트북 핫스팟을 {}니다.'.format('켭' if on else '끕'))
        QtWidgets.QApplication.processEvents()
        try:
            control.local_hotspot_switch(item['uuid'], on)
        except RuntimeError as error:
            self.say('핫스팟 {} 실패: {}'.format('켜기' if on else '끄기', error))
            return
        self.refresh_hotspots()
        if on and then_apply_laptop:
            self.apply_laptop_addresses()
            self.say('보드가 붙는 데 수십 초 걸릴 수 있습니다. 잠시 뒤 ‘현재 상태 확인’을 누르세요.')

    # ---------- discovery ----------
    def discover(self):
        if self.worker is not None or self.finder is not None:
            return
        import ipaddress
        arguments = [str(Path(__file__).parent / 'scripts/find_ssh_boards.sh')]
        subnet = self.subnet.text().strip()
        if subnet:
            try:
                network = ipaddress.ip_network(subnet, strict=False)
                if network.version != 4 or network.num_addresses > 4096:
                    raise ValueError()
                arguments += ['--subnet', str(network)]
            except ValueError:
                self.message.setText('검색 대역은 IPv4 /20 또는 더 작은 대역을 사용하세요.')
                return
        self.busy(True)
        self.find_output = ''
        self.finder = QtCore.QProcess(self)
        discovery_env = QtCore.QProcessEnvironment.systemEnvironment()
        discovery_env.insert("NOMAD_BOARD_PASSWORD", self.ssh_password.text())
        self.finder.setProcessEnvironment(discovery_env)
        self.finder.readyReadStandardOutput.connect(self.read_discovery)
        self.finder.finished.connect(self.discovery_finished)
        self.finder.errorOccurred.connect(self.discovery_error)
        self.finder.start('bash', arguments)
        self.find_deadline.start(90000)
        self.message.setText('현재 PC 네트워크에서 실제 보드 모델을 확인하고 있습니다.')

    def read_discovery(self):
        self.find_output += bytes(self.finder.readAllStandardOutput()).decode('utf-8', errors='replace')
        self.find_output = self.find_output[-16384:]

    def discovery_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self.discovery_finished(-1, QtCore.QProcess.CrashExit)

    def cancel_discovery(self):
        if self.finder is not None:
            self.finder.kill()

    def discovery_finished(self, code, status):
        if self.finder is None:
            return
        self.read_discovery()
        self.find_deadline.stop()
        found = 0
        for line in self.find_output.splitlines():
            match = re.fullmatch(r'(Pi 4|Pi 5|Jetson):\s+ssh\s+(\S+)', line.strip())
            if match:
                board = {'Pi 4': 'pi4', 'Pi 5': 'pi5', 'Jetson': 'jetson'}[match[1]]
                self.hosts[board].setText(match[2])
                found += 1
        self.finder.deleteLater()
        self.finder = None
        self.busy(False)
        self.message.setText('보드 {}개를 찾았습니다. ‘현재 상태 확인’으로 연결을 확인하세요.'.format(found))

    def done(self, code):
        if self.worker is not None or self.finder is not None:
            return
        self.ssh_password.clear()
        self.wifi_password.clear()
        self.new_password.clear()
        for field in self.passwords.values():
            field.clear()
        super().done(code)

    def closeEvent(self, event):
        if self.worker is not None or self.finder is not None:
            event.ignore()
        else:
            self.done(QtWidgets.QDialog.Rejected)
            event.accept()
