"""Operator connection window for the three installed network agents."""
from pathlib import Path
import re
from PyQt5 import QtCore, QtWidgets
import network_control as control

LABELS = {'pi4': 'Pi4 / 플래닝 제어', 'pi5': 'Pi5 / 위치추정', 'jetson': 'Jetson / 인지 지도'}
STATE_TEXT = {'idle': '현재 Wi-Fi', 'prepared': '준비 완료 / 현재 연결 유지',
              'queued': '전환 예약 / 8초 뒤 시작', 'switching': '연결 전환 중',
              'awaiting_confirmation': '새 Wi-Fi 연결 / 재접속 확정 필요', 'committed': '재접속 확정 완료',
              'restore_queued': '이전 Wi-Fi 복귀 예약 / 8초 뒤 시작', 'rolling_back': '복귀 중',
              'rolled_back': '이전 Wi-Fi 복귀 확인', 'rollback_failed': '복귀 실패', 'schedule_failed': '예약 실패 / 전환 안 됨'}
REASON_TEXT = {'authentication_failed': 'SSH 인증 실패', 'ssh_unreachable': 'SSH 연결 불가',
               'ssh_timeout_unconfirmed': 'SSH 응답 시간 초과 / 성공 여부 미확인',
               'agent_missing_or_invalid_result': '보드 기능 설치 또는 응답 확인 필요',
               'wrong_board': '보드 종류가 다름', 'wrong_machine': '준비한 보드와 다름',
               'switch_pending': '기존 전환 진행 중', 'reconnect_via_wifi_address': '새 Wi-Fi 주소로 접속 필요',
               'confirmation_expired': '확정 시간 만료', 'previous_wifi_not_connected': '복귀할 기존 Wi-Fi 연결 필요',
               'host_key_check_failed': 'SSH 호스트 키 확인 필요', 'ssh_password_required': 'SSH 비밀번호 필요',
               'connection_not_confirmed': '재접속 확정을 먼저 완료하세요',
               'board_peer_unreachable': '보드 사이 SSH 연결 불가 / 핫스팟 기기 격리 확인'}


class NetworkWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str, dict)
    result = QtCore.pyqtSignal(dict)

    def __init__(self, action, targets, passwords, options, identities, parent=None):
        super().__init__(parent)
        self.action, self.targets, self.passwords = action, targets, passwords
        self.options, self.identities = options, identities

    def run(self):
        try:
            if self.action == 'connect':
                value = control.connect_many(self.targets, self.passwords, **self.options,
                                             progress=lambda b, v: self.progress.emit(b, v))
            else:
                values = {}
                for board, host in self.targets.items():
                    identity = self.identities.get(board, {})
                    options = {}
                    if self.action in ('confirm', 'restore'):
                        # Refresh from the entered address, retaining an earlier
                        # machine identity when this dialog prepared the switch.
                        state = control.request(board, host, self.passwords[board], 'status',
                                                **({'machine_id': identity['machine_id']} if identity.get('machine_id') else {}))
                        if not state.get('ok') or not state.get('token'):
                            values[board] = state if not state.get('ok') else control.failed('not_prepared')
                            self.progress.emit(board, values[board])
                            continue
                        if identity.get('token') and state['token'] != identity['token']:
                            values[board] = control.failed('stale_request')
                            self.progress.emit(board, values[board])
                            continue
                        options = {'token': state['token'], 'machine_id': state['machine_id']}
                    if self.action in ('status', 'peers') and identity.get('machine_id'):
                        options['machine_id'] = identity['machine_id']
                    if self.action == 'peers':
                        options['peers'] = self.options['peers']
                    values[board] = control.request(board, host, self.passwords[board], self.action, **options)
                    self.progress.emit(board, values[board])
                value = {'ok': bool(values) and all(v.get('ok') for v in values.values()),
                         'phase': self.action, 'boards': values}
            self.result.emit(value)
        except Exception:
            self.result.emit({'ok': False, 'phase': self.action, 'boards': {}, 'reason': 'local_network_control_error'})
        finally:
            self.passwords.clear()
            self.options.clear()


class NetworkDialog(QtWidgets.QDialog):
    hosts_applied = QtCore.pyqtSignal(dict)

    def __init__(self, hosts=None, ssh_password='', parent=None):
        super().__init__(parent)
        self.setWindowTitle('NOMAD / Wi-Fi 및 핫스팟 연결')
        self.resize(980, 680)
        self.worker = None
        self.finder = None
        self.identities = {}
        self.latest = {}
        self.find_output = ''
        layout = QtWidgets.QVBoxLayout(self)
        note = QtWidgets.QLabel('기존 핫스팟에 세 보드를 연결합니다. 이름은 IP 주소가 아닌 Wi-Fi 이름(SSID)입니다.\n'
                               '전환 후 이 PC도 같은 Wi-Fi에 연결하고 새 보드 주소로 재접속을 확정하세요. Pi5와 Jetson 사이 직결 LAN은 계속 사용합니다.')
        note.setWordWrap(True)
        layout.addWidget(note)
        grid = QtWidgets.QGridLayout()
        for col, text in enumerate(('대상', 'SSH 주소 / 전환 뒤 새 주소', '개별 SSH 비밀번호 (빈칸이면 공통)', '상태')):
            grid.addWidget(QtWidgets.QLabel(text), 0, col)
        self.selected, self.hosts, self.passwords, self.statuses = {}, {}, {}, {}
        defaults = {'pi4': 'user@lpp.local', 'pi5': 'user@user-desktop.local', 'jetson': 'nvidia@ubuntu.local'}
        defaults.update(hosts or {})
        for row, board in enumerate(LABELS, 1):
            selected = QtWidgets.QCheckBox(LABELS[board])
            selected.setChecked(True)
            host = QtWidgets.QLineEdit(defaults[board])
            password = QtWidgets.QLineEdit()
            password.setEchoMode(QtWidgets.QLineEdit.Password)
            password.setAccessibleName(LABELS[board] + ' SSH 비밀번호')
            label = QtWidgets.QLabel('미확인')
            label.setWordWrap(True)
            label.setMinimumWidth(260)
            self.selected[board], self.hosts[board], self.passwords[board], self.statuses[board] = selected, host, password, label
            for col, widget in enumerate((selected, host, password, label)):
                grid.addWidget(widget, row, col)
        layout.addLayout(grid)
        form = QtWidgets.QFormLayout()
        self.ssh_password = QtWidgets.QLineEdit(ssh_password)
        self.ssh_password.setEchoMode(QtWidgets.QLineEdit.Password)
        form.addRow('공통 SSH 비밀번호', self.ssh_password)
        self.ssid = QtWidgets.QLineEdit()
        self.ssid.setPlaceholderText('휴대폰 또는 공유기의 Wi-Fi 이름')
        form.addRow('핫스팟 이름 / SSID', self.ssid)
        self.wifi_password = QtWidgets.QLineEdit()
        self.wifi_password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.wifi_password.setPlaceholderText('SSH 비밀번호와 구분 / WPA2 호환 핫스팟')
        form.addRow('핫스팟 비밀번호', self.wifi_password)
        self.reveal = QtWidgets.QCheckBox('핫스팟 비밀번호 표시')
        self.reveal.toggled.connect(lambda checked: self.wifi_password.setEchoMode(
            QtWidgets.QLineEdit.Normal if checked else QtWidgets.QLineEdit.Password))
        form.addRow('', self.reveal)
        self.timeout = QtWidgets.QSpinBox()
        self.timeout.setRange(120, 900)
        self.timeout.setValue(300)
        self.timeout.setSuffix('초 / 미확정 시 이전 Wi-Fi 복귀 시도')
        form.addRow('재접속 제한 시간', self.timeout)
        layout.addLayout(form)
        find_row = QtWidgets.QHBoxLayout()
        self.subnet = QtWidgets.QLineEdit()
        self.subnet.setPlaceholderText('주소 검색 대역 / 빈칸이면 현재 PC 네트워크')
        self.find_button = QtWidgets.QPushButton('현재 네트워크에서 보드 찾기')
        self.find_button.clicked.connect(self.discover)
        find_row.addWidget(self.subnet, 1)
        find_row.addWidget(self.find_button)
        layout.addLayout(find_row)
        buttons = QtWidgets.QHBoxLayout()
        self.actions = {}
        for action, label in (('status', '현재 상태 확인'), ('connect', '선택 보드 연결 전환'),
                              ('confirm', '새 주소 재접속 / 확정'), ('restore', '이전 Wi-Fi로 복귀'),
                              ('peers', '확인된 주소를 GUI에 적용')):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, key=action: self.start_action(key))
            self.actions[action] = button
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.message = QtWidgets.QLabel('SSID와 비밀번호를 입력한 뒤 연결 전환을 누르세요. 전환과 재접속 확인은 각 보드별로 표시됩니다.')
        self.message.setWordWrap(True)
        self.message.setMinimumHeight(56)
        layout.addWidget(self.message)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(150)
        layout.addWidget(self.log, 1)
        privacy = QtWidgets.QLabel('PC에는 비밀번호를 저장하지 않습니다. 보드에는 NetworkManager의 root 전용 연결 프로필로 저장합니다.')
        privacy.setWordWrap(True)
        layout.addWidget(privacy)
        self.close_button = QtWidgets.QPushButton('닫기')
        self.close_button.clicked.connect(self.accept)
        layout.addWidget(self.close_button)
        self.find_deadline = QtCore.QTimer(self)
        self.find_deadline.setSingleShot(True)
        self.find_deadline.timeout.connect(self.cancel_discovery)

    def busy(self, value):
        for widget in [*self.actions.values(), self.find_button, self.subnet, self.ssid, self.wifi_password,
                       self.ssh_password, self.timeout, self.reveal, *self.hosts.values(),
                       *self.selected.values(), *self.passwords.values()]:
            widget.setEnabled(not value)
        self.close_button.setEnabled(not value)

    def on_progress(self, board, value):
        if value.get('machine_id') and value.get('token'):
            self.identities[board] = {k: value[k] for k in ('machine_id', 'token')}
        self.latest[board] = value
        if value.get('ok'):
            text = STATE_TEXT.get(value.get('status'), '미확인')
            text += '\n' + value.get('active_ssid', '') + ' / ' + ', '.join(value.get('ipv4', []))
            if 'seconds_until_restore' in value:
                text += ' / 복귀까지 약 {}초'.format(value['seconds_until_restore'])
        else:
            reason = value.get('reason', 'remote_network_error')
            text = REASON_TEXT.get(reason, reason)
        self.statuses[board].setText(text)
        self.log.appendPlainText(LABELS[board] + ': ' + text.replace('\n', ' / '))

    def start_action(self, action):
        if self.worker is not None or self.finder is not None:
            return
        targets = {}
        try:
            for board, checkbox in self.selected.items():
                if checkbox.isChecked():
                    targets[board] = control.validate_host(self.hosts[board].text())
            if not targets:
                raise ValueError('대상 보드를 선택하세요.')
            if len({v.split('@')[1].lower() for v in targets.values()}) != len(targets):
                raise ValueError('서로 다른 보드 주소를 입력하세요.')
            passwords = {b: self.passwords[b].text() or self.ssh_password.text() for b in targets}
            if not all(passwords.values()):
                raise ValueError('보드 SSH 비밀번호를 입력하세요.')
            options = {}
            if action == 'connect':
                control.validate_credentials(self.ssid.text(), self.wifi_password.text())
                options = {'ssid': self.ssid.text(), 'password': self.wifi_password.text(), 'timeout': self.timeout.value()}
            if action == 'peers':
                if set(targets) != set(LABELS):
                    raise ValueError('주소 적용에는 세 보드를 모두 선택하고 현재 상태를 확인하세요.')
                peers = {}
                for b, host in targets.items():
                    state = self.latest.get(b, {})
                    address = host.split('@')[1]
                    if not state.get('ok') or address not in state.get('ipv4', []):
                        raise ValueError('현재 상태 확인 후 각 보드의 실제 IPv4 주소를 SSH 주소 칸에 입력하세요.')
                    peers[b] = address
                options = {'peers': peers}
        except ValueError as error:
            self.message.setText(str(error))
            return
        self.busy(True)
        self.message.setText('보드 응답을 확인하고 있습니다. 연결 전환 중 SSH가 끊겨도 성공으로 처리하지 않습니다.')
        self.worker = NetworkWorker(action, targets, passwords, options, dict(self.identities), self)
        self.worker.progress.connect(self.on_progress)
        self.worker.result.connect(self.on_result)
        self.worker.finished.connect(self.worker_finished)
        self.worker.start()

    def on_result(self, result):
        phase = result.get('phase')
        if phase == 'connect' or phase in ('prepare', 'switch'):
            self.wifi_password.clear()
        if phase == 'switch':
            self.message.setText('전환을 예약했습니다. 이 PC를 같은 핫스팟에 연결하고 보드 찾기 또는 새 IP 입력 후 재접속 / 확정을 누르세요.'
                                 if result.get('ok') else
                                 '일부 전환 응답을 확인하지 못했습니다. 보드별 상태를 확인하세요. 예약된 보드는 미확정 시 이전 Wi-Fi 복귀를 시도합니다.')
        elif phase == 'prepare' and not result.get('ok'):
            self.message.setText('준비 실패. 어느 보드에도 전환을 예약하지 않았습니다. 보드별 오류를 확인하세요.')
        elif phase == 'peers' and result.get('ok'):
            hosts = {b: self.hosts[b].text().strip() for b in LABELS}
            self.hosts_applied.emit(hosts)
            self.message.setText('세 보드의 현재 주소를 GUI와 다음 지도 시작의 플래닝 연결 설정에 적용했습니다.')
        elif result.get('ok'):
            self.message.setText('선택 보드 응답을 확인했습니다. 위의 보드별 상태를 확인하세요.')
        else:
            self.message.setText('일부 보드 확인에 실패했습니다. 위의 보드별 상태를 확인하세요.')

    def worker_finished(self):
        self.worker.deleteLater()
        self.worker = None
        self.busy(False)

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
        self.message.setText('보드 {}개를 찾았습니다. 새 주소 재접속 / 확정으로 연결을 확인하세요.'.format(found))

    def done(self, code):
        if self.worker is not None or self.finder is not None:
            return
        self.ssh_password.clear()
        self.wifi_password.clear()
        for field in self.passwords.values():
            field.clear()
        super().done(code)

    def closeEvent(self, event):
        if self.worker is not None or self.finder is not None:
            event.ignore()
        else:
            self.done(QtWidgets.QDialog.Rejected)
            event.accept()
