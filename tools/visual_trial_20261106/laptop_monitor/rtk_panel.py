"""RTK controls on the operating laptop; recordings/spectators are read-only."""
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
from PyQt5 import QtCore, QtWidgets
from control import validate_host

DEFAULTS = dict(host='gnss.eseoul.go.kr',port=2101,mountpoint='VRS-RTCM32',username='seoul',password='seoul',tls=False)
LABELS = {
    'OFF':'꺼짐', 'DISABLED':'보정 공급 중지 · 일반 GNSS/VIO 유지',
    'CHECKING':'서버·계정 확인 중', 'CHECK_OK':'접속·로그인 확인 완료',
    'CHECK_FAILED':'접속 확인 실패', 'CONNECTING':'연결 중', 'CONNECTED':'연결됨 · 보정 데이터 대기',
    'AUTH_OK':'접속·인증 성공', 'RECEIVING':'보정 데이터 수신 중',
    'WAITING_FIX':'유효한 GNSS 위치 대기', 'WAITING_VALID_GGA':'서버에 보낼 최신 GNSS 위치 대기',
    'RETRYING':'재연결 대기', 'AUTH_FAILED':'ID·비밀번호 확인 필요',
    'MOUNTPOINT_REJECTED':'보정 채널 확인 필요', 'NETWORK_ERROR':'서버 연결 실패 · 인터넷/주소 확인',
    'NO_CORRECTIONS':'보정 데이터가 오지 않음', 'CONNECTION_CLOSED':'서버 연결 끊김',
    'RESPONSE_TIMEOUT':'서버 응답 시간 초과', 'GNSS_NOT_RUNNING':'GNSS 실행 후 다시 확인하세요',
    'BUSY_STOP_FIRST':'기존 연결을 중지하고 잠시 후 다시 시도하세요',
    'RECEIVER_WRITE_FAILED':'수신기 보정 전송 실패 · 보정 중지됨',
    'SSH_DISCONNECTED':'Pi 5 상태 연결 끊김 · 현재 보정 상태 미확인',
    'SSH_TIMEOUT':'Pi 5 응답 지연 · 현재 보정 상태 미확인',
    'NO_FIX':'위치 미확정', 'SINGLE':'일반 GNSS', 'DGNSS':'DGNSS',
    'RTK_FLOAT':'RTK FLOAT', 'RTK_FIXED':'RTK FIXED', 'UNKNOWN':'미확인',
}
SOLUTION={1:'NO_FIX',2:'SINGLE',3:'DGNSS',4:'RTK_FLOAT',5:'RTK_FIXED'}
PUBLIC_FIELDS=('session','enabled','state','reason','receiver','receiver_age_s','receiving',
               'rx_age_s','tx_age_s','used_age_s','rx_bytes','tx_bytes','crc_errors',
               'dropped_frames','events','mountpoints','check_ok','working')


def public_status(value):
    if not isinstance(value,dict):raise ValueError('Invalid RTK status')
    result={k:value[k] for k in PUBLIC_FIELDS if k in value}
    for key in ('enabled','receiving','check_ok','working'):
        if key in result and type(result[key]) is not bool:raise ValueError('Invalid RTK flag')
    for key in ('receiver_age_s','rx_age_s','tx_age_s','used_age_s'):
        v=result.get(key)
        if v is not None and (type(v) not in (int,float) or not math.isfinite(v) or v<0):
            raise ValueError('Invalid RTK age')
    for key in ('rx_bytes','tx_bytes','crc_errors','dropped_frames'):
        if key in result and (type(result[key]) is not int or result[key]<0):raise ValueError('Invalid RTK count')
    for key in ('session','state','reason'):
        if key in result and (not isinstance(result[key],str) or not re.fullmatch(r'[A-Za-z0-9_-]{0,64}',result[key])):
            raise ValueError('Invalid RTK state')
    receiver=result.get('receiver',{})
    if not isinstance(receiver,dict):raise ValueError('Invalid receiver state')
    result['receiver']={k:receiver[k] for k in ('solution','satellites','hacc') if k in receiver}
    for key in ('solution','satellites'):
        if key in receiver and (type(receiver[key]) is not int or not 0<=receiver[key]<=255):
            raise ValueError('Invalid receiver count')
    hacc=receiver.get('hacc')
    if hacc is not None and (type(hacc) not in (int,float) or not math.isfinite(hacc) or hacc<0):
        raise ValueError('Invalid receiver accuracy')
    events=result.get('events',[])
    if not isinstance(events,list) or len(events)>80:raise ValueError('Invalid RTK events')
    clean=[]
    for event in events:
        if (not isinstance(event,dict) or type(event.get('id')) is not int or
                type(event.get('t')) not in (int,float) or not 0<=event['t']<=253402300799 or
                not isinstance(event.get('code'),str) or not re.fullmatch(r'[A-Z0-9_]{1,64}',event['code'])):
            raise ValueError('Invalid RTK event')
        clean.append({k:event[k] for k in ('id','t','code')})
    result['events']=clean
    names=result.get('mountpoints',[])
    if not isinstance(names,list) or len(names)>100 or any(
            not isinstance(n,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}',n) for n in names):
        raise ValueError('Invalid RTK mountpoints')
    if len(json.dumps(result,allow_nan=False))>32768:raise ValueError('RTK status too large')
    return result


def summary(status,age=None):
    if not status or age is None or age>3:
        return 'RTK: 상태 미확인' if age is None else 'RTK: 상태 오래됨 · 연결 확인 필요'
    receiver=status.get('receiver',{})
    receiver_age=status.get('receiver_age_s')
    solution=SOLUTION.get(receiver.get('solution'),'UNKNOWN')
    if receiver_age is None or receiver_age+age>3:solution='UNKNOWN'
    state=LABELS.get(status.get('state'),'미확인')
    if status.get('state')=='RECEIVING' and (status.get('rx_age_s') is None or status['rx_age_s']+age>=3):
        state='보정 데이터 지연'
    return f'RTK: {state}  |  측위: {LABELS[solution]}'


class RtkClient(QtCore.QObject):
    reply=QtCore.pyqtSignal(dict)
    def __init__(self,host,parent=None):
        super().__init__(parent)
        self.host=host
        self.process=QtCore.QProcess(self)
        self.process.readyReadStandardOutput.connect(self.read)
        self.process.readyReadStandardError.connect(lambda:self.process.readAllStandardError())
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(lambda *_:self.reply.emit(dict(ok=False,reason='SSH_DISCONNECTED')))
        self.buffer=bytearray();self.pending=False;self.sent=0.;self.temporary=None;self.queued=None
        self.timer=QtCore.QTimer(self);self.timer.setInterval(1000);self.timer.timeout.connect(self.poll)

    def connect_board(self):
        if self.process.state()!=QtCore.QProcess.NotRunning:return
        host=validate_host(self.host)
        if host.split('@')[0]!='user':raise ValueError('Pi5 주소를 확인해 주세요.')
        self.buffer.clear();self.pending=False
        if self.temporary:self.temporary.cleanup()
        self.temporary=tempfile.TemporaryDirectory(prefix='nomad-rtk-ssh-')
        env=QtCore.QProcessEnvironment.systemEnvironment()
        password=os.environ.get('NOMAD_BOARD_PASSWORD1','')
        if password:
            askpass=Path(self.temporary.name)/'askpass'
            askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_BOARD_PASSWORD1"\n')
            askpass.chmod(0o700)
            env.insert('SSH_ASKPASS',str(askpass));env.insert('SSH_ASKPASS_REQUIRE','force')
            if not env.contains('DISPLAY'):env.insert('DISPLAY',':0')
        known=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))/'nomad-trial'
        known.mkdir(parents=True,exist_ok=True,mode=0o700)
        args=['-T','-o','ConnectTimeout=5','-o','ServerAliveInterval=3','-o','ServerAliveCountMax=1',
              '-o','StrictHostKeyChecking=accept-new','-o',f'UserKnownHostsFile="{known / "known_hosts"}"',
              '-o','LogLevel=ERROR','-o','NumberOfPasswordPrompts=1','-o','BatchMode='+('no' if password else 'yes')]
        key=os.environ.get('NOMAD_SSH_KEY')
        if key:args+=['-i',key]
        # Fixed command. NTRIP credentials are sent ONLY as JSON on stdin.
        import board_address  # follow Pi5 if its address changed (recovered route)
        target=board_address.route(host)
        args += [*board_address.ssh_route_options(target, args[1:]), target['host'],
                 'PYTHONPATH="$HOME/nomad_ws/src/nomad_gnss" exec python3 -m nomad_gnss.rtk_control']
        self.process.setProcessEnvironment(env)
        self.process.start('ssh',args)
        self.timer.start()

    def send(self,request):
        if self.pending or self.process.state()!=QtCore.QProcess.Running:return False
        self.process.write((json.dumps(request,allow_nan=False)+'\n').encode())
        self.pending=True;self.sent=time.monotonic()
        return True

    def command(self, request):
        if self.process.state()!=QtCore.QProcess.Running:return False
        if self.pending:
            self.queued=request
            return True
        return self.send(request)

    def poll(self):
        if self.pending:
            if time.monotonic()-self.sent>6:
                self.reply.emit(dict(ok=False,reason='SSH_TIMEOUT'));self.process.kill()
            return
        self.send(dict(action='status'))

    def read(self):
        self.buffer.extend(bytes(self.process.readAllStandardOutput()))
        if len(self.buffer)>131072:
            self.process.kill();return
        while b'\n' in self.buffer:
            line,_,rest=self.buffer.partition(b'\n');self.buffer=bytearray(rest)
            try:
                result=json.loads(line)
                if not isinstance(result,dict):continue
                if 'status' in result:result['status']=public_status(result['status'])
            except (ValueError,TypeError):continue
            self.pending=False
            self.reply.emit(result)
            if self.queued is not None:
                value=self.queued;self.queued=None;self.send(value)

    def finished(self,*_):
        self.pending=False;self.queued=None;self.timer.stop()
        self.reply.emit(dict(ok=False,reason='SSH_DISCONNECTED'))

    def close(self):
        self.timer.stop()
        self.process.closeWriteChannel()
        if not self.process.waitForFinished(700):
            self.process.kill();self.process.waitForFinished(700)
        if self.temporary:self.temporary.cleanup();self.temporary=None


class RtkPanel(QtCore.QObject):
    telemetry=QtCore.pyqtSignal(dict)
    def __init__(self,host,readonly=False,parent=None):
        super().__init__(parent)
        self.readonly=readonly;self.status={};self.age=None;self.seen=set();self.client=None
        self.host=host
        self.bar=QtWidgets.QWidget(parent);row=QtWidgets.QHBoxLayout(self.bar);row.setContentsMargins(0,0,0,0)
        self.button=QtWidgets.QPushButton('RTK 설정 / 상태');self.button.clicked.connect(self.open)
        self.label=QtWidgets.QLabel('RTK: 상태 미확인 · 설정 창에서 확인');self.label.setWordWrap(True)
        row.addWidget(self.button);row.addWidget(self.label,1)
        self.dialog=QtWidgets.QDialog(parent);self.dialog.setWindowTitle('RTK 보정 · Pi 5');self.dialog.resize(650,630)
        layout=QtWidgets.QVBoxLayout(self.dialog)
        note=QtWidgets.QLabel('서울시 공개 서비스 · 별도 회원가입 불필요\n보정이 끊겨도 기존 위치추정이 유효하면 주행을 유지합니다.')
        note.setWordWrap(True);layout.addWidget(note)
        form=QtWidgets.QFormLayout();self.fields={}
        for key,label in [('host','서버'),('port','포트'),('mountpoint','보정 채널'),('username','NTRIP ID'),('password','NTRIP 비밀번호')]:
            if key=='mountpoint':
                widget=QtWidgets.QComboBox();widget.setEditable(True);widget.addItem(DEFAULTS[key])
            else:widget=QtWidgets.QLineEdit(str(DEFAULTS[key]))
            if key=='password':widget.setEchoMode(QtWidgets.QLineEdit.Password)
            widget.setEnabled(not readonly);self.fields[key]=widget;form.addRow(label,widget)
        self.tls=QtWidgets.QCheckBox('TLS 연결');self.tls.setEnabled(not readonly);form.addRow(self.tls)
        layout.addLayout(form)
        actions=QtWidgets.QHBoxLayout()
        self.connect=QtWidgets.QPushButton('상태 연결');self.connect.clicked.connect(self.connect_board)
        self.check=QtWidgets.QPushButton('접속 / 로그인 확인');self.check.clicked.connect(lambda:self.command('check'))
        self.start=QtWidgets.QPushButton('보정 시작');self.start.clicked.connect(lambda:self.command('start'))
        self.stop=QtWidgets.QPushButton('보정 중지');self.stop.clicked.connect(lambda:self.command('stop'))
        for b in (self.connect,self.check,self.start,self.stop):actions.addWidget(b);b.setEnabled(not readonly)
        layout.addLayout(actions)
        self.detail=QtWidgets.QLabel('상태 연결 후 접속 정보를 확인하세요.');self.detail.setWordWrap(True);layout.addWidget(self.detail)
        self.log=QtWidgets.QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(200);layout.addWidget(self.log,1)
        helptext=QtWidgets.QLabel('서울시: seoul / seoul → 접속/로그인 확인 → 보정 시작\n'
            'Pi 5 인터넷과 유효한 GNSS 위치가 필요합니다. 서버 연결만으로 RTK FIX를 보장하지 않습니다.\n'
            '중지 시 보정 공급만 끕니다. 표시되는 측위 상태는 수신기 실측값입니다.\n'
            '설정·비밀번호는 이 창의 메모리에만 보관합니다. 창을 닫아도 Pi 5 보정은 유지됩니다.'
            if not readonly else '관전·기록 재생은 읽기 전용입니다. 보드에 접속하거나 보정을 제어하지 않습니다.')
        helptext.setWordWrap(True);layout.addWidget(helptext)

    def open(self):
        self.dialog.show();self.dialog.raise_()
        if not self.readonly and self.client is None:self.connect_board()

    def connect_board(self):
        if self.readonly:return
        if self.client is None:
            self.client=RtkClient(self.host,self);self.client.reply.connect(self.reply)
        try:self.client.connect_board()
        except ValueError:self.detail.setText('Pi 5 주소를 확인해 주세요.')

    def config(self):
        cfg={k:(v.currentText() if k=='mountpoint' else v.text()) for k,v in self.fields.items()}
        cfg['port']=int(cfg['port']);cfg['tls']=self.tls.isChecked()
        return cfg

    def command(self,action):
        if self.readonly:return
        try:
            value=dict(action=action)
            if action!='stop':value['config']=self.config()
        except ValueError:
            self.detail.setText('포트는 1~65535 숫자로 입력하세요.');return
        if self.client is None or not self.client.command(value):
            self.detail.setText('상태 연결을 확인하거나 현재 요청 완료 후 다시 눌러주세요.')

    def reply(self,result):
        if result.get('ok') and 'status' in result:
            self.telemetry.emit(dict(kind='telemetry',source='rtk',t=time.time(),rtk=result['status']))
        else:
            reason=result.get('reason','UNKNOWN')
            self.detail.setText(LABELS.get(reason,reason))
            if reason in ('GNSS_NOT_RUNNING','SSH_TIMEOUT','SSH_DISCONNECTED'):
                self.telemetry.emit(dict(kind='telemetry',source='rtk',t=time.time(),rtk=dict(state='UNKNOWN',reason=reason)))

    def update_status(self,status,age):
        self.status=public_status(status or {});self.age=age
        self.label.setText(summary(self.status,age))
        receiver=self.status.get('receiver',{})
        def seconds(key):
            value=self.status.get(key)
            return '미확인' if value is None or age is None else f'{value+age:.1f}초 전'
        self.detail.setText(self.label.text()+'\n'+LABELS.get(self.status.get('reason'),self.status.get('reason',''))+
            f'\n보정 수신 {self.status.get("rx_bytes",0)} B · 수신기에 전송 {self.status.get("tx_bytes",0)} B'
            f'\n마지막 보정 수신: {seconds("rx_age_s")} · 수신기 사용 보고: {seconds("used_age_s")}'
            f'\n사용 위성: {receiver.get("satellites","미확인")} · 수신기 추정 수평 정확도: '
            +(f'{receiver["hacc"]:.3f} m' if receiver.get('hacc') is not None else '미확인'))
        if not self.readonly:
            busy=self.status.get('enabled') or self.status.get('working')
            known=age is not None and age<=3 and self.status.get('state') not in (None,'UNKNOWN')
            self.check.setEnabled(known and not busy);self.start.setEnabled(known and not busy)
            self.stop.setEnabled(known and bool(busy))
        names=self.status.get('mountpoints',[])
        if names:
            combo=self.fields['mountpoint'];current=combo.currentText()
            if [combo.itemText(i) for i in range(combo.count())]!=names:
                combo.clear();combo.addItems(names);combo.setCurrentText(current)
        # The snapshot contains bounded events. Rebuild to support backward replay seeks.
        lines=[]
        for event in self.status.get('events',[]):
            stamp=time.strftime('%H:%M:%S',time.localtime(event.get('t',0)))
            lines.append(f'{stamp}  {LABELS.get(event.get("code"),event.get("code","UNKNOWN"))}')
        text='\n'.join(lines)
        if text!=self.log.toPlainText():self.log.setPlainText(text);self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def close(self):
        if self.client:self.client.close()
        self.dialog.close()
