"""Opt-in recording of selected module messages; live display is independent."""
from datetime import datetime, timezone
from pathlib import Path
import os
import tempfile
import json
from collections import deque
from log_filter import entry, matches, category, GROUPS
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import (QHBoxLayout,QVBoxLayout,QPushButton,QMessageBox,QLabel,
    QComboBox,QLineEdit,QDialog,QCheckBox,QDialogButtonBox)
from log_data import save_session


class LogActions:
    def init_logs(self, root, layout):
        self.log_root=Path(os.environ.get('NOMAD_LOG_DIR',str(root/'log/gui_sessions')))
        self.log_spool=tempfile.TemporaryDirectory(prefix='nomad-gui-console-')
        self.run_history=[];self.tail_positions={};self.tail_buffers={}
        self.record_stream=None;self.record_directory=None;self.record_groups=set()
        self.pending_display=deque(maxlen=2000)
        self.display_timer=QTimer(self);self.display_timer.setInterval(100)
        self.display_timer.timeout.connect(self.flush_live_log);self.display_timer.start()
        self.tail_timer=QTimer(self);self.tail_timer.setInterval(1000)
        self.tail_timer.timeout.connect(self.tail_estimator_logs);self.tail_timer.start()
        row=QHBoxLayout()
        self.record_button=QPushButton('기록 시작');self.record_button.clicked.connect(self.toggle_recording)
        row.addWidget(self.record_button)
        button=QPushButton('로그 보기');button.clicked.connect(self.view_logs);row.addWidget(button)
        layout.addLayout(row)
        self.log_hint=QLabel('기록 꺼짐 · 시작을 누르면 기록할 파트를 선택합니다.')
        self.log_hint.setWordWrap(True);layout.addWidget(self.log_hint)

    def init_live_filters(self, layout):
        self.live_entries=deque(maxlen=3000)
        row=QHBoxLayout()
        self.group_filter=QComboBox();self.group_filter.addItem('전체 파트','')
        for group in GROUPS:self.group_filter.addItem(group,group)
        self.level_filter=QComboBox()
        for label,key in [('모든 레벨',''),('INFO','INFO'),('WARN 이상','WARN+'),('ERROR 이상','ERROR+'),('DEBUG','DEBUG'),('기타','OTHER')]:
            self.level_filter.addItem(label,key)
        self.log_query=QLineEdit();self.log_query.setPlaceholderText('로그 내용 검색')
        for widget in (self.group_filter,self.level_filter,self.log_query):row.addWidget(widget)
        layout.addLayout(row)
        self.group_filter.currentIndexChanged.connect(self.render_live_log)
        self.level_filter.currentIndexChanged.connect(self.render_live_log)
        self.log_query.textChanged.connect(self.render_live_log)

    def log_matches(self,item):
        return matches(item,level=self.level_filter.currentData(),query=self.log_query.text(),group=self.group_filter.currentData())

    def render_live_log(self,*_):
        if not hasattr(self,'log'):return
        self.pending_display.clear()
        visible=[item['text'] for item in self.live_entries if self.log_matches(item)]
        self.log.setPlainText('\n'.join(visible[-1500:]))

    def flush_live_log(self):
        if not self.pending_display:return
        lines=list(self.pending_display);self.pending_display.clear()
        # Keep Qt work bounded under warning floods; recording below keeps every selected line.
        cursor=self.log.textCursor();cursor.movePosition(QTextCursor.End)
        cursor.insertText(('\n' if not self.log.document().isEmpty() else '')+'\n'.join(lines[-200:]))
        self.log.setTextCursor(cursor)

    def append_log(self, text, source='GUI'):
        recorded=[];timestamp=None
        for line in text.splitlines():
            item=entry(line,source)
            self.live_entries.append(item)
            if self.log_matches(item):self.pending_display.append(item['text'])
            if self.record_stream and category(item) in self.record_groups:
                timestamp=timestamp or datetime.now(timezone.utc).isoformat()
                recorded.append(timestamp+' ['+category(item)+'] ['+source+'] '+line+'\n')
        if recorded:
            try:self.record_stream.write(''.join(recorded))
            except OSError as error:
                self.finish_recording();self.log_hint.setText(f'기록 오류: {error}')

    def tail_estimator_logs(self):
        wants_vio=self.group_filter.currentData() in ('','위치추정') or '위치추정' in self.record_groups
        if not wants_vio:return
        # Inspect active runs only, not all past sessions recursively.
        for active in self.runs.values():
            run=active['record'];root=Path(run['output_directory'])/'vio'
            for file in root.glob('estimator_epoch_*.log'):
                try:
                    offset=self.tail_positions.get(file,0)
                    if file.stat().st_size<offset:offset=0
                    with file.open('rb') as stream:
                        stream.seek(offset);data=stream.read(65536);self.tail_positions[file]=stream.tell()
                    if not data:continue
                    buffered=self.tail_buffers.get(file,'')+data.decode(errors='replace')
                    lines=buffered.split('\n');self.tail_buffers[file]=lines.pop()
                    if lines:self.append_log('\n'.join(lines),source='VIO')
                except OSError:continue

    def start_recording(self,name,groups):
        if self.record_stream or not groups:return
        blank=Path(self.log_spool.name)/'blank.log';blank.write_text('')
        directory=save_session(self.log_root,name,blank,self.run_history,self.effective_options(),[])
        manifest_path=directory/'manifest.json';manifest=json.loads(manifest_path.read_text())
        manifest.update(collection='selected module console messages only',record_groups=sorted(groups),
                        recording_started_at=datetime.now(timezone.utc).isoformat(),console_metrics_processed=True)
        manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
        # Do not include estimator output emitted before recording starts.
        for active in self.runs.values():
            root=Path(active['record']['output_directory'])/'vio'
            for file in root.glob('estimator_epoch_*.log'):
                self.tail_positions[file]=file.stat().st_size;self.tail_buffers[file]=''
        self.record_directory=directory;self.record_groups=set(groups)
        self.record_stream=(directory/'console.log').open('a',buffering=65536)
        self.record_button.setText('기록 종료')
        self.log_hint.setText('기록 중: '+', '.join(sorted(groups)))

    def finish_recording(self):
        if not self.record_stream:return
        stream=self.record_stream;self.record_stream=None;error=None
        try:
            stream.flush()
        except OSError as failure:error=failure
        finally:
            try:stream.close()
            except OSError as failure:error=error or failure
        try:
            path=self.record_directory/'manifest.json';manifest=json.loads(path.read_text())
            manifest['recording_stopped_at']=datetime.now(timezone.utc).isoformat()
            manifest['runs']=self.run_history
            path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
        except (OSError,ValueError) as failure:error=error or failure
        self.record_groups.clear();self.record_button.setText('기록 시작')
        self.log_hint.setText(f'기록 오류: {error}' if error else f'기록 완료: {self.record_directory.name}')

    def toggle_recording(self):
        if self.record_stream:
            self.finish_recording();return
        dialog=QDialog(self);dialog.setWindowTitle('기록할 로그 선택');layout=QVBoxLayout(dialog)
        name=QLineEdit(datetime.now().astimezone().strftime('실험_%m%d_%H%M'));layout.addWidget(name)
        checks=[]
        for group in GROUPS:
            check=QCheckBox(group);layout.addWidget(check);checks.append(check)
        layout.addWidget(QLabel('선택한 파트의 실행 메시지만 기록합니다. 수치·지도·그림은 자동 저장하지 않습니다.'))
        buttons=QDialogButtonBox(QDialogButtonBox.Ok|QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText('기록 시작')
        buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
        if dialog.exec_()!=QDialog.Accepted:return
        groups={check.text() for check in checks if check.isChecked()}
        if not groups:
            QMessageBox.information(self,'로그 기록','기록할 파트를 하나 이상 선택하세요.');return
        try:self.start_recording(name.text().strip() or '실행',groups)
        except (OSError,ValueError) as error:QMessageBox.warning(self,'로그 기록',str(error))

    def view_logs(self):
        if self.record_stream:
            try:self.record_stream.flush()
            except OSError:self.finish_recording()
        from log_viewer import LogViewer
        dialog=LogViewer(self,self.log_root);dialog.exec_()
