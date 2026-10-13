"""Save only console, selected result files and reproducible configuration."""
from datetime import datetime, timezone
from pathlib import Path
import os
import tempfile
import html
from collections import deque
from log_filter import entry, matches
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout,QPushButton,QInputDialog,QMessageBox,QLabel,QComboBox,QLineEdit
from log_data import save_session, summary_plots, discover_outputs, recover_console_metrics


class LogActions:
    def init_logs(self, root, layout):
        self.log_root=Path(os.environ.get('NOMAD_LOG_DIR',str(root/'log/gui_sessions')))
        self.log_spool=tempfile.TemporaryDirectory(prefix='nomad-gui-console-')
        self.console_file=Path(self.log_spool.name)/'console.log';self.console_file.touch()
        self.run_history=[];self.log_attachments=[];self.tail_positions={};self.tail_buffers={}
        self.tail_timer=QTimer(self);self.tail_timer.setInterval(300)
        self.tail_timer.timeout.connect(self.tail_estimator_logs);self.tail_timer.start()
        row=QHBoxLayout()
        for name,callback in [('로그 저장',self.save_logs),('로그 보기',self.view_logs)]:
            button=QPushButton(name);button.clicked.connect(callback);row.addWidget(button)
        layout.addLayout(row)
        self.log_hint=QLabel('저장 대상: 전체 실행 로그·설정·자동 생성된 수치 기록');self.log_hint.setWordWrap(True);layout.addWidget(self.log_hint)

    def init_live_filters(self, layout):
        self.live_entries=deque(maxlen=5000)
        row=QHBoxLayout()
        self.node_filter=QComboBox();self.node_filter.addItem('모든 노드','')
        self.level_filter=QComboBox()
        for label,key in [('모든 레벨',''),('INFO','INFO'),('WARN 이상','WARN+'),('ERROR 이상','ERROR+'),('DEBUG','DEBUG'),('기타','OTHER')]:
            self.level_filter.addItem(label,key)
        self.log_query=QLineEdit();self.log_query.setPlaceholderText('로그 내용 검색')
        for widget in (self.node_filter,self.level_filter,self.log_query):row.addWidget(widget)
        layout.addLayout(row)
        self.node_filter.currentIndexChanged.connect(self.render_live_log)
        self.level_filter.currentIndexChanged.connect(self.render_live_log)
        self.log_query.textChanged.connect(self.render_live_log)

    def log_matches(self,item):
        return matches(item,self.node_filter.currentData(),self.level_filter.currentData(),self.log_query.text())

    def render_live_log(self,*_):
        if not hasattr(self,'log'):return
        visible=[item['text'] for item in self.live_entries if self.log_matches(item)]
        self.log.setPlainText('\n'.join(visible[-1500:]))
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def append_log(self, text, source='GUI'):
        lines=text.splitlines()
        for line in lines:
            item=entry(line,source)
            if hasattr(self,'live_entries'):
                self.live_entries.append(item)
                if self.node_filter.findData(item['node'])<0:
                    self.node_filter.blockSignals(True);self.node_filter.addItem(item['node'],item['node']);self.node_filter.blockSignals(False)
                if self.log_matches(item):self.log.append(html.escape(item['text']))
            else:self.log.append(html.escape(line))
        if hasattr(self,'console_file'):
            with self.console_file.open('a') as stream:
                for line in lines:stream.write(datetime.now(timezone.utc).isoformat()+' ['+source+'] '+line+'\n')

    def tail_estimator_logs(self):
        for run in self.run_history:
            if not run.get('output_directory'):continue
            root=Path(run['output_directory'])
            for file in root.rglob('estimator_epoch_*.log'):
                try:
                    offset=self.tail_positions.get(file,0)
                    if file.stat().st_size<offset:offset=0
                    with file.open('rb') as stream:
                        stream.seek(offset);data=stream.read(262144);self.tail_positions[file]=stream.tell()
                    if not data:continue
                    buffered=self.tail_buffers.get(file,'')+data.decode(errors='replace')
                    lines=buffered.split('\n');self.tail_buffers[file]=lines.pop()
                    if lines:self.append_log('\n'.join(lines),source=run.get('title','VIO'))
                except OSError:continue

    def save_logs(self):
        name,ok=QInputDialog.getText(self,'로그 저장','기록 이름',text=datetime.now().astimezone().strftime('실험_%m%d_%H%M'))
        if not ok or not name.strip():return
        try:
            directory=save_session(self.log_root,name.strip(),self.console_file,self.run_history,self.effective_options(),list(dict.fromkeys([*self.log_attachments,*discover_outputs(self.run_history)])))
            recover_console_metrics(directory)
            plots=summary_plots(directory)
        except (OSError,ValueError,TypeError) as error:
            QMessageBox.warning(self,'로그 저장',str(error));return
        self.log_hint.setText(f'저장 완료: {directory.name} · 요약 그림 {len(plots)}개')
        self.append_log(f'[로그 저장] {directory}')

    def view_logs(self):
        from log_viewer import LogViewer
        dialog=LogViewer(self,self.log_root);dialog.exec_()
