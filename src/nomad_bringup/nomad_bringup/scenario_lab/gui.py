"""Queue, live Gazebo access and per-scenario recorded results."""
import argparse
import csv
import fcntl
import json
import os
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import (QApplication,QMainWindow,QWidget,QTabWidget,QVBoxLayout,QHBoxLayout,QLabel,
    QPushButton,QComboBox,QSpinBox,QDoubleSpinBox,QTableWidget,QTableWidgetItem,QHeaderView,QMessageBox,QSplitter)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib import font_manager,rcParams
from .core import workspace,catalog,scenario,read,save,ALGORITHM,terrain_profile

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if Path(FONT).exists():font_manager.fontManager.addfont(FONT);rcParams['font.family']=font_manager.FontProperties(fname=FONT).get_name()
rcParams['axes.unicode_minus']=False


class Lab(QMainWindow):
    def __init__(self,root,algorithms=None,title='플래닝 시나리오 실험실'):
        super().__init__();self.algorithms=algorithms or {ALGORITHM:'Hybrid A* + RPP'};self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.lock=(self.root/'gui.lock').open('a');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.jobs=read(self.root/'queue.json',[]);self.process=None;self.client=None;self.current=None;self.running=False;self.closing=False;self.log=None;self.stopping=False
        # Never silently rerun a partly completed experiment after a GUI crash.
        for job in self.jobs:
            if job['state']=='running':job['state']='interrupted';job['reason']='GUI reopened; inspect saved result/ownership before retry'
        self.setWindowTitle('NOMAD · '+title);self.resize(1260,850)
        body=QWidget();layout=QVBoxLayout(body);self.setCentralWidget(body)
        title=QLabel(title);title.setStyleSheet('font-size:24px;font-weight:700;color:#264958');layout.addWidget(title)
        label=QLabel(' / '.join(self.algorithms.values())+' · PC CPU · Gazebo 실제 물리 / 정답 위치·관측 지도\n기록은 큐에서 실행한 시험에만 저장합니다. p05는 차량 외곽과 장애물의 XY 여유 거리입니다.');label.setWordWrap(True);layout.addWidget(label)
        tabs=QTabWidget();self.tabs=tabs;layout.addWidget(tabs)
        queue=QWidget();ql=QVBoxLayout(queue);tabs.addTab(queue,'실행 큐')
        row=QHBoxLayout();self.algorithm=QComboBox();row.addWidget(self.algorithm)
        for key,label in self.algorithms.items():self.algorithm.addItem(label,key)
        self.select=QComboBox()
        for s in catalog()['scenarios']:self.select.addItem(f"{s['id']}. {s['name']}",s['id'])
        row.addWidget(self.select,1);self.repeat=QSpinBox();self.repeat.setRange(1,30);self.repeat.setSuffix(' 회');row.addWidget(self.repeat)
        self.timeout=QSpinBox();self.timeout.setRange(30,1800);self.timeout.setValue(180);self.timeout.setSuffix(' sim초 제한');row.addWidget(self.timeout)
        self.button(row,'큐에 추가',self.add);self.button(row,'6개 모두 추가',self.add_all);ql.addLayout(row)
        row=QHBoxLayout();row.addWidget(QLabel('3번 지형 최대 경사각'))
        profile=terrain_profile()
        self.slope=QDoubleSpinBox();self.slope.setRange(0.,profile['max_selectable_slope_deg']);self.slope.setDecimals(1);self.slope.setSingleStep(.5);self.slope.setValue(profile['default_max_slope_deg']);self.slope.setSuffix(' °');row.addWidget(self.slope)
        row.addWidget(QLabel(f"지형·GPP 비용 동시 반영 · 차단 기준 {profile['slope_limit_deg']:g}° · 경사 가중치 {profile['slope_weight']:g}"));row.addStretch();ql.addLayout(row)
        self.select.currentIndexChanged.connect(lambda:self.slope.setEnabled(self.select.currentData()==3));self.slope.setEnabled(self.select.currentData()==3)
        row=QHBoxLayout();self.button(row,'큐 시작',self.start);self.button(row,'현재 시험 중지 / 큐 일시정지',self.stop);self.button(row,'선택한 대기 항목 삭제',self.remove);self.button(row,'실행 중 Gazebo 보기',self.show_gazebo);ql.addLayout(row)
        self.queue=QTableWidget(0,6);self.queue.setHorizontalHeaderLabels(['번호','시나리오','알고리즘','상태','실행 폴더','최대 경사']);self.queue.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents);self.queue.horizontalHeader().setStretchLastSection(True);self.queue.setSelectionBehavior(QTableWidget.SelectRows);self.queue.setEditTriggers(QTableWidget.NoEditTriggers);ql.addWidget(self.queue)
        self.live=QLabel('대기 중 · 실행할 시나리오를 큐에 추가하세요.');self.live.setWordWrap(True);self.live.setStyleSheet('padding:15px;background:#e9f2f3;color:#294a58');ql.addWidget(self.live)
        results=QWidget();rl=QVBoxLayout(results);tabs.addTab(results,'시나리오 결과')
        row=QHBoxLayout();self.result_select=QComboBox()
        for s in catalog()['scenarios']:self.result_select.addItem(f"{s['id']}. {s['name']}",s['id'])
        self.result_select.currentIndexChanged.connect(self.refresh_results);row.addWidget(self.result_select,1);self.button(row,'결과 새로고침',self.refresh_results);self.button(row,'기록 폴더 열기',lambda:QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.root))));rl.addLayout(row)
        self.results=QTableWidget(0,12);self.results.setHorizontalHeaderLabels(['알고리즘','시작 시각','결과','목표 도착','기대 결과','여유 p05(m)','주행(sim s)','GPP CPU(core·s)','LPP CPU(core·s)','GPP 발행','무효 후 재계획','최대 경사']);self.results.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents);self.results.setMaximumHeight(230);self.results.setSelectionBehavior(QTableWidget.SelectRows);self.results.setEditTriggers(QTableWidget.NoEditTriggers);self.results.itemSelectionChanged.connect(self.draw_result);rl.addWidget(self.results)
        self.figure=Figure(figsize=(11,5),tight_layout=True);self.canvas=FigureCanvasQTAgg(self.figure);rl.addWidget(self.canvas)
        self.result_note=QLabel('완주와 기대 결과 충족을 구분합니다. 5번=벽 앞 안전 정지, 6번=전진 경로 부재 확인. 결측 지표는 —로 표시합니다.');self.result_note.setWordWrap(True);rl.addWidget(self.result_note)
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(500);self.refresh_queue();self.refresh_results()

    @staticmethod
    def button(row,label,action):
        b=QPushButton(label);b.clicked.connect(action);row.addWidget(b);return b
    @staticmethod
    def slope_label(number,value):
        return '—' if number!=3 else '기존 지형·합성비용' if value is None else f'{value:.1f}°'
    def persist(self):save(self.root/'queue.json',self.jobs)
    def append(self,number):
        for _ in range(self.repeat.value()):self.jobs.append(dict(id=datetime.now().strftime('%Y%m%d_%H%M%S_%f'),scenario=number,algorithm=self.algorithm.currentData(),state='queued',timeout=self.timeout.value(),directory='',slope_deg=self.slope.value() if number==3 else None))
    def add(self):self.append(self.select.currentData());self.refresh_queue()
    def add_all(self):
        for i in range(1,7):self.append(i)
        self.refresh_queue()
    def remove(self):
        i=self.queue.currentRow()
        if 0<=i<len(self.jobs) and self.jobs[i]['state']=='queued':self.jobs.pop(i);self.refresh_queue()
    def refresh_queue(self):
        self.persist();self.queue.setRowCount(len(self.jobs))
        for i,j in enumerate(self.jobs):
            for k,v in enumerate([i+1,scenario(j['scenario'])['name'],self.algorithms.get(j['algorithm'],j['algorithm']),j['state'],j['directory'],self.slope_label(j['scenario'],j.get('slope_deg'))]):self.queue.setItem(i,k,QTableWidgetItem(str(v)))
    def start(self):self.running=True;self.launch_next()
    def launch_next(self):
        if self.process or not self.running:return
        self.current=next((j for j in self.jobs if j['state']=='queued'),None)
        if not self.current:self.running=False;self.live.setText('큐 완료');return
        job=self.current;folder=self.root/'runs'/job['id'];folder.parent.mkdir(parents=True,exist_ok=True);job.update(state='running',directory=str(folder))
        self.stopping=False
        self.log=(self.root/(job['id']+'.runner.log')).open('w')
        command=[sys.executable,'-m','nomad_bringup.scenario_lab.runner','--scenario',str(job['scenario']),'--output',str(folder),'--timeout',str(job['timeout']),'--algorithm',job['algorithm']]
        if job.get('slope_deg') is not None:command+=['--slope-deg',str(job['slope_deg'])]
        self.process=subprocess.Popen(command,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        self.refresh_queue()
    def stop(self):
        self.running=False
        if self.current and self.process:
            self.stopping=True
            folder=Path(self.current['directory'])
            if folder.is_dir():(folder/'STOP').touch()
            self.live.setText('현재 시험 정지·소유 프로세스 정리 중…')
    def show_gazebo(self):
        if not self.current or not self.process:return
        if self.client and self.client.poll() is None:return
        folder=Path(self.current['directory']);m=read(folder/'manifest.json')
        if not m or not (folder/'viewer.config').exists():return
        env=dict(os.environ);env['GZ_PARTITION']=m['gz_partition'];env['ROS_DOMAIN_ID']=str(m['ros_domain'])
        self.viewer_log=(folder/'gazebo_gui.log').open('a')
        self.client=subprocess.Popen(['gz','sim','-g','--gui-config',str(folder/'viewer.config')],env=env,stdout=self.viewer_log,stderr=subprocess.STDOUT,start_new_session=True)
    def close_viewer(self):
        if self.client:
            from .runner import cleanup
            cleanup([self.client]);self.client=None;self.viewer_log.close()
    def poll(self):
        if not self.process:return
        if self.process.poll() is None:
            if self.stopping:
                folder=Path(self.current['directory'])
                if folder.is_dir():(folder/'STOP').touch()
                return
            d=read(Path(self.current['directory'])/'live.json',{})
            self.live.setText(f"실행 중: {self.current['scenario']}번 · {d.get('sim_s',0):.1f} sim s\n{d.get('status','Gazebo/ROS 준비 중')}\n{d.get('gpp_status','')}\nGPP 경로 발행 {d.get('gpp_paths',0)}회 · 정상 검사 {d.get('valid_checks',0)}회 · 무효 후 재계획 {d.get('invalidity_replans',0)}회")
            return
        result=read(Path(self.current['directory'])/'outcome.json',{});self.current['state']=result.get('result','worker_error');self.current['expected_result_met']=result.get('expected_result_met',False)
        self.current['exit_code']=self.process.returncode
        self.log.close();self.log=None;self.process=None;self.close_viewer()
        if not result or str(result.get('result','')).startswith('startup_error:'):
            self.running=False
            reason=result.get('result',f'worker_error: 실행 결과 없음 (종료 코드 {self.current.get("exit_code")})')
            self.current['reason']=reason
            self.live.setText(f'실행 준비 오류로 큐를 일시정지했습니다.\n{reason}\n로그: {self.root / (self.current["id"] + ".runner.log")}')
        elif not result.get('cleanup_ok',False):self.running=False;self.live.setText('정리 확인 실패: 다음 시험은 시작하지 않습니다. 실행 폴더를 확인하세요.')
        self.refresh_queue();self.refresh_results();self.current=None
        if self.closing:self.close()
        else:self.launch_next()
    def refresh_results(self):
        n=self.result_select.currentData();self.records=[]
        for path in sorted((self.root/'runs').glob('*/outcome.json'),key=lambda p:p.stat().st_mtime):
            d=read(path,{})
            if d.get('scenario')==n:self.records.append((path.parent,d))
        self.results.blockSignals(True)
        self.results.setRowCount(len(self.records))
        def fmt(x):return '—' if x is None else f'{x:.3f}' if isinstance(x,float) else str(x)
        for i,(folder,d) in enumerate(self.records):
            started=d.get('started_at')
            if started:
                try:started=datetime.fromisoformat(started).astimezone().strftime('%m-%d %H:%M:%S %Z')
                except ValueError:pass
            values=[self.algorithms.get(d.get('algorithm',ALGORITHM),d.get('algorithm',ALGORITHM)),started,d.get('result'),'예' if d.get('completed') else '아니오','충족' if d.get('expected_result_met') else '미충족',d.get('clearance_p05_m'),d.get('elapsed_sim_s'),d.get('cpu',{}).get('gpp'),d.get('cpu',{}).get('lpp'),d.get('gpp_path_publications'),d.get('invalidity_replans'),self.slope_label(d.get('scenario'),(d.get('terrain') or {}).get('max_slope_deg'))]
            for j,v in enumerate(values):
                item=QTableWidgetItem(fmt(v));item.setToolTip(str(folder));self.results.setItem(i,j,item)
        if self.records:self.results.selectRow(len(self.records)-1)
        self.results.blockSignals(False)
        self.draw_result()
    def draw_result(self):
        self.figure.clear();i=self.results.currentRow()
        if i<0 or i>=len(self.records):self.figure.text(.5,.5,'선택한 시나리오의 실행 기록이 없습니다.',ha='center');self.canvas.draw();return
        folder,d=self.records[i];spec=read(folder/'map'/'scenario.json') or scenario(d['scenario']);ax=self.figure.add_subplot(121)
        import numpy as np
        ax.imshow(np.array(spec.get('planning_grid',spec['grid'])).reshape(spec['height'],spec['width']),origin='lower',extent=(0,24,0,16.2),cmap='Greys',vmin=0,vmax=100,alpha=.4)
        rows=[]
        try:
            with (folder/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
        except OSError:pass
        ax.scatter(*spec['start'][:2],marker='o',label='출발');ax.scatter(*spec['goal'],marker='*',s=90,label='목표')
        if rows:ax.plot([float(r['x']) for r in rows],[float(r['y']) for r in rows],color='#287ba8',label='Gazebo 주행 궤적')
        ax.set(xlabel='X (m)',ylabel='Y (m)',title='기록된 실제 시뮬레이션 궤적');ax.legend(fontsize=8)
        bx=self.figure.add_subplot(222);cx=self.figure.add_subplot(224)
        if rows:
            t=[float(r['time_s']) for r in rows];bx.plot(t,[float(r['speed']) for r in rows]);valid=[r for r in rows if r['clearance_m']];cx.plot([float(r['time_s']) for r in valid],[float(r['clearance_m']) for r in valid],color='#27816e')
            if d.get('clearance_p05_m') is not None:cx.axhline(d['clearance_p05_m'],color='#bc7425',ls='--',label='p05');cx.legend(fontsize=8)
        bx.set(ylabel='속도 (m/s)',title='주행 기록');cx.set(xlabel='주행 시간 (sim s)',ylabel='장애물 여유 (m)');self.canvas.draw()
        self.result_note.setText(f"{d.get('result')} · p05: 주행 중 약 10 Hz 표본의 하위 5% 여유 거리. 차량 외곽의 XY 겹침은 접촉력 측정과 구분합니다.\n기록 폴더: {folder}")
    def closeEvent(self,event):
        if self.process:self.closing=True;self.stop();event.ignore();return
        self.close_viewer();self.persist();self.lock.close();event.accept()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,default=workspace()/'sessions/planning_lab');parser.add_argument('--screenshot',type=Path);a=parser.parse_args();app=QApplication(sys.argv[:1]);app.setStyle('Fusion')
    try:window=Lab(a.root)
    except BlockingIOError:QMessageBox.warning(None,'중복 실행','같은 실험실 GUI가 이미 실행 중입니다.');return 1
    window.show()
    if a.screenshot:
        def capture():window.grab().save(str(a.screenshot));window.close();app.quit()
        QTimer.singleShot(800,capture)
    return app.exec_()

if __name__=='__main__':sys.exit(main())
