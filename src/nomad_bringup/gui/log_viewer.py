"""Offline result browser: select files/fields, inspect 3D, plots and playback."""
from pathlib import Path
from datetime import datetime
import json
import time
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QSplitter, QWidget,
    QPushButton, QLabel, QTableWidget, QTableWidgetItem, QFileDialog, QTreeWidget,
    QTreeWidgetItem, QTabWidget, QPlainTextEdit, QSlider, QComboBox, QCheckBox,
    QDoubleSpinBox, QMessageBox, QAbstractItemView, QLineEdit)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from log_data import load_numeric, sessions, rigid_alignment, configure_fonts, load_snapshot, recover_console_metrics, summary_plots


class LogViewer(QDialog):
    def __init__(self, parent, root):
        super().__init__(parent)
        self.setWindowTitle('NOMAD 로그 분석 · 저장한 파일만 사용')
        self.resize(1280,860)
        configure_fonts()
        self.map_files=[]
        self.root=Path(root);self.datasets=[];self.files=[];self.loading=False;self.session_dir=None;self.base_console="";self.console_text=""
        layout=QVBoxLayout(self)
        bar=QHBoxLayout()
        for title,callback in [('목록 새로고침',self.refresh),('로그 폴더 선택',self.choose_root),('결과 파일 열기',self.open_files),('현재 그림 저장',self.export_plot)]:
            button=QPushButton(title);button.clicked.connect(callback);bar.addWidget(button)
        layout.addLayout(bar)
        split=QSplitter();layout.addWidget(split,1)
        left=QWidget();lv=QVBoxLayout(left);split.addWidget(left)
        self.catalog=QTableWidget(0,3)
        self.catalog.setHorizontalHeaderLabels(['기록명','저장 시간','결과 파일'])
        self.catalog.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.catalog.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.catalog.horizontalHeader().setStretchLastSection(True)
        self.catalog.setColumnWidth(0,105);self.catalog.setColumnWidth(1,155);self.catalog.setColumnWidth(2,65)
        self.catalog.setMaximumHeight(220)
        self.catalog.cellClicked.connect(self.open_session)
        lv.addWidget(self.catalog)
        lv.addWidget(QLabel('보고 싶은 파일·수치 선택'))
        self.tree=QTreeWidget();self.tree.setHeaderLabels(['기록 / 항목'])
        self.tree.itemChanged.connect(self.selection_changed);lv.addWidget(self.tree,1)
        time_row=QHBoxLayout();time_row.addWidget(QLabel('입력 시간 단위'))
        self.units=QComboBox();self.units.addItems(['auto','s','ms','us','ns'])
        self.units.currentTextChanged.connect(self.reload_files);time_row.addWidget(self.units);lv.addLayout(time_row)
        lv.addWidget(QLabel('SE(3) 정렬 기준 파일'))
        self.reference=QComboBox();self.reference.currentIndexChanged.connect(self.selection_changed);lv.addWidget(self.reference)
        self.align=QCheckBox('SE(3) 정렬 · 스케일 유지')
        self.align.setToolTip('선택한 기준 궤적에 시간으로 대응시켜 회전·이동만 보정합니다. 같은 측정 기준점과 단위를 사용하는 파일끼리 비교하세요.')
        self.align.toggled.connect(self.selection_changed);lv.addWidget(self.align)
        gaprow=QHBoxLayout();gaprow.addWidget(QLabel('보간 최대 공백 [s]'))
        self.max_gap=QDoubleSpinBox();self.max_gap.setRange(.01,60);self.max_gap.setValue(.3);self.max_gap.setSingleStep(.1)
        self.max_gap.valueChanged.connect(self.selection_changed);gaprow.addWidget(self.max_gap);lv.addLayout(gaprow)
        right=QWidget();rv=QVBoxLayout(right);split.addWidget(right)
        self.tabs=QTabWidget();rv.addWidget(self.tabs,1)
        self.figures={};self.canvases={}
        for name in ['3D 궤적','수치 그래프','분포·통계']:
            page=QWidget();pl=QVBoxLayout(page);fig=Figure(tight_layout=True);canvas=FigureCanvasQTAgg(fig)
            pl.addWidget(NavigationToolbar2QT(canvas,self));pl.addWidget(canvas,1)
            self.tabs.addTab(page,name);self.figures[name]=fig;self.canvases[name]=canvas
        map_page=QWidget();map_layout=QVBoxLayout(map_page)
        self.map_choice=QComboBox();self.map_choice.currentIndexChanged.connect(self.draw_map);map_layout.addWidget(self.map_choice)
        map_figure=Figure(tight_layout=True);map_canvas=FigureCanvasQTAgg(map_figure)
        map_layout.addWidget(NavigationToolbar2QT(map_canvas,self));map_layout.addWidget(map_canvas,1)
        self.tabs.addTab(map_page,'지도·이미지');self.figures['지도·이미지']=map_figure;self.canvases['지도·이미지']=map_canvas
        text_page=QWidget();text_layout=QVBoxLayout(text_page)
        filter_row=QHBoxLayout();self.text_query=QLineEdit();self.text_query.setPlaceholderText('로그 내용 검색')
        self.errors_only=QCheckBox('오류·경고만');filter_row.addWidget(self.text_query,1);filter_row.addWidget(self.errors_only)
        self.text_query.textChanged.connect(self.filter_console);self.errors_only.toggled.connect(self.filter_console)
        text_layout.addLayout(filter_row)
        self.text=QPlainTextEdit();self.text.setReadOnly(True);text_layout.addWidget(self.text,1)
        self.tabs.addTab(text_page,'실행 로그·오류')
        self.settings=QPlainTextEdit();self.settings.setReadOnly(True);self.tabs.addTab(self.settings,'실행 설정')
        self.notice=QLabel('저장한 기록을 선택하거나 결과 파일을 여세요. 실시간 토픽을 수집하지 않습니다.')
        self.notice.setWordWrap(True);rv.addWidget(self.notice)
        controls=QHBoxLayout();self.play=QPushButton('재생');self.play.clicked.connect(self.toggle_play);controls.addWidget(self.play)
        self.slider=QSlider(Qt.Horizontal);self.slider.setRange(0,1000);self.slider.setValue(1000);self.slider.valueChanged.connect(self.draw)
        controls.addWidget(self.slider,1);self.speed=QComboBox();self.speed.addItems(['0.5×','1×','2×','5×']);self.speed.setCurrentIndex(1);controls.addWidget(self.speed)
        self.time_label=QLabel('');controls.addWidget(self.time_label);rv.addLayout(controls)
        self.timer=QTimer(self);self.timer.setInterval(100);self.timer.timeout.connect(self.advance)
        self.bounds=(0.,1.);self.position_cache={};self.metrics=[]
        split.setSizes([380,880]);self.refresh()

    def refresh(self):
        self.rows=sessions(self.root);self.catalog.setRowCount(len(self.rows))
        for i,row in enumerate(self.rows):
            try:date=datetime.fromisoformat(row['created_at']).astimezone().strftime('%Y-%m-%d %H:%M:%S')
            except (ValueError,KeyError):date='시간 정보 없음'
            for j,value in enumerate([row.get('name',''),date,str(len(row.get('files',[])))]):
                self.catalog.setItem(i,j,QTableWidgetItem(value))

    def choose_root(self):
        path=QFileDialog.getExistingDirectory(self,'저장 로그 폴더',str(self.root))
        if path:self.root=Path(path);self.refresh()

    def open_session(self,row,_column=0):
        item=self.rows[row];self.session_dir=item['directory']
        if not item.get('console_metrics_processed'):
            previous=len(item.get('files',[]))
            repaired=recover_console_metrics(self.session_dir)
            item={**repaired,'directory':self.session_dir};self.rows[row]=item
            self.catalog.setItem(row,2,QTableWidgetItem(str(len(item.get('files',[])))))
            if len(item.get('files',[]))>previous:summary_plots(self.session_dir)
        console=self.session_dir/item.get('console','console.log')
        self.base_console=console.read_text(errors='replace') if console.exists() else '실행 텍스트 로그가 없습니다.'
        config=self.session_dir/'settings.json'
        self.settings.setPlainText(config.read_text() if config.exists() else '')
        self.load_files([self.session_dir/file['path'] for file in item.get('files',[])])

    def open_files(self):
        paths,_=QFileDialog.getOpenFileNames(self,'기존 결과 파일 선택',str(self.root),'결과 파일 (*.tum *.csv *.json *.jsonl *.txt *.log *.npz *.png *.jpg *.jpeg);;모든 파일 (*)')
        if paths:
            self.session_dir=None;self.base_console="";self.text.clear();self.settings.clear();self.load_files(paths)

    def reload_files(self,*_):
        if self.files:self.load_files(self.files)

    def load_files(self,paths):
        self.timer.stop();self.play.setText('재생');self.files=[Path(path) for path in paths]
        self.loading=True;self.tree.clear();self.reference.clear();self.datasets=[];self.map_files=[];self.map_choice.clear();errors=[];text=[]
        for path in self.files:
            if path.suffix.lower() in ('.npz','.png','.jpg','.jpeg'):
                try:
                    snapshot=load_snapshot(path);self.map_files.append((path,snapshot));self.map_choice.addItem(path.name)
                except (OSError,ValueError) as error:errors.append(f'{path.name}: {error}')
                continue
            try:
                data=load_numeric(path,self.units.currentText());index=len(self.datasets);self.datasets.append(data)
            except (OSError,ValueError,TypeError,json.JSONDecodeError) as error:
                errors.append(f'{path.name}: {error}')
                if path.suffix.lower() in ('.txt','.log') and path.is_file():text.append(f'=== {path.name} ===\n'+path.read_text(errors='replace'))
                continue
            item=QTreeWidgetItem(self.tree,[path.name]);item.setToolTip(0,str(path)+'\n파일 시각: '+datetime.fromtimestamp(path.stat().st_mtime).astimezone().strftime('%Y-%m-%d %H:%M:%S'));item.setData(0,Qt.UserRole,(index,None))
            item.setFlags(item.flags()|Qt.ItemIsUserCheckable);item.setCheckState(0,Qt.Checked if data.xyz else Qt.Unchecked)
            for j,key in enumerate(data.fields):
                child=QTreeWidgetItem(item,[key]);child.setData(0,Qt.UserRole,(index,key))
                child.setFlags(child.flags()|Qt.ItemIsUserCheckable);child.setCheckState(0,Qt.Checked if j<2 and not data.xyz else Qt.Unchecked)
            item.setExpanded(True)
            if data.xyz:self.reference.addItem(path.name,index)
        self.console_text=self.base_console+'\n'+'\n'.join(text)
        self.filter_console()
        gt=next((i for i in range(self.reference.count()) if 'gt' in self.reference.itemText(i).lower() or 'ground_truth' in self.reference.itemText(i).lower()),0)
        if self.reference.count():self.reference.setCurrentIndex(gt)
        timed=[data for data in self.datasets if data.has_time]
        if timed:self.bounds=(min(data.time[0] for data in timed),max(data.time[-1] for data in timed))
        elif self.datasets:self.bounds=(0,max(data.time[-1] for data in self.datasets))
        else:self.bounds=(0.,1.)
        self.loading=False
        self.play.setEnabled(bool(self.datasets) and self.bounds[1]>self.bounds[0])
        self.slider.setEnabled(bool(self.datasets))
        target = '3D 궤적' if any(data.xyz for data in self.datasets) else '수치 그래프' if self.datasets else '지도·이미지' if self.map_files else '실행 로그·오류'
        self.tabs.setCurrentIndex(next(i for i in range(self.tabs.count()) if self.tabs.tabText(i)==target))
        self.draw_map()
        self.load_errors=errors;self.slider.setValue(1000);self.selection_changed()

    def filter_console(self,*_):
        query=self.text_query.text().casefold();only_errors=self.errors_only.isChecked()
        lines=[]
        for line in self.console_text.splitlines():
            lower=line.casefold()
            if query and query not in lower:continue
            if only_errors and not any(word in lower for word in ('error','warn','fault','failed','exception','오류','경고','실패')):continue
            lines.append(line)
        self.text.setPlainText('\n'.join(lines))

    def selected(self):
        poses=[];scalars=[]
        for i in range(self.tree.topLevelItemCount()):
            item=self.tree.topLevelItem(i);index=item.data(0,Qt.UserRole)[0]
            if item.checkState(0)==Qt.Checked and self.datasets[index].xyz:poses.append(index)
            for j in range(item.childCount()):
                child=item.child(j)
                if child.checkState(0)==Qt.Checked:scalars.append(child.data(0,Qt.UserRole))
        return poses,scalars

    def selection_changed(self,*_):
        if self.loading:return
        self.position_cache={};self.metrics=[]
        poses,_=self.selected()
        reference_index=self.reference.currentData()
        for index in poses:
            data=self.datasets[index];points=np.column_stack([data.fields[key] for key in data.xyz])
            if self.align.isChecked() and reference_index is not None and index!=reference_index:
                try:
                    points,rmse,count=rigid_alignment(self.datasets[reference_index],data,self.max_gap.value())
                    self.metrics.append(f'{data.path.name}: RMSE {rmse:.4f} (위치 단위), 대응 {count}/{len(data.time)}')
                except ValueError as error:self.metrics.append(f'{data.path.name}: 정렬 미적용 — {error}')
            self.position_cache[index]=points
        self.draw()

    def draw(self,*_):
        if self.loading:return
        lo,hi=self.bounds;now=lo+(hi-lo)*self.slider.value()/1000
        poses,scalars=self.selected()
        self.time_label.setText(f'{now-lo:.2f} / {hi-lo:.2f} '+('s' if any(d.has_time for d in self.datasets) else 'samples'))
        fig=self.figures['3D 궤적'];view=(fig.axes[0].elev,fig.axes[0].azim) if fig.axes else (30,-60)
        fig.clear();axes=fig.add_subplot(111,projection='3d');axes.view_init(*view)
        limits=[]
        for index in poses:
            data=self.datasets[index];points=self.position_cache.get(index)
            if points is None:continue
            valid=np.isfinite(points).all(axis=1);limits.append(points[valid])
            mask=valid & (data.time<=now)
            shown=points[mask]
            if shown.size:
                axes.plot(*shown[::max(1,len(shown)//5000)].T,label=data.path.name)
                axes.scatter(*shown[-1],s=25)
        if limits and any(len(x) for x in limits):
            points=np.concatenate([p for p in limits if len(p)])
            center=(points.min(axis=0)+points.max(axis=0))/2;span=max(np.ptp(points,axis=0).max(),.1)/2
            axes.set_xlim(center[0]-span,center[0]+span);axes.set_ylim(center[1]-span,center[1]+span);axes.set_zlim(center[2]-span,center[2]+span)
            axes.set_box_aspect((1,1,1))
        axes.set(xlabel='X (file units)',ylabel='Y (file units)',zlabel='Z (file units)')
        if axes.lines:axes.legend(loc='best',fontsize=8)
        self.canvases['3D 궤적'].draw_idle()
        fig=self.figures['수치 그래프'];fig.clear()
        for i,(index,key) in enumerate(scalars[:12]):
            data=self.datasets[index];axes=fig.add_subplot((min(len(scalars),12)+1)//2,2,i+1)
            if len(data.time)==1:axes.bar([key],[data.fields[key][0]])
            else:
                axes.plot(data.time-lo if data.has_time else data.time,data.fields[key],linewidth=1)
                if data.has_time:axes.axvline(now-lo,color='red',linewidth=.8)
            axes.set_title(f'{data.path.name}: {key}',fontsize=9);axes.set_xlabel('Time [s]' if data.has_time else 'Sample')
            axes.grid(True,alpha=.2)
        self.canvases['수치 그래프'].draw_idle()
        fig=self.figures['분포·통계'];fig.clear()
        for i,(index,key) in enumerate(scalars[:12]):
            data=self.datasets[index];values=data.fields[key];values=values[np.isfinite(values)]
            axes=fig.add_subplot((min(len(scalars),12)+1)//2,2,i+1)
            if values.size:
                axes.hist(values,bins=min(30,max(1,round(np.sqrt(len(values))))),alpha=.8)
                axes.set_title(f'{data.path.name}: {key}',fontsize=9)
                axes.set_xlabel(f'median={np.median(values):.4g}, P95={np.percentile(values,95):.4g}')
                axes.set_ylabel('Count');axes.grid(True,alpha=.2)
        self.canvases['분포·통계'].draw_idle()
        info=[f'{len(self.datasets)}개 수치 파일 · 시간 단위 '+', '.join(sorted({d.time_unit for d in self.datasets}))]
        if poses:info.append('파일의 좌표·기준점·위치 단위를 확인하세요. 자동 원점/기체 변환은 하지 않습니다.')
        if len(scalars)>12:info.append('한 화면은 선택한 수치 중 12개까지 표시합니다.')
        info+=self.metrics+self.load_errors[:3]
        self.notice.setText('\n'.join(info))

    def toggle_play(self):
        if self.timer.isActive():self.timer.stop();self.play.setText('재생')
        else:
            if self.slider.value()==1000:self.slider.setValue(0)
            self.playhead=self.bounds[0]+(self.bounds[1]-self.bounds[0])*self.slider.value()/1000
            self.last_tick=time.monotonic();self.timer.start();self.play.setText('일시 정지')

    def advance(self):
        duration=max(self.bounds[1]-self.bounds[0],.1)
        now=time.monotonic();elapsed=now-self.last_tick;self.last_tick=now
        self.playhead+=elapsed*float(self.speed.currentText().rstrip('×'))
        value=round(1000*(self.playhead-self.bounds[0])/duration)
        self.slider.setValue(min(1000,value))
        if self.playhead>=self.bounds[1]:self.timer.stop();self.play.setText('재생')

    def draw_map(self,*_):
        if self.loading:return
        figure=self.figures['지도·이미지'];figure.clear()
        index=self.map_choice.currentIndex()
        if index<0 or index>=len(self.map_files):
            self.canvases['지도·이미지'].draw_idle();return
        path,snapshot=self.map_files[index];layers=list(snapshot['layers'].items())
        for i,(name,data) in enumerate(layers):
            axes=figure.add_subplot(1,len(layers),i+1)
            extent=None
            if snapshot['resolution'] is not None:
                x,y=snapshot['origin'];h,w=data.shape
                extent=[x,x+w*snapshot['resolution'],y,y+h*snapshot['resolution']]
            image=axes.imshow(data,origin='lower' if extent else 'upper',extent=extent,
                              cmap='tab10' if name=='classes' else 'viridis')
            axes.set_title(name+{'height':' [m]','slope':' [deg]','classes':' [class ID]'}.get(name,''))
            if data.ndim==2:figure.colorbar(image,ax=axes,shrink=.7)
        self.canvases['지도·이미지'].draw_idle()

    def export_plot(self):
        title=self.tabs.tabText(self.tabs.currentIndex())
        if title not in self.figures:
            QMessageBox.information(self,'그림 선택','궤적·수치·분포·지도 탭에서 그림을 선택하세요.');return
        path,_=QFileDialog.getSaveFileName(self,'현재 그림 저장',str(self.session_dir or self.root)+'/'+('trajectory.png' if title=='3D 궤적' else 'scalars.png'),'PNG (*.png);;PDF (*.pdf)')
        if path:self.figures[title].savefig(path,dpi=160)

    def closeEvent(self,event):
        self.timer.stop();super().closeEvent(event)
