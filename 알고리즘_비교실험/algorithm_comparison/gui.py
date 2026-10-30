"""Scenario laboratory queue and plots plus five-pair comparison tables."""
import argparse
import csv
import sys
from pathlib import Path
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,
    QTableWidget,QTableWidgetItem,QHeaderView,QMessageBox,QComboBox)
from nomad_bringup.scenario_lab.gui import Lab
from nomad_bringup.scenario_lab.core import read,catalog
from .model import ALGORITHMS,HEADERS,comparison

ROOT=Path(__file__).resolve().parents[1]


class ComparisonLab(Lab):
    def __init__(self,root):
        super().__init__(root,algorithms=ALGORITHMS,title='알고리즘 비교 실험실')
        panel=QWidget();layout=QVBoxLayout(panel)
        note=QLabel('시나리오 결과 탭에서 선택한 시나리오를 비교합니다.\n'
                    '시간·여유·CPU 통계는 기대 결과 충족 + 프로세스 정리 성공 시험만 집계합니다. '
                    '실패·오류·중단은 별도 건수이며, 짧게 끝난 실패를 성능 개선으로 계산하지 않습니다.\n'
                    '1~4: 목표 도착 / 5: 벽 앞 정지 / 6: 전진 불가 정지. '
                    'CPU는 프로세스 누적 core·s이며 전략 함수 지연시간이 아닙니다.')
        note.setWordWrap(True);layout.addWidget(note)
        row=QHBoxLayout()
        self.compare_select=QComboBox()
        for s in catalog()['scenarios']:self.compare_select.addItem(f"{s['id']}. {s['name']}",s['id'])
        self.compare_select.currentIndexChanged.connect(self.result_select.setCurrentIndex)
        self.result_select.currentIndexChanged.connect(self.compare_select.setCurrentIndex)
        row.addWidget(self.compare_select)
        self.button(row,'선택 시나리오 × 5조합 추가',self.add_comparison)
        self.button(row,'6시나리오 × 5조합 추가',self.add_matrix)
        self.button(row,'비교표 CSV 저장',self.export)
        layout.addLayout(row)
        self.comparison_title=QLabel();layout.addWidget(self.comparison_title)
        self.summary=QTableWidget(0,len(HEADERS));self.summary.setHorizontalHeaderLabels(HEADERS)
        self.summary.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.summary.setEditTriggers(QTableWidget.NoEditTriggers);self.summary.verticalHeader().hide();layout.addWidget(self.summary)
        self.caveat=QLabel('미실행은 0건·—입니다. 최적 조합을 자동 선정하지 않습니다. '
                          '반복 횟수와 시간 제한은 실행 큐 탭의 값을 사용합니다. '
                          '비교표 추가 대상은 시나리오 결과 탭의 선택입니다.\n'
                          '동일 환경의 5조합 비교이며, GPP와 LPP를 동시에 바꾼 조합의 차이를 한 알고리즘의 효과로 해석하지 마세요.')
        self.caveat.setWordWrap(True);layout.addWidget(self.caveat)
        self.tabs.addTab(panel,'알고리즘 비교표');self.refresh_comparison()
        # Also expose matrix controls next to the existing queue controls.
        row=QHBoxLayout()
        self.button(row,'선택 시나리오 × 5조합 추가',lambda:self.add_comparison(self.select.currentData()))
        self.button(row,'6시나리오 × 5조합 추가',self.add_matrix)
        self.tabs.widget(0).layout().insertLayout(2,row)

    def add_comparison(self,number=None):
        if number is None or isinstance(number,bool):number=self.result_select.currentData()
        previous=self.algorithm.currentIndex();repeats=self.repeat.value()
        # Rotate order across repetitions to reduce fixed execution-order effects.
        self.repeat.setValue(1)
        for repeat in range(repeats):
            for offset in range(len(ALGORITHMS)):
                self.algorithm.setCurrentIndex((repeat+offset)%len(ALGORITHMS));self.append(number)
        self.repeat.setValue(repeats);self.algorithm.setCurrentIndex(previous);self.refresh_queue()

    def add_matrix(self):
        for number in range(1,7):self.add_comparison(number)

    def refresh_results(self):
        super().refresh_results()
        self.results.setColumnCount(13)
        self.results.setHorizontalHeaderItem(12,QTableWidgetItem('LPP 연산 p95(ms)'))
        for i,(_,d) in enumerate(self.records):
            latency=d.get('strategy_timing',{}).get('lpp_strategy',{}).get('p95_ms')
            self.results.setItem(i,12,QTableWidgetItem('—' if latency is None else f'{latency:.3f}'))
        if hasattr(self,'summary'):self.refresh_comparison()

    def refresh_comparison(self):
        self.comparison_title.setText(self.result_select.currentText())
        self.summary_rows=comparison([d for _,d in self.records]);self.summary.setRowCount(len(self.summary_rows))
        for i,row in enumerate(self.summary_rows):
            for j,value in enumerate(row):
                text='—' if value is None else f'{value:.3f}' if isinstance(value,float) else str(value)
                self.summary.setItem(i,j,QTableWidgetItem(text))
        limits={read(folder/'manifest.json',{}).get('timeout_sim_s') for folder,_ in self.records}
        limits.discard(None)
        if len(limits)>1:self.comparison_title.setText(self.result_select.currentText()+' · 주의: 시간 제한이 다른 기록이 섞여 있습니다.')

    def export(self):
        self.refresh_results()
        path=self.root/f'comparison_scenario_{self.result_select.currentData()}.csv'
        with path.open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.writer(f);writer.writerow(HEADERS);writer.writerows(self.summary_rows)
        QMessageBox.information(self,'비교표 저장',str(path))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT/'results')
    parser.add_argument('--screenshot',type=Path)
    args=parser.parse_args();app=QApplication(sys.argv[:1]);app.setStyle('Fusion')
    try:window=ComparisonLab(args.root)
    except BlockingIOError:
        QMessageBox.warning(None,'중복 실행','같은 비교 실험실 GUI가 이미 실행 중입니다.');return 1
    window.show()
    if args.screenshot:
        def capture():
            window.tabs.setCurrentIndex(2);window.grab().save(str(args.screenshot));window.close();app.quit()
        QTimer.singleShot(800,capture)
    return app.exec_()

if __name__=='__main__':sys.exit(main())
