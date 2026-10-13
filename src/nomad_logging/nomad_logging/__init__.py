"""Log only selected scalar values already computed by a module."""
import csv
import os
import sys
from pathlib import Path


class CsvLog:
    def __init__(self,name,fields,interval=.2):
        self.name=name;self.fields=list(fields);self.interval=interval;self.last=None;self.epoch=0;self.file=None;self.writer=None
        root=os.environ.get('NOMAD_RUN_LOG_DIR')
        self.root=Path(root) if root else None
        if self.root:self.open()

    def open(self):
        try:
            self.root.mkdir(parents=True,exist_ok=True)
            self.path=self.root/f'{self.name}_{os.getpid()}_{self.epoch}.csv'
            self.file=self.path.open('w',newline='',buffering=1)
            self.writer=csv.writer(self.file);self.writer.writerow(['time']+self.fields)
        except OSError as error:
            self.file=None;print(f'[WARN] [nomad_logging] numeric logging unavailable: {error}',file=sys.stderr)

    def emit(self,stamp,*values):
        if self.file is None:return
        if self.last is not None and stamp<self.last:
            self.file.close();self.epoch+=1;self.last=None;self.open()
        if self.file is None:return
        if self.last is not None and stamp-self.last<self.interval:return
        try:
            self.writer.writerow([stamp]+list(values));self.last=stamp
        except OSError as error:
            self.file.close();self.file=None
            print(f'[WARN] [nomad_logging] numeric logging stopped: {error}',file=sys.stderr)

    def close(self):
        if self.file:self.file.close();self.file=None
