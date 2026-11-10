"""Local operator-button timing, separate from vehicle commands and motor state.

The same module is shipped beside the driving GUI and the log viewer. The wire
command is unchanged; only local click/ack/connection events are persisted.
"""
from collections import defaultdict, deque
import json
import os
from pathlib import Path
import re
import time
import uuid

RUN_ID = re.compile(r'planning_output_\d{8}_\d{6}_\d+')


def default_directory():
    return Path(os.environ.get('NOMAD_PLANNING_TIMING_DIR',
                str(Path.home()/'.local/share/nomad-planning-log-gui/operator_events'))).expanduser()


class OperatorJournal:
    def __init__(self,directory=None):
        self.client_id=uuid.uuid4().hex
        self.path=Path(directory) / (self.client_id+'.jsonl') if directory is not None else default_directory()/(self.client_id+'.jsonl')
        self.run_id=None;self.pending=defaultdict(deque);self.error='';self.stderr_buffer=''
    def write(self,event,**fields):
        record=dict(schema='nomad_operator_timing_v1',client_id=self.client_id,run_id=self.run_id,
                    event=event,event_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),**fields)
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('a',encoding='utf-8') as f:
                f.write(json.dumps(record,ensure_ascii=False,separators=(',',':'))+'\n')
            self.error=''
        except OSError as e:self.error='주행 클릭 시각 저장 실패: '+str(e)
    def observe_stderr(self,chunk):
        # Preserve only the launcher-generated run identifier, never raw SSH
        # diagnostics, credentials, hostnames or arbitrary command output.
        self.stderr_buffer=(self.stderr_buffer+chunk)[-8192:]
        while '\n' in self.stderr_buffer:
            line,self.stderr_buffer=self.stderr_buffer.split('\n',1)
            if 'logs:' not in line:continue
            match=RUN_ID.search(line.split('logs:',1)[1])
            if match and match[0]!=self.run_id:
                self.run_id=match[0];self.write('bind')
    def request(self,action,*,timing=None,origin='command_send',sent=True):
        if action not in ('arm','disarm'):return
        click_ns,click_mono_ns=timing or (time.time_ns(),time.monotonic_ns())
        request_id=uuid.uuid4().hex
        self.write('request',request_id=request_id,action=action,click_ns=click_ns,
                   click_mono_ns=click_mono_ns,origin=origin,sent=bool(sent))
        if sent:self.pending[action].append(request_id)
    def acknowledge(self,action,accepted):
        if action not in ('arm','disarm'):return
        request_id=self.pending[action].popleft() if self.pending[action] else None
        self.write('ack',request_id=request_id,action=action,accepted=accepted is True)
    def close(self):self.write('closed')


def read_records(paths,run_id):
    """Match exact launcher run id, including bind events arriving after a click."""
    records=[]
    for path in paths:
        try:
            for line in Path(path).read_text(encoding='utf-8',errors='replace').splitlines():
                try:
                    d=json.loads(line)
                    if isinstance(d,dict) and d.get('schema')=='nomad_operator_timing_v1':records.append(d)
                except ValueError:pass
        except OSError:continue
    bound={d.get('client_id') for d in records if d.get('run_id')==run_id}
    found=[d for d in records if d.get('client_id') in bound]
    unique={json.dumps(d,sort_keys=True):d for d in found}
    return sorted(unique.values(),key=lambda d:d.get('monotonic_ns',0))


def sessions_from_records(records,observation_end_ns):
    """Accepted arm uses its original click; repeated arm never resets a session."""
    sessions=[];requests={};active=None
    def finish(ns,mono,kind):
        nonlocal active
        if active is None:return
        duration=(mono-active['start_mono_ns'])/1e9 if isinstance(mono,int) else None
        active.update(end_ns=ns,end_mono_ns=mono,end_kind=kind,closed=True,
                      elapsed_s=duration if duration is not None and duration>=0 else None)
        sessions.append(active);active=None
    for d in records:
        event=d.get('event')
        if event=='request' and d.get('sent') is True:requests[d.get('request_id')]=d
        elif event=='ack' and d.get('accepted') is True:
            r=requests.get(d.get('request_id'))
            if not r:continue
            start,mono=r.get('click_ns'),r.get('click_mono_ns')
            if not isinstance(start,int) or not isinstance(mono,int) or start<=0 or mono<0:continue
            if r.get('action')=='arm':
                if active is None:
                    active=dict(start_ns=start,start_mono_ns=mono,request_id=r['request_id'],
                                origin=r.get('origin'),client_id=r.get('client_id'),
                                exact_button=r.get('origin')=='output_allow_button',
                                accepted_ns=d.get('event_ns'),closed=False)
            elif r.get('action')=='disarm':finish(start,mono,r.get('origin','disarm_request'))
        elif event=='closed' and active and d.get('client_id')==active['client_id']:
            finish(d.get('event_ns'),d.get('monotonic_ns'),'connection_closed')
    if active:
        active.update(end_ns=max(active['start_ns'],observation_end_ns),end_mono_ns=None,
                      end_kind='last_log_observation',closed=False,elapsed_s=None)
        sessions.append(active)
    return sessions
