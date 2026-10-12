import csv
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import pytest
import comparison
import experiment
from common import read, save, WORKSPACE
from report import evaluate


def test_original_43_clicks_preserved_and_18_trials_are_balanced(tmp_path):
    plan=comparison.load_route()
    assert len(plan['goals'])==43
    assert plan['goals'][0]==[-26.4062,-5.13177]
    assert plan['goals'][-1]==[-28.3995,.485478]
    assert plan['goals'][12:14]==[[10.8141,-18.4269],[10.0604,-18.0664]]
    manifest=dict(limits={},notes=[])
    conditions=[dict(number=i,label=p[0],gpp=p[1],lpp=p[2]) for i,p in enumerate(experiment.pairs(),1)]
    comparison.configure(tmp_path,manifest,plan,conditions)
    assert manifest['pilot_requires_success'] is False
    assert len(manifest['trials'])==18 and len(manifest['pilots'])==6
    for i in range(1,7):
        assert sorted(t['repeat'] for t in manifest['trials'] if t['pair']['number']==i)==[1,2,3]
    assert all(t['waypoints']==plan['goals'] for t in manifest['trials'])
    assert manifest['references']['04_vio_flat_loop']['validated'] is False


@pytest.mark.parametrize('kw',[dict(repeats=0),dict(selected=[1,1]),dict(selected=[7]),dict(source='../bad')])
def test_invalid_comparison_conditions_rejected(kw):
    with pytest.raises(ValueError):comparison.load_route(**kw)


def test_source_checksum_prevents_silent_route_edit(tmp_path,monkeypatch):
    folder=tmp_path/'route';folder.mkdir()
    save(folder/'checksums.json',{'waypoints.csv':'wrong'})
    (folder/'waypoints.csv').write_text('changed')
    monkeypatch.setattr(comparison,'ROOT',tmp_path)
    with pytest.raises(ValueError,match='체크섬'):comparison.load_route('route')


@pytest.mark.parametrize('maps_count',[1,8])
def test_report_handles_actual_map_count_and_planned_repeats(tmp_path,maps_count):
    maps=[f'{i:02}_map' for i in range(1,maps_count+1)]
    trials=[dict(id=f'{mid}_p{p}_r{r}',map=mid,pair=dict(number=p,label=f'Pair {p}'),repeat=r,pilot=False)
            for mid in maps for p in [1,3] for r in range(1,4)]
    manifest=dict(name='fixture',commit='test',weights=dict(sr=.5,spl=.25,sct=.25),pilots=[],trials=trials,
                  references={m:dict(validated=False) for m in maps})
    save(tmp_path/'manifest.json',manifest)
    # Three valid trials for pair1 on first map, including a performance failure.
    for trial in trials[:3]:
        attempt=tmp_path/'runs'/trial['id']/'attempt_001'
        save(attempt/'outcome.json',dict(result='success' if trial['repeat']<3 else 'no_progress',valid=True,
             bag_verified=True,distance_xy_m=150.,distance_3d_m=151.,duration_sim_s=400.,
             reverse_distance_m=2.,waypoints_reached=43 if trial['repeat']<3 else 10,total_waypoints=43))
        timing=attempt/'timing';timing.mkdir()
        (timing/'gpp.csv').write_text('stage,elapsed_wall_ns,thread_cpu_ns\ngpp,1000000,500000\n')
    evaluate(tmp_path)
    summary=read(tmp_path/'results/summary.json')['summary']
    assert summary[0]['valid_runs']==3 and summary[0]['planned_runs']==3*maps_count
    assert summary[0]['complete']==(maps_count==1)
    assert summary[0]['sr']==pytest.approx(2/3)
    assert summary[0]['weighted_score'] is None
    assert summary[1]['sr'] is None
    with (tmp_path/'results/timing_summary.csv').open() as f:timing=list(csv.DictReader(f))
    assert timing[0]['calls']=='3' and float(timing[0]['mean_thread_cpu_ms'])==.5
    assert (tmp_path/'results/figures/trajectories.png').stat().st_size>0


def test_resume_skips_valid_failures_but_pilot_failure_blocks(tmp_path,monkeypatch):
    pilot=dict(id='pilot',pilot=True)
    main=dict(id='main',pilot=False)
    manifest=dict(pilots=[pilot],trials=[main],pilot_requires_success=True)
    monkeypatch.setattr(experiment,'execute',lambda *args:dict(result='timeout',valid=True))
    assert experiment.run(tmp_path,manifest)==2
    assert read(tmp_path/'status.json')['state']=='blocked'
    save(tmp_path/'runs/pilot/attempt_001/outcome.json',dict(result='success',valid=True,bag_verified=True))
    save(tmp_path/'runs/main/attempt_001/outcome.json',dict(result='no_progress',valid=True,bag_verified=True))
    monkeypatch.setattr(experiment,'execute',lambda *args:pytest.fail('Should skip completed trials'))
    assert experiment.run(tmp_path,manifest)==0
    assert read(tmp_path/'status.json')['completed']==1


def test_background_lock_is_inherited_and_duplicate_start_rejected(tmp_path,monkeypatch):
    original=subprocess.Popen
    children=[]
    child_code='import os,time; f=int(os.environ["NOMAD_EXPERIMENT_LOCK_FD"]); os.fstat(f); time.sleep(2)'
    def spawn(command,**kwargs):
        child=original([sys.executable,'-c',child_code],**kwargs);children.append(child);return child
    monkeypatch.setattr(comparison.subprocess,'Popen',spawn)
    try:
        assert comparison.background(tmp_path,{})==0
        with pytest.raises(RuntimeError,match='이미 실행'):comparison.background(tmp_path,{})
        assert read(tmp_path/'status.json')['state']=='starting'
        for child in children:assert child.wait(timeout=5)==0
        with (tmp_path/'run.lock').open('a') as f:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
    finally:
        for child in children:
            if child.poll() is None:child.terminate();child.wait()


@pytest.mark.parametrize('valid',[True,False])
def test_comparison_gate_distinguishes_performance_from_data_failure(tmp_path,monkeypatch,valid):
    pilot=dict(id='pilot',pilot=True)
    main=dict(id='main',pilot=False)
    manifest=dict(pilots=[pilot],trials=[main],pilot_requires_success=False)
    calls=[]
    def execute(directory,manifest,trial):
        calls.append(trial['id'])
        outcome=dict(result='timeout' if valid else 'recording_incomplete',
                     valid=valid,bag_verified=valid,archive_verified=valid)
        save(directory/'runs'/trial['id']/'attempt_001/outcome.json',outcome)
        return outcome
    monkeypatch.setattr(experiment,'execute',execute)
    assert experiment.run(tmp_path,manifest)==(0 if valid else 2)
    assert calls==(['pilot','main'] if valid else ['pilot'])
    if valid:
        calls.clear()
        assert experiment.run(tmp_path,manifest)==0
        assert calls==[]
        state=comparison.status(tmp_path,manifest,experiment.finished,experiment.recover_interrupted)
        assert state['pilots_successes']==0 and state['pilots_performance_failures']==1
        assert state['completed']==1 and state['performance_failures']==1
