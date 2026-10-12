"""Saved manual waypoints and background lifecycle for the existing runner."""
import csv
import fcntl
import math
import os
from pathlib import Path
import random
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from common import ROOT, WORKSPACE, read, save, sha

DEFAULT_ROUTE = 'manual_map4_waypoints_20261008'


def load_route(source=DEFAULT_ROUTE, repeats=3, selected=None, pair_count=6):
    if not re.fullmatch(r'[\w-]{1,80}', source):
        raise ValueError('경유점 기록은 experiments 안의 폴더명으로 지정하세요.')
    selected = list(range(1, pair_count+1)) if selected is None else selected
    if not 1 <= repeats <= 20 or not selected or len(set(selected)) != len(selected):
        raise ValueError('반복은 1~20회, 조합 번호는 중복 없이 선택하세요.')
    if any(n < 1 or n > pair_count for n in selected):
        raise ValueError('알고리즘 조합 번호가 잘못되었습니다.')
    folder = ROOT/source
    checks = read(folder/'checksums.json')
    for name in ('waypoints.csv', 'mission_template.json', 'metadata.json'):
        if checks.get(name) != sha(folder/name):
            raise ValueError(f'원본 경유점 체크섬 불일치: {name}')
    meta, mission = read(folder/'metadata.json'), read(folder/'mission_template.json')
    with (folder/'waypoints.csv').open() as f:
        rows = list(csv.DictReader(f))
    goals = []
    for i, row in enumerate(rows, 1):
        if int(row['sequence']) != i or row['frame_id'] != 'odom':
            raise ValueError('경유점 순서/좌표계가 잘못되었습니다.')
        point = [float(row['x_m']), float(row['y_m'])]
        if not all(map(math.isfinite, point)):
            raise ValueError('경유점 좌표는 유한한 값이어야 합니다.')
        goals.append(point)
    if (not goals or len(goals) != meta['waypoint_count'] or goals != mission['waypoints']
            or meta['map'] != '04_vio_flat_loop' or mission['map'] != meta['map']):
        raise ValueError('4번 맵 경유점 기록이 일치하지 않습니다.')
    world = WORKSPACE/'src/nomad_gazebo/maps'/meta['map']/'worlds/forest.sdf'
    pose = ET.parse(world).getroot().find("world/model[@name='nomad_vehicle']/pose")
    if pose is None or len(pose.text.split()) != 6 or len(meta['nominal_world_spawn_xyz_rpy']) != 6 or any(abs(a-b)>1e-6 for a,b in zip(
            map(float, pose.text.split()), meta['nominal_world_spawn_xyz_rpy'])):
        raise ValueError('맵 출발 자세가 수동 경유점 기록과 다릅니다.')
    return dict(source=source, repeats=repeats, selected=selected, goals=goals,
                nominal_spawn=meta['nominal_world_spawn_xyz_rpy'])


def configure(directory, manifest, plan, conditions):
    """Use the same run_trial goal progression and metrics as every other mission."""
    folder = ROOT/plan['source']
    dest = directory/'route_input';dest.mkdir()
    for name in ('waypoints.csv', 'mission_template.json', 'metadata.json', 'checksums.json'):
        shutil.copy2(folder/name, dest/name)
    chosen = [p for p in conditions if p['number'] in plan['selected']]
    trials = []
    for repeat in range(1, plan['repeats']+1):
        block = [dict(id=f'04_manual_pair{p["number"]}_r{repeat}', map='04_vio_flat_loop',
                      pair=p, repeat=repeat, pilot=False, mission='ordered_route',
                      goal=plan['goals'][0], waypoints=plan['goals'], waypoint_radius_m=1.,
                      waypoints_per_lap=len(plan['goals']), laps=1, timeout_sim_s=1200.,
                      route_source='route_input/waypoints.csv: original manual clicks, including corrections')
                 for p in chosen]
        random.Random(740+repeat).shuffle(block);trials.extend(block)
    manifest.update(scope='map4_manual_waypoints', comparison=plan, trials=trials,
                    pilots=[dict(id=f'pilot_pair{p["number"]}', map='04_vio_flat_loop',
                                 pair=p, repeat=1, goal=None, timeout_sim_s=15., pilot=True)
                            for p in chosen],
                    references={'04_vio_flat_loop':dict(validated=False,
                        reason='Manual clicked polyline is not a shortest feasible path or minimum-time reference.')},
                    route_files={str(p.relative_to(directory)):sha(p) for p in dest.iterdir()},
                    background_default=True, auto_evaluate=True, pilot_requires_success=False)
    manifest['limits']['trial_wall_s'] = 14400.
    manifest['notes'] += ['Original click times are not replayed; advance within 1m, final goal within 0.35m and stopped.',
                          'All 43 clicks including corrections are retained; this is not the historic driven trajectory.',
                          'Simulator grass oracle enabled equally across pairs; this does not evaluate learned grass recognition.']


def background(directory, manifest, pilot_only=False):
    """Pass an already-held flock into the child: no duplicate-start race."""
    if (directory/'CANCELLED.json').exists():
        raise ValueError('사용자가 취소한 실험입니다. 새 실험명을 사용하세요.')
    lock = (directory/'run.lock').open('a')
    try:
        try:fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('이미 실행 중인 실험입니다.')
        (directory/'STOP_REQUESTED').unlink(missing_ok=True)
        env = dict(os.environ, NOMAD_EXPERIMENT_LOCK_FD=str(lock.fileno()))
        command = [sys.executable, str(WORKSPACE/'src/nomad_bringup/experiments/experiment.py'),
                   'resume', directory.name, '--foreground']
        if pilot_only:command.append('--pilot-only')
        save(directory/'status.json', dict(state='starting', started=time.time()))
        with (directory/'background.log').open('a') as log:
            child = subprocess.Popen(command, env=env, cwd=WORKSPACE, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                     pass_fds=(lock.fileno(),))
        save(directory/'runner.json', dict(pid=child.pid, started=time.time(), command=command))
        print(f'백그라운드 시작: {directory.name} (PID {child.pid})\n'
              f'진행: ./scripts/run_experiment.sh status {directory.name} --watch\n'
              f'로그: {directory / "background.log"}')
    except Exception as error:
        if not isinstance(error, RuntimeError) or str(error) != '이미 실행 중인 실험입니다.':
            save(directory/'status.json', dict(state='blocked', reason=str(error), retry='resume'))
        raise
    finally:
        # Do not explicitly unlock: the inherited file description owns the lock.
        lock.close()
    return 0


def status(directory, manifest, finished, recover_interrupted):
    with (directory/'run.lock').open('a') as lock:
        active = False
        try:fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:active = True
        state = read(directory/'status.json')
        if not active and state.get('state') in ('starting','running','finalizing','evaluating'):
            recover_interrupted(directory)
            state = dict(state='interrupted', retry='resume', reason='실행 프로세스가 종료되었습니다.')
            save(directory/'status.json', state)
    completed = [(t, finished(directory,t)) for t in manifest['trials']]
    outcomes = [o for t,o in completed if o is not None]
    pilot_outcomes = [finished(directory,t) for t in manifest['pilots']]
    result = dict(state, worker_active=active, completed=len(outcomes), total=len(completed),
                  successes=sum(o['result']=='success' for o in outcomes),
                  performance_failures=sum(o['result']!='success' for o in outcomes),
                  pilots_completed=sum(o is not None for o in pilot_outcomes),
                  pilots_successes=sum(o is not None and o['result']=='success' for o in pilot_outcomes),
                  pilots_performance_failures=sum(o is not None and o['result']!='success' for o in pilot_outcomes),
                  pilots_total=len(manifest['pilots']))
    current = next((t for t in manifest['pilots']+manifest['trials'] if t['id']==state.get('trial')), None)
    if current:result.update(pair=current['pair'], repeat=current['repeat'], pilot=current['pilot'])
    if state.get('attempt'):
        live = directory/state['attempt']/'live.json'
        if live.exists():
            result['live'] = read(live)
            result['live_age_wall_s'] = round(time.time()-result['live']['updated_wall_s'],1)
    return result
