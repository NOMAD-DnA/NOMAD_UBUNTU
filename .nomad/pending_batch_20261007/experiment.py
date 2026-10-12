#!/usr/bin/env python3
"""Thin experiment orchestrator around NOMAD's existing launcher and rosbag2."""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import random
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid
import yaml
from common import ROOT, WORKSPACE, MAPS, save, read, fingerprint, make_world, goal_for


def pairs():
    file=WORKSPACE/'src/nomad_path_planning/scripts/select_algorithms.py'
    spec=importlib.util.spec_from_file_location('nomad_pair_menu',file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.PAIRS


def locate(name):
    ROOT.mkdir(exist_ok=True)
    if not name:
        names=sorted(p.name for p in ROOT.iterdir() if (p/'manifest.json').is_file())
        print('\n'.join(f'{i}. {n}' for i,n in enumerate(names,1)))
        if not names:raise ValueError('저장된 실험이 없습니다. record 명령으로 생성하세요.')
        number=input('실험 번호: ').strip()
        if not number.isdigit() or not 1<=int(number)<=len(names):raise ValueError('잘못된 번호')
        name=names[int(number)-1]
    if not re.fullmatch(r'[\w-]{1,80}',name):raise ValueError('실험명은 문자·숫자·밑줄·하이픈만 사용하세요.')
    return ROOT/name


def initialize(directory, route_pair=None, all_maps=False, exclude_map3=False):
    if (directory/'manifest.json').exists():return read(directory/'manifest.json')
    if directory.exists():raise ValueError('불완전한 실험 폴더입니다. 새 실험명을 사용하세요.')
    directory.mkdir(parents=True)
    files=fingerprint()
    for relative in files:
        dest=directory/'snapshot'/relative
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(WORKSPACE/relative,dest)
    if fingerprint()!=files:raise RuntimeError('스냅샷 생성 중 코드가 변경됐습니다. 새 실험명으로 재시도하세요.')
    subprocess.run(['git','diff','--binary','HEAD'],cwd=WORKSPACE,stdout=(directory/'source.patch').open('w'),check=True)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WORKSPACE,text=True).strip()
    conditions=[dict(number=i,label=label,gpp=gpp,lpp=lpp) for i,(label,gpp,lpp) in enumerate(pairs(),1)]
    trials=[]
    for repeat in range(1,4):
        block=[]
        for mid in MAPS:
            for pair in conditions:
                block.append(dict(id=f"{mid}_pair{pair['number']}_r{repeat}",map=mid,pair=pair,
                                  repeat=repeat,goal=goal_for(mid),timeout_sim_s=600.,pilot=False))
        random.Random(740+repeat).shuffle(block)
        trials.extend(block)
    pilots=[dict(id=f"pilot_pair{p['number']}",map=MAPS[0],pair=p,repeat=1,goal=None,timeout_sim_s=15.,pilot=True) for p in conditions]
    if route_pair is not None:
        from route_mission import loop_waypoints
        mid = MAPS[0]
        waypoints, per_lap = loop_waypoints(directory/'snapshot/src/nomad_gazebo/maps'/mid/'route.csv')
        pair = conditions[route_pair-1]
        pilots = []
        trials = [dict(id=f'{mid}_route_pair{route_pair}_r1', map=mid, pair=pair,
            repeat=1, goal=waypoints[0], timeout_sim_s=1200., pilot=False,
            waypoints=waypoints, waypoint_radius_m=1., waypoints_per_lap=per_lap,
            mission='ordered_route', laps=2,
            route_source='route.csv: design guidance, not shortest-path ground truth')]
    if all_maps:
        from batch_missions import ALL_MAPS, mission
        selected_maps = [m for m in ALL_MAPS if not (exclude_map3 and m.startswith('03'))]
        trials = []
        map_root = directory/'snapshot/src/nomad_gazebo/maps'
        for repeat in range(1, 4):
            block = []
            for mid in selected_maps:
                for pair in conditions:
                    block.append(dict(mission(map_root, mid), id=f"{mid}_pair{pair['number']}_r{repeat}",
                                      map=mid, pair=pair, repeat=repeat))
            random.Random(740+repeat).shuffle(block)
            trials.extend(block)
    manifest=dict(schema=1,name=directory.name,created=time.time(),commit=commit,files=files,
                  weights=dict(sr=.5,spl=.25,sct=.25),trials=trials,pilots=pilots,
                  limits=dict(startup_wall_s=120.,trial_wall_s=7200.,min_free_gb=8.),
                  safety=dict(goal_radius_m=.35,stop_speed_mps=.05,stop_hold_sim_s=1.,rollover_deg=60.,rollover_hold_sim_s=.5),
                  references={mid:dict(validated=False,reason='Independent shortest-distance and dynamics-aware minimum-time references not verified; route.csv is not ground truth.') for mid in MAPS},
                  notes=['Simulation only; no proof of real-vehicle performance.',
                         'Default perception, control and recovery fixed; map and planner pair vary.',
                         'Field D* is a custom variant plus a Hybrid A* reference adapter.',
                         'Safe success is used in SPL/SCT; distinguish from arrival-only literature metrics.'])
    if route_pair is not None or all_maps:
        manifest['limits']['trial_wall_s'] = 14400.
    manifest['scope'] = 'all_maps_reference_routes' if all_maps else ('map4_route' if route_pair is not None else 'legacy_54')
    manifest['references'] = {mid: dict(validated=False, reason='Mission design route is not shortest-distance/minimum-time ground truth.') for mid in sorted({t['map'] for t in trials})}
    manifest['recording'] = dict(scope='all discovered non-image ROS topics including hidden topics', excluded_types=['sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage'], compression='MCAP Zstd fastest lossless', split_bytes=536870912, timing='per algorithm call wall and thread CPU ns; excludes recorder write overhead')
    save(directory/'manifest.json',manifest)
    import platform
    environment=dict(platform=platform.platform(),python=sys.version,cpu_count=os.cpu_count(),
        ros={k:v for k,v in os.environ.items() if k in ('ROS_DISTRO','RMW_IMPLEMENTATION','ROS_AUTOMATIC_DISCOVERY_RANGE')},
        purpose='Non-image data collection; no automatic plots or Notion publication')
    save(directory/'environment.json',environment)
    save(directory/'status.json',dict(state='created',total=len(manifest['trials']),completed=0))
    return manifest


def assert_version(manifest):
    current=fingerprint()
    if current!=manifest['files']:
        changed=sorted(k for k in set(current)|set(manifest['files']) if current.get(k)!=manifest['files'].get(k))
        raise RuntimeError('코드·맵·설정 변경 감지: 기존 결과에 섞지 않습니다. 새 실험명을 사용하세요. '+', '.join(changed[:8]))


def recover_interrupted(directory):
    # A hard process interruption can skip finally; preserve its files and mark it.
    for attempt in (directory/'runs').glob('*/attempt_*'):
        if not (attempt/'outcome.json').exists():
            save(attempt/'outcome.json',dict(result='interrupted',valid=False,bag_verified=False,
                 error='Previous process ended before trial finalization; raw files preserved.'))


def finished(directory,trial):
    files=sorted((directory/'runs'/trial['id']).glob('attempt_*/outcome.json'))
    for file in reversed(files):
        outcome=read(file)
        if outcome.get('valid') and outcome.get('bag_verified'):return outcome
    return None


def check_domain(domain):
    if domain in (0,42,43) or not 1<=domain<=220:raise ValueError('Use an isolated experiment domain, not 0/42/43')
    import rclpy
    rclpy.init()
    node=rclpy.create_node('nomad_experiment_domain_probe')
    try:
        until=time.monotonic()+2.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
        others=[n for n in node.get_node_names() if n!=node.get_name()]
        if others:raise RuntimeError(f'Experiment domain {domain} already contains nodes: {others}')
    finally:node.destroy_node();rclpy.shutdown()


def stop_owned(processes):
    # Sessions belong only to Popen(start_new_session=True) calls in this trial.
    for process in reversed(processes):
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:continue
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                process.wait(timeout=5)
        # Gazebo wrappers can exit before their own children.
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass


def execute(directory,manifest,trial):
    assert_version(manifest)
    if shutil.disk_usage(directory).free<manifest['limits']['min_free_gb']*1024**3:
        raise RuntimeError('Less than reserved free disk space; recording paused')
    domain=int(os.environ.get('NOMAD_EXPERIMENT_DOMAIN','186'))
    os.environ['ROS_DOMAIN_ID']=str(domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    check_domain(domain)
    base=directory/'runs'/trial['id'];base.mkdir(parents=True,exist_ok=True)
    attempt=base/f'attempt_{len(list(base.glob("attempt_*")))+1:03d}';attempt.mkdir()
    env=dict(os.environ)
    partition='nomad_experiment_'+uuid.uuid4().hex
    env.update(NOMAD_ROS_DOMAIN_ID=str(domain),NOMAD_GZ_PARTITION=partition,GZ_PARTITION=partition)
    # Python implementations are loaded from the saved snapshot. The unchanged
    # launcher is version-checked before/after each run as are SDFs and configs.
    package_dirs=[str(p) for p in (directory/'snapshot/src').iterdir() if p.is_dir()]
    env['PYTHONPATH']=os.pathsep.join(package_dirs+[env.get('PYTHONPATH','')])
    env['ROS_LOG_DIR']=str(attempt/'ros_logs')
    env['NOMAD_EXPERIMENT_TIMING_DIR']=str(attempt/'timing')
    world_source=directory/'snapshot/src/nomad_gazebo/maps'/trial['map']/'worlds/forest.sdf'
    contact_topics=make_world(world_source,attempt/'world.sdf')
    bridge=[dict(ros_topic_name=t,gz_topic_name=t,ros_type_name='ros_gz_interfaces/msg/Contacts',
                 gz_type_name='gz.msgs.Contacts',direction='GZ_TO_ROS') for t in contact_topics]
    (attempt/'contacts.yaml').write_text(yaml.safe_dump(bridge))
    from nomad_bringup.configuration import load_topics
    topic_map=load_topics(directory/'snapshot/src/nomad_bringup/config/topics.yaml')
    keys=[k for k in topic_map if k.startswith(('planning.','control.','localization.','perception.'))]
    record_topics=sorted(set([topic_map[k] for k in keys]+[topic_map['sensors.odometry'],'/clock','/tf','/tf_static']+contact_topics))
    latched=[topic_map[k] for k in ('planning.goal','planning.global_path','perception.global_costmap','perception.local_costmap','planning.recovery_gates')]+['/tf_static']
    (attempt/'qos.yaml').write_text(yaml.safe_dump({t:dict(durability='transient_local',reliability='reliable',history='keep_last',depth=1) for t in latched}))
    cfg=directory/'snapshot/src/nomad_bringup/config'
    commands=[
        ['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-r','__node:=experiment_contacts','-p',f'config_file:={attempt/"contacts.yaml"}'],
        ['ros2','bag','record','--node-name','experiment_bag','--disable-keyboard-controls','--use-sim-time','-s','mcap','-o',str(attempt/'bag'),
         '--qos-profile-overrides-path',str(attempt/'qos.yaml'), '--storage-preset-profile','zstd_fast',
         '--max-bag-size','536870912','--all-topics','--include-hidden-topics',
         '--exclude-topic-types','sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage'],
        ['ros2','launch','nomad_bringup','forest.launch.py','headless:=true','rviz:=false',f'world_file:={attempt/"world.sdf"}',
         f'gpp:={trial["pair"]["gpp"]}',f'lpp:={trial["pair"]["lpp"]}',f'topics_file:={cfg/"topics.yaml"}',
         f'modules_file:={cfg/"modules.yaml"}',f'vehicle_file:={cfg/"vehicle.yaml"}']]
    metadata=dict(trial=trial,domain=domain,partition=partition,commands=commands,topics=record_topics,commit=manifest['commit'],started=time.time())
    save(attempt/'metadata.json',metadata)
    processes=[];streams=[];outcome=dict(result='interrupted',valid=False)
    try:
        for name,command in zip(('contacts','bag','launch'),commands):
            stream=(attempt/f'{name}.log').open('w');streams.append(stream)
            processes.append(subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,cwd=WORKSPACE))
        save(directory/'status.json',dict(state='running',trial=trial['id'],attempt=str(attempt.relative_to(directory)),pid=os.getpid(),started=time.time()))
        from observe import run_trial
        outcome=run_trial(attempt,manifest,trial,contact_topics,processes)
    except KeyboardInterrupt:
        outcome=dict(result='interrupted',valid=False)
        raise
    except Exception as error:
        outcome=dict(result='infrastructure_error',valid=False,error=str(error))
    finally:
        stop_owned(processes)
        for stream in streams:stream.close()
        try:
            assert_version(manifest)
        except RuntimeError as error:
            outcome.update(valid=False,result='source_changed',error=str(error))
        bag_verified=False
        try:
            info=yaml.safe_load((attempt/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
            counts={x['topic_metadata']['name']:x['message_count'] for x in info['topics_with_message_count']}
            bag_verified=all(counts.get(t,0)>0 for t in [topic_map['sensors.odometry'],'/clock',topic_map['planning.goal']]) and sum(counts.get(t,0) for t in contact_topics)>0
            save(attempt/'bag_counts.json',counts)
        except (OSError,KeyError,TypeError):pass
        outcome['bag_verified']=bag_verified
        if outcome.get('valid') and not bag_verified:outcome.update(valid=False,result='recording_incomplete')
        if bag_verified:
            from archive import verify
            try:
                audit=verify(attempt)
                outcome['archive_verified']=audit['valid']
                if not audit['valid']:outcome.update(valid=False,result='archive_verification_failed')
            except Exception as error:
                outcome.update(valid=False,result='archive_verification_failed',archive_error=str(error))
        save(attempt/'outcome.json',dict(outcome,ended=time.time()))
        from archive import checksum
        checksum(attempt)
    print(trial['id'],outcome['result'],flush=True)
    return outcome


def run(directory,manifest,pilot_only=False):
    with (directory/'run.lock').open('w') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('이미 실행 중인 실험입니다.')
        try:
            recover_interrupted(directory)
            for trial in manifest['pilots']+( [] if pilot_only else manifest['trials']):
                if (directory/'STOP_REQUESTED').exists():
                    save(directory/'status.json',dict(state='interrupted',retry='resume'))
                    return 130
                previous=finished(directory,trial)
                if previous:continue
                outcome=execute(directory,manifest,trial)
                save(directory/'progress.json',dict(completed=sum(finished(directory,t) is not None for t in manifest['trials']),total=len(manifest['trials']),last_trial=trial['id'],last_result=outcome['result']))
                if not outcome.get('valid'):
                    save(directory/'status.json',dict(state='blocked',trial=trial['id'],reason=outcome['result'],retry='resume'))
                    print('Small-test/data-quality gate stopped the batch. Inspect show/status, then resume.',flush=True)
                    return 2
            save(directory/'status.json',dict(state='pilot_complete' if pilot_only else 'complete',total=len(manifest['trials']),completed=sum(finished(directory,t) is not None for t in manifest['trials'])))
        except KeyboardInterrupt:
            save(directory/'status.json',dict(state='interrupted',retry='resume'))
            return 130
        except Exception as error:
            save(directory/'status.json',dict(state='blocked',reason=str(error),retry='resume'))
            raise
    return 0


def main():
    parser=argparse.ArgumentParser(description='NOMAD experiment: record/evaluate/show/status/resume')
    parser.add_argument('command',choices=['record','evaluate','show','status','resume','prepare','export','route','batch','stop'])
    parser.add_argument('name',nargs='?')
    parser.add_argument('--pilot-only',action='store_true')
    parser.add_argument('--topic',help='ROS topic for lossless JSONL export, e.g. /odom')
    parser.add_argument('--pair', type=int, choices=range(1,7), default=3, help='route mission planner pair (default 3)')
    parser.add_argument('--exclude-map3', action='store_true', help='Exclude map3 from the batch')
    args=parser.parse_args()
    name=args.name
    if args.command in ('record','prepare','route','batch') and not name:name=input('새 실험명: ').strip()
    directory=locate(name)
    if args.command in ('record','prepare','route','batch'):manifest=initialize(directory, args.pair if args.command=='route' else None, args.command=='batch', args.exclude_map3)
    else:manifest=read(directory/'manifest.json')
    if args.command=='route' and any(t.get('mission')!='ordered_route' or t['pair']['number']!=args.pair for t in manifest['trials']):
        raise ValueError('기존 실험 조건과 다릅니다. 새 실험명을 사용하세요.')
    if args.command=='batch' and manifest.get('scope')!='all_maps_reference_routes':
        raise ValueError('다른 범위의 기존 실험입니다. 새 실험명을 사용하세요.')
    if args.command=='prepare':print(directory);return 0
    if args.command=='stop':
        (directory/'STOP_REQUESTED').touch()
        print('중단 요청을 저장했습니다. 기록 파일 정리를 기다리세요.')
        return 0
    if args.command=='export':
        if not args.topic:raise ValueError('export requires --topic /topic_name')
        from archive import export
        export(directory,args.topic)
        return 0
    if args.command in ('record','resume','route','batch'):
        if (directory/'CANCELLED.json').exists():
            raise ValueError('사용자가 취소한 실험입니다. 새 실험명을 사용하세요.')
        if args.command=='resume':
            (directory/'STOP_REQUESTED').unlink(missing_ok=True)
        return run(directory,manifest,args.pilot_only)
    if args.command in ('evaluate','show'):
        from report import evaluate
        evaluate(directory)
        print(directory/'results/report.html')
    if args.command=='status':
        with (directory/'run.lock').open('a') as lock:
            try:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                if read(directory/'status.json').get('state')=='running':
                    recover_interrupted(directory)
                    save(directory/'status.json',dict(state='interrupted',retry='resume'))
            except BlockingIOError:pass
        print(json.dumps(read(directory/'status.json'),ensure_ascii=False,indent=2))
        print(f"본 실험 완료: {sum(finished(directory,t) is not None for t in manifest['trials'])}/{len(manifest['trials'])}")
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,RuntimeError,FileNotFoundError) as error:
        print(str(error),file=sys.stderr);sys.exit(2)
