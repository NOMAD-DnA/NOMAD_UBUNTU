"""One isolated trial. Own-process cleanup only; queue never overlaps runs."""
import argparse
import hashlib
import fcntl
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
from .core import PACKAGE, scenario, save, read, summarize, ALGORITHM, workspace, strategy_timings
from .worlds import build, viewer_config


def process_identity(pid):
    try:return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (OSError,IndexError):return None


def group_members(pgid):
    members=[]
    for p in Path('/proc').glob('[0-9]*'):
        try:
            f=(p/'stat').read_text().rsplit(')',1)[1].split()
            if int(f[2])==pgid and f[0]!='Z':members.append(int(p.name))
        except (OSError,ValueError,IndexError):pass
    return members


def cleanup(processes):
    # Sessions were created by this worker and recorded before any signal.
    groups=[p.pid for p in processes]
    for sig,seconds in [(signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,1)]:
        for group in groups:
            if group_members(group):
                try:os.killpg(group,sig)
                except ProcessLookupError:pass
        end=time.monotonic()+seconds
        while time.monotonic()<end and any(group_members(g) for g in groups):time.sleep(.1)
        for p in processes:p.poll()
    remaining={str(g):group_members(g) for g in groups if group_members(g)}
    return remaining


def check_conflicts(domain,partition):
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name)==os.getpid():continue
        try:
            if p.stat().st_uid!=os.getuid():continue
            args=(p/'cmdline').read_bytes().decode(errors='replace').split('\0')
            names={Path(a).name for a in args};title=args[0].strip() if args else ''
            relevant=bool(names & {'dstar_lite_gpp','ackermann_lpp','planning_command','controller','parameter_bridge'}) or title.startswith('gz sim server') or ('-s' in args and any(Path(a).name=='gz' for a in args))
            if not relevant:continue
            env=dict(v.split(b'=',1) for v in (p/'environ').read_bytes().split(b'\0') if b'=' in v)
            if env.get(b'ROS_DOMAIN_ID')==str(domain).encode() or env.get(b'GZ_PARTITION')==partition.encode():raise RuntimeError(f'Existing module PID {p.name} in test domain/partition; stop it explicitly first')
        except (OSError,ValueError):pass


def executable(package,name):return str(Path(get_package_prefix(package))/'lib'/package/name)


def algorithm_config(algorithm):
    from nomad_path_planning.gpp.registry import NAMES as GPP_NAMES
    from nomad_path_planning.lpp.registry import NAMES as LPP_NAMES
    parts=algorithm.split('+')
    if len(parts)!=2 or parts[0] not in GPP_NAMES or parts[1] not in LPP_NAMES:
        raise ValueError(f'Unsupported algorithm combination: {algorithm}')
    config=yaml.safe_load((PACKAGE/'config.yaml').read_text())
    for node in ('dstar_lite_gpp','ackermann_rollout_lpp'):
        config[node]['ros__parameters'].update(gpp_algorithm=parts[0],lpp_algorithm=parts[1])
    return config


def run(number,directory,timeout=180,algorithm=ALGORITHM):
    config=algorithm_config(algorithm)
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    domain=int(os.environ.get('ROS_DOMAIN_ID','142'));partition=os.environ['GZ_PARTITION']
    lock=open(f'/tmp/nomad-planning-lab-{os.getuid()}-domain-{domain}.lock','a')
    processes=[];streams=[];reason=None
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);check_conflicts(domain,partition)
        world=build(number,directory/'map');viewer_config(world,directory/'viewer.config')
        spec=scenario(number)
        source_files = list(PACKAGE.glob('*.py')) + [PACKAGE/'authoring.json', PACKAGE/'config.yaml']
        for package in ('nomad_path_planning', 'nomad_control'):
            source_files.extend((workspace()/'src'/package).rglob('*.py'))
        source_hashes={str(p.relative_to(workspace())):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
        (directory/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        save(directory/'manifest.json',dict(schema_version=1,scenario=spec,algorithm=algorithm,timeout_sim_s=timeout,ros_domain=domain,gz_partition=partition,world=str(world),input_mode='GT observation; no perception/localization nodes',source_sha256=source_hashes))
        env=dict(os.environ);env.pop('NOMAD_RUN_LOG_DIR',None);env['NOMAD_EXPERIMENT_TIMING_DIR']=str(directory/'timing')
        env['ROS_LOG_DIR']=str(directory/'ros_logs')
        env['GZ_SIM_RESOURCE_PATH']=str(world.parent)+':'+env.get('GZ_SIM_RESOURCE_PATH','')
        cfg=str(directory/'config.yaml');pids={}
        def start(name,cmd):
            stream=(directory/(name+'.log')).open('w');streams.append(stream)
            p=subprocess.Popen(cmd,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(p);pids[name]=p.pid;save(directory/'pids.json',pids)
            save(directory/'ownership.json',[{'pid':q.pid,'pgid':q.pid,'starttime':process_identity(q.pid)} for q in processes])
            return p
        start('gazebo',['gz','sim','-s','-r','-v','2',str(world)])
        bridge=[]
        for ros,gz,rt,gt,direction in [('/clock','/world/nomad_forest/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS'),('/odom','/nomad/vehicle/odom','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),('/joint_states','/nomad/vehicle/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS'),('/nomad/vehicle/cmd_vel','/nomad/vehicle/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ')]:
            bridge.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction=direction))
        bp=directory/'bridge.yaml';bp.write_text(yaml.safe_dump(bridge))
        start('bridge',[executable('ros_gz_bridge','parameter_bridge'),'--ros-args','-p',f'config_file:={bp}','-p','use_sim_time:=true'])
        for name,pkg,exe in [('watchdog','nomad_gazebo','command_watchdog.py'),('control','nomad_control','controller'),('selector','nomad_path_planning','planning_command'),('gpp','nomad_path_planning','dstar_lite_gpp'),('lpp','nomad_path_planning','ackermann_lpp')]:
            start(name,[executable(pkg,exe),'--ros-args','--params-file',cfg])
        worker=start('recorder',[sys.executable,'-m','nomad_bringup.scenario_lab.worker',str(directory),'--ros-args','-p','use_sim_time:=true'])
        deadline=time.monotonic()+max(120,timeout*4)
        save(directory/'runner.json',dict(state='running',partition=partition,domain=domain))
        while worker.poll() is None:
            if (directory/'STOP').exists():
                try:worker.send_signal(signal.SIGINT)
                except ProcessLookupError:pass
                reason='cancelled';break
            failed=[name for name,pid in pids.items() if name!='recorder' and next(q for q in processes if q.pid==pid).poll() is not None]
            if failed:reason='process_failure: '+','.join(failed);break
            if time.monotonic()>deadline:reason='wall_timeout';break
            time.sleep(.2)
        if worker.poll() is not None and worker.returncode and not reason:reason='recorder_failure'
    except (Exception,KeyboardInterrupt) as exc:
        reason='cancelled' if isinstance(exc,KeyboardInterrupt) else 'startup_error: '+str(exc)
    finally:
        remaining=cleanup(processes)
        for f in streams:f.close()
        outcome=read(directory/'outcome.json')
        if outcome is None:outcome=summarize([],reason or 'worker_missing_result',scenario(number),0,0,{},None)
        if reason and outcome.get('result') not in ('cancelled',):outcome.update(result=reason,completed=False,expected_result_met=False)
        outcome['algorithm']=algorithm
        outcome['strategy_timing']=strategy_timings(directory)
        outcome['cleanup_ok']=not remaining;outcome['remaining_owned_processes']=remaining
        if remaining:outcome.update(result='cleanup_failed',expected_result_met=False)
        save(directory/'outcome.json',outcome);save(directory/'runner.json',dict(state='finished' if not remaining else 'cleanup_failed'))
        lock.close()
    return outcome


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',type=int,choices=range(1,7),required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--timeout',type=float,default=180);parser.add_argument('--algorithm',default=ALGORITHM);a=parser.parse_args()
    if a.timeout<=0:parser.error('timeout must be positive')
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    result=run(a.scenario,a.output,a.timeout,a.algorithm);print(json.dumps(result,ensure_ascii=False));return 0 if result.get('cleanup_ok') else 2

if __name__=='__main__':sys.exit(main())
