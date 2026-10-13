"""Stop previous processes positively associated with this workspace and domain."""
import os
import signal
import time
from pathlib import Path
from process_guard import resource_set


PACKAGES={'nomad_gazebo','nomad_bringup','nomad_perception','nomad_vio','nomad_localization','nomad_path_planning','nomad_control'}


def owned(args,env,workspace):
    root=str(Path(workspace).resolve())
    # A generic ROS/Python name or NOMAD package name alone is never sufficient.
    if env.get('NOMAD_WS_ROOT')!=root:return False
    project_path=any(arg.startswith(root+'/') for arg in args)
    if resource_set(args) and project_path:return True
    if 'launch' in args and set(args)&PACKAGES and resource_set(args):return True
    title=args[0].strip() if args else ''
    if title in ('gz sim server','gz sim gui'):return True
    if any(Path(arg).name=='rviz2' for arg in args):
        return project_path or any(arg.startswith('/tmp/nomad-gui-options-') for arg in args)
    return False


def snapshot(proc=Path('/proc')):
    result={}
    for folder in proc.iterdir():
        if not folder.name.isdigit():continue
        try:
            if folder.stat().st_uid!=os.getuid():continue
            stat=folder.joinpath('stat').read_text().rsplit(')',1)[1].split()
            args=folder.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
            # Read only the ownership markers; never log process environments.
            env={}
            for pair in folder.joinpath('environ').read_bytes().split(b'\0'):
                key,sep,value=pair.partition(b'=')
                if sep and key in (b'NOMAD_WS_ROOT',b'ROS_DOMAIN_ID'):env[key.decode()]=value.decode(errors='replace')
            result[int(folder.name)]={'parent':int(stat[1]),'start':stat[19],'state':stat[0],'args':args,'env':env}
        except (OSError,ValueError,IndexError):continue
    return result


def select_targets(processes,workspace,domain):
    roots={pid for pid,p in processes.items() if p['env'].get('ROS_DOMAIN_ID')==str(domain) and owned(p['args'],p['env'],workspace)}
    targets=set(roots)
    while True:
        descendants={pid for pid,p in processes.items() if p['parent'] in targets and p['env'].get('NOMAD_WS_ROOT')==str(Path(workspace).resolve()) and p['env'].get('ROS_DOMAIN_ID')==str(domain)}
        expanded=targets|descendants
        if expanded==targets:break
        targets=expanded
    targets.discard(os.getpid())
    return {pid:processes[pid]['start'] for pid in targets}


def remaining(targets):
    alive={}
    for pid,start in targets.items():
        try:
            stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
            if stat[19]==start and stat[0]!='Z':alive[pid]=start
        except (OSError,IndexError):pass
    return alive


def cleanup(workspace,domain,waits=(3.,2.,1.)):
    targets=select_targets(snapshot(),workspace,domain)
    if not targets:return 0
    print(f'[GUI 준비] 이전 NOMAD 실행 {len(targets)}개 종료 요청 (domain {domain})',flush=True)
    alive=targets
    for sig,seconds in zip((signal.SIGINT,signal.SIGTERM,signal.SIGKILL),waits):
        for pid in remaining(alive):
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            alive=remaining(alive)
            if not alive:break
            time.sleep(.05)
        if not alive:break
    if remaining(alive):raise RuntimeError('이전 NOMAD 실행을 종료하지 못했습니다. GUI를 열지 않습니다.')
    print('[GUI 준비] 이전 NOMAD 실행 정리 완료',flush=True)
    return len(targets)


if __name__=='__main__':
    root=Path(__file__).resolve().parents[3]
    cleanup(root,os.environ.get('ROS_DOMAIN_ID','42'))
