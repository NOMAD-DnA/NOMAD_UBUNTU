"""Development-only lifecycle checks. No sensors or physical Jetson are accessed."""
import argparse,hashlib,json,os,shutil,signal,subprocess,tempfile,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--prefix',type=Path,required=True);p.add_argument('--ros-prefix',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
manager=a.prefix.resolve()/'lib/nomad_costmap_cpp/nomad_runtime_manager'
env=dict(os.environ,ROS_DOMAIN_ID='210')
results={}
def wait_for(path,predicate,timeout=15):
    due=time.monotonic()+timeout
    while time.monotonic()<due:
        if path.exists():
            value=json.loads(path.read_text())
            if predicate(value):return value
        time.sleep(.05)
    raise AssertionError('Timed out waiting for '+str(path))
def alive(pid):
    try:
        raw=(Path('/proc')/str(pid)/'stat').read_text();return raw[raw.rfind(')')+2:].split()[0]!='Z'
    except OSError:return False
def descendants(root):
    found={root}
    for _ in range(12):
        new=set()
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                raw=(proc/'stat').read_text();fields=raw[raw.rfind(')')+2:].split()
                if int(fields[1]) in found:new.add(int(proc.name))
            except (OSError,ValueError,IndexError):pass
        new-=found
        if not new:break
        found.update(new)
    return found
with tempfile.TemporaryDirectory(prefix='nomad-native-lifecycle-') as tmp:
    states=Path(tmp)/'runs'
    def command(run,action,*extra):
        return subprocess.run([str(manager),'--prefix',str(a.prefix.resolve()),'--state-root',str(states),'--run-id',run,'--action',action,*extra],env=env,text=True,capture_output=True,timeout=20)
    # Refuse malformed run IDs before launching or touching any process.
    r=command('../unsafe','start');assert r.returncode==2;results['invalid_run_id_rejected']=True
    for run,kill in [('normal_stop',False),('worker_death',True)]:
        r=command(run,'start','--external-lidar','--ros-prefix',a.ros_prefix,'--startup-timeout','30');assert r.returncode==0,r.stderr
        state=states/run/'state.json'
        value=wait_for(state,lambda v:len(v.get('children',{}))==2)
        mapper=value['children']['mapper']['pid'];owned=descendants(value['manager']['pid'])
        due=time.monotonic()+8
        while time.monotonic()<due and len(descendants(mapper))<4:time.sleep(.05)
        owned|=descendants(mapper)
        assert len(descendants(mapper))>=4,owned
        native=[]
        for pid in sorted(owned):
            proc=Path('/proc')/str(pid)
            executable=os.readlink(proc/'exe')
            maps=(proc/'maps').read_text().lower()
            loader=None
            if 'qemu' in Path(executable).name:
                # binfmt reports the host emulator as /proc/PID/exe. Inspect
                # the explicitly launched guest image and its mapped libraries.
                assert os.environ.get('NOMAD_PC_QEMU_SOCKET_COMPAT')=='1'
                loader=executable
                args=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
                candidates=[v for v in args if v.startswith(str(a.prefix.resolve())+'/lib/nomad_costmap_cpp/')]
                assert candidates,(pid,args)
                executable=candidates[0]
                assert Path(executable).name in ('nomad_mapper','nomad_monitor','nomad_runtime_manager')
            with open(executable,'rb') as stream:
                magic=stream.read(4)
            assert magic==b'\x7fELF',(pid,executable,magic,(proc/'cmdline').read_bytes())
            assert 'python' not in Path(executable).name.lower() and 'libpython' not in maps
            native.append(dict(pid=pid,executable=executable,python_loaded=False,emulator=loader))
        results[run+'_native_runtime']=native
        r=command(run,'start','--external-lidar','--ros-prefix',a.ros_prefix);assert r.returncode==2
        results[run+'_duplicate_start_rejected']=True
        if kill:
            os.kill(mapper,signal.SIGKILL)
            final=wait_for(state,lambda v:v['state']=='FAILED')
        else:
            before=json.loads(state.read_text())
            owner=before['manager']['pid']
            owner_stat=(Path('/proc')/str(owner)/'stat').read_text()
            r=command(run,'stop')
            assert r.returncode==0,dict(error=r.stderr,state=before,owner_stat=owner_stat,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text(),after=json.loads(state.read_text()))
            final=json.loads(r.stdout);assert final['state']=='STOPPED'
        due=time.monotonic()+5
        while time.monotonic()<due and any(alive(pid) for pid in owned):time.sleep(.05)
        assert not any(alive(pid) for pid in owned),[pid for pid in owned if alive(pid)]
        results[run]=dict(passed=True,final_state=final['state'],owned_processes=len(owned))
    # A stale process identity must never authorize sending a stop signal.
    unrelated=subprocess.Popen(['/bin/sleep','30'])
    try:
        raw=(Path('/proc')/str(unrelated.pid)/'stat').read_text();fields=raw[raw.rfind(')')+2:].split()
        fake=states/'stale_owner';fake.mkdir()
        (fake/'state.json').write_text(json.dumps(dict(state='WAITING',manager=dict(pid=unrelated.pid,start_ticks=int(fields[19])+1,pgid=int(fields[2]),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))))
        r=command('stale_owner','stop');assert r.returncode==2 and unrelated.poll() is None
        results['stale_owner_not_signalled']=True
    finally:unrelated.terminate();unrelated.wait()
    r=command('startup_timeout','start','--external-lidar','--ros-prefix',a.ros_prefix,'--startup-timeout','2');assert r.returncode==0
    final=wait_for(states/'startup_timeout/state.json',lambda v:v['state']=='FAILED')
    assert 'Startup deadline expired' in final['reason'];results['startup_timeout_stops_children']=True
    # Integrity failure is checked against a separate disposable copy.
    damaged=Path(tmp)/'damaged';shutil.copytree(a.prefix,damaged)
    with (damaged/'share/nomad_costmap_cpp/config/slope.json').open('a') as f:f.write(' ')
    r=subprocess.run([str(damaged/'lib/nomad_costmap_cpp/nomad_runtime_manager'),'--prefix',str(damaged),'--action','check'],text=True,capture_output=True)
    assert r.returncode==2 and 'hash mismatch' in r.stderr;results['tamper_rejected']=True
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(dict(passed=True,checks=results,
    runtime_manifest_sha256=hashlib.sha256((a.prefix/'runtime_manifest.json').read_bytes()).hexdigest(),
    manager_sha256=hashlib.sha256(manager.read_bytes()).hexdigest()),indent=2)+'\n')
print(json.dumps(results))
