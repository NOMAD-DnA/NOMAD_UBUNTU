"""Development-only serial trials; never launches a compiler or accesses Jetson."""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path

root=Path(__file__).resolve().parents[1]
benchmark=root.parent/'costmap_benchmark_20261105'
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
def write(name,value):
    path=a.output/name;temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');temp.replace(path)
def builders():
    result=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            name=(proc/'comm').read_text().strip()
            if name in ('cc1plus','cc1','ninja','make','cmake'):
                result.append(dict(pid=int(proc.name),name=name))
        except OSError:pass
    return result
assert not builders(),'Finish all builds before comparison'
for architecture in ('pc','arm64'):
    evidence=json.loads((root/'verification/event_dispatch_20261105'/(architecture+'_manager.json')).read_text())
    prefix=root/'release'/(architecture+'-cpp-2')
    assert evidence['passed']
    assert evidence['runtime_manifest_sha256']==hashlib.sha256((prefix/'runtime_manifest.json').read_bytes()).hexdigest()
    assert evidence['manager_sha256']==hashlib.sha256((prefix/'lib/nomad_costmap_cpp/nomad_runtime_manager').read_bytes()).hexdigest()
trials=[('python-7','python-reference',None,220),
        ('cpp-1','cpp',root/'release/pc-cpp-1',221),
        ('cpp-2','cpp',root/'release/pc-cpp-2',222)]
results=[]
for name,mapper,prefix,domain in trials:
    command=[sys.executable,str(benchmark/'run.py'),'--pc-native','--mapper',mapper,
        '--duration','60','--hz','5','--points','6000','--faults','all',
        '--domain',str(domain),'--no-serve']
    if prefix:command+=['--cpp-prefix',str(prefix)]
    log=a.output/(name+'.log');observed_builds=[];run=None
    with log.open('w') as output:
        process=subprocess.Popen(command,stdout=output,stderr=subprocess.STDOUT,
            env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
        try:
            while process.poll() is None:
                observed_builds+=builders()
                if run is None:
                    paths=list((benchmark/'runs').glob('*_'+str(process.pid)))
                    if paths:run=paths[0]
                write('status.json',dict(state='running',trial=name,pid=process.pid,
                    run=str(run) if run else None,completed=[r['trial'] for r in results]))
                time.sleep(1)
        except BaseException:
            if run:(run/'stop.request').touch()
            process.wait(timeout=30)
            raise
    assert process.returncode==0,log.read_text()[-4000:]
    assert run is not None
    report=json.loads((run/'report.json').read_text())
    assert report['interpretation']=='pass',(name,report.get('checks'))
    result=dict(trial=name,run=str(run),command=command,
        interval_ms=report['overall_interval_ms'],input_to_map_ms=report['overall_input_publish_to_map_receive_ms'],
        ready_fraction=report['measurement_ready_fraction'],map_count=report['measured_map_count'],
        mapper_resources=report['resources']['mapper'],input_rates=report['input_rates'],
        target=report['map_update_target'],pipeline=report['per_phase']['drive'],
        checks=report['checks'],build_processes_observed=observed_builds)
    results.append(result);write('comparison.json',dict(trials=results,
        one_trial_per_variant=True,scope='Native PC, same settings, serial runs, no deliberate background builds; not a hard real-time guarantee'))
write('status.json',dict(state='completed',completed=[r['trial'] for r in results]))
print('Serial Python-7, C++-1 and C++-2 comparisons completed')
