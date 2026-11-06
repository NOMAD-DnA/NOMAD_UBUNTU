"""Development/build-time audit. Run with the target architecture's loader."""
import argparse,json,os,subprocess
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('prefix',type=Path)
p.add_argument('--ros-prefix',required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();prefix=a.prefix.resolve()
env=dict(os.environ,LD_LIBRARY_PATH=str(prefix/'lib')+':'+a.ros_prefix+'/lib')
rows=[]
for file in sorted(prefix.rglob('*')):
    if not file.is_file():continue
    if file.suffix in ('.py','.pyc','.pyo'):
        raise SystemExit('Python runtime file: '+str(file))
    with file.open('rb') as stream:head=stream.read(20)
    if head[:4]!=b'\x7fELF':continue
    result=subprocess.run(['ldd',str(file)],env=env,text=True,capture_output=True)
    output=result.stdout+result.stderr
    assert result.returncode==0,(file,output)
    assert 'not found' not in output and 'libpython' not in output.lower(),(file,output)
    assert all(term not in output.lower() for term in ('libcuda','libcudart','libnvinfer')),(file,output)
    rows.append(dict(file=str(file.relative_to(prefix)),elf_machine=int.from_bytes(head[18:20],'little'),dependencies=output))
assert rows
check=subprocess.run([str(prefix/'lib/nomad_costmap_cpp/nomad_runtime_manager'),'--action','check','--prefix',str(prefix)],env=env,text=True,capture_output=True)
assert check.returncode==0,check.stderr
value=dict(passed=True,elf_count=len(rows),no_python=True,no_cuda=True,files=rows,manager_check=check.stdout)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(value,indent=2)+'\n')
print('Native ELF audit passed:',len(rows),'files, Python and CUDA absent')
