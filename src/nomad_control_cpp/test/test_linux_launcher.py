"""Linux launcher integration with fake SSH/HTTP: never connects to a vehicle."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

source = Path(__file__).resolve().parents[1] / 'linux'
checks = 0

def expect(condition, description):
    global checks
    checks += 1
    assert condition, description

with tempfile.TemporaryDirectory(prefix='nomad-linux-') as temporary:
    base = Path(temporary)
    app = base / 'folder with spaces' / 'NOMAD GUI'
    shutil.copytree(source, app)
    mock = base / 'mock-bin'
    mock.mkdir()
    runtime = base / 'runtime'
    runtime.mkdir()
    home = base / 'home'
    (home / 'Desktop').mkdir(parents=True)
    log = base / 'calls.jsonl'
    master = base / 'master'
    stub = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
name=Path(sys.argv[0]).name
args=sys.argv[1:]
with open(os.environ['MOCK_LOG'],'a') as f:f.write(json.dumps([name,args])+'\\n')
master=Path(os.environ['MOCK_MASTER'])
if name=='ssh':
 if '-O' in args:
  operation=args[args.index('-O')+1]
  if operation=='check':sys.exit(0 if master.exists() else 1)
  if operation=='exit':master.unlink(missing_ok=True);sys.exit(0)
 if '-M' in args:
  if os.environ.get('MOCK_BIND_FAIL'):sys.exit(255)
  master.touch();sys.exit(0)
 sys.exit(int(os.environ.get('MOCK_PREPARE_CODE','0')))
if name=='curl':sys.exit(22 if os.environ.get('MOCK_HTTP_FAIL') else 0)
if name=='xdg-user-dir':print(os.environ['HOME']+'/Desktop')
'''
    for name in ['ssh', 'curl', 'xdg-open', 'xdg-user-dir']:
        path = mock / name
        path.write_text(stub)
        path.chmod(0o755)
    env = dict(os.environ, PATH=str(mock)+':'+os.environ['PATH'], HOME=str(home),
               XDG_RUNTIME_DIR=str(runtime), XDG_DATA_HOME=str(home/'data'),
               MOCK_LOG=str(log), MOCK_MASTER=str(master))

    def run(script='start.sh', arguments=(), overrides=None):
        log.write_text('')
        result = subprocess.run(['bash', str(app/script), *arguments], env=env | (overrides or {}),
                                capture_output=True, text=True, timeout=10)
        calls = [json.loads(line) for line in log.read_text().splitlines()]
        return result, calls

    result, calls = run(arguments=['--check'])
    expect(result.returncode == 0 and not calls, 'check does not connect')
    result, calls = run(arguments=['--no-browser'])
    expect(result.returncode == 0 and master.exists(), 'first startup retains authenticated tunnel')
    masters = [args for command,args in calls if command=='ssh' and '-M' in args]
    expect(len(masters)==1 and '127.0.0.1:8765:127.0.0.1:8765' in masters[0], 'one loopback-only master')
    prepare = [args for command,args in calls if command=='ssh' and any('ensure_gui.sh' in x for x in args)]
    expect(len(prepare)==1 and '-S' in prepare[0], 'service preparation reuses authenticated master')
    expect(not any('/api/commissioning' in str(args) or '/api/manual' in str(args) for _,args in calls), 'no actuator route')
    result, calls = run(arguments=['--no-browser'])
    expect(result.returncode==0 and not any('-M' in args for _,args in calls), 'repeat launch reuses tunnel')
    result, calls = run(arguments=['--no-browser'], overrides={'MOCK_PREPARE_CODE':'3'})
    expect(result.returncode==0 and master.exists() and '이미 조종 중' in result.stdout, 'active session is left running')
    result, calls = run(arguments=['--no-browser'], overrides={'MOCK_PREPARE_CODE':'1'})
    expect(result.returncode!=0 and master.exists(), 'failed preparation does not close preexisting tunnel')
    result, calls = run('disconnect.sh')
    expect(result.returncode==0 and not master.exists(), 'disconnect targets own master')
    result, calls = run(arguments=['--no-browser'], overrides={'MOCK_PREPARE_CODE':'1'})
    expect(result.returncode!=0 and not master.exists(), 'failed new startup closes its own tunnel')
    result, calls = run(arguments=['--no-browser'], overrides={'MOCK_HTTP_FAIL':'1'})
    expect(result.returncode!=0 and not master.exists(), 'HTTP failure closes new tunnel')
    result, calls = run(arguments=['--no-browser'], overrides={'MOCK_BIND_FAIL':'1'})
    expect(result.returncode!=0 and not any('ensure_gui.sh' in str(args) for _,args in calls), 'occupied port never starts services')
    local = app/'connection.local.conf'
    local.write_text('PI_HOST=$(touch '+str(base/'injected')+')\nPI_USER=user\nPI_WORKSPACE=/home/user/nomad_cpp_ws\nSSH_KEY=\n')
    result, calls = run(arguments=['--check'])
    expect(result.returncode!=0 and not calls and not (base/'injected').exists(), 'configuration is not executable shell')
    local.unlink()
    result, calls = run('install-desktop.sh')
    entry = home/'data/applications/nomad-gui.desktop'
    expect(result.returncode==0 and entry.exists() and 'Terminal=true' in entry.read_text(), 'desktop entry installed')
    expect(str(app/'start.sh') in entry.read_text() and (home/'Desktop/NOMAD-GUI.desktop').exists(), 'desktop preserves folder with spaces')
    result, calls = run(arguments=['--no-browser'])
    expect(result.returncode==0, 'clean reconnect after prior errors')
print(f'{checks} Linux launcher checks passed (mock SSH/HTTP; no vehicle output)')
