"""Launcher contract using mock commands only; never contacts ROS or hardware."""
import os
import pathlib
import shutil
import subprocess
import tempfile

project=pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as temp:
    root=pathlib.Path(temp)
    pkg=root/'ws/src/nomad_control_cpp'
    (pkg/'scripts').mkdir(parents=True)
    (root/'ws/install').mkdir()
    (root/'bin').mkdir()
    shutil.copy2(project/'scripts/start_lpp.sh',pkg/'scripts/start_lpp.sh')
    (pkg/'CMakeLists.txt').write_text('# fixture')
    (root/'ws/install/setup.bash').write_text('export NOMAD_TEST_OVERLAY=loaded\n')
    for name,body in {
        'colcon':'printf "build %s\\n" "$*" >> "$TEST_LOG"\n',
        'ros2':'if [[ $1 == pkg ]]; then exit 0; fi\nprintf "launch %s overlay=%s\\n" "$*" "$NOMAD_TEST_OVERLAY" >> "$TEST_LOG"\n'
    }.items():
        p=root/'bin'/name;p.write_text('#!/bin/bash\n'+body);p.chmod(0o755)
    env=dict(os.environ,PATH=str(root/'bin')+':'+os.environ['PATH'],TEST_LOG=str(root/'log'))
    subprocess.run(['bash',str(pkg/'scripts/start_lpp.sh')],env=env,check=True)
    result=(root/'log').read_text()
    assert 'colcon' not in result # executable name is intentionally not logged
    assert '--packages-up-to nomad_control_cpp' in result
    assert '-DNOMAD_ENABLE_PLANNING=ON' in result
    assert 'launch nomad_control_cpp lpp.launch.xml overlay=loaded' in result
    # Build failure must prevent launch of an older installed executable.
    (root/'log').write_text('')
    (root/'bin/colcon').write_text('#!/bin/bash\nexit 7\n')
    failure=subprocess.run(['bash',str(pkg/'scripts/start_lpp.sh')],env=env)
    assert failure.returncode==7 and (root/'log').read_text()==''
print('LPP entry: auto build, planning enabled, overlay loaded, single launch, build failure stops launch: PASS')
