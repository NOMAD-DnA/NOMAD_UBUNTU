"""Build-time native dependency packaging for the existing Jetson TF2 overlay."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

p = argparse.ArgumentParser()
for name in ('runtime', 'ros-prefix', 'workspace', 'install'):
    p.add_argument('--' + name, required=True, type=Path)
a = p.parse_args()
runtime, ros, workspace = a.runtime.resolve(), a.ros_prefix.resolve(), a.workspace.resolve()
# The frozen packager deliberately knows only original costmap executables.
# Add the separately built native adapter before auditing every runtime ELF.
for relative in ('lib/nomad_costmap_cpp/nomad_semantic_id_adapter',
                 'share/nomad_costmap_cpp/config/ade20k_to_field12.json'):
    origin = a.install.resolve() / relative
    if not origin.is_file():
        raise SystemExit('Missing built semantic adapter artifact: ' + str(origin))
    destination = runtime / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(origin), str(destination))
copied = {}
forbidden = ('libpython', 'libcuda', 'libcudart', 'libcublas', 'libcudnn', 'libnvinfer')

def within(path, parent):
    return path == parent or parent in path.parents

def elf_files():
    for path in sorted((runtime / 'lib').rglob('*')):
        if path.is_file():
            with path.open('rb') as stream:
                if stream.read(4) == b'\x7fELF':
                    yield path

def copy_workspace_library(path):
    resolved = path.resolve()
    if not within(resolved, workspace):
        raise SystemExit('Refusing an unexpected overlay library: ' + str(path))
    if any(token in path.name for token in forbidden):
        raise SystemExit('Forbidden native dependency: ' + str(path))
    destination = runtime / 'lib' / path.name
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if destination.exists():
        if hashlib.sha256(destination.read_bytes()).hexdigest() != digest:
            raise SystemExit('Conflicting native library name: ' + path.name)
        return False
    shutil.copy2(str(path), str(destination), follow_symlinks=True)
    copied[path.name] = {'source': str(path), 'sha256': digest}
    return True

def dependencies(path, environment):
    dynamic = subprocess.check_output(['readelf', '-d', str(path)], universal_newlines=True)
    needed = re.findall(r'\(NEEDED\).*?\[(.*?)\]', dynamic)
    if any(any(token in item for token in forbidden) for item in needed):
        raise SystemExit('Forbidden ELF dependency in ' + str(path))
    result = subprocess.run(['ldd', str(path)], env=environment, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True)
    if result.returncode or 'not found' in result.stdout:
        raise SystemExit('Unresolved runtime dependency in ' + str(path) + ':\n' + result.stdout)
    return [Path(value) for value in re.findall(r'=>\s+(/\S+)\s+\(', result.stdout)]

# ROS chooses these C++ typesupport plugins dynamically, outside the direct ldd closure.
for path in sorted((workspace / 'install/tf2_msgs/lib').glob('libtf2_msgs__*cpp.so*')):
    if path.is_file():
        copy_workspace_library(path)
build_env = os.environ.copy()
build_env['LD_LIBRARY_PATH'] = str(runtime / 'lib') + ':' + build_env.get('LD_LIBRARY_PATH', '')
while True:
    changed = False
    for path in list(elf_files()):
        for dependency in dependencies(path, build_env):
            if within(dependency.resolve(), workspace) and not within(dependency.resolve(), runtime):
                changed = copy_workspace_library(dependency) or changed
    if not changed:
        break
clean_env = os.environ.copy()
clean_env['LD_LIBRARY_PATH'] = str(runtime / 'lib') + ':' + str(ros / 'lib')
for path in elf_files():
    dependencies(path, clean_env)
if any(path.suffix in ('.py', '.pyc') for path in runtime.rglob('*') if path.is_file()):
    raise SystemExit('Python source unexpectedly present in the native runtime')
provenance = runtime / 'share/nomad_costmap_cpp/native_dependency_provenance.json'
provenance.write_text(json.dumps({
    'layout_revision': 'traversability-jetson-20261106-3',
    'ros_underlay': str(ros), 'workspace_libraries': copied,
    'audit': 'ELF dependencies resolve with runtime/lib and ROS underlay/lib only; no Python or CUDA',
}, indent=2) + '\n')
license_source = workspace / 'src/geometry2/LICENSE'
if license_source.is_file():
    destination = runtime / 'share/nomad_costmap_cpp/licenses/geometry2-LICENSE'
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(license_source), str(destination))
print('Packaged', len(copied), 'workspace native dependencies; runtime ELF audit passed.')
