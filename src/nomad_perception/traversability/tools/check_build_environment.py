"""Read-only build preflight. No compiler, ROS node or device is started."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess

p = argparse.ArgumentParser()
for option in ('root', 'ros-prefix', 'workspace', 'build', 'install', 'runtime'):
    p.add_argument('--' + option, required=True, type=Path)
a = p.parse_args()
root = a.root.resolve()

def within(path, parent):
    return path == parent or parent in path.parents

for name in ('build', 'install', 'runtime'):
    path = getattr(a, name).resolve()
    if not within(path, root) or path == root or within(path, root / 'source'):
        raise SystemExit(name + ' must be a separate output directory inside traversability')
outputs = [getattr(a, name).resolve() for name in ('build', 'install', 'runtime')]
if any(within(left, right) or within(right, left)
       for i, left in enumerate(outputs) for right in outputs[i + 1:]):
    raise SystemExit('Build, install and runtime directories must not overlap')
for command in ('cmake', 'ctest', 'g++', 'make', 'python3', 'readelf', 'ldd', 'tar', 'tee'):
    if shutil.which(command) is None:
        raise SystemExit('Missing build tool: ' + command)
for command in (['g++', '-dumpfullversion'], ['cmake', '--version'], ['ctest', '--version']):
    print(subprocess.check_output(command, universal_newlines=True).splitlines()[0])
prefixes = [Path(value) for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH')
            for value in os.environ.get(key, '').split(':') if value]
prefixes.append(a.ros_prefix)
packages = ('ament_cmake', 'rosidl_cmake', 'rosidl_generator_cpp',
            'rosidl_typesupport_cpp', 'rosidl_typesupport_fastrtps_cpp',
            'rosidl_typesupport_introspection_cpp', 'rclcpp', 'tf2_ros',
            'sensor_msgs', 'nav_msgs', 'std_msgs', 'builtin_interfaces',
            'geometry_msgs', 'diagnostic_msgs', 'std_srvs', 'visualization_msgs')
for package in packages:
    matches = [prefix / 'share' / package / 'cmake' / (package + 'Config.cmake')
               for prefix in prefixes]
    found = next((path for path in matches if path.is_file()), None)
    if found is None:
        raise SystemExit('Missing ROS C++ development package: ' + package)
    print('ROS development package:', package, str(found))
for name in ('PlanningCostLayers', 'DriveCommand', 'VehicleState',
             'ModuleStatus', 'PlannedMotion', 'GnssQuality'):
    incoming = root / 'source/interfaces/msg' / (name + '.msg')
    existing = a.workspace / 'src/nomad_interfaces/msg' / (name + '.msg')
    if not existing.is_file() or incoming.read_bytes() != existing.read_bytes():
        raise SystemExit('Existing nomad_interfaces contract differs: ' + name)
print('All six existing project message definitions match the frozen source.')
for package in ('tf2', 'tf2_ros'):
    lib = a.workspace / 'install' / package / 'lib' / ('lib' + package + '.so')
    if not lib.is_file():
        raise SystemExit('Missing existing workspace C++ library: ' + str(lib))
    result = subprocess.run(['ldd', str(lib)], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True)
    if result.returncode or 'not found' in result.stdout:
        raise SystemExit('Unresolved TF2 C++ dependency:\n' + result.stdout)
    if any(token in result.stdout for token in
           ('libpython', 'libcuda', 'libcudart', 'libcublas', 'libcudnn', 'libnvinfer')):
        raise SystemExit('Existing TF2 library requires a forbidden runtime dependency:\n' + result.stdout)
    print('Resolved native workspace library:', lib)
print('Available disk bytes:', shutil.disk_usage(str(root)).free)
print('Output paths:', *(str(path) for path in outputs))
