"""Build-time packaging. Copy only native runtime files, never setup launchers."""
import argparse, shutil, subprocess, sys
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('install',type=Path)
p.add_argument('runtime',type=Path)
a=p.parse_args()
source=a.install.resolve();target=a.runtime.resolve()
if source==target or source in target.parents:
    raise SystemExit('Runtime must be a separate directory outside the install prefix')
target.mkdir(parents=True,exist_ok=True)
# This tool refuses to overwrite a nonempty published runtime. Use a new path.
if any(target.iterdir()):raise SystemExit('Runtime destination must be empty: '+str(target))
for name in ('nomad_mapper','nomad_monitor','nomad_runtime_manager','nomad_core_probe'):
    destination=target/'lib/nomad_costmap_cpp'/name
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source/'lib/nomad_costmap_cpp'/name,destination)
for file in (source/'lib').glob('*.so*'):
    if file.is_file():shutil.copy2(file,target/'lib'/file.name,follow_symlinks=True)
driver=source/'lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node'
if driver.exists():
    destination=target/'lib/ydlidar_ros2_driver'/driver.name
    destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(driver,destination)
configs=('semantic.real.json','slope.json','g2b.yaml','classes.csv',
         'oak_g2b_selected_20261030.json','zed2_g2b_selected_20261030.json')
for name in configs:
    destination=target/'share/nomad_costmap_cpp/config'/name
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source/'share/nomad_costmap_cpp/config'/name,destination)
for package in ('nomad_interfaces','nomad_costmap_cpp','ydlidar_ros2_driver'):
    marker=target/'share/ament_index/resource_index/packages'/package
    marker.parent.mkdir(parents=True,exist_ok=True);marker.write_text('')
if (source/'share/nomad_interfaces/msg').exists():
    shutil.copytree(source/'share/nomad_interfaces/msg',target/'share/nomad_interfaces/msg')
licenses=target/'share/nomad_costmap_cpp/licenses';licenses.mkdir(parents=True,exist_ok=True)
for file in (Path(__file__).resolve().parents[1]/'vendor').glob('LICENSE*'):
    shutil.copy2(file,licenses/file.name)
subprocess.check_call([sys.executable,str(Path(__file__).with_name('make_manifest.py')),str(target)])
print('Native runtime:',target)
