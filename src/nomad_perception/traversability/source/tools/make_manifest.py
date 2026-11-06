"""Build-time packaging only. The installed runtime does not invoke Python."""
import argparse, hashlib, json, platform
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('prefix',type=Path)
p.add_argument('--build-id',default='nomad-costmap-cpp-20261105-2')
a=p.parse_args()
files={}
for path in sorted(a.prefix.rglob('*')):
    if path.is_file() and path.name!='runtime_manifest.json':
        files[str(path.relative_to(a.prefix))]=hashlib.sha256(path.read_bytes()).hexdigest()
with (a.prefix/'lib/nomad_costmap_cpp/nomad_mapper').open('rb') as stream: elf=stream.read(20)
machine=int.from_bytes(elf[18:20],'little')
architecture={183:'aarch64',62:'x86_64'}.get(machine,'unknown')
if elf[:4]!=b'\x7fELF' or architecture=='unknown':raise SystemExit('Unknown runtime ELF architecture')
manifest=dict(build_id=a.build_id,runtime_language='C++17',python_runtime=False,
              architecture=architecture,reference_build='nomad-pi5-oak-zed-native-20261105-7',
              numerical_policy='no fast-math; nearest-even; int64 capture nanoseconds; ordered observations',files=files)
(a.prefix/'runtime_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Recorded',len(files),'native runtime files')
