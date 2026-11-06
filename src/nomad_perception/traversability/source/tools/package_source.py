"""Create a separate source bundle without build trees or runtime prototypes."""
import hashlib,json,shutil,tarfile
from pathlib import Path
root=Path(__file__).resolve().parents[1]
destination=root.parent/'costmap_cpp_source_20261105_2'
archive=root.parent/'jetson_costmap_cpp_source_20261105_2.tar.gz'
if destination.exists() or archive.exists():raise SystemExit('Published source destination already exists')
destination.mkdir()
for name in ('CMakeLists.txt','build_on_jetson.sh','README_JETSON.txt','TOPICS_JETSON.txt',
             'src','include','cmake','interfaces','config','vendor','tests','tools'):
    source=root/name
    if source.is_dir():shutil.copytree(source,destination/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    else:shutil.copy2(source,destination/name)
reference=destination/'reference';reference.mkdir()
shutil.copy2(root/'reference/baseline.json',reference/'baseline.json')
if (root/'analysis').exists():shutil.copytree(root/'analysis',destination/'analysis')
files={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest()
       for p in sorted(destination.rglob('*')) if p.is_file()}
manifest=dict(build_id='nomad-costmap-cpp-20261105-2',reference_build='nomad-pi5-oak-zed-native-20261105-7',files=files)
for folder in (root,destination):
    (folder/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
with tarfile.open(archive,'w:gz') as tar:tar.add(destination,arcname=destination.name)
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
archive.with_name(archive.name+'.sha256').write_text(digest+'  '+archive.name+'\n')
print(json.dumps(dict(source=str(destination),archive=str(archive),sha256=digest,files=len(files))))
