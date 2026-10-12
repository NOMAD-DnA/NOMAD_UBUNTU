#!/usr/bin/env python3
"""Pinned upstream source import. Never reset or overwrite an existing checkout."""
import argparse,json,subprocess,shutil
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--cache-root',type=Path);args=p.parse_args()
here=Path(__file__).resolve().parent
ws=here.parents[1]  # .nomad/vio/import_sources.py
root=ws/'src/third_party/vio';root.mkdir(parents=True,exist_ok=True)
def run(*cmd):subprocess.run(cmd,check=True)
for repo in json.loads((here/'sources.json').read_text()):
 dst=root/repo['path'];dst.parent.mkdir(parents=True,exist_ok=True)
 if not dst.exists():
  cache=args.cache_root/repo['cache_path'] if args.cache_root else None
  if cache and cache.exists():
   if repo['path']=='basalt_wrapper/basalt':
    # Existing recursive dependency trees are self contained; only our known
    # local patch is accepted below. Never modify the cache workspace.
    shutil.copytree(cache,dst)
   else:
    run('git','clone','--no-hardlinks',str(cache),str(dst))
    run('git','-C',str(dst),'remote','set-url','origin',repo['url'])
    run('git','-C',str(dst),'checkout','--detach',repo['commit'])
  else:
   dst.mkdir()
   run('git','-C',str(dst),'init')
   run('git','-C',str(dst),'remote','add','origin',repo['url'])
   run('git','-C',str(dst),'fetch','--depth','1','origin',repo['commit'])
   run('git','-C',str(dst),'checkout','--detach','FETCH_HEAD')
 probe=subprocess.run(['git','-C',str(dst),'rev-parse','HEAD'],text=True,capture_output=True)
 if probe.returncode:
  # Retry an interrupted first fetch only when no worktree files exist.
  if any(child.name!='.git' for child in dst.iterdir()):
   raise RuntimeError(f'{dst}: incomplete checkout has files; not overwritten')
  origin=subprocess.check_output(['git','-C',str(dst),'remote','get-url','origin'],text=True).strip()
  if origin!=repo['url']:raise RuntimeError(f'{dst}: unexpected origin; not overwritten')
  run('git','-C',str(dst),'fetch','--depth','1','origin',repo['commit'])
  run('git','-C',str(dst),'checkout','--detach','FETCH_HEAD')
  probe=subprocess.run(['git','-C',str(dst),'rev-parse','HEAD'],check=True,text=True,capture_output=True)
 head=probe.stdout.strip()
 if head!=repo['commit']:raise RuntimeError(f'{dst}: expected {repo["commit"]}, found {head}; not overwritten')
 if (dst/'.gitmodules').exists() and not args.cache_root:
  run('git','-C',str(dst),'submodule','update','--init','--recursive')
for folder,patch in [('basalt_wrapper/basalt','basalt_core'),('basalt_ros','basalt_ros'),('open_vins','openvins')]:
 dst=root/folder;file=here/'patches'/f'{patch}.patch'
 reverse=subprocess.run(['git','-C',str(dst),'apply','--reverse','--check',str(file)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 if reverse.returncode==0:print(f'{patch}: already applied');continue
 run('git','-C',str(dst),'apply','--check',str(file))
 run('git','-C',str(dst),'apply',str(file))
 print(f'{patch}: applied')
print('Pinned Basalt/OpenVINS sources ready in',root)
