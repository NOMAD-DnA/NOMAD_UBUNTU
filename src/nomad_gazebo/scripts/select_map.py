#!/usr/bin/env python3
"""Discover saved world bundles and select one without changing launch syntax."""
import argparse,json,os,sys
from pathlib import Path


def discover(root,bare=False):
    maps=[]
    if not root.is_dir():raise ValueError(f'Map folder not found: {root}')
    for folder in sorted(root.iterdir()):
        if folder.is_file() and folder.suffix=='.sdf':
            maps.append({'id':folder.stem,'name':folder.stem,'world':folder.resolve()});continue
        if not folder.is_dir():continue
        file=folder/'map.json';meta=json.loads(file.read_text()) if file.exists() else {}
        relative=meta.get('world_bare' if bare else 'world','worlds/forest_bare.sdf' if bare else 'worlds/forest.sdf')
        world=folder/relative
        if not world.exists() and not bare and (folder/'world.sdf').exists():world=folder/'world.sdf'
        if world.is_file():maps.append({'id':folder.name,'name':meta.get('name',folder.name),'world':world.resolve()})
    if not maps:raise ValueError(f'No SDF maps found: {root}')
    return maps


def choose(maps,selection=None,interactive=True):
    if selection is None and not interactive:
        return next((m for m in maps if m['id']=='02_fork2_gentle'),maps[0])
    if selection is None:
        print('\n맵 선택:',file=sys.stderr)
        for i,m in enumerate(maps,1):print(f"  {i}. {m['name']} [{m['id']}]",file=sys.stderr)
        while True:
            print('번호 입력: ',end='',file=sys.stderr,flush=True)
            selection=input().strip()
            try:return choose(maps,selection,False)
            except ValueError:print('목록의 번호를 입력해 주세요.',file=sys.stderr)
    if selection.isdigit() and 1<=int(selection)<=len(maps):return maps[int(selection)-1]
    for m in maps:
        if selection==m['id']:return m
    raise ValueError(f'Unknown map: {selection}')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--maps-dir',type=Path);parser.add_argument('--select');parser.add_argument('--list',action='store_true');parser.add_argument('--bare',action='store_true');args=parser.parse_args()
    if args.maps_dir:root=args.maps_dir
    elif os.environ.get('NOMAD_MAPS_DIR'):root=Path(os.environ['NOMAD_MAPS_DIR'])
    elif (Path(__file__).resolve().parents[1]/'maps').is_dir():
        root=Path(__file__).resolve().parents[1]/'maps'
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            root=Path(get_package_share_directory('nomad_gazebo'))/'maps'
        except (ImportError,LookupError):root=Path(__file__).resolve().parents[1]/'maps'
    maps=discover(root,args.bare)
    if args.list:
        for i,m in enumerate(maps,1):print(f"{i}. {m['name']} [{m['id']}]")
        return
    selected=choose(maps,args.select or os.environ.get('NOMAD_MAP'),sys.stdin.isatty())
    print(f"선택: {selected['name']}",file=sys.stderr)
    print(selected['world'])

if __name__=='__main__':
    try:main()
    except (ValueError,OSError,EOFError) as exc:raise SystemExit(str(exc))
