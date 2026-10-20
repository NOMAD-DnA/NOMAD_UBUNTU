#!/usr/bin/env bash
set -euo pipefail
ws_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS/ament setup scripts read optional unset variables; restore nounset afterward.
set +u
source "$ws_root/.nomad/env.sh"
set -u
exec /usr/bin/python3 - "$ws_root" "${1:-map4_selected_$(date +%Y%m%d_%H%M%S)}" <<'PYRUN'
import json,os,sys,re,subprocess,time,tempfile,yaml
from pathlib import Path
root=Path(sys.argv[1]);name=sys.argv[2]
if not re.fullmatch(r'[A-Za-z0-9_-]+',name):raise SystemExit('Use letters, digits, underscore or hyphen for experiment name')
if (root/'experiments'/name).exists():raise SystemExit('Experiment already exists; choose a new name')
args=json.loads((root/'experiments/map4_goal_followup_20261020/bearing1_command.json').read_text())
args[2]=name
experiment=root/'experiments'/name
print(f'실험: {name} (Gazebo/RViz 화면 없이 실행)',flush=True)
print(f'기록: {experiment}',flush=True)
print('초기 준비 최대240초, 본 주행 최대3600초. 진행 상태를 5초마다 표시합니다.',flush=True)
# Keep historical trial files immutable; apply the current default speed.
speed_config=yaml.safe_load((root/'src/nomad_perception/config/terrain_planning.yaml').read_text())
idx=args.index('--planning-overrides')+1
override=yaml.safe_load(Path(args[idx]).read_text())
override['ackermann_rollout_lpp']['ros__parameters']['forward_speed']=speed_config['ackermann_rollout_lpp']['ros__parameters']['forward_speed']
# Keep the historical weight vector except for the current slope setting.
weight_index=args.index('--weights')+1
weights=args[weight_index].split(',')
weights[1]=str(speed_config['dstar_lite_gpp']['ros__parameters']['gpp.slope_weight'])
args[weight_index]=','.join(weights)
override.setdefault('dstar_lite_gpp',{}).setdefault('ros__parameters',{})['gpp.slope_weight']=float(weights[1])
runtime_settings=tempfile.TemporaryDirectory(prefix='nomad-selected-settings-')
args[idx]=str(Path(runtime_settings.name)/'planning.yaml')
Path(args[idx]).write_text(yaml.safe_dump(override))
child=subprocess.Popen(args)
def show_status():
    try:
        s=json.loads((experiment/'status.json').read_text())
    except (OSError,ValueError):
        print('환경 및 실행 기록 준비 중...',flush=True);return
    age=max(0,time.time()-s.get('updated',time.time()))
    pos=s.get('position')
    where=f"({pos[0]:.2f}, {pos[1]:.2f})" if pos else '-'
    detail=s.get('last_status',{}).get('planning.command_status','')
    result=s.get('outcome',{}).get('result','')
    print(f"[{s.get('state','?')}/{s.get('phase','-')}] 경과 {s.get('elapsed_wall_s',0):.0f}s 위치 {where} {detail} {result} (갱신 {age:.0f}초 전)",flush=True)
try:
    while True:
        show_status()
        try:
            code=child.wait(timeout=5);break
        except subprocess.TimeoutExpired:
            pass
except KeyboardInterrupt:
    # The foreground process group receives Ctrl+C together. Let the existing
    # experiment runner finish its owned-process cleanup and bag flush.
    print('중단 요청: 실험 프로세스 정리와 기록 저장을 기다립니다.',flush=True)
    while True:
        try:
            code=child.wait(timeout=5);break
        except (subprocess.TimeoutExpired,KeyboardInterrupt):
            continue
show_status()
print(f'실험 실행 프로세스 종료: exit={code}. 위 상태는 마지막 저장값입니다.',flush=True)
raise SystemExit(code)
PYRUN
