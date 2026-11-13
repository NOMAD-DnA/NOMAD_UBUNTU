#!/usr/bin/env bash
# 서비스 준비만 수행합니다. START/속도/서보 명령을 전송하지 않습니다.
set -euo pipefail
script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/workspace_paths.sh"
session="$NOMAD_WORKSPACE/commission_session"
# Recording only subscribes to ROS; a missing or slow recorder cannot block startup.
recorder="$script_dir/start_recording.sh"
if [[ ! -f $recorder ]]; then recorder="$NOMAD_WORKSPACE/diagnostics/start_recording.sh"; fi
if [[ -f $recorder ]]; then
    ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 timeout 3s bash "$recorder" || echo 'Passive recorder could not start.' >&2
fi
wanted=false
if [[ ${NOMAD_LIFTED_TEST:-NO} == YES ]]; then wanted=true; fi
exec 8>"$session/startup.lock"
flock -n 8 || { echo 'Another startup is running. Try again shortly.'; exit 1; }
export NOMAD_SETUP_ACK=$wanted
set +e
python3 - "$wanted" <<'PY'
import json,sys,urllib.request
try: s=json.load(urllib.request.urlopen('http://127.0.0.1:8765/api/status',timeout=2))
except Exception: sys.exit(2)
if s.get('connected') and s.get('state')=='RUNNING':
 print('Vehicle session is active. Use STOP in the existing GUI before changing mode.');sys.exit(3)
if (s.get('connected') and s.get('telemetry_fresh') and s.get('control_units')=='raw'
 and s.get('commissioning_setup_acknowledged')==(sys.argv[1]=='true')
 and 'ground_drive_supported' in s):
 print('NOMAD already connected; no START sent.');sys.exit(0)
sys.exit(2)
PY
status=$?
set -e
if [[ $status == 0 ]]; then exit 0; fi
if [[ $status == 3 ]]; then exit 3; fi
# 기존 PID 파일과 실행 경로가 일치하는 NOMAD 서비스만 정상 종료합니다.
bash "$session/stop.sh"
bash "$NOMAD_WORKSPACE/monitor_session/stop.sh"
sleep 2
exec 9>"$NOMAD_VESC_WORKSPACE/device.lock"
flock -n 9 || { echo 'Another control app owns VESC. Close the keyboard teleop window, then retry.'; exit 1; }
port=/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00
[[ -e "$port" ]] || { echo 'VESC USB not found. Check VESC power and USB cable.'; exit 1; }
if fuser -s "$port"; then echo 'VESC serial port in use. Close the other control app first.'; exit 1; fi
[[ $(cat "$session/build.exit") == 0 ]] || { echo 'Installed build has not passed tests.'; exit 1; }
# 펌웨어 설정을 읽어 검증할 뿐, 설정을 쓰거나 모터를 구동하지 않습니다.
# 기존 조회기가 같은 장치 잠금을 직접 취득하므로 조회 동안 잠금을 넘깁니다.
flock -u 9
python3 "$NOMAD_VESC_WORKSPACE/inspect_vesc.py" > "$session/firmware_readback.json"
python3 - "$session/firmware_readback.json" <<'PY'
import json,sys
v=json.load(open(sys.argv[1]));m=v['motor']
assert 0<v['timeout_ms']<=300 and v['timeout_brake_a']==0 and v['servo_output'] and v['app_to_use']==3
assert m['motor_type']==2 and 0<m['current_max_a']<=50 and -50<=m['current_min_a']<=-30, 'VESC motor current limits differ from the 2026-11-13 configuration (max <=50 A, min -50..-30 A for the stop brake)'
assert 0<m['battery_max_a']<=30 and -30<=m['battery_regen_a']<=0, 'VESC battery limits differ from the 2026-11-13 configuration (max <=30 A, regen -30..0 A)'
assert m['speed_pid_min_erpm']==50, 'VESC speed PID minimum differs from validated configuration'
print('VESC configuration checked. No configuration changed.')
PY
flock -n 9 || { echo 'Another app acquired VESC during inspection. Retry after closing it.'; exit 1; }
if fuser -s "$port"; then echo 'VESC became busy during inspection.'; exit 1; fi
cleanup_failure(){ code=$?; if [[ $code != 0 ]]; then bash "$session/stop.sh" || true; fi; }
trap cleanup_failure EXIT
for role in driver controller gui; do
 nohup bash "$session/run_$role.sh" > "$session/$role.log" 2>&1 < /dev/null 8>&- &
 echo $! > "$session/$role.pid"
done
# FD9은 서비스에 유지됩니다. 다른 키보드 조종기의 장치 동시 사용을 막습니다.
python3 - "$wanted" <<'PY'
import json,sys,time,urllib.request
end=time.monotonic()+20
while time.monotonic()<end:
 try:
  s=json.load(urllib.request.urlopen('http://127.0.0.1:8765/api/status',timeout=2))
  if (s.get('connected') and s.get('telemetry_fresh') and s.get('driver_ready')
   and s.get('state')=='IDLE' and s.get('commissioning_setup_acknowledged')==(sys.argv[1]=='true')
   and s.get('ground_drive_supported') is False):
   print('Connected. Mode: '+('raised-wheel manual READY' if sys.argv[1]=='true' else 'VIEW ONLY - controls locked')+'. No START sent.');break
 except Exception: pass
 time.sleep(.3)
else: raise SystemExit('Startup incomplete. Check controller.log and driver.log. No automatic drive.')
PY
