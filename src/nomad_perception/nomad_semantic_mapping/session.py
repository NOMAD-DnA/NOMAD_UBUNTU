"""Start, finish, list and reopen owned semantic mapping sessions."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from zoneinfo import ZoneInfo
from zipfile import BadZipFile


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def workspace() -> Path:
    configured = os.environ.get('NOMAD_WS_ROOT')
    if not configured:
        raise ValueError('먼저 이 폴더의 .nomad/env.sh를 읽어 주세요.')
    return Path(configured).resolve()


def process_identity(pid: int) -> str | None:
    try:
        directory = Path('/proc') / str(pid)
        if directory.stat().st_uid != os.getuid():
            return None
        return (directory / 'stat').read_text().rsplit(') ', 1)[1].split()[19]
    except (OSError, IndexError, ValueError):
        return None


def process_has_token(pid: int, key: str, token: str) -> bool:
    try:
        entries = (Path('/proc') / str(pid) / 'environ').read_bytes().split(b'\0')
        return os.fsencode(key + '=' + token) in entries
    except OSError:
        return False


def active_is_alive(active: dict) -> bool:
    try:
        pid = int(active['supervisor_pid'])
        token = active['supervisor_token']
        return (pid > 1 and isinstance(token, str) and bool(token)
                and process_identity(pid) == active['supervisor_start_ticks']
                and process_has_token(pid, 'NOMAD_SEMANTIC_SUPERVISOR_TOKEN', token))
    except (KeyError, TypeError, ValueError):
        return False


def owned_group_exists(group: int, token: str) -> bool:
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            fields = (directory / 'stat').read_text().rsplit(') ', 1)[1].split()
            if int(fields[2]) == group and process_has_token(
                    int(directory.name), 'NOMAD_SEMANTIC_SESSION_TOKEN', token):
                return True
        except (OSError, IndexError, ValueError):
            continue
    return False


def signal_owned_group(group: int, token: str, selected_signal: int) -> None:
    if owned_group_exists(group, token):
        try:
            os.killpg(group, selected_signal)
        except ProcessLookupError:
            pass


def stop_process(process: subprocess.Popen, token: str) -> None:
    if process.poll() is None:
        # ROS launch propagates shutdown and lets the mapper save its archive.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=90)
        except subprocess.TimeoutExpired:
            signal_owned_group(process.pid, token, signal.SIGTERM)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                signal_owned_group(process.pid, token, signal.SIGKILL)
                process.wait(timeout=5)
    # Descendants left after launch exit must belong to this session's group.
    signal_owned_group(process.pid, token, signal.SIGTERM)
    deadline = time.monotonic() + 3
    while owned_group_exists(process.pid, token) and time.monotonic() < deadline:
        time.sleep(.1)
    signal_owned_group(process.pid, token, signal.SIGKILL)


def archive_status(directory: Path) -> dict:
    """Only a committed, closed map is a completed final save."""
    import numpy as np
    from .archive import load_snapshot, resolve_snapshot
    try:
        snapshot = resolve_snapshot(directory)
        arrays, metadata = load_snapshot(snapshot)
        complete = snapshot.name == 'map.npz' and metadata.get('status') == 'closed'
        return {
            'final_save_complete': complete,
            'snapshot_file': snapshot.name,
            'has_data': bool(np.any(arrays['cost'] >= 0)),
            'state': 'finished' if complete else 'recovered',
        }
    except (OSError, ValueError, KeyError, TypeError, EOFError, BadZipFile):
        return {'final_save_complete': False, 'snapshot_file': None,
                'has_data': False, 'state': 'incomplete'}


def map_has_data(directory: Path) -> bool:
    return archive_status(directory)['has_data']


def run_directories(root: Path) -> list[Path]:
    sessions = root / 'sessions' / 'semantic'
    if not sessions.is_dir():
        return []
    return sorted((item for item in sessions.iterdir() if item.is_dir()),
                  key=lambda item: item.name, reverse=True)


def selected_session(root: Path, supplied: str | None) -> Path:
    if supplied:
        candidate = Path(supplied).expanduser()
        if not candidate.is_dir():
            candidate = root / 'sessions' / 'semantic' / supplied
        if not candidate.is_dir():
            raise ValueError(f'저장된 주행 폴더를 찾을 수 없습니다: {supplied}')
        if not map_has_data(candidate):
            raise ValueError(f'이 주행 폴더에는 아직 생성된 지도 셀이 없습니다: {candidate}')
        return candidate.resolve()
    for directory in run_directories(root):
        if map_has_data(directory):
            return directory
    raise ValueError('저장된 지도가 없습니다. 먼저 ros2 run nomad_perception semantic_session로 주행해 주세요.')


def normalized_arguments(arguments: list[str]) -> list[str]:
    translated = []
    for argument in arguments:
        if ':=' not in argument:
            translated.append(argument)
            continue
        key, value = argument.split(':=', 1)
        if key in ('headless', 'rviz', 'vegetation') and value.lower() not in (
                'true', '1', 'yes', 'false', '0', 'no'):
            raise ValueError(f'설정은 true 또는 false여야 합니다: {argument}')
        truth = value.lower() in ('true', '1', 'yes')
        if key == 'map':
            translated.extend(('--map', value))
        elif key == 'headless' and truth:
            translated.append('--headless')
        elif key == 'rviz' and not truth:
            translated.append('--no-rviz')
        elif key == 'vegetation' and not truth:
            translated.append('--bare')
        elif key not in ('headless', 'rviz', 'vegetation'):
            raise ValueError(f'지원하지 않는 실행 설정: {argument}')
    return translated


def choose_world(root: Path, args) -> tuple[str, Path]:
    selector = root / 'src' / 'nomad_gazebo' / 'scripts' / 'select_map.py'
    command = [sys.executable, str(selector)]
    if args.map:
        command.extend(('--select', args.map))
    if args.bare:
        command.append('--bare')
    result = subprocess.run(command, stdout=subprocess.PIPE, text=True, check=False)
    if result.returncode:
        raise ValueError('맵 선택을 완료하지 못했습니다.')
    world = Path(result.stdout.strip()).resolve()
    if not world.is_file():
        raise ValueError(f'맵 월드 파일이 없습니다: {world}')
    return world.parent.parent.name, world


def ensure_supervisor_token() -> str:
    token = os.environ.get('NOMAD_SEMANTIC_SUPERVISOR_TOKEN')
    if not token:
        environment = os.environ.copy()
        environment['NOMAD_SEMANTIC_SUPERVISOR_TOKEN'] = uuid.uuid4().hex
        # The token must be in the process's initial environment for /proc checks.
        os.execvpe(sys.executable, [sys.executable, *sys.argv], environment)
    return token


def simulator_partition_busy(partition: str) -> bool:
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal():
            continue
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            title = (directory / 'cmdline').read_bytes().split(b'\0', 1)[0]
            if title.startswith(b'gz sim ') and process_has_token(
                    int(directory.name), 'GZ_PARTITION', partition):
                return True
        except OSError:
            continue
    return False


def run_mapping(root: Path, args) -> int:
    supervisor_token = ensure_supervisor_token()
    lock = (root / '.semantic_mapping.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise ValueError('이 폴더의 주행 또는 저장된 지도 보기가 실행 중입니다. 해당 실행 창을 먼저 종료해 주세요.') from error
    partition = os.environ.get('GZ_PARTITION', '')
    if partition and simulator_partition_busy(partition):
        raise ValueError('같은 Gazebo 환경에서 시뮬레이션이 이미 실행 중입니다. 기존 실행을 먼저 종료해 주세요.')
    map_id, world = choose_world(root, args)
    sessions = root / 'sessions' / 'semantic'
    sessions.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
    directory = Path(tempfile.mkdtemp(prefix=f'{stamp}_{map_id}_', dir=sessions))
    (directory / 'logs' / 'ros').mkdir(parents=True)
    token = uuid.uuid4().hex
    command = [
        'ros2', 'launch', 'nomad_perception', 'semantic_mapping.launch.py',
        f'session_directory:={directory}', f'world_file:={world}', f'map_id:={map_id}',
        f'headless:={str(args.headless).lower()}', f'rviz:={str(not args.no_rviz).lower()}',
        f'vegetation:={str(not args.bare).lower()}',
    ]
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(directory / 'logs' / 'ros')
    environment['NOMAD_SEMANTIC_SESSION_TOKEN'] = token
    environment['NOMAD_STAGE1_PIXEL_OFFSET'] = '0.5'
    metadata = {
        'map_id': map_id, 'world_file': str(world), 'session_directory': str(directory),
        'started_at': datetime.now(timezone.utc).isoformat(),
        'ros_domain_id': environment.get('ROS_DOMAIN_ID'), 'gz_partition': partition,
        'supervisor_pid': os.getpid(), 'session_token': token,
        'headless': args.headless, 'rviz': not args.no_rviz, 'bare': args.bare,
        'command': command, 'state': 'starting',
    }
    metadata_path = directory / 'session.json'
    atomic_json(metadata_path, metadata)
    active_path = root / 'semantic_active_session.json'
    active = {
        'session_directory': str(directory), 'map_id': map_id,
        'supervisor_pid': os.getpid(), 'supervisor_start_ticks': process_identity(os.getpid()),
        'supervisor_token': supervisor_token,
    }
    interrupted = False

    def request_stop(signum, frame):
        nonlocal interrupted
        interrupted = True

    previous = {kind: signal.signal(kind, request_stop) for kind in (signal.SIGINT, signal.SIGTERM)}
    process = None
    launch_exit_code = 1
    final_status = None
    try:
        atomic_json(active_path, active)
        print(f'선택한 맵: {map_id}\n지도 저장 폴더: {directory}', flush=True)
        print('다른 터미널에서 이 폴더의 ./scripts/run_teleop.sh로 직접 주행하세요.', flush=True)
        print('주행 종료: Ctrl+C 또는 ros2 run nomad_perception semantic_session stop. 종료한 뒤 ros2 run nomad_perception semantic_session view로 지도를 다시 볼 수 있습니다.', flush=True)
        process = subprocess.Popen(command, cwd=directory, env=environment, start_new_session=True)
        metadata.update(state='running', launch_pid=process.pid)
        atomic_json(metadata_path, metadata)
        while process.poll() is None and not interrupted:
            time.sleep(.2)
        if interrupted:
            print('주행을 종료하고 생성된 지도를 저장하고 있습니다.', flush=True)
        stop_process(process, token)
        returned = process.returncode or 0
        launch_exit_code = 128 - returned if returned < 0 else returned
    finally:
        if process is not None and process.poll() is None:
            stop_process(process, token)
        final_status = archive_status(directory)
        metadata.update(finished_at=datetime.now(timezone.utc).isoformat(),
                        launch_exit_code=launch_exit_code,
                        exit_code=0 if final_status['final_save_complete'] else 1,
                        stop_requested=interrupted, **final_status)
        atomic_json(metadata_path, metadata)
        if read_json(active_path).get('supervisor_token') == supervisor_token:
            active_path.unlink(missing_ok=True)
        for kind, handler in previous.items():
            signal.signal(kind, handler)
        lock.close()
    print(f'주행 기록: {directory}', flush=True)
    if final_status['final_save_complete']:
        print('지도 최종 저장 완료. ros2 run nomad_perception semantic_session view로 확인하세요.' if final_status['has_data'] else
              '최종 저장은 완료됐지만 관측된 지도 셀이 없습니다. 실행 로그를 확인해 주세요.', flush=True)
    elif final_status['snapshot_file']:
        print('최종 저장이 완료되지 않았습니다. 복구 가능한 체크포인트는 보관했습니다. '
              'ros2 run nomad_perception semantic_session view로 마지막 저장 지도를 확인할 수 있습니다.', flush=True)
    else:
        print('지도 저장이 완료되지 않았습니다. 실행 로그는 주행 폴더에 보관했습니다.', flush=True)
    return 0 if final_status['final_save_complete'] else 1


def stop_mapping(root: Path) -> int:
    active = read_json(root / 'semantic_active_session.json')
    if not active_is_alive(active):
        print('이 폴더에서 실행 중인 지도 생성 주행이 없습니다.')
        return 1
    pid = int(active['supervisor_pid'])
    descriptor = os.pidfd_open(pid)
    try:
        # Recheck after acquiring the stable PID handle to prevent PID reuse.
        if not active_is_alive(active):
            raise ValueError('주행 프로세스가 이미 종료되었습니다.')
        signal.pidfd_send_signal(descriptor, signal.SIGINT)
    finally:
        os.close(descriptor)
    print('지도 저장 후 종료를 요청했습니다. 주행 터미널의 저장 완료 메시지를 기다려 주세요.')
    return 0


def list_sessions(root: Path) -> int:
    directories = run_directories(root)
    if not directories:
        print('저장된 주행 기록이 없습니다.')
        return 0
    active = read_json(root / 'semantic_active_session.json')
    for directory in reversed(directories):
        metadata = read_json(directory / 'session.json')
        live = str(directory) == active.get('session_directory') and active_is_alive(active)
        saved = archive_status(directory)
        state = ('주행 중' if live else '최종 저장' if saved['final_save_complete'] and saved['has_data']
                 else '복구 체크포인트' if saved['has_data'] else '관측 지도 없음')
        print(f'{directory.name} | {metadata.get("map_id", "unknown")} | {state}')
    return 0


def viewer_configuration(directory: Path, template: Path, destination: Path) -> Path:
    """Start close enough to read cell labels, centered on the saved vehicle."""
    import numpy as np
    import yaml
    from .archive import load_snapshot, resolve_snapshot
    arrays, metadata = load_snapshot(resolve_snapshot(directory))
    trajectory = arrays['trajectory']
    if len(trajectory):
        center_x, center_y = map(float, trajectory[-1, 1:3])
    else:
        rows, columns = np.nonzero(arrays['cost'] >= 0)
        if len(columns):
            center_x = metadata['origin_x'] + (int(columns.min()) + int(columns.max()) + 1) * .5 * metadata['resolution_m']
            center_y = metadata['origin_y'] + (int(rows.min()) + int(rows.max()) + 1) * .5 * metadata['resolution_m']
        else:
            center_x = metadata['origin_x'] + arrays['cost'].shape[1] * .5 * metadata['resolution_m']
            center_y = metadata['origin_y'] + arrays['cost'].shape[0] * .5 * metadata['resolution_m']
    configuration = yaml.safe_load(template.read_text(encoding='utf-8'))
    view = configuration['Visualization Manager']['Views']['Current']
    view.update({'Target Frame': metadata['frame_id'], 'Scale': 180,
                 'X': center_x, 'Y': center_y})
    temporary = destination.with_name(destination.name + '.tmp')
    temporary.write_text(yaml.safe_dump(configuration, sort_keys=False), encoding='utf-8')
    os.replace(temporary, destination)
    return destination


def view_session(root: Path, supplied: str | None) -> int:
    if active_is_alive(read_json(root / 'semantic_active_session.json')):
        raise ValueError('현재 주행을 먼저 종료해 주세요. 주행 중에는 실행 중인 RViz에서 지도를 볼 수 있습니다.')
    lock = (root / '.semantic_mapping.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock.close()
        raise ValueError('이 폴더의 주행 또는 저장된 지도 보기가 실행 중입니다. 해당 실행 창을 먼저 종료해 주세요.') from error
    directory = selected_session(root, supplied)
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('nomad_perception'))
    environment = os.environ.copy()
    token = uuid.uuid4().hex
    environment['NOMAD_SEMANTIC_SESSION_TOKEN'] = token
    viewer_log = directory / 'logs' / 'viewer'
    viewer_log.mkdir(parents=True, exist_ok=True)
    environment['ROS_LOG_DIR'] = str(viewer_log)
    rviz_configuration = viewer_configuration(
        directory, share / 'rviz' / 'semantic_saved_mapping.rviz', viewer_log / 'saved_mapping_runtime.rviz')
    commands = [
        ['ros2', 'run', 'nomad_perception', 'semantic_saved_map_viewer', '--ros-args',
         '-p', f'session_directory:={directory}', '-p', 'use_sim_time:=false'],
        ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(rviz_configuration),
         '--ros-args', '-p', 'use_sim_time:=false'],
        ['ros2', 'run', 'nomad_perception', 'semantic_labels', '--ros-args',
         '-p', 'use_sim_time:=false'],
    ]
    stopped = False

    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True

    previous = {kind: signal.signal(kind, request_stop) for kind in (signal.SIGINT, signal.SIGTERM)}
    processes = []
    print(f'저장된 지도 열기: {directory}\n닫기: RViz 창 닫기 또는 Ctrl+C', flush=True)
    try:
        for command in commands:
            processes.append(subprocess.Popen(command, cwd=directory, env=environment, start_new_session=True))
        while not stopped and all(process.poll() is None for process in processes):
            time.sleep(.2)
    finally:
        for process in processes:
            stop_process(process, token)
        for kind, handler in previous.items():
            signal.signal(kind, handler)
        lock.close()
    return 0


def main(arguments=None):
    arguments = list(sys.argv[1:] if arguments is None else arguments)
    try:
        root = workspace()
        operation = arguments.pop(0) if arguments and arguments[0] in ('view', 'list', 'stop') else 'run'
        if operation == 'view':
            parser = argparse.ArgumentParser(description='저장된 지도를 RViz로 다시 엽니다.')
            parser.add_argument('session', nargs='?')
            return view_session(root, parser.parse_args(arguments).session)
        if operation in ('list', 'stop'):
            if arguments:
                raise ValueError('이 명령에는 추가 인수가 필요하지 않습니다.')
            return list_sessions(root) if operation == 'list' else stop_mapping(root)
        parser = argparse.ArgumentParser(description='직접 주행하면서 의미 비용 지도와 셀별 대표 클래스 지도를 만듭니다.')
        parser.add_argument('--map', help='맵 번호 또는 전체 맵 폴더 이름')
        parser.add_argument('--headless', action='store_true', help='Gazebo 그래픽 창 없이 실행')
        parser.add_argument('--no-rviz', action='store_true', help='RViz 창 없이 실행')
        parser.add_argument('--bare', action='store_true', help='선택한 맵의 무식생 변형 사용')
        parser.add_argument('--list', action='store_true', help='사용할 수 있는 맵 목록 표시')
        args = parser.parse_args(normalized_arguments(arguments))
        if args.list:
            return subprocess.call([sys.executable, str(root / 'src/nomad_gazebo/scripts/select_map.py'), '--list'])
        return run_mapping(root, args)
    except (ValueError, OSError, ImportError) as error:
        print(f'실행할 수 없습니다: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
