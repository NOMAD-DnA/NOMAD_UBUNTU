"""Opt-in attempt logs; independent of Qt/ROS and safe across worker threads."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import uuid

PARTS = {'runtime': '실행 관리', 'localization': '위치추정', 'perception': '인지',
         'planning': '플래닝', 'control': '제어', 'simulation': '시뮬레이션'}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def classify_node(name):
    name = name.lower()
    for part, words in (
        ('simulation', ('gazebo', 'simulation', 'ros_gz')),
        ('control', ('control', 'vesc', 'watchdog', 'steering')),
        ('planning', ('planning', 'planner', 'gpp', 'lpp', 'rpp', 'command_selector', 'recovery')),
        ('localization', ('localization', 'vio', 'basalt', 'openvins', 'gnss', 'gps', 'fusion', 'ekf')),
        ('perception', ('perception', 'semantic', 'oak', 'depthai', 'lidar', 'costmap', 'terrain')),
    ):
        if any(word in name for word in words):
            return part
    return 'runtime'


def default_log_root():
    # The wrapper supplies the source package directory, also for non-symlink installs.
    configured = os.environ.get('NOMAD_OPERATOR_GUI_LOG_DIR')
    return Path(configured) if configured else Path(__file__).resolve().parent.parent / 'logs'


class AttemptLog:
    def __init__(self, root=None):
        self.root = Path(root) if root is not None else default_log_root()
        self.lock = threading.RLock()
        self.stream = None
        self.directory = None
        self.manifest = {}
        self.error = ''

    @property
    def active(self):
        with self.lock:
            return self.stream is not None

    def _manifest(self):
        temp = self.directory / 'manifest.json.tmp'
        temp.write_text(json.dumps(self.manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temp.replace(self.directory / 'manifest.json')

    def start(self, parts, context):
        with self.lock:
            if self.stream is not None:
                raise RuntimeError('이미 기록 중입니다')
            parts = sorted(set(parts))
            if not parts or set(parts) - PARTS.keys():
                raise ValueError('기록할 파트를 선택하세요')
            attempt_id = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8]
            self.directory = self.root / attempt_id
            self.directory.mkdir(parents=True, exist_ok=False)
            self.error = ''
            self.manifest = {'schema_version': 1, 'attempt_id': attempt_id,
                             'started_at': utc_now(), 'ended_at': None, 'status': 'recording',
                             'selected_parts': parts, 'events': 0, 'context': context,
                             'pid': os.getpid(), 'board_runs': {}}
            self._manifest()
            try:
                self.stream = (self.directory / 'events.jsonl').open('x', encoding='utf-8', buffering=65536)
            except OSError as exc:
                self.manifest.update(status='error', ended_at=utc_now(), error=str(exc))
                self._manifest()
                raise
            return self.directory

    def write(self, part, event, **data):
        with self.lock:
            if self.stream is None or part not in self.manifest['selected_parts']:
                return
            try:
                row = dict(at=utc_now(), part=part, event=event, data=data)
                self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                self.manifest['events'] += 1
            except (OSError, ValueError, TypeError) as exc:
                self._fail(exc)

    def board_event(self, event):
        with self.lock:
            payload = event.get('result', {})
            if self.stream is not None and payload.get('run_id'):
                self.manifest['board_runs'][event['board']] = payload['run_id']
            self.write('runtime', 'board_response', **event)

    def _fail(self, exc):
        self.error = str(exc)
        stream, self.stream = self.stream, None
        if stream is not None:
            try:
                stream.close()
            except OSError:
                pass
        self.manifest.update(status='error', ended_at=utc_now(), error=self.error)
        try:
            self._manifest()
        except OSError:
            pass  # Disk full may also prevent metadata writes; the GUI shows the error.

    def flush(self):
        with self.lock:
            if self.stream is not None:
                try:
                    self.stream.flush()
                    self._manifest()
                except OSError as exc:
                    self._fail(exc)

    def stop(self, reason='user'):
        with self.lock:
            if self.stream is None:
                return
            try:
                self.stream.flush()
                self.stream.close()
                self.stream = None
                self.manifest.update(status='finished', ended_at=utc_now(), stop_reason=reason)
                self._manifest()
            except OSError as exc:
                self._fail(exc)
