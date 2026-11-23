"""One read-only SSH stream owned by the operator's main monitor."""
import json
import math
import time
from module_log_client import ModuleLogClient, MAX_LINE


def valid_status(value):
    if not isinstance(value, dict) or value.get('kind') != 'slope_status' or value.get('schema') != 1:
        return False
    stamp = value.get('t')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
        return False
    if value.get('active_mode') not in (None, 'sequential', 'parallel'):
        return False
    cpu = value.get('cpu')
    if not isinstance(cpu, dict): return False
    percent = cpu.get('percent_one_core')
    if percent is not None and (isinstance(percent, bool) or not isinstance(percent, (int, float)) or
                                not math.isfinite(percent) or not 0 <= percent <= 100000):
        return False
    if cpu.get('state') == 'live' and (percent is None or value.get('active_mode') is None): return False
    return True


class SlopeCpuClient(ModuleLogClient):
    def __init__(self, host, runtime_dir, password=None, parent=None):
        super().__init__('jetson', host, runtime_dir, parent)
        self.password = password

    def remote_command(self):
        return 'exec nice -n 15 python3 /home/nvidia/nomad_ws/tools/visual_trial_20261106/slope_control/slope_control.py watch'

    def credential_environment(self):
        env = super().credential_environment()
        if self.password: env['NOMAD_BOARD_PASSWORD1'] = self.password
        return env

    def stop(self):
        stopped = super().stop()
        self.password = None
        return stopped

    def _accept_line(self, raw, arrived=None):
        try:
            if len(raw) > MAX_LINE: raise ValueError('oversize')
            value = json.loads(raw)
            if not valid_status(value): raise ValueError('status')
            event = {**value, 'kind': 'telemetry', 'source': 'slope_cpu',
                     '_source_session': self._error_session}
        except (ValueError, TypeError, UnicodeError):
            self.invalid_events += 1
            return False
        self._enqueue(event, arrived)
        return True

    def _failure(self):
        self._enqueue({'kind': 'telemetry', 'source': 'slope_cpu', 't': time.time(),
                       '_source_session': self._error_session, 'state': 'connection_failed',
                       'active_mode': None, 'cpu': {'state': 'connection_failed', 'percent_one_core': None}})
