"""Existing/parallel slope selection using the installed external controller."""
import json
from pathlib import Path
from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
MODES = {'sequential': '기존 방식 (1스레드)', 'parallel': '병렬 방식 (3스레드)'}
RESULT_MARKER = 'NOMAD_SLOPE_RESULT='


def switch_slope_mode(host, mode, password=None):
    failed = {'ok': False, 'state': 'UNCONFIRMED', 'mode': None, 'map_reset': False}
    if mode not in (*MODES, 'status'):
        return {**failed, 'reason': 'invalid_mode'}
    try:
        host = validate_host(host)
        if host.split('@')[0] != 'nvidia': raise ValueError('account')
        result = run_remote(host, BASE_DIR / 'remote' / ('switch_slope_' + mode + '.sh'),
                            password, timeout_s=22)
        lines = [line[len(RESULT_MARKER):] for line in result.get('stdout', '').splitlines()
                 if line.startswith(RESULT_MARKER)]
        if len(lines) != 1: return {**failed, 'reason': 'missing_or_ambiguous_result'}
        value = json.loads(lines[0])
        if not isinstance(value, dict) or value.get('map_reset') is not False:
            raise ValueError('contract')
        actual = value.get('mode')
        if actual is not None and actual not in MODES: raise ValueError('mode')
        if value.get('ok') is True:
            if result.get('ok') is not True: raise ValueError('transport')
            if mode != 'status' and (actual != mode or value.get('state') not in ('MODE_CONFIRMED', 'SAVED_FOR_NEXT_START')):
                raise ValueError('unconfirmed')
        return value
    except Exception:
        return {**failed, 'reason': 'slope_control_failed'}
