import json
from concurrent.futures import ThreadPoolExecutor
import pytest
from nomad_operator_gui.attempt_log import AttemptLog, classify_node
from nomad_operator_gui import remote


def rows(folder):
    return [json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines()]


def test_opt_in_and_separate_attempts(tmp_path):
    root = tmp_path / 'logs'
    log = AttemptLog(root)
    log.write('runtime', 'ignored')
    log.flush()
    assert not root.exists()
    a = log.start(['planning'], {'ros_domain_id': '213'})
    log.write('runtime', 'excluded')
    log.write('planning', 'goal_published', x=3.0, y=4.0)
    log.stop('user')
    b = log.start(['runtime'], {})
    log.write('runtime', 'second')
    log.stop('gui_closed')
    assert a != b
    assert [r['event'] for r in rows(a)] == ['goal_published']
    assert [r['event'] for r in rows(b)] == ['second']
    manifest = json.loads((a / 'manifest.json').read_text())
    assert manifest['ended_at'] and manifest['status'] == 'finished'
    assert manifest['events'] == 1


def test_concurrent_ros_and_board_events(tmp_path):
    log = AttemptLog(tmp_path)
    folder = log.start(['runtime'], {})
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(lambda n: log.write('runtime', 'message', sequence=n), range(400)))
    log.board_event({'board': 'Pi 5', 'result': {'run_id': 'trial-5', 'state': 'STARTING'}})
    log.flush()
    assert len(rows(folder)) == 401
    log.stop()
    assert json.loads((folder / 'manifest.json').read_text())['board_runs'] == {'Pi 5': 'trial-5'}


def test_disk_error_does_not_escape_into_control(tmp_path):
    log = AttemptLog(tmp_path)
    folder = log.start(['runtime'], {})
    log.stream.close()
    class FullDisk:
        def write(self, _): raise OSError('disk full')
        def close(self): pass
    log.stream = FullDisk()
    log.write('runtime', 'board_start')
    assert not log.active and log.error == 'disk full'
    assert json.loads((folder / 'manifest.json').read_text())['status'] == 'error'
    log.stop()


def test_failed_start_and_rollback_are_recorded(tmp_path, monkeypatch):
    log = AttemptLog(tmp_path)
    folder = log.start(['runtime'], {})
    boards = [remote.Board('Pi 5', 'host5', 'user', '/start'),
              remote.Board('Jetson', 'hostj', 'user', '/start')]
    calls = []
    def fake(board, action, domain, run_id, actuation):
        calls.append((board.name, action))
        if board.name == 'Jetson': raise remote.RemoteError('camera unavailable')
        return {'state': 'STARTING' if action == 'start' else 'STOPPED', 'run_id': 'owned5'}
    monkeypatch.setattr(remote, '_invoke', fake)
    owned = {}
    with pytest.raises(remote.RemoteError, match='camera unavailable'):
        remote.start_sequence(boards, '213', lambda _: None, owned, log.board_event)
    log.stop()
    assert not owned
    assert calls == [('Pi 5', 'start'), ('Jetson', 'start'), ('Pi 5', 'stop')]
    assert rows(folder)[1]['data']['error'] == 'camera unavailable'
    assert rows(folder)[2]['data']['result']['state'] == 'STOPPED'


@pytest.mark.parametrize('node,part', [('oak', 'perception'), ('basalt', 'localization'),
    ('command_selector', 'planning'), ('vesc_controller', 'control'),
    ('ros_gz_bridge', 'simulation'), ('unknown_node', 'runtime')])
def test_node_categories(node, part):
    assert classify_node(node) == part
