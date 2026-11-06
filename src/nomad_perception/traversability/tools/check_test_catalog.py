"""Reject an empty or incomplete CTest run before executing the native tests."""
import json
import subprocess

catalog = json.loads(subprocess.check_output(
    ['ctest', '--show-only=json-v1'], universal_newlines=True))
names = {item['name'] for item in catalog.get('tests', [])}
required = {'native_runtime', 'native_ros_contract', 'native_event_dispatch'}
if not required.issubset(names):
    raise SystemExit('Required native tests are missing: ' + ', '.join(sorted(required - names)))
print('CTest discovered', len(names), 'tests including all three native contract tests.')
