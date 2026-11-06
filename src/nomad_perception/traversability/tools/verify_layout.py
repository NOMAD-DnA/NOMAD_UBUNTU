"""Build-time integrity check of this delivery, including the frozen source."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'deployment_manifest.json').read_text())
for name, expected in manifest['files'].items():
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts:
        raise SystemExit('Unsafe delivery manifest path: ' + name)
    path = root / relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise SystemExit('Delivery integrity mismatch: ' + name)
print('Verified', len(manifest['files']), 'delivery files;', manifest['layout_revision'])
