"""Build-time source integrity check. No sensor or runtime is started."""
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
manifest=root/'source_manifest.json'
if not manifest.exists():
    raise SystemExit('source_manifest.json is missing; use the complete verified source bundle')
value=json.loads(manifest.read_text())
for name,expected in value['files'].items():
    path=Path(name)
    if path.is_absolute() or '..' in path.parts or hashlib.sha256((root/path).read_bytes()).hexdigest()!=expected:
        raise SystemExit('Source integrity mismatch: '+name)
print('Verified',len(value['files']),'source files')
