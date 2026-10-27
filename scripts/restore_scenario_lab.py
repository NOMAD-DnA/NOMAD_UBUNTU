#!/usr/bin/env python3
"""Verify and restore the committed planning-lab snapshot to a new directory."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, default=ROOT/'datasets/planning_lab/20261027')
    parser.add_argument('--output', type=Path, default=ROOT/'sessions/planning_lab_archives/20261027')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((args.snapshot/'manifest.json').read_text())
    archive = args.snapshot/manifest['archive']
    if hashlib.sha256(archive.read_bytes()).hexdigest() != manifest['archive_sha256']:
        parser.error('Archive checksum mismatch')
    expected = {e['path']: e for e in manifest['files']}
    with tarfile.open(archive, 'r:*') as tar:
        members = tar.getmembers()
        if len(members) != len(expected) or {m.name for m in members} != set(expected):
            parser.error('Archive file inventory mismatch')
        for member in members:
            path = PurePosixPath(member.name)
            if not member.isfile() or path.is_absolute() or '..' in path.parts:
                parser.error(f'Unsupported archive entry: {member.name}')
            data = tar.extractfile(member).read()
            entry = expected[member.name]
            if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
                parser.error(f'File checksum mismatch: {member.name}')
        print(f"Verified: {manifest['queue_count']} queue records, {manifest['all_results']} outcomes, {len(members)} files")
        if args.verify_only:
            return
        output = args.output.resolve()
        if output.exists():
            parser.error(f'Output already exists; choose a new directory: {output}')
        output.mkdir(parents=True)
        for member in members:
            target = output/member.name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(tar.extractfile(member).read())
    # Historical metadata/logs remain original. Only GUI queue folder pointers
    # move to this checkout; the immutable archive preserves the original queue.
    for queue in output.rglob('queue.json'):
        jobs = json.loads(queue.read_text())
        for job in jobs:
            directory = job.get('directory')
            if directory:
                try:
                    relative = Path(directory).relative_to(manifest['source_root'])
                except ValueError:
                    continue
                job['directory'] = str(output/relative)
        queue.write_text(json.dumps(jobs, ensure_ascii=False, indent=2)+'\n')
    print(f'Restored to: {output}')
    print(f'Open: ./scripts/run_scenario_lab.sh --root {output}')


if __name__ == '__main__':
    main()
