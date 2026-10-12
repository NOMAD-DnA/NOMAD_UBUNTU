"""Lossless raw-record inventory, checksums and optional open-format export."""
import csv
import gzip
import json
from collections import Counter
from pathlib import Path
import rosbag2_py
import yaml
from common import save, sha, read

EXCLUDED_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')


def reader(path):
    result = rosbag2_py.SequentialReader()
    result.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return result


def verify(attempt):
    """Read every saved message; compare counts and retain type/time manifest."""
    attempt = Path(attempt)
    info = yaml.safe_load((attempt/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    stream = reader(attempt/'bag')
    types = {t.name: t.type for t in stream.get_all_topics_and_types()}
    counts, first, last = Counter(), {}, {}
    while stream.has_next():
        topic, data, stamp = stream.read_next()
        counts[topic] += 1
        first.setdefault(topic, stamp)
        last[topic] = stamp
    expected = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
    mismatches = {t: [n, counts[t]] for t, n in expected.items() if n != counts[t]}
    unexpected_images = [t for t in types if types[t] in EXCLUDED_TYPES and counts[t]]
    with (attempt/'topics.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['topic', 'type', 'messages', 'first_bag_sim_ns', 'last_bag_sim_ns'])
        writer.writerows((t, types[t], counts[t], first.get(t), last.get(t)) for t in sorted(types))
    inventory = read(attempt/'topic_inventory.json') if (attempt/'topic_inventory.json').exists() else []
    missing = [t['name'] for t in inventory if t['publishers'] and not set(t['types']).intersection(EXCLUDED_TYPES) and not counts[t['name']]]
    required = ('/odom', '/clock', '/nomad/goal', '/nomad/control/vehicle_state', '/nomad/planning/drive_command')
    missing_required = [t for t in required if not counts[t]]
    result = dict(readable=True, message_count=sum(counts.values()), count_mismatches=mismatches,
                  unexpected_images=unexpected_images, advertised_but_empty=missing,
                  missing_required=missing_required,
                  valid=not mismatches and not unexpected_images and not missing_required,
                  note='Readability/count checks cannot prove no DDS loss before recording. Topic inventory distinguishes unpublished outputs.')
    save(attempt/'archive_audit.json', result)
    return result


def checksum(attempt):
    attempt = Path(attempt)
    files = [p for p in attempt.rglob('*') if p.is_file() and p.name != 'checksums.sha256']
    with (attempt/'checksums.sha256').open('w') as f:
        for p in sorted(files): f.write(f'{sha(p)}  {p.relative_to(attempt)}\n')


def export(directory, topic):
    """Explicit topic export: all message fields retained in gzipped JSONL."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    directory = Path(directory)
    for attempt in sorted((directory/'runs').glob('*/attempt_*')):
        if not (attempt/'bag/metadata.yaml').exists(): continue
        r = reader(attempt/'bag')
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        if topic not in types: continue
        r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        cls = get_message(types[topic])
        output = directory/'exports'/attempt.parent.name/attempt.name
        output.mkdir(parents=True, exist_ok=True)
        target = output/(topic.strip('/').replace('/', '__')+'.jsonl.gz')
        with gzip.open(target, 'wt') as f:
            while r.has_next():
                name, data, stamp = r.read_next()
                msg = message_to_ordereddict(deserialize_message(data, cls))
                f.write(json.dumps(dict(topic=name, type=types[name], bag_sim_ns=stamp, message=msg), ensure_ascii=False)+'\n')
        print(target)
