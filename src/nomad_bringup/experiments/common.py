"""Experiment files and safety semantics, independent of planner implementations."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

WORKSPACE = Path(__file__).resolve().parents[3]
ROOT = WORKSPACE / 'experiments'
MAPS = ('04_vio_flat_loop', '07_vio_undulating_terrain', '08_vio_reverse_dead_end')


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(dir=path.parent, prefix='.writing-')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        Path(temp).unlink(missing_ok=True)


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def source_files():
    files = [WORKSPACE/'scripts/run_autonomy.sh', WORKSPACE/'scripts/run_experiment.sh', WORKSPACE/'.nomad/env.sh']
    for package in sorted((WORKSPACE/'src').glob('nomad_*')):
        if package.name == 'nomad_sim':
            continue
        for path in sorted(package.rglob('*')):
            if path.is_file() and not {'__pycache__', '.pytest_cache'}.intersection(path.parts):
                if package.name == 'nomad_gazebo' and 'maps' in path.parts:
                    if not any(mid in path.parts for mid in __import__('batch_missions').ALL_MAPS):
                        continue
                files.append(path)
    return files


def fingerprint():
    return {str(p.relative_to(WORKSPACE)): sha(p) for p in source_files()}


def contact_classification(first, second):
    """Only wheel/terrain contacts are allowed. Names must be scoped Gazebo names."""
    a, b = first.split('::'), second.split('::')
    av, bv = 'nomad_vehicle' in a, 'nomad_vehicle' in b
    if not av and not bv:
        return 'unresolved'  # A vehicle sensor must identify its vehicle collision.
    if av and bv:
        return 'self_contact'
    vehicle, other = (a, b) if av else (b, a)
    wheels = {'front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'}
    ground_models = {'terrain', 'test_flat_ground'} | {f'ramp_{a:02d}_deg' for a in (5,10,15,20,25,30)}
    if wheels.intersection(vehicle) and ground_models.intersection(other):
        return 'wheel_ground'
    return 'forbidden'


def make_world(original, destination):
    """Instrumentation-only SDF copy; preserve every physical element and asset URI."""
    original, destination = Path(original), Path(destination)
    tree = ET.parse(original)
    world = tree.getroot().find('world')
    vehicle = world.find("model[@name='nomad_vehicle']")
    if vehicle is None:
        include = next((i for i in world.findall('include') if i.findtext('name') == 'nomad_vehicle'), None)
        if include is None:
            raise ValueError('No nomad_vehicle model or include in world')
        reference = include.findtext('uri', '')
        if not reference or '://' in reference:
            raise ValueError('Vehicle include must reference a local SDF')
        if any(c.tag not in ('uri', 'name', 'pose', 'static') for c in include) or include.attrib:
            raise ValueError('Unsupported vehicle include override')
        model_path = (original.parent/reference).resolve()
        vehicle = ET.parse(model_path).getroot().find('model')
        if vehicle is None:
            raise ValueError('Vehicle SDF has no model')
        for uri in vehicle.iter('uri'):
            text = (uri.text or '').strip()
            if text and '://' not in text and not Path(text).is_absolute():
                asset = (model_path.parent/text).resolve()
                if not asset.exists():raise ValueError(f'Missing vehicle asset: {asset}')
                uri.text = str(asset)
        vehicle.set('name', 'nomad_vehicle')
        for tag in ('pose', 'static'):
            override = include.find(tag)
            if override is not None:
                existing = vehicle.find(tag)
                if existing is not None:vehicle.remove(existing)
                vehicle.append(copy.deepcopy(override))
        world.remove(include)
        world.append(vehicle)
    for uri in tree.getroot().iter('uri'):
        text = (uri.text or '').strip()
        if text and '://' not in text and not Path(text).is_absolute():
            resolved = (original.parent/text).resolve()
            if not resolved.exists():
                raise ValueError(f'Missing asset: {resolved}')
            uri.text = str(resolved)
    if not any(p.get('name') == 'gz::sim::systems::Contact' for p in world.findall('plugin')):
        ET.SubElement(world, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    topics = []
    for link in vehicle.findall('link'):
        collisions = link.findall('collision')
        if not collisions:
            continue
        topic = '/nomad/experiment/contacts/' + link.get('name')
        topics.append(topic)
        sensor = ET.SubElement(link, 'sensor', name='experiment_contacts', type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '50'
        ET.SubElement(sensor, 'topic').text = topic
        contact = ET.SubElement(sensor, 'contact')
        # Harmonic Contact reads contact/topic, not the generic sensor/topic.
        ET.SubElement(contact, 'topic').text = topic
        for collision in collisions:
            ET.SubElement(contact, 'collision').text = collision.get('name')
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding='unicode', xml_declaration=True)
    return topics


def goal_for(mid):
    # Opposite side of the loop for flat/rough; behind the dead end for map 8.
    return [34., 0.] if mid.startswith('08') else [28., 0.]


def metric_values(outcome, reference=None):
    """Unverified reference values NEVER yield SPL/SCT or a composite score."""
    result = dict(safe_success=None, spl=None, sct=None, score=None)
    if outcome.get('valid'):
        result['safe_success'] = int(outcome.get('result') == 'success')
    if result['safe_success'] is None or not reference or not reference.get('validated'):
        return result
    length, duration = reference.get('shortest_distance_m'), reference.get('minimum_time_s')
    if not all(isinstance(x,(int,float)) and math.isfinite(x) and x>0 for x in (length,duration)):
        return result
    for key, optimal, actual in [('spl',length,outcome['distance_xy_m']),('sct',duration,outcome['duration_sim_s'])]:
        result[key] = result['safe_success']*optimal/max(optimal,actual)
    return result
