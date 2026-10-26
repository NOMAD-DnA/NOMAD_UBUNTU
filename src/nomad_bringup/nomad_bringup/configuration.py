"""Single source of editable ROS topic names and module provider selection."""
from pathlib import Path
import re
import yaml
from .topic_defaults import DEFAULT_TOPICS


def flatten(mapping):
    return {f'{section}.{key}':value for section,items in mapping.items() for key,value in items.items()}


def load_topics(filename):
    doc = yaml.safe_load(Path(filename).read_text())
    if not isinstance(doc,dict) or not isinstance(doc.get('topics'),dict):
        raise ValueError('topics_file must contain a topics mapping')
    try:
        topics = flatten(doc['topics'])
    except (TypeError,AttributeError) as exc:
        raise ValueError('Invalid topic sections') from exc
    expected = flatten(DEFAULT_TOPICS)
    if set(topics) != set(expected):
        raise ValueError(f'Topic keys differ: missing={set(expected)-set(topics)}, extra={set(topics)-set(expected)}')
    for key,value in topics.items():
        if not isinstance(value,str) or not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z_0-9]*)+',value):
            raise ValueError(f'Invalid absolute topic name: {key}={value!r}')
    if len(set(topics.values())) != len(topics):
        raise ValueError('Topic aliases must be unique; duplicate publishers/input-output loops are not allowed')
    return topics


def load_modules(filename):
    data = yaml.safe_load(Path(filename).read_text())
    if not isinstance(data,dict) or set(data) != {'perception','vio','control','tf_owner'}:
        raise ValueError('modules_file needs perception, vio, control and tf_owner')
    if any(data[k] not in ('builtin','external') for k in ('perception','vio','control')):
        raise ValueError('Provider must be builtin or external')
    expected = 'gazebo' if data['vio']=='builtin' else 'vio'
    if data['tf_owner'] != expected:
        raise ValueError(f"VIO provider {data['vio']} requires tf_owner={expected}; one TF owner only")
    return data


def remappings(topics, role=None):
    defaults = flatten(DEFAULT_TOPICS)
    result = {value:topics[key] for key,value in defaults.items()}
    if role == 'gpp':
        result['/nomad/costmap'] = topics['perception.global_costmap']
    elif role in ('lpp','command'):
        result['/nomad/costmap'] = topics['perception.local_costmap']
    elif role == 'recovery':
        result['/nomad/local_costmap'] = topics['perception.local_costmap']
    return list(result.items())


SPECS = {
    'perception':[('nomad_perception','perception','perception')],
    'vio':[('nomad_vio','sim_vio','vio')],
    'planning':[('nomad_path_planning','planning_inputs','inputs'),
                ('nomad_path_planning','dstar_lite_gpp','gpp'),
                ('nomad_path_planning','ackermann_lpp','lpp'),
                ('nomad_path_planning','recovery_supervisor','recovery'),
                ('nomad_path_planning','planning_command','command')],
    'control':[('nomad_control','controller','control')],
}


def selected_specs(providers, module='all'):
    if module not in ('all',*SPECS):
        raise ValueError('Unknown module selection')
    return [spec for name,specs in SPECS.items()
            if module in ('all',name) and (name=='planning' or providers[name]=='builtin')
            for spec in specs]
