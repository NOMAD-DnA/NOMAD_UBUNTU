"""Launch any module independently or the complete replaceable stack."""
from pathlib import Path
import fcntl
import os
import atexit
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from .configuration import load_topics,load_modules,remappings,selected_specs

_locks = []


def merged_parameter_file(files, overrides=None):
    """Resolve file precedence before ROS merges wildcard and named selectors.

    ROS can retain selector insertion order across files, letting an earlier
    wildcard override a later named node. Expand its values into each named
    selector in file order; emit the wildcard first in one canonical file.
    """
    common, nodes = {}, {}
    for filename in files:
        document = yaml.safe_load(Path(filename).read_text()) or {}
        wildcard = document.get('/**', {}).get('ros__parameters', {})
        common.update(wildcard)
        for values in nodes.values():
            values.update(wildcard)
        for selector, entry in document.items():
            if selector == '/**':
                continue
            values = entry.get('ros__parameters') if isinstance(entry, dict) else None
            if values is None:
                raise ValueError(f'Expected flat ROS parameter selector in {filename}: {selector}')
            nodes.setdefault(selector, dict(common)).update(values)
    if overrides:
        common.update(overrides)
        for values in nodes.values():
            values.update(overrides)
    from nomad_vehicle_model import profile
    if 'drive_layout' in common:
        profile(common)
        # The second file is the shared vehicle profile. Module-specific overrides
        # may tune behavior/limits, but cannot change simulator geometry alone.
        reference=yaml.safe_load(Path(files[1]).read_text()).get('/**',{}).get('ros__parameters',{}) if len(files)>1 else common
        geometry_keys=('drive_layout','steering_axle','wheelbase','track','wheel_radius','joint_limit','reference_offset')
        for values in [common,*nodes.values()]:
            for key in geometry_keys:
                if key in reference and values.get(key,reference[key])!=reference[key]:
                    raise ValueError(f'{key}: change shared vehicle_file, not a module-only override')
        for values in nodes.values():
            profile(values)
    document = {'/**': {'ros__parameters': common}}
    document.update({name: {'ros__parameters': values} for name, values in nodes.items()})
    with tempfile.NamedTemporaryFile(mode='w', prefix='nomad_parameters_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        filename = stream.name
    atexit.register(lambda: Path(filename).unlink(missing_ok=True))
    return filename


def start(context,module):
    value = lambda name: LaunchConfiguration(name).perform(context)
    topics,providers = load_topics(value('topics_file')),load_modules(value('modules_file'))
    legacy_command_topic = value("cmd_vel_topic").strip()
    if legacy_command_topic:
        import re
        if not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z_0-9]*)+", legacy_command_topic):
            raise ValueError("cmd_vel_topic must be an absolute ROS topic name")
        topics["control.simulator_command"] = legacy_command_topic
    algorithm_overrides = {}
    if module in ('all','planning'):
        from nomad_path_planning.gpp.registry import NAMES as GPP_NAMES
        from nomad_path_planning.lpp.registry import NAMES as LPP_NAMES
        for argument, parameter, choices in (('gpp','gpp_algorithm',GPP_NAMES),('lpp','lpp_algorithm',LPP_NAMES)):
            choice=value(argument).strip()
            if choice:
                if choice not in choices:
                    raise ValueError(f'Unknown {argument}={choice!r}; choose {choices}')
                algorithm_overrides[parameter]=choice
    specs = selected_specs(providers,module)
    domain = os.getenv('ROS_DOMAIN_ID','0')
    # All-stack and single-module launches share locks, so they cannot duplicate nodes.
    roles = {'nomad_perception':'perception','nomad_vio':'vio',
             'nomad_control':'control','nomad_path_planning':'planning'}
    for name in sorted({roles[package] for package,_,_ in specs}):
        lock = open(f'/tmp/nomad_module_{os.getuid()}_{domain}_{name}.lock','a')
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock.close()
            raise RuntimeError(f'{name} already launched in ROS domain {domain}') from exc
        _locks.append(lock)
    nodes = []
    override = value('params_file').strip()
    for package,executable,role in specs:
        share = Path(get_package_share_directory(package))
        config = 'planning.yaml' if package=='nomad_path_planning' else 'parameters.yaml'
        params = [str(share/'config'/config),value('vehicle_file')]
        if override:
            params.append(override)
        final_overrides = {}
        if package == 'nomad_perception' and value('simulator_grass_world').strip():
            if providers['vio'] != 'builtin' or providers['tf_owner'] != 'gazebo':
                raise ValueError('Simulator grass oracle requires builtin VIO and gazebo TF ownership')
            final_overrides['terrain.simulator_grass_world'] = value('simulator_grass_world')
        if role in ('gpp','lpp') and algorithm_overrides:
            final_overrides.update(algorithm_overrides)
        params = [merged_parameter_file(params, final_overrides)]
        nodes.append(Node(package=package,executable=executable,output='screen',
                          parameters=params,remappings=remappings(topics,role),
                          on_exit=Shutdown(reason=f'{package}/{executable} exited')))
    if module in ('all','planning') and value('rviz').lower()=='true':
        share = Path(get_package_share_directory('nomad_path_planning'))
        nodes.append(Node(package='rviz2',executable='rviz2',
                          arguments=['-d',str(share/'rviz/planning.rviz')],
                          parameters=[value('vehicle_file')],remappings=remappings(topics)))
    return nodes


def description(module='all'):
    share = Path(get_package_share_directory('nomad_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('topics_file',default_value=str(share/'config/topics.yaml')),
        DeclareLaunchArgument('modules_file',default_value=str(share/'config/modules.yaml')),
        DeclareLaunchArgument('vehicle_file',default_value=str(share/'config/vehicle.yaml')),
        DeclareLaunchArgument('params_file',default_value='',description='Optional ROS parameter overrides'),
        DeclareLaunchArgument('simulator_grass_world',default_value='',description='Simulation-only authored grass oracle SDF'),
        DeclareLaunchArgument('gpp',default_value='',description='GPP strategy; empty uses planning.yaml/params_file'),
        DeclareLaunchArgument('lpp',default_value='',description='LPP strategy; empty uses planning.yaml/params_file'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic',default_value='',description='Legacy actuator-output override; prefer topics_file'),
        OpaqueFunction(function=lambda context:start(context,module)),
    ])
