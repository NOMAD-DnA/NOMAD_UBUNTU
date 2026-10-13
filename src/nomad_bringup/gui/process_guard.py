"""Detect conflicting local modules and reap children after a launch parent exits."""
import os
import signal
from pathlib import Path


def resource_set(args):
    names={Path(arg).name for arg in args}
    resources=set()
    if 'terrain_navigation.launch.py' in names or 'autonomy.launch.py' in names:
        return {'sim','localization','semantic','map','planning','control'}
    for resource,executables in (
        ('localization',{'fusion_node','fusion_adapter','vio_supervisor.py'}),
        ('map',{'navigation_terrain'}),
        ('planning',{'planning_inputs','dstar_lite_gpp','ackermann_lpp','recovery_supervisor','planning_command'}),
        ('control',{'controller'}),('teleop',{'teleop.py'}),
    ):
        if names & executables:resources.add(resource)
    if 'nomad_semantic.node' in args:resources.add('semantic')
    if 'parameter_bridge' in names and any('__node:=nomad_gz_bridge' in a for a in args):resources.add('sim')
    return resources


def conflicts(resources,domain,proc=Path('/proc')):
    result=[]
    wanted=set(resources)-{'rviz'}
    for folder in proc.iterdir():
        if not folder.name.isdigit() or int(folder.name)==os.getpid():continue
        try:
            args=folder.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
            overlap=wanted & resource_set(args)
            if not overlap:continue
            env=folder.joinpath('environ').read_bytes().split(b'\0')
            if f'ROS_DOMAIN_ID={domain}'.encode() not in env:continue
            result.append((int(folder.name),sorted(overlap)))
        except (OSError,ProcessLookupError):continue
    return result


def group_members(group,proc=Path('/proc')):
    if not group:return []
    result=[]
    for folder in proc.iterdir():
        if not folder.name.isdigit():continue
        try:
            fields=folder.joinpath('stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2])==group and int(folder.name)!=os.getpid():result.append(int(folder.name))
        except (OSError,ValueError,IndexError):continue
    return result


def signal_children(group,sig):
    for pid in group_members(group):
        try:os.kill(pid,sig)
        except ProcessLookupError:pass
