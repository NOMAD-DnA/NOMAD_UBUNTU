"""ROS console attribution and display filtering; never drop saved lines."""
import re
ANSI=re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
ROS=re.compile(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]\s*\[[^\]]+\]\s*\[([^\]]+)\]')


def entry(line,source='GUI'):
    text=ANSI.sub('',line)
    match=ROS.search(text)
    if match:
        level,node=match.groups()
    else:
        level_match=re.search(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL|Wrn|Err)\]',text)
        level=level_match.group(1) if level_match else 'OTHER'
        wrappers=re.findall(r'\[([^\]]+)\]',text)
        node=next((re.sub(r'-\d+$','',name) for name in wrappers if re.search(r'-\d+$',name)),source)
        short=re.search(r'\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*\[([A-Za-z_][A-Za-z0-9_.]*)\]',text)
        if short:node=short[1]
    level={'WARNING':'WARN','Wrn':'WARN','Err':'ERROR'}.get(level,level)
    return {'node':node,'level':level,'text':text,'source':source}


def matches(item,node='',level='',query='',group=''):
    if group and category(item)!=group:return False
    if node and item['node']!=node:return False
    if level=='WARN+' and item['level'] not in ('WARN','ERROR','FATAL'):return False
    if level=='ERROR+' and item['level'] not in ('ERROR','FATAL'):return False
    if level and level not in ('WARN+','ERROR+') and item['level']!=level:return False
    return not query or query.casefold() in item['text'].casefold()


GROUPS = ('시뮬레이션', '위치추정', '인지', '플래닝', '제어', '실행 관리')


def category(item):
    name = item['node'].lower()
    for group, keys in (
        ('위치추정', ('fusion', 'vio', 'basalt', 'openvins', 'gps', 'gnss', 'estimator', 'component_container', 'localization', '위치추정')),
        ('인지', ('terrain', 'semantic', 'segmentation', 'perception', '인지', '지도')),
        ('플래닝', ('planning', 'planner', 'gpp', 'lpp', 'rollout', 'rpp', 'navigation_start', '플래닝')),
        ('제어', ('control', 'watchdog', 'teleop', '제어', '조종')),
        ('시뮬레이션', ('gazebo', 'gz_', 'bridge', 'sensor', 'robot_state', 'rviz', '시뮬레이션')),
    ):
        if any(key in name for key in keys):
            return group
    # The semantic launcher is an ExecuteProcess rather than a named ROS node.
    if name == 'run.sh':
        return '인지'
    for group in GROUPS:
        if '['+group+']' in item['text']:return group
    return '실행 관리'
