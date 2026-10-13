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


def matches(item,node='',level='',query=''):
    if node and item['node']!=node:return False
    if level=='WARN+' and item['level'] not in ('WARN','ERROR','FATAL'):return False
    if level=='ERROR+' and item['level'] not in ('ERROR','FATAL'):return False
    if level and level not in ('WARN+','ERROR+') and item['level']!=level:return False
    return not query or query.casefold() in item['text'].casefold()
