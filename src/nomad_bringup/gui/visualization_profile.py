"""Use existing RViz with only the displays relevant to the selected module."""
from pathlib import Path
import yaml


def rviz_profile(root, menu, directory):
    if directory is None:
        return '<이번 실행/visualization.rviz>'
    data = yaml.safe_load((root / 'src/nomad_perception/rviz/terrain_navigation.rviz').read_text())
    manager = data['Visualization Manager']
    displays = manager['Displays']
    for display in displays:
        kind = display['Class'].split('/')[-1]
        if kind == 'Map':
            display['Enabled'] = display['Enabled'] if menu == 'all' else (menu == 'map' and display.get('Topic', {}).get('Value') == '/nomad/navigation_local_costmap')
        elif kind == 'Path':
            display['Enabled'] = menu in ('all', 'planning')
        elif kind in ('Image', 'LaserScan'):
            display['Enabled'] = False
    if menu in ('all', 'localization'):
        displays += [
            {'Class': 'rviz_default_plugins/Path', 'Name': 'GNSS + VIO fused trajectory',
             'Enabled': True, 'Topic': {'Value': '/fusion/path'}, 'Color': '255; 120; 40', 'Line Width': .06},
            {'Class': 'rviz_default_plugins/Odometry', 'Name': 'Aligned VIO poses', 'Enabled': True,
             'Topic': {'Value': '/fusion/raw_vio_aligned'}, 'Keep': 80, 'Shape': 'Axes'},
        ]
    path = Path(directory) / 'visualization.rviz'
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    return path
