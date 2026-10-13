import importlib.util
from pathlib import Path
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('visualization_profile_test',ROOT/'src/nomad_bringup/gui/visualization_profile.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

@pytest.mark.parametrize('menu',['all','localization','map','planning','control'])
def test_visualization_shows_only_requested_module_layers(tmp_path,menu):
    before=(ROOT/'src/nomad_perception/rviz/terrain_navigation.rviz').read_bytes()
    path=module.rviz_profile(ROOT,menu,tmp_path)
    data=yaml.safe_load(path.read_text())
    active=[d for d in data['Visualization Manager']['Displays'] if d['Enabled']]
    topics={d.get('Topic',{}).get('Value') for d in active}
    assert ('/fusion/path' in topics)==(menu in ('all','localization'))
    assert ('/nomad/global_path' in topics)==(menu in ('all','planning'))
    assert ('/nomad/navigation_local_costmap' in topics)==(menu in ('all','map'))
    assert (ROOT/'src/nomad_perception/rviz/terrain_navigation.rviz').read_bytes()==before
