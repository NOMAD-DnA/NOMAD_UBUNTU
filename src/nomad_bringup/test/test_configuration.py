from pathlib import Path
import yaml
import pytest
from nomad_bringup.configuration import load_topics,load_modules,remappings,selected_specs

CONFIG=Path(__file__).resolve().parents[1]/'config'


def test_renamed_topics_follow_each_role_and_select_external_modules(tmp_path):
    data=yaml.safe_load((CONFIG/'topics.yaml').read_text())
    for section,items in data['topics'].items():
        for key in items:
            items[key]=f'/team/{section}/{key}'
    target=tmp_path/'topics.yaml'
    target.write_text(yaml.safe_dump(data))
    topics=load_topics(target)
    assert dict(remappings(topics,'gpp'))['/nomad/costmap']=='/team/perception/global_costmap'
    for role in ('lpp','command'):
        assert dict(remappings(topics,role))['/nomad/costmap']=='/team/perception/local_costmap'
    assert dict(remappings(topics,'recovery'))['/nomad/local_costmap']=='/team/perception/local_costmap'
    assert dict(remappings(topics,'control'))['/nomad/planning/drive_command']=='/team/planning/drive_command'
    providers=dict(perception='external',vio='external',control='builtin',tf_owner='vio')
    target.write_text(yaml.safe_dump(providers))
    assert len(selected_specs(load_modules(target)))==6
    assert selected_specs(providers,'perception')==[]
    assert len(selected_specs(providers,'planning'))==5


@pytest.mark.parametrize('error',['missing','extra','duplicate','invalid'])
def test_invalid_topic_config_fails_before_starting_nodes(tmp_path,error):
    data=yaml.safe_load((CONFIG/'topics.yaml').read_text())
    if error=='missing': del data['topics']['control']['status']
    if error=='extra': data['topics']['control']['unexpected']='/extra'
    if error=='duplicate': data['topics']['control']['status']='/cmd_vel'
    if error=='invalid': data['topics']['control']['status']='relative name'
    target=tmp_path/'topics.yaml'
    target.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError): load_topics(target)


def test_external_vio_requires_single_tf_owner(tmp_path):
    target=tmp_path/'modules.yaml'
    target.write_text('perception: builtin\nvio: external\ncontrol: builtin\ntf_owner: gazebo\n')
    with pytest.raises(ValueError,match='tf_owner=vio'): load_modules(target)
