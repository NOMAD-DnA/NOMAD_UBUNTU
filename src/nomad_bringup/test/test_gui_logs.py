"""Saved-file logging must not depend on ROS subscriptions."""
import importlib.util
from pathlib import Path
import sys
import json
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('nomad_gui_log_data_test',ROOT/'src/nomad_bringup/gui/log_data.py')
module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)


def test_vins_trailing_comma_and_nan_are_not_headers(tmp_path):
    path=tmp_path/'vio.csv';path.write_text('10,0,1,2,nan,\n11,1,2,3,0,\n')
    data=module.load_numeric(path)
    assert data.xyz==('x','y','z') and data.time.tolist()==[10,11]
    assert np.isnan(data.fields['column_5'][0])


def test_explicit_timestamp_unit_and_named_numeric_fields(tmp_path):
    path=tmp_path/'control.csv';path.write_text('time,target_speed,steer\n1000,.4,.2\n1200,.3,.1\n')
    data=module.load_numeric(path,'ms')
    np.testing.assert_allclose(data.time,[1,1.2])
    assert data.has_time and data.xyz is None


def test_se3_recovery_keeps_scale(tmp_path):
    t=np.arange(10.)*.2
    source=np.column_stack([np.cos(t),np.sin(t),t*.1])
    r=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
    target=source@r.T+[3,2,1]
    def dataset(name,p):
        path=tmp_path/name;np.savetxt(path,np.column_stack([t,p,np.zeros((len(t),3)),np.ones(len(t))]))
        return module.load_numeric(path)
    gt=dataset('gt.tum',target);estimate=dataset('vio.tum',source)
    aligned,rmse,count=module.rigid_alignment(gt,estimate)
    assert rmse<1e-10 and count==len(t)
    np.testing.assert_allclose(aligned,target,atol=1e-10)
    np.testing.assert_allclose(np.linalg.norm(np.diff(aligned,axis=0),axis=1),np.linalg.norm(np.diff(source,axis=0),axis=1))


def test_interpolation_does_not_bridge_large_gaps(tmp_path):
    a=tmp_path/'gt.tum';b=tmp_path/'vio.tum'
    np.savetxt(a,[[0,0,0,0,0,0,0,1],[1,1,0,0,0,0,0,1]])
    np.savetxt(b,[[.5,.5,0,0,0,0,0,1]])
    mask,_,_=module.associate(module.load_numeric(a),module.load_numeric(b),.3)
    assert not mask.any()


def test_snapshot_is_independent_of_source_and_preserves_settings(tmp_path):
    source=tmp_path/'source.csv';source.write_text('time,speed\n0,.4\n')
    console=tmp_path/'console.log';console.write_text('original console')
    options=tmp_path/'options';options.mkdir();(options/'fusion.yaml').write_text('fusion: {}')
    root=tmp_path/'saved'
    target=module.save_session(root,'기록 이름',console,[{'options_directory':str(options)}],{'map':2},[source])
    source.write_text('changed');console.write_text('changed')
    manifest=json.loads((target/'manifest.json').read_text())
    assert (target/manifest['files'][0]['path']).read_text()=='time,speed\n0,.4\n'
    assert (target/'console.log').read_text()=='original console'
    assert (target/'config/run_01/fusion.yaml').is_file()
    assert module.sessions(root)[0]['name']=='기록 이름'


def test_snapshot_never_loads_pickle(tmp_path):
    path=tmp_path/'map.npz';np.savez(path,height=np.array([[object()]],dtype=object))
    with pytest.raises(ValueError):module.load_snapshot(path)


def test_json_metric_has_sample_axis_not_fabricated_time(tmp_path):
    path=tmp_path/'summary.json';path.write_text(json.dumps({'rmse':.3,'cpu':{'median':180}}))
    data=module.load_numeric(path)
    assert not data.has_time and data.fields['cpu.median'][0]==180


def test_console_recovery_only_recovers_printed_metrics(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps({'files':[],'console':'console.log'}))
    original='[INFO] [1790939680.6] [semantic] frames=3, inference=25.4 ms\n'
    (tmp_path/'console.log').write_text(original)
    result=module.recover_console_metrics(tmp_path)
    assert len(result['files'])==1
    data=module.load_numeric(tmp_path/result['files'][0]['path'])
    assert data.xyz is None and data.fields['inference_ms'][0]==25.4
    assert (tmp_path/'console.log').read_text()==original
    assert module.recover_console_metrics(tmp_path)==result


def test_console_metrics_do_not_mix_with_ros_time_files(tmp_path):
    original=[{'path':'results/control.csv'}]
    (tmp_path/'manifest.json').write_text(json.dumps({'files':original}))
    (tmp_path/'console.log').write_text('[INFO] [1790939680.6] [semantic] frames=3, inference=25.4 ms\n')
    assert module.recover_console_metrics(tmp_path)['files']==original


def test_live_node_and_severity_filter():
    spec=importlib.util.spec_from_file_location('live_filter_test',ROOT/'src/nomad_bringup/gui/log_filter.py')
    filters=importlib.util.module_from_spec(spec);spec.loader.exec_module(filters)
    item=filters.entry('[controller-5] [WARN] [1790939680.6] [nomad_control]: WAIT: path')
    assert item['node']=='nomad_control'
    assert filters.matches(item,node='nomad_control',level='WARN+',query='wait')
    assert not filters.matches(item,node='semantic')
    assert not filters.matches(item,level='ERROR+')


def test_optional_numeric_logger_rate_and_clock_reset(tmp_path,monkeypatch):
    spec=importlib.util.spec_from_file_location('csv_log_test',ROOT/'src/nomad_logging/nomad_logging/__init__.py')
    logging=importlib.util.module_from_spec(spec);spec.loader.exec_module(logging)
    monkeypatch.delenv('NOMAD_RUN_LOG_DIR',raising=False)
    disabled=logging.CsvLog('control',['speed']);disabled.emit(0,.4)
    assert disabled.file is None
    monkeypatch.setenv('NOMAD_RUN_LOG_DIR',str(tmp_path))
    logger=logging.CsvLog('control',['speed']);logger.emit(1,.4);logger.emit(1.1,.5);logger.emit(1.3,.6);logger.emit(0,.7);logger.close()
    files=sorted(tmp_path.glob('*.csv'))
    assert len(files)==2
    assert len(files[0].read_text().splitlines())==3
    assert len(files[1].read_text().splitlines())==2
