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


def test_module_categories_and_saved_prefix():
    spec=importlib.util.spec_from_file_location('group_filter_test',ROOT/'src/nomad_bringup/gui/log_filter.py')
    filters=importlib.util.module_from_spec(spec);spec.loader.exec_module(filters)
    for node,group in [('basalt','위치추정'),('fusion_adapter','위치추정'),('semantic','인지'),('navigation_terrain','인지'),('gpp_node','플래닝'),('nomad_control','제어'),('gazebo','시뮬레이션')]:
        item=filters.entry(f'[INFO] [1790939680.6] [{node}]: ready')
        assert filters.category(item)==group
        assert filters.matches(item,group=group)
    assert filters.category(filters.entry('2026-10-13T00:00:00 [위치추정] [VIO] tracker ready'))=='위치추정'


def test_recording_is_opt_in_selected_and_has_no_results(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen');monkeypatch.setenv('NOMAD_LOG_DIR',str(tmp_path))
    gui_path=str(ROOT/'src/nomad_bringup/gui');sys.path.insert(0,gui_path)
    try:
        from PyQt5.QtWidgets import QApplication
        from launcher import Launcher
        app=QApplication.instance() or QApplication([])
        window=Launcher()
        window.append_log('[INFO] [1790939680.6] [semantic]: BEFORE')
        assert window.record_stream is None
        assert not list(tmp_path.glob('*/manifest.json'))
        assert not (Path(window.log_spool.name)/'console.log').exists()
        window.start_recording('selected',{'인지'})
        window.append_log('[INFO] [1790939681.6] [semantic]: INCLUDED')
        window.append_log('[ERROR] [1790939682.6] [basalt]: EXCLUDED')
        window.finish_recording()
        directory=window.record_directory
        window.append_log('[INFO] [1790939683.6] [semantic]: AFTER')
        text=(directory/'console.log').read_text()
        assert 'INCLUDED' in text
        assert all(word not in text for word in ('BEFORE','EXCLUDED','AFTER'))
        manifest=json.loads((directory/'manifest.json').read_text())
        assert manifest['files']==[] and manifest['console_metrics_processed']
        assert not list(directory.glob('*.png'))
        window.group_filter.setCurrentIndex(window.group_filter.findData('인지'))
        window.flush_live_log()
        assert 'EXCLUDED' not in window.log.toPlainText()
        from log_viewer import LogViewer
        viewer=LogViewer(window,tmp_path);viewer.open_session(0,0)
        assert 'INCLUDED' in viewer.text.toPlainText();viewer.close();window.close()
    finally:
        sys.path.remove(gui_path)


def test_gui_launch_arguments_are_accepted_by_real_ros_parser(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen');monkeypatch.setenv('NOMAD_LOG_DIR',str(tmp_path))
    gui_path=str(ROOT/'src/nomad_bringup/gui');sys.path.insert(0,gui_path)
    try:
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtCore import QProcess
        from launcher import Launcher
        from ros2launch.api.api import parse_launch_arguments
        app=QApplication.instance() or QApplication([])
        import launcher
        monkeypatch.setattr(launcher,'conflicts',lambda *_:[])
        captured=[];monkeypatch.setattr(QProcess,'start',lambda self,exe,args:captured.append(args))
        window=Launcher();window.launch()
        args=[a for a in captured[0] if ':=' in a]
        parsed=dict(parse_launch_arguments(args))
        assert parsed['record_results']=='false'
        assert 'record_directory' not in parsed
        for run in window.runs.values():assert not run['process'].processEnvironment().contains('NOMAD_RUN_LOG_DIR')
        window.runs.clear();window.close()
    finally:sys.path.remove(gui_path)


def test_process_guard_detects_other_gui_and_orphans_by_domain(tmp_path):
    spec=importlib.util.spec_from_file_location('process_guard_test',ROOT/'src/nomad_bringup/gui/process_guard.py')
    guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    for pid,domain,args in [(12001,42,['parameter_bridge','-r','__node:=nomad_gz_bridge']),
                            (12002,99,['fusion_node']),
                            (12003,42,['python3','ros2','launch','nomad_perception','terrain_navigation.launch.py'])]:
        folder=tmp_path/str(pid);folder.mkdir()
        (folder/'cmdline').write_bytes('\0'.join(args).encode())
        (folder/'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0'.encode())
    result=guard.conflicts(['sim','localization'],42,tmp_path)
    assert sorted(pid for pid,_ in result)==[12001,12003]
    assert guard.conflicts(['control'],99,tmp_path)==[]


def test_process_guard_only_signals_its_own_group(tmp_path,monkeypatch):
    spec=importlib.util.spec_from_file_location('process_reap_test',ROOT/'src/nomad_bringup/gui/process_guard.py')
    guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    for pid,group in [(12001,11000),(12002,22000)]:
        folder=tmp_path/str(pid);folder.mkdir();(folder/'stat').write_text(f'{pid} (python worker) S 1 {group} 0')
    assert guard.group_members(11000,tmp_path)==[12001]
    calls=[];monkeypatch.setattr(guard,'group_members',lambda group:guard_members[group])
    guard_members={11000:[12001]};monkeypatch.setattr(guard.os,'kill',lambda pid,sig:calls.append(pid))
    guard.signal_children(11000,15);assert calls==[12001]


def test_startup_cleanup_scopes_workspace_domain_and_children():
    gui_path=str(ROOT/'src/nomad_bringup/gui');sys.path.insert(0,gui_path)
    try:
        import cleanup_previous as cleanup
        def proc(args,domain=42,parent=1,workspace=ROOT):return {'args':args,'env':{'ROS_DOMAIN_ID':str(domain),'NOMAD_WS_ROOT':str(workspace)},'parent':parent,'start':'1'}
        processes={12001:proc([str(ROOT/'install/nomad_control/lib/nomad_control/controller')]),
                   12002:proc(['external_worker'],parent=12001),
                   12003:proc(['/other/install/nomad_control/lib/nomad_control/controller'],workspace='/other'),
                   12004:proc([str(ROOT/'install/nomad_control/lib/nomad_control/controller')],99),
                   12005:proc(['python3','unrelated.py']),
                   12006:proc(['python3',str(ROOT/'src/nomad_bringup/gui/launcher.py')]),
                   12007:proc(['ros2','launch','nomad_perception','terrain_navigation.launch.py'],workspace='/other')}
        assert set(cleanup.select_targets(processes,ROOT,42))=={12001,12002}
    finally:sys.path.remove(gui_path)


def test_log_write_failure_does_not_crash_gui(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen');monkeypatch.setenv('NOMAD_LOG_DIR',str(tmp_path))
    gui_path=str(ROOT/'src/nomad_bringup/gui');sys.path.insert(0,gui_path)
    try:
        from PyQt5.QtWidgets import QApplication
        from launcher import Launcher
        app=QApplication.instance() or QApplication([]);window=Launcher()
        class BrokenStream:
            def write(self,_):raise OSError('disk full')
            def flush(self):raise OSError('disk full')
            def close(self):pass
        window.record_stream=BrokenStream();window.record_groups={'인지'};window.record_directory=tmp_path/'missing'
        window.append_log('[INFO] [1790939680.6] [semantic]: test')
        assert window.record_stream is None and '기록 오류' in window.log_hint.text()
        window.append_log('[INFO] [1790939681.6] [semantic]: {"value": "quoted"}')
        window.flush_live_log()
        assert '{"value": "quoted"}' in window.log.toPlainText() and '&quot;' not in window.log.toPlainText()
        window.close()
    finally:sys.path.remove(gui_path)


def test_all_gui_menus_generate_nonempty_ros_values(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen');monkeypatch.setenv('NOMAD_LOG_DIR',str(tmp_path))
    gui_path=str(ROOT/'src/nomad_bringup/gui');sys.path.insert(0,gui_path)
    try:
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtCore import QProcess
        import launcher
        from ros2launch.api.api import parse_launch_arguments
        app=QApplication.instance() or QApplication([])
        monkeypatch.setattr(launcher,'conflicts',lambda *_:[])
        captured=[];monkeypatch.setattr(QProcess,'start',lambda self,exe,args:captured.append(args))
        window=launcher.Launcher()
        for index,menu in enumerate(window.menus):
            window.menu.setCurrentIndex(index);window.launch()
            assert len(captured)==index+1,menu['id']
            parsed=dict(parse_launch_arguments([arg for arg in captured[-1] if ':=' in arg]))
            if menu['id'] in ('all','map'):assert parsed['record_results']=='false'
            window.runs.clear()
        window.close()
    finally:sys.path.remove(gui_path)
