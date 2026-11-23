"""Synthetic local record/replay regressions; no boards or motor commands."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import time
from pathlib import Path
import unittest
from unittest.mock import patch, Mock
from PyQt5 import QtWidgets
import test_recording_ui as fixture
from session_recording import SessionRecorder, SessionReader
from planning_footprint import Footprint
from recording_state import comparison_baseline
from module_logs import LogHistory
from developer_control import ControlWindow


class ReplayIntegrityTest(unittest.TestCase):
    setUpClass = classmethod(fixture.RecordingUiTest.setUpClass.__func__)
    setUp = fixture.RecordingUiTest.setUp
    tearDown = fixture.RecordingUiTest.tearDown
    window = fixture.RecordingUiTest.window
    seek = fixture.RecordingUiTest.seek

    def recorded(self, window):
        window.toggle_recording()
        self.assertIsNotNone(window.recorder, window.record_label.text())
        window.stop_recording()
        self.assertTrue(window.recording_result['complete'], window.recording_result)
        return Path(window.recording_result['path'])

    def read_events(self, path):
        reader = SessionReader(path)
        try:
            return list(reader.iter_until(reader.duration)), dict(reader.metadata)
        finally:
            reader.close()

    def test_existing_comparison_tracks_are_saved_without_wire_pose_spoofing(self):
        w = self.window('--no-ssh')
        for source in ('gnss', 'vio_compare'):
            for n in (1, 2):
                w.histories[source].append(dict(t=fixture.STAMP+n, x=n, y=2., z=0.,
                    frame_id='odom', viewer_epoch=3, position_only=True, source=source, kind='pose'))
            w.received['pose:'+source] = w.now()-.8
        path = self.recorded(w)
        rows, _ = self.read_events(path)
        self.assertEqual(len([r for r in rows if r['event']['kind']=='comparison_baseline']), 2)
        self.assertFalse(any(r['board']=='pi' and r['event'].get('source') in ('gnss','vio_compare')
                             and r['event'].get('kind')=='pose' for r in rows))
        replay = self.window('--replay', str(path));self.seek(replay, replay.reader.duration)
        for source in ('gnss','vio_compare'):
            self.assertEqual([p['x'] for p in replay.histories[source]], [1,2])
            self.assertGreaterEqual(replay.age('pose',source), .8)
        self.assertEqual(replay._replay_rejection_count, 0)

    def test_footprint_is_stored_and_used_without_current_config(self):
        w = self.window('--no-ssh');w.display_footprint = Footprint(.2,.1,-.15)
        path = self.recorded(w)
        _, meta = self.read_events(path)
        self.assertEqual(meta['event_schema_version'],2)
        self.assertIn('session_ui.py',meta['gui_source_sha256'])
        with patch('planning_footprint.load_footprint', side_effect=AssertionError('current config forbidden')), \
                patch('developer_planning.load_footprint', side_effect=AssertionError('current overlay forbidden')):
            replay=self.window('--replay',str(path))
            replay.open_developer_window('planning')
        self.assertEqual(replay.developer_windows['planning'].map.footprint, w.display_footprint)
        replay.developer_windows['planning'].grab().save(str(self.root/'recorded-footprint.png'))

    def test_legacy_record_opens_without_guessing_old_footprint(self):
        path=self.root/'legacy.nomad';r=SessionRecorder(path,{'started_wall_s':fixture.STAMP});r.close()
        with patch('planning_footprint.load_footprint',side_effect=AssertionError('no live config')):
            w=self.window('--replay',str(path));w.open_developer_window('planning')
        p=w.developer_windows['planning']
        self.assertIsNone(p.map.footprint)
        self.assertIn('미저장',p.footprint_info.text())
        self.assertIn('미저장',w.record_label.text())

    def test_corrupt_latest_frame_at_record_start_retains_displayed_frame(self):
        w=self.window('--no-ssh')
        w.on_packet('jetson',fixture.image('rgb','red',fixture.STAMP));w.refresh()
        bad=fixture.image('rgb','blue',fixture.STAMP+1);bad['data']='bad-base64'
        w.on_packet('jetson',bad);w.refresh()
        self.assertTrue(w.images['rgb']['decode_error'])
        path=self.recorded(w);replay=self.window('--replay',str(path));self.seek(replay,replay.reader.duration)
        self.assertEqual(replay.latest_qimages['rgb'].pixelColor(0,0).name(),'#ff0000')
        replay.open_developer_window('perception')
        p=replay.developer_windows['perception']
        self.assertIsNotNone(p.panels['rgb'].pixmap)
        self.assertIn('새 영상 오류',p.panels['rgb'].meta.text())
        self.assertGreater(replay._replay_rejection_count,0)

    def test_unlocated_map_at_start_keeps_located_map_in_new_developer_window(self):
        w=self.window('--no-ssh')
        w.on_packet('jetson',fixture.image('costmap','green',fixture.STAMP));w.refresh()
        bad=fixture.image('costmap','red',fixture.STAMP+1);bad['georeferenced']=False
        w.on_packet('jetson',bad);w.refresh()
        path=self.recorded(w);replay=self.window('--replay',str(path));self.seek(replay,replay.reader.duration)
        replay.open_developer_window('planning')
        p=replay.developer_windows['planning'].map
        self.assertTrue(p.event['georeferenced'])
        self.assertEqual(p.pixels[0,0].tolist(),[0,128,0])
        self.assertEqual(replay.developer_snapshot()['image_meta']['costmap']['t'],fixture.STAMP)

    def test_initial_successful_restart_does_not_clear_already_restored_history(self):
        w=self.window('--no-ssh');w.on_packet('pi',fixture.pose(2.,2.))
        w.vio_restart_result={'ok':True,'state':'RESTARTED'}
        # Give raw VIO an already running post-restart track.
        event={**fixture.pose(3.,3.),'source':'vio'};w.on_packet('pi',event)
        path=self.recorded(w);replay=self.window('--replay',str(path));self.seek(replay,replay.reader.duration)
        self.assertEqual([p['x'] for p in replay.histories['vio']],[3.])

    def test_rejected_event_has_persistent_seq_warning_and_rewind_resets_it(self):
        path=self.root/'rejected.nomad';r=SessionRecorder(path,{'started_wall_s':fixture.STAMP})
        r.append('jetson',fixture.pose(1,1),1.) # wrong owner, valid JSON
        r.append('pi',fixture.pose(2,2),2.);r.close(duration_s=3.)
        w=self.window('--replay',str(path));self.seek(w,3.)
        self.assertEqual(w._replay_rejection_count,1)
        self.assertIn('적용 제외 1건',w.record_label.text());self.assertIn('seq 1',w.record_label.toolTip())
        self.assertEqual(len(w.histories['public']),1)
        self.seek(w,0.);self.assertEqual(w._replay_rejection_count,0)

    def test_unsupported_future_schema_is_rejected(self):
        path=self.root/'future.nomad';r=SessionRecorder(path,{'event_schema_version':900});r.close()
        with self.assertRaisesRegex(ValueError,'이벤트 버전'):
            self.window('--replay',str(path))

    def test_malformed_view_enters_error_state_instead_of_callback_exception(self):
        path=self.root/'view.nomad';r=SessionRecorder(path,{'started_wall_s':fixture.STAMP})
        r.append('gui',dict(kind='view',source='gui',slope_cpu_history=[]),1.);r.close(duration_s=2.)
        w=self.window('--replay',str(path));w.seek_replay(2.)
        self.assertIsNotNone(w._replay_error);self.assertFalse(w.replay_play.isEnabled())

    def test_replay_log_and_control_trace_update_without_real_time_wait(self):
        w=ControlWindow();self.windows.append(w);h=LogHistory()
        snap={'replay':True,'now':100.,'wall_now':fixture.STAMP+1,
              'module_logs':{'planning':h},'planning_ui':{},'received':{},'telemetry':{}}
        def trace(seq,n):
            row=fixture.log(f'control_trace source=LPP phase=RUNNING requested_erpm={n} target_erpm={n} output_erpm={n} measured_erpm={n} current_a=1 start_gate=ready reset_required=0',seq,n)
            row['board']='pi4';return row
        with patch('module_logs.time.monotonic',return_value=200.):
            h.append(trace(1,1));w.update_snapshot(snap)
            h.append(trace(2,2));snap.update(now=104.,wall_now=fixture.STAMP+2);w.update_snapshot(snap)
        self.assertEqual(w._trace['requested_erpm'],2.)
        self.assertIn('requested_erpm=2',w.log_panel.text.toPlainText())

    def test_stale_replay_never_claims_to_send_stop(self):
        path=self.root/'stale.nomad';r=SessionRecorder(path,{'started_wall_s':fixture.STAMP})
        r.append('pi4',dict(kind='telemetry',source='planning',t=fixture.STAMP,frame='odom',
                  state='WAIT',plan_only=False,path=[],trajectory=[],path_valid=False),0.)
        r.close(duration_s=8.)
        w=self.window('--replay',str(path));self.seek(w,8.)
        self.assertNotIn('정지 요청',w.planning_message.text())
        self.assertIn('명령 전송 없음',w.planning_message.text())

    def test_switch_result_details_are_preserved(self):
        w=self.window('--no-ssh');w.cloud_switch_result={'ok':False,'rollback_confirmed':True,'reason':'fixture'}
        w.slope_switch_result={'ok':True,'state':'MODE_CONFIRMED'}
        path=self.recorded(w);r=self.window('--replay',str(path));self.seek(r,r.reader.duration)
        self.assertEqual(r.cloud_switch_result,w.cloud_switch_result)
        self.assertEqual(r.slope_switch_result,w.slope_switch_result)

    def test_comparison_baseline_rejects_nonfinite_or_forged_source(self):
        with self.assertRaises(ValueError):
            comparison_baseline(dict(history_source='public',positions=[],receive_age_s=0))
        with self.assertRaises(ValueError):
            comparison_baseline(dict(history_source='gnss',positions=[],receive_age_s=float('nan')))

    def test_prototype_caption_does_not_invent_parameter_limits(self):
        from planning_client import output_summary
        text=output_summary(dict(prototype_output=True))
        self.assertNotIn('0.30rad',text);self.assertIn('실행 설정 확인',text)

    def test_code_version_difference_is_visible_without_overriding_saved_geometry(self):
        from recording_state import display_metadata
        metadata=display_metadata(Footprint(.2,.1,-.15))
        metadata['gui_source_sha256']['monitor.py']='0'*64
        path=self.root/'version.nomad';r=SessionRecorder(path,metadata);r.close()
        w=self.window('--replay',str(path))
        self.assertIn('코드 버전 다름',w.record_label.text())
        self.assertEqual(w.display_footprint,Footprint(.2,.1,-.15))

    def test_bad_nested_cpu_packet_is_rejected_before_mutating_display(self):
        w=self.window('--no-ssh')
        self.assertFalse(w.on_packet('jetson',dict(kind='telemetry',source='slope_cpu',t=fixture.STAMP,cpu=[])))
        self.assertNotIn('slope_cpu',w.telemetry)

    def test_shutdown_waits_for_receiver_before_accepting_close(self):
        w=self.window('--no-ssh');worker=Mock();worker.wait.return_value=False;w.workers=[worker]
        event=Mock()
        with patch('monitor.QtCore.QTimer.singleShot'):
            w.closeEvent(event)
        event.ignore.assert_called_once();event.accept.assert_not_called()
        worker.wait.return_value=True;w.workers=[]


if __name__=='__main__':unittest.main()
