"""Offline monitor integration: real packet routing and guarded user actions."""
import os
from pathlib import Path
import sys
import time
import unittest
from unittest import mock
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import monitor

class LocalizationMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    def setUp(self):
        with mock.patch.object(sys,'argv',['monitor','--no-ssh']):
            self.w=monitor.MonitorWindow(monitor.parse_args())
        self.t=time.time()
    def tearDown(self):
        self.w.heading_reset_in_progress=self.w.vio_restart_in_progress=False
        self.w.close(); self.app.processEvents()
    def comparison(self,epoch='0',stamp=None,**extra):
        stamp=stamp or self.t
        values=dict(mode='VIO_ONLY',position_valid='true',orientation_valid='true',
                    heading_reset_available='true',viewer_frame='odom',viewer_epoch=epoch,
                    viewer_gnss_valid='true',viewer_gnss_t=str(stamp-.4),viewer_gnss_x='3',viewer_gnss_y='4',viewer_gnss_z='5',
                    viewer_vio_valid='true',viewer_vio_t=str(stamp),viewer_vio_x='1',viewer_vio_y='2',viewer_vio_z='3')
        values.update(extra)
        return dict(kind='telemetry',source='fusion',t=stamp,values=values)
    def pose(self,source,x=1):
        return dict(kind='pose',source=source,t=self.t,x=x,y=2,z=3,qx=0,qy=0,qz=0,qw=1,
                    frame_id='odom',child_frame_id='base_link')
    def test_comparison_uses_explicit_positions_and_original_measurement_age(self):
        event=self.comparison()
        event['measurement_age_s']=.3
        self.w.on_packet('pi',event)
        self.assertEqual(self.w.histories['gnss'][-1]['x'],3)
        self.assertNotIn('qw',self.w.histories['gnss'][-1])
        self.assertGreaterEqual(self.w.age('pose','gnss'),.69)
        receipt=self.w.received['pose:gnss']
        self.w.on_packet('pi',{**event,'t':self.t+.2})
        self.assertEqual(len(self.w.histories['gnss']),1)
        self.assertEqual(self.w.received['pose:gnss'],receipt)
    def test_epoch_breaks_comparison_segments_and_invalid_epoch_does_not_clear(self):
        self.w.on_packet('pi',self.comparison())
        self.w.on_packet('pi',self.comparison(epoch=True,stamp=self.t+.1))
        self.assertEqual(len(self.w.histories['gnss']),1)
        self.w.on_packet('pi',self.comparison(epoch='1',stamp=self.t+.2))
        self.assertEqual(len(self.w.histories['gnss']),1)
        self.assertEqual(self.w.histories['gnss'][-1]['viewer_epoch'],1)
    def test_spectators_and_replay_split_raw_vio_at_the_reported_epoch(self):
        self.w.on_packet('pi',self.comparison(vio_epoch='0'))
        for source in ('vio','public'):self.w.on_packet('pi',self.pose(source))
        self.w.on_packet('pi',self.comparison(stamp=self.t+.1,vio_epoch='1'))
        self.assertFalse(self.w.histories['vio'])
        self.assertEqual(len(self.w.histories['public']),1)

    def test_old_backend_does_not_invent_gnss_or_aligned_vio(self):
        self.w.on_packet('pi',dict(kind='telemetry',source='fusion',t=self.t,values={'mode':'VIO_ONLY'}))
        self.assertFalse(self.w.histories['gnss']);self.assertFalse(self.w.histories['vio_compare'])
    def test_read_only_requests_cannot_create_workers(self):
        self.w.on_packet('pi',self.comparison())
        with mock.patch.object(monitor,'HeadingResetWorker') as heading, mock.patch.object(monitor,'VioRestartWorker') as vio:
            self.w.reset_heading_from_magnetometer();self.w.restart_vio()
            heading.assert_not_called();vio.assert_not_called()
            self.w.args.no_ssh=False;self.w.spectator_mode=True
            self.w.reset_heading_from_magnetometer();self.w.restart_vio()
            heading.assert_not_called();vio.assert_not_called()
    def test_heading_user_action_needs_fresh_backend_authorization_and_never_rotates_gui_poses(self):
        self.w.args.no_ssh=False
        with mock.patch.object(monitor,'HeadingResetWorker') as factory, mock.patch.dict(os.environ,{'NOMAD_BOARD_PASSWORD1':'fixture'}):
            self.w.reset_heading_from_magnetometer();factory.assert_not_called()
            self.w.on_packet('pi',self.comparison())
            self.w.on_packet('pi',self.pose('public'))
            before=list(self.w.histories['public'])
            self.w.reset_heading_from_magnetometer();factory.assert_called_once()
            self.assertFalse(self.w.vio_ui()['enabled'])
            self.w.on_heading_reset_result({'ok':True,'state':'HEADING_RESET','reason':'ok'})
            self.assertEqual(before,list(self.w.histories['public']))
            self.w.on_heading_reset_finished()
    def test_vio_restart_preserves_public_and_fusion_histories(self):
        for source in ('vio','public','fusion'):self.w.on_packet('pi',self.pose(source))
        self.w.on_vio_restarted({'ok':True,'state':'VIO_RESTARTED','reason':'fusion_preserved','fusion_preserved':True})
        self.assertFalse(self.w.histories['vio'])
        self.assertEqual(len(self.w.histories['public']),1)
        self.assertEqual(len(self.w.histories['fusion']),1)
        self.assertIn('completed_wall_s',self.w.vio_restart_result)
    def test_live_restart_is_one_explicit_worker_and_other_action_is_locked_while_pending(self):
        self.w.args.no_ssh=False
        with mock.patch.object(monitor,'VioRestartWorker') as factory, mock.patch.dict(os.environ,{'NOMAD_BOARD_PASSWORD1':'fixture'}):
            self.w.restart_vio();self.w.restart_vio()
            factory.assert_called_once()
            self.assertFalse(self.w.heading_ui()['enabled'])
            self.w.on_vio_restart_finished()
    def test_main_label_exposes_mode_without_waiting_for_detail_window(self):
        self.w.on_packet('pi',self.comparison())
        self.w.refresh()
        self.assertIn('VIO 단독',self.w.localization_mode_label.text())
if __name__=='__main__': unittest.main()
