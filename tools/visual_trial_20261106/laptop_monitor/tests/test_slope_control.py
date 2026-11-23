import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from slope_control import switch_slope_mode,RESULT_MARKER
from slope_cpu_client import SlopeCpuClient


def response(value):return {'ok':True,'stdout':RESULT_MARKER+json.dumps(value)}

class SlopeControlTest(unittest.TestCase):
    def test_live_and_offline_ack_are_distinct(self):
        for state in ['MODE_CONFIRMED','SAVED_FOR_NEXT_START']:
            with patch('slope_control.run_remote',return_value=response({'ok':True,'state':state,'mode':'parallel','map_reset':False})):
                self.assertEqual(switch_slope_mode('nvidia@192.0.2.1','parallel')['state'],state)
        for value in [{'ok':True,'state':'MODE_CONFIRMED','mode':'sequential','map_reset':False},
                      {'ok':True,'state':'MODE_CONFIRMED','mode':'parallel','map_reset':True}]:
            with patch('slope_control.run_remote',return_value=response(value)):
                self.assertFalse(switch_slope_mode('nvidia@192.0.2.1','parallel')['ok'])
        with patch('slope_control.run_remote') as remote:
            self.assertFalse(switch_slope_mode('user@192.0.2.1','parallel')['ok']);remote.assert_not_called()

    def test_transport_is_readonly_watch_and_validates_cpu(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=SlopeCpuClient('nvidia@192.0.2.1',tmp)
            value={'kind':'slope_status','schema':1,'t':time.time(),'active_mode':'parallel','state':'live',
                   'cpu':{'state':'live','percent_one_core':235.5}}
            self.assertTrue(c._accept_line(json.dumps(value)))
            self.assertEqual(c.drain()[0][1]['source'],'slope_cpu')
            for bad in [float('nan'),-1,True]:
                value['cpu']['percent_one_core']=bad;self.assertFalse(c._accept_line(json.dumps(value)))
            self.assertIn('watch',c.remote_command());self.assertNotIn('parallel',c.remote_command())
            import relay
            forwarded={'kind':'telemetry','source':'slope_cpu','t':time.time(),
                       'state':'live','active_mode':'parallel','cpu':{'state':'live','percent_one_core':235.5}}
            self.assertIsNotNone(relay._validate('jetson',forwarded))
            self.assertFalse(relay.owns_stream('pi',forwarded))

class SlopeGuiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        import monitor
        self.monitor=monitor
        with patch.object(sys,'argv',['monitor','--no-ssh','--enable-trial-control']):
            self.w=monitor.MonitorWindow(monitor.parse_args())

    def tearDown(self):
        self.w.slope_switch_in_progress=False;self.w.close();self.app.processEvents()

    def test_selector_readonly_and_cpu_only_inside_perception(self):
        w=self.w
        self.assertEqual([w.slope_combo.itemData(i) for i in range(2)],['sequential','parallel'])
        self.assertFalse(w.slope_button.isEnabled());self.assertFalse(hasattr(w,'slope_cpu_values'))
        w.open_developer_window('perception');p=w.developer_windows['perception']
        self.assertIn('parallel',p.slope_cpu_values);self.assertIsNone(w.slope_cpu_client)
        now=time.monotonic()
        w.on_packet('jetson',{'kind':'telemetry','source':'slope_cpu','t':time.time(),
                   'state':'live','active_mode':'parallel','cpu':{'state':'live','percent_one_core':212.5},
                   'worker':{'pid':123,'threads':3}})
        p.update_snapshot(w.developer_snapshot())
        self.assertEqual(p.slope_cpu_values['parallel'].text(),'212.5%')
        self.assertEqual(p.slope_cpu_states['sequential'].text(),'미실행')
        self.assertIn('PID 123',p.slope_cpu_status.text())
        w.received['telemetry:slope_cpu']=now-5;p.update_snapshot(w.developer_snapshot())
        self.assertNotIn('실시간',p.slope_cpu_states['parallel'].text())

    def test_switch_disables_other_actions_and_saved_mode_does_not_claim_running(self):
        w=self.w;w.args.no_ssh=False
        with patch.dict(os.environ,{'NOMAD_BOARD_PASSWORD1':'fixture'}), \
                patch.object(w,'ensure_slope_cpu_reader'),patch.object(self.monitor,'SlopeSwitchWorker') as worker:
            w.apply_slope_mode();worker.return_value.start.assert_called_once()
            self.assertFalse(w.cloud_button.isEnabled());self.assertFalse(w.stop_button.isEnabled())
            w.apply_model();self.assertFalse(w.model_switch_in_progress)
            w.on_slope_switched({'ok':True,'state':'SAVED_FOR_NEXT_START','mode':'parallel'})
            self.assertIsNone(w.active_slope_mode);self.assertIn('다음 지도 시작',w.slope_message.text())
            w.on_slope_switch_finished();self.assertTrue(w.cloud_button.isEnabled())
        w.on_slope_switched({'ok':True,'state':'MODE_CONFIRMED','mode':'parallel'})
        self.assertEqual(w.active_slope_mode,'parallel')
        w.on_slope_switched({'ok':False,'reason':'timeout'});self.assertIsNone(w.active_slope_mode)

if __name__=='__main__':unittest.main()
