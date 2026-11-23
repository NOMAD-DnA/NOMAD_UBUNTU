import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore,QtWidgets
from planning_client import PlanningClient
from operator_timing import read_records,sessions_from_records


class ClickTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    def test_same_wire_command_click_and_ack_journal(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,NOMAD_PLANNING_TIMING_DIR=d):
            client=PlanningClient('user@192.0.2.1','not executed',d)
            client.process=Mock();client.process.state.return_value=QtCore.QProcess.Running
            client.process.write.side_effect=len;client.ready=True
            client.process.readAllStandardError.return_value=b'Pi4 domain 59 logs: /home/user/nomad_cpp_ws/log/planning_output_20261110_101635_2703\n'
            client.drain_error()
            click=(1794273400000000000,1000000000)
            self.assertTrue(client.send({'cmd':'arm'},timing=click,origin='output_allow_button'))
            self.assertEqual(client.process.write.call_args.args[0],b'{"cmd":"arm"}\n')
            client.process.readAllStandardOutput.return_value=b'{"type":"ack","cmd":"arm","ok":true}\n'
            client.read_output()
            records=read_records([client.operator_journal.path],'planning_output_20261110_101635_2703')
            session=sessions_from_records(records,click[0]+10**9)[0]
            self.assertEqual(session['start_ns'],click[0]);self.assertEqual(session['start_mono_ns'],click[1])
            self.assertTrue(session['exact_button']);self.assertFalse(session['closed'])
            client.deleteLater()
    def test_failed_send_does_not_create_pending_drive(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,NOMAD_PLANNING_TIMING_DIR=d):
            client=PlanningClient('user@192.0.2.1','not executed',d)
            client.process=Mock();client.process.state.return_value=QtCore.QProcess.Running
            client.process.write.return_value=-1;client.ready=True
            self.assertFalse(client.send({'cmd':'arm'},timing=(1794273400000000000,1),origin='output_allow_button'))
            rows=[json.loads(x) for x in client.operator_journal.path.read_text().splitlines()]
            self.assertFalse(rows[0]['sent']);self.assertFalse(sessions_from_records(rows,1794273400000000001))
            client.deleteLater()


if __name__=='__main__':unittest.main()
