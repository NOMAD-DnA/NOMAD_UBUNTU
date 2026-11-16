"""CPU transport, ownership, recording/replay and offline export (no boards)."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad-resource-tests")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/nomad-resource-tests/mpl")
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from resource_metrics import validate_resource, ResourceHistory, brief
from module_log_client import ModuleLogClient
from relay import _validate, CaptureOrder, LatestPacketBuffer
from session_recording import SessionRecorder, SessionReader


def sample(board="pi4", seq=1):
    return dict(kind="telemetry", schema=1, source="resources_"+board, board=board,
                t=1794500000.+seq, sample_monotonic_s=100.+seq, session="fixture", seq=seq,
                boot_id="boot-fixture", model="test fixture", kernel="fixture",
                cpu=dict(state="live", interval_s=1., online_cores=1, total_percent=25.,
                         cores=[dict(id="cpu0",percent=25.,frequency_mhz=1000.)]),
                memory=dict(total_mib=1024.,used_mib=256.,available_mib=768.,swap_total_mib=0.,swap_used_mib=0.),
                gpu=dict(load_percent=None,frequency_mhz=None),temperatures=[],
                processes=[dict(pid=42,ppid=1,start_ticks=200,name="planner",group="planning_gpp_lpp",
                                cpu_percent_one_core=25.,rss_mib=10.,threads=3)],
                processes_total=1,processes_omitted=0,proc_read_failures=0,scan_ms=.5)


class ResourceContractTest(unittest.TestCase):
    def test_invalid_owner_nan_bool_duplicate_and_live_null_rejected(self):
        validate_resource(sample())
        for mutate in (lambda e:e.update(board="jetson"),lambda e:e.update(seq=True),
                       lambda e:e["cpu"].update(total_percent=float("nan")),
                       lambda e:e["cpu"].update(total_percent=None),
                       lambda e:e["processes"].append(dict(e["processes"][0]))):
            event=sample();mutate(event)
            with self.assertRaises(ValueError):validate_resource(event)

    def test_gpu_mean_max_and_read_count(self):
        # 2026-11-13: Jetson helper reads the GPU load ~20x/s; older helpers omit max/reads.
        event = sample("jetson")
        event["gpu"] = dict(load_percent=12.5, frequency_mhz=1109., max_percent=61., reads=20)
        validate_resource(event)
        self.assertIn("GPU 12.5% (최대 61%)", brief(event, 0.))
        for bad in (dict(max_percent=150.), dict(max_percent=True), dict(reads=-1), dict(reads=1.5)):
            broken = copy.deepcopy(event); broken["gpu"].update(bad)
            with self.assertRaises(ValueError):
                validate_resource(broken)
        old = sample("jetson"); old["gpu"] = dict(load_percent=0., frequency_mhz=114.)
        validate_resource(old)
        self.assertNotIn("최대", brief(old, 0.))

    def test_helper_source_samples_gpu_between_reports(self):
        root = Path(__file__).resolve().parents[1] / "board_tools"
        tail, sampler = (root / "module_log_tail.cpp").read_text(), (root / "resource_sampler.hpp").read_text()
        self.assertIn("sampler.gpu_tick(board)", tail)
        self.assertIn("std::min<long long>(left,50)", tail)
        self.assertIn("max_percent", sampler)

    def test_shared_log_stream_accepts_resource_and_relay_ownership(self):
        with tempfile.TemporaryDirectory() as folder:
            for board in ("jetson","pi4"):
                client=ModuleLogClient(board,("nvidia" if board=="jetson" else "user")+"@192.0.2.1",folder)
                self.assertIn("--resources",client.remote_command())
                self.assertTrue(client._accept_line(json.dumps(sample(board)),arrived=10.))
                owner,event=client.drain()[0]
                self.assertIsNotNone(_validate(owner,event))
                self.assertIsNone(_validate("pi",event))
                self.assertEqual(event["_local_received_monotonic"],10.)
                self.assertEqual(client.drain(),[])

    def test_monotonic_order_survives_wall_clock_step(self):
        order=CaptureOrder();one=sample(seq=1);two=sample(seq=2);two["t"]=one["t"]-500
        self.assertTrue(order.accept(("pi4","telemetry","resources_pi4"),one))
        self.assertTrue(order.accept(("pi4","telemetry","resources_pi4"),two))
        self.assertFalse(order.accept(("pi4","telemetry","resources_pi4"),one))
        # Ordinary sensor streams still enforce original wall timestamp order.
        self.assertTrue(order.accept("pose",dict(t=100.)))
        self.assertFalse(order.accept("pose",dict(t=99.)))

    def test_bounded_history_and_stale_not_zero(self):
        history=ResourceHistory(limit=2)
        for seq in range(1,5):history.add(sample(seq=seq),seq)
        self.assertEqual(len(history.rows["pi4"]),2)
        history.add(sample(seq=4),4)
        self.assertEqual(len(history.rows["pi4"]),2)
        self.assertIn("수신 지연",brief(sample(),4.))
        self.assertIn("미수신",brief(None,None))

    def test_export_excludes_cached_baseline_preserves_missing_and_original(self):
        from resource_report import export
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"fixture.nomad";r=SessionRecorder(path)
            r.append("pi4",sample(seq=1),0.)
            r.append("pi4",sample(seq=2),1.)
            e=sample(seq=4);e["cpu"]["total_percent"]=75.
            r.append("pi4",e,3.);r.close(duration_s=3.)
            before=path.read_bytes()
            out=export(path)
            report=json.loads((out/"summary.json").read_text())
            self.assertEqual(report["boards"]["pi4"]["samples"],2)
            self.assertEqual(report["boards"]["pi4"]["missing_sequences"],1)
            self.assertEqual(report["boards"]["pi4"]["cpu_time_weighted_mean_percent"],50.)
            self.assertEqual(report["boards"]["jetson"]["samples"],0)
            self.assertIsNone(report["boards"]["jetson"]["cpu_sample_max_percent"])
            self.assertTrue((out/"resources.png").is_file())
            self.assertEqual(path.read_bytes(),before)


class ResourceGuiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_resource_window_recording_and_replay_no_board_access(self):
        import monitor
        with tempfile.TemporaryDirectory() as folder:
            with mock.patch.object(sys,"argv",["monitor","--no-ssh"]):
                w=monitor.MonitorWindow(monitor.parse_args())
            try:
                w.timer.stop();w.ingest_timer.stop()
                path=Path(folder)/"fixture.nomad"
                w._record_started=time.monotonic();w.recorder=SessionRecorder(path)
                for board in ("pi4","jetson"):
                    self.assertTrue(w.on_packet(board,sample(board)))
                w.open_developer_window("resources")
                self.assertEqual(w.log_workers,{})
                self.assertEqual(w.workers,[])
                self.assertEqual(w.developer_windows["resources"].table.rowCount(),2)
                self.assertEqual(len(w.resource_history.rows["pi4"]),1)
                w.stop_recording()
                reader=SessionReader(path)
                try:
                    resources=[row for row in reader.iter_until(reader.duration) if row["event"].get("source","").startswith("resources_")]
                    self.assertEqual(len(resources),2)
                finally:reader.close()
            finally:w.close();self.app.processEvents()
            with mock.patch.object(sys,"argv",["monitor","--replay",str(path)]):
                replay=monitor.MonitorWindow(monitor.parse_args())
            try:
                replay.timer.stop();replay.ingest_timer.stop()
                replay.seek_replay(replay.reader.duration)
                while replay._replay_seeking:replay.advance_replay()
                replay.open_developer_window("resources")
                self.assertEqual(replay.log_workers,{})
                self.assertEqual(len(replay.resource_history.rows["pi4"]),1)
                replay.seek_replay(0.)
                while replay._replay_seeking:replay.advance_replay()
                self.assertEqual(len(replay.resource_history.rows["pi4"]),0)
            finally:replay.close();self.app.processEvents()


if __name__=="__main__":unittest.main()
