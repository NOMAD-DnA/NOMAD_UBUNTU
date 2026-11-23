"""Built-in front video player and the 재생 choice. No board, no install is run."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtGui, QtWidgets

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
import front_player as fp
import front_recording as fr


def jpeg(value):
    image = QtGui.QImage(64, 36, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(value, value, value))
    from PyQt5 import QtCore
    data = QtCore.QByteArray()
    stream = QtCore.QBuffer(data)
    stream.open(QtCore.QIODevice.WriteOnly)
    image.save(stream, "JPG")
    return bytes(data)


class PlayerTests(unittest.TestCase):
    def make_trial(self, folder, n=12):
        raw = folder / "원본"
        raw.mkdir(parents=True)
        stream, rows, offset = b"", ["frame,stamp_ns,received_ns,offset,bytes"], 0
        for i in range(n):
            data = jpeg(i * 20)
            rows.append(f"{i},{1_000_000_000 + i * 100_000_000},0,{offset},{len(data)}")
            stream += data
            offset += len(data)
        (raw / "seg_0000.mjpeg").write_bytes(stream)
        (raw / "seg_0000.csv").write_text("\n".join(rows) + "\n")
        return n

    def test_frames_from_raw_and_from_avi_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            n = self.make_trial(folder)
            self.assertEqual(len(fp.frame_index(folder)), n)
            with patch.object(fr.shutil, "which", return_value=None):
                (folder / "seg_0000.mjpeg").write_bytes((folder / "원본/seg_0000.mjpeg").read_bytes())
                (folder / "seg_0000.csv").write_text((folder / "원본/seg_0000.csv").read_text())
                fr.write_mjpeg_avi_parts(folder)
            self.assertEqual(len(fp.avi_frames(folder / "전체.avi")), n)

    def test_player_shows_and_steps_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            n = self.make_trial(folder)
            player = fp.PlayerDialog(folder, "시험")
            self.assertAlmostEqual(player.fps, 10.0, places=1)
            player.seek(5)
            self.assertIsNotNone(player.view.pixmap())
            self.assertIn(f"6/{n}", player.time.text())
            player.toggle()
            self.assertTrue(player.timer.isActive())
            player.position = n - 1
            player.next_frame()
            self.assertFalse(player.timer.isActive())
            player.close()

    def test_play_uses_system_player_when_registered(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / "전체.avi").write_bytes(b"RIFF")
            with patch.object(fp, "system_player_available", return_value=True), \
                    patch.object(fp.subprocess, "Popen") as popen:
                fp.play(folder)
            self.assertEqual(popen.call_args.args[0][0], "xdg-open")

    def test_without_player_asks_and_can_play_inside(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            self.make_trial(folder)
            (folder / "전체.avi").write_bytes(b"RIFF")
            def choose(box_self):
                for button in box_self.buttons():
                    if button.text() == "GUI 안에서 바로 보기":
                        box_self._clicked = button
                return 0
            with patch.object(fp, "system_player_available", return_value=False), \
                    patch.object(fp.shutil, "which", return_value=None), \
                    patch.object(QtWidgets.QMessageBox, "exec_", choose), \
                    patch.object(QtWidgets.QMessageBox, "clickedButton", lambda s: s._clicked), \
                    patch.object(fp.subprocess, "Popen") as popen:
                player = fp.play(folder)
            self.assertIsInstance(player, fp.PlayerDialog)
            popen.assert_not_called()          # nothing installed or opened outside
            player.close()


if __name__ == "__main__":
    unittest.main()
