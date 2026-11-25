"""Indoor routing and offline perception window. No board connections."""
import os,time,unittest
from pathlib import Path
os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from PyQt5 import QtWidgets,QtGui
from developer_perception import PerceptionWindow,semantic_display
from monitor import ImagePanel
class IndoorDisplayTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
 def test_indoor_and_replay_legend(self):
  self.assertEqual(semantic_display({'trial_mapper':'indoor','active_model':'b2'})[0],'indoor5')
  self.assertEqual(semantic_display({'replay':True,'image_meta':{'rgb':{'image_source':'oak_color_jpeg'}}})[0],'indoor5')
  self.assertEqual(semantic_display({'trial_mapper':'terrain','active_model':'b2'})[0],'field12')
 def test_indoor_window_displays_both_streams(self):
  window=PerceptionWindow(components={'ImagePanel':ImagePanel});now=time.monotonic()
  try:
   image=QtGui.QImage(20,16,QtGui.QImage.Format_RGB888);image.fill(QtGui.QColor('#399ac4'))
   snapshot={'trial_mapper':'indoor','now':now,'received':{'image:rgb':now,'image:semantic':now},'images':{'rgb':image,'semantic':image.copy()},'image_meta':{'rgb':{'image_source':'oak_color_jpeg','width':20,'height':16},'semantic':{'width':20,'height':16}}}
   window.update_snapshot(snapshot)
   self.assertEqual(window.panels['rgb'].title.text(),'OAK 컬러 영상');self.assertIn('indoor5',window.panels['semantic'].title.text())
   self.assertIsNotNone(window.panels['rgb'].pixmap);self.assertIsNotNone(window.panels['semantic'].pixmap)
   self.assertIn('화면 표시용',window.model.text())
  finally:window.close();window.deleteLater()
 def test_startup_display_is_independent_of_mapper_inputs(self):
  root=Path(__file__).resolve().parents[1];helper=(root/'remote/indoor_display_helpers.sh').read_text()
  self.assertIn('--indoor -p rate_hz:=1.0',helper);self.assertIn('publish_tf:=false publish_map_tf:=false',helper)
  for token in ['run_costmap','run_semantic_adapter','external_stereo_cloud']:self.assertNotIn(token,helper)
  start=(root/'remote/start_jetson.sh').read_text();branch=start.split('# Indoor branch exits',1)[1].split('\nexec 9>/tmp/nomad_resume_jetson_final_preview.lock',1)[0]
  self.assertLess(branch.index('nomad_indoor_display_check'),branch.index('setsid env'))
  self.assertIn('-p enable_device_depth:=false',branch);self.assertIn('nomad_indoor_display_start "$run"',branch)
if __name__=='__main__':unittest.main()
