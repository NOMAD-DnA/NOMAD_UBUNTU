"""Asymmetric geometry and display regression checks; no hardware I/O."""
import copy
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import numpy as np
from planning_footprint import load_footprint, cell_polygons
from test_developer_planning import fixture, COMPONENTS
from developer_planning import PlanningWindow


class GeometryTest(unittest.TestCase):
    def test_front_rear_reference_is_not_rectangle_center(self):
        f = load_footprint()
        np.testing.assert_allclose(f.corners(2., 3., 0.),
            [[2.025, 3.05], [2.025, 2.95], [1.825, 2.95], [1.825, 3.05]])
        np.testing.assert_allclose(f.corners(2., 3., math.pi/2),
            [[1.95, 3.025], [2.05, 3.025], [2.05, 2.825], [1.95, 2.825]])

    def test_grid_rotation_translation_and_outside_indices(self):
        f = load_footprint()
        g = dict(resolution=.2, width=2, height=2, origin=dict(x=0., y=0.))
        cells = set(f.cells(.03, .04, 0., g, 0.))
        self.assertEqual(cells, {(x, y) for x in (-1, 0) for y in (-1, 0)})
        rotated = dict(g, origin=dict(x=10., y=20.))
        self.assertEqual(set(f.cells(9.96, 20.03, math.pi/2, rotated, math.pi/2)), cells)
        np.testing.assert_allclose(cell_polygons([(0, 0)], rotated, math.pi/2)[0],
                                   [[10, 20], [10, 20.2], [9.8, 20.2], [9.8, 20]])
        # A malformed very fine map cannot create millions of artists.
        self.assertIsNone(f.cells(0, 0, 0, dict(g, resolution=.00001), 0))


class OverlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.window = PlanningWindow(components=COMPONENTS)
        _, _, self.snapshot = fixture()
        self.window.update_snapshot(self.snapshot)
        self.window.map.redraw()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()

    def test_overlay_uses_full_camera_attitude_and_exact_configured_size(self):
        s = self.snapshot
        s['forward_axis'] = (0, 0, -1)
        pose = s['histories']['public'][0]
        pose['qy'], pose['qw'] = -math.sin(math.pi/4), math.cos(math.pi/4)
        self.window.update_snapshot(s)
        p = self.window.map
        p.redraw()
        np.testing.assert_allclose(p._footprint_artist.get_xy()[:4],
            [[9.525, 21.05], [9.525, 20.95], [9.325, 20.95], [9.325, 21.05]])
        self.assertTrue(p._footprint_artist.get_visible())
        self.assertGreater(len(p._cell_artist.get_paths()), 0)
        # No red/blocked inference from image colours.
        self.assertIn('displayed grid', p._footprint_note.get_text())

    def test_origin_changes_move_cells_without_moving_vehicle_and_preserve_viewport(self):
        p = self.window.map
        p.closeup_vehicle()
        limits = p._limits()
        old_vehicle = p._footprint_artist.get_xy().copy()
        old_cell = p._cell_artist.get_paths()[0].vertices.copy()
        event = copy.deepcopy(p.event)
        event['grid']['origin']['x'] += .03
        p.set_image(self.snapshot['images']['costmap'], event)
        p.redraw()
        np.testing.assert_allclose(p._footprint_artist.get_xy(), old_vehicle)
        self.assertFalse(np.allclose(p._cell_artist.get_paths()[0].vertices, old_cell))
        self.assertEqual(p._limits(), limits)
        self.assertAlmostEqual(limits[0][1]-limits[0][0], 2.)

    def test_stale_invalid_and_wrong_frame_are_not_fresh_footprints(self):
        p = self.window.map
        self.snapshot['now'] += 3.
        self.window.update_snapshot(self.snapshot)
        p.redraw()
        self.assertEqual(p._footprint_artist.get_linestyle(), '--')
        self.assertIn('STALE', p._footprint_note.get_text())
        pose = self.snapshot['histories']['public'][0]
        pose.update(qx=0., qy=0., qz=0., qw=0.)
        self.window.update_snapshot(self.snapshot)
        p.redraw()
        self.assertFalse(p._footprint_artist.get_visible())
        self.assertEqual(len(p._cell_artist.get_paths()), 0)
        pose.update(qw=1., frame_id='other')
        self.window.update_snapshot(self.snapshot)
        p.redraw()
        self.assertFalse(p._footprint_artist.get_visible())

    def test_toggles_and_identical_snapshot_keep_artist_count_and_skip_redraw(self):
        p = self.window.map
        count = len(p.ax.get_children())
        p.canvas.draw_idle = Mock()
        for _ in range(5):
            self.window.update_snapshot(self.snapshot)
            p.redraw()
        p.canvas.draw_idle.assert_not_called()
        self.window.cells_toggle.setChecked(False)
        self.assertEqual(len(p._cell_artist.get_paths()), 0)
        self.assertTrue(p._footprint_artist.get_visible())
        self.window.footprint_toggle.setChecked(False)
        self.assertFalse(p._footprint_artist.get_visible())
        self.assertFalse(self.window.cells_toggle.isEnabled())
        self.window.footprint_toggle.setChecked(True)
        self.window.cells_toggle.setChecked(True)
        self.assertTrue(p._footprint_artist.get_visible())
        self.assertGreater(len(p._cell_artist.get_paths()), 0)
        self.assertEqual(len(p.ax.get_children()), count)


if __name__ == '__main__':
    unittest.main()
