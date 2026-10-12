import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from nomad_geometry_mapping.labels import build_labels, build_cost_cells


def test_labels_are_unitless_and_have_correct_cell_centers():
    grid = OccupancyGrid()
    grid.header.frame_id = 'odom'
    grid.info.resolution = .2
    grid.info.width, grid.info.height = 3, 1
    grid.info.origin.position.x, grid.info.origin.position.y = -1., 2.
    grid.data = [20, -1, 100]
    message = build_labels(grid, np.array([[13.5, np.nan, np.nan]], dtype=np.float32))
    assert message.markers[0].action == Marker.DELETEALL
    assert [marker.text for marker in message.markers[1:]] == ['13.5', 'N/A']
    assert np.isclose(message.markers[1].pose.position.x, -.9)
    assert np.isclose(message.markers[1].pose.position.y, 2.1)
    assert all(marker.header.frame_id == 'odom' for marker in message.markers)


def test_cost_cells_omit_unknown_and_keep_blocked_black():
    grid = OccupancyGrid()
    grid.info.resolution = .2
    grid.info.width, grid.info.height = 3, 1
    grid.data = [0, -1, 100]
    marker = build_cost_cells(grid)
    assert len(marker.points) == len(marker.colors) == 2
    assert marker.colors[0].r == marker.colors[0].g == marker.colors[0].b == 1.
    assert marker.colors[1].r == marker.colors[1].g == marker.colors[1].b == 0.
    assert np.isclose(marker.points[1].x, .5)
