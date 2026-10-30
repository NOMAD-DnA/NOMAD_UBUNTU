from nomad_operator_gui.grid import GridGeometry


def test_top_left_and_bottom_right_are_correct_ros_cells():
    grid = GridGeometry(10, 5, 0.2, -1.0, 2.0)
    assert grid.valid(50)
    assert grid.cell_from_fraction(0.01, 0.01) == (0, 4)
    assert grid.cell_from_fraction(0.99, 0.99) == (9, 0)
    assert grid.center(0, 4) == (-0.9, 2.9)
    assert grid.cell_from_fraction(1, 0.5) is None


def test_vehicle_coordinate_maps_back_to_display_fraction():
    grid = GridGeometry(10, 10, 0.2, 1.0, -1.0)
    x, y = grid.center(4, 6)
    fx, fy = grid.fraction_from_world(x, y)
    assert grid.cell_from_fraction(fx, fy) == (4, 6)
